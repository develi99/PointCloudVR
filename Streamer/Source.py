from abc import ABC, abstractmethod
import cv2
import zmq
import struct
import numpy as np
import Datasources as ds
import threading
import open3d as o3d
import time
from collections import defaultdict
import pickle
from multiprocessing import Process, Manager, Lock, shared_memory, Value
import copy
import pyzed.sl as sl
import os
import ctypes
import pyrealsense2 as rs
import numba as nb



@nb.njit
def compact_inplace(points, colors, mask):
    write_idx = 0
    for read_idx in range(len(mask)):
        if mask[read_idx]:
            points[write_idx] = points[read_idx]
            colors[write_idx] = colors[read_idx]
            write_idx += 1
    return write_idx

class Source(ABC):
    def __init__(self, width, height, use_imgs=False, use_segmentation=False, use_pc_creation=True, name=None, id=None):
        num_points = width * height

        # SharedMemory for RGB and Depth
        self.rgb_shm = shared_memory.SharedMemory(create=True, size=width * height * 3 * np.uint8().nbytes) # uint8
        self.depth_shm = shared_memory.SharedMemory(create=True, size=height * width * np.uint16().nbytes) # uint16

        #Numpy-Views for RGB and Depth
        self.rgb_view = np.ndarray((height, width, 3), dtype=np.uint8, buffer=self.rgb_shm.buf)
        self.depth_view = np.ndarray((height, width), dtype=np.uint16, buffer=self.depth_shm.buf)
        self.lock_rgbd = Lock()

        # SharedMemory for Pointcloud
        self.points_shm = shared_memory.SharedMemory(create=True, size=num_points*3*4)  # float32
        self.colors_shm = shared_memory.SharedMemory(create=True, size=num_points*3*4)  # float32

        # NumPy-Views for pointcloud
        self.points_view = np.ndarray((num_points, 3), dtype=np.float32, buffer=self.points_shm.buf)
        self.colors_view = np.ndarray((num_points, 3), dtype=np.float32, buffer=self.colors_shm.buf)
        self.lock_pc = Lock()

        # additional shared variables, smaller values, that dont need shared memory
        manager = Manager()
        self.lock_ns = Lock()
        self.shared_ns = manager.Namespace()
        self.shared_ns.new_pc_available = False
        self.shared_ns.new_rgbd_available = False
        self.shared_ns.config = None
        self.shared_ns.running = True
        self.shared_ns.n_valid = 0
        self.shared_ns.use_imgs = use_imgs
        self.shared_ns.use_segmentation = use_segmentation
        self.shared_ns.use_pc_creation = use_pc_creation

        # Start Process for Camera
        self.process = Process(target=self._camera_loop, args=(name, self.shared_ns, self.lock_ns, self.rgb_shm, self.depth_shm, self.lock_rgbd, self.points_shm, self.colors_shm, self.lock_pc, width, height, id))
        self.process.start()

    def stop_process(self):
        self.shared_ns.running = False
        if self.process.is_alive():
            self.process.join()

    def _camera_loop(self, name, shared_ns, lock_ns, rgb_buffer, depth_buffer, lock_rgbd, buffer_points, buffer_colors, lock_pc, width, height, id):
        frame_count = 0
        start_time_fps = time.time()

        # init
        self.name = name
        self.height = height
        self.width = width
        self.lock_ns = lock_ns

        # init YOLO (segmentation model)
        self._zmq_context = zmq.Context.instance()
        self._zmq_socket = self._zmq_context.socket(zmq.DEALER)
        self._zmq_socket.identity = self.name.encode()
        self._zmq_socket.connect("tcp://localhost:25555")
        print(f"[INFO] YOLO-Client für {self.name} verbunden.")

        # init shared memories
        num_points = self.width * self.height
        # shared memory for RGBD
        self.rgb_view = np.ndarray((height, width, 3), dtype=np.uint8, buffer=rgb_buffer.buf)
        self.depth_view = np.ndarray((height, width), dtype=np.uint16, buffer=depth_buffer.buf)
        self.lock_rgbd = lock_rgbd

        # shared memory for PointCloud
        self.points_view = np.ndarray((num_points, 3), dtype=np.float32, buffer=buffer_points.buf)
        self.colors_view = np.ndarray((num_points, 3), dtype=np.float32, buffer=buffer_colors.buf)
        self.lock_pc = lock_pc

        # logging
        self.avg_seg = 0.0
        self.seg_count = 0
        self.avg_pcc = 0.0
        self.pcc_count = 0

        # Connect to camera
        self.id = id
        self.connect()

        while shared_ns.running:
            r = self.get_frame()
            if r is None:
                time.sleep(0.0001)
                continue
            
            rgb, depth, config = r


            # adapt intrinsics and size
            h = rgb.shape[0]
            w = rgb.shape[1]

            if w < self.width or h < self.height:
                print("[ERROR] received heigt or width is smaller than given")
                continue

            if h != self.height or w != self.width:
                scale_x = self.width / w
                scale_y = self.height / h
                config.fx *= scale_x
                config.fy *= scale_y
                config.cx *= scale_x
                config.cy *= scale_y

                rgb = cv2.resize(rgb, (self.width, self.height), interpolation=cv2.INTER_AREA)
                depth = cv2.resize(depth, (self.width, self.height), interpolation=cv2.INTER_NEAREST)

            if self.shared_ns.use_pc_creation:
                pc_thread = threading.Thread(target=self.create_pointcloud, args=(rgb, depth, config))
                pc_thread.start()

            if self.shared_ns.use_segmentation:
                mask = self.apply_segmentation(rgb, depth)
                if mask is None:
                    mask_bool = np.ones(self.width * self.height, dtype=bool)
                else:
                    mask_vis = mask.squeeze().astype(np.uint8) * 255
                    overlay = rgb.copy()
                    overlay[mask_vis == 0] = (0, 0, 0)

                    mask = np.flip(mask, axis=1)

                    mask_bool = mask.squeeze().astype(bool).ravel()
            else:
                mask_bool = np.ones(self.width * self.height, dtype=bool)
            
            if self.shared_ns.use_pc_creation:
                pc_thread.join()
                
                with self.lock_pc:
                    # apply mask
                    if self.shared_ns.use_segmentation:
                        n_valid = compact_inplace(self.points_view, self.colors_view, mask_bool)
                    else:
                        n_valid = self.width * self.height

            if self.shared_ns.use_imgs:
                with self.lock_rgbd:
                    self.rgb_view[:] = rgb
                    self.depth_view[:] = depth
                    self.shared_ns.config = config

            with self.lock_ns:
                self.shared_ns.new_pc_available = True
                self.shared_ns.new_rgbd_available = True
                self.shared_ns.n_valid = n_valid
            
            # ---- FPS Calculation ----
            frame_count += 1
            elapsed = time.time() - start_time_fps
            if elapsed >= 1.0:  # every second
                fps = frame_count / elapsed
                print(f"[Camera FPS] Name: {self.name} FPS: {fps:.2f}")
                frame_count = 0
                start_time_fps = time.time()
                print(f"avg. segmentation time - {self.avg_seg:.4f} seconds.")
                print(f"avg. pointcloud creation time - {self.avg_pcc:.4f} seconds.")

    def apply_segmentation(self, rgb, depth):
        try:
            start = time.time()
            # Send RGB to Yolo
            msg = pickle.dumps(rgb)
            self._zmq_socket.send_multipart([b'', msg]) 

            # receive mask
            reply = self._zmq_socket.recv_multipart()            
            mask = pickle.loads(reply[-1])

            if mask is not None:
                # --- Handle mask size relative to RGB ---
                if mask.shape[:2] != rgb.shape[:2]:
                    mask_resized = cv2.resize(mask, (rgb.shape[1], rgb.shape[0]), interpolation=cv2.INTER_NEAREST)
                else:
                    mask_resized = mask

                # Ensure mask has channel dimension for RGB masking
                if mask_resized.ndim == 2:
                    mask_resized = mask_resized[..., None]

                mask = mask_resized

        except zmq.error.Again:
            print("⚠️ Timeout beim YOLO-Server")
            mask = None
        
        end = time.time()
        elapsed = end - start
        self.seg_count += 1
        self.avg_seg = ((self.avg_seg * (self.seg_count - 1)) + elapsed) / self.seg_count

        return mask


    def create_pointcloud(self, rgb, depth, config):
        start = time.time()
        H, W = depth.shape

        # --- Flip Bilder ---
        rgb = np.flip(rgb, axis=1)
        depth = np.flip(depth, axis=1)

        # --- Intrinsics ---
        fx, fy, cx, cy = config.fx, config.fy, config.cx, config.cy
        # Angepasstes cx wegen Flip:
        cx_flipped = W - cx - 1  

        # --- Pixel-Koordinaten (u,v) ---
        u = np.arange(W)
        v = np.arange(H)
        uu, vv = np.meshgrid(u, v)
  
        # --- Depth in Meter ---
        z = depth.astype(np.float32) / 1000.0  # mm → m

        x = (uu - cx_flipped) * z / fx
        y = (vv - cy) * z / fy

        # --- In SharedMemory schreiben ---
        with self.lock_pc:
            self.points_view[:, 0] = x.ravel()
            self.points_view[:, 1] = y.ravel()
            self.points_view[:, 2] = z.ravel()

            self.colors_view[:, 0] = rgb[:, :, 0].ravel() / 255.0
            self.colors_view[:, 1] = rgb[:, :, 1].ravel() / 255.0
            self.colors_view[:, 2] = rgb[:, :, 2].ravel() / 255.0

        end = time.time()
        elapsed = end - start
        self.pcc_count += 1
        self.avg_pcc = ((self.avg_pcc * (self.pcc_count - 1)) + elapsed) / self.pcc_count

    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def close(self):
        self.stop_process()

    def get_pcd(self, dev="CPU:0", wait=False, sleep=0.001):
        if self.shared_ns.use_pc_creation is False:
            print("[Error] set use_pc_creation in Contructor as True, to get pointcloud values")
            return None

        if wait:
            while not self.shared_ns.new_pc_available:
                time.sleep(sleep)
        else:
            if not self.shared_ns.new_pc_available:
                return None

        # SharedMemory sicher lesen
        with self.lock_pc:
            n_valid = self.shared_ns.n_valid
            pts_np  = self.points_view[:n_valid]    # (N,3), dtype vermutlich float64
            col_np  = self.colors_view[:n_valid]    # (N,3)
            self.shared_ns.new_pc_available = False

        device = o3d.core.Device(dev)
        pcd = o3d.t.geometry.PointCloud(device)

        if device.get_type() == o3d.core.Device.DeviceType.CPU:
            # CPU: float64 beibehalten (zero-copy)
            if pts_np.dtype != np.float64 or col_np.dtype != np.float64:
                # Falls deine Quellen doch float32 liefern, kannst du hier explizit auf 64 gehen:
                pts_np = pts_np.astype(np.float64, copy=False)
                col_np = col_np.astype(np.float64, copy=False)

            pcd.point["positions"] = o3d.core.Tensor.from_numpy(pts_np)  # teilt Speicher (zero-copy)
            pcd.point["colors"]    = o3d.core.Tensor.from_numpy(col_np)

        else:
            # GPU: i.d.R. float32 sinnvoll (Bandbreite/Kompatibilität), Host->Device Transfer passiert sowieso
            if pts_np.dtype != np.float32:
                pts_np = pts_np.astype(np.float32, copy=False)
            if col_np.dtype != np.float32:
                col_np = col_np.astype(np.float32, copy=False)

            pcd.point["positions"] = o3d.core.Tensor(pts_np, device=device)
            pcd.point["colors"]    = o3d.core.Tensor(col_np, device=device)

        return pcd
    
    def get_rgbd_and_config(self, wait=False, sleep=0.001):
        if self.shared_ns.use_imgs is False:
            print("[Error] set use_imgs in Contructor as True, to get rgb and depth values")
            return None
        if wait:
            while self.shared_ns.new_rgbd_available is False:
                time.sleep(sleep)
        
        if self.shared_ns.new_rgbd_available:
            rgb = self.rgb_view.copy()
            depth = self.depth_view.copy()
            config = copy.copy(self.shared_ns.config)
            return rgb, depth, config
        else:
            return None



class RealSenseCamera(Source):
    def __init__(self, id, org_width=640, org_height=480, res_width=640, res_height=480, fps=30, use_imgs=False, use_segmentation=False, use_pc_creation=True):
        self.org_width = org_width
        self.org_height = org_height
        self.fps = fps
        super().__init__(name=f"RealSense_{id}", width=res_width, height=res_height, use_imgs=use_imgs, use_segmentation=use_segmentation, use_pc_creation=use_pc_creation, id=id)

    def connect(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.id)
        config.enable_stream(rs.stream.depth, self.org_width, self.org_height, rs.format.z16, self.fps) # FPS here hardcoded 60
        config.enable_stream(rs.stream.color, self.org_width, self.org_height, rs.format.bgr8, self.fps)

        self.temp_filter = rs.temporal_filter(
                                smooth_alpha=0.8,     # high smoothing
                                smooth_delta=20.0,    # allow max 20 mm depth change per frame
                                persistence_control=2 # consider last 3 frames
                            )
        
        self.spatial = rs.spatial_filter(smooth_alpha=0.8, smooth_delta= 20.0, magnitude=2, hole_fill=0.0)

        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        time.sleep(1)

        # Warm Start
        for _ in range(60):
            self.get_frame()

    def get_frame(self):
        try:
            frames = self.pipeline.wait_for_frames()
            aligned = self.align.process(frames)
            depth_frame = aligned.get_depth_frame()

            #depth_frame = self.apply_filters(depth_frame)

            color_frame = aligned.get_color_frame()
            if not depth_frame or not color_frame:
                return None
            #depth_frame = self.apply_filters(depth_frame)

            o = self.get_intrinsics()
            return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), ds.CameraConfig(o["fx"], o["fy"], o["ppx"], o["ppy"])
        except Exception as e:
            print(f"[Error] RealSense get_frame: {e}")
            return None
        
    def apply_filters(self, depth_frame):
        filtered = self.temp_filter.process(depth_frame)
        filtered = self.spatial.process(filtered)
        #filtered = self.temporal.process(filtered)
        #filtered = self.hole_filling.process(filtered)
        return filtered

    def get_intrinsics(self):
        if self.profile is None:
            raise RuntimeError("Camera not found.")

        depth_stream = self.profile.get_streams()[1]
        depth_stream = depth_stream.as_video_stream_profile()
        intr = depth_stream.get_intrinsics()

        return {
            "width": intr.width,
            "height": intr.height,
            "fx": intr.fx,
            "fy": intr.fy,
            "ppx": intr.ppx,
            "ppy": intr.ppy,
            "model": str(intr.model),
            "distortion_coeffs": list(intr.coeffs)
        }

    def close(self):
        super().close()
        self.pipeline.stop()



class VirtualCamera(Source):
    def __init__(self, name, streamer, width=640, height=480, use_imgs=False, use_segmentation=False, use_pc_creation=True):
        self.streamer = streamer
        self.buffer = streamer.device_buffers
        self.lock_buffer = streamer.lock
        super().__init__(name=name, width=width, height=height, use_imgs=use_imgs, use_segmentation=use_segmentation, use_pc_creation=use_pc_creation)

    def connect(self):
        pass  # Optional, falls gebraucht

    def read_buffer(self):
        r = None
        with self.lock_buffer:
            if self.name in self.buffer:
                r = self.buffer[self.name]
            if r is None:
                return None
            if self.buffer[self.name]['new'] is False:
                return None
            
            self.buffer[self.name]['new'] = False

        # SharedMemory öffnen
        rgb_shm = shared_memory.SharedMemory(name=r['rgb_name'])
        depth_shm = shared_memory.SharedMemory(name=r['depth_name'])

        rgb = np.ndarray(r['rgb_shape'], dtype=np.uint8, buffer=rgb_shm.buf).copy()
        depth = np.ndarray(r['depth_shape'], dtype=np.uint16, buffer=depth_shm.buf).copy()
        config = r['config']

        return rgb, depth, config

    def get_frame(self):
        return self.streamer.get_frame_from_buffer(self.buffer, self.lock_buffer, self.name)

    def close(self):
        self.streamer.close()
        self.points_shm.close()
        self.colors_shm.close()
        self.depth_shm.close()
        self.rgb_shm.close()



class Receiver():
    def __init__(self, address, port):
        self.address = address
        self.port = port

        self.manager = Manager()
        self.device_buffers = self.manager.dict()
        self.lock = Lock()
        self.running = Value(ctypes.c_bool, True)

        self.start_receiver_process()

    def connect(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.setsockopt(zmq.RCVHWM, 1)
        self.socket.bind(f"tcp://*:{self.port}")
        print(f"[INFO] Verbunden mit ZMQ-Server auf Port {self.address}:{self.port}...")

    def start_receiver_process(self):
        self.running = True
        self.process = Process(target=self._receive_loop, args=(self.running,self.device_buffers,self.lock))
        self.process.start()
        print("[INFO] Empfangs-Prozess gestartet.")

    def stop_receiver_process(self):
        self.running = False
        if hasattr(self, "process") and self.process.is_alive():
            self.process.join()
            print("[INFO] Empfangs-Prozess gestoppt.")

    def _receive_loop(self, running, device_buffers, lock):
        self.connect()
        self.frame_counts = defaultdict(int)
        self.last_fps_time = time.time()
        self.arrays = {}  # Cache für offene np.ndarray Views
        self.lock = lock
        self.device_buffers = device_buffers
        self.running = running

        while self.running:
            start = time.time()
            result = self.get_frame()
            if result is None:
                continue
            end = time.time()
            # print(f"[DEBUG] Frame empfangen in {end - start:.4f} Sekunden.")

            start = time.time()
            rgb, d, config, name = result
            self.add_frame_to_buffer(name, rgb, d, config)
            end = time.time()
            # print(f"[DEBUG] Frame in Puffer gespeichert in {end - start:.4f} Sekunden.")    

            # ---- FPS Tracking ----
            self.frame_counts[name] += 1
            now = time.time()
            if now - self.last_fps_time >= 1.0:  # jede Sekunde ausgeben
                for cam_name, count in self.frame_counts.items():
                    print(f"[Receiving FPS] {cam_name}: {count} fps")
                self.frame_counts = defaultdict(int)  # reset counters
                self.last_fps_time = now

    def get_frame(self):
        try:
            packet = self.socket.recv()
        except zmq.Again:
            print("[WARN] Kein Paket verfügbar (recv timeout oder non-blocking).")
            return None, None
        except zmq.ZMQError as e:
            print(f"[ERROR] ZMQ-Fehler beim Empfang: {e}")
            return None, None

        try:
            offset = 0
            
            # Read Namelength of the camera
            str_len = struct.unpack_from('<I', packet, offset)[0]
            offset += 4
            
            # Read Name
            name_bytes = packet[offset:offset + str_len]
            name = name_bytes.decode('utf-8')
            offset += str_len

            # Height and width
            h = struct.unpack_from('<I', packet, offset)[0]
            offset += 4
            w = struct.unpack_from('<I', packet, offset)[0]
            offset += 4

            # RGB-datalength (uint32)
            rgb_len = struct.unpack_from('<I', packet, offset)[0]
            offset += 4

            # RGB-Data
            rgb_bytes = packet[offset:offset + rgb_len]
            offset += rgb_len

            # Depth-datalength (uint32)
            depth_len = struct.unpack_from('<I', packet, offset)[0]
            offset += 4

            # Depth-Data
            depth_bytes = packet[offset:offset + depth_len]
            offset += depth_len

            # Camera-Parameter (4 float32)
            fx, fy, cx, cy = struct.unpack_from('<4f', packet, offset)
            offset += 16   
            
        except (struct.error, ValueError) as e:
            print(f"[ERROR] Fehler beim Parsen des Pakets: {e}")
            return None

        try:
            # convert to nparray
            rgb_array = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((h, w, 3))
            #rgb_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            depth_array = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((h, w))
        except ValueError as e:
            print(f"[ERROR] Fehler beim Umwandeln der Bilddaten: {e}")
            return None
        """
        print("[INFO] Paket erfolgreich empfangen und verarbeitet.")
        print(f" - RGB-Shape: {rgb_array.shape}")
        print(f" - Depth-Shape: {depth_array.shape}")
        """

        return rgb_array, depth_array, ds.CameraConfig(fx, fy, cx, cy), name

    def add_frame_to_buffer(self, name, rgb, d, config):
        # Lock sorgt für thread/process-sicheren Zugriff
        start = time.time()
        H, W, C = rgb.shape
        with self.lock:
            if name not in self.device_buffers:
                print(f"append {name}")
                # RGB Shared Memory erstellen
                
                rgb_shm = shared_memory.SharedMemory(create=True, size=H*W*C)
                rgb_array = np.ndarray((H, W, C), dtype=np.uint8, buffer=rgb_shm.buf)

                # Depth Shared Memory erstellen (16-bit)
                depth_shm = shared_memory.SharedMemory(create=True, size=H*W*2)
                depth_array = np.ndarray((H, W), dtype=np.uint16, buffer=depth_shm.buf)

                # Config als normales Objekt speichern (klein, kein Shared Memory nötig)
                self.device_buffers[name] = self.manager.dict({
                    'rgb_name': rgb_shm.name,
                    'depth_name': depth_shm.name,
                    'rgb_shape': rgb_array.shape,
                    'depth_shape': depth_array.shape,
                    'config': config,
                    'new': False
                })

                # Lokale Views cachen (keine Pickle!)
                self.arrays[name] = {
                    "rgb": np.ndarray(rgb.shape, dtype=rgb.dtype, buffer=rgb_shm.buf),
                    "depth": np.ndarray(d.shape, dtype=d.dtype, buffer=depth_shm.buf)
                }

                # Speicher offen halten (nicht schließen!)
                self.arrays[name]["rgb_shm"] = rgb_shm
                self.arrays[name]["depth_shm"] = depth_shm
            
            self.device_buffers[name]['new'] = True
            arr = self.arrays[name]
        
        end = time.time()
        # print(f"[DEBUG] buffer erhalten {end - start:.4f} Sekunden.")    


        # Kopieren direkt in den gecachten Array
        start = time.time()
        np.copyto(arr["rgb"], rgb, casting='no')
        np.copyto(arr["depth"], d, casting='no')
        end = time.time()
        # print(f"[DEBUG] Frame in Puffer kopiert in {end - start:.4f} Sekunden.")    

        start = time.time()
        self.device_buffers[name]["config"] = config
        end = time.time()

        # print(f"[DEBUG] config in Buffer eingefuegt in {end - start:.4f} Sekunden.")    
        # Neuen Frame einfach überschreiben
        # self.device_buffers[name] = (rgb, d, config)
    
    @staticmethod
    def get_frame_from_buffer(buffer, lock_buffer, name):
        r = None
        with lock_buffer:
            if name in buffer:
                r = buffer[name]
            if r is None:
                return None
            if buffer[name]['new'] is False:
                return None
            
            buffer[name]['new'] = False
        
        # SharedMemory öffnen
        rgb_shm = shared_memory.SharedMemory(name=r['rgb_name'])
        depth_shm = shared_memory.SharedMemory(name=r['depth_name'])

        rgb = np.ndarray(r['rgb_shape'], dtype=np.uint8, buffer=rgb_shm.buf).copy()
        depth = np.ndarray(r['depth_shape'], dtype=np.uint16, buffer=depth_shm.buf).copy()
        config = r['config']

        return rgb, depth, config


    def close(self):
        self.socket.close()
        self.context.term()
        self.stop_receiver_thread()
