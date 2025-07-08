from abc import ABC, abstractmethod
import cv2
import Datasources as ds
import threading
import socket
import zmq
import numpy as np
import struct
import time
import pickle

class Action(ABC):
    @abstractmethod
    def execute(self, rgb_frame, depth_frame):
        """Führt eine Aktion auf den verarbeiteten Frames aus"""
        pass

"""
class ShowImageAction(Action):
    def __init__(self, window_name="RGB", depth_window_name="Depth"):
        self.window_name = window_name
        self.depth_window_name = depth_window_name

    def execute(self, rgb_frame, depth_frame):
        if rgb_frame is not None:
            np_arr = np.frombuffer(rgb_frame.tobytes(), np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            cv2.imshow(self.window_name, img)
        if depth_frame is not None:
            depth_color = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frame, alpha=0.03),
                cv2.COLORMAP_JET
            )
            cv2.imshow(self.depth_window_name, depth_color)
        cv2.waitKey(1)
"""
class ShowImageAction(Action):
    def __init__(self, window_name="RGB", depth_window_name="Depth", image_width=640, image_height=480, channels=3):
        self.window_name = window_name
        self.depth_window_name = depth_window_name
        self.image_width = image_width
        self.image_height = image_height
        self.channels = channels

    def is_jpeg(self, data):
        return data[:2] == b'\xff\xd8' and data[-2:] == b'\xff\xd9'

    def execute(self, rgb_frame, depth_frame):
        if rgb_frame is not None:
            rgb_bytes = rgb_frame.tobytes()

            if self.is_jpeg(rgb_bytes):
                np_arr = np.frombuffer(rgb_bytes, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else: # Expected to be height x width x channels
                img = rgb_frame

            if img is not None:
                cv2.imshow(self.window_name, img)

        if depth_frame is not None:
            depth_color = cv2.applyColorMap(
                cv2.convertScaleAbs(depth_frame, alpha=0.03),
                cv2.COLORMAP_JET
            )
            cv2.imshow(self.depth_window_name, depth_color)

        cv2.waitKey(1)


class ZMQPublishAction(Action):
    # Camera Config contains all relevant information that is camera specific, like fx,fy,cx,cy,...
    # This Information is also resolution specific, if the resolution changes with filtering for example
    # the intrinsics has to be scaled, for example if height and width is 1/2, the scale is also 1/2
    def __init__(self, culling: ds.Culling, config_scaling=1.0, port=5555):
        self.culling = culling
        self.config_scaling = config_scaling

        self.start_discovery_server(port=5556)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(f"tcp://*:{port}")
        print(f"[INFO] ZMQ Server läuft auf Port {port}...")

    def execute(self, rgb_frame, depth_frame):
        rgb_bytes = rgb_frame.tobytes()
        depth_bytes = depth_frame.tobytes()

        # Depends on Application, here this is my Packageformat
        packet = (
            struct.pack('<2I', self.width, self.height) +
            struct.pack('<I', len(rgb_bytes)) + rgb_bytes +
            struct.pack('<I', len(depth_bytes)) + depth_bytes +
            struct.pack('<4f', self.config.fx*self.config_scaling, self.config.fy*self.config_scaling, self.config.cx*self.config_scaling, self.config.cy*self.config_scaling) +
            struct.pack('<2f', self.culling.zcullmin, self.culling.zcullmax) +
            struct.pack('<2f', self.culling.x_cull, self.culling.y_cull)
        )
        try:
            self.socket.send(packet, zmq.NOBLOCK)
        except zmq.Again:
            pass

    def start_discovery_server(self, port=5556, response_message=b"ZMQ_SERVER_HERE"):
        def discovery_loop():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', port))
            print(f"[INFO] Discovery-Server läuft auf Port {port}...")

            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    if data == b"DISCOVER_ZMQ_SERVER":
                        print(f"[DISCOVERY] Anfrage von {addr[0]}")
                        sock.sendto(response_message, addr)
                except Exception as e:
                    print("[ERROR] Discovery-Server:", e)
                    break

            sock.close()

        thread = threading.Thread(target=discovery_loop, daemon=True)
        thread.start()

    def set_width_height(self, width, height):
        self.width = width
        self.height = height

    def set_config(self, config: ds.CameraConfig):
        self.config = config

class ZMQPublishPointCloudAction(Action):
    def __init__(self, culling: ds.Culling, config_scaling=1.0, port=5555):
        self.culling = culling
        self.config_scaling = config_scaling
        self.start_discovery_server(port=5556)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(f"tcp://*:{port}")
        print(f"[INFO] ZMQ Server läuft auf Port {port}...")

        self.topic = "PointCloud"
        self.last_time = time.time()
        self.last_fps_log_time = self.last_time  # Nur 1x/Sekunde loggen
        self.call_counter = 0


    def execute(self, rgb_frame, depth_frame):
            current_time = time.time()
            delta = current_time - self.last_time
            self.last_time = current_time
            fps = 1.0 / delta if delta > 0 else 0.0

            if current_time - self.last_fps_log_time >= 1.0:
                print(f"[INFO] FPS: {fps:.2f}")
                self.last_fps_log_time = current_time

            self.call_counter += 1

            # Punktwolke erzeugen
            # Vertikal flippen der Eingabebilder
            rgb_frame_flipped = np.flip(rgb_frame, axis=0)
            rgb_frame_flipped = rgb_frame_flipped[:, :, ::-1]  # BGR → RGB
            depth_frame_flipped = np.flip(depth_frame, axis=0)

            # Punktwolke mit geflippten Frames erzeugen
            xyz, rgb = self.create_pointcloud(rgb_frame_flipped, depth_frame_flipped)
            print(xyz.shape)
            """
            if xyz.shape[0] > 0:
                print(type(xyz[0][0]))
                print(type(xyz[0][1]))
                print(type(xyz[0][2]))
                print(type(rgb[0][0]))
                print(type(rgb[0][1]))
                print(type(rgb[0][2]))
            """

            if self.call_counter == 100:
                print("[INFO] Speichere 100ste Punktwolke als .ply...")
                self.save_pointcloud_as_ply(np.concatenate((xyz, rgb), axis=1), "pointcloud_100.ply")

            # Anzahl Punkte
            num_points = self.width*self.height

            xyz_bytes = xyz.tobytes()
            rgb_bytes = rgb.tobytes()

            


            num_points_bytes = struct.pack('<I', num_points)
            try:
                self.socket.send_multipart([
                    self.topic.encode(),
                    num_points_bytes,
                    xyz_bytes,
                    rgb_bytes
                ])
            except zmq.Again:
                print("[WARN] ZMQ send failed (socket busy)")

            """
              # Objekt laden
              with open('saved_packages.pkl', 'rb') as f:
                  my_object = pickle.load(f)

              data = my_object[0][2]
              arr = np.frombuffer(data, dtype=np.float32)
              arr = arr.reshape((-1, 6)).astype(np.float32)



              # 2. In 6-Spalten-Array umformen
              num_points = 10000
              xyzrgb_bytes = arr.tobytes()
              """

            """
            arr = arr.copy()

            # XYZ (erste 3 Spalten)
            xyz = arr[:, :3]

            # RGB normalisiert (letzte 3 Spalten)
            rgb_normalized = arr[:, 3:]

            # RGB auf 0-255 skalieren und in int konvertieren
            rgb_255 = (rgb_normalized * 255).astype(int)

            # Wieder zusammenfügen (XYZ float + RGB int → alles float oder casten je nach save-Funktion)
            # Falls save_pointcloud_as_ply RGB als ints erwartet, kannst du hier z.B. float konvertieren:
            combined = np.concatenate((xyz, rgb_255.astype(float)), axis=1)
            print(combined)
            # Jetzt speichern
            self.save_pointcloud_as_ply(combined, "pointcloud_100.ply")
            """


            """
            # Dein Byte-Array (hier als `data`)
            data = b'Z}\xc5\xc3\x8fl\xd1\xc2\xfc\xda\x8eB\x00\x00\xc0@\x00\x00@A\x00\x00`A\xfeZ\xef\xc3\xf5\xcf\xad\xc2%9\xcfA\x00\x00\x18B\x00\x00\x98B\x00\x00\xc6B(\xc7\x03\xc4\xa8S\xd4\xc3\xcdlT\xc3\x00\x00\x0bC\x00\x00\\B\x00\x00\xb8Ae\x19\xc2\xc3\x17\x11t\xc2\x96\xb1KA\x00\x000A\x00\x00\xe0A\x00\x00 B\x9e\x85\x01\xc4\xa5\x1b\xc4\xc3\x01\x11k\xc3\x00\x002C\x00\x00xB\x00\x00\x00\x00=a\x03\xc4\xb6\xd6\xdc\xc3w=4\xc3\x00\x00?C\x00\x00\x11C\x00\x00\tC\xc8\x1c\x0e\xc4\x82=\xbc\xc38[k\xc3\x00\x00)C\x00\x00\xf6B\x00\x00\xbcB\x03\x8b\x12\xc4\xc4&\xda\xc3\xcf\xdaX\xc3\x00\x00YC\x00\x00\xe6B\x00\x00\x0cB>q\r\xc4\xa5\xb7\xa6\xc3\xa0`D\xc3\x00\x00DC\x00\x00|B\x00\x00\xa0@P\xc6\x04\xc4[\x17\xa2\xc3;\x83u\xc3\x00\x00FC\x00\x00\xe2B\x00\x00\x84B'
            # 1. Als float32 interpretieren
            arr = np.frombuffer(data, dtype=np.float32)
    
            # 2. In 6-Spalten-Array umformen
            arr = arr.reshape((-1, 6))  # (N, 6)
    
            # 3. Optional: anzeigen
            print(arr.shape)
            print(arr)
            """

    """
    def create_pointcloud(self, rgb, depth):
        h, w = depth.shape
        fx = self.config.fx * self.config_scaling
        fy = self.config.fy * self.config_scaling
        cx = self.config.cx * self.config_scaling
        cy = self.config.cy * self.config_scaling

        x, y = np.meshgrid(np.arange(w), np.arange(h))

        z = depth.astype(np.float32) * 0.001
        x3 = (x - cx) * z / fx
        y3 = (y - cy) * z / fy
        z3 = z

        valid = (z > 0) & np.isfinite(z)

        xyz = np.stack((x3[valid], y3[valid], z3[valid]), axis=-1)
        rgb_valid = rgb[valid]
        rgb_norm = rgb_valid.astype(np.float32) / 255

        return xyz, rgb_norm
    """

    def create_pointcloud(self, rgb, depth):
        h, w = depth.shape
        fx = self.config.fx * self.config_scaling
        fy = self.config.fy * self.config_scaling
        cx = self.config.cx * self.config_scaling
        cy = self.config.cy * self.config_scaling

        x, y = np.meshgrid(np.arange(w), np.arange(h))

        z = depth.astype(np.float32)
        x3 = (x - cx) * z / fx
        y3 = (y - cy) * z / fy
        z3 = z

        valid = (z > 0) & np.isfinite(z)

        # XYZ in millimeters, cast to int16
        xyz = np.stack((x3[valid], y3[valid], z3[valid]), axis=-1)
        xyz_mm = np.round(xyz).astype(np.int16)  # Convert to int16 in mm

        # RGB in uint8 (0–255)
        rgb_valid = rgb[valid].astype(np.uint8)

        # Combine: shape (N, 6), dtype = mixed (will promote to larger type if stored in a single array)
        return xyz_mm, rgb_valid


    def start_discovery_server(self, port=5556, response_message=b"ZMQ_SERVER_HERE"):
        def discovery_loop():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', port))
            print(f"[INFO] Discovery-Server läuft auf Port {port}...")

            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    if data == b"DISCOVER_ZMQ_SERVER":
                        print(f"[DISCOVERY] Anfrage von {addr[0]}")
                        sock.sendto(response_message, addr)
                except Exception as e:
                    print("[ERROR] Discovery-Server:", e)
                    break

            sock.close()

        thread = threading.Thread(target=discovery_loop, daemon=True)
        thread.start()

    def set_width_height(self, width, height):
        self.width = width
        self.height = height

    def set_config(self, config: ds.CameraConfig):
        self.config = config

    def save_pointcloud_as_ply(self, pointcloud, filename):
        """Speichert XYZRGB-Punktwolke als ASCII PLY-Datei."""
        with open(filename, 'w') as f:
            f.write("ply\n")
            f.write("format ascii 1.0\n")
            f.write(f"element vertex {pointcloud.shape[0]}\n")
            f.write("property float x\n")
            f.write("property float y\n")
            f.write("property float z\n")
            f.write("property uchar red\n")
            f.write("property uchar green\n")
            f.write("property uchar blue\n")
            f.write("end_header\n")
            for point in pointcloud:
                x, y, z, r, g, b = point
                f.write(f"{x:.4f} {y:.4f} {z:.4f} {int(r * 255)} {int(g * 255)} {int(b * 255)}\n")


class ActionPipeline:
    def __init__(self):
        self.actions = []

    def add_action(self, action: Action):
        self.actions.append(action)

    def execute_all(self, rgb_frame, depth_frame):
        for action in self.actions:
            action.execute(rgb_frame, depth_frame)
