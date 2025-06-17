from abc import ABC, abstractmethod
import cv2
import zmq
import struct
import socket
import numpy as np
import streamer.Datasources as ds

#from pyk4a import PyK4A, Config, CalibrationType, ColorResolution, DepthMode
#from pyk4a.calibration import Calibration

class Source(ABC):
    @abstractmethod
    def connect(self):
        pass

    @abstractmethod
    def get_frame(self):
        pass

    @abstractmethod
    def close(self):
        pass



class CameraStrategy(Source):
    @abstractmethod
    def apply_filters(self, depth_frame):
        pass

    @abstractmethod
    def get_intrinsics(self):
        pass



try:
    import depthai as dai
except ImportError:
    dai = None

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

class RealSenseCameraStrategy(CameraStrategy):
    def __init__(self, width=640, height=480):
        if rs is None:
            raise RuntimeError("RealSense SDK nicht installiert.")
        self.width = width
        self.height = height
        self.pipeline = None
        self.profile = None

    def connect(self):
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned = self.align.process(frames)
        depth_frame = aligned.get_depth_frame()
        color_frame = aligned.get_color_frame()
        if not depth_frame or not color_frame:
            return None, None
        #depth_frame = self.apply_filters(depth_frame)

        o = self.get_intrinsics()
        return np.asanyarray(color_frame.get_data()), np.asanyarray(depth_frame.get_data()), ds.CameraConfig(o["fx"], o["fy"], o["ppx"], o["ppy"])

    def apply_filters(self, depth_frame):
        #filtered = self.spatial.process(depth_frame)
        #filtered = self.temporal.process(filtered)
        #filtered = self.hole_filling.process(filtered)
        pass

    def get_intrinsics(self):
        if self.profile is None:
            raise RuntimeError("Kamera nicht verbunden.")

        depth_stream = self.profile.get_stream(rs.stream.depth).as_video_stream_profile()
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
        self.pipeline.stop()



class LuxonisCameraStrategy(CameraStrategy):
    def __init__(self, width=640, height=480):
        if dai is None:
            raise RuntimeError("DepthAI SDK nicht installiert.")
        self.width = width
        self.height = height
        self.device = None

    def connect(self):
        pipeline = dai.Pipeline()

        # Mono-Kameras für Stereo
        monoLeft = pipeline.createMonoCamera()
        monoRight = pipeline.createMonoCamera()
        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)

        stereo = pipeline.createStereoDepth()
        stereo.initialConfig.setConfidenceThreshold(180)
        stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_5x5)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(True)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(self.width, self.height)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb = pipeline.createColorCamera()
        camRgb.setPreviewSize(self.width, self.height)
        camRgb.setInterleaved(False)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        xoutDepth = pipeline.createXLinkOut()
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)

        self.device = dai.Device(pipeline)
        self.rgbQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    def get_frame(self):
        rgb = self.rgbQueue.get().getCvFrame()
        depth = self.depthQueue.get().getFrame()
        o = self.get_intrinsics()
        return rgb, depth, ds.CameraConfig(o["fx"], o["fy"], o["ppx"], o["ppy"])

    def apply_filters(self, depth_frame):
        return depth_frame

    def get_intrinsics(self):
        if self.device is None:
            raise RuntimeError("Kamera nicht verbunden.")

        calib = self.device.readCalibration()
        depth_intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, self.width, self.height)
        fx, fy = depth_intrinsics[0][0], depth_intrinsics[1][1]
        ppx, ppy = depth_intrinsics[0][2], depth_intrinsics[1][2]

        return {
            "fx": fx,
            "fy": fy,
            "ppx": ppx,
            "ppy": ppy,
            "model": "perspective",
            "distortion_coeffs": calib.getDistortionCoefficients(dai.CameraBoardSocket.CAM_A)
        }

    def close(self):
        self.device.close()



class AzureKinectCameraStrategy(CameraStrategy):
    def __init__(self, width=640, height=480):
        self.width = width
        self.height = height
        self.device = None

    def connect(self):
        self.device = PyK4A(Config(
            color_resolution=self._get_color_resolution(),
            depth_mode=self._get_depth_mode(),
            synchronized_images_only=True
        ))
        self.device.start()

    def get_frame(self):
        capture = self.device.get_capture()
        if capture.color is None or capture.depth is None:
            return None, None, None

        color = capture.color
        depth = capture.transformed_depth  # depth aligned to color

        intr = self.get_intrinsics()
        return color, depth, CameraConfig(intr["fx"], intr["fy"], intr["ppx"], intr["ppy"])

    def get_intrinsics(self):
        calib: Calibration = self.device.calibration
        cam = calib.get_camera_matrix(CalibrationType.COLOR)
        fx = cam[0, 0]
        fy = cam[1, 1]
        ppx = cam[0, 2]
        ppy = cam[1, 2]
        return {
            "fx": fx,
            "fy": fy,
            "ppx": ppx,
            "ppy": ppy,
            "matrix": cam.tolist()
        }

    def close(self):
        if self.device is not None:
            self.device.stop()

    def _get_color_resolution(self):
        # Map width x height to Azure Kinect color resolution
        if self.width == 1280 and self.height == 720:
            return ColorResolution.RES_720P
        elif self.width == 1920 and self.height == 1080:
            return ColorResolution.RES_1080P
        else:
            return ColorResolution.RES_720P  # default

    def _get_depth_mode(self):
        return DepthMode.NFOV_UNBINNED  # 640x576 (highest quality narrow FOV)

    def apply_filters(self, depth_frame):
        pass



class CameraContext:
    def __init__(self, strategy: Source):
        self.strategy = strategy
        self.is_camera = isinstance(strategy, CameraStrategy)

    def init(self):
        self.strategy.connect()

    def get_frame(self):
        rgb, depth, config = self.strategy.get_frame()
        return rgb, depth, config

    def apply_filters(self, depth_frame):
        if self.is_camera:
            return self.strategy.apply_filters(depth_frame)
        else:
            print("Filter nicht verfügbar für diese Quelle.")
            return depth_frame

    def get_intrinsics(self):
        if self.is_camera:
            return self.strategy.get_intrinsics()
        else:
            return None

    def close(self):
        self.strategy.close()



class InternetStrategy(Source):
    def __init__(self, address, port, width, height):
        self.address = address
        self.port = port
        self.height = height
        self.width = width

    def connect(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PULL)
        self.socket.connect(f"tcp://{self.address}:{self.port}")
        # setsockopt(zmq.RCVHWM, 1) # Nur ein Paket empfangen
        print(f"[INFO] Verbunden mit ZMQ-Server auf Port {self.port}...")

    def get_frame(self):
        try:
            print("read message")
            packet = self.socket.recv()
            print("read message finished")
        except zmq.Again:
            print("[WARN] Kein Paket verfügbar (recv timeout oder non-blocking).")
            return None, None
        except zmq.ZMQError as e:
            print(f"[ERROR] ZMQ-Fehler beim Empfang: {e}")
            return None, None

        try:
            offset = 0

            # 1. RGB-Datenlänge (uint32)
            rgb_len = struct.unpack_from('<I', packet, offset)[0]
            offset += 4

            # 2. RGB-Daten
            rgb_bytes = packet[offset:offset + rgb_len]
            offset += rgb_len

            # 3. Depth-Datenlänge (uint32)
            depth_len = struct.unpack_from('<I', packet, offset)[0]
            offset += 4

            # 4. Depth-Daten
            depth_bytes = packet[offset:offset + depth_len]
            offset += depth_len

            # 5. Kamera-Parameter (4 float32)
            fx, fy, cx, cy = struct.unpack_from('<4f', packet, offset)
            offset += 16   
            
            print("read message")
        except (struct.error, ValueError) as e:
            print(f"[ERROR] Fehler beim Parsen des Pakets: {e}")
            return None, None

        try:
            # convert to nparray and adapt size
            rgb_array = np.frombuffer(rgb_bytes, dtype=np.uint8).reshape((self.height, self.width, 3))
            rgb_array = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
            depth_array = np.frombuffer(depth_bytes, dtype=np.uint16).reshape((self.height, self.width))
        except ValueError as e:
            print(f"[ERROR] Fehler beim Umwandeln der Bilddaten: {e}")
            return None, None
        """
        print("[INFO] Paket erfolgreich empfangen und verarbeitet.")
        print(f" - RGB-Shape: {rgb_array.shape}")
        print(f" - Depth-Shape: {depth_array.shape}")
        """
        return rgb_array, depth_array, ds.CameraConfig(fx, fy, cx, cy)

    def close(self):
        self.socket.close()
        self.context.term()
