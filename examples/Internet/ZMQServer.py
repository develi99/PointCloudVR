import numpy as np
import zmq
import struct


try:
    import depthai as dai
except ImportError:
    dai = None

"""
Here it uses a Luxonis Camera, you could also use this streamer as streamer source
"""

def connect_camera(camera_type, width=640, height=480):
    if camera_type == "luxonis":
        if dai is None:
            raise RuntimeError("DepthAI SDK nicht installiert.")
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
        stereo.setOutputSize(width, height)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)

        camRgb = pipeline.createColorCamera()
        camRgb.setPreviewSize(width, height)
        camRgb.setInterleaved(False)
        camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

        xoutRgb = pipeline.createXLinkOut()
        xoutRgb.setStreamName("rgb")
        camRgb.preview.link(xoutRgb.input)

        xoutDepth = pipeline.createXLinkOut()
        xoutDepth.setStreamName("depth")
        stereo.depth.link(xoutDepth.input)

        device = dai.Device(pipeline)
        rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        depthQueue = device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        def get_frames():
            rgb = rgbQueue.get().getCvFrame()
            depth = depthQueue.get().getFrame()
            return rgb, depth

        def get_intrinsics():
            if device is None:
                raise RuntimeError("Kamera nicht verbunden.")

            calib = device.readCalibration()
            depth_intrinsics = calib.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B, width, height)
            fx, fy = depth_intrinsics[0][0], depth_intrinsics[1][1]
            ppx, ppy = depth_intrinsics[0][2], depth_intrinsics[1][2]

            return fx, fy, ppx, ppy

        return get_frames, get_intrinsics, lambda: device.close()


class Server():

    def __init__(self, port, get_frames, fx, fy, cx, cy):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(f"tcp://*:{port}")
        print(f"[INFO] ZMQ Server läuft auf Port {port}...")
        self.get_frames = get_frames
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

    def stream_data(self):
        # Create Package to send to receiver
        rgb, depth = self.get_frames()
        rgb_bytes = rgb.astype(np.uint8).tobytes()
        depth_bytes = depth.astype(np.uint16).tobytes()
        packet = (
                struct.pack('<I', len(rgb_bytes)) + rgb_bytes +
                struct.pack('<I', len(depth_bytes)) + depth_bytes +
                struct.pack('<4f', self.fx, self.fy, self.cx, self.cy)
        )
        try:
            self.socket.send(packet, zmq.NOBLOCK)
        except zmq.Again:
            # No Receiver availabe: do nothing
            pass


get_frames, get_intrinsics, close = connect_camera("luxonis")
fx, fy, cx, cy = get_intrinsics()
server = Server(35555, get_frames, fx, fy, cx, cy)

while True:
    server.stream_data()

