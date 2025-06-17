from abc import ABC, abstractmethod
import cv2
import streamer.Datasources as ds
import threading
import socket
import zmq
import numpy as np
import struct

class Action(ABC):
    @abstractmethod
    def execute(self, rgb_frame, depth_frame):
        """Führt eine Aktion auf den verarbeiteten Frames aus"""
        pass


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


class ActionPipeline:
    def __init__(self):
        self.actions = []

    def add_action(self, action: Action):
        self.actions.append(action)

    def execute_all(self, rgb_frame, depth_frame):
        for action in self.actions:
            action.execute(rgb_frame, depth_frame)
