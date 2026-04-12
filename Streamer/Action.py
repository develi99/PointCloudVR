from abc import ABC, abstractmethod
import cv2
import Datasources as ds
import threading
import socket
import zmq
import numpy as np
import struct
import cv2
import time
from collections import deque

class ZMQPublishPointCloudAction():
    def __init__(self, port=5555):
        self.start_discovery_server(port=5556)

        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 1)
        self.socket.bind(f"tcp://*:{port}")
        print(f"[INFO] ZMQ Server runs on Port {port}...")

        self.topic = "PointCloud"
        self.last_fps_log_time = time.time()
        self.call_counter = 0

        self.xyz = None
        self.rgb = None
        
        self.recent_points = deque(maxlen=100)
        self.avg_points = 0

    def execute(self, num_points = 100000):
            if self.xyz is None or self.rgb is None:
                print("[WARN] No Pointcloud, skip sending")
                return

            xyz_m = np.asarray(self.xyz)        # shape (N, 3), float32, Meter
            rgb_f = np.asarray(self.rgb)        # shape (N, 3), float32, [0.0, 1.0]

            # convert XYZ → int16 into Millimeter
            xyz_mm = (xyz_m * 1000.0).astype(np.int16)

            # convert RGB → uint8 (0–255)
            rgb_u8 = (rgb_f * 255.0).astype(np.uint8)

            xyz_bytes = xyz_mm.tobytes()
            rgb_bytes = rgb_u8.tobytes()

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

            current_time = time.time()
            self.call_counter += 1            
            self.recent_points.append(self.xyz.shape[0])
            
            # Calculate FPS
            elapsed = current_time - self.last_fps_log_time

            if elapsed >= 1.0:
                fps = self.call_counter / elapsed
                self.call_counter = 0
                self.last_fps_log_time = current_time
                self.avg_points = sum(self.recent_points) / len(self.recent_points)
                print(f"[INFO] Sending FPS: {fps:.2f}, Points: {self.avg_points}")

            self.xyz = None
            self.rgb = None

    def start_discovery_server(self, port=5556, response_message=b"ZMQ_SERVER_HERE"):
        def discovery_loop():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('', port))
            print(f"[INFO] Discovery-Server runs on Port {port}...")

            while True:
                try:
                    data, addr = sock.recvfrom(1024)
                    if data == b"DISCOVER_ZMQ_SERVER":
                        print(f"[DISCOVERY] request from {addr[0]}")
                        sock.sendto(response_message, addr)
                except Exception as e:
                    print("[ERROR] Discovery-Server:", e)
                    break

            sock.close()

        thread = threading.Thread(target=discovery_loop, daemon=True)
        thread.start()

    def set_pointcloud(self, xyz, rgb):
        self.xyz = xyz
        self.rgb = rgb