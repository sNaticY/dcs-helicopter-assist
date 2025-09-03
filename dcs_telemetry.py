import json
import math
import time
import socket
import threading
import numpy as np

class DcsTelemetry(threading.Thread):
    """
    接收 Export.lua 发来的 JSON 行（UDP），解析为最新状态。
    """
    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))
        self.latest = {
            "Vx": 0.0, "Vy": 0.0, "Vz": 0.0,
            "Ax": 0.0, "Ay": 0.0, "Az": 0.0,
            "Pitch": 0.0, "Roll": 0.0, "Yaw": 0.0,
            "PitchRate": 0.0, "RollRate": 0.0, "YawRate": 0.0,
            "t": time.time()
        }

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                line = data.decode("utf-8").strip()
                for part in line.split("\n"):
                    if not part:
                        continue
                    js = json.loads(part)
                    js["t"] = time.time()
                    self.latest = js
            except Exception:
                pass
    def debug_print(self):
        return f"Pitch={self.latest['Pitch']:.2f} Roll={self.latest['Roll']:.2f} Yaw={self.latest['Yaw']:.2f} | PitchRate={self.latest['PitchRate']:.2f} RollRate={self.latest['RollRate']:.2f} YawRate={self.latest['YawRate']:.2f}"
