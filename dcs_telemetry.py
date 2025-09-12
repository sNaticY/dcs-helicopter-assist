import json
import time
import socket
import threading


class DcsTelemetry(threading.Thread):
    """
    接收 Export.lua 发来的 JSON 行（UDP），解析为最新状态。
    """

    UDP_BUF = 4096

    def __init__(self, host, port):
        super().__init__(daemon=True)
        self.host = host
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.host, self.port))

        # 最近一次状态（兼容缺失字段）
        self.latest = {
            "Vx": 0.0,
            "Vy": 0.0,
            "Vz": 0.0,
            "Ax": 0.0,
            "Ay": 0.0,
            "Az": 0.0,
            "Pitch": 0.0,
            "Roll": 0.0,
            "Yaw": 0.0,
            "PitchRate": 0.0,
            "RollRate": 0.0,
            "YawRate": 0.0,
            "PosX": 0.0,
            "PosY": 0.0,
            "PosZ": 0.0,
            "t": time.time(),
        }

        self._expected_keys = set(self.latest.keys())

    def run(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(self.UDP_BUF)
                text = data.decode("utf-8", errors="ignore")
                for line in text.splitlines():
                    if not line:
                        continue
                    obj = json.loads(line)
                    self._fill_defaults(obj)
                    obj["t"] = time.time()
                    self.latest = obj
            except Exception:
                # 忽略异常，继续接收
                pass

    def _fill_defaults(self, obj: dict):
        for k in self._expected_keys:
            if k not in obj:
                obj[k] = self.latest.get(k, 0.0)
