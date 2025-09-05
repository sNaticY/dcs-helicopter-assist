import pyvjoy
import time
import threading
import random
from utils import clamp, norm_to_vjoy

try:
    import inputs
except ImportError:
    inputs = None

class JoystickMonitor(threading.Thread):
    """
    只監控物理搖桿輸入，忽略 vJoy 虛擬設備。
    """
    def __init__(self, assist):
        super().__init__(daemon=True)
        self.assist = assist

        # 過濾出物理搖桿設備
        self.physical_gamepads = []
        self.lx = 0.0
        self.ly = 0.0
        self.rx = 0.0
        if inputs:
            for d in inputs.devices.gamepads:
                name = getattr(d, 'name', '').lower()
                if "vjoy" not in name and "virtual" not in name:
                    self.physical_gamepads.append(d)

    def run(self):
        if not inputs or not self.physical_gamepads:
            print("[WARN] 未找到物理搖桿，搖桿監控不可用")
            return

        while True:
            for pad in self.physical_gamepads:
                try:
                    events = pad.read()
                except Exception:
                    continue
                for e in events:
                    if e.code == "ABS_X":
                        self.lx = e.state / 32767.0
                    if e.code == "ABS_Y":
                        self.ly = e.state / 32767.0
                    if e.code == "ABS_RX":
                        self.rx = e.state / 32767.0

            if not self.assist.cyclic_blocked:
                self.assist.manual_cyclic_x = self.lx
            if not self.assist.cyclic_blocked:
                self.assist.manual_cyclic_y = self.ly
            if not self.assist.rudder_blocked:
                self.assist.manual_rudder = self.rx

            time.sleep(0.00001)