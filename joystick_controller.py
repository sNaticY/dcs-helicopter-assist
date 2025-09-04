import pyvjoy
import time
import threading
import random
from utils import clamp, norm_to_vjoy


try:
    from inputs import get_gamepad
except ImportError:
    get_gamepad = None

class JoystickController(threading.Thread):
    """
    監控物理搖桿輸入，並直接發送 vJoy 指令。
    """
    def __init__(self, assist, deadzone=0.15, resume_delay=0.2, vjoy_id=3):
        super().__init__(daemon=True)
        self.assist = assist
        self.deadzone = deadzone



    


    def run(self):
        if not get_gamepad:
            print("[WARN] 未安裝 inputs，搖桿監控不可用")
            return

        SENS_CYCLIC = 0.01
        SENS_RUDDER = 0.3

        while True:
            events = get_gamepad()
            ly = rx = 0.0
            for e in events:
                if e.code == "ABS_Y":
                    ly = e.state / 32767.0
                if e.code == "ABS_RX":
                    rx = e.state / 32767.0
            # 相對累加全局偏移
            # if abs(ly) > self.deadzone and not self.assist.cyclic_blocked:
            #     self.assist.pitch_offset += ly * SENS_CYCLIC
            #     self.assist.pitch_offset = clamp(self.assist.pitch_offset, -1, 1)
            # if abs(rx) > self.deadzone and not self.assist.rudder_blocked:
            #     self.assist.yawRate_offset = rx * SENS_RUDDER
            #     self.assist.yawRate_offset = clamp(self.assist.yawRate_offset, -1, 1)

            # 發送 vJoy 指令（自動控制優先，否則歸中）
            # cx = getattr(self.assist, "cyclic_x", 0.0)
            # cy = getattr(self.assist, "cyclic_y", 0.0)
            # rud = getattr(self.assist, "rudder", 0.0)
            # if self.assist.cyclic_enabled or self.assist.rudder_enabled:
            #     self.write_vjoy(cx, cy, rud)

            time.sleep(0.01)