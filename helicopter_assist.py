import random
import pyvjoy
from config import *
from utils import EMA, norm_to_vjoy
from dcs_telemetry import DcsTelemetry
from cyclic_helper import CyclicHelper
from rudder_helper_old import RudderHelperOld
from rudder_helper import RudderHelper
from joystick_monitor import JoystickMonitor
import keyboard
import time



class HelicopterAssist:
    def __init__(self):
        self.j = pyvjoy.VJoyDevice(VJOY_DEVICE_ID)

        self.cyclic_enabled = False
        self.rudder_enabled = False
        self.cyclic_blocked = False
        self.rudder_blocked = False

        self.cyclic_helper = CyclicHelper()

        # self.rudder_helper = RudderHelperOld(
        #     Kp_rudder=Kp_rudder,
        #     Kd_rudder=Kd_rudder,
        #     K_bias=K_bias,
        #     max_auth=MAX_RUDDER_AUTH,
        #     dt=0.04
        # )

        self.rudder_helper = RudderHelper()

        self.ema_vx = EMA(EMA_ALPHA)
        self.ema_vy = EMA(EMA_ALPHA)
        self.ema_vz = EMA(EMA_ALPHA)

        self.previous_cyclic_x = 0.0
        self.previous_cyclic_y = 0.0
        self.previous_rudder = 0.0

        self.manual_cyclic_x = 0.0
        self.manual_cyclic_y = 0.0
        self.manual_rudder = 0.0

        self.neutral_all()

    def compute_outputs(self, s):
        Vx = self.ema_vx.update(s.get("Vx", 0.0))
        Vy = self.ema_vy.update(s.get("Vy", 0.0))
        Vz = self.ema_vz.update(s.get("Vz", 0.0))

        Pitch = s.get("Pitch", 0.0)  # 单位：度
        Roll  = s.get("Roll", 0.0)   # 单位：度
        Yaw   = s.get("Yaw", 0.0)    # 单位：度
        RollRate = s.get("RollRate", 0.0)
        PitchRate = s.get("PitchRate", 0.0)
        YawRate = s.get("YawRate", 0.0)

        # RUDDER 控制
        if self.rudder_enabled:
            rudder, balanced_rudder = self.rudder_helper.update(Yaw, YawRate, self.rudder_blocked, self.manual_rudder)
        else:
            rudder, balanced_rudder = None, None

        # CYCLIC 控制
        if self.cyclic_enabled and not self.cyclic_blocked:
            cyclic_x, cyclic_y = self.cyclic_helper.update(Vx, Vy, Vz, Pitch, Roll, PitchRate, RollRate)  
        else: 
            cyclic_x, cyclic_y = (None, None)

        return cyclic_x, cyclic_y, rudder, balanced_rudder

    def loop(self, tel):
        last_debug = time.time()
        while True:
            s = tel.latest
            cx, cy, rud, b_rud = self.compute_outputs(s)
            self.cyclic_x = cx
            self.cyclic_y = cy
            self.rudder = rud
            self.balanced_rudder = b_rud

            if self.cyclic_enabled or self.rudder_enabled:
                self.write_vjoy(cx, cy, rud)

            now = time.time()
            if now - last_debug > 1.0:
                last_debug = now
                print(f"{tel.debug_print()} | {self.cyclic_helper.debug_print()} | {self.debug_print()}")

            time.sleep(0.02)

    def debug_print(self):
        s = ""
        if self.cyclic_x is not None and self.cyclic_y is not None:
            s += f"CyclicX={self.cyclic_x:+.2f} CyclicY={self.cyclic_y:+.2f} "
        if self.rudder is not None:
            s += f"Rudder={self.manual_rudder:+.2f} BalRudder={self.balanced_rudder:+.2f} "
        if self.rudder_helper.target_yaw is not None:
            s += f"TargetYaw={self.rudder_helper.target_yaw:+.2f} "
        return s
    
    def neutral_all(self):
        self.write_vjoy(0.0, 0.0, 0.0)
    
    def write_vjoy(self, cx, cy, rud):
        if cx is not None:
            if abs(self.previous_cyclic_x - cx) < 0.001:
                cx = cx + random.uniform(-0.001, 0.001)
            self.previous_cyclic_x = cx
            self.j.set_axis(pyvjoy.HID_USAGE_X,  norm_to_vjoy(cx))
        if cy is not None:
            if abs(self.previous_cyclic_y - cy) < 0.001:
                cy = cy + random.uniform(-0.001, 0.001)
            self.previous_cyclic_y = cy
            self.j.set_axis(pyvjoy.HID_USAGE_Y,  norm_to_vjoy(-cy))
        if rud is not None:
            if abs(self.previous_rudder - rud) < 0.001:
                rud = rud + random.uniform(-0.001, 0.001)
            self.previous_rudder = rud
            self.j.set_axis(pyvjoy.HID_USAGE_RZ, norm_to_vjoy(rud))

def main():
    tel = DcsTelemetry(UDP_HOST, UDP_PORT)
    tel.start()
    time.sleep(0.02)
    assist = HelicopterAssist()
    jm = JoystickMonitor(assist)
    jm.start()

    def on_keyboard_event(e):
        if e.event_type == "down":
            assist.cyclic_blocked = True
            assist.rudder_blocked = True
        elif e.event_type == "up":
            assist.cyclic_blocked = False
            assist.rudder_blocked = False
            assist.pitch_offset = 0.0
            assist.yawRate_offset = 0.0
    keyboard.hook_key('left ctrl', on_keyboard_event, suppress=False)
    keyboard.hook_key('left shift', on_keyboard_event, suppress=False)

    # 加回 F9/F8 熱鍵
    keyboard.add_hotkey('f9', lambda: toggle_cyclic(assist))
    keyboard.add_hotkey('f8', lambda: toggle_rudder(assist))

    assist.loop(tel)

def toggle_cyclic(assist):
    assist.cyclic_enabled = not assist.cyclic_enabled
    print(f"[INFO] Cyclic assist: {'ON' if assist.cyclic_enabled else 'OFF'}")

def toggle_rudder(assist):
    assist.rudder_enabled = not assist.rudder_enabled
    print(f"[INFO] Rudder assist: {'ON' if assist.rudder_enabled else 'OFF'}")

if __name__ == "__main__":
    main()
