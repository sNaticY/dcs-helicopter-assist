import random
from numpy import sign
import pyvjoy
from config import *
from utils import EMA, apply_curve, norm_to_vjoy
from dcs_telemetry import DcsTelemetry
from cyclic_helper import CyclicHelper
from rudder_helper import RudderHelper
from joystick_monitor import JoystickMonitor
import keyboard
import time
import winsound



class HelicopterAssist:
    def __init__(self):
        self.j = pyvjoy.VJoyDevice(VJOY_DEVICE_ID)

        self.cyclic_mode = 0
        self.cyclic_enabled = False
        self.cyclic_hovering = False
        self.rudder_enabled = False

        self.cyclic_blocked = False
        self.rudder_blocked = False

        self.cyclic_helper = CyclicHelper()
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
        self.smoothed_cyclic_x = 0.0
        self.smoothed_cyclic_y = 0.0
        self.max_cyclic_rate_up = 1.0   # 增加時最大變化量（每秒）
        self.max_cyclic_rate_down = 2.0 # 減少時最大變化量（每秒）

        self.neutral_all()

    def compute_outputs(self, s):
        Vx = s.get("Vx", 0.0)
        Vy = s.get("Vy", 0.0)
        Vz = s.get("Vz", 0.0)

        Pitch = s.get("Pitch", 0.0)
        Roll  = s.get("Roll", 0.0)
        Yaw   = s.get("Yaw", 0.0)
        RollRate = s.get("RollRate", 0.0)
        PitchRate = s.get("PitchRate", 0.0)
        YawRate = s.get("YawRate", 0.0)

        # RUDDER 控制
        if self.rudder_enabled:
            rudder, balanced_rudder = self.rudder_helper.update(Yaw, YawRate, self.rudder_blocked, self.manual_rudder)
        else:
            rudder, balanced_rudder = self.manual_rudder, None

        # CYCLIC 控制
        if self.cyclic_enabled:
            manual_cyclic_x = apply_curve(self.smoothed_cyclic_x, expo=0.5)
            manual_cyclic_y = apply_curve(self.smoothed_cyclic_y, expo=0.5)
            cyclic_x, cyclic_y = self.cyclic_helper.update(Vx, Vy, Vz, Pitch, Roll, Yaw, PitchRate, RollRate, self.cyclic_blocked, manual_cyclic_x, manual_cyclic_y, self.cyclic_hovering)
        else:
            cyclic_x, cyclic_y = (apply_curve(self.manual_cyclic_x), apply_curve(self.manual_cyclic_y))

        return cyclic_x, cyclic_y, rudder, balanced_rudder

    def loop(self, tel):
        last_debug = time.time()
        last_time = time.time()
        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # 平滑限制左搖桿變化率（增大慢，減少快）
            def rate_limit(target, current, max_rate_up, max_rate_down, dt):
                # 判斷是否跨過零點
                if sign(target) != sign(current) and abs(current) > 0.001:
                    # 先快速回到零
                    delta = -current
                    max_delta = max_rate_down * dt
                    if abs(delta) > abs(max_delta):
                        delta = sign(delta) * abs(max_delta)
                    return current + delta
                else:
                    # 離開零用慢速，回歸零用快速
                    moving_towards_zero = abs(target) < abs(current)
                    if moving_towards_zero:
                        max_delta = max_rate_down * dt
                    else:
                        max_delta = max_rate_up * dt
                    delta = target - current
                    if abs(delta) > abs(max_delta):
                        delta = sign(delta) * abs(max_delta)
                    return current + delta

            self.smoothed_cyclic_x = rate_limit(
                self.manual_cyclic_x, self.smoothed_cyclic_x,
                self.max_cyclic_rate_up, self.max_cyclic_rate_down, dt
            )
            self.smoothed_cyclic_y = rate_limit(
                self.manual_cyclic_y, self.smoothed_cyclic_y,
                self.max_cyclic_rate_up, self.max_cyclic_rate_down, dt
            )

            s = tel.latest

            # 用平滑後的搖桿值作為手動輸入
            cx, cy, rud, b_rud = self.compute_outputs(s)

            self.cyclic_x = cx
            self.cyclic_y = cy
            self.rudder = rud
            self.balanced_rudder = b_rud

            if not self.cyclic_blocked and not self.rudder_blocked:
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
        if self.rudder is not None and self.balanced_rudder is not None:
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
            assist.rudder_helper.reset()
            assist.cyclic_helper.reset()

    keyboard.hook_key(TOGGLE_PAUSE_HOTKEY, on_keyboard_event, suppress=False)
    keyboard.add_hotkey(TOGGLE_CYCLIC_HOTKEY, lambda: toggle_cyclic(assist))
    keyboard.add_hotkey(TOGGLE_RUDDER_HOTKEY, lambda: toggle_rudder(assist))

    assist.loop(tel)

def toggle_cyclic(assist):
    assist.cyclic_mode = (assist.cyclic_mode + 1) % 3
    if assist.cyclic_mode == 0:
        assist.cyclic_enabled = False
        assist.cyclic_hovering = False
        assist.cyclic_helper.reset()
        play_beep("off")
        print("[INFO] Cyclic assist: OFF")
    elif assist.cyclic_mode == 1:
        assist.cyclic_enabled = True
        assist.cyclic_hovering = False
        play_beep("on")
        print("[INFO] Cyclic assist: ON (manual/auto)")
    elif assist.cyclic_mode == 2:
        assist.cyclic_enabled = True
        assist.cyclic_hovering = True
        play_beep("hover")
        print("[INFO] Cyclic assist: HOVERING")

def toggle_rudder(assist):
    assist.rudder_enabled = not assist.rudder_enabled
    assist.rudder_helper.reset()
    play_beep("on" if assist.rudder_enabled else "off")
    print(f"[INFO] Rudder assist: {'ON' if assist.rudder_enabled else 'OFF'}")

def play_beep(mode):
    if mode == "on":
        winsound.Beep(1200, 120)   # 高頻短音
    elif mode == "hover":
        winsound.Beep(900, 120)    # 第一聲
        time.sleep(0.08)           # 間隔
        winsound.Beep(900, 120)    # 第二聲
    elif mode == "off":
        winsound.Beep(500, 120)    # 低頻短音

if __name__ == "__main__":
    main()
