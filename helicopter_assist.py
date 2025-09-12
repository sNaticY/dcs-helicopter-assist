import random
import time
import keyboard
import winsound

# from numpy import sign   # 不再需要 random/sign 处理扰动
import pyvjoy

from config import *
from motion_state import MotionState
from utils import EMA, apply_curve, norm_to_vjoy
from dcs_telemetry import DcsTelemetry
from cyclic_helper import CyclicHelper
from rudder_helper import RudderHelper
from joystick_monitor import JoystickMonitor
from input_processor import InputProcessor


LOOP_DT = 0.02  # 主循环周期（秒）


class HelicopterAssist:
    def __init__(self):
        # vJoy 设备
        self.vjoy = pyvjoy.VJoyDevice(VJOY_DEVICE_ID)

        # 模式/开关
        self.cyclic_mode = 0
        self.cyclic_enabled = False
        self.cyclic_hovering = False
        self.rudder_enabled = False

        # 阻塞状态（例如键盘按下时暂停输出）
        self.cyclic_blocked = False
        self.rudder_blocked = False

        # 控制辅助
        self.cyclic_helper = CyclicHelper()
        self.rudder_helper = RudderHelper()
        self.motion_state = MotionState()

        # 手动原始输入（JoystickMonitor 仍写这里）
        self.manual_cyclic_x = 0.0
        self.manual_cyclic_y = 0.0
        self.manual_rudder = 0.0

        # 新增：统一输入处理器（限速 + 曲线整形）
        self.inputs = InputProcessor(
            expo_cyclic=0.5,
            expo_rudder=0.5,
            rate_up=1.0,
            rate_down=2.0,
        )

        self.neutral_all()

    def compute_outputs(self, state: dict):
        # 读取最新状态（保持键名与导出一致，局部变量采用蛇形命名）
        vx = state.get("Vx", 0.0)
        vy = state.get("Vy", 0.0)
        vz = state.get("Vz", 0.0)
        ax = state.get("Ax", 0.0)
        ay = state.get("Ay", 0.0)
        az = state.get("Az", 0.0)

        pitch = state.get("Pitch", 0.0)
        roll = state.get("Roll", 0.0)
        yaw = state.get("Yaw", 0.0)

        roll_rate = state.get("RollRate", 0.0)
        pitch_rate = state.get("PitchRate", 0.0)
        yaw_rate = state.get("YawRate", 0.0)

        pos_x = state.get("PosX", 0.0)
        pos_y = state.get("PosY", 0.0)
        pos_z = state.get("PosZ", 0.0)

        # 更新运动状态
        self.motion_state.update(
            vx, vy, vz,
            pitch, roll, yaw,
            ax, ay, az,
            pitch_rate, roll_rate, yaw_rate,
            pos_x, pos_y, pos_z,
        )

        # RUDDER 控制（使用处理后的手动输入）
        if self.rudder_enabled:
            rudder = self.rudder_helper.update(
                yaw, yaw_rate, self.rudder_blocked, self.inputs.input_rudder
            )
        else:
            rudder = self.inputs.input_rudder

        # CYCLIC 控制（使用处理后的手动输入）
        if self.cyclic_enabled:
            cyclic_x, cyclic_y = self.cyclic_helper.update(
                vx, vy, vz,
                pitch, roll, yaw,
                pitch_rate, roll_rate,
                self.cyclic_blocked,
                self.inputs.input_cyclic_x,
                self.inputs.input_cyclic_y,
                self.cyclic_hovering,
            )
        else:
            cyclic_x = self.inputs.input_cyclic_x
            cyclic_y = self.inputs.input_cyclic_y

        return cyclic_x, cyclic_y, rudder

    def loop(self, tel: DcsTelemetry):
        last_debug = time.time()
        last_time = time.time()

        while True:
            now = time.time()
            dt = now - last_time
            last_time = now

            # 将外部写入的原始手动输入交给处理器，并更新
            self.inputs.set_manual(self.manual_cyclic_x, self.manual_cyclic_y, self.manual_rudder)
            self.inputs.update(dt)

            state = tel.latest
            cyclic_x, cyclic_y, rudder = self.compute_outputs(state)

            self.cyclic_x = cyclic_x
            self.cyclic_y = cyclic_y
            self.rudder = rudder

            if not self.cyclic_blocked and not self.rudder_blocked:
                self.write_vjoy(cyclic_x, cyclic_y, rudder)

            if now - last_debug > 1.0:
                last_debug = now
                print(f"{tel.debug_print()} | {self.cyclic_helper.debug_print()} | {self.debug_print()}")

            time.sleep(LOOP_DT)

    def debug_print(self) -> str:
        parts = []
        if self.cyclic_x is not None and self.cyclic_y is not None:
            parts.append(f"CyclicX={self.cyclic_x:+.2f} CyclicY={self.cyclic_y:+.2f}")
        if self.rudder is not None:
            parts.append(f"Rudder={self.rudder:+.2f}")
        if self.rudder_helper.target_yaw is not None:
            parts.append(f"TargetYaw={self.rudder_helper.target_yaw:+.2f}")
        return " ".join(parts)

    def neutral_all(self):
        # 清零输出并复位扰动记忆
        self.inputs.reset_dither()
        self.write_vjoy(0.0, 0.0, 0.0)

    def write_vjoy(self, cyclic_x, cyclic_y, rudder):
        if cyclic_x is not None:
            cyclic_x = self.inputs.apply_output_dither("cyclic_x", cyclic_x)
            self.vjoy.set_axis(pyvjoy.HID_USAGE_X, norm_to_vjoy(cyclic_x))

        if cyclic_y is not None:
            cyclic_y = self.inputs.apply_output_dither("cyclic_y", cyclic_y)
            self.vjoy.set_axis(pyvjoy.HID_USAGE_Y, norm_to_vjoy(-cyclic_y))

        if rudder is not None:
            rudder = self.inputs.apply_output_dither("rudder", rudder)
            self.vjoy.set_axis(pyvjoy.HID_USAGE_RZ, norm_to_vjoy(rudder))


def main():
    tel = DcsTelemetry(UDP_HOST, UDP_PORT)
    tel.start()
    time.sleep(LOOP_DT)

    assist = HelicopterAssist()

    jm = JoystickMonitor(assist)
    jm.start()

    def on_keyboard_event(event):
        if event.event_type == "down":
            assist.cyclic_blocked = True
            assist.rudder_blocked = True
        elif event.event_type == "up":
            assist.cyclic_blocked = False
            assist.rudder_blocked = False
            assist.rudder_helper.reset()
            assist.cyclic_helper.reset()

    keyboard.hook_key(TOGGLE_PAUSE_HOTKEY, on_keyboard_event, suppress=False)
    keyboard.add_hotkey(TOGGLE_CYCLIC_HOTKEY, lambda: toggle_cyclic(assist))
    keyboard.add_hotkey(TOGGLE_RUDDER_HOTKEY, lambda: toggle_rudder(assist))

    assist.loop(tel)


def toggle_cyclic(assist: HelicopterAssist):
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


def toggle_rudder(assist: HelicopterAssist):
    assist.rudder_enabled = not assist.rudder_enabled
    assist.rudder_helper.reset()
    play_beep("on" if assist.rudder_enabled else "off")
    print(f"[INFO] Rudder assist: {'ON' if assist.rudder_enabled else 'OFF'}")


def play_beep(mode: str):
    if mode == "on":
        winsound.Beep(1200, 120)  # 高频短音
    elif mode == "hover":
        winsound.Beep(900, 120)   # 第一声
        time.sleep(0.08)          # 间隔
        winsound.Beep(900, 120)   # 第二声
    elif mode == "off":
        winsound.Beep(500, 120)   # 低频短音


if __name__ == "__main__":
    main()
