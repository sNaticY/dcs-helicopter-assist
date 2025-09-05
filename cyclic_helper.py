import math
import config
from pid_calculator import PIDCalculator
from utils import EMA, world_to_body_velocity


class CyclicHelper:
    def __init__(self):
        # 参数
        self.Kp_roll_base = 0.5
        self.Kp_pitch_base = 0.5
        self.Kp_v_base = 0.5
        self.ki = 0.1
        self.kd = 0.08
        self.adaptive_factor = 0.003
        self.dt = 0.02
        self.max_auth = 0.35
        self.integral_max = 5.0
        self.integral_min = -5.0
        self.learning_rate = 0.001
        self.threshold = 0.15

        # 状态
        self.pitch_integral = 0.0
        self.roll_integral = 0.0

        self.prev_forward = 0.0
        self.prev_right = 0.0
        self.forward_integral = 0.0
        self.right_integral = 0.0

        self.cyclic_x_auto = 0.0
        self.cyclic_y_auto = 0.0
        self.right_auto = 0.0
        self.forward_auto = 0.0
        self.balanced_cyclic_x = 0.0
        self.balanced_cyclic_y = 0.0

        # 最近一次 update 的中間量
        self.forward = 0.0
        self.right = 0.0
        self.forward_acc = 0.0
        self.right_acc = 0.0

        self.ema_pitch_rate = EMA(config.EMA_ALPHA)
        self.ema_roll_rate = EMA(config.EMA_ALPHA)
        self.ema_forward_acc = EMA(config.EMA_ALPHA)
        self.ema_right_acc = EMA(config.EMA_ALPHA)

        self.cyclic_x_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)
        self.cyclic_y_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)
        # self.v_x_pid = PIDCalculator(max_auth=0.2, learning_rate=self.learning_rate)
        # self.v_y_pid = PIDCalculator(max_auth=0.2, learning_rate=self.learning_rate)

    def update(self, Vx, Vy, Vz, Pitch, Roll, Yaw, PitchRate, RollRate, blocked = False, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):

        pre_error_x = 0.0
        pre_error_y = 0.0
        
        self.forward, self.right, _ = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, Yaw)
        self.forward_acc = self.ema_forward_acc.update((self.forward - self.prev_forward) / self.dt)
        self.right_acc = self.ema_right_acc.update((self.right - self.prev_right) / self.dt)
        self.prev_forward = self.forward
        self.prev_right = self.right

        if hovering:
            # pre_error_roll = Roll if abs(self.right) > 5 else 0.0
            # pre_error_pitch = Pitch if abs(self.forward) > 5 else 0.0
            # self.v_x_pid.update(-0.08 * self.right, -0.1*self.right_acc, preError=pre_error_roll)
            # self.v_y_pid.update(-0.08 * self.forward, -0.1*self.forward_acc, preError=pre_error_pitch)
            pre_error_x = min(max(-0.05*(self.right) - 0.045, -0.2), 0.2)
            pre_error_y = min(max(-0.05*(self.forward) - 0.085, -0.2), 0.2)

        self.cyclic_x_pid.update(-Roll + 1 * pre_error_x, -RollRate, manual=manual_cyclic_x)
        self.cyclic_y_pid.update(Pitch + 1 * pre_error_y, PitchRate, manual=manual_cyclic_y)

        if blocked:
            return None, None

        # x_result = self.smooth_blend(self.cyclic_x_pid.auto + self.cyclic_x_pid.balanced, 0.05*(self.v_x_pid.auto + self.v_x_pid.balanced), (abs(self.right)-5) if hovering else 50)
        # y_result = self.smooth_blend(self.cyclic_y_pid.auto + self.cyclic_y_pid.balanced, 0.05*(self.v_y_pid.auto + self.v_y_pid.balanced), (abs(self.forward)-5) if hovering else 50)

        x_result = self.cyclic_x_pid.auto + self.cyclic_x_pid.balanced
        y_result = self.cyclic_y_pid.auto + self.cyclic_y_pid.balanced

        x_balanced = self.cyclic_x_pid.balanced # if not hovering else self.v_x_pid.balanced
        y_balanced = self.cyclic_y_pid.balanced # if not hovering else self.v_y_pid.balanced
        if abs(manual_cyclic_x) > 0.02:
            x_result = manual_cyclic_x + x_balanced
        if abs(manual_cyclic_y) > 0.02:
            y_result = manual_cyclic_y + y_balanced

        return x_result, y_result

    def debug_print(self):
        return f"Vf={self.forward:+.2f} Vr={self.right:+.2f} | balanced_cyclic_x={self.balanced_cyclic_x:+.2f} balanced_cyclic_y={self.balanced_cyclic_y:+.2f}"
    

    def smooth_blend(self, val1, val2, speed, threshold=5.0):
        # speed: abs(v_x) 或 abs(v_y)
        # threshold: 過渡區間最大速度
        alpha = max(min(speed / threshold, 1.0), 0.0)
        # alpha 越大越偏向 val，越小越偏向 auto_val
        return alpha * val1 + (1 - alpha) * val2