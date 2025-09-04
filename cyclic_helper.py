import math
import config
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

    def update(self, Vx, Vy, Vz, Pitch, Roll, PitchRate, RollRate):
        # 坐标转换
        self.forward, self.right, _ = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, 0)
        self.forward_acc = self.ema_forward_acc.update((self.forward - self.prev_forward) / self.dt)
        self.right_acc = self.ema_right_acc.update((self.right - self.prev_right) / self.dt)
        self.prev_forward = self.forward
        self.prev_right = self.right
        PitchRate = self.ema_pitch_rate.update(PitchRate)
        RollRate = self.ema_roll_rate.update(RollRate)

        # 自适应比例增益
        KpRoll = self.Kp_roll_base + self.adaptive_factor * abs(Roll)
        KpPitch = self.Kp_pitch_base + self.adaptive_factor * abs(Pitch)
        KpForward = self.Kp_v_base + self.adaptive_factor * abs(self.forward)
        KpRight = self.Kp_v_base + self.adaptive_factor * abs(self.right)

        # 积分项
        self.pitch_integral += Pitch * self.dt
        self.pitch_integral = max(min(self.pitch_integral, self.integral_max), self.integral_min)
        self.roll_integral += Roll * self.dt
        self.roll_integral = max(min(self.roll_integral, self.integral_max), self.integral_min)
        self.forward_integral += self.forward * self.dt
        self.forward_integral = max(min(self.forward_integral, self.integral_max), self.integral_min)
        self.right_integral += self.right * self.dt
        self.right_integral = max(min(self.right_integral, self.integral_max), self.integral_min)

        # 横滚通道
        self.cyclic_x_auto = -1 * (KpRoll * Roll + self.ki * self.roll_integral - self.kd * RollRate)
        self.cyclic_x_auto = max(min(self.cyclic_x_auto, self.max_auth), -self.max_auth)
        #self.cyclic_y_auto += 0.05 * (KpRight * self.right + self.ki * self.right_integral - self.kd * self.right_acc)
        #self.cyclic_y_auto = max(min(self.cyclic_y_auto, self.max_auth), -self.max_auth)

        # 俯仰通道
        self.cyclic_y_auto = 1 * (KpPitch * Pitch + self.ki * self.pitch_integral + self.kd * PitchRate)
        self.cyclic_y_auto = max(min(self.cyclic_y_auto, self.max_auth), -self.max_auth)
        #forward_auto += 0.05 * (KpForward * self.forward + self.ki * self.forward_integral - self.kd * self.forward_acc)
        #forward_auto = max(min(forward_auto, self.max_auth), -self.max_auth)


        if abs(Roll) < self.threshold and abs(RollRate) < self.threshold:
            delta = self.learning_rate * (self.cyclic_x_auto)
            self.balanced_cyclic_x += delta

        if abs(Pitch) < self.threshold and abs(PitchRate) < self.threshold:
            delta = self.learning_rate * (self.cyclic_y_auto)
            self.balanced_cyclic_y += delta

        return 1 * (self.cyclic_x_auto + self.balanced_cyclic_x), 1 * (self.cyclic_y_auto + self.balanced_cyclic_y)

    def debug_print(self):
        return f"Vf={self.forward:+.2f} Vr={self.right:+.2f} | Af={self.forward_acc:+.2f} Ar={self.right_acc:+.2f} | auto_cyclic_x={self.cyclic_x_auto:+.2f} auto_cyclic_y={self.cyclic_y_auto:+.2f} | balanced_cyclic_x={self.balanced_cyclic_x:+.2f} balanced_cyclic_y={self.balanced_cyclic_y:+.2f}"