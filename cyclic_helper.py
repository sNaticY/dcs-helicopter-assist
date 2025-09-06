from numpy import sign
import config
from pid_calculator import PIDCalculator
from utils import EMA, world_to_body_velocity


class CyclicHelper:
    def __init__(self):
        # 参数
        self.dt = 0.02
        self.max_auth = 0.65
        self.learning_rate = 0.001

        # 状态
        self.prev_forward = 0.0
        self.prev_right = 0.0
        self.prev_up = 0.0

        # 最近一次 update 的中間量
        self.forward = 0.0
        self.right = 0.0
        self.pitch_baseline = -0.085
        self.roll_baseline = -0.045

        self.cyclic_x_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)
        self.cyclic_y_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate * 10)

        # pitch 歷史紀錄
        self.pitch_history = []
        self.ema_acc_up = EMA(config.EMA_ALPHA)


    def update(self, Vx, Vy, Vz, Pitch, Roll, Yaw, PitchRate, RollRate, blocked = False, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):
        
        # 计算当前速度和加速度
        self.forward, self.right, _ = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, Yaw)
        acc_up = self.ema_acc_up.update((Vz - self.prev_up) / self.dt)
        
        pre_error_x = 0.0
        pre_error_y = 0.0
        pre_error_acc_up = 0.0

        # 只有在懸停且穩定時才調整基準線
        if hovering and abs(PitchRate) < 0.01 and abs(-Pitch -self.pitch_baseline) < 0.02:
            self.pitch_baseline -= 0.0002 * self.forward
        if hovering and abs(RollRate) < 0.01 and abs(Roll -self.roll_baseline) < 0.02:
            self.roll_baseline -= 0.0002 * self.right

        if hovering:
            pre_error_x = min(max(-0.05*(self.right) + self.roll_baseline, -0.2), 0.2)
            pre_error_y = min(max(-0.05*(self.forward) + self.pitch_baseline, -0.15), 0.15)
        elif abs(manual_cyclic_y) < 0.05:
            pre_error_acc_up = 0.0001 * acc_up * min(max(Pitch, -1), 1)

        # 記錄 pitch 歷史
        if not hovering and abs(manual_cyclic_y) > 0.02 and not hovering and abs(manual_cyclic_x) < 0.05:
            self.pitch_history.append(Pitch)
            if len(self.pitch_history) > int(1.0 / self.dt):
                self.pitch_history.pop(0)

        target_pitch = self.get_pitch_avg() if not hovering else 0.0

        self.cyclic_x_pid.update(-Roll + 1 * pre_error_x, -RollRate, manual=manual_cyclic_x)
        self.cyclic_y_pid.update(Pitch - target_pitch + 1 * pre_error_y, PitchRate, preError=pre_error_acc_up, manual=manual_cyclic_y, forgetting_factor=0.0)

        if blocked:
            self.pitch_history.clear()
            return None, None
        
        if hovering:
            self.pitch_history.clear()
        
        x_result = self.cyclic_x_pid.auto + self.cyclic_x_pid.balanced
        y_result = self.cyclic_y_pid.auto + self.cyclic_y_pid.balanced

        if abs(manual_cyclic_x) > 0.02:
            x_result = manual_cyclic_x + self.cyclic_x_pid.balanced
        if abs(manual_cyclic_y) > 0.02:
            y_result = manual_cyclic_y + self.cyclic_y_pid.balanced

        return x_result, y_result

    def debug_print(self):
        return f"Vf={self.forward:+.2f} Vr={self.right:+.2f} | balanced_cyclic_x={self.cyclic_x_pid.balanced:+.2f} balanced_cyclic_y={self.cyclic_y_pid.balanced:+.2f} | pitch_baseline={self.pitch_baseline:+.3f} roll_baseline={self.roll_baseline:+.3f} | target_pitch={self.get_pitch_avg():+.3f}"

    def get_pitch_avg(self):
        if self.pitch_history:
            return sum(self.pitch_history) / len(self.pitch_history)
        else:
            return 0.0
        
    def reset(self):
        self.pitch_history.clear()
        self.target_pitch = 0.0
        self.roll_baseline = -0.045
        self.pitch_baseline = -0.085


