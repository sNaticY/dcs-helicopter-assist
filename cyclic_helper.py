from numpy import sign
import config
from pid_calculator import PIDCalculator
from pid_calculator_new import PIDCalculatorNew
from utils import EMA, world_to_body_velocity


class CyclicHelper:
    def __init__(self):
        # 参数
        self.dt = 0.02
        self.max_auth = 0.65
        self.learning_rate = 0.001

        # 状态
        self.prev_up = 0.0

        self.target_pitch = 0.0
        self.pitch_baseline = -0.105
        self.roll_baseline = -0.046

        self.cyclic_x_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)
        self.cyclic_y_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate * 10)

        self.right_offset_pid = PIDCalculatorNew(Kp_base=0.2, Ki=0.01, Kd=0.0)
        self.right_v_pid = PIDCalculatorNew(Kp_base=0.5, Ki=0.02, Kd=0.0)
        self.roll_pid = PIDCalculatorNew(Kp_base=1.0, Ki=0.1, Kd=0.0)
        self.roll_rate_pid = PIDCalculatorNew(Kp_base=1.0, Ki=0.05, Kd=0.02)

        # pitch 歷史紀錄
        self.pitch_history = []
        self.ema_acc_up = EMA(config.EMA_ALPHA)


    def update(self, motion_state, blocked = False, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):
        
        # 计算当前速度和加速度
        self.prev_up = motion_state.up_v
        
        pre_error_x = 0.0
        pre_error_y = 0.0
        pre_error_acc_up = 0.0

        # 只有在懸停且穩定時才調整基準線
        if hovering and abs(motion_state.pitch_rate) < 0.01 and abs(-motion_state.pitch - self.pitch_baseline) < 0.02:
            self.pitch_baseline -= 0.0002 * motion_state.forward_v
        if hovering and abs(motion_state.roll_rate) < 0.01 and abs(motion_state.roll - self.roll_baseline) < 0.02:
            self.roll_baseline -= 0.0002 * motion_state.right_v

        # 記錄 pitch 歷史
        if not hovering and abs(manual_cyclic_y) > 0.02 and not hovering and abs(manual_cyclic_x) < 0.05:
            self.pitch_history.append(motion_state.pitch)
            if len(self.pitch_history) > int(1.0 / self.dt):
                self.pitch_history.pop(0)

        self.target_pitch = self.get_pitch_avg() if not hovering else 0.0

        if hovering:
            pre_error_x = min(max(-0.05*(motion_state.right_v) + self.roll_baseline, -0.2), 0.2)
            pre_error_y = min(max(-0.05*(motion_state.forward_v) + self.pitch_baseline, -0.15), 0.15)
        elif abs(manual_cyclic_y) < 0.05:
            pre_error_acc_up = 0 #sign(Pitch) * 2 * acc_up * min(max(Pitch - target_pitch, -0.2), 0.2)

       

        self.cyclic_x_pid.update(-motion_state.roll + 1 * pre_error_x, -motion_state.roll_rate, manual=manual_cyclic_x)
        self.cyclic_y_pid.update(motion_state.pitch - self.target_pitch + 1 * pre_error_y, motion_state.pitch_rate, preError=pre_error_acc_up, manual=manual_cyclic_y, forgetting_factor=0.0)

        if blocked:
            self.pitch_history.clear()
            return None, None
        
        if hovering:
            self.pitch_history.clear()
        
        x_result = self.cyclic_x_pid.auto + self.cyclic_x_pid.balanced
        y_result = self.cyclic_y_pid.auto + self.cyclic_y_pid.balanced

        if abs(manual_cyclic_x) > 0.01:
            x_result += manual_cyclic_x
        if abs(manual_cyclic_y) > 0.01:
            y_result += manual_cyclic_y
        return x_result, y_result

    def debug_print(self):
        return f"target_pitch={self.get_pitch_avg():+.3f}"

    def get_pitch_avg(self):
        if self.pitch_history:
            return sum(self.pitch_history) / len(self.pitch_history)
        else:
            return 0.0
        
    def reset(self):
        self.pitch_history.clear()
        self.target_pitch = 0.0
        self.roll_baseline = -0.046
        self.pitch_baseline = -0.105
        self.cyclic_x_pid.reset()
        self.cyclic_y_pid.reset()
        self.prev_up = 0.0
        self.forward = 0.0
        self.right = 0.0
        self.up = 0.0
        self.ema_acc_up = EMA(config.EMA_ALPHA)
