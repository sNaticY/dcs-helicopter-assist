from numpy import sign
import config
from pid_calculator import PIDCalculator
from pid_calculator_new import PIDCalculatorNew
from utils import EMA, world_to_body_velocity


class CyclicHelper:
    def __init__(self):
        # 参数
        self.dt = 0.02

        # 状态
        self.target_pitch = 0.0
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_pos_z = 0.0


        self.right_offset_pid = PIDCalculatorNew(Kp_base=1.0, Ki=0.3, Kd=0.01, skip=2, max_auth=2.0)
        self.right_v_pid = PIDCalculatorNew(Kp_base=0.08, Ki=0.0002, Kd=0.0, skip=3, max_auth=0.12)
        self.roll_pid = PIDCalculatorNew(Kp_base=6.0, Ki=0.0012, Kd=0.0, skip=4, max_auth=2.0)
        self.roll_rate_pid = PIDCalculatorNew(Kp_base=0.1, Ki=0.0001, Kd=0.05, stable_threshold=0.02)

        self.forward_offset_pid = PIDCalculatorNew(Kp_base=1.0, Ki=0.3, Kd=0.01, skip=2, max_auth=2.0)
        self.forward_v_pid = PIDCalculatorNew(Kp_base=0.12, Ki=0.0004, Kd=0.0, skip=3, max_auth=0.12)
        self.pitch_pid = PIDCalculatorNew(Kp_base=3.0, Ki=0.0012, Kd=0.0, skip=4, max_auth=2.0)
        self.pitch_rate_pid = PIDCalculatorNew(Kp_base=2.0, Ki=0.002, Kd=0.5, stable_threshold=0.02)
        

        # pitch 歷史紀錄
        self.pitch_history = []
        self.prev_manual_active = False


    def update(self, motion_state, blocked = False, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):
        manual_active = abs(manual_cyclic_x) >= 0.02 or abs(manual_cyclic_y) >= 0.02

        # 記錄 pitch 歷史
        if not hovering and abs(manual_cyclic_y) > 0.02 and not hovering and abs(manual_cyclic_x) < 0.05:
            self.pitch_history.append(motion_state.pitch)
            if len(self.pitch_history) > int(0.1 / self.dt):
                self.pitch_history.pop(0)

        self.target_pitch = self.get_pitch_avg() if not hovering else 0.0

        if hovering:
            self.roll_pid.update_ki(0.0012)
            self.pitch_pid.update_ki(0.0012)
        else:
            self.roll_pid.update_ki(0.12)
            self.pitch_pid.update_ki(0.12)

        forward_offset, right_offset, up_offset = motion_state.get_position_delta(self.last_pos_x, self.last_pos_y, self.last_pos_z)
        if abs(forward_offset) > 0.5:
            forward_offset = sign(forward_offset) * 0.5
        if abs(right_offset) > 0.5:
            right_offset = sign(right_offset) * 0.5

        if not manual_active:
            if hovering:
                if self.right_offset_pid.is_available():
                    self.right_offset_pid.update(error=right_offset, rate=None, delta_time=self.dt * self.right_v_pid.skip * self.roll_pid.skip * self.right_offset_pid.skip)
                if self.forward_offset_pid.is_available():
                    self.forward_offset_pid.update(error=forward_offset, rate=None, delta_time=self.dt * self.forward_v_pid.skip * self.pitch_pid.skip * self.forward_offset_pid.skip)
                    self.last_pos_x = motion_state.x
                    self.last_pos_y = motion_state.y
                    self.last_pos_z = motion_state.z
                if self.right_v_pid.is_available():
                    self.right_v_pid.update(error=-motion_state.right_v + self.right_offset_pid.auto, rate=None, delta_time=self.dt * self.right_v_pid.skip * self.roll_pid.skip)
                    self.right_offset_pid.update_skip()
                if self.forward_v_pid.is_available():
                    self.forward_v_pid.update(error=-motion_state.forward_v + self.forward_offset_pid.auto, rate=None, delta_time=self.dt * self.forward_v_pid.skip * self.pitch_pid.skip)
                    self.forward_offset_pid.update_skip()
            if self.roll_pid.is_available():
                self.roll_pid.update(error=-motion_state.roll+self.right_v_pid.auto, rate=None, delta_time=self.dt * self.roll_pid.skip)
                self.right_v_pid.update_skip()
            if self.pitch_pid.is_available():
                self.pitch_pid.update(error=motion_state.pitch+self.forward_v_pid.auto-self.target_pitch, rate=None, delta_time=self.dt * self.pitch_pid.skip)
                self.forward_v_pid.update_skip()
            self.roll_rate_pid.update(error=-motion_state.roll_rate + self.roll_pid.auto, rate=None, delta_time=self.dt)
            self.pitch_rate_pid.update(error=motion_state.pitch_rate + self.pitch_pid.auto, rate=None, delta_time=self.dt)
            self.roll_pid.update_skip()
            self.pitch_pid.update_skip()

        if blocked:
            self.pitch_history.clear()
            return None, None
        
        if hovering:
            self.pitch_history.clear()
        
        x_result = self.roll_rate_pid.auto
        y_result = self.pitch_rate_pid.auto

        if abs(manual_cyclic_x) > 0.01:
            x_result += manual_cyclic_x
        if abs(manual_cyclic_y) > 0.01:
            y_result += manual_cyclic_y

        self.prev_manual_active = manual_active
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
