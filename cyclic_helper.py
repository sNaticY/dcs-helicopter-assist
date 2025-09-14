from numpy import sign
from config import EMA_ALPHA
from pid_calculator_new import PIDCalculatorNew
from utils import EMA


class CyclicHelper:
    def __init__(self):
        # 参数
        self.dt = 0.02

        # 状态
        self.target_pitch = 0.0
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_pos_z = 0.0


        self.right_offset_pid = PIDCalculatorNew(Kp_base=0.5, Ki=0.3, Kd=0.01, integral_max=40, skip=2, max_auth=20.0)
        self.right_v_pid = PIDCalculatorNew(Kp_base=0.02, Ki=0.0001, Kd=0.0, integral_max=0.68, skip=3, max_auth=0.34)
        self.roll_pid = PIDCalculatorNew(Kp_base=3.0, Ki=0.0, Kd=0.0, integral_max=10, skip=4, max_auth=5.0)
        self.roll_rate_pid = PIDCalculatorNew(Kp_base=0.2, Ki=0.0001, Kd=0.05, integral_max=2)

        self.forward_offset_pid = PIDCalculatorNew(Kp_base=0.5, Ki=0.3, Kd=0.01, integral_max=40, skip=2, max_auth=8.0)
        self.forward_v_pid = PIDCalculatorNew(Kp_base=0.05, Ki=0.0001, Kd=0.0, integral_max=0.68, skip=3, max_auth=0.34)
        self.pitch_pid = PIDCalculatorNew(Kp_base=3.0, Ki=0.0, Kd=0.0, integral_max=1, skip=4, max_auth=0.5)
        self.pitch_rate_pid = PIDCalculatorNew(Kp_base=2.0, Ki=0.0001, Kd=0.5, integral_max=2)
        
        self.ema_cyclic_x = EMA(EMA_ALPHA)
        self.ema_cyclic_y = EMA(EMA_ALPHA)

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

        if self.prev_manual_active and not manual_active :
            self.pitch_pid.manual_override(
                error=motion_state.pitch + self.forward_v_pid.auto - self.target_pitch,
                rate=None,
                delta_time=self.dt * self.pitch_pid.skip,
                manual_input=manual_cyclic_y + self.pitch_pid.auto,
                prev_error=motion_state.pitch + self.forward_v_pid.auto - self.target_pitch - (motion_state.pitch - motion_state.prev_pitch) * self.pitch_pid.skip,
                skip=self.pitch_pid.skip
            )
            self.pitch_rate_pid.manual_override(
                error=motion_state.pitch_rate + self.pitch_pid.auto,
                rate=None,
                delta_time=self.dt,
                manual_input=manual_cyclic_y + self.pitch_rate_pid.auto,
                prev_error=motion_state.pitch_rate + self.pitch_pid.auto - (motion_state.pitch_rate - motion_state.prev_pitch_rate),
                skip=1
            )

        if hovering:
            self.roll_pid.update_ki(0.0)
            self.pitch_pid.update_ki(0.0)
        else:
            self.roll_pid.update_ki(0.24)
            self.pitch_pid.update_ki(0.24)

        forward_offset, right_offset, up_offset = motion_state.get_position_delta(self.last_pos_x, self.last_pos_y, self.last_pos_z)
        if abs(forward_offset) > 3.5:
            forward_offset = sign(forward_offset) * 3.5
        if abs(right_offset) > 3.5:
            right_offset = sign(right_offset) * 3.5

        if not manual_active:
            if hovering:
                if self.right_offset_pid.is_available() and self.forward_offset_pid.is_available():
                    self.right_offset_pid.update(error=right_offset, rate=None, delta_time=self.dt * self.right_v_pid.skip * self.roll_pid.skip * self.right_offset_pid.skip)
                    self.forward_offset_pid.update(error=forward_offset, rate=None, delta_time=self.dt * self.forward_v_pid.skip * self.pitch_pid.skip * self.forward_offset_pid.skip)
                    self.last_pos_x = motion_state.x
                    self.last_pos_y = motion_state.y
                    self.last_pos_z = motion_state.z
                if self.right_v_pid.is_available() and self.forward_v_pid.is_available():
                    self.right_v_pid.update(error=-motion_state.right_v + self.right_offset_pid.auto, rate=None, delta_time=self.dt * self.right_v_pid.skip * self.roll_pid.skip)
                    self.right_offset_pid.update_skip()
                    self.forward_v_pid.update(error=-motion_state.forward_v + self.forward_offset_pid.auto, rate=None, delta_time=self.dt * self.forward_v_pid.skip * self.pitch_pid.skip)
                    self.forward_offset_pid.update_skip()
            if self.roll_pid.is_available() and self.pitch_pid.is_available():
                self.roll_pid.update(error=-motion_state.roll+self.right_v_pid.auto, rate=None, delta_time=self.dt * self.roll_pid.skip)
                self.right_v_pid.update_skip()
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
        self.ema_cyclic_x.update(x_result)
        y_result = self.pitch_rate_pid.auto
        self.ema_cyclic_y.update(y_result)

        if abs(manual_cyclic_x) > 0.01:
            x_result = self.ema_cyclic_x.y + manual_cyclic_x
        if abs(manual_cyclic_y) > 0.01:
            y_result = self.ema_cyclic_y.y + manual_cyclic_y

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
        self.right_offset_pid.reset()
        self.right_v_pid.reset()
        self.roll_pid.reset()
        self.roll_rate_pid.reset()
        self.forward_offset_pid.reset()
        self.forward_v_pid.reset()
        self.pitch_pid.reset()
        self.pitch_rate_pid.reset()
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_pos_z = 0.0
        self.prev_manual_active = False
