import math
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
        self.roll_pid = PIDCalculatorNew(Kp_base=1.9, Ki=0.0, Kd=0.001, integral_max=0.1, integral_leak=0.02, skip=4, max_auth=0.5)
        self.roll_rate_pid = PIDCalculatorNew(Kp_base=0.05, Ki=0.001, Kd=0.02, integral_max=20, integral_leak=0.001)

        self.forward_offset_pid = PIDCalculatorNew(Kp_base=0.01, Ki=0.0008, Kd=0.003, integral_max=0.01, skip=2, max_auth=2)
        self.forward_v_pid = PIDCalculatorNew(Kp_base=0.05, Ki=0.02, Kd=0.1, integral_max=0.17, skip=3, max_auth=0.25)
        self.pitch_pid = PIDCalculatorNew(Kp_base=1.2, Ki=0.02, Kd=0.03, integral_max=0.001, integral_leak=0.02, skip=4, max_auth=1)
        self.pitch_rate_pid = PIDCalculatorNew(Kp_base=0.18, Ki=0.03, Kd=0.04, integral_max=0.5, integral_leak=0.001, max_auth=0.5)
        
        self.ema_cyclic_x = EMA(EMA_ALPHA)
        self.ema_cyclic_y = EMA(EMA_ALPHA)

        self.pitch_rate_ki = 0.35

        self.prev_manual_active = False
        self.prev_manual_cyclic_y = 0.0
        self.prev_hovering_active = False


    def update(self, motion_state, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):
        manual_active = abs(manual_cyclic_x) >= 0.05 or abs(manual_cyclic_y) >= 0.05

        if not self.prev_hovering_active and hovering:
            self.pitch_rate_pid.reset()
            self.target_pitch = 0.0
        elif self.prev_hovering_active and not hovering:
            self.forward_v_pid.reset()
            self.forward_offset_pid.reset()
            self.right_v_pid.reset()
            self.right_offset_pid.reset()

        if not hovering:
            self.target_pitch = None
        
        # 手动 -> 自动 切换瞬间，避免回弹
        if self.prev_manual_active and not manual_active:
            self.pitch_pid.reset()
            self.pitch_rate_pid.manual_override(
                error=motion_state.pitch_rate,
                rate=None,
                delta_time=self.dt,
                manual_input=self.ema_cyclic_y.y,
                prev_error=motion_state.prev_pitch_rate,
                skip=self.pitch_rate_pid.skip
            )
            self.pitch_rate_pid.update_ki(self.pitch_rate_ki)

        forward_offset, right_offset, up_offset = motion_state.get_position_delta(self.last_pos_x, self.last_pos_y, self.last_pos_z)
        if abs(forward_offset) > 1:
            forward_offset = sign(forward_offset) * 1
        if abs(right_offset) > 1:
            right_offset = sign(right_offset) * 1

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
                    self.forward_v_pid.update(error=-motion_state.forward_v + math.asin(self.forward_offset_pid.auto), rate=None, delta_time=self.dt * self.forward_v_pid.skip * self.pitch_pid.skip)
                    self.forward_offset_pid.update_skip()
            if self.roll_pid.is_available() and self.pitch_pid.is_available():
                self.roll_pid.update(error=-motion_state.roll+self.right_v_pid.auto, rate=None, delta_time=self.dt * self.roll_pid.skip)
                self.right_v_pid.update_skip()
                if self.target_pitch is not None:
                    self.pitch_pid.update(error=motion_state.pitch+self.forward_v_pid.auto-self.target_pitch, rate=None, delta_time=self.dt * self.pitch_pid.skip)
                self.forward_v_pid.update_skip()
            error_roll_rate = -motion_state.roll_rate + self.roll_pid.auto
            sign_correction = sign(error_roll_rate)
            self.roll_rate_pid.update(error=sign_correction * math.sqrt(abs(error_roll_rate)), rate=None, delta_time=self.dt)
            error_pitch_rate = motion_state.pitch_rate + self.pitch_pid.auto
            sign_correction = sign(error_pitch_rate)
            self.pitch_rate_pid.update(error=sign_correction * math.sqrt(abs(error_pitch_rate)), rate=None, delta_time=self.dt)
            self.roll_pid.update_skip()
            self.pitch_pid.update_skip()

        
        x_result = self.roll_rate_pid.auto
        self.ema_cyclic_x.update(x_result)
        y_result = self.pitch_rate_pid.auto
        self.ema_cyclic_y.update(y_result)

        if manual_active:
            x_result = self.ema_cyclic_x.y + manual_cyclic_x
            y_result = self.ema_cyclic_y.y + manual_cyclic_y

        self.prev_manual_active = manual_active
        self.prev_hovering_active = hovering

        return x_result, y_result
        
    def reset(self):
        self.target_pitch = None
        self.right_offset_pid.reset()
        self.right_v_pid.reset()
        self.roll_pid.reset()
        self.roll_rate_pid.reset()
        self.forward_offset_pid.reset()
        self.forward_v_pid.reset()
        self.pitch_pid.reset()
        # self.pitch_rate_pid.reset()
        self.last_pos_x = 0.0
        self.last_pos_y = 0.0
        self.last_pos_z = 0.0
        self.prev_manual_active = False
