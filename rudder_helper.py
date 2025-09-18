import math
import time
from config import EMA_ALPHA
from config import EMA_ALPHA
from pid_calculator_new import PIDCalculatorNew
from utils import EMA, sign

class RudderHelper:
    def __init__(self):
        # 参数
        self.adaptive_factor = 0.03
        self.dt = 0.02
        self.yaw_rate_ki = 1.6

        # 状态
        self.target_yaw = None
        self.target_yaw_rate = 0.0
        self.yaw_pid = PIDCalculatorNew(Kp_base=1, Ki=0.04, Kd=0, max_auth=0.5, integral_max=0.05, skip=5)
        self.yaw_rate_pid = PIDCalculatorNew(Kp_base=1.4, Ki=self.yaw_rate_ki, Kd=0.35, adaptive_factor=0.06, max_auth=0.99, integral_max=1/self.yaw_rate_ki)
        
        self.prev_manual_active = False
        self.prev_manual_rudder = 0.0
        self.ema_target_yaw_rate = EMA(EMA_ALPHA)

    # -------------------------------
    # 控制循环调用
    # -------------------------------
    def update(self, motion_state, rudder_manual=0.0):
        manual_active = abs(rudder_manual) >= 0.01

        # 手动 -> 自动 切换瞬间，避免回弹
        if not manual_active and self.prev_manual_active:
            self.yaw_pid.reset()
            # self.yaw_rate_pid.manual_override(
            #     error=motion_state.yaw_rate,
            #     rate=None,
            #     delta_time=self.dt,
            #     manual_input=rudder_manual + self.yaw_rate_pid.auto,
            #     prev_error=motion_state.prev_yaw_rate,
            #     skip=self.yaw_rate_pid.skip
            # )
            self.yaw_rate_pid.update_ki(self.yaw_rate_ki)

        if not manual_active and (abs(motion_state.yaw_rate) < 0.01) and self.target_yaw is None:
            self.target_yaw = motion_state.yaw
            # self.yaw_pid.manual_override(
            #     error=0.0, 
            #     rate=motion_state.yaw_rate,
            #     delta_time=self.dt * self.yaw_pid.skip,
            #     manual_input=0.0,
            #     prev_error=motion_state.yaw - (motion_state.yaw - motion_state.prev_yaw) * self.yaw_pid.skip,
            #     skip=self.yaw_pid.skip
            # )
            self.yaw_pid.reset()
            # self.yaw_rate_pid.update_ki(0)
            

        # 手动时不维持目标，自动时维持/建立目标
        if manual_active:
            self.target_yaw = None
            self.target_yaw_rate = self.ema_target_yaw_rate.update(rudder_manual * 1.0)
        else:
            self.target_yaw_rate = 0.0

        if self.target_yaw is not None:
            yaw_error = self.target_yaw - motion_state.yaw
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

        yaw_cmd = 0.0
        # 计算输出     
        if not manual_active:
            # 外环
            yaw_cmd = self.yaw_pid.auto
            if self.target_yaw is not None and self.yaw_pid.is_available():
                self.yaw_pid.update(error=yaw_error, rate=None, delta_time=self.dt)
                yaw_cmd = self.yaw_pid.auto
            elif self.target_yaw is None:
                yaw_cmd = 0.0

        # 内环
        correction = motion_state.yaw_rate + 0.0 + self.target_yaw_rate + yaw_cmd
        sign_correction = sign(correction)
        self.yaw_rate_pid.update(error=sign_correction * (abs(correction) ** 0.75), rate=None, delta_time=self.dt)
        self.yaw_pid.update_skip()

        # 合成输出：手动优先
        if manual_active:
            out = self.yaw_rate_pid.auto
            self.prev_manual_rudder = rudder_manual
        else:
            out = self.yaw_rate_pid.auto

        # 限幅
        out = max(-1.0, min(1.0, out))

        self.prev_manual_active = manual_active

        return out

    def reset(self):
        self.target_yaw = None
        self.prev_manual_active = False
        self.yaw_pid.reset()
        self.yaw_rate_pid.reset()
