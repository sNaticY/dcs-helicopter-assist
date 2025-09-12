import math
import time
from pid_calculator_new import PIDCalculatorNew
from utils import sign

class RudderHelper:
    def __init__(self):
        # 参数
        self.adaptive_factor = 0.03
        self.outer_skip = 5
        self.outer_count = 0
        self.dt = 0.02

        # 状态
        self.target_yaw = None
        self.yaw_pid = PIDCalculatorNew(Kp_base=0.6, Ki=0.1, Kd=0, max_auth=0.5, integral_max=0.5)
        self.yaw_rate_pid = PIDCalculatorNew(Kp_base=4.5, adaptive_factor=0.06, Ki=5, Kd=0.005, max_auth=0.7, integral_max=3.0)
        
        self.prev_manual_active = False
        self.prev_manual_rudder = 0.0
        self.manual_yaw_rate = 0.0

    # -------------------------------
    # 控制循环调用
    # -------------------------------
    def update(self, yaw, yaw_rate, blocked, rudder_manual=0.0):
        manual_active = abs(rudder_manual) >= 0.02

        if blocked:
            self.target_yaw = None
            self.prev_manual_active = manual_active
            self.prev_manual_rudder = rudder_manual
            return None, None

        # 手动 -> 自动 切换瞬间，锁定当前航向，避免回弹
        if (not manual_active) and self.prev_manual_active:
            self.manual_yaw_rate = yaw_rate
            self.yaw_rate_pid.manual_override_integral(
                error=yaw_rate,
                rate=None,
                delta_time=self.dt,
                manual_input=rudder_manual + self.yaw_rate_pid.auto,
            )

        if not manual_active and (abs(yaw_rate) < 0.05 or self.prev_manual_rudder * yaw_rate > 0) and self.target_yaw is None:
            self.target_yaw = yaw
            # 重置外环，清掉历史积分/导数，避免旧命令残留
            self.yaw_pid.reset()
            # self.yaw_rate_pid.reset()
            # 强制外环本帧立刻更新一次
            self.outer_count = self.outer_skip
            

        # 手动时不维持目标，自动时维持/建立目标
        if manual_active:
            self.target_yaw = None

        if self.target_yaw is not None:
            yaw_error = self.target_yaw - yaw
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi
                

        if not manual_active:
            yaw_cmd = self.yaw_pid.auto
            if self.target_yaw is not None:
                # 外环
                self.outer_count += 1
                if self.outer_count >= self.outer_skip:
                    self.outer_count = 0
                    self.yaw_pid.update(
                        error=yaw_error, rate=None, delta_time=self.dt * self.outer_skip
                    )
                yaw_cmd = self.yaw_pid.auto
            
            else:
                yaw_cmd = 0.0

            # 内环
            self.yaw_rate_pid.update(
                error=(yaw_rate + yaw_cmd), rate=None, delta_time=self.dt
            )

        raw_inner = self.yaw_rate_pid.auto

    
        # 合成输出：手动优先
        if manual_active:
            out = rudder_manual + raw_inner
            
        else:
            out = raw_inner

        # 限幅
        out = max(-1.0, min(1.0, out))

        self.prev_manual_active = manual_active
        if manual_active:
            self.prev_manual_rudder = rudder_manual
        return out

    def reset(self):
        self.target_yaw = None
        self.prev_manual_active = False
        self.yaw_pid.reset()
        self.yaw_rate_pid.reset()
