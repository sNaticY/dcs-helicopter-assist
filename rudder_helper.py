import math
import time
from pid_calculator_new import PIDCalculatorNew
from trim_module import TrimModule


class RudderHelper:
    def __init__(self):
        # 参数
        self.adaptive_factor = 0.03
        self.outer_skip = 10
        self.outer_count = 0
        self.dt = 0.02

        # 状态
        self.target_yaw = None
        self.yaw_pid = PIDCalculatorNew(Kp_base=1, Ki=0.12, Kd=0, max_auth=1)
        self.yaw_rate_pid = PIDCalculatorNew(Kp_base=2.0, Ki=0.16, Kd=0.48, max_auth=0.7)
        self.yaw_rate_pid_trim = TrimModule(
            use_kalman=False,
            zero_rate_threshold=0.10,  # 假设 yaw_rate 单位是 rad/s
            zero_stable_seconds=0.5,
            zero_fast_gain=0.22,
            apply_lp_alpha=0.30,
            apply_slew_rate=0.6,
            trim_min=-0.5,
            trim_max=0.5,
        )
        self.freeze_until = 0.0

    # -------------------------------
    # 控制循环调用
    # -------------------------------
    def update(self, yaw, yaw_rate, blocked, rudder_manual=0.0):
        manual_active = abs(rudder_manual) >= 0.02
        now = time.monotonic()

        if blocked:
            self.target_yaw = None
            self.prev_manual_active = manual_active
            return None, None

        # 手动 -> 自动 切换瞬间，锁定当前航向，避免回弹
        if (not manual_active) and self.prev_manual_active:
            self.freeze_until = now + 0.60

        if now >= self.freeze_until and self.target_yaw is None:
            self.target_yaw = yaw
            # 重置内外环，清掉历史积分/导数，避免旧命令残留
            self.yaw_pid.reset()
            self.yaw_rate_pid.reset()
            # 强制外环本帧立刻更新一次
            self.outer_count = self.outer_skip

        # 手动时不维持目标，自动时维持/建立目标
        if manual_active:
            self.target_yaw = None
        else:
            if self.target_yaw is None:
                self.target_yaw = yaw

        if self.target_yaw is not None:
            yaw_error = self.target_yaw - yaw
            if yaw_error > math.pi:
                yaw_error -= 2 * math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2 * math.pi

            if abs(yaw_error) > 0.06:
                self.target_yaw = yaw
                yaw_error = 0.0

        if not manual_active and now >= self.freeze_until:
            yaw_cmd = self.yaw_pid.auto
            if self.target_yaw is not None:
                yaw_error = self.target_yaw - yaw
                if yaw_error > math.pi:
                    yaw_error -= 2 * math.pi
                elif yaw_error < -math.pi:
                    yaw_error += 2 * math.pi

                # 外环
                self.outer_count += 1
                if self.outer_count >= self.outer_skip:
                    self.outer_count = 0
                    self.yaw_pid.update(
                        error=yaw_error, rate=None, delta_time=self.dt * self.outer_skip
                    )
            
            else:
                yaw_cmd = 0.0

            # 内环
            self.yaw_rate_pid.update(
                error=(yaw_rate + yaw_cmd), rate=None, delta_time=self.dt
            )

        raw_inner = self.yaw_rate_pid.auto

        # Trim 学习与应用（只在非手动时学习；稳态时快速吸收）
        diag = self.yaw_rate_pid_trim.update(
            measured_rate=yaw_rate,
            control_preview=rudder_manual + raw_inner + self.yaw_rate_pid_trim.get_trim_applied(),
            manual_active=manual_active,
            dt=self.dt,
        )
        trim = self.yaw_rate_pid_trim.get_trim_applied()

        # 合成输出：手动优先
        if manual_active:
            out = rudder_manual + raw_inner + trim
        else:
            out = raw_inner + trim

        # 限幅
        out = max(-1.0, min(1.0, out))

        self.prev_manual_active = manual_active
        return out, trim

    def reset(self):
        self.target_yaw = None
        self.prev_manual_active = False
        self.yaw_pid.reset()
        self.yaw_rate_pid.reset()
        self.yaw_rate_pid_trim.reset()
