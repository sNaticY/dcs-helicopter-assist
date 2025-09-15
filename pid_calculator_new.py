import config
from utils import EMA


class PIDCalculatorNew:
    def __init__(
        self,
        Kp_base=0.5,
        Ki=0.1,
        Kd=0.08,
        adaptive_factor=0.003,
        max_auth=0.35,
        integral_max=5.0,
        skip=1,
        stable_threshold=0.02,
    ):
        # 参数
        self.Kp_base = Kp_base
        self.Ki = Ki
        self.Kd = Kd
        self.adaptive_factor = adaptive_factor
        self.max_auth = max_auth
        self.integral_max = integral_max
        self.skip = skip
        self.stable_threshold = stable_threshold

        # 状态
        self.auto = 0.0
        self.error_integral = 0.0
        self.prev_error = 0.0
        self.rate = 0.0
        self._skip = 0

        self.ema_rate = EMA(config.EMA_ALPHA)

    def update(self, error, rate, delta_time):

        # 自适应比例增益
        Kp = self.Kp_base + self.adaptive_factor * abs(error)

        # 误差微分
        if rate == None:
            self.rate = self.ema_rate.update((error - self.prev_error) / delta_time)
            self.prev_error = error
        else:
            self.rate = self.ema_rate.update(rate)
            self.prev_error = error

        # 积分泄漏
        #self.error_integral -= 0.01 * max(abs(self.rate), 0.1) * self.error_integral

        # 积分项
        self.error_integral += error * delta_time
        self.error_integral = max(min(self.error_integral, self.integral_max), -self.integral_max)

        # PID 控制
        self.auto = Kp * error + self.Ki * self.error_integral + self.Kd * self.rate
        self.auto = max(min(self.auto, self.max_auth), -self.max_auth)
        self._skip = 0

    def manual_override(self, error, rate, delta_time, manual_input, prev_error, skip):
        # 自适应比例增益
        Kp = self.Kp_base + self.adaptive_factor * abs(error)
        self.prev_error = prev_error
        self._skip = skip

        # 误差微分
        if rate == None:
            self.rate = self.ema_rate.update((error - self.prev_error) / delta_time)
            self.prev_error = error
        else:
            self.rate = self.ema_rate.update(rate)
            self.prev_error = error

        # 积分反馈
        self.error_integral = ((manual_input - Kp * error - self.Kd * self.rate) / self.Ki)
        print(f"Manual override integral: {self.error_integral:.3f}")

    def update_ki(self, new_ki):
        if new_ki == 0 or self.error_integral == 0:
            self.error_integral = 0.0
            self.Ki = new_ki
            return
        self.error_integral = self.Ki / new_ki * self.error_integral
        self.Ki = new_ki

    def update_skip(self):
        self._skip += 1

    def is_available(self):
        return self._skip >= self.skip
    
    def is_stable(self):
        return self.prev_error <= self.stable_threshold
        
    def reset(self):
        self.auto = 0.0
        self.balanced = 0.0
        self.error_integral = 0.0
        self.prev_error = 0.0
        self.rate = 0.0
        self.ema_rate = EMA(config.EMA_ALPHA)
