import config
from utils import EMA

class PIDCalculator:
    def __init__(self,
                 Kp_pre=0.75,
                 Kp_base=0.5,
                 Ki=0.1,
                 Kd=0.08,
                 adaptive_factor=0.003,
                 max_auth=0.35,
                 integral_max=5.0,
                 integral_min=-5.0,
                 learning_rate=0.005,
                 learning_threshold=0.15):
        # 参数
        self.Kp_pre = Kp_pre
        self.Kp_base = Kp_base
        self.Ki = Ki
        self.Kd = Kd
        self.adaptive_factor = adaptive_factor
        self.max_auth = max_auth
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.learning_rate = learning_rate
        self.learning_threshold = learning_threshold
        self.dt = 0.02

        # 状态
        self.auto = 0.0
        self.balanced = 0.0
        self.error_integral = 0.0
        self.prev_error = 0.0
        self.rate = 0.0

        self.ema_rate = EMA(config.EMA_ALPHA)

    def update(self, error, rate, preError=0.0, manual=0.0):
        if abs(manual) < 0.06:
        # 自适应比例增益
            Kp = self.Kp_base + self.adaptive_factor * abs(error)

            # 外环
            PreErrorCmd = self.Kp_pre * preError
            if abs(PreErrorCmd) > 0.01:
                self.error_integral = 0.0

            # 积分项
            self.error_integral += error * self.dt
            self.error_integral = max(min(self.error_integral, self.integral_max), self.integral_min)

            # 误差微分
            if rate == None:
                self.rate = self.ema_rate.update((error - self.prev_error) / self.dt)
                self.prev_error = error
            else:
                self.rate = self.ema_rate.update(rate)
                self.prev_error = error

            # PID 控制
            self.auto = PreErrorCmd + Kp * error + self.Ki * self.error_integral + self.Kd * self.rate
            self.auto = max(min(self.auto, self.max_auth), -self.max_auth)

            # 平衡点遗忘
            self.balanced *= (1 - 0.3 * self.learning_rate)

            # 平衡点学习
            self.balanced += 0.05 * self.learning_rate * self.auto
            if abs(error) < self.learning_threshold and abs(self.rate) < self.learning_threshold:
                self.balanced += self.learning_rate * self.auto
            self.balanced = max(min(self.balanced, self.max_auth), -self.max_auth)
        else:
            self.balanced += 0.05 * self.learning_rate * (manual - self.balanced)
            if abs(error) < self.learning_threshold:
                self.balanced += self.learning_rate * (manual - self.balanced)
            self.balanced = max(min(self.balanced, self.max_auth), -self.max_auth)
            self.error_integral = 0.0
            self.prev_error = error

        

        return self.auto, self.balanced


