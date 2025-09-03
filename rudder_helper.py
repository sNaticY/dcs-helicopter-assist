import time
from utils import EMA
import config

class RudderHelper:
    def __init__(self,
                 Kp_base=0.5,
                 Ki=0.1,
                 Kd=0.08,
                 adaptive_factor=0.03,
                 dt=0.02,
                 yawRate_threshold=0.5,
                 rudder_min=-0.7,
                 rudder_max=0.7,
                 integral_max=5.0,
                 integral_min=-5.0,
                 manual_threshold=0.05,
                 learning_rate=0.001):
        # 参数
        self.Kp_base = Kp_base
        self.Ki = Ki
        self.Kd = Kd
        self.adaptive_factor = adaptive_factor
        self.dt = dt
        self.yawRate_threshold = yawRate_threshold
        self.rudder_min = rudder_min
        self.rudder_max = rudder_max
        self.integral_max = integral_max
        self.integral_min = integral_min
        self.manual_threshold = manual_threshold
        self.learning_rate = learning_rate

        # 状态
        self.rudder_auto = 0.0
        self.yaw_rate_integral = 0.0
        self.prev_yaw_rate = 0.0
        self.balanced_rudder = 0.0

        self.ema_yaw_rate = EMA(config.EMA_ALPHA)
        self.ema_yaw_rate_derivative = EMA(config.EMA_ALPHA)

    # -------------------------------
    # 控制循环调用
    # -------------------------------
    def update(self, yaw, yaw_rate, rudder_manual = 0):

        # yawRate 微分
        #ema_yaw_rate = self.ema_yaw_rate.update(yaw_rate)
        yaw_rate_derivative = self.ema_yaw_rate_derivative.update((yaw_rate - self.prev_yaw_rate) / self.dt)
        self.prev_yaw_rate = yaw_rate

        # 自适应比例增益
        Kp = self.Kp_base + self.adaptive_factor * abs(yaw_rate)

        # 积分项
        self.yaw_rate_integral += yaw_rate * self.dt
        self.yaw_rate_integral = max(min(self.yaw_rate_integral, self.integral_max), self.integral_min)

        # 自动舵 PID
        self.rudder_auto += 0.05 * (Kp * yaw_rate + self.Ki * self.yaw_rate_integral + self.Kd * yaw_rate_derivative)
        self.rudder_auto = max(min(self.rudder_auto, self.rudder_max), self.rudder_min)

        # 手控接管判断
        # manual_active = abs(rudder_manual - self.rudder_auto) > self.manual_threshold
        manual_active = False

        if manual_active:
            rudder_out = rudder_manual
            if abs(yaw_rate) < self.yawRate_threshold and abs(yaw_rate_derivative) < self.yawRate_threshold:
                self.balanced_rudder += self.learning_rate * (rudder_manual - self.balanced_rudder)
        else:
            rudder_out = self.rudder_auto + self.balanced_rudder
            if abs(yaw_rate) < self.yawRate_threshold and abs(yaw_rate_derivative) < self.yawRate_threshold:
                self.balanced_rudder += self.learning_rate * ((self.rudder_auto + self.balanced_rudder) - self.balanced_rudder)

        self.balanced_rudder = max(min(self.balanced_rudder, self.rudder_max / 2), self.rudder_min / 2)

        return rudder_out, self.balanced_rudder
