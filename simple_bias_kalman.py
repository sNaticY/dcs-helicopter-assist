class SimpleBiasKalman:
    """
    非常简单的 2-state Kalman 用于慢 bias 估计。
    状态 x = [bias] (we model measured_rate = true_rate + bias + v)
    We keep it minimal: bias_{k+1} = bias_k + w  (random walk)
    z = measured_rate - predicted_rate_from_control  (approx)
    """
    def __init__(self, q_bias=1e-6, r_meas=1e-3, init_bias=0.0, init_p=1e-3):
        # 单变量 Kalman for bias
        self.q = q_bias   # process noise for bias (small)
        self.r = r_meas   # measurement noise
        self.x = init_bias
        self.P = init_p

    def predict(self):
        # bias is random walk: x = x
        self.P += self.q

    def update(self, z):
        # z: measurement residual that relates to bias (scalar)
        # Kalman gain
        K = self.P / (self.P + self.r)
        self.x = self.x + K * (z - self.x)
        self.P = (1 - K) * self.P
        return self.x

    def get(self):
        return self.x

    def get_uncertainty(self):
        return self.P