import math

class RudderHelperOld:
    def __init__(self, 
                 Kp_rudder=1.80,
                 Kd_rudder=0.15,
                 K_bias=0.05,
                 max_auth=0.45,
                 dt=0.04):
        self.Kp_rudder = Kp_rudder
        self.Kd_rudder = Kd_rudder
        self.K_bias = K_bias
        self.max_auth = max_auth
        self.dt = dt

        self.prev_yawrate = 0.0
        self.yaw_acc = 0.0
        self.rudder_bias = 0.0
        self.target_yaw = None

    def update(self, Yaw, YawRate):
        # RUDDER 控制
        self.yaw_acc = (YawRate - self.prev_yawrate) / self.dt
        self.prev_yawrate = YawRate

        self.rudder_bias += YawRate * self.K_bias * 0.1

        # --- RUDDER: 让 YawRate -> 0 （不锁航向）
        rudder_pid = (self.Kp_rudder * YawRate + self.Kd_rudder * self.yaw_acc)
        rudder = rudder_pid + self.rudder_bias
        rudder = max(min(rudder, self.max_auth), -self.max_auth)
        

        if abs(YawRate) <= 0.1:
            if self.target_yaw is None or abs((Yaw - self.target_yaw)) > 5.0:
                self.target_yaw = Yaw  # 锁定当前航向
            yaw_error = (Yaw - self.target_yaw)
            # 处理 -π ~ +π 跨界
            if yaw_error > math.pi:
                yaw_error -= 2*math.pi
            elif yaw_error < -math.pi:
                yaw_error += 2*math.pi
            rudder -= max(min(self.Kp_rudder * yaw_error * 0.5, self.max_auth), -self.max_auth)
        else:
            self.target_yaw = None

        return rudder, 0.0

    def reset(self):
        self.prev_yawrate = 0.0
        self.yaw_acc = 0.0
        self.rudder_bias = 0.0
        self.target_yaw = None