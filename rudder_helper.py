import math
import time
from utils import EMA
import config
from pid_calculator import PIDCalculator

class RudderHelper:
    def __init__(self):
        # 参数
        self.adaptive_factor = 0.03
        self.yawRate_threshold = 0.5
        self.rudder_max = 0.7

        # 状态
        self.rudder_auto = 0.0
        self.balanced_rudder = 0.0

        self.yaw_rate_pid = PIDCalculator(adaptive_factor=self.adaptive_factor, 
                                          max_auth=self.rudder_max, 
                                          learning_threshold=self.yawRate_threshold)
        
        self.target_yaw = None

    # -------------------------------
    # 控制循环调用
    # -------------------------------
    def update(self, yaw, yaw_rate, blocked, rudder_manual = 0.0):
        if abs(rudder_manual) < 0.1 and abs(yaw_rate) < 0.3 and not blocked:
            if self.target_yaw == None:
                self.target_yaw = yaw
        else:
            self.target_yaw = None

        pre_yaw_error =  self.target_yaw - yaw if self.target_yaw != None else 0.0
        if pre_yaw_error > math.pi:
            pre_yaw_error -= 2 * math.pi
        elif pre_yaw_error < -math.pi:
            pre_yaw_error += 2 * math.pi
        
        self.rudder_auto, self.balanced_rudder = self.yaw_rate_pid.update(
            error = yaw_rate,
            rate = None,
            preError = pre_yaw_error,
            manual = rudder_manual
        )

        if blocked:
            return None, None
        
        if abs(rudder_manual) >= 0.02:
            return rudder_manual + self.balanced_rudder, self.balanced_rudder
        
        return self.rudder_auto + self.balanced_rudder, self.balanced_rudder
