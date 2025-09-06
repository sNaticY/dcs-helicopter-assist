from pid_calculator import PIDCalculator
from utils import world_to_body_velocity


class CyclicHelper:
    def __init__(self):
        # 参数
        self.dt = 0.02
        self.max_auth = 0.35
        self.learning_rate = 0.001

        # 状态
        self.prev_forward = 0.0
        self.prev_right = 0.0
        self.balanced_cyclic_x = 0.0
        self.balanced_cyclic_y = 0.0

        # 最近一次 update 的中間量
        self.forward = 0.0
        self.right = 0.0

        self.cyclic_x_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)
        self.cyclic_y_pid = PIDCalculator(max_auth=self.max_auth, learning_rate=self.learning_rate)

    def update(self, Vx, Vy, Vz, Pitch, Roll, Yaw, PitchRate, RollRate, blocked = False, manual_cyclic_x=0.0, manual_cyclic_y=0.0, hovering=False):
        
        # 计算当前速度和加速度
        self.forward, self.right, _ = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, Yaw)
        
        pre_error_x = 0.0
        pre_error_y = 0.0

        if hovering:
            pre_error_x = min(max(-0.05*(self.right) - 0.045, -0.2), 0.2)
            pre_error_y = min(max(-0.05*(self.forward) - 0.085, -0.2), 0.2)

        self.cyclic_x_pid.update(-Roll + 1 * pre_error_x, -RollRate, manual=manual_cyclic_x)
        self.cyclic_y_pid.update(Pitch + 1 * pre_error_y, PitchRate, manual=manual_cyclic_y)

        if blocked:
            return None, None
        
        x_result = self.cyclic_x_pid.auto + self.cyclic_x_pid.balanced
        y_result = self.cyclic_y_pid.auto + self.cyclic_y_pid.balanced

        x_balanced = self.cyclic_x_pid.balanced
        y_balanced = self.cyclic_y_pid.balanced
        if abs(manual_cyclic_x) > 0.02:
            x_result = manual_cyclic_x + x_balanced
        if abs(manual_cyclic_y) > 0.02:
            y_result = manual_cyclic_y + y_balanced

        return x_result, y_result

    def debug_print(self):
        return f"Vf={self.forward:+.2f} Vr={self.right:+.2f} | balanced_cyclic_x={self.balanced_cyclic_x:+.2f} balanced_cyclic_y={self.balanced_cyclic_y:+.2f}"
    

