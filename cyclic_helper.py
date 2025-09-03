import math
from utils import world_to_body_velocity


class CyclicHelper:
    def __init__(self,
                 Kp_roll=0.18,
                 Kd_rollrate=0.08,
                 Kp_pitch=0.18,
                 Kd_pitchrate=0.08,
                 Kp_vx=0.04,
                 Kd_ax=0.02,
                 Kp_vy=0.03,
                 Kd_ay=0.01,
                 K_bias=0.05,
                 max_auth=0.45,
                 dt=0.04,
                 bias_learn_rate=0.0002):
        # 参数
        self.Kp_roll = Kp_roll
        self.Kd_rollrate = Kd_rollrate
        self.Kp_pitch = Kp_pitch
        self.Kd_pitchrate = Kd_pitchrate
        self.Kp_vx = Kp_vx
        self.Kd_ax = Kd_ax
        self.Kp_vy = Kp_vy
        self.Kd_ay = Kd_ay
        self.K_bias = K_bias
        self.max_auth = max_auth
        self.dt = dt
        self.bias_learn_rate = bias_learn_rate

        # 状态
        self.u_x_bias = 0.0
        self.u_y_bias = 0.0
        self.prev_V_forward = 0.0
        self.prev_V_right = 0.0

        # 最近一次 update 的中間量
        self.V_forward = 0.0
        self.V_right = 0.0
        self.a_forward = 0.0
        self.a_right = 0.0

    def update(self, Vx, Vy, Vz, Pitch, Roll, PitchRate, RollRate):
        # 坐标转换
        self.V_forward, self.V_right, _ = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, 0)
        self.a_forward = (self.V_forward - self.prev_V_forward) / self.dt
        self.a_right = (self.V_right - self.prev_V_right) / self.dt
        self.prev_V_forward = self.V_forward
        self.prev_V_right = self.V_right

        # 横滚通道
        u_x = -(self.Kp_roll * Roll + self.Kd_rollrate * RollRate) - 0.1 * (self.Kp_vx * self.V_right + self.Kd_ax * self.a_right)
        if abs(self.V_right) <= 3 and abs(self.a_right) <= 0.5:
            self.u_x_bias -= self.V_right * self.K_bias * self.bias_learn_rate
        else:
            self.u_x_bias = 0.0
        u_x = max(min(u_x + self.u_x_bias, self.max_auth), -self.max_auth)

        # 俯仰通道
        u_y = (self.Kp_pitch * (Pitch) - self.Kd_pitchrate * PitchRate)
        if abs(self.V_forward) <= 10:
            u_y -= 0.1 * (self.Kp_vy * self.V_forward + self.Kd_ay * self.a_forward)
        if abs(self.V_forward) <= 8 and abs(self.a_forward) <= 0.5:
            self.u_y_bias -= self.V_forward * self.K_bias * self.bias_learn_rate
        else:
            self.u_y_bias = 0.0
        u_y = max(min(u_y + self.u_y_bias, self.max_auth), -self.max_auth)

        return u_x, u_y

    def debug_print(self):
        return f"Vf={self.V_forward:+.2f} Vr={self.V_right:+.2f} | Af={self.a_forward:+.2f} Ar={self.a_right:+.2f} | X_Bias={self.u_x_bias:+.2f} Y_Bias={self.u_y_bias:+.2f}"
        