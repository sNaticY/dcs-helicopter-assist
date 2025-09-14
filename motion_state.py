import math
from utils import EMA, world_to_body_velocity
import config

class MotionState:

    def __init__(self, dt=0.02):
        self.dt = dt

        # 加速度滤波器
        self.ema_forward_acc = EMA(config.EMA_ALPHA)
        self.ema_right_acc = EMA(config.EMA_ALPHA)
        self.ema_up_acc = EMA(config.EMA_ALPHA)

        # 机体坐标系下的速度和加速度
        self.forward_v = 0.0
        self.right_v = 0.0
        self.up_v = 0.0

        self.forward_acc = 0.0
        self.right_acc = 0.0
        self.up_acc = 0.0

        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0

        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.yaw_rate = 0.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        # 上一帧数据
        self.prev_forward_v = 0.0
        self.prev_right_v = 0.0
        self.prev_up_v = 0.0

        self.prev_forward_acc = 0.0
        self.prev_right_acc = 0.0
        self.prev_up_acc = 0.0

        self.prev_pitch = 0.0
        self.prev_roll = 0.0
        self.prev_yaw = 0.0

        self.prev_pitch_rate = 0.0
        self.prev_roll_rate = 0.0
        self.prev_yaw_rate = 0.0

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

    def update(self, Vx, Vy, Vz, Pitch, Roll, Yaw, Ax, Ay, Az, PitchRate, RollRate, YawRate, x, y, z):

        # 保存上一帧数据
        self.prev_forward_v, self.prev_right_v, self.prev_up_v = self.forward_v, self.right_v, self.up_v
        self.prev_forward_acc, self.prev_right_acc, self.prev_up_acc = self.forward_acc, self.right_acc, self.up_acc
        self.prev_pitch, self.prev_roll, self.prev_yaw = self.pitch, self.roll, self.yaw
        self.prev_pitch_rate, self.prev_roll_rate, self.prev_yaw_rate = self.pitch_rate, self.roll_rate, self.yaw_rate
        self.prev_x, self.prev_y, self.prev_z = self.x, self.y, self.z

        
        # 计算当前速度和加速度
        self.forward_v, self.right_v, self.up_v = world_to_body_velocity(Vx, Vy, Vz, Pitch, Roll, Yaw)
        forward_acc, right_acc, up_acc = world_to_body_velocity(Ax, Ay, Az, Pitch, Roll, Yaw)
        self.forward_acc = self.ema_forward_acc.update(forward_acc)
        self.right_acc = self.ema_right_acc.update(right_acc)   
        self.up_acc = self.ema_up_acc.update(up_acc)
        self.pitch = Pitch
        self.roll = Roll
        self.yaw = Yaw
        self.pitch_rate = PitchRate
        self.roll_rate = RollRate
        self.yaw_rate = YawRate
        self.x = x
        self.y = y
        self.z = z

    def get_position_delta(self, x, y, z):
        dx, dy, dz = x - self.x, y - self.y, z - self.z
        return world_to_body_velocity(dx, dy, dz, self.pitch, self.roll, self.yaw)

    def debug_print(self):
        return f" Vf={self.forward_v:+.2f} Vr={self.right_v:+.2f} |" \
               f" Af={self.forward_acc:+.2f} Ar={self.right_acc:+.2f} |" \
               f" Pitch={self.pitch:+.2f} Roll={self.roll:+.2f} Yaw={self.yaw:+.2f} |" \
               f" PitchRate={self.pitch_rate:+.2f} RollRate={self.roll_rate:+.2f} YawRate={self.yaw_rate:+.2f} |" \
               f" X={self.x:+.1f} Y={self.y:+.1f} Z={self.z:+.1f} |"