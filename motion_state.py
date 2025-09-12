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

    def update(self, Vx, Vy, Vz, Pitch, Roll, Yaw, Ax, Ay, Az, PitchRate, RollRate, YawRate, x, y, z):
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
               f" Pitch={math.degrees(self.pitch):+.1f} Roll={math.degrees(self.roll):+.1f} Yaw={math.degrees(self.yaw):+.1f} |" \
               f" PitchRate={math.degrees(self.pitch_rate):+.1f} RollRate={math.degrees(self.roll_rate):+.1f} YawRate={math.degrees(self.yaw_rate):+.1f} |" \
               f" X={self.x:+.1f} Y={self.y:+.1f} Z={self.z:+.1f} |"