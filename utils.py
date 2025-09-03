import math
import random

import numpy as np


def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def norm_to_vjoy(v):
    v = clamp(v, -1.0, 1.0)
    if v == -1.0 or v == 1.0:
        v = v * random.uniform(0.95, 1.0)
    return int((v + 1.0) * 0.5 * 32767)

def world_to_body_velocity(Vx, Vy, Vz, pitch, roll, yaw):
    """
    将世界坐标系速度 (东, 上, 北) 转换为机体坐标系 (前, 右, 下)
    pitch, roll, yaw 单位为弧度
    """
    cy = math.cos(-yaw)
    sy = math.sin(-yaw)
    R_yaw = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1]
    ])
    V_world = np.array([Vx, Vz, Vy])  # 东、北、上
    V_body = R_yaw.dot(V_world)
    V_forward = V_body[0]
    V_right   = V_body[1]
    V_up      = V_body[2]
    return V_forward, V_right, V_up

class EMA:
    def __init__(self, alpha, init=0.0):
        self.alpha = alpha
        self.y = init
        self.inited = False

    def update(self, x):
        if not self.inited:
            self.y = x
            self.inited = True
        else:
            self.y = self.alpha * x + (1 - self.alpha) * self.y
        return self.y