import math
import random
from numpy import sign
from utils import apply_curve


class InputProcessor:
    """
    统一处理手动输入：限速（离零快/远离慢/跨零先回零）+ 曲线整形 + 输出轻微扰动。
    公开属性：
      - manual_cyclic_x/y/rudder: 原始输入（外部写）
      - input_cyclic_x/y/rudder: 处理后（供控制使用）
    """

    def __init__(
        self,
        expo_cyclic: float = 0.5,
        expo_rudder: float = 0.5,
        rate_up: float = 2.0,
        rate_down: float = 2.0,
        dither_threshold: float = 0.001,
        dither_amplitude: float = 0.001,
    ):
        # 原始输入（外部写入）
        self.manual_cyclic_x = 0.0
        self.manual_cyclic_y = 0.0
        self.manual_rudder = 0.0

        # 平滑内部状态
        self._smoothed_cyclic_x = 0.0
        self._smoothed_cyclic_y = 0.0
        self._smoothed_rudder = 0.0

        # 输出（曲线整形后）
        self.input_cyclic_x = 0.0
        self.input_cyclic_y = 0.0
        self.input_rudder = 0.0

        # 处理参数
        self.expo_cyclic = expo_cyclic
        self.expo_rudder = expo_rudder
        self.max_rate_up = rate_up
        self.max_rate_down = rate_down

        # 轻微扰动配置与状态
        self.dither_threshold = dither_threshold
        self.dither_amplitude = dither_amplitude
        self._prev_output = {
            "cyclic_x": 0.0,
            "cyclic_y": 0.0,
            "rudder": 0.0,
        }

    def set_manual(self, cyclic_x: float, cyclic_y: float, rudder: float):
        self.manual_cyclic_x = float(cyclic_x)
        self.manual_cyclic_y = float(cyclic_y)
        self.manual_rudder = float(rudder)

    def update(self, dt: float):
        # 限速
        self._smoothed_cyclic_x = self._rate_limit(self.manual_cyclic_x, self._smoothed_cyclic_x, dt)
        self._smoothed_cyclic_y = self._rate_limit(self.manual_cyclic_y, self._smoothed_cyclic_y, dt)
        self._smoothed_rudder   = self._rate_limit(self.manual_rudder,   self._smoothed_rudder,   dt)
        # 曲线整形
        self.input_cyclic_x = apply_curve(self._smoothed_cyclic_x, expo=self.expo_cyclic)
        self.input_cyclic_y = apply_curve(self._smoothed_cyclic_y, expo=self.expo_cyclic)
        self.input_rudder   = apply_curve(self._smoothed_rudder,   expo=self.expo_rudder)

    def apply_output_dither(self, name: str, value: float) -> float:
        """
        对输出做轻微扰动以避免 vJoy 卡死；name in {"cyclic_x","cyclic_y","rudder"}。
        """
        prev = self._prev_output.get(name, 0.0)
        if abs(prev - value) < self.dither_threshold:
            value += random.uniform(-self.dither_amplitude, self.dither_amplitude)
        self._prev_output[name] = value
        return value

    def reset_dither(self):
        self._prev_output = {"cyclic_x": 0.0, "cyclic_y": 0.0, "rudder": 0.0}

    def _rate_limit(self, target: float, current: float, dt: float) -> float:
        # 跨零：优先快速回零
        if sign(target) != sign(current) and abs(current) > 1e-3:
            delta = -current
            max_delta = self.max_rate_down * dt
            if abs(delta) > abs(max_delta):
                delta = sign(delta) * abs(max_delta)
            return current + delta

        # 离零慢，回零快
        moving_towards_zero = abs(target) < abs(current)
        max_rate = self.max_rate_down if moving_towards_zero else self.max_rate_up
        max_delta = max_rate * dt

        delta = target - current
        if abs(delta) > abs(max_delta):
            delta = sign(delta) * abs(max_delta)
        return current + delta