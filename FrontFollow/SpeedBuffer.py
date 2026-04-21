import math
import numpy as np


class EMABuffer:
    """
    指数平滑 + 死区吸附。
    默认 ratio=0.8，snap_tol=1e-3。
    """
    def __init__(self):
        self.pos = 0.0
        self.ratio = 0.8
        self.snap_tol = 1e-3

    def update(self, target):
        diff = target - self.pos
        if abs(diff) <= self.snap_tol:
            self.pos = target
        else:
            self.pos += self.ratio * diff
        return self.pos

    def quick_settle(self, target):
        """强制立即吸附到目标，用于转弯→直行等紧急状态切换"""
        self.pos = target
        return self.pos


class SCurvePlanner:
    """
    S曲线（梯形速度规划），参数按通道类型内置。
    channel: 'speed' | 'omega' | 'radius'
    """
    _PRESETS = {
        'speed':  {'vmax': 0.20, 'amax': 0.50},
        'omega':  {'vmax': 0.50, 'amax': 2.50},  # B: 更激进，转弯→直行更快
        'radius': {'vmax': 1.00, 'amax': 4.00},  # B: 更激进
    }

    def __init__(self, channel='speed', dt=0.1):
        cfg = self._PRESETS.get(channel, self._PRESETS['speed'])
        self.vmax = cfg['vmax']
        self.amax = cfg['amax']
        self.dt = dt

        self.pos = 0.0
        self.vel = 0.0
        self.target = 0.0

    def update(self, target):
        self.target = target
        dist = target - self.pos

        if abs(dist) < 1e-4 and abs(self.vel) < 1e-3:
            self.pos = target
            self.vel = 0.0
            return self.pos

        dec_dist = (self.vel ** 2) / (2.0 * self.amax) if self.amax > 0 else 0.0
        desired_dir = 1.0 if dist > 0 else -1.0 if dist < 0 else 0.0
        moving_toward = (desired_dir > 0 and self.vel > 0) or (desired_dir < 0 and self.vel < 0)

        if moving_toward and abs(dist) > dec_dist + 1e-6:
            if abs(self.vel) < self.vmax:
                self.vel += desired_dir * self.amax * self.dt
                self.vel = max(min(self.vel, self.vmax), -self.vmax)
        else:
            if abs(self.vel) < 1e-6:
                self.vel += desired_dir * self.amax * self.dt
                self.vel = max(min(self.vel, self.vmax), -self.vmax)
            else:
                slow_dir = 1.0 if self.vel < 0 else -1.0
                self.vel += slow_dir * self.amax * self.dt
                if (self.vel > 0 and slow_dir < 0) or (self.vel < 0 and slow_dir > 0):
                    self.vel = 0.0

        next_pos = self.pos + self.vel * self.dt
        if (next_pos - target) * (self.pos - target) < 0:
            self.pos = target
            self.vel = 0.0
        else:
            self.pos = next_pos

        return self.pos

    def quick_settle(self, target):
        """强制立即吸附到目标，清空速度状态"""
        self.pos = target
        self.vel = 0.0
        return self.pos


class MinJerkPlanner:
    """
    Minimum Jerk 轨迹规划。
    过渡时间根据距离动态计算，支持按通道类型预设不同激进程度。
    channel: 'speed' | 'omega' | 'radius'
    """
    _PRESETS = {
        'speed':  {'T_min': 0.30, 'T_max': 0.80, 'scale': 1.5},  # 线速度：偏平滑
        'omega':  {'T_min': 0.15, 'T_max': 0.40, 'scale': 1.0},  # 角速度：更激进
        'radius': {'T_min': 0.15, 'T_max': 0.40, 'scale': 1.0},  # 半径：更激进
    }

    def __init__(self, channel='speed', dt=0.1):
        cfg = self._PRESETS.get(channel, self._PRESETS['speed'])
        self.T_min = cfg['T_min']
        self.T_max = cfg['T_max']
        self.scale = cfg['scale']
        self.dt = dt

        self.p = 0.0
        self.v = 0.0
        self.a = 0.0
        self.t = 0.0
        self.T = 0.5
        self.p_target = 0.0
        self.planning = False
        self.coeffs = np.zeros(6)

    def _plan(self, p0, v0, a0, pT, vT, aT, T):
        T2, T3, T4, T5 = T ** 2, T ** 3, T ** 4, T ** 5
        M = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 2, 0, 0, 0],
            [1, T, T2, T3, T4, T5],
            [0, 1, 2 * T, 3 * T2, 4 * T3, 5 * T4],
            [0, 0, 2, 6 * T, 12 * T2, 20 * T3]
        ], dtype=float)
        b = np.array([p0, v0, a0, pT, vT, aT], dtype=float)
        self.coeffs = np.linalg.solve(M, b)
        self.t = 0.0
        self.T = T
        self.planning = True

    def update(self, target):
        if not self.planning or abs(target - self.p_target) > 1e-3:
            self.p_target = target
            dist = abs(target - self.p)
            T = max(self.T_min, min(self.T_max, self.scale * math.sqrt(dist)))
            self._plan(self.p, self.v, self.a, target, 0.0, 0.0, T)

        if self.t >= self.T:
            self.p = self.p_target
            self.v = 0.0
            self.a = 0.0
            self.planning = False
            return self.p

        t = self.t
        c = self.coeffs
        self.p = (c[0] + c[1] * t + c[2] * t ** 2 +
                  c[3] * t ** 3 + c[4] * t ** 4 + c[5] * t ** 5)
        self.v = (c[1] + 2 * c[2] * t + 3 * c[3] * t ** 2 +
                  4 * c[4] * t ** 3 + 5 * c[5] * t ** 4)
        self.a = (2 * c[2] + 6 * c[3] * t +
                  12 * c[4] * t ** 2 + 20 * c[5] * t ** 3)

        self.t += self.dt
        return self.p

    def quick_settle(self, target):
        """强制立即吸附到目标，清空轨迹状态"""
        self.p = target
        self.v = 0.0
        self.a = 0.0
        self.planning = False
        return self.p


if __name__ == "__main__":
    import time
    print("=" * 60)
    print("SpeedBuffer 独立测试")
    print("=" * 60)
    # 测试 quick_settle
    sc = SCurvePlanner(channel='omega')
    for t in [0.3, 0.3, 0.3, 0.0, 0.0]:
        print(f"target={t:.1f} -> normal={sc.update(t):.4f}")
    print("--- quick_settle(0) ---")
    print(f"snap to 0: {sc.quick_settle(0.0):.4f}")
    print(f"next normal update to 0: {sc.update(0.0):.4f}")