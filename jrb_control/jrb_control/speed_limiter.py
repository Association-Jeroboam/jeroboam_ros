from math import copysign


def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))


class SpeedLimiter:
    def __init__(
        self,
        max_velocity: float,
        max_acceleration: float = None,
        max_jerk: float = None,
    ):
        self.max_velocity = abs(max_velocity)
        self.max_acceleration = abs(max_acceleration or 0)
        self.max_jerk = abs(max_jerk or 0)

    def limit(self, v: float, v0: float, v1: float, dt: float):
        new_v = v

        new_v *= self.limit_jerk(new_v, v0, v1, dt)
        new_v *= self.limit_acceleration(new_v, v0, dt)
        new_v *= self.limit_velocity(new_v)

        return new_v / v if abs(v) > 0.001 else 1.0

    def limit_jerk(self, v: float, v0, v1, dt: float):
        if self.max_jerk == 0:
            return 1.0

        dt2 = 2.0 * dt**2
        da_max = self.max_jerk * dt2

        dv = v - v0
        dv0 = v0 - v1
        da = dv - dv0
        old_da = da

        # Accelerating and positive jerk
        if v0 * dv >= 0 and dv0 * da >= 0:
            if abs(da) > da_max:
                da = copysign(da_max, da)
                pass

        new_v = v0 + dv0 + da
        ratio = new_v / v if abs(v) > 0.001 else 1.0
        if ratio > 1.1:
            print("FRERE PUTAIN", ratio)
            print(new_v, v, old_da, da)

        return ratio

    def limit_acceleration(self, v: float, v0: float, dt: float):
        if self.max_acceleration == 0:
            return 1.0

        # if v * v0 < 0:
        #     v = 0.0

        dv_max = self.max_acceleration * dt
        dv = v - v0

        # Accelerating
        if v0 * dv >= 0:
            if abs(dv) > dv_max:
                dv = copysign(dv_max, dv)
        # else:
        #     # TODO: decelerating, maybe another value for dv_max
        #     if abs(dv) > dv_max * 10:
        #         dv = copysign(dv_max * 10, dv)

        new_v = v0 + dv

        return new_v / v if abs(v) > 0.001 else 1.0

    def limit_velocity(self, v):
        new_v = clamp(v, -self.max_velocity, self.max_velocity)

        return new_v / v if abs(v) > 0.001 else 1.0
