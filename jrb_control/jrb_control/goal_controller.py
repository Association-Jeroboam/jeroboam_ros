from __future__ import division, print_function
from math import pi, sqrt, cos, atan2, radians
from .speed_limiter import SpeedLimiter


class Pose2D:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.xVel = 0.0
        self.yVel = 0.0
        self.thetaVel = 0.0

    def __str__(self):
        return str(
            {
                "x": self.x,
                "y": self.y,
                "theta": self.theta,
                "xVel": self.xVel,
                "yVel": self.yVel,
                "thetaVel": self.thetaVel,
            }
        )


class GoalController:
    """Finds linear and angular velocities necessary to drive toward
    a goal pose.
    """

    def __init__(self):
        self.kP = 3
        self.kA = 8
        self.kB = -1.5
        self.max_linear_speed = 1.0
        self.min_linear_speed = 0
        self.max_angular_speed = 2 * pi
        self.min_angular_speed = 0
        self.max_linear_acceleration = 1e9
        self.max_angular_acceleration = 1e9
        self.max_linear_jerk = 1e9
        self.max_angular_jerk = 1e9
        self.linear_tolerance = 0.025  # 2.5cm
        self.angular_tolerance = 3 / 180 * pi  # 3 degrees
        self.forward_movement_only = False
        self.linear_speed_limiter = SpeedLimiter(
            self.max_linear_speed, self.max_linear_acceleration, self.max_linear_jerk
        )
        self.angular_speed_limiter = SpeedLimiter(
            self.max_angular_speed, self.max_angular_acceleration, self.max_angular_jerk
        )
        self.last_linear_speed_cmd = [0.0, 0.0]
        self.last_angular_speed_cmd = [0.0, 0.0]

    def set_constants(self, kP, kA, kB):
        self.kP = kP
        self.kA = kA
        self.kB = kB

    def set_max_linear_speed(self, speed):
        self.linear_speed_limiter.max_velocity = speed

    def set_min_linear_speed(self, speed):
        self.min_linear_speed = speed

    def set_max_angular_speed(self, speed):
        self.angular_speed_limiter.max_velocity = speed

    def set_min_angular_speed(self, speed):
        self.min_angular_speed = speed

    def set_max_linear_acceleration(self, accel):
        self.linear_speed_limiter.max_acceleration = accel

    def set_max_angular_acceleration(self, accel):
        self.angular_speed_limiter.max_acceleration = accel

    def set_max_linear_jerk(self, jerk):
        self.linear_speed_limiter.max_jerk = jerk

    def set_max_angular_jerk(self, jerk):
        self.angular_speed_limiter.max_jerk = jerk

    def set_linear_tolerance(self, tolerance):
        self.linear_tolerance = tolerance

    def set_angular_tolerance(self, tolerance):
        self.angular_tolerance = tolerance

    def set_forward_movement_only(self, forward_only):
        self.forward_movement_only = forward_only

    def get_goal_distance(self, cur, goal):
        if goal is None:
            return 0
        diffX = cur.x - goal.x
        diffY = cur.y - goal.y
        return sqrt(diffX * diffX + diffY * diffY)

    def at_goal(self, cur, goal, isRotation):
        if goal is None:
            return True

        dTh = abs(self.normalize_pi(cur.theta - goal.theta))
        if isRotation:
            return dTh < self.angular_tolerance

        d = self.get_goal_distance(cur, goal)
        return d < self.linear_tolerance and dTh < self.angular_tolerance

    def get_velocity(self, cur, goal, dT, isRotation):
        desired = Pose2D()

        goal_heading = atan2(goal.y - cur.y, goal.x - cur.x)
        a = -cur.theta + goal_heading

        # In Automomous Mobile Robots, they assume theta_G=0. So for
        # the error in heading, we have to adjust theta based on the
        # (possibly non-zero) goal theta.
        theta = self.normalize_pi(cur.theta - goal.theta)
        b = -theta - a

        d = self.get_goal_distance(cur, goal)

        if isRotation:
            desired.xVel = 0
            desired.thetaVel = self.kB * theta
        else:
            if self.forward_movement_only:
                direction = 1
                a = self.normalize_pi(a)
                b = self.normalize_pi(b)
            else:
                direction = self.sign(cos(a))
                a = self.normalize_half_pi(a)
                b = self.normalize_half_pi(b)

            if abs(d) < self.linear_tolerance:
                desired.xVel = 0
                desired.thetaVel = self.kB * theta
            else:
                desired.xVel = self.kP * d * direction
                desired.thetaVel = self.kA * a + self.kB * b

        # Limit speed, acceleration, jerk
        linear_ratio = self.linear_speed_limiter.limit(
            desired.xVel,
            self.last_linear_speed_cmd[0],
            self.last_linear_speed_cmd[1],
            dT,
        )
        desired.xVel *= linear_ratio
        desired.thetaVel *= linear_ratio

        angular_ratio = self.angular_speed_limiter.limit(
            desired.thetaVel,
            self.last_angular_speed_cmd[0],
            self.last_angular_speed_cmd[1],
            dT,
        )
        desired.xVel *= angular_ratio
        desired.thetaVel *= angular_ratio

        # Adjust velocities if too low, so robot does not stall.
        # if abs(desired.xVel) > 0 and abs(desired.xVel) < self.min_linear_speed:
        #     ratio = self.min_linear_speed / abs(desired.xVel)
        #     desired.xVel *= ratio
        #     desired.thetaVel *= ratio
        # elif desired.xVel == 0 and abs(desired.thetaVel) < self.min_angular_speed:
        #     ratio = self.min_angular_speed / abs(desired.thetaVel)
        #     desired.xVel *= ratio
        #     desired.thetaVel *= ratio

        self.last_linear_speed_cmd[1] = self.last_linear_speed_cmd[0]
        self.last_linear_speed_cmd[0] = desired.xVel

        self.last_angular_speed_cmd[1] = self.last_angular_speed_cmd[0]
        self.last_angular_speed_cmd[0] = desired.thetaVel

        return desired

    def normalize_half_pi(self, alpha):
        alpha = self.normalize_pi(alpha)
        if alpha > pi / 2:
            return alpha - pi
        elif alpha < -pi / 2:
            return alpha + pi
        else:
            return alpha

    def normalize_pi(self, alpha):
        while alpha > pi:
            alpha -= 2 * pi
        while alpha < -pi:
            alpha += 2 * pi
        return alpha

    def sign(self, x):
        if x >= 0:
            return 1
        else:
            return -1
