"""Vehicle and obstacle models shared by the closed-loop test drivers (no ROS in here)."""
import math


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def vector_limit(values, limit):
    length = math.sqrt(sum(value * value for value in values))
    if length <= limit or length == 0:
        return values
    return [value * limit / length for value in values]


def wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def box_gap(center_a, bbox_a, center_b, bbox_b):
    """Signed gap between two axis-aligned boxes; negative means they overlap."""
    return max(abs(a - b) - 0.5 * (sa + sb) for a, b, sa, sb in zip(center_a, center_b, bbox_a, bbox_b))


class Wanderer:
    """Moves to random waypoints inside a box at random speeds up to the bound."""

    def __init__(self, rng, low, high, max_speed):
        self.rng = rng
        self.low = low
        self.high = high
        self.max_speed = max_speed
        self.velocity = [0.0, 0.0, 0.0]
        self.position = self._sample()
        self._retarget()

    def _sample(self):
        return [self.rng.uniform(lo, hi) for lo, hi in zip(self.low, self.high)]

    def _retarget(self):
        self.target = self._sample()
        self.speed = self.rng.uniform(0.4, 1.0) * self.max_speed

    def step(self, dt, _elapsed):
        offset = [t - p for t, p in zip(self.target, self.position)]
        remaining = math.sqrt(sum(value * value for value in offset))
        travel = self.speed * dt
        if remaining <= travel:
            self.position = list(self.target)
            self.velocity = [0.0, 0.0, 0.0]
            self._retarget()
        else:
            self.velocity = [o * self.speed / remaining for o in offset]
            self.position = [p + o * travel / remaining for p, o in zip(self.position, offset)]


class SmoothWanderer(Wanderer):
    """A Wanderer whose velocity can only change at a bounded rate on each axis, like a vehicle."""

    def __init__(self, rng, low, high, max_speed, max_acceleration):
        super().__init__(rng, low, high, max_speed)
        self.max_acceleration = max_acceleration
        self.velocity = [0.0, 0.0, 0.0]

    def step(self, dt, _elapsed):
        offset = [t - p for t, p in zip(self.target, self.position)]
        remaining = math.sqrt(sum(value * value for value in offset))
        if remaining < 0.3:
            self._retarget()
            return
        # Per axis, so that both the speed and the acceleration bound hold per axis, as the planner assumes.
        limit = self.max_acceleration * dt
        for axis in range(3):
            wanted = max(-self.max_speed, min(self.max_speed, offset[axis] * self.speed / remaining))
            self.velocity[axis] += max(-limit, min(limit, wanted - self.velocity[axis]))
            self.position[axis] += self.velocity[axis] * dt


class Charger:
    """Waits, then drives in a straight line through a point and beyond."""

    def __init__(self, start, through, start_after_s, speed):
        self.position = list(start)
        direction = [t - s for t, s in zip(through, start)]
        length = math.sqrt(sum(value * value for value in direction))
        self.velocity = [value * speed / length for value in direction]
        self.start_after_s = start_after_s
        self.stop_after_s = start_after_s + 2.0 * length / speed

    def step(self, dt, elapsed):
        if self.start_after_s <= elapsed < self.stop_after_s:
            self.position = [p + v * dt for p, v in zip(self.position, self.velocity)]


class Unicycle:
    """A rover that tracks position/velocity setpoints the way the simulator bridge does: bounded speed,
    acceleration and turn rate, no sideways motion, and turning on the spot when told to or before a leg."""

    MAX_SPEED = 0.6
    MAX_ACCELERATION = 1.0
    MAX_TURN_RATE = 1.0

    def __init__(self, position, yaw):
        self.position = list(position)
        self.yaw = yaw
        self.speed = 0.0
        self.velocity = [0.0, 0.0, 0.0]

    def turn_towards(self, target_yaw, dt):
        """Turns on the spot. Returns True once it faces target_yaw and stands still."""
        error = wrap(target_yaw - self.yaw)
        done = abs(error) < 0.15 and self.speed < 0.02
        self._apply(0.0, 0.0 if done else max(-self.MAX_TURN_RATE, min(self.MAX_TURN_RATE, 2.5 * error)), dt)
        return done

    def track(self, target, feedforward, setpoint_yaw, dt):
        desired = [feedforward[i] + 1.5 * (target[i] - self.position[i]) for i in range(2)]
        desired_speed = math.hypot(desired[0], desired[1])
        if desired_speed > 0.03:
            error = wrap(math.atan2(desired[1], desired[0]) - self.yaw)
            speed_command = min(self.MAX_SPEED, desired_speed * max(0.0, math.cos(error)))
        else:
            # Standing still: the setpoint yaw is the planner asking to be turned on the spot.
            error = wrap(setpoint_yaw - self.yaw)
            speed_command = 0.0
        self._apply(speed_command, max(-self.MAX_TURN_RATE, min(self.MAX_TURN_RATE, 2.5 * error)), dt)

    def stop(self, dt):
        self._apply(0.0, 0.0, dt)

    def _apply(self, speed_command, turn_command, dt):
        limit = self.MAX_ACCELERATION * dt
        self.speed += max(-limit, min(limit, speed_command - self.speed))
        self.yaw = wrap(self.yaw + turn_command * dt)
        self.velocity = [self.speed * math.cos(self.yaw), self.speed * math.sin(self.yaw), 0.0]
        self.position = [self.position[0] + self.velocity[0] * dt, self.position[1] + self.velocity[1] * dt,
                         self.position[2]]
