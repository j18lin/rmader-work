#!/usr/bin/env python3
"""Closed-loop check of the corridor backend with a map built from range scans.

Nothing tells the planner where the static obstacles are: it only receives the
scans of a simulated range sensor on the rover (sensor_msgs/PointCloud2 on
/<name>/point_cloud, limited range, no sight through obstacles) and has to
build its own map of free, occupied and unknown space.

Scenarios (launch tests/map.launch):
  crates   A rover shuttles across an arena with three crates in the way. Its
           goals are further away than the sensor reaches.
  ambush   The rover drives along a wall. An obstacle waits behind the far end
           of the wall, out of sight, and comes round it across the rover's
           way as the rover approaches. The obstacle is observed (on
           /obstacle_observations) only while the rover has a line of sight to
           it. With --no-observations it is not reported at all and only shows
           up in the scans, for the planner's own tracker.      [tracker:=true]

Pass --expect-collision for a control run with the map switched off
(map:=false), which has to hit a crate for the scenario to mean anything.
"""
import argparse
import json
import math
import os
import random
import sys
import time

import rospy
from rmader_msgs.msg import ObstacleObservation
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from observed_obstacles_test import OBSERVATION_NOISE, Robot  # noqa: E402
from sim_models import box_gap, distance  # noqa: E402

SCAN_HZ = 10.0
SCAN_RAYS = 360
OBSERVATION_HZ = 20.0


class Crate:
    """An axis-aligned box that stands still."""

    def __init__(self, center, size):
        self.position = list(center)
        self.size = tuple(size)


class Lurker:
    """Stands still until the rover has come past `trigger_x`, then drives through a point and on."""

    def __init__(self, start, through, speed, trigger_x):
        self.position = list(start)
        self.size = (0.6, 0.6, 0.6)
        self.through = through
        self.speed = speed
        self.trigger_x = trigger_x
        self.velocity = [0.0, 0.0, 0.0]
        self.started_at = None
        self.travelled = 0.0
        self.length = 3.0 * distance(start, through)  # it does not park beside the rover's way

    def step(self, dt, elapsed, rover):
        if self.started_at is None:
            if rover[0] < self.trigger_x:
                return
            self.started_at = elapsed
            direction = [t - p for t, p in zip(self.through, self.position)]
            norm = math.sqrt(sum(d * d for d in direction))
            self.velocity = [d * self.speed / norm for d in direction]
        if self.travelled < self.length:
            self.position = [p + v * dt for p, v in zip(self.position, self.velocity)]
            self.travelled += self.speed * dt
        else:
            self.velocity = [0.0, 0.0, 0.0]


def ray_box(origin, direction, center, size, limit):
    """Distance along a horizontal ray to an axis-aligned box, or `limit` if it is missed."""
    t_in, t_out = 0.0, limit
    for axis in range(2):
        low = center[axis] - 0.5 * size[axis]
        high = center[axis] + 0.5 * size[axis]
        if abs(direction[axis]) < 1e-12:
            if origin[axis] < low or origin[axis] > high:
                return limit
            continue
        a = (low - origin[axis]) / direction[axis]
        b = (high - origin[axis]) / direction[axis]
        if a > b:
            a, b = b, a
        t_in = max(t_in, a)
        t_out = min(t_out, b)
        if t_in > t_out:
            return limit
    return t_in


def scan(origin, bodies, sensor_range):
    """End points of a planar sweep. A ray that meets nothing ends beyond the sensor's range."""
    points = []
    beyond = sensor_range + 1.0
    for index in range(SCAN_RAYS):
        angle = 2.0 * math.pi * index / SCAN_RAYS
        direction = (math.cos(angle), math.sin(angle))
        reach = beyond
        for body in bodies:
            reach = min(reach, ray_box(origin, direction, body.position, body.size, beyond))
        if reach >= sensor_range:
            reach = beyond
        points.append((origin[0] + direction[0] * reach, origin[1] + direction[1] * reach, origin[2]))
    return points


def visible(origin, target, blockers, sensor_range):
    """True if the straight line from the sensor to the target is within range and crosses no blocker."""
    offset = [target[0] - origin[0], target[1] - origin[1]]
    length = math.hypot(*offset)
    if length > sensor_range or length < 1e-6:
        return length < 1e-6
    direction = (offset[0] / length, offset[1] / length)
    return all(ray_box(origin, direction, blocker.position, blocker.size, length) >= length for blocker in blockers)


def build_scenario(name):
    if name == "crates":
        robot = Robot("SQ01s", [-5.0, 0.0, 0.0], [[5.0, 0.0, 0.0], [-5.0, 0.0, 0.0]], ground=True)
        crates = [Crate([-1.8, 0.3, 0.3], (1.2, 1.2, 0.6)), Crate([1.6, -0.6, 0.3], (1.0, 1.6, 0.6)),
                  Crate([0.2, 2.6, 0.3], (1.4, 1.0, 0.6))]
        return robot, crates, []
    if name == "ambush":
        robot = Robot("SQ01s", [-5.0, 0.0, 0.0], [[5.0, 0.0, 0.0], [-5.0, 0.0, 0.0]], ground=True)
        # A long wall beside the rover's way. The obstacle waits behind its far end and comes round it,
        # across the rover's way, when the rover is 3 m short of that end.
        wall = [Crate([-1.4, 1.5, 0.3], (3.6, 0.6, 0.6))]
        lurker = Lurker(start=[0.9, 1.9, 0.0], through=[0.9, 0.0, 0.0], speed=0.25, trigger_x=-2.6)
        return robot, wall, [lurker]
    raise ValueError(name)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", choices=("crates", "ambush"), default="crates")
    parser.add_argument("--duration", type=float, default=150.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-legs", type=int, default=1)
    parser.add_argument("--sensor-range", type=float, default=4.0)
    parser.add_argument("--no-observations", action="store_true",
                        help="the moving obstacle is only in the scans, never on /obstacle_observations")
    parser.add_argument("--report", default="test-results/map.json")
    parser.add_argument("--expect-collision", action="store_true",
                        help="control run with the map disabled: pass only if the rover hits a crate")
    args = parser.parse_args()

    rospy.init_node("map_test", anonymous=False)
    rng = random.Random(args.seed)
    robot, crates, movers = build_scenario(args.scenario)
    cloud_pub = rospy.Publisher("/" + robot.name + "/point_cloud", PointCloud2, queue_size=2)
    observation_pub = rospy.Publisher("/obstacle_observations", ObstacleObservation, queue_size=50)

    rate = rospy.Rate(50)
    deadline = time.monotonic() + 25.0
    while not rospy.is_shutdown():
        if robot.connected():
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("the planner did not subscribe to state and term_goal")
        rate.sleep()

    def publish_scan():
        header = Header(stamp=rospy.Time.now(), frame_id="world")
        cloud_pub.publish(point_cloud2.create_cloud_xyz32(header, scan(robot.position, crates + movers,
                                                                        args.sensor_range)))

    seen = {"count": 0, "first_at_s": None}

    def publish_observations(elapsed):
        stamp = rospy.Time.now()
        for index, mover in enumerate(movers):
            if not visible(robot.position, mover.position, crates, args.sensor_range):
                continue
            seen["count"] += 1
            if seen["first_at_s"] is None:
                seen["first_at_s"] = round(elapsed, 2)
            message = ObstacleObservation()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.id = 4000 + index
            noisy = [value + rng.uniform(-OBSERVATION_NOISE, OBSERVATION_NOISE) for value in mover.position]
            message.pos.x, message.pos.y, message.pos.z = noisy
            message.bbox.x, message.bbox.y, message.bbox.z = mover.size
            observation_pub.publish(message)

    # The planner gets its own state and a few scans before the first goal.
    for tick in range(100):
        robot.publish_state()
        if tick % 5 == 0:
            publish_scan()
        rate.sleep()

    start_time = time.monotonic()
    robot.start_leg()
    leg_start = start_time
    reached_since = None
    last_tick = start_time
    last_scan = 0.0
    last_observation = 0.0
    last_progress = start_time
    crate_gap = {"value": float("inf"), "at_s": 0.0, "positions": None}
    mover_gap = {"value": float("inf"), "at_s": 0.0, "positions": None, "robot_speed": 0.0}

    while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed >= args.duration:
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now

        for mover in movers:
            mover.step(dt, elapsed, robot.position)
        robot.step(dt)
        robot.publish_state()
        if now - last_scan >= 1.0 / SCAN_HZ:
            publish_scan()
            last_scan = now
        if now - last_observation >= 1.0 / OBSERVATION_HZ and not args.no_observations:
            publish_observations(elapsed)
            last_observation = now

        for crate in crates:
            gap = box_gap(robot.position[:2], robot.bbox[:2], crate.position[:2], crate.size[:2])
            if gap < crate_gap["value"]:
                crate_gap = {"value": gap, "at_s": elapsed, "positions": [list(robot.position), list(crate.position)]}
        for mover in movers:
            gap = box_gap(robot.position[:2], robot.bbox[:2], mover.position[:2], mover.size[:2])
            if gap < mover_gap["value"]:
                mover_gap = {"value": gap, "at_s": elapsed,
                             "positions": [list(robot.position), list(mover.position)],
                             "robot_speed": math.hypot(robot.velocity[0], robot.velocity[1])}

        if not robot.reached and not robot.aligning and distance(robot.position, robot.goal) < robot.reach_tolerance:
            robot.reached = True
            robot.legs.append(round(now - leg_start, 3))
        if robot.reached:
            if reached_since is None:
                reached_since = now
            elif now - reached_since >= 1.0:
                robot.goal_index = (robot.goal_index + 1) % len(robot.goals)
                robot.start_leg()
                leg_start = now
                reached_since = None

        if now - last_progress >= 10.0:
            print("elapsed %.1fs legs %d min crate gap %.3f m min mover gap %.3f m trajectories %d at %s"
                  % (elapsed, len(robot.legs), crate_gap["value"], mover_gap["value"], robot.trajectories,
                     [round(v, 2) for v in robot.position[:2]]), flush=True)
            last_progress = now
        rate.sleep()

    def rounded(value):
        return None if value == float("inf") else round(value, 4)

    hit_crate = crate_gap["value"] < 0.0
    hit_mover = mover_gap["value"] < 0.0
    gates = {"planned": robot.committed > 0}
    if args.expect_collision:
        gates["scenario_collides_without_map"] = hit_crate
    else:
        gates["no_collision_with_static"] = not hit_crate
        gates["no_collision_with_moving"] = not hit_mover
        gates["legs"] = len(robot.legs) >= args.min_legs

    report = {
        "scenario": args.scenario,
        "control_run_map_disabled": args.expect_collision,
        "seed": args.seed,
        "duration_s": round(time.monotonic() - start_time, 3),
        "sensor_range_m": args.sensor_range,
        "minimum_gap_to_static_m": rounded(crate_gap["value"]),
        "minimum_gap_to_static_at_s": round(crate_gap["at_s"], 3),
        "positions_at_minimum_static_gap_m": crate_gap["positions"],
        "minimum_gap_to_moving_m": rounded(mover_gap["value"]),
        "minimum_gap_to_moving_at_s": round(mover_gap["at_s"], 3),
        "rover_speed_at_minimum_moving_gap_mps": round(mover_gap["robot_speed"], 3),
        "positions_at_minimum_moving_gap_m": mover_gap["positions"],
        "moving_obstacle_reported_to_planner": not args.no_observations,
        "moving_obstacle_first_seen_at_s": seen["first_at_s"],
        "moving_obstacle_started_at_s": [None if m.started_at is None else round(m.started_at, 2) for m in movers],
        "legs_reached": len(robot.legs),
        "leg_completion_s": robot.legs,
        "committed_trajectory_count": robot.committed,
        "final_error_m": round(distance(robot.position, robot.goal), 4),
        "gates": gates,
        "passed": all(gates.values()),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w") as handle:
        json.dump(report, handle, indent=2, sort_keys=True)
    print(json.dumps(report, indent=2, sort_keys=True))
    sys.exit(0 if report["passed"] else 1)


if __name__ == "__main__":
    main()
