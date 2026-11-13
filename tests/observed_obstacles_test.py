#!/usr/bin/env python3
"""Closed-loop check of R-MADER among obstacles it can only observe.

The obstacles never broadcast a trajectory. The planner receives their
positions on /obstacle_observations (no velocities, bounded noise) and has to
stay clear using only the configured speed bound.

Scenarios:
  crossing  The agent shuttles between two goals while three obstacles wander
            erratically (random waypoints, random speeds up to the bound)
            through the corridor it has to fly along.
  hover     The agent is parked at its goal. An obstacle drives straight
            through that spot. With hover avoidance the agent has to get out
            of the way and come back.

Run the same scenario with dyn_obstacles disabled (pass --expect-collision)
to confirm that the scenario does produce collisions without the feature.
"""
import argparse
import json
import math
import os
import random
import sys
import time

import rospy
from geometry_msgs.msg import PoseStamped
from rmader_msgs.msg import DynTraj, ObstacleObservation
from snapstack_msgs.msg import Goal, State

NAME = "SQ01s"
AGENT_BBOX = 0.35
OBSTACLE_BBOX = 0.6
OBSTACLE_MAX_SPEED = 0.5      # must match dyn_obstacles/obst_max_vel in the launch file
OBSERVATION_NOISE = 0.03      # [m] bound on the position error; matches dyn_obstacles/obst_position_error
OBSERVATION_HZ = 20.0
HEIGHT = 1.5


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def vector_limit(values, limit):
    length = math.sqrt(sum(value * value for value in values))
    if length <= limit or length == 0:
        return values
    return [value * limit / length for value in values]


def box_gap(agent, obstacle):
    """Signed gap between the two axis-aligned boxes; negative means they overlap."""
    reach = 0.5 * (AGENT_BBOX + OBSTACLE_BBOX)
    return max(abs(a - o) - reach for a, o in zip(agent, obstacle))


class Wanderer:
    """Moves to random waypoints inside a box at random speeds up to the bound."""

    def __init__(self, rng, low, high):
        self.rng = rng
        self.low = low
        self.high = high
        self.position = self._sample()
        self._retarget()

    def _sample(self):
        return [self.rng.uniform(lo, hi) for lo, hi in zip(self.low, self.high)]

    def _retarget(self):
        self.target = self._sample()
        self.speed = self.rng.uniform(0.4, 1.0) * OBSTACLE_MAX_SPEED

    def step(self, dt, _now):
        offset = [t - p for t, p in zip(self.target, self.position)]
        remaining = math.sqrt(sum(value * value for value in offset))
        travel = self.speed * dt
        if remaining <= travel:
            self.position = list(self.target)
            self._retarget()
        else:
            self.position = [p + o * travel / remaining for p, o in zip(self.position, offset)]


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


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", choices=("crossing", "hover"), default="crossing")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--report", default="test-results/observed_obstacles.json")
    parser.add_argument("--expect-collision", action="store_true",
                        help="control run with the feature disabled: pass only if the scenario collides")
    args = parser.parse_args()

    rospy.init_node("observed_obstacles_test", anonymous=False)
    rng = random.Random(args.seed)

    if args.scenario == "crossing":
        start = (-4.0, 0.0, HEIGHT)
        goals = [(4.0, 0.0, HEIGHT), start]
        # Three obstacles wandering inside the corridor the straight line passes through.
        low, high = (-2.0, -0.7, HEIGHT), (2.0, 0.7, HEIGHT)
        obstacles = [Wanderer(rng, low, high) for _ in range(3)]
    else:
        start = (0.0, 0.0, HEIGHT)
        goals = [start]
        obstacles = [Charger((-5.0, 0.0, HEIGHT), start, start_after_s=6.0, speed=0.4)]

    agent = {"position": list(start), "velocity": [0.0, 0.0, 0.0], "setpoint": None,
             "trajectories": 0, "committed": 0}

    def on_goal(message):
        agent["setpoint"] = ([message.p.x, message.p.y, message.p.z], [message.v.x, message.v.y, message.v.z])

    def on_trajectory(message):
        agent["trajectories"] += 1
        if message.is_committed:
            agent["committed"] += 1

    state_pub = rospy.Publisher("/" + NAME + "/state", State, queue_size=10)
    terminal_pub = rospy.Publisher("/" + NAME + "/term_goal", PoseStamped, queue_size=1, latch=True)
    observation_pub = rospy.Publisher("/obstacle_observations", ObstacleObservation, queue_size=50)
    listeners = [
        rospy.Subscriber("/" + NAME + "/goal", Goal, on_goal, queue_size=20),
        rospy.Subscriber("/" + NAME + "/rmader/trajs", DynTraj, on_trajectory, queue_size=20),
    ]

    rate = rospy.Rate(50)
    deadline = time.monotonic() + 25.0
    while not rospy.is_shutdown():
        if state_pub.get_num_connections() > 0 and terminal_pub.get_num_connections() > 0:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("planner did not subscribe to state and term_goal")
        rate.sleep()

    def publish_state():
        stamp = rospy.Time.now()
        message = State()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.state_stamp = stamp
        message.pos.x, message.pos.y, message.pos.z = agent["position"]
        message.vel.x, message.vel.y, message.vel.z = agent["velocity"]
        message.quat.w = 1.0
        state_pub.publish(message)

    def publish_observations():
        stamp = rospy.Time.now()
        for index, obstacle in enumerate(obstacles):
            message = ObstacleObservation()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.id = 4000 + index
            noisy = [value + rng.uniform(-OBSERVATION_NOISE, OBSERVATION_NOISE) / math.sqrt(3.0)
                     for value in obstacle.position]
            message.pos.x, message.pos.y, message.pos.z = noisy
            message.has_vel = False
            message.bbox.x = message.bbox.y = message.bbox.z = OBSTACLE_BBOX
            observation_pub.publish(message)

    def publish_goal(target):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = target
        message.pose.orientation.w = 1.0
        terminal_pub.publish(message)

    # Let the planner see its own state and the obstacles before the first goal.
    for _ in range(100):
        publish_state()
        publish_observations()
        rate.sleep()

    start_time = time.monotonic()
    goal_index = 0
    publish_goal(goals[goal_index])
    legs = []
    leg_start = start_time
    reached_since = None
    last_tick = start_time
    last_observation = 0.0
    last_progress = start_time
    min_gap = float("inf")
    min_gap_at_s = 0.0
    min_gap_positions = None
    min_gap_agent_speed = 0.0
    collision_s = 0.0
    max_excursion = 0.0

    while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed >= args.duration:
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now

        for obstacle in obstacles:
            obstacle.step(dt, elapsed)

        setpoint = agent["setpoint"]
        if setpoint is not None:
            position_target, velocity_target = setpoint
            acceleration = [
                5.0 * (position_target[i] - agent["position"][i]) + 3.0 * (velocity_target[i] - agent["velocity"][i])
                for i in range(3)
            ]
            acceleration = vector_limit(acceleration, 4.0)
            agent["velocity"] = vector_limit(
                [agent["velocity"][i] + acceleration[i] * dt for i in range(3)], 2.0)
            agent["position"] = [agent["position"][i] + agent["velocity"][i] * dt for i in range(3)]

        publish_state()
        if now - last_observation >= 1.0 / OBSERVATION_HZ:
            publish_observations()
            last_observation = now

        for obstacle in obstacles:
            gap = box_gap(agent["position"], obstacle.position)
            if gap < min_gap:
                min_gap = gap
                min_gap_at_s = elapsed
                min_gap_positions = [list(agent["position"]), list(obstacle.position)]
                min_gap_agent_speed = math.sqrt(sum(value * value for value in agent["velocity"]))
            if gap < 0.0:
                collision_s += dt
        max_excursion = max(max_excursion, distance(agent["position"], goals[goal_index]))

        if args.scenario == "crossing":
            if reached_since is None and distance(agent["position"], goals[goal_index]) < 0.25:
                reached_since = now
                legs.append(round(now - leg_start, 3))
            if reached_since is not None and now - reached_since >= 1.0:
                goal_index = (goal_index + 1) % len(goals)
                publish_goal(goals[goal_index])
                leg_start = now
                reached_since = None

        if now - last_progress >= 10.0:
            print("elapsed %.1fs legs %d min box gap %.3f m trajectories %d"
                  % (elapsed, len(legs), min_gap, agent["trajectories"]), flush=True)
            last_progress = now
        rate.sleep()

    final_error = distance(agent["position"], goals[goal_index])
    collided = min_gap < 0.0
    gates = {"planned": agent["committed"] > 0 if args.scenario == "crossing" else True}
    if args.expect_collision:
        gates["scenario_collides_without_feature"] = collided
    else:
        gates["no_collision"] = not collided
        if args.scenario == "crossing":
            gates["legs"] = len(legs) >= 2
        else:
            gates["evaded"] = max_excursion >= 1.0
            gates["returned_to_goal"] = final_error < 0.3

    report = {
        "scenario": args.scenario,
        "control_run_feature_disabled": args.expect_collision,
        "seed": args.seed,
        "duration_s": round(time.monotonic() - start_time, 3),
        "obstacles": len(obstacles),
        "obstacle_max_speed_mps": OBSTACLE_MAX_SPEED,
        "minimum_box_gap_m": round(min_gap, 4),
        "minimum_box_gap_at_s": round(min_gap_at_s, 3),
        "positions_at_minimum_gap_m": min_gap_positions,
        "agent_speed_at_minimum_gap_mps": round(min_gap_agent_speed, 3),
        "time_in_collision_s": round(collision_s, 3),
        "legs_reached": len(legs),
        "leg_completion_s": legs,
        "max_distance_from_goal_m": round(max_excursion, 3),
        "final_error_m": round(final_error, 4),
        "trajectory_count": agent["trajectories"],
        "committed_trajectory_count": agent["committed"],
        "gates": gates,
        "passed": all(gates.values()),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    del listeners
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("observed_obstacles_test: " + str(error), file=sys.stderr, flush=True)
        sys.exit(2)
