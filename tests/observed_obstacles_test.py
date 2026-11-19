#!/usr/bin/env python3
"""Closed-loop check of R-MADER among obstacles it can only observe.

The obstacles never broadcast a trajectory. The planners receive their
positions on /obstacle_observations (no velocities, bounded noise) and have to
stay clear using only the configured speed bound.

Scenarios (launch tests/observed_obstacles.launch with the arguments shown):
  crossing   One aerial agent shuttles through a corridor in which three
             obstacles wander erratically (random waypoints, random speeds up
             to the bound).
  crossing2  Two aerial agents swap ends through that same corridor, so they
             also have to deconflict with each other.      [agents:=2]
  hover      An aerial agent is parked at its goal and an obstacle drives
             straight through that spot.                    [hover_avoidance:=true]
  ground     One rover (ground mode, unicycle tracker) shuttles across an
             arena in which two slower obstacles wander.
                                        [ground:=true obst_max_vel:=0.25]
  ground2    Two rovers cross at right angles through that arena.
                               [ground:=true obst_max_vel:=0.25 agents:=2]

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

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_models import Charger, Unicycle, Wanderer, box_gap, distance, vector_limit  # noqa: E402

OBSERVATION_NOISE = 0.03      # [m] per-axis bound on the position error; matches dyn_obstacles/obst_position_error
OBSERVATION_HZ = 20.0
OBSTACLE_BBOX = (0.6, 0.6, 0.6)


class Robot:
    """A planner under test plus the vehicle model that tracks its setpoints."""

    def __init__(self, name, start, goals, ground=False, yaw=0.0):
        self.name = name
        self.position = list(start)
        self.velocity = [0.0, 0.0, 0.0]
        self.yaw = yaw
        self.unicycle = Unicycle(start, yaw) if ground else None
        self.goals = goals
        self.goal_index = 0
        self.ground = ground
        self.bbox = (0.7, 0.7, 0.4) if ground else (0.35, 0.35, 0.35)
        self.reach_tolerance = 0.5 if ground else 0.25
        self.setpoint = None
        self.setpoint_yaw = yaw
        self.aligning = False
        self.trajectories = 0
        self.committed = 0
        self.legs = []
        self.reached = False
        self.max_excursion = 0.0
        self.state_pub = rospy.Publisher("/" + name + "/state", State, queue_size=10)
        self.goal_pub = rospy.Publisher("/" + name + "/term_goal", PoseStamped, queue_size=1, latch=True)
        self.subscribers = [
            rospy.Subscriber("/" + name + "/goal", Goal, self.on_setpoint, queue_size=20),
            rospy.Subscriber("/" + name + "/rmader/trajs", DynTraj, self.on_trajectory, queue_size=20),
        ]

    @property
    def goal(self):
        return self.goals[self.goal_index]

    def connected(self):
        return self.state_pub.get_num_connections() > 0 and self.goal_pub.get_num_connections() > 0

    def on_setpoint(self, message):
        self.setpoint = ([message.p.x, message.p.y, message.p.z], [message.v.x, message.v.y, message.v.z])
        self.setpoint_yaw = message.psi

    def on_trajectory(self, message):
        self.trajectories += 1
        if message.is_committed:
            self.committed += 1

    def publish_state(self):
        stamp = rospy.Time.now()
        message = State()
        message.header.stamp = stamp
        message.header.frame_id = "world"
        message.state_stamp = stamp
        message.pos.x, message.pos.y, message.pos.z = self.position
        message.vel.x, message.vel.y, message.vel.z = self.velocity
        message.quat.z = math.sin(self.yaw / 2.0)
        message.quat.w = math.cos(self.yaw / 2.0)
        self.state_pub.publish(message)

    def publish_goal(self):
        message = PoseStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "world"
        message.pose.position.x, message.pose.position.y, message.pose.position.z = self.goal
        message.pose.orientation.w = 1.0
        self.goal_pub.publish(message)

    def start_leg(self):
        """A rover first turns towards its goal, as the simulator bridge does; the goal follows once it faces it."""
        self.reached = False
        if self.ground:
            self.aligning = True
        else:
            self.publish_goal()

    def step(self, dt):
        if self.ground:
            self._step_unicycle(dt)
        else:
            self._step_point_mass(dt)
        self.max_excursion = max(self.max_excursion, distance(self.position, self.goal))

    def _step_point_mass(self, dt):
        if self.setpoint is None:
            return
        target, feedforward = self.setpoint
        acceleration = [5.0 * (target[i] - self.position[i]) + 3.0 * (feedforward[i] - self.velocity[i])
                        for i in range(3)]
        acceleration = vector_limit(acceleration, 4.0)
        self.velocity = vector_limit([self.velocity[i] + acceleration[i] * dt for i in range(3)], 2.0)
        self.position = [self.position[i] + self.velocity[i] * dt for i in range(3)]

    def _step_unicycle(self, dt):
        model = self.unicycle
        if self.aligning:
            heading = math.atan2(self.goal[1] - model.position[1], self.goal[0] - model.position[0])
            if model.turn_towards(heading, dt):
                self.aligning = False
                self.setpoint = None
                self.publish_goal()
        elif self.setpoint is not None:
            model.track(self.setpoint[0], self.setpoint[1], self.setpoint_yaw, dt)
        else:
            model.stop(dt)
        self.position, self.velocity, self.yaw = model.position, model.velocity, model.yaw


def build_scenario(name, rng):
    """Returns (robots, obstacles, obstacle speed bound)."""
    if name in ("crossing", "crossing2"):
        height = 1.5
        west, east = (-4.0, 0.0, height), (4.0, 0.0, height)
        robots = [Robot("SQ01s", west, [east, west])]
        if name == "crossing2":
            robots.append(Robot("SQ02s", east, [west, east]))
        speed = 0.5
        obstacles = [Wanderer(rng, (-2.0, -0.7, height), (2.0, 0.7, height), speed) for _ in range(3)]
    elif name == "hover":
        height = 1.5
        home = (0.0, 0.0, height)
        robots = [Robot("SQ01s", home, [home])]
        speed = 0.5
        obstacles = [Charger((-5.0, 0.0, height), home, start_after_s=6.0, speed=0.4)]
    else:
        west, east = (-5.0, 0.0, 0.0), (5.0, 0.0, 0.0)
        south, north = (0.0, -5.0, 0.0), (0.0, 5.0, 0.0)
        robots = [Robot("SQ01s", west, [east, west], ground=True, yaw=0.0)]
        if name == "ground2":
            robots.append(Robot("SQ02s", south, [north, south], ground=True, yaw=math.pi / 2.0))
        speed = 0.25
        obstacles = [Wanderer(rng, (-2.0, -2.0, 0.0), (2.0, 2.0, 0.0), speed) for _ in range(2)]
    return robots, obstacles, speed


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--scenario", choices=("crossing", "crossing2", "hover", "ground", "ground2"),
                        default="crossing")
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-legs", type=int, default=2, help="legs every robot has to complete")
    parser.add_argument("--no-obstacles", action="store_true", help="baseline: remove the obstacles")
    parser.add_argument("--report", default="test-results/observed_obstacles.json")
    parser.add_argument("--expect-collision", action="store_true",
                        help="control run with the feature disabled: pass only if the scenario collides")
    args = parser.parse_args()

    rospy.init_node("observed_obstacles_test", anonymous=False)
    rng = random.Random(args.seed)
    robots, obstacles, obstacle_speed = build_scenario(args.scenario, rng)
    if args.no_obstacles:
        obstacles = []
    observation_pub = rospy.Publisher("/obstacle_observations", ObstacleObservation, queue_size=50)

    rate = rospy.Rate(50)
    deadline = time.monotonic() + 25.0
    while not rospy.is_shutdown():
        if all(robot.connected() for robot in robots):
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("planners did not subscribe to state and term_goal")
        rate.sleep()

    def publish_observations():
        stamp = rospy.Time.now()
        for index, obstacle in enumerate(obstacles):
            message = ObstacleObservation()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.id = 4000 + index
            noisy = [value + rng.uniform(-OBSERVATION_NOISE, OBSERVATION_NOISE) for value in obstacle.position]
            message.pos.x, message.pos.y, message.pos.z = noisy
            message.has_vel = False
            message.bbox.x, message.bbox.y, message.bbox.z = OBSTACLE_BBOX
            observation_pub.publish(message)

    # Let the planners see their own state and the obstacles before the first goal.
    for _ in range(100):
        for robot in robots:
            robot.publish_state()
        publish_observations()
        rate.sleep()

    start_time = time.monotonic()
    for robot in robots:
        robot.start_leg()
    leg_start = start_time
    all_reached_since = None
    last_tick = start_time
    last_observation = 0.0
    last_progress = start_time
    obstacle_gap = {"value": float("inf"), "at_s": 0.0, "robot": None, "positions": None, "robot_speed": 0.0}
    robot_gap = {"value": float("inf"), "at_s": 0.0, "positions": None}
    collision_s = 0.0

    while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed >= args.duration:
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now

        for obstacle in obstacles:
            obstacle.step(dt, elapsed)
        for robot in robots:
            robot.step(dt)
            robot.publish_state()
        if now - last_observation >= 1.0 / OBSERVATION_HZ:
            publish_observations()
            last_observation = now

        colliding = False
        for robot in robots:
            for obstacle in obstacles:
                gap = box_gap(robot.position, robot.bbox, obstacle.position, OBSTACLE_BBOX)
                if gap < obstacle_gap["value"]:
                    obstacle_gap = {"value": gap, "at_s": elapsed, "robot": robot.name,
                                    "positions": [list(robot.position), list(obstacle.position)],
                                    "robot_speed": math.sqrt(sum(v * v for v in robot.velocity))}
                colliding = colliding or gap < 0.0
        if colliding:
            collision_s += dt
        for index, first in enumerate(robots):
            for second in robots[index + 1:]:
                gap = box_gap(first.position, first.bbox, second.position, second.bbox)
                if gap < robot_gap["value"]:
                    robot_gap = {"value": gap, "at_s": elapsed,
                                 "positions": [list(first.position), list(second.position)]}

        # Legs are synchronized: the next goals go out one second after every robot has arrived.
        if args.scenario != "hover":
            for robot in robots:
                if not robot.reached and not robot.aligning and \
                        distance(robot.position, robot.goal) < robot.reach_tolerance:
                    robot.reached = True
                    robot.legs.append(round(now - leg_start, 3))
            if all(robot.reached for robot in robots):
                if all_reached_since is None:
                    all_reached_since = now
                elif now - all_reached_since >= 1.0:
                    for robot in robots:
                        robot.goal_index = (robot.goal_index + 1) % len(robot.goals)
                        robot.start_leg()
                    leg_start = now
                    all_reached_since = None

        if now - last_progress >= 10.0:
            print("elapsed %.1fs legs %s min obstacle gap %.3f m min robot gap %.3f m trajectories %s"
                  % (elapsed, [len(robot.legs) for robot in robots], obstacle_gap["value"], robot_gap["value"],
                     [robot.trajectories for robot in robots]), flush=True)
            last_progress = now
        rate.sleep()

    collided = obstacle_gap["value"] < 0.0
    gates = {}
    if args.scenario != "hover":
        gates["planned"] = all(robot.committed > 0 for robot in robots)
    if len(robots) > 1:
        gates["robots_apart"] = robot_gap["value"] >= 0.0
    if args.expect_collision:
        gates["scenario_collides_without_feature"] = collided
    else:
        gates["no_collision"] = not collided
        if args.scenario == "hover":
            gates["evaded"] = robots[0].max_excursion >= 1.0
            gates["returned_to_goal"] = distance(robots[0].position, robots[0].goal) < 0.3
        else:
            gates["legs"] = all(len(robot.legs) >= args.min_legs for robot in robots)

    def rounded(value):
        return None if value == float("inf") else round(value, 4)

    report = {
        "scenario": args.scenario,
        "control_run_feature_disabled": args.expect_collision,
        "seed": args.seed,
        "duration_s": round(time.monotonic() - start_time, 3),
        "obstacles": len(obstacles),
        "obstacle_max_speed_mps": obstacle_speed,
        "minimum_box_gap_m": rounded(obstacle_gap["value"]),
        "minimum_box_gap_at_s": round(obstacle_gap["at_s"], 3),
        "minimum_box_gap_robot": obstacle_gap["robot"],
        "positions_at_minimum_gap_m": obstacle_gap["positions"],
        "agent_speed_at_minimum_gap_mps": round(obstacle_gap["robot_speed"], 3),
        "time_in_collision_s": round(collision_s, 3),
        "minimum_robot_gap_m": rounded(robot_gap["value"]),
        "minimum_robot_gap_at_s": round(robot_gap["at_s"], 3),
        "legs_reached": min(len(robot.legs) for robot in robots),
        "max_distance_from_goal_m": round(max(robot.max_excursion for robot in robots), 3),
        "final_error_m": round(max(distance(robot.position, robot.goal) for robot in robots), 4),
        "robots": {robot.name: {"legs_reached": len(robot.legs), "leg_completion_s": robot.legs,
                                "trajectory_count": robot.trajectories,
                                "committed_trajectory_count": robot.committed,
                                "final_error_m": round(distance(robot.position, robot.goal), 4)}
                   for robot in robots},
        "gates": gates,
        "passed": all(gates.values()),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.report)), exist_ok=True)
    with open(args.report, "w") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("observed_obstacles_test: " + str(error), file=sys.stderr, flush=True)
        sys.exit(2)
