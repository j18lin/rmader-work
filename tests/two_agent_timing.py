#!/usr/bin/env python3
"""Closed-loop, two-agent timing check for the checked-out ROS 1 R-MADER."""
import argparse
import json
import math
import os
import sys
import time

import rosgraph
import rospy
from geometry_msgs.msg import PoseStamped
from rmader_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State

NAMES = ("SQ01s", "SQ02s")
STARTS = ((-2.0, -0.5, 1.5), (2.0, 0.5, 1.5))
TARGETS = (STARTS[1], STARTS[0])


def distance(a, b):
    return math.sqrt(sum((x - y) ** 2 for x, y in zip(a, b)))


def percentile(values, fraction):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1)]


def vector_limit(values, limit):
    length = math.sqrt(sum(value * value for value in values))
    if length <= limit or length == 0:
        return values
    return [value * limit / length for value in values]


def trajectory_subscriptions():
    try:
        _, subscribers, _ = rosgraph.Master("/two_agent_timing").getSystemState()
        by_topic = dict(subscribers)
        return {
            NAMES[0]: "/SQ02s/rmader" in by_topic.get("/SQ01s/rmader/trajs", []),
            NAMES[1]: "/SQ01s/rmader" in by_topic.get("/SQ02s/rmader/trajs", []),
        }
    except Exception:
        return {name: False for name in NAMES}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=90.0)
    parser.add_argument("--report", default="/results/two_agent_timing.json")
    parser.add_argument("--leg-timeout", type=float, default=30.0)
    args = parser.parse_args()
    rospy.init_node("two_agent_timing", anonymous=False)

    agents = {}
    listeners = []
    for name, start in zip(NAMES, STARTS):
        agents[name] = {
            "position": list(start),
            "velocity": [0.0, 0.0, 0.0],
            "setpoint": None,
            "goal_times": [],
            "trajectory_count": 0,
            "committed_count": 0,
            "leg_completion_s": [],
            "state_pub": rospy.Publisher("/" + name + "/state", State, queue_size=10),
            "terminal_pub": rospy.Publisher(
                "/" + name + "/term_goal", PoseStamped, queue_size=1, latch=True
            ),
        }

        def on_goal(message, agent_name=name):
            agent = agents[agent_name]
            agent["setpoint"] = (
                [message.p.x, message.p.y, message.p.z],
                [message.v.x, message.v.y, message.v.z],
            )
            agent["goal_times"].append(time.monotonic())

        def on_trajectory(message, agent_name=name):
            agents[agent_name]["trajectory_count"] += 1
            if message.is_committed:
                agents[agent_name]["committed_count"] += 1

        listeners.append(rospy.Subscriber("/" + name + "/goal", Goal, on_goal, queue_size=20))
        listeners.append(rospy.Subscriber(
            "/" + name + "/rmader/trajs", DynTraj, on_trajectory, queue_size=20
        ))

    rate = rospy.Rate(50)
    connect_deadline = time.monotonic() + 25.0
    while not rospy.is_shutdown():
        ready = all(
            item["state_pub"].get_num_connections() > 0
            and item["terminal_pub"].get_num_connections() > 0
            for item in agents.values()
        )
        if ready:
            break
        if time.monotonic() >= connect_deadline:
            raise RuntimeError("planner nodes did not subscribe to state and term_goal")
        rate.sleep()

    def publish_states():
        stamp = rospy.Time.now()
        for name in NAMES:
            agent = agents[name]
            message = State()
            message.header.stamp = stamp
            message.header.frame_id = "world"
            message.state_stamp = stamp
            message.pos.x, message.pos.y, message.pos.z = agent["position"]
            message.vel.x, message.vel.y, message.vel.z = agent["velocity"]
            message.quat.w = 1.0
            agent["state_pub"].publish(message)

    for _ in range(100):
        publish_states()
        rate.sleep()

    start_time = time.monotonic()
    current_targets = list(TARGETS)
    returning = False
    leg_start = start_time
    recorded = set()
    timed_out = False

    def publish_goals(targets):
        for name, target in zip(NAMES, targets):
            message = PoseStamped()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = "world"
            message.pose.position.x, message.pose.position.y, message.pose.position.z = target
            message.pose.orientation.w = 1.0
            agents[name]["terminal_pub"].publish(message)

    publish_goals(current_targets)
    closest = distance(agents[NAMES[0]]["position"], agents[NAMES[1]]["position"])
    closest_at_s = 0.0
    closest_positions = [list(agents[name]["position"]) for name in NAMES]
    last_tick = time.monotonic()
    last_progress = last_tick
    stable_since = None
    while not rospy.is_shutdown():
        now = time.monotonic()
        if now - start_time >= args.duration:
            break
        if now - leg_start >= args.leg_timeout:
            timed_out = True
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now
        for name, target in zip(NAMES, current_targets):
            agent = agents[name]
            setpoint = agent["setpoint"]
            if setpoint is not None:
                position_target, velocity_target = setpoint
                acceleration = [
                    5.0 * (position_target[i] - agent["position"][i])
                    + 3.0 * (velocity_target[i] - agent["velocity"][i])
                    for i in range(3)
                ]
                acceleration = vector_limit(acceleration, 4.0)
                agent["velocity"] = vector_limit(
                    [agent["velocity"][i] + acceleration[i] * dt for i in range(3)],
                    2.0,
                )
                agent["position"] = [
                    agent["position"][i] + agent["velocity"][i] * dt for i in range(3)
                ]
            error = distance(agent["position"], target)
            if error < 0.25 and name not in recorded:
                agent["leg_completion_s"].append(round(now - leg_start, 3))
                recorded.add(name)
        publish_states()
        separation = distance(agents[NAMES[0]]["position"], agents[NAMES[1]]["position"])
        if separation < closest:
            closest = separation
            closest_at_s = now - start_time
            closest_positions = [list(agents[name]["position"]) for name in NAMES]
        if len(recorded) == len(NAMES):
            if stable_since is None:
                stable_since = now
            if now - stable_since >= 1.0:
                returning = not returning
                current_targets = list(STARTS if returning else TARGETS)
                leg_start = now
                recorded.clear()
                stable_since = None
                publish_goals(current_targets)
        else:
            stable_since = None
        if now - last_progress >= 10.0:
            errors = [
                round(distance(agents[name]["position"], target), 2)
                for name, target in zip(NAMES, current_targets)
            ]
            print(
                "elapsed %.1fs legs %s errors %s trajectories %s"
                % (
                    now - start_time,
                    [len(agents[name]["leg_completion_s"]) for name in NAMES],
                    errors,
                    [agents[name]["trajectory_count"] for name in NAMES],
                ),
                flush=True,
            )
            last_progress = now
        rate.sleep()

    subscriptions = trajectory_subscriptions()
    agent_results = {}
    for name, target in zip(NAMES, current_targets):
        agent = agents[name]
        times = agent["goal_times"]
        gaps = [second - first for first, second in zip(times, times[1:])]
        agent_results[name] = {
            "final_error_m": round(distance(agent["position"], target), 4),
            "legs_reached": len(agent["leg_completion_s"]),
            "leg_completion_s": agent["leg_completion_s"],
            "setpoint_count": len(times),
            "setpoint_gap_p95_s": percentile(gaps, 0.95),
            "setpoint_gap_max_s": max(gaps) if gaps else None,
            "trajectory_count": agent["trajectory_count"],
            "committed_trajectory_count": agent["committed_count"],
            "peer_subscribed": subscriptions[name],
        }

    gates = {
        "legs": not timed_out and all(
            item["legs_reached"] >= 2 for item in agent_results.values()
        ),
        "separation": closest >= 0.5,
        "setpoints": all(
            item["setpoint_count"] >= 20
            and item["setpoint_gap_p95_s"] is not None
            and item["setpoint_gap_p95_s"] <= 0.25
            for item in agent_results.values()
        ),
        "trajectory_exchange": all(
            item["trajectory_count"] > 0
            and item["committed_trajectory_count"] > 0
            and item["peer_subscribed"]
            for item in agent_results.values()
        ),
    }
    report = {
        "scenario": "upstream R-MADER ROS Noetic, two aerial agents, bounded closed-loop tracking",
        "solver": "NLOPT",
        "duration_s": round(time.monotonic() - start_time, 3),
        "minimum_separation_m": round(closest, 4),
        "minimum_separation_at_s": round(closest_at_s, 3),
        "positions_at_minimum_separation_m": closest_positions,
        "leg_timeout": timed_out,
        "agents": agent_results,
        "gates": gates,
        "passed": all(gates.values()),
    }
    os.makedirs(os.path.dirname(args.report), exist_ok=True)
    with open(args.report, "w") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("two_agent_timing: " + str(error), file=sys.stderr, flush=True)
        raise