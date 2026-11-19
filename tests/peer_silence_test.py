#!/usr/bin/env python3
"""Closed-loop check of what two planners do when the link between them drops.

Two aerial robots swap places over and over, with the geometry and limits of
tests/two_agent_timing.py, so they meet head-on in the middle of every leg. Part-way through, SQ02s is made deaf and mute for a
while (its planner drops everything it receives from its teammate and sends
nothing), then the link comes back.

With peer_silence enabled both robots have to finish the plans they had
committed to, stop, stay apart, and carry on once the link is back. The control
run (launch with peer_silence:=false, pass --control) shows what upstream does:
each keeps flying, planning around the last trajectory it received from the
other.
"""
import argparse
import json
import math
import os
import sys
import time

import rospy
from std_msgs.msg import Int8

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from observed_obstacles_test import Robot  # noqa: E402
from sim_models import box_gap, distance  # noqa: E402

WEST, EAST = (-2.0, -0.5, 1.5), (2.0, 0.5, 1.5)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=80.0)
    parser.add_argument("--fault-start", type=float, default=22.0)
    parser.add_argument("--fault-duration", type=float, default=15.0)
    parser.add_argument("--fault", type=int, choices=(1, 2, 3), default=3,
                        help="what SQ02s loses: 1 = hearing, 2 = its voice, 3 = both (default)")
    parser.add_argument("--report", default="test-results/peer_silence.json")
    parser.add_argument("--control", action="store_true",
                        help="control run with peer_silence off: pass if the robots keep flying while the link is down")
    args = parser.parse_args()

    rospy.init_node("peer_silence_test", anonymous=False)
    robots = [Robot("SQ01s", WEST, [EAST, WEST]), Robot("SQ02s", EAST, [WEST, EAST])]
    fault_pub = rospy.Publisher("/SQ02s/rmader/sim_comm_fault", Int8, queue_size=1, latch=True)

    rate = rospy.Rate(50)
    deadline = time.monotonic() + 25.0
    while not rospy.is_shutdown():
        if all(robot.connected() for robot in robots) and fault_pub.get_num_connections() > 0:
            break
        if time.monotonic() >= deadline:
            raise RuntimeError("planners did not subscribe")
        rate.sleep()
    fault_pub.publish(Int8(data=0))
    for _ in range(100):
        for robot in robots:
            robot.publish_state()
        rate.sleep()

    start_time = time.monotonic()
    for robot in robots:
        robot.start_leg()
    leg_start = start_time
    all_reached_since = None
    last_tick = start_time
    last_progress = start_time
    fault_end = args.fault_start + args.fault_duration
    fault_state = 0
    gap = {"value": float("inf"), "at_s": 0.0, "positions": None}
    gap_by_phase = {"before": float("inf"), "during": float("inf"), "after": float("inf")}
    max_speed_late_in_fault = 0.0
    legs_before = legs_during = legs_after = 0

    while not rospy.is_shutdown():
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed >= args.duration:
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now

        wanted = args.fault if args.fault_start <= elapsed < fault_end else 0
        if wanted != fault_state:
            fault_state = wanted
            fault_pub.publish(Int8(data=wanted))
            print("elapsed %.1fs: link %s" % (elapsed, "cut" if wanted else "restored"), flush=True)

        for robot in robots:
            robot.step(dt)
            robot.publish_state()

        current = box_gap(robots[0].position, robots[0].bbox, robots[1].position, robots[1].bbox)
        if current < gap["value"]:
            gap = {"value": current, "at_s": elapsed, "positions": [list(r.position) for r in robots]}
        phase = "before" if elapsed < args.fault_start else ("during" if elapsed < fault_end else "after")
        gap_by_phase[phase] = min(gap_by_phase[phase], current)
        if fault_state:
            if elapsed > args.fault_start + 6.0:  # committed plans last a few seconds at most
                max_speed_late_in_fault = max(
                    max_speed_late_in_fault, max(math.sqrt(sum(v * v for v in r.velocity)) for r in robots))

        for robot in robots:
            if not robot.reached and distance(robot.position, robot.goal) < robot.reach_tolerance:
                robot.reached = True
                robot.legs.append(round(now - leg_start, 3))
        if all(robot.reached for robot in robots):
            if all_reached_since is None:
                all_reached_since = now
                if elapsed < args.fault_start:
                    legs_before += 1
                elif elapsed < fault_end:
                    legs_during += 1
                else:
                    legs_after += 1
            elif now - all_reached_since >= 1.0:
                for robot in robots:
                    robot.goal_index = (robot.goal_index + 1) % len(robot.goals)
                    robot.start_leg()
                leg_start = now
                all_reached_since = None

        if now - last_progress >= 10.0:
            print("elapsed %.1fs legs before/during/after %d/%d/%d min robot gap %.3f m"
                  % (elapsed, legs_before, legs_during, legs_after, gap["value"]), flush=True)
            last_progress = now
        rate.sleep()

    # What happens before the link is cut is plain R-MADER and is reported, not gated: on a slow machine it
    # sometimes lets two head-on robots touch (see the two-agent timing test). The gates are about the fault.
    gates = {"swapped_before_the_fault": legs_before >= 1}
    if args.control:
        # Whether two robots that cannot hear each other actually touch depends on small differences between
        # their paths (it happened in some runs and not in others). What is certain is that they keep going.
        gates["kept_flying_while_link_down"] = max_speed_late_in_fault > 0.5
    else:
        gates["apart_while_link_down"] = gap_by_phase["during"] >= 0.0
        gates["apart_after_link_restored"] = gap_by_phase["after"] >= 0.0
        if args.fault == 3:
            # With the link down one way only, the robot that still hears the other is free to keep going. It
            # then finds its goal occupied by the one that stopped (the two swap places in this test), which
            # can leave them in R-MADER's usual face-to-face deadlock, so resuming is not required either.
            gates["stopped_while_link_down"] = max_speed_late_in_fault < 0.1
            gates["resumed_after_link_restored"] = legs_after >= 1

    report = {
        "scenario": "peer_silence",
        "control_run_feature_disabled": args.control,
        "overlapped_while_link_down": gap_by_phase["during"] < 0.0,
        "duration_s": round(time.monotonic() - start_time, 3),
        "fault_window_s": [args.fault_start, fault_end],
        "fault": {1: "SQ02s deaf", 2: "SQ02s mute", 3: "SQ02s deaf and mute"}[args.fault],
        "minimum_robot_gap_m": round(gap["value"], 4),
        "minimum_robot_gap_at_s": round(gap["at_s"], 3),
        "minimum_robot_gap_before_during_after_m": [
            None if gap_by_phase[key] == float("inf") else round(gap_by_phase[key], 4)
            for key in ("before", "during", "after")],
        "positions_at_minimum_gap_m": gap["positions"],
        "max_speed_late_in_fault_mps": round(max_speed_late_in_fault, 3),
        "legs_before_during_after_fault": [legs_before, legs_during, legs_after],
        "robots": {robot.name: {"legs_reached": len(robot.legs), "leg_completion_s": robot.legs,
                                "trajectory_count": robot.trajectories,
                                "committed_trajectory_count": robot.committed}
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
        print("peer_silence_test: " + str(error), file=sys.stderr, flush=True)
        sys.exit(2)
