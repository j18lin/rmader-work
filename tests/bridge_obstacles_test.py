#!/usr/bin/env python3
"""End-to-end check of the rover bridge with obstacles, without Gazebo.

This script plays the simulator's side of integration/ros1_bridge.py: it
connects to the bridge over TCP, sends rover states, synchronized goals and
`obstacle` rows, and drives two unicycle rover models from the setpoints that
come back. Everything on the ROS 1 side is the real thing: launch
integration/rover_sim.launch with dyn_obstacles:=true.

Scenario: JR81 crosses the arena west to east and RR83 south to north, as in
the simulator's crossing scenario. A crate sits on JR81's line (reported once
a second as static) and an obstacle wanders around the middle (reported at
20 Hz, position only).
"""
import argparse
import json
import math
import os
import random
import socket
import subprocess
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from sim_models import Unicycle, Wanderer, box_gap, distance  # noqa: E402

ROVER_BBOX = (0.7, 0.7, 0.4)
MOVING_BBOX = (0.6, 0.6, 0.4)
CRATE_BBOX = (0.8, 0.8, 0.4)
CRATE_POSITION = [-2.5, 0.0, 0.0]
OBSTACLE_SPEED = 0.25          # [m/s] must not exceed obst_max_vel in integration/rover_sim.launch
OBSERVATION_NOISE = 0.03       # [m] per axis
REACH_TOLERANCE = 0.5


class Rover:
    def __init__(self, name, start, goals, yaw):
        self.name = name
        self.model = Unicycle(start, yaw)
        self.goals = goals
        self.goal_index = 0
        self.setpoint = None
        self.aligning = True
        self.reached = False
        self.legs = []
        self.trajectories = 0
        self.committed = 0

    @property
    def goal(self):
        return self.goals[self.goal_index]

    @property
    def position(self):
        return self.model.position


class Link:
    """Line-delimited JSON over TCP, as the bridge speaks it."""

    def __init__(self, port, timeout_s):
        deadline = time.monotonic() + timeout_s
        while True:
            try:
                self.socket = socket.create_connection(("127.0.0.1", port), timeout=1.0)
                break
            except OSError:
                if time.monotonic() >= deadline:
                    raise RuntimeError("bridge did not open its TCP port")
                time.sleep(0.2)
        self.socket.settimeout(0.5)
        self.lock = threading.Lock()
        self.rows = []
        self.alive = True
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        buffer = b""
        while self.alive:
            try:
                chunk = self.socket.recv(65536)
            except socket.timeout:
                continue
            except OSError:
                break
            if not chunk:
                break
            buffer += chunk
            while b"\n" in buffer:
                line, buffer = buffer.split(b"\n", 1)
                try:
                    row = json.loads(line)
                except ValueError:
                    continue
                with self.lock:
                    self.rows.append(row)
        self.alive = False

    def send(self, row):
        self.socket.sendall((json.dumps(row, separators=(",", ":")) + "\n").encode("utf-8"))

    def take(self):
        with self.lock:
            rows, self.rows = self.rows, []
        return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--seed", type=int, default=7)
    parser.add_argument("--min-legs", type=int, default=1)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--report", default="test-results/bridge_obstacles.json")
    parser.add_argument("--expect-collision", action="store_true",
                        help="control run with dyn_obstacles off: pass only if a rover hits an obstacle")
    args = parser.parse_args()

    rng = random.Random(args.seed)
    rovers = {
        "JR81": Rover("JR81", (-5.0, 0.0, 0.0), [(5.0, 0.0, 0.0), (-5.0, 0.0, 0.0)], yaw=0.0),
        "RR83": Rover("RR83", (0.0, -5.0, 0.0), [(0.0, 5.0, 0.0), (0.0, -5.0, 0.0)], yaw=math.pi / 2.0),
    }
    wanderer = Wanderer(rng, (-1.5, -2.0, 0.0), (2.0, 2.0, 0.0), OBSTACLE_SPEED)

    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    log_dir = os.path.dirname(os.path.abspath(args.report))
    os.makedirs(log_dir, exist_ok=True)
    bridge_log = open(os.path.join(log_dir, "bridge.log"), "w")
    bridge = subprocess.Popen([sys.executable, os.path.join(repo, "integration", "ros1_bridge.py")],
                              stdout=bridge_log, stderr=subprocess.STDOUT)
    try:
        report = run(args, rovers, wanderer, rng)
    finally:
        bridge.terminate()
        try:
            bridge.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            bridge.kill()
        bridge_log.close()

    with open(args.report, "w") as output:
        json.dump(report, output, indent=2, sort_keys=True)
        output.write("\n")
    print(json.dumps(report, indent=2, sort_keys=True), flush=True)
    return 0 if report["passed"] else 1


def run(args, rovers, wanderer, rng):
    link = Link(args.port, timeout_s=30.0)

    def send_states():
        for rover in rovers.values():
            model = rover.model
            link.send({"type": "state", "agent": rover.name, "p": model.position[:2], "v": model.velocity[:2],
                       "yaw": model.yaw})

    def send_moving_obstacle():
        noisy = [value + rng.uniform(-OBSERVATION_NOISE, OBSERVATION_NOISE) for value in wanderer.position[:2]]
        link.send({"type": "obstacle", "id": 4000, "p": noisy, "size": list(MOVING_BBOX[:2]), "age_s": 0.0})

    def send_crate():
        link.send({"type": "obstacle", "id": 4100, "p": CRATE_POSITION[:2], "size": list(CRATE_BBOX[:2]),
                   "static": True})

    # Wait until both planners listen to the states the bridge republishes.
    obstacle_rows_seen = 0
    deadline = time.monotonic() + 40.0
    subscribed = False
    while not subscribed:
        send_states()
        send_crate()
        send_moving_obstacle()
        for row in link.take():
            if row.get("type") == "heartbeat":
                subscribed = bool(row.get("peer_subscribed"))
                obstacle_rows_seen = max(obstacle_rows_seen, int(row.get("obstacle_rows", 0)))
        if time.monotonic() >= deadline:
            raise RuntimeError("planners did not subscribe to the bridge")
        time.sleep(0.05)
    for _ in range(60):
        send_states()
        send_moving_obstacle()
        time.sleep(0.02)

    start_time = time.monotonic()
    leg_start = start_time
    all_reached_since = None
    last_tick = start_time
    last_moving = last_crate = last_progress = 0.0
    obstacle_gap = {"value": float("inf"), "at_s": 0.0, "rover": None, "obstacle": None, "speed": 0.0}
    rover_gap = {"value": float("inf"), "at_s": 0.0}
    collision_s = 0.0
    goal_acks = 0

    while link.alive:
        now = time.monotonic()
        elapsed = now - start_time
        if elapsed >= args.duration:
            break
        dt = min(now - last_tick, 0.05)
        last_tick = now

        for row in link.take():
            kind = row.get("type")
            rover = rovers.get(row.get("agent"))
            if kind == "setpoint" and rover is not None:
                rover.setpoint = (row["p"], row["v"], row.get("yaw", rover.model.yaw))
            elif kind == "trajectory" and rover is not None:
                rover.trajectories += 1
                rover.committed += 1 if row.get("committed") else 0
            elif kind == "heartbeat":
                obstacle_rows_seen = max(obstacle_rows_seen, int(row.get("obstacle_rows", 0)))
            elif kind == "goal_batch_ack":
                goal_acks += 1

        wanderer.step(dt, elapsed)

        # As the simulator bridge does, rovers first turn towards their goals; the paired goals go out together.
        if all(rover.aligning for rover in rovers.values()):
            facing = []
            for rover in rovers.values():
                heading = math.atan2(rover.goal[1] - rover.position[1], rover.goal[0] - rover.position[0])
                facing.append(rover.model.turn_towards(heading, dt))
            if all(facing):
                for rover in rovers.values():
                    rover.aligning = False
                    rover.setpoint = None
                link.send({"type": "goal_batch", "leg": sum(len(r.legs) for r in rovers.values()),
                           "goals": {rover.name: list(rover.goal[:2]) for rover in rovers.values()}})
        else:
            for rover in rovers.values():
                if rover.setpoint is None:
                    rover.model.stop(dt)
                else:
                    target, feedforward, yaw = rover.setpoint
                    rover.model.track(target, feedforward, yaw, dt)

        send_states()
        if now - last_moving >= 0.05:
            send_moving_obstacle()
            last_moving = now
        if now - last_crate >= 1.0:
            send_crate()
            last_crate = now

        colliding = False
        for rover in rovers.values():
            for label, position, bbox in (("moving", wanderer.position, MOVING_BBOX), ("crate", CRATE_POSITION, CRATE_BBOX)):
                gap = box_gap(rover.position, ROVER_BBOX, position, bbox)
                if gap < obstacle_gap["value"]:
                    obstacle_gap = {"value": gap, "at_s": elapsed, "rover": rover.name, "obstacle": label,
                                    "speed": rover.model.speed}
                colliding = colliding or gap < 0.0
        if colliding:
            collision_s += dt
        gap = box_gap(rovers["JR81"].position, ROVER_BBOX, rovers["RR83"].position, ROVER_BBOX)
        if gap < rover_gap["value"]:
            rover_gap = {"value": gap, "at_s": elapsed}

        for rover in rovers.values():
            if not rover.reached and not rover.aligning and distance(rover.position, rover.goal) < REACH_TOLERANCE:
                rover.reached = True
                rover.legs.append(round(now - leg_start, 3))
        if all(rover.reached for rover in rovers.values()):
            if all_reached_since is None:
                all_reached_since = now
            elif now - all_reached_since >= 1.0:
                for rover in rovers.values():
                    rover.goal_index = (rover.goal_index + 1) % len(rover.goals)
                    rover.reached = False
                    rover.aligning = True
                leg_start = now
                all_reached_since = None

        if now - last_progress >= 10.0:
            print("elapsed %.1fs legs %s min obstacle gap %.3f m min rover gap %.3f m obstacle rows %d"
                  % (elapsed, [len(r.legs) for r in rovers.values()], obstacle_gap["value"], rover_gap["value"],
                     obstacle_rows_seen), flush=True)
            last_progress = now
        time.sleep(0.02)

    link.alive = False
    collided = obstacle_gap["value"] < 0.0
    gates = {
        "bridge_stayed_up": time.monotonic() - start_time >= args.duration - 1.0,
        "planned": all(rover.committed > 0 for rover in rovers.values()),
        "goals_acknowledged": goal_acks > 0,
        "rovers_apart": rover_gap["value"] >= 0.0,
    }
    if args.expect_collision:
        gates["scenario_collides_without_feature"] = collided
    else:
        gates["obstacle_rows_reached_bridge"] = obstacle_rows_seen > 0
        gates["no_collision"] = not collided
        gates["legs"] = all(len(rover.legs) >= args.min_legs for rover in rovers.values())

    return {
        "scenario": "bridge",
        "control_run_feature_disabled": args.expect_collision,
        "seed": args.seed,
        "duration_s": round(time.monotonic() - start_time, 3),
        "obstacle_rows_counted_by_bridge": obstacle_rows_seen,
        "minimum_box_gap_m": round(obstacle_gap["value"], 4),
        "minimum_box_gap_at_s": round(obstacle_gap["at_s"], 3),
        "minimum_box_gap_robot": obstacle_gap["rover"],
        "minimum_box_gap_obstacle": obstacle_gap["obstacle"],
        "agent_speed_at_minimum_gap_mps": round(obstacle_gap["speed"], 3),
        "time_in_collision_s": round(collision_s, 3),
        "minimum_robot_gap_m": round(rover_gap["value"], 4),
        "minimum_robot_gap_at_s": round(rover_gap["at_s"], 3),
        "legs_reached": min(len(rover.legs) for rover in rovers.values()),
        "robots": {rover.name: {"legs_reached": len(rover.legs), "leg_completion_s": rover.legs,
                                "trajectory_count": rover.trajectories,
                                "committed_trajectory_count": rover.committed,
                                "final_error_m": round(distance(rover.position, rover.goal), 4)}
                   for rover in rovers.values()},
        "gates": gates,
        "passed": all(gates.values()),
    }


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as error:
        print("bridge_obstacles_test: " + str(error), file=sys.stderr, flush=True)
        sys.exit(2)
