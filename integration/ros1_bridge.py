#!/usr/bin/env python3
"""ROS 1 end of the private TCP bridge to the ROS 2 rover simulator."""
import json
import math
import socket
import threading
import time

import rospy
from geometry_msgs.msg import PoseStamped
from rmader_msgs.msg import DynTraj
from snapstack_msgs.msg import Goal, State

NAMES = {"JR81": "SQ81s", "RR83": "SQ83s"}


class Bridge:
    def __init__(self):
        self.client = None
        self.lock = threading.Lock()
        self.states = {}
        self.goals = {}
        self.counts = {name: {"setpoints": 0, "trajs": 0, "committed": 0,
                              "max_plan_yaw_rate": 0.0, "yaw_rate_violations": 0,
                              "max_abs_vertical": 0.0} for name in NAMES}
        self.last_setpoint = {name: 0.0 for name in NAMES}
        for name, ns in NAMES.items():
            self.states[name] = rospy.Publisher("/" + ns + "/state", State, queue_size=5)
            self.goals[name] = rospy.Publisher("/" + ns + "/term_goal", PoseStamped, queue_size=1, latch=True)
            rospy.Subscriber("/" + ns + "/goal", Goal, self.on_setpoint, callback_args=name, queue_size=10)
            rospy.Subscriber("/" + ns + "/rmader/trajs", DynTraj, self.on_traj, callback_args=name, queue_size=10)
        rospy.Timer(rospy.Duration(5.0), self.heartbeat)

    def send(self, row):
        blob = (json.dumps(row, separators=(",", ":")) + "\n").encode("utf-8")
        with self.lock:
            if self.client is None:
                return
            try:
                self.client.sendall(blob)
            except OSError:
                self.client = None

    def on_setpoint(self, msg, name):
        self.counts[name]["setpoints"] += 1
        self.last_setpoint[name] = time.monotonic()
        vertical = max(abs(msg.p.z), abs(msg.v.z), abs(msg.a.z), abs(msg.j.z))
        self.counts[name]["max_abs_vertical"] = max(
            self.counts[name]["max_abs_vertical"], vertical)
        speed2 = msg.v.x**2 + msg.v.y**2
        if speed2 >= 0.15**2:
            rate = abs(msg.v.x*msg.a.y - msg.v.y*msg.a.x) / speed2
            self.counts[name]["max_plan_yaw_rate"] = max(
                self.counts[name]["max_plan_yaw_rate"], rate)
            if rate > 0.95:
                self.counts[name]["yaw_rate_violations"] += 1
        self.send({"type": "setpoint", "agent": name, "p": [msg.p.x, msg.p.y],
                   "v": [msg.v.x, msg.v.y], "yaw": msg.psi})

    def on_traj(self, msg, name):
        self.counts[name]["trajs"] += 1
        if msg.is_committed:
            self.counts[name]["committed"] += 1
        self.send({"type": "trajectory", "agent": name, "committed": bool(msg.is_committed)})

    def heartbeat(self, _event):
        self.send({"type": "heartbeat", "counts": self.counts,
                   "peer_subscribed": all(self.states[n].get_num_connections() > 0 for n in NAMES)})

    def receive(self, row):
        name = row.get("agent")
        if name not in NAMES:
            return
        if row.get("type") == "state":
            vals = list(row.get("p", [])) + list(row.get("v", [])) + [row.get("yaw", 0.0)]
            if len(vals) != 5 or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in vals):
                return
            m = State()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "world"
            m.state_stamp = m.header.stamp
            m.pos.x, m.pos.y = vals[:2]
            m.vel.x, m.vel.y = vals[2:4]
            m.quat.z = math.sin(vals[4] / 2.0)
            m.quat.w = math.cos(vals[4] / 2.0)
            self.states[name].publish(m)
        elif row.get("type") == "goal":
            p = row.get("p")
            if not isinstance(p, list) or len(p) != 2 or not all(isinstance(v, (float, int)) and math.isfinite(v) for v in p):
                return
            m = PoseStamped()
            m.header.stamp = rospy.Time.now()
            m.header.frame_id = "world"
            m.pose.position.x, m.pose.position.y = p
            m.pose.orientation.w = 1.0
            self.goals[name].publish(m)

    def serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("0.0.0.0", 8765))
        server.listen(1)
        server.settimeout(1.0)
        rospy.loginfo("R-MADER bridge listening on private Docker network port 8765")
        while not rospy.is_shutdown():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            rospy.loginfo("R-MADER bridge connected: %s", addr)
            conn.settimeout(1.0)
            with self.lock:
                self.client = conn
            buf = b""
            while not rospy.is_shutdown():
                try:
                    chunk = conn.recv(65536)
                    if not chunk:
                        break
                    buf += chunk
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        if len(line) > 4096:
                            continue
                        try:
                            self.receive(json.loads(line))
                        except (ValueError, TypeError, KeyError) as exc:
                            rospy.logwarn("bad bridge message: %s", exc)
                except socket.timeout:
                    continue
                except OSError:
                    break
            with self.lock:
                self.client = None
            conn.close()
            rospy.logwarn("R-MADER bridge disconnected")
        server.close()


if __name__ == "__main__":
    rospy.init_node("rover_sim_bridge", disable_signals=True)
    Bridge().serve()
