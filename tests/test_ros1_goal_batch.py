"""Paired crossing goals share one ROS timestamp and are published together."""
import sys
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "integration"))
import ros1_bridge
from ros1_bridge import Bridge


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class GoalBatchTest(unittest.TestCase):
    def test_paired_goals_use_one_stamp(self):
        pubs = {name: Publisher() for name in ("JR81", "RR83")}
        replies = []
        bridge = SimpleNamespace(goals=pubs, send=replies.append)
        with mock.patch.object(ros1_bridge.rospy.Time, "now",
                               return_value=ros1_bridge.rospy.Time.from_sec(123.0)):
            Bridge.receive(bridge, {"type": "goal_batch", "leg": 0,
                                    "goals": {"JR81": [3.0, 0.0], "RR83": [0.0, 3.0]}})
        self.assertEqual([len(p.messages) for p in pubs.values()], [1, 1])
        self.assertEqual(pubs["JR81"].messages[0].header.stamp,
                         pubs["RR83"].messages[0].header.stamp)
        self.assertEqual(replies[0]["type"], "goal_batch_ack")
        self.assertEqual(replies[0]["stamp"], pubs["JR81"].messages[0].header.stamp.to_sec())

    def test_invalid_pair_publishes_neither_goal(self):
        pubs = {name: Publisher() for name in ("JR81", "RR83")}
        bridge = SimpleNamespace(goals=pubs, send=lambda row: None)
        Bridge.receive(bridge, {"type": "goal_batch", "leg": 0,
                                "goals": {"JR81": [3.0, 0.0], "RR83": [float("nan"), 3.0]}})
        self.assertEqual([len(p.messages) for p in pubs.values()], [0, 0])


if __name__ == "__main__":
    unittest.main()
