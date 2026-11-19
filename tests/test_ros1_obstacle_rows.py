"""Obstacle rows from the simulator become ObstacleObservation messages; malformed rows are dropped."""
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


def receive(row, now=100.0):
    publisher = Publisher()
    bridge = SimpleNamespace(obstacles=publisher, obstacle_rows=0, send=lambda reply: None)
    with mock.patch.object(ros1_bridge.rospy.Time, "now", return_value=ros1_bridge.rospy.Time.from_sec(now)):
        Bridge.receive(bridge, row)
    return publisher.messages, bridge.obstacle_rows


class ObstacleRowTest(unittest.TestCase):
    def test_planar_row_is_published(self):
        messages, count = receive({"type": "obstacle", "id": 7, "p": [1.0, -2.0], "size": [0.6, 0.5]})
        self.assertEqual(count, 1)
        message = messages[0]
        self.assertEqual(message.id, 7)
        self.assertEqual((message.pos.x, message.pos.y, message.pos.z), (1.0, -2.0, 0.0))
        self.assertEqual((message.bbox.x, message.bbox.y, message.bbox.z), (0.6, 0.5, ros1_bridge.ROVER_HEIGHT))
        self.assertFalse(message.has_vel)
        self.assertFalse(message.is_static)
        self.assertEqual(message.header.stamp.to_sec(), 100.0)

    def test_velocity_static_and_age(self):
        messages, _ = receive({"type": "obstacle", "id": 3, "p": [0.0, 0.0, 0.2], "size": [1.0, 1.0, 1.0],
                               "v": [0.1, -0.2], "static": True, "age_s": 0.25})
        message = messages[0]
        self.assertTrue(message.has_vel)
        self.assertEqual((message.vel.x, message.vel.y, message.vel.z), (0.1, -0.2, 0.0))
        self.assertTrue(message.is_static)
        self.assertAlmostEqual(message.header.stamp.to_sec(), 99.75)

    def test_malformed_rows_are_dropped(self):
        good = {"type": "obstacle", "id": 1, "p": [0.0, 0.0], "size": [0.5, 0.5]}
        bad = [
            dict(good, id="1"),
            dict(good, id=True),
            dict(good, p=[0.0]),
            dict(good, p=[0.0, float("nan")]),
            dict(good, size=[0.5, 0.0]),
            dict(good, size=[0.5, -1.0]),
            dict(good, v=[0.1]),
            dict(good, v=["fast", 0.0]),
            dict(good, static="yes"),
            dict(good, age_s=-1.0),
            dict(good, age_s=60.0),
            {"type": "obstacle"},
        ]
        for row in bad:
            messages, count = receive(row)
            self.assertEqual((messages, count), ([], 0), row)


if __name__ == "__main__":
    unittest.main()
