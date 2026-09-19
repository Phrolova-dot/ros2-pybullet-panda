"""Independently verify a running sorting batch using simulator ground truth."""

import argparse
import json
import math
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--timeout', type=float, default=240)
    parser.add_argument('--image', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node('sorting_smoke_test')
    latest = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    node.create_subscription(String, '/sorting/status',
                             lambda msg: latest.update(status=json.loads(msg.data)), qos)
    node.create_subscription(MarkerArray, '/simulation/objects',
                             lambda msg: latest.update(objects=msg.markers), qos)
    node.create_subscription(Image, '/sorting/annotated_image',
                             lambda msg: latest.update(image=msg), qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
            status = latest.get('status', {})
            if status.get('state') in ('ERROR', 'STOPPED'):
                raise RuntimeError(str(status))
            if status.get('state') == 'COMPLETE' and 'objects' in latest:
                break
        else:
            raise RuntimeError('Timed out waiting for COMPLETE')
        assert status['counts'] == {'red': 2, 'blue': 2}, status
        parts = [m for m in latest['objects'] if m.ns == 'pybullet_objects']
        assert len(parts) == 4, len(parts)
        occupied = set()
        for part in parts:
            color = 'red' if part.color.r > part.color.b else 'blue'
            y = .28 if color == 'red' else -.28
            p = part.pose.position
            x = min((.36, .50), key=lambda value: abs(value - p.x))
            assert math.dist((p.x, p.y, p.z), (x, y, .035)) < .03, (part.text, p)
            assert (color, x) not in occupied, occupied
            occupied.add((color, x))
            print(f'{part.text}: {color} ({p.x:.4f}, {p.y:.4f}, {p.z:.4f})')
        if args.image:
            # Wait for an annotated frame reflecting the final status.
            until = time.monotonic() + 2
            while time.monotonic() < until:
                rclpy.spin_once(node, timeout_sec=.1)
            msg = latest['image']
            rgb = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
            rgb = rgb[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
            args.image.parent.mkdir(parents=True, exist_ok=True)
            assert cv2.imwrite(str(args.image), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        print('PASS: four physical parts in unique, color-correct slots')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
