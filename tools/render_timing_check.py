"""Measure received joint/camera cadence without changing the running simulation."""

import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, JointState


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seconds', type=float, default=15)
    args = parser.parse_args()
    rclpy.init()
    node = Node('render_timing_check')
    samples = {'joints': [], 'camera': []}
    def record(name, msg):
        samples[name].append((time.monotonic(),
                             msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9))
    node.create_subscription(JointState, '/joint_states',
                             lambda msg: record('joints', msg), qos_profile_sensor_data)
    node.create_subscription(Image, '/cameras/external/image_raw',
                             lambda msg: record('camera', msg), qos_profile_sensor_data)
    try:
        warmup = time.monotonic() + 2
        while time.monotonic() < warmup:
            rclpy.spin_once(node, timeout_sec=.1)
        samples['joints'].clear()
        samples['camera'].clear()
        deadline = time.monotonic() + args.seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=.1)
        result = {}
        for name, data in samples.items():
            assert len(data) > 2, f'No {name} stream'
            values = np.array(data)
            gaps = np.diff(values[:, 0]) * 1000
            duration = values[-1, 0] - values[0, 0]
            result[name] = {
                'wall_hz': round((len(data) - 1) / duration, 2),
                'gap_p95_ms': round(float(np.percentile(gaps, 95)), 2),
                'gap_max_ms': round(float(gaps.max()), 2),
                'gaps_over_50ms': int(np.count_nonzero(gaps > 50)),
                'sim_wall_ratio': round((values[-1, 1] - values[0, 1]) / duration, 3)}
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
