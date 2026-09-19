#!/usr/bin/env python3
"""Validate synchronized ROS camera streams and optionally save actual RGB frames."""

import argparse
from collections import OrderedDict
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage


def stamp_key(header):
    return header.stamp.sec * 1_000_000_000 + header.stamp.nanosec


class CameraCheck(Node):
    def __init__(self):
        super().__init__('camera_smoke_test')
        self.samples = OrderedDict()
        self.complete = []
        self.start = time.monotonic()
        for name in ('external', 'wrist'):
            self.create_subscription(
                Image, f'/cameras/{name}/image_raw',
                lambda msg, name=name: self.receive(name + '_image', msg),
                qos_profile_sensor_data)
            self.create_subscription(
                CameraInfo, f'/cameras/{name}/camera_info',
                lambda msg, name=name: self.receive(name + '_info', msg),
                qos_profile_sensor_data)
        self.create_subscription(JointState, '/joint_states',
                                 lambda msg: self.receive('joints', msg), 20)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)

    def transforms(self, message):
        for transform in message.transforms:
            for name in ('external', 'wrist'):
                if transform.child_frame_id == f'{name}_camera_optical_frame':
                    self.receive(name + '_tf', transform)

    def receive(self, key, message):
        stamp = stamp_key(message.header)
        sample = self.samples.setdefault(stamp, {})
        sample[key] = message
        if len(sample) == 7 and not sample.get('checked'):
            for name in ('external', 'wrist'):
                img, info, tf = (sample[name + suffix] for suffix in ('_image', '_info', '_tf'))
                assert img.header.frame_id == info.header.frame_id == tf.child_frame_id
                assert tf.header.frame_id == 'base_link'
                assert img.encoding == 'rgb8'
                assert img.width == info.width and img.height == info.height
                assert img.step == img.width * 3 and len(img.data) == img.step * img.height
                assert info.k[0] > 0 and info.k[4] > 0 and info.k[8] == 1
                assert info.p[0] == info.k[0] and info.p[5] == info.k[4]
                rgb = self.pixels(img)
                assert np.std(rgb) > 5, f'{name}: empty or constant image'
            assert len(sample['joints'].position) == 9
            sample['checked'] = True
            self.complete.append((stamp, sample))
        while len(self.samples) > 180:
            self.samples.popitem(last=False)

    @staticmethod
    def pixels(message):
        return np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.width, 3)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--frames', type=int, default=20)
    args = parser.parse_args()
    rclpy.init()
    node = CameraCheck()
    try:
        deadline = time.monotonic() + 20
        while len(node.complete) < args.frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(node.complete) >= args.frames, (
            f'Only {len(node.complete)} matched samples; start simulation with cameras:=true')
        times = [stamp for stamp, _ in node.complete]
        assert all(b > a for a, b in zip(times, times[1:])), 'Camera stamps did not advance'
        simulation_hz = (len(times) - 1) * 1e9 / (times[-1] - times[0])
        elapsed = time.monotonic() - node.start
        print(f'PASS: {len(times)} synchronized RGB/CameraInfo/TF/joint samples, '
              f'{simulation_hz:.2f} Hz simulation time; collected in {elapsed:.2f}s wall time')
        if args.output_dir:
            from PIL import Image as PillowImage
            args.output_dir.mkdir(parents=True, exist_ok=True)
            for name in ('external', 'wrist'):
                rgb = node.pixels(node.complete[-1][1][name + '_image'])
                output = args.output_dir / f'{name}.png'
                PillowImage.fromarray(rgb).save(output)
                print(output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
