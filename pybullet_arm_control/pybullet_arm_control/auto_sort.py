"""Finite-batch RGB color sorting with physical grasp feedback."""

from collections import OrderedDict
import json
import math
import time

import cv2
import numpy as np
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float64, String
from tf2_msgs.msg import TFMessage

from .color_detection import detect_blocks, infeed
from .pick_place import PickPlace


SLOTS = {'red': ((.36, .28), (.50, .28)), 'blue': ((.36, -.28), (.50, -.28))}
FINGER_LINKS = {'panda_leftfinger', 'panda_rightfinger'}


def grasped_bodies(contacts):
    grouped = {}
    for contact in contacts:
        if contact.normal_force > .05 and contact.link_name in FINGER_LINKS:
            grouped.setdefault(contact.body_id, set()).add(contact.link_name)
    return {body for body, fingers in grouped.items() if FINGER_LINKS.issubset(fingers)}


class GraspFailure(RuntimeError):
    pass


class AutoSort(PickPlace):
    def __init__(self):
        # Only camera images, calibration, TF and physical feedback are used.
        super().__init__(node_name='auto_sort', use_object_feedback=False)
        self.declare_parameter('expected_parts', 4)
        self.declare_parameter('max_retries', 1)
        self.declare_parameter('keep_alive', True)
        self.expected = int(self.get_parameter('expected_parts').value)
        self.max_retries = int(self.get_parameter('max_retries').value)
        if not 1 <= self.expected <= 4 or not 0 <= self.max_retries <= 2:
            raise ValueError('expected_parts must be 1..4 and max_retries 0..2')
        self.samples = OrderedDict()
        self.observation = None
        self.held_body = None
        self.touching_bodies = set()
        self.counts = {'red': 0, 'blue': 0}
        self.status = {'state': 'WAITING', 'counts': self.counts.copy(), 'detail': ''}
        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.status_pub = self.create_publisher(String, '/sorting/status', status_qos)
        self.image_pub = self.create_publisher(Image, '/sorting/annotated_image', qos_profile_sensor_data)
        self.create_subscription(Image, '/cameras/external/image_raw',
                                 lambda msg: self.sensor('image', msg), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, '/cameras/external/camera_info',
                                 lambda msg: self.sensor('info', msg), qos_profile_sensor_data)
        self.create_subscription(TFMessage, '/tf', self.transforms, 100)
        self.create_timer(1.0, self.publish_status)

    def publish_status(self):
        self.status_pub.publish(String(data=json.dumps(self.status)))

    def state(self, state, detail=''):
        self.status = {
            'state': state, 'counts': self.counts.copy(),
            'sorted': sum(self.counts.values()), 'expected': self.expected, 'detail': detail}
        self.get_logger().info(f'{state}: {detail} | red={self.counts["red"]} blue={self.counts["blue"]}')
        self.publish_status()

    def contacts(self, message):
        self.touching_bodies = grasped_bodies(message.contacts)
        self.contact_time = time.monotonic()

    def gripping(self):
        if time.monotonic() - self.contact_time > .5:
            return False
        if self.held_body is None:
            return len(self.touching_bodies) == 1
        return self.held_body in self.touching_bodies

    def transforms(self, message):
        for transform in message.transforms:
            if (transform.child_frame_id == 'external_camera_optical_frame'
                    and transform.header.frame_id == 'base_link'):
                self.sensor('tf', transform)

    def sensor(self, kind, message):
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        sample = self.samples.setdefault(stamp, {})
        sample[kind] = message
        while len(self.samples) > 30:
            self.samples.popitem(last=False)
        if len(sample) != 3:
            return
        image, info, transform = sample['image'], sample['info'], sample['tf']
        if (image.encoding != 'rgb8' or image.header.frame_id != info.header.frame_id
                or image.header.frame_id != transform.child_frame_id
                or image.width != info.width or image.height != info.height
                or len(image.data) != image.height * image.step):
            return
        pixels = np.frombuffer(image.data, dtype=np.uint8).reshape(image.height, image.step)
        rgb = pixels[:, :image.width * 3].reshape(image.height, image.width, 3)
        t, q = transform.transform.translation, transform.transform.rotation
        detections = detect_blocks(rgb, info.k, (t.x, t.y, t.z), (q.x, q.y, q.z, q.w))
        self.observation = (stamp, time.monotonic(), detections)
        canvas = rgb.copy()
        for detection in detections:
            point = tuple(round(v) for v in detection.pixel)
            cv2.circle(canvas, point, 7, (0, 255, 0), 1)
            label = f'{detection.color} {detection.x:.2f},{detection.y:.2f}'
            cv2.putText(canvas, label, (point[0] - 25, point[1] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, .32, (0, 80, 0), 1)
        cv2.rectangle(canvas, (0, 0), (image.width, 43), (25, 25, 25), -1)
        cv2.putText(canvas, self.status['state'], (8, 17),
                    cv2.FONT_HERSHEY_SIMPLEX, .45, (255, 255, 255), 1)
        cv2.putText(canvas, f'Red: {self.counts["red"]}/2   Blue: {self.counts["blue"]}/2',
                    (8, 36), cv2.FONT_HERSHEY_SIMPLEX, .40, (255, 255, 255), 1)
        annotated = Image()
        annotated.header = image.header
        annotated.width, annotated.height = image.width, image.height
        annotated.encoding = 'rgb8'
        annotated.step = image.width * 3
        annotated.data = canvas.tobytes()
        self.image_pub.publish(annotated)

    def scan(self):
        previous = self.observation[0] if self.observation else -1
        # A delayed rendered frame must have been captured after the arm stopped.
        minimum_stamp = max(previous, self.joint_stamp)
        self.wait(lambda: self.observation is not None and self.observation[0] > minimum_stamp
                  and time.monotonic() - self.observation[1] < 1.0,
                  8.0, 'fresh synchronized RGB/CameraInfo/TF (enable cameras and resume simulation)')
        return self.observation[2]

    def park(self):
        self.move((.40, 0.0, .42), 'PARK FOR VISION')

    def grasp(self, target):
        self.held_body = None
        self.open_gripper()
        self.state('APPROACH', target.color)
        self.move((target.x, target.y, .20), 'APPROACH')
        self.state('DESCEND')
        self.move((target.x, target.y, .045), 'DESCEND')
        self.state('GRASP')
        self.gripper.publish(Float64(data=0.0))
        try:
            self.wait(self.gripping, 4.0, 'both fingers contacting the same part')
        except RuntimeError as error:
            raise GraspFailure(str(error)) from error
        self.held_body = next(iter(self.touching_bodies))
        self.settle(.8)
        if not self.gripping():
            raise GraspFailure('Contact was lost before lifting')

    def transfer(self, target, destination):
        self.state('LIFT')
        self.move((target.x, target.y, .25), 'LIFT')
        if not self.gripping():
            raise RuntimeError('Part lost during lift; stopped for inspection')
        self.state('TRANSFER', f'{target.color} -> {destination}')
        self.move((*destination, .25), 'TRANSFER')
        if not self.gripping():
            raise RuntimeError('Part lost during transfer; stopped for inspection')
        self.state('LOWER')
        self.move((*destination, .045), 'LOWER')
        self.state('RELEASE')
        self.open_gripper()
        self.held_body = None
        self.move((*destination, .25), 'RETREAT')
        self.park()
        self.state('VERIFY')
        detections = self.scan()
        if not any(d.color == target.color and math.dist((d.x, d.y), destination) < .03
                   for d in detections):
            raise RuntimeError('Placed part not confirmed by the external camera')
        self.counts[target.color] += 1
        self.state('SORTED', target.color)

    def run(self):
        self.state('WAITING')
        if not self.ik.wait_for_service(timeout_sec=10) or not self.action.wait_for_server(timeout_sec=10):
            raise RuntimeError('Start sorting.launch.py or a simulation with sorting_scene:=true')
        self.wait(lambda: self.tip is not None and self.gripper.get_subscription_count() > 0,
                  10.0, 'robot feedback')
        self.open_gripper()
        self.park()
        detections = self.scan()
        for color, slots in SLOTS.items():
            self.counts[color] = sum(any(
                d.color == color and math.dist((d.x, d.y), slot) < .03 for d in detections)
                for slot in slots)
        if sum(infeed(d) for d in detections) + sum(self.counts.values()) != self.expected:
            raise RuntimeError('Expected parts not visible; check the infeed, lighting and camera')
        while sum(self.counts.values()) < self.expected:
            self.state('DETECT')
            detections = self.scan()
            candidates = [d for d in detections if infeed(d)]
            if not candidates:
                raise RuntimeError('Unprocessed parts disappeared from the infeed')
            target = candidates[0]
            occupied = [any(math.dist((d.x, d.y), slot) < .04 for d in detections)
                        for slot in SLOTS[target.color]]
            available = [slot for slot, used in zip(SLOTS[target.color], occupied) if not used]
            if not available:
                raise RuntimeError(f'{target.color} destination is full')
            destination = available[0]
            for attempt in range(self.max_retries + 1):
                try:
                    self.grasp(target)
                    break
                except GraspFailure:
                    self.state('RETRY', f'grasp attempt {attempt + 1} failed')
                    self.open_gripper()
                    self.held_body = None
                    self.move((target.x, target.y, .25), 'RECOVER UP')
                    self.park()
                    if attempt >= self.max_retries:
                        raise
                    nearby = [d for d in self.scan() if infeed(d) and d.color == target.color
                              and math.dist((d.x, d.y), (target.x, target.y)) < .05]
                    if not nearby:
                        raise RuntimeError('Failed part could not be located for retry')
                    target = min(nearby, key=lambda d: math.dist((d.x, d.y), (target.x, target.y)))
            self.transfer(target, destination)
        if any(infeed(d) for d in self.scan()):
            raise RuntimeError('Parts remain in the infeed after the requested batch')
        self.state('COMPLETE', 'Batch sorted and verified by camera')


def main(args=None):
    rclpy.init(args=args)
    node = AutoSort()
    success = False
    try:
        node.run()
        success = True
        if bool(node.get_parameter('keep_alive').value):
            rclpy.spin(node)
    except KeyboardInterrupt:
        if not success:
            node.state('STOPPED', 'Interrupted by operator')
    except RuntimeError as error:
        node.state('ERROR', str(error))
    finally:
        if node.active_goal is not None and node.active_goal.accepted:
            future = node.active_goal.cancel_goal_async()
            rclpy.spin_until_future_complete(node, future, timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not success:
        raise SystemExit(1)
