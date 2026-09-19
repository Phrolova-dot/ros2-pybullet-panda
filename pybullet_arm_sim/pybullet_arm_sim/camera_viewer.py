"""Display the two ROS RGB streams side by side."""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')
        self.frames = {}
        self.received = {}
        for name in ('external', 'wrist'):
            self.create_subscription(
                Image, f'/cameras/{name}/image_raw',
                lambda message, name=name: self.receive(name, message),
                qos_profile_sensor_data)

    def receive(self, name, message):
        if message.encoding != 'rgb8' or len(message.data) != message.height * message.step:
            self.get_logger().warning(f'Ignoring invalid {name} image')
            return
        pixels = np.frombuffer(message.data, dtype=np.uint8).reshape(message.height, message.step)
        rgb = pixels[:, :message.width * 3].reshape(message.height, message.width, 3)
        stamp = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        self.frames[name] = (cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), stamp)
        self.received[name] = time.monotonic()

    def canvas(self):
        panels = []
        for name in ('external', 'wrist'):
            panel = np.full((380, 448, 3), 25, dtype=np.uint8)
            cv2.putText(panel, name.upper(), (14, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (235, 235, 235), 1)
            if name in self.frames:
                picture, stamp = self.frames[name]
                height, width = picture.shape[:2]
                ratio = min(448 / width, 320 / height)
                resized = cv2.resize(picture, (round(width * ratio), round(height * ratio)))
                ph, pw = resized.shape[:2]
                panel[35:35 + ph, (448 - pw)//2:(448 - pw)//2 + pw] = resized
                age = time.monotonic() - self.received[name]
                status = f'sim {stamp:.2f}s' if age < 2 else 'Paused / no new frames'
            else:
                status = 'Waiting for camera topic...'
            cv2.putText(panel, status, (14, 373), cv2.FONT_HERSHEY_SIMPLEX,
                        0.45, (200, 200, 200), 1)
            panels.append(panel)
        return np.hstack(panels)


def main(args=None):
    rclpy.init(args=args)
    node = CameraViewer()
    title = 'Panda cameras - Q / Esc to close'
    cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            cv2.imshow(title, node.canvas())
            if cv2.waitKey(1) & 0xff in (27, ord('q')):
                break
            if cv2.getWindowProperty(title, cv2.WND_PROP_VISIBLE) < 1:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
