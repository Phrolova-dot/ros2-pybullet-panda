"""Read-only RGB cameras with ROS optical-frame coordinates."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
import pybullet as bullet

from .bullet_world import BulletWorld


@dataclass(frozen=True)
class CameraSpec:
    frame_id: str
    parent_frame: str
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]
    fov: float
    near: float = 0.02
    far: float = 4.0


@dataclass(frozen=True)
class CameraFrame:
    rgb: np.ndarray
    position: tuple[float, float, float]
    orientation: tuple[float, float, float, float]


def _look_at_orientation(eye, target, up) -> tuple[float, float, float, float]:
    """Return an optical x-right, y-down, z-forward quaternion."""
    forward = np.asarray(target, dtype=float) - np.asarray(eye, dtype=float)
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, np.asarray(up, dtype=float))
    right /= np.linalg.norm(right)
    down = np.cross(forward, right)
    rotation = np.column_stack((right, down, forward))

    # The symmetric quaternion matrix avoids Euler-angle singularities.
    r = rotation
    symmetric = np.array([
        [r[0, 0] - r[1, 1] - r[2, 2], r[0, 1] + r[1, 0],
         r[0, 2] + r[2, 0], r[2, 1] - r[1, 2]],
        [r[0, 1] + r[1, 0], r[1, 1] - r[0, 0] - r[2, 2],
         r[1, 2] + r[2, 1], r[0, 2] - r[2, 0]],
        [r[0, 2] + r[2, 0], r[1, 2] + r[2, 1],
         r[2, 2] - r[0, 0] - r[1, 1], r[1, 0] - r[0, 1]],
        [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0],
         r[1, 0] - r[0, 1], np.trace(r)],
    ]) / 3.0
    _, eigenvectors = np.linalg.eigh(symmetric)
    quaternion = eigenvectors[:, -1]
    if quaternion[3] < 0.0:
        quaternion = -quaternion
    return tuple(float(value) for value in quaternion)


class CameraRig:
    """Render a fixed scene camera and a hand-mounted camera without stepping."""

    names = ('external', 'wrist')

    def __init__(self, world: BulletWorld, width: int = 224, height: int = 224):
        if any(isinstance(value, bool) or not isinstance(value, int) or value < 1
               for value in (width, height)):
            raise ValueError('camera width and height must be positive integers')
        self.world = world
        self.width = width
        self.height = height
        external_eye = (0.90, -0.80, 0.80)
        wrist_eye = (0.08, 0.0, 0.02)
        self.specs = {
            'external': CameraSpec(
                'external_camera_optical_frame', 'base_link', external_eye,
                _look_at_orientation(external_eye, (0.33, 0.06, 0.32), (0, 0, 1)),
                60.0,
            ),
            'wrist': CameraSpec(
                'wrist_camera_optical_frame', 'panda_hand', wrist_eye,
                _look_at_orientation(wrist_eye, (0.0, 0.0, 0.38), (0, 1, 0)),
                75.0,
            ),
        }

    def projection(self, name: str) -> tuple[float, ...]:
        """Return the column-major OpenGL projection used for rendering."""
        spec = self.specs[name]
        return tuple(bullet.computeProjectionMatrixFOV(
            spec.fov, self.width / self.height, spec.near, spec.far))

    def intrinsics(self, name: str) -> tuple[float, ...]:
        """Return ROS K, matching PyBullet's centered projection and image axes."""
        projection = self.projection(name)
        return (
            projection[0] * self.width / 2.0, 0.0, self.width / 2.0,
            0.0, projection[5] * self.height / 2.0, self.height / 2.0,
            0.0, 0.0, 1.0,
        )

    def optical_pose(self, name: str):
        """Return the camera optical pose in world coordinates, not link COM."""
        spec = self.specs[name]
        parent_position, parent_orientation = self.world.link_pose(spec.parent_frame)
        position, orientation = bullet.multiplyTransforms(
            parent_position, parent_orientation, spec.position, spec.orientation)
        return tuple(position), tuple(orientation)

    def render(self, name: str) -> CameraFrame:
        position, orientation = self.optical_pose(name)
        rotation = np.asarray(bullet.getMatrixFromQuaternion(orientation)).reshape(3, 3)
        # OpenGL uses y-up and looks along -z; ROS optical uses y-down and +z.
        target = np.asarray(position) + rotation[:, 2]
        view = bullet.computeViewMatrix(position, target, -rotation[:, 1])
        result = bullet.getCameraImage(
            self.width,
            self.height,
            viewMatrix=view,
            projectionMatrix=self.projection(name),
            renderer=bullet.ER_TINY_RENDERER,
            flags=bullet.ER_NO_SEGMENTATION_MASK,
            physicsClientId=self.world.client_id,
        )
        rgba = np.asarray(result[2], dtype=np.uint8).reshape(self.height, self.width, 4)
        rgb = np.ascontiguousarray(rgba[:, :, :3])
        return CameraFrame(rgb, position, orientation)
