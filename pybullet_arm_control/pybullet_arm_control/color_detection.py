"""Locate upright, known-height colored blocks using RGB and camera calibration."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass(frozen=True)
class Detection:
    color: str
    x: float
    y: float
    pixel: tuple[float, float]
    area: float


def rotation_matrix(quaternion):
    q = np.asarray(quaternion, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-8:
        raise ValueError('Invalid optical-frame quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def pixel_to_plane(pixel, intrinsic, position, orientation, height):
    k = np.asarray(intrinsic, dtype=float).reshape(3, 3)
    origin = np.asarray(position, dtype=float)
    ray = rotation_matrix(orientation) @ np.linalg.solve(k, [*pixel, 1.0])
    if abs(ray[2]) < 1e-8:
        raise ValueError('Camera ray is parallel to the object plane')
    distance = (height - origin[2]) / ray[2]
    if distance <= 0:
        raise ValueError('Object plane lies behind the camera')
    return origin + distance * ray


def detect_blocks(rgb, intrinsic, position, orientation, center_height=0.035):
    """Project silhouette centers onto the known block-center plane.

    This approximation assumes separated, upright 6x6x7 cm blocks on a flat
    surface. It deliberately consumes neither simulator object poses nor IDs.
    """
    hsv = cv2.cvtColor(np.ascontiguousarray(rgb), cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv)
    saturated = (s >= 145) & (v >= 55)
    masks = {
        'red': saturated & ((h <= 12) | (h >= 170)),
        'blue': saturated & (h >= 95) & (h <= 130),
    }
    detections = []
    for color, mask in masks.items():
        contours, _ = cv2.findContours(
            mask.astype(np.uint8) * 255, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < max(8.0, rgb.shape[0] * rgb.shape[1] * 0.00010):
                continue
            x, y, width, height = cv2.boundingRect(contour)
            if min(width, height) < 3 or not 0.3 < width / height < 3.0:
                continue
            pixel = (x + (width - 1) / 2, y + (height - 1) / 2)
            try:
                point = pixel_to_plane(pixel, intrinsic, position, orientation, center_height)
            except ValueError:
                continue
            if 0.25 <= point[0] <= 0.60 and -0.38 <= point[1] <= 0.38:
                detections.append(Detection(color, float(point[0]), float(point[1]), pixel, area))
    return sorted(detections, key=lambda d: (d.x, d.y, d.color))


def infeed(detection):
    return 0.31 <= detection.x <= 0.55 and abs(detection.y) < 0.15
