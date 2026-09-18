"""Render the actual project model with PyBullet's software camera."""

from pathlib import Path
import sys

import numpy as np
from PIL import Image
import pybullet as bullet

from pybullet_arm_sim.bullet_world import BulletWorld


def main():
    root = Path(__file__).resolve().parents[1]
    output = Path(sys.argv[1])
    output.parent.mkdir(parents=True, exist_ok=True)
    world = BulletWorld(str(root / 'pybullet_arm_description/urdf/lab_arm.urdf.xacro'))
    try:
        world.spawn_box(
            name='training_cube', position=(0.43, 0.0, 0.035),
            orientation=(0.0, 0.0, 0.0, 1.0), size=(0.06, 0.06, 0.07),
            mass=0.10, color=(0.95, 0.25, 0.08, 1.0),
        )
        for _ in range(120):
            world.step()
        view = bullet.computeViewMatrix(
            cameraEyePosition=(1.25, -1.5, 1.05),
            cameraTargetPosition=(0.35, 0.0, 0.35),
            cameraUpVector=(0.0, 0.0, 1.0),
        )
        projection = bullet.computeProjectionMatrixFOV(40.0, 1.5, 0.02, 10.0)
        camera = bullet.getCameraImage(
            1500, 1000, viewMatrix=view, projectionMatrix=projection,
            shadow=1, lightDirection=(-1.0, -2.0, 4.0),
            renderer=bullet.ER_TINY_RENDERER, physicsClientId=world.client_id,
        )
        pixels = np.asarray(camera[2], dtype=np.uint8).reshape(1000, 1500, 4)
        Image.fromarray(pixels).convert('RGB').save(output)
        print(output)
    finally:
        world.disconnect()


if __name__ == '__main__':
    main()
