"""A seeded batch of parts and visual destination pads for the sorting demo."""

import random

import pybullet as bullet


def create_sorting_scene(world, seed):
    rng = random.Random(seed)
    colors = [(0.95, 0.12, 0.04, 1.0)] * 2 + [(0.03, 0.12, 0.95, 1.0)] * 2
    rng.shuffle(colors)
    for index, (x, y) in enumerate(((.36, -.075), (.50, -.075), (.36, .075), (.50, .075))):
        world.spawn_box(
            name=f'part_{index}',
            position=(x + rng.uniform(-.006, .006), y + rng.uniform(-.006, .006), .035),
            orientation=(0, 0, 0, 1), size=(.06, .06, .07), mass=.1,
            color=colors[index],
        )
    zones = [
        ('RED', (.43, .28, -.0005), (.28, .14, .002), (.9, .75, .75, 1.0)),
        ('BLUE', (.43, -.28, -.0005), (.28, .14, .002), (.75, .80, .95, 1.0)),
    ]
    for label, position, size, color in zones:
        visual = bullet.createVisualShape(
            bullet.GEOM_BOX, halfExtents=[v / 2 for v in size], rgbaColor=color,
            physicsClientId=world.client_id)
        bullet.createMultiBody(
            baseMass=0, baseVisualShapeIndex=visual, baseCollisionShapeIndex=-1,
            basePosition=position, physicsClientId=world.client_id)
        bullet.addUserDebugText(
            label, (position[0], position[1], .005), textColorRGB=(.1, .1, .1),
            textSize=1.4, physicsClientId=world.client_id)
    return zones
