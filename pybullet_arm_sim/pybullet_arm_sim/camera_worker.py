"""Render immutable physics snapshots in separate DIRECT-only processes."""

from concurrent.futures import ProcessPoolExecutor
import multiprocessing
from multiprocessing.util import Finalize
import signal

import pybullet as bullet

from .bullet_world import BulletWorld
from .cameras import CameraRig


def snapshot(world, stamp_ns, generation, zones):
    names, positions, _, _ = world.joint_state()
    return {
        'stamp_ns': stamp_ns, 'generation': generation,
        'joints': dict(zip(names, positions)),
        'objects': [(box.name, box.size, box.color, *world.box_pose(body))
                    for body, box in world.objects.items()],
        'zones': zones,
    }


class SnapshotRenderer:
    def __init__(self, xacro_path, width, height):
        self.world = BulletWorld(xacro_path)
        self.rig = CameraRig(self.world, width, height)
        self.generation = None

    def render(self, state, names=None):
        world = self.world
        if self.generation != state['generation']:
            world.reset_world()
            self.generation = state['generation']
            for _, position, size, color in state['zones']:
                visual = bullet.createVisualShape(
                    bullet.GEOM_BOX, halfExtents=[v / 2 for v in size],
                    rgbaColor=color, physicsClientId=world.client_id)
                bullet.createMultiBody(
                    baseMass=0, baseVisualShapeIndex=visual,
                    baseCollisionShapeIndex=-1, basePosition=position,
                    physicsClientId=world.client_id)
        for name, position in state['joints'].items():
            bullet.resetJointState(world.robot_id, world.joints[name].index,
                                   position, physicsClientId=world.client_id)
        object_names = {item[0] for item in state['objects']}
        for name in set(world.object_names) - object_names:
            body = world.object_names.pop(name)
            bullet.removeBody(body, physicsClientId=world.client_id)
            del world.objects[body]
        for name, size, color, position, orientation in state['objects']:
            if name not in world.object_names:
                world.spawn_box(name, position, orientation, size, 0, color)
            bullet.resetBasePositionAndOrientation(
                world.object_names[name], position, orientation,
                physicsClientId=world.client_id)
        # Never step this replica: its only source of motion is the snapshot.
        frames = {name: self.rig.render(name) for name in (names or self.rig.names)}
        return state['stamp_ns'], state['generation'], frames


_renderer = None


def _initialize(xacro_path, width, height):
    global _renderer
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    _renderer = SnapshotRenderer(xacro_path, width, height)
    Finalize(_renderer, _renderer.world.disconnect, exitpriority=10)


def _render(state, name):
    return _renderer.render(state, (name,))


class CameraWorker:
    """At most one in-flight frame pair; busy workers cause sampling to skip."""

    def __init__(self, xacro_path, width, height):
        self.pool = ProcessPoolExecutor(
            max_workers=2, mp_context=multiprocessing.get_context('spawn'),
            initializer=_initialize, initargs=(xacro_path, width, height))
        self.pending = None

    @property
    def idle(self):
        return self.pending is None

    def submit(self, state):
        if not self.idle:
            return False
        self.pending = [self.pool.submit(_render, state, name) for name in CameraRig.names]
        return True

    def poll(self):
        if self.pending is None or not all(future.done() for future in self.pending):
            return None
        futures, self.pending = self.pending, None
        results = [future.result() for future in futures]
        frames = {name: frame for _, _, pair in results for name, frame in pair.items()}
        return results[0][0], results[0][1], frames

    def close(self):
        self.pool.shutdown(wait=True, cancel_futures=True)
