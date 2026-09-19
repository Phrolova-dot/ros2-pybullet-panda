"""The renderer replica must match snapshots without advancing live physics."""

import time
from types import SimpleNamespace

import numpy as np
import pytest

from test_bullet_world import model_path
from pybullet_arm_sim.bullet_world import BulletWorld
from pybullet_arm_sim.camera_worker import CameraWorker, SnapshotRenderer, snapshot
from pybullet_arm_sim.cameras import CameraRig
from pybullet_arm_sim.sorting_scene import create_sorting_scene
from pybullet_arm_sim.simulation_node import SimulationNode


def test_replica_matches_live_pixels_and_reset():
    world = BulletWorld(model_path())
    renderer = SnapshotRenderer(model_path(), 224, 224)
    try:
        zones = create_sorting_scene(world, 7)
        for _ in range(120):
            world.step()
        before = world.joint_state()
        rig = CameraRig(world)
        stamp, generation, frames = renderer.render(snapshot(world, 123, 0, zones))
        assert (stamp, generation) == (123, 0)
        for name in rig.names:
            direct = rig.render(name)
            np.testing.assert_array_equal(frames[name].rgb, direct.rgb)
            np.testing.assert_allclose(frames[name].position, direct.position)
        assert world.joint_state() == before
        world.set_joint_targets({'panda_joint1': .2})
        world.spawn_box('new_part', (.5, -.2, .04), (0, 0, 0, 1),
                        (.05, .05, .08), .1, (1, 0, 0, 1))
        for _ in range(120):
            world.step()
        _, _, frames = renderer.render(snapshot(world, 234, 0, zones))
        for name in rig.names:
            np.testing.assert_array_equal(frames[name].rgb, rig.render(name).rgb)
        world.reset_world()
        _, generation, frames = renderer.render(snapshot(world, 456, 1, []))
        assert generation == 1
        assert renderer.world.objects == {}
        np.testing.assert_array_equal(frames['external'].rgb, rig.render('external').rgb)
    finally:
        renderer.world.disconnect()
        world.disconnect()


def test_process_worker_bounded_and_nonblocking():
    world = BulletWorld(model_path())
    worker = CameraWorker(model_path(), 224, 224)
    try:
        zones = create_sorting_scene(world, 42)
        state = snapshot(world, 123, 0, zones)
        assert worker.submit(state)
        assert not worker.submit(state)
        result = None
        deadline = time.monotonic() + 15
        while result is None and time.monotonic() < deadline:
            # The owner continues stepping while the other process renders.
            world.step()
            result = worker.poll()
            time.sleep(.005)
        assert result is not None
        assert result[:2] == (123, 0)
        assert worker.idle
    finally:
        worker.close()
        world.disconnect()


@pytest.mark.parametrize('paused,generation,expected', [
    (False, 1, True), (False, 0, False), (True, 1, False)])
def test_completed_frames_are_filtered_after_reset_or_pause(paused, generation, expected):
    published = []
    fake = SimpleNamespace(
        camera_worker=SimpleNamespace(poll=lambda: (123, generation, {})),
        camera_generation=1, paused=paused,
        _stamp=lambda stamp: stamp,
        _publish_cameras=lambda *args: published.append(args),
        world=SimpleNamespace(step=lambda: None),
        sim_time_nanoseconds=0, physics_hz=240, step_count=0, publish_every=4)
    SimulationNode._step_callback(fake)
    assert bool(published) == expected
