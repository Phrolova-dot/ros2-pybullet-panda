# ROS 2 + PyBullet Robotic Arm Lab

[中文](README_ZH.md) | [English](README.md)

This is a medium-sized ROS 2 Jazzy project for project-based learning. RViz2
and PyBullet use the same Xacro model: ROS 2 handles communication, control,
and visualization, while PyBullet performs the physics simulation.

The current version includes:

- A seven-axis Franka Panda arm with a two-finger gripper
- Deterministic PyBullet simulation at 240 Hz
- Joint states, simulation clock, end-effector pose and path, contacts, and
  object markers
- External and wrist RGB cameras with matching camera intrinsics and TF
- Camera-guided red/blue sorting with physical grasp checks and annotated images
- A standard `FollowJointTrajectory` action
- Services for world reset, pause, inverse kinematics, and box spawning
- RViz2 integration, predefined trajectories, an IK demo, and automated tests
- A headless mode suitable for WSL2 and continuous integration

## Architecture

```text
demo / IK client
        |
        v
FollowJointTrajectory action
        |
        v
trajectory_controller ----> /arm_controller/joint_trajectory
                                      |
                                      v
                               PyBullet simulator
                                  |         |
                         /joint_states    /clock
                                  |         |
                                  v         v
                         robot_state_publisher
                                  |
                                  v
                             TF + RViz2
```

The workspace contains five ROS 2 packages:

| Package | Responsibility |
| --- | --- |
| `pybullet_arm_description` | Xacro/URDF model and RViz2 configuration |
| `pybullet_arm_interfaces` | Custom messages and services |
| `pybullet_arm_sim` | PyBullet world and ROS 2 adapter |
| `pybullet_arm_control` | Trajectory action server and demo clients |
| `pybullet_arm_bringup` | Parameters and launch orchestration |

See the [architecture notes](docs/architecture.md) and
[interface reference](docs/interfaces.md) for details.

## Quick Start

Initialize every new terminal:

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/ros2_pybullet/bin/activate
source ~/ros2_ws/install/setup.bash
```

Start the complete simulation with RViz2:

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py
```

In another initialized terminal, run the predefined trajectory:

```bash
ros2 run pybullet_arm_control demo_sequence
```

Run the IK target demo:

```bash
ros2 run pybullet_arm_control ik_demo --ros-args \
  -p target_x:=0.42 -p target_y:=0.12 -p target_z:=0.24
```

## Automatic Color Sorting

After initializing the terminal as above, stop old simulations and run:

```bash
ros2 launch pybullet_arm_bringup sorting.launch.py
```

Panda sorts a batch of four blocks (two red, two blue) into matching colored
zones. The launch opens PyBullet and a camera preview showing detections,
task state and counts. A batch takes roughly two minutes on the tested host.
The arm parks after completion; Ctrl+C stops the launch. Relaunch for a new batch,
optionally adding `seed:=7` to change the initial layout.

The controller uses synchronized external RGB, camera calibration and TF:
HSV segmentation → planar localization → grasp → transfer → visual verification.
Both fingers must contact the same physical part. Failed grasps are retried once;
lost contact during transport, missing images or unverified placement stop the task.
No object poses from simulator markers are used by this controller.

This version assumes separated, upright 6×6×7 cm blocks on a flat surface,
known block height and an unobstructed workspace. Destination pads are visual
zones, not walled bins. It is a four-part batch demo, not a conveyor controller.
Do not run another motion client or reset/pause the simulation during sorting.

```bash
# Headless batch; the status node stays alive after completion.
ros2 launch pybullet_arm_bringup sorting.launch.py pybullet_gui:=false camera_viewer:=false
# In another initialized terminal:
ros2 topic echo /sorting/status
python ~/ros2_ws/src/ros2_pybullet_arm/tools/sorting_smoke_test.py
```

The smoke test independently checks the final physical object positions.
`/sorting/annotated_image` carries the detection overlay. The standalone
`auto_sort` node supports `max_retries:=1`, `expected_parts:=4` and
`keep_alive:=false` via ROS parameters; use it only with an existing sorting scene.

Camera rendering runs in two bounded background processes, separate from physics.
The default sorting view is 320×320 with a 15 Hz sampling target (actual FPS depends
on CPU load). Built-in PyBullet image previews are disabled. Motion uses 120 Hz
commands and smooth quintic ramps; grasp/verification pauses remain intentional.
Override `camera_width`, `camera_height` and `camera_hz` in the sorting launch
to trade image detail for frame rate. See [timing measurements](docs/rendering.md).

## Pick and Place

Stop any old simulation process, then start the native PyBullet window. RViz2
can also be enabled if desired.

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py rviz:=false pybullet_gui:=true
```

Run the task in another initialized terminal:

```bash
ros2 run pybullet_arm_control pick_place
```

The program reads the simulated pose of `training_cube`, then opens the
gripper, approaches, descends, grasps, lifts, transfers, lowers, releases, and
retreats. The default placement center is `(0.43, 0.25, 0.035) m`.

The cube is held through physical contact and friction. The program verifies
two-finger contact, lift height, and final object position, and returns a
non-zero exit code on failure. The 2.5 cm acceptance tolerance is a simulation
test criterion, not a claim about real-hardware accuracy.

Move the cube back to its initial position:

```bash
ros2 run pybullet_arm_control pick_place --ros-args -p place_y:=0.0
```

This demo is designed for the standard 6 × 6 × 7 cm training cube on an
obstacle-free ground plane. It uses simulation ground truth and does not yet
include camera-based perception or obstacle-aware planning. Do not run another
trajectory client at the same time.

Reset the simulation to start over. Resetting also removes other spawned
objects.

```bash
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
```

## Cameras

Two cameras are enabled by default: an external camera fixed in the world
and a wrist camera attached to `panda_hand`. Both publish 224 × 224 `rgb8`
images, matching `CameraInfo`, and optical-frame TF. Rendering uses PyBullet's
CPU TinyRenderer and also works without a PyBullet window.

Start the PyBullet window and a separate side-by-side camera preview:

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=true camera_viewer:=true
```

If the simulation is already running, open only the preview in another
initialized terminal:

```bash
ros2 run pybullet_arm_sim camera_viewer
```

Press **Q** or **Esc** in the preview to close it. The preview is optional;
launching the simulation alone does not open this window. It uses the system
OpenCV package inherited by the virtual environment.

| Camera | Image topic | Calibration topic | Optical frame |
| --- | --- | --- | --- |
| External | `/cameras/external/image_raw` | `/cameras/external/camera_info` | `external_camera_optical_frame` |
| Wrist | `/cameras/wrist/image_raw` | `/cameras/wrist/camera_info` | `wrist_camera_optical_frame` |

Optical frames use x-right, y-down, z-forward axes. Each image and its
`CameraInfo` share the simulation timestamp of the joint states published
with that camera sample. The default rate is 10 Hz in simulation time;
actual wall-clock frame rate depends on rendering speed. Pausing the
simulation stops new camera frames.

The image and calibration topics use sensor-data QoS (best effort).
Select **Best Effort** when adding an Image display in RViz2.

Configure resolution and sampling frequency at launch:

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  camera_width:=224 camera_height:=224 camera_hz:=10.0
```

Use `cameras:=false` to disable camera rendering, image/calibration topics,
and camera TF. Increasing image size or frame rate adds CPU work and can
reduce delivered camera FPS. Cameras provide RGB observations; `auto_sort` performs
color-based detection, while `pick_place` continues to use simulated object
poses. Depth and model inference are not implemented.

## Display Modes

Display only the robot model with interactive joint sliders:

```bash
ros2 launch pybullet_arm_bringup display.launch.py
```

Run without graphical windows:

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=false camera_viewer:=false
```

Camera topics remain available in headless mode unless `cameras:=false` is
also set.

## Build

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/ros2_pybullet/bin/activate
cd ~/ros2_ws
rosdep install --from-paths src/ros2_pybullet_arm --ignore-src -r -y
python -m colcon build --symlink-install --packages-up-to pybullet_arm_bringup
source install/setup.bash
```

Additional Python dependencies are pinned in `requirements.txt`. When
creating the virtual environment for the first time, inherit the ROS 2 system
packages:

```bash
python3 -m venv --system-site-packages ~/venvs/ros2_pybullet
source ~/venvs/ros2_pybullet/bin/activate
python -m pip install -r ~/ros2_ws/src/ros2_pybullet_arm/requirements.txt
```

## Verification

Run the unit tests:

```bash
cd ~/ros2_ws
python -m colcon test --packages-select \
  pybullet_arm_description pybullet_arm_sim pybullet_arm_control
python -m colcon test-result --verbose
```

After starting the headless simulation, run the end-to-end smoke test in
another terminal:

```bash
python ~/ros2_ws/src/ros2_pybullet_arm/tools/smoke_test.py
```

The script verifies `/clock`, `/joint_states`, world reset, object spawning,
IK, and the trajectory action.

With cameras enabled, check synchronized images, calibration, TF, and joint
states (optionally saving actual frames):

```bash
python ~/ros2_ws/src/ros2_pybullet_arm/tools/camera_smoke_test.py \
  --output-dir /tmp/panda_camera_frames
```

## Useful Inspection Commands

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 action info /arm_controller/follow_joint_trajectory
ros2 service list
```

Pause and resume:

```bash
ros2 service call /simulation/set_paused std_srvs/srv/SetBool "{data: true}"
ros2 service call /simulation/set_paused std_srvs/srv/SetBool "{data: false}"
```

The gripper command uses the total opening width, from `0.0` to `0.08`
meters:

```bash
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 0.04}"
```

## Learning Path

Do not begin by reading the entire project line by line. Follow the
[project-based learning path](docs/learning_path.md), change one module at a
time, and use the acceptance commands after each change. A useful order is:
topics and launch files, trajectory control, then the PyBullet world and IK.

## Current Scope

- The controller is an educational ROS 2 action server, not a
  `ros2_control` hardware interface.
- IK uses PyBullet's numerical solver and currently has no collision
  constraints or reachability residual check.
- Feedback-checked pick-and-place is implemented for the standard training
  cube; arbitrary-object grasping and obstacle-aware planning are not.
- Color sorting uses external RGB with a known-height planar approximation;
  it does not estimate arbitrary 3D object poses.
- The PyBullet node publishes simulation time, while the controller
  intentionally uses monotonic wall time so pausing the simulation does not
  deadlock an action.

These boundaries are natural extension points for future work.

## License

Project code is licensed under MIT. The Panda model and meshes are licensed
under Apache-2.0; see `pybullet_arm_description/vendor/NOTICE.md`. The model's
inertial parameters are simplified for education and are not calibrated
hardware values.
