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

Display only the robot model with interactive joint sliders:

```bash
ros2 launch pybullet_arm_bringup display.launch.py
```

Run without graphical windows:

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=false
```

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
- The PyBullet node publishes simulation time, while the controller
  intentionally uses monotonic wall time so pausing the simulation does not
  deadlock an action.

These boundaries are natural extension points for future work.

## License

Project code is licensed under MIT. The Panda model and meshes are licensed
under Apache-2.0; see `pybullet_arm_description/vendor/NOTICE.md`. The model's
inertial parameters are simplified for education and are not calibrated
hardware values.
