# Franka Panda model

Source: the franka_panda data directory bundled with PyBullet 3.2.7.
Upstream: https://github.com/bulletphysics/bullet3/tree/master/examples/pybullet/gym/pybullet_data/franka_panda

The model and meshes retain their Apache-2.0 license in franka_panda/LICENSE.txt.
The URDF mesh paths and collision/link6.mtl texture paths are adapted for ROS;
the wrapper adds base_link,
tool0 and tool_tip frame aliases. Original model geometry and joint limits are
preserved. Inertial values in this educational model are simplified and must not
be treated as calibrated hardware dynamics.
