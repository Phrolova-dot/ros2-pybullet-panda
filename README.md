# ROS 2 + PyBullet 机械臂实验平台

[中文](README.md) | [English](README_EN.md)

这是一个用于项目式学习的 ROS 2 Jazzy 中型项目：同一份 Xacro 模型同时交给 RViz2 与 PyBullet，ROS 2 负责通信、控制和可视化，PyBullet 负责物理仿真。

当前版本包含：

- Franka Panda 七轴机械臂与双指夹爪模型；
- PyBullet 240 Hz 确定步长仿真；
- 关节状态、仿真时钟、末端位姿、轨迹、接触与物体 Marker；
- 标准 `FollowJointTrajectory` Action；
- 世界重置、暂停、逆运动学和生成方块服务；
- RViz2、预设轨迹、IK 演示及自动化测试；
- 无界面运行模式，适合 WSL2 和持续集成。

## 系统结构

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

五个 ROS 2 包各自承担单一职责：

| 包 | 作用 |
| --- | --- |
| `pybullet_arm_description` | Xacro/URDF 与 RViz2 配置 |
| `pybullet_arm_interfaces` | 自定义消息与服务 |
| `pybullet_arm_sim` | PyBullet 世界及 ROS 2 适配层 |
| `pybullet_arm_control` | 轨迹 Action 服务器与示例客户端 |
| `pybullet_arm_bringup` | 参数和 Launch 编排 |

详细说明见 [架构文档](docs/architecture.md) 和 [接口清单](docs/interfaces.md)。

## 快速启动

每个新终端先执行：

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/ros2_pybullet/bin/activate
source ~/ros2_ws/install/setup.bash
```

启动完整仿真与 RViz2：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py
```

启动后，在另一个已完成环境初始化的终端运行预设轨迹：

```bash
ros2 run pybullet_arm_control demo_sequence
```

运行 IK 目标点演示：

```bash
ros2 run pybullet_arm_control ik_demo --ros-args \
  -p target_x:=0.42 -p target_y:=0.12 -p target_z:=0.24
```

## 夹取搬运

更新后先关闭旧仿真，再启动 PyBullet 窗口（也可同时启用 RViz）：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py rviz:=false pybullet_gui:=true
```

在另一个完成环境初始化的终端运行：

```bash
ros2 run pybullet_arm_control pick_place
```

程序读取 training_cube 的仿真位姿，依次执行张开、靠近、下探、夹持、
抬升、搬运、下降、松开和退回。默认放置中心为 (0.43, 0.25, 0.035) m。
使用接触和摩擦进行物理夹持；检查两指接触、离地高度和最终物体位置，
失败返回非零退出码。验收容差为 2.5 cm，不是实机精度声明。

搬回初始位置：

```bash
ros2 run pybullet_arm_control pick_place --ros-args -p place_y:=0.0
```

此演示针对地面上的标准 6×6×7 cm 训练方块和无障碍场景，使用仿真真值，
尚未接入相机识别或避障规划。不要同时运行其他轨迹客户端。
重置仿真可重新开始；重置会移除另外生成的物体：

```bash
ros2 service call /simulation/reset std_srvs/srv/Trigger '{}'
```

只看模型并手动拖动关节滑块：

```bash
ros2 launch pybullet_arm_bringup display.launch.py
```

无图形界面运行：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=false
```

## 构建

```bash
source /opt/ros/jazzy/setup.bash
source ~/venvs/ros2_pybullet/bin/activate
cd ~/ros2_ws
rosdep install --from-paths src/ros2_pybullet_arm --ignore-src -r -y
python -m colcon build --symlink-install --packages-up-to pybullet_arm_bringup
source install/setup.bash
```

Python 额外依赖固定在 `requirements.txt`。首次创建虚拟环境时建议继承 ROS 2 的系统包：

```bash
python3 -m venv --system-site-packages ~/venvs/ros2_pybullet
source ~/venvs/ros2_pybullet/bin/activate
python -m pip install -r ~/ros2_ws/src/ros2_pybullet_arm/requirements.txt
```

## 验收

单元测试：

```bash
cd ~/ros2_ws
python -m colcon test --packages-select \
  pybullet_arm_description pybullet_arm_sim pybullet_arm_control
python -m colcon test-result --verbose
```

启动无界面仿真后，在另一个终端执行端到端验收：

```bash
python ~/ros2_ws/src/ros2_pybullet_arm/tools/smoke_test.py
```

脚本会验证 `/clock`、`/joint_states`、世界重置、生成物体、IK 和轨迹 Action。

## 常用观察命令

```bash
ros2 node list
ros2 topic list
ros2 topic echo /joint_states --once
ros2 topic hz /joint_states
ros2 action info /arm_controller/follow_joint_trajectory
ros2 service list
```

暂停与继续：

```bash
ros2 service call /simulation/set_paused std_srvs/srv/SetBool "{data: true}"
ros2 service call /simulation/set_paused std_srvs/srv/SetBool "{data: false}"
```

夹爪开度使用总宽度，范围为 `0.0` 到 `0.08` 米：

```bash
ros2 topic pub --once /gripper/command std_msgs/msg/Float64 "{data: 0.04}"
```

## 学习方法

不要先逐行读完整项目。按 [项目式学习路线](docs/learning_path.md) 的顺序，每次只改一个模块并用验收命令确认结果。推荐先从话题和 Launch 入手，再读轨迹控制器，最后进入 PyBullet 世界与逆运动学。

## 当前边界

- 控制器是教学用 ROS 2 Action 服务器，不是 `ros2_control` 硬件接口；
- IK 使用 PyBullet 数值解，目前不做碰撞约束和可达性残差判定；
- 已实现标准训练方块的反馈检查搬运流程，尚未实现任意物体抓取和避障规划；
- 仿真时钟由 PyBullet 节点发布，控制器故意使用墙上时间，避免暂停仿真时 Action 自锁。

这些边界正好对应后续可独立实现的升级任务。

## 许可证

项目代码采用 MIT；Panda 模型和网格采用 Apache-2.0，见描述包 vendor/NOTICE.md。模型惯性参数为教学简化值，不代表实机标定结果。
