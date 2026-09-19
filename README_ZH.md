# ROS 2 + PyBullet 机械臂实验平台

[中文](README_ZH.md) | [English](README.md)

这是一个用于项目式学习的 ROS 2 Jazzy 中型项目：同一份 Xacro 模型同时交给 RViz2 与 PyBullet，ROS 2 负责通信、控制和可视化，PyBullet 负责物理仿真。

当前版本包含：

- Franka Panda 七轴机械臂与双指夹爪模型；
- PyBullet 240 Hz 确定步长仿真；
- 关节状态、仿真时钟、末端位姿、轨迹、接触与物体 Marker；
- 外部与腕部 RGB 双相机，配套相机内参与 TF；
- 相机引导的红蓝自动分拣、物理夹持检查及识别画面；
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

## 自动颜色分拣

初始化终端环境（命令见下方）并停止旧仿真后，一条命令启动：

```bash
ros2 launch pybullet_arm_bringup sorting.launch.py
```

Panda 将四个方块（红、蓝各两个）逐个放进对应颜色区域。默认打开 PyBullet
窗口和相机预览，显示识别结果、任务状态与计数。本机测试一批约两分钟。
完成后机械臂停靠，Ctrl+C 结束；重新启动生成新一批，添加 `seed:=7` 可改变初始布局。

流程：同步外部 RGB、内参与 TF → HSV 颜色分割 → 平面定位 → 夹取 →
搬运 → 相机验收。控制程序不读取仿真 Marker 的物体位姿。
双指必须接触同一个物体；夹取失败默认重试一次，搬运中失去接触、图像缺失
或放置验收失败则停止并报告错误。

当前适用范围：平面上分开放置、直立的 6×6×7 cm 方块，已知高度、无障碍工作区。
目标区域是可视化垫板，不是带围墙的料箱；每批四件，暂不涉及传送带。
分拣期间不要运行其他运动客户端，也不要重置或暂停仿真。

```bash
# 无窗口运行；完成后状态节点仍保持运行。
ros2 launch pybullet_arm_bringup sorting.launch.py pybullet_gui:=false camera_viewer:=false
# 在另一个完成环境初始化的终端查看状态、验收：
ros2 topic echo /sorting/status
python ~/ros2_ws/src/ros2_pybullet_arm/tools/sorting_smoke_test.py
```

验收脚本独立核对四个物体的实际落点。`/sorting/annotated_image` 发布识别叠加画面。
独立节点 `auto_sort` 提供 ROS 参数 `max_retries:=1`、`expected_parts:=4`、
`keep_alive:=false`，须配合已经运行的分拣场景使用。

相机由两个独立后台进程渲染，不占用物理循环；默认 320×320、目标采样率
15 Hz，实际帧率取决于 CPU。PyBullet 内置图像预览已关闭。
运动使用 120 Hz 指令与五次平滑曲线，抓取和视觉验收仍保留必要停顿。
可通过 `camera_width`、`camera_height`、`camera_hz` 调整画质与帧率，
实测数据见 [流畅度说明](docs/rendering.md)。

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

## 双相机

默认启用两台相机：固定在世界中的外部相机，以及随 `panda_hand` 运动的腕部相机。
两路均发布 224×224 的 `rgb8` 图像、匹配的 `CameraInfo` 和光学坐标系 TF。
图像由 PyBullet 的 CPU TinyRenderer 渲染，不依赖 PyBullet 图形窗口。

同时启动 PyBullet 窗口与独立的双画面相机预览：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=true camera_viewer:=true
```

如果仿真已经运行，在另一个已初始化环境的终端单独打开预览：

```bash
ros2 run pybullet_arm_sim camera_viewer
```

在预览窗口按 **Q** 或 **Esc** 关闭。预览是可选窗口，单独启动仿真不会自动打开它；
使用虚拟环境继承的系统 OpenCV。

| 相机 | 图像话题 | 内参话题 | 光学坐标系 |
| --- | --- | --- | --- |
| 外部 | `/cameras/external/image_raw` | `/cameras/external/camera_info` | `external_camera_optical_frame` |
| 腕部 | `/cameras/wrist/image_raw` | `/cameras/wrist/camera_info` | `wrist_camera_optical_frame` |

光学坐标系采用 x 向右、y 向下、z 向前的约定。每次采样的图像、`CameraInfo`
与采样时的关节状态使用相同仿真时间戳；图像异步渲染，会晚于关节状态到达。
默认目标为仿真时间 10 Hz 采样，实际墙钟帧率取决于渲染速度；暂停后不发布新帧。

图像与内参话题使用传感器 QoS（Best Effort）；在 RViz2 添加 Image 显示时，
将可靠性设置为 **Best Effort**。

启动时可调整分辨率和采样频率：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  camera_width:=224 camera_height:=224 camera_hz:=10.0
```

使用 `cameras:=false` 可关闭相机渲染、图像/内参话题与相机 TF。
提高分辨率或目标帧率会增加 CPU 开销，可能降低相机实际帧率。当前提供 RGB 观测，尚无深度图。
`auto_sort` 使用外部相机颜色识别；`pick_place` 仍然读取仿真物体位姿。

## 显示方式

只看模型并手动拖动关节滑块：

```bash
ros2 launch pybullet_arm_bringup display.launch.py
```

无图形界面运行：

```bash
ros2 launch pybullet_arm_bringup simulation.launch.py \
  rviz:=false pybullet_gui:=false camera_viewer:=false
```

无界面模式仍然发布相机话题；如需一并关闭，添加 `cameras:=false`。

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

启用相机后，可检查图像、内参、TF 与关节状态的同步，并保存实际帧：

```bash
python ~/ros2_ws/src/ros2_pybullet_arm/tools/camera_smoke_test.py \
  --output-dir /tmp/panda_camera_frames
```

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
- 颜色分拣使用外部 RGB 与已知高度平面近似，不估计任意物体的三维姿态；
- 仿真时钟由 PyBullet 节点发布，控制器故意使用墙上时间，避免暂停仿真时 Action 自锁。

这些边界正好对应后续可独立实现的升级任务。

## 许可证

项目代码采用 MIT；Panda 模型和网格采用 Apache-2.0，见描述包 vendor/NOTICE.md。模型惯性参数为教学简化值，不代表实机标定结果。
