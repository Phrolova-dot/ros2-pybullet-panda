# ROS 2 接口

## 话题

| 名称 | 类型 | 方向 | 含义 |
| --- | --- | --- | --- |
| `/joint_states` | `sensor_msgs/msg/JointState` | 仿真发布 | 九个可动关节（七轴与两指）的实际状态 |
| `/clock` | `rosgraph_msgs/msg/Clock` | 仿真发布 | 固定步长仿真时间 |
| `/arm_controller/joint_trajectory` | `trajectory_msgs/msg/JointTrajectory` | 仿真订阅 | 控制器产生的即时关节目标 |
| `/gripper/command` | `std_msgs/msg/Float64` | 仿真订阅 | 两指总开度，单位米 |
| `/arm/end_effector_pose` | `geometry_msgs/msg/PoseStamped` | 仿真发布 | `tool_tip` 位姿 |
| `/arm/end_effector_path` | `nav_msgs/msg/Path` | 仿真发布 | 最近 500 个末端位姿 |
| `/simulation/contacts` | `pybullet_arm_interfaces/msg/ContactStateArray` | 仿真发布 | 机械臂与已登记物体的接触 |
| `/simulation/objects` | `visualization_msgs/msg/MarkerArray` | 仿真发布 | RViz2 中的仿真物体 |

物体 Marker 的 `text` 字段携带物体名称，`pick_place` 据此识别
`training_cube`；地面 Marker 使用独立命名空间。

## 搬运程序

`ros2 run pybullet_arm_control pick_place` 调用 IK 与轨迹 Action，并发布夹爪命令。
订阅物体 Marker、两指接触、末端位姿和关节反馈进行阶段验收。
参数 `place_x=0.43`、`place_y=0.25` 为放置中心的地面坐标（米）。
程序只支持标准训练方块和无障碍桌面区域，不执行视觉识别或避障规划。

## Action

`/arm_controller/follow_joint_trajectory`

- 类型：`control_msgs/action/FollowJointTrajectory`
- 关节名必须与七个手臂关节的固定顺序完全一致；
- 每个 `time_from_start` 必须严格递增；
- 当前插值方式为分段线性插值；
- 成功条件是最终最大关节误差不超过配置阈值。

## 服务

| 名称 | 类型 | 作用 |
| --- | --- | --- |
| `/simulation/reset` | `std_srvs/srv/Trigger` | 重建世界并回到 Home 位姿 |
| `/simulation/set_paused` | `std_srvs/srv/SetBool` | 暂停或继续物理步进 |
| `/arm/solve_ik` | `pybullet_arm_interfaces/srv/SolveIK` | 求指定末端目标的七关节数值解 |
| `/simulation/spawn_box` | `pybullet_arm_interfaces/srv/SpawnBox` | 创建带质量、尺寸和颜色的方块 |

## 参数

### `pybullet_sim`

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `robot_description_path` | 必填 | Xacro 文件路径 |
| `pybullet_gui` | `false` | 是否打开 PyBullet 原生窗口 |
| `physics_hz` | `240.0` | 物理步进频率 |
| `publish_hz` | `60.0` | ROS 状态发布频率 |
| `seed` | `42` | 预留的确定性随机种子 |
| `spawn_default_box` | `true` | 是否创建训练方块 |

### `trajectory_controller`

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `command_hz` | `50.0` | 插值命令频率 |
| `goal_tolerance` | `0.05` | 最终最大关节误差，单位弧度 |
| `settle_timeout` | `2.0` | 到点后的最大等待时间，单位秒 |
