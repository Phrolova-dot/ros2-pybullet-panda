# 架构说明

## 设计目标

项目将“机器人描述、物理世界、控制策略、ROS 接口、启动配置”分开，便于逐层替换。核心约束是 RViz2 与 PyBullet 必须读取同一份 Xacro，避免显示模型和碰撞模型漂移。

## 数据流

1. `pybullet_sim` 从 Xacro 生成临时 URDF，加载机械臂、地面和训练方块。
2. 仿真节点按 240 Hz 推进物理世界，按 60 Hz 发布 `/clock` 和状态。
3. `robot_state_publisher` 接收 `/joint_states`，结合 `robot_description` 发布 TF。
4. RViz2 使用 TF 显示机械臂，并订阅末端路径和物体 Marker。
5. `trajectory_controller` 接收标准 Action 目标，以 50 Hz 插值并发布关节目标。
6. 仿真节点将目标交给 PyBullet 位置控制器，反馈实际关节状态。

## 线程与时间

- PyBullet 世界只由 `pybullet_sim` 的执行线程访问，避免客户端状态竞争。
- 仿真时间由固定步长累计，暂停时 `/clock` 停止前进。
- `robot_state_publisher` 和 RViz2 使用仿真时间。
- 轨迹控制器使用单调墙上时间。即使仿真暂停，取消请求仍能被处理。
- Action 服务器使用 `MultiThreadedExecutor` 和可重入回调组，使关节反馈、取消和执行回调能并行处理。

## 模型约定

- 世界坐标与机器人根坐标均为 `base_link`。
- 机械臂受控关节顺序固定为：`panda_joint1` 到 `panda_joint7`。
- 末端计算链接为 `tool_tip`。
- 两个夹爪关节由一个总开度命令控制，不进入手臂轨迹 Action。

## 关键扩展点

### ros2_control

保留现有描述包和接口包，实现 `hardware_interface::SystemInterface` 或仿真专用硬件插件，再用 `joint_trajectory_controller` 替换教学控制器。

### MoveIt 2

增加 SRDF、运动学参数、规划管线和 PlanningScene。此时 IK 服务可以保留作对照，但任务规划应通过 MoveIt 2 完成。

### 自动抓取

新增行为树或状态机节点，顺序执行“观察目标—预抓取—下探—闭合—抬升—放置”。接触话题可作为夹取成功判据之一。

### 视觉感知

在 PyBullet 中增加相机，发布 `sensor_msgs/Image`、`CameraInfo` 和深度图；再实现目标检测和坐标变换。

## 故障隔离

| 现象 | 优先检查 |
| --- | --- |
| RViz2 显示 `No transform` | `/clock`、`/joint_states`、`robot_state_publisher` |
| Action 被拒绝 | 关节名顺序、时间递增、关节限制、状态是否已到达 |
| 机械臂不动 | `/arm_controller/joint_trajectory` 是否有数据 |
| IK 返回异常 | 目标是否可达且满足关节限制、链接名是否为 `tool_tip` |
| Python 找不到 `rclpy` | 先 source ROS，再激活带 `--system-site-packages` 的虚拟环境 |
