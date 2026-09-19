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
7. 双相机默认按仿真时间 10 Hz 渲染，发布 RGB 图像、内参与相机 TF。
   外部相机观察整个工作区，腕部相机随 `panda_hand` 运动。
8. 可选的 `camera_viewer` 节点订阅两路图像并并排显示；后续感知或策略节点
   可直接订阅相同话题。

## 线程与时间

- PyBullet 世界只由 `pybullet_sim` 的执行线程访问，避免客户端状态竞争。
- 仿真时间由固定步长累计，暂停时 `/clock` 停止前进。
- `robot_state_publisher` 和 RViz2 使用仿真时间。
- 轨迹控制器使用单调墙上时间。即使仿真暂停，取消请求仍能被处理。
- Action 服务器使用 `MultiThreadedExecutor` 和可重入回调组，使关节反馈、取消和执行回调能并行处理。
- 同次相机采样的图像、内参与关节状态共享仿真时间戳；暂停时不产生新图像。
- 相机使用 CPU TinyRenderer，支持无界面渲染。渲染在仿真线程中执行，
  分辨率和采样频率会影响墙钟下的仿真速度；10 Hz 是仿真时间采样率。

## 模型约定

- 世界坐标与机器人根坐标均为 `base_link`。
- 机械臂受控关节顺序固定为：`panda_joint1` 到 `panda_joint7`。
- 末端计算链接为 `tool_tip`。
- 两个夹爪关节由一个总开度命令控制，不进入手臂轨迹 Action。
- 相机光学坐标系采用 x 向右、y 向下、z 向前，内参对应实际图像尺寸与投影。

## 相机模块

默认输出两路 224×224 的 `rgb8` 图像：固定外部视角和腕部视角。
每路使用独立的 `/cameras/<external|wrist>/image_raw` 与 `camera_info` 话题。
相机数据发布与显示窗口分离：PyBullet 原生窗口观察物理世界，RViz2 用于 ROS
状态调试，OpenCV 预览显示两台相机实际产生的图像。

Launch 参数 `cameras` 控制相机数据与 TF 发布，`camera_viewer` 控制独立预览。
`camera_width`、`camera_height` 和 `camera_hz` 配置图像大小与采样频率。
相机模块暂不产生深度图，也不执行视觉识别或 VLA 推理。

## 关键扩展点

### ros2_control

保留现有描述包和接口包，实现 `hardware_interface::SystemInterface` 或仿真专用硬件插件，再用 `joint_trajectory_controller` 替换教学控制器。

### MoveIt 2

增加 SRDF、运动学参数、规划管线和 PlanningScene。此时 IK 服务可以保留作对照，但任务规划应通过 MoveIt 2 完成。

### 自动抓取

现有 `pick_place` 使用仿真物体位姿顺序执行靠近、夹持、抬升、搬运和放置，
并检查双指接触、抬升高度与最终位置。后续可用视觉估计替换目标位姿来源，
或将流程组织为行为树；相机接入本身不改变搬运决策。

### 视觉感知

RGB 图像、`CameraInfo` 和相机 TF 已具备。下一步可增加深度图、目标检测与
坐标变换，或将同步图像和关节状态发送给视觉语言动作模型。

## 故障隔离

| 现象 | 优先检查 |
| --- | --- |
| RViz2 显示 `No transform` | `/clock`、`/joint_states`、`robot_state_publisher` |
| Action 被拒绝 | 关节名顺序、时间递增、关节限制、状态是否已到达 |
| 机械臂不动 | `/arm_controller/joint_trajectory` 是否有数据 |
| IK 返回异常 | 目标是否可达且满足关节限制、链接名是否为 `tool_tip` |
| Python 找不到 `rclpy` | 先 source ROS，再激活带 `--system-site-packages` 的虚拟环境 |
| 相机预览没有画面 | `cameras:=true`、图像话题是否存在、仿真是否暂停 |
| 相机帧率或仿真速度偏低 | 图像分辨率、`camera_hz` 与 CPU 渲染负载 |
