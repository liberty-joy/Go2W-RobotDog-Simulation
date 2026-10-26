# Go2W ROS 2 与运动训练

> 文档导航：编号与复现入口见 [00_复现总览与文档索引](00_复现总览与文档索引.md)。本文保留历史记录或规划，不代表本次复验结果；本次结果以 [阶段0环境验收](sensor_pipeline/00_environment.md) 为准。

本工程使用 Isaac Sim 6.2、Isaac Lab 3.0、RSL-RL 5.5.1、ROS 2 Jazzy。原键盘演示和相机继续可用；ROS 2、学习策略均按参数启用。

## 1. 启动 ROS 2 仿真

关闭不用的 Isaac 窗口后运行：

```bash
cd /home/joy/桌面/RobotDog
./scripts/run_go2w_ros2.sh
```

打开另一个终端：

```bash
cd /home/joy/桌面/RobotDog
source scripts/ros2/env.sh
ros2 topic list
ros2 topic pub --rate 10 /go2w/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.12}, angular: {z: 0.0}}'
```

Ctrl+C 停止发布，0.5 秒未收到新指令后轮速目标归零。键盘 WASD 在按住时优先于 ROS 指令；松开后恢复 ROS 控制。按住空格可制动，松开后允许新 ROS 指令。R 复位，C 切换视角，P 保存图像。当前速度指令只使用 linear.x、angular.z；分别限制为 ±0.3m/s、±0.6rad/s。直接轮速映射尚未完成速度标定，不保证实际速度精确跟随；训练任务按物理反馈计算速度跟踪奖励。

默认 `ROS_DOMAIN_ID=42`、`rmw_fastrtps_cpp`。在另一台设备上使用同样配置及可互通的网络。要换域，在启动两个终端前设置同一个 `ROS_DOMAIN_ID`。不需要 roscore。

仿真启动器固定使用 Isaac Sim 自带的 Jazzy 运行库；外部终端优先使用 `/opt/ros/jazzy`。系统安装尚未完成时，外部终端自动使用 Isaac Sim 自带的 Jazzy rclpy、标准消息及 ROS CLI。自带环境已经可以跨进程通信；它不替代原生 RViz/colcon 的完整系统安装。CLI 包装器为 `scripts/ros2/ros2`，Python 节点启动器为 `scripts/ros2/python.sh`。

| 话题 | 类型 | 内容 |
|---|---|---|
| `/go2w/cmd_vel` | geometry_msgs/Twist | 前进速度与转向速度输入 |
| `/clock` | rosgraph_msgs/Clock | 仿真时钟，暂停时停止推进，复位时可回退 |
| `/go2w/odom` | nav_msgs/Odometry | 仿真真值位姿；twist 位于 base_link 坐标系 |
| `/go2w/joint_states` | sensor_msgs/JointState | 12 个腿关节及 4 个轮关节的位置和速度 |
| `/tf` | tf2_msgs/TFMessage | odom → base_link |
| `/tf_static` | tf2_msgs/TFMessage | base_link → front_camera_link → front_camera_optical_frame |
| `/go2w/front_camera/image_raw` | sensor_msgs/Image | 640×480 RGB8，目标 15Hz，实际受渲染性能限制 |
| `/go2w/front_camera/camera_info` | sensor_msgs/CameraInfo | 与当前 80° 水平视野一致的内参 |

状态、时钟和动态 TF 的目标发布频率为 30Hz。图像/内参/关节状态使用 sensor_data QoS（best effort），订阅时选相应 QoS。仿真坐标为 X 前、Y 左、Z 上；相机 optical 为 X 右、Y 下、Z 前。相机相对机身俯角 5°，位置 (0.32,0,0.08)m。里程计是仿真真值，不是已实现的轮速/IMU 定位。

```bash
ros2 topic echo /go2w/odom --once
ros2 topic hz /go2w/front_camera/image_raw
ros2 topic echo /go2w/joint_states --qos-reliability best_effort --once
```

原生 Jazzy Desktop 安装完成后可运行 `./scripts/ros2/rviz.sh`，已配好图像、TF 和真值里程计显示。手动设置 RViz2 时，Fixed Frame 设 `odom`；添加 Image，选择上述 image_raw，并将 Reliability 设为 Best Effort。ROS 节点需要使用仿真时间时设置 `use_sim_time=true`。

跨进程自动验收（会另启无窗口仿真，请先关闭正在运行的演示）：

```bash
./scripts/ros2/test_bridge.sh
```

检查外部速度指令产生移动、停止发布后超时制动、RGB 长度/编码、内参、16 个关节、TF、仿真时钟。结果为 `outputs/ros2/verification.json`。

## 2. 第一阶段：轮腿协同 PPO

```bash
cd /home/joy/桌面/RobotDog
./scripts/train_go2w.sh --headless --mode wheel --num_envs 256 --iterations 1500
```

启动器使用独立的 `/home/joy/robotics/isaaclab-env`，不升级现有仿真环境。使用本地 Go2W USD 的派生覆盖层，保留原始模型。PhysX 120Hz，策略 30Hz；平地并行环境、随机速度指令和轻微初始关节扰动。

动作：12 维腿部位置残差（±0.2rad）+ 4 维轮速残差（±3rad/s）。轮速基线来自目标前进/转向指令，PPO 学习修正腿姿和轮速；这是残差控制训练。保留原有关节力矩与速度上限。观测包含机身速度、角速度、重力方向、目标速度、腿部关节角、全部关节速度、上一步动作和相位。奖励包括速度跟踪、水平姿态、机身高度和动作平滑；低机身、明显倾倒或越界时重置。

当前阶段不含复杂地形、接触传感器奖励、推力恢复、质量/摩擦/延迟随机化，也不表示具备实机部署能力。50 轮短测权重用于验证训练链路，另已运行 256 环境 × 300 轮的初始平地训练（`outputs/training/wheel_flat_initial/`），不能据此判断策略已收敛。完整训练的效果应另做长回合、多随机种子评估。

输出目录默认 `outputs/training/wheel_时间/`：

- `model_final.pt`：含优化器等训练状态的 checkpoint。
- `policy_jit.pt` 与 `policy_jit.json`：包含观测归一化的导出策略及关节顺序契约。
- `run.json`、`events.out.tfevents.*`：运行配置与 TensorBoard 日志。

继续训练：

```bash
./scripts/train_go2w.sh --headless --mode wheel --num_envs 256 --iterations 1500 \
  --checkpoint /完整路径/model_final.pt
```

无窗口回放已有权重（去掉 --headless 可打开图形训练回放，已通过复测；也可以用后面的原场景 --policy 命令查看轮腿策略）：

```bash
./scripts/train_go2w.sh --headless --mode wheel --num_envs 4 --play \
  --checkpoint outputs/training/wheel_flat_initial/model_final.pt --steps 1800
```

在原键盘/ROS 场景启用导出的轮腿策略：

```bash
./scripts/run_go2w_ros2.sh --policy outputs/training/wheel_flat_initial/policy_jit.pt
```

不传 `--policy` 时继续使用已验证的固定站姿 + 轮速控制。启用策略后，零速度命令仍会强制轮子制动并恢复稳定站姿。当前单机场景只支持 wheel 策略；walk 策略在训练回放器中查看。

TensorBoard：

```bash
/home/joy/robotics/isaaclab-env/bin/tensorboard --logdir /home/joy/桌面/RobotDog/outputs/training --port 6006
```

浏览器打开 http://localhost:6006，关注速度误差、机身高度、直立度、回合奖励及存活时间。训练时优先关闭额外 Isaac 进程，避免重复启动 Kit 竞争资源；并行训练使用一个进程里的 num_envs。

## 3. 第二阶段：锁轮四足迈步

```bash
./scripts/train_go2w.sh --headless --mode walk --num_envs 256 --iterations 3000
```

使用 12 维腿关节残差动作，轮子通过位置驱动保持零转角。以对角腿交替的相位参考作为起点，叠加足轮中心抬高奖励和速度/姿态奖励。它是独立任务，不把 wheel 策略当作已学会迈步；应独立训练和验收。当前实现仍是平地起步配置，未完成步态质量评估。

回放时选择 `--mode walk --play --checkpoint <walk 目录中的 model_final.pt>`。wheel 与 walk 的动作/观测维度不同，不可混用 checkpoint。

快速动力学检查：

```bash
./scripts/train_go2w.sh --headless --mode wheel --smoke --num_envs 4 --steps 240
./scripts/train_go2w.sh --headless --mode walk --smoke --num_envs 4 --steps 240
```

## 4. 资料与实现边界

安装方案参照 [ROS 2 Jazzy 官方说明](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) 和 [Isaac Sim 6.2 ROS 2 配置](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/installation/install_ros.html)。训练接口以本机 Isaac Lab 3.0 源码与 RSL-RL 5.5.1 为准。

这是本地编写的 Go2W 入门训练任务，并非下载的宇树成熟步态策略。宇树 [Unitree RL Lab](https://github.com/unitreerobotics/unitree_rl_lab) 可以作为进一步对照，但普通 Go2 环境不能直接当作已适配 Go2W 的轮腿控制。


固定指令评估（静止 → 前进 → 转向 → 停车，记录速度误差、直立度和提前重置次数）：

```bash
./scripts/train_go2w.sh --headless --mode wheel --num_envs 8 --evaluate \
  --checkpoint outputs/training/wheel_flat_initial/model_final.pt --steps 600
```

验收记录见 `docs/Go2W_ROS2训练验收.md`。
