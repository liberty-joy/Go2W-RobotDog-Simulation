# RobotDog — Go2-W 学习与研究项目

**先读：[00_复现总览与文档索引](docs/00_复现总览与文档索引.md)**。包含本机命令行复现、文档编号规则、工作区职责与最新阶段入口。

当前传感阶段状态：[progress.md](docs/sensor_pipeline/progress.md)；本次验收：[阶段0环境记录](docs/sensor_pipeline/00_environment.md)。下方训练记录为历史结果，不能代替本次执行。

本地工作区：`/home/joy/桌面/RobotDog`。Isaac Sim / Isaac Lab 环境位于 `/home/joy/robotics`。

- [键盘控制与前置相机](docs/Go2W键盘与相机.md)：`./scripts/run_go2w.sh`
- [ROS 2 接入与 Go2W 轮腿/迈步训练](docs/Go2W_ROS2与训练.md)：`./scripts/run_go2w_ros2.sh`、`./scripts/train_go2w.sh`
- [停车、摩擦/扰动/坡面训练与轮腿切换](docs/Go2W鲁棒训练与模式切换.md)：`./scripts/run_go2w_dual.sh`；静止后 M 切换。
- [从零到论文路线](docs/从零到论文路线.md)：研究与学习计划，文中的旧状态需结合当前验收记录阅读。

当前具备轮腿控制、相机、ROS 2 标准消息接口、摩擦/小扰动/±3° 坡面训练与评估、实验性纯腿控制及静止模式切换。轮腿 5 种条件共 50 项分项检查通过；纯腿步态质量须结合最新验收报告阅读。尚未接入实机、SLAM、Nav2 或复杂地形。

模型源文件在 `models/go2w`，导入模型在 `sim/assets/go2w`；实验日志、图像和策略权重在 `outputs`。

- [视觉雷达仿真实施提示词](docs/视觉雷达仿真实施提示词.md)：复制总提示词，再依次执行阶段 0–10。
