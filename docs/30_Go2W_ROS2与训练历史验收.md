# Go2W ROS 2 与训练验收

> 文档导航：编号与复现入口见 [00_复现总览与文档索引](00_复现总览与文档索引.md)。本文保留历史记录或规划，不代表本次复验结果；本次结果以 [阶段0环境验收](sensor_pipeline/00_environment.md) 为准。

新增：轮腿继续训练和 ROS 联调已完成。旧分项评估的停车尖峰含有超时重置影响，修正后完整连续测试 10/10 通过；详情及后续鲁棒训练见 [Go2W鲁棒训练与模式切换](Go2W鲁棒训练与模式切换.md)。控制器保留零指令制动，不能把控制规则的效果归因为网络独立学会制动。

2026-10-26 更新：轮腿平地/摩擦/小扰动/±3° 坡面共 50/50 分项检查通过；模式切换与纯腿直行/停车 8/8 通过。选定的实验性纯腿策略在训练环境中低速跟踪 10/10 通过，但逐腿抬足仍未全部达标；不能将其称为成熟四足步态。组合启动为 `./scripts/run_go2w_dual.sh`，静止后 M 切换，完整结果见上述新文档。

日期：2026-10-25；运行目录 `/home/joy/桌面/RobotDog`。

## 软件与运行方式

- Ubuntu 24.04；NVIDIA 驱动仍为 595.99.02。
- 原生 ROS 2 Jazzy Desktop、RViz2、teleop_twist_keyboard、Fast DDS、colcon 已安装在 `/opt/ros/jazzy`。
- ROS apt 源使用清华镜像并保留 ROS 官方签名；安装过程中临时切换 Ubuntu 镜像，结束后已恢复原 Ubuntu archive/security 源。
- 仿真使用 Isaac Sim 6.2 自带 Jazzy 运行库；外部节点使用原生 `/usr/bin/python3` 和 `/opt/ros/jazzy`。两者通过 DDS 通信。
- Isaac Lab 3.0.0、RSL-RL 5.5.1、PyTorch 2.11；训练使用独立环境，不替换现有 Isaac Sim 环境。
- 机器人使用本地 Go2W 资产；训练地面及纹理已复制到 `sim/assets/ground`，避免训练启动时联网加载地面。

原生安装日志：`outputs/ros2/install-final.log`，含 `ROBOTDOG_ROS2_INSTALL_OK`。安装过程中因源下载缓慢的早期尝试日志保留作诊断，以最终日志和 dpkg 状态为准。

## ROS 跨进程验收

执行 `scripts/ros2/test_bridge.sh --policy outputs/training/wheel_flat_initial/policy_jit.pt`，仿真与检查节点为两个独立进程。

`outputs/ros2/verification-native-policy.json` 中 8 项检查全部通过：外部速度指令驱动、停止发布后的超时制动、RGB8 图像、相机内参、16 个关节、动态 TF、相机 optical 静态 TF、时钟推进。

最后一轮发送前进指令后总位移约 0.490m，指令过期后机身平面速度约 0.00362m/s。测试期间约 4.467 秒仿真时间对应 4.502 秒墙钟时间。位移包含发送指令与等待超时过程，不能据此直接计算稳态速度跟踪误差。

话题及使用方式见 `docs/Go2W_ROS2与训练.md`。里程计为仿真真值，未实现真实里程计估计；没有接入实机。

## 训练与评估

| 检查 | 规模 | 结果 |
|---|---|---|
| wheel 动力学检查 | 4 环境 × 240 控制步 | 有限观测/奖励；最小机身高度约 0.385m |
| walk 动力学检查 | 4 环境 × 240 控制步 | 有限观测/奖励；最小机身高度约 0.346m |
| wheel 短程 PPO | 64 环境 × 50 轮 × 24 步 | 权重保存、加载、导出与回放通过 |
| wheel 初始平地 PPO | 256 环境 × 300 轮 × 24 步 | 约 184 万条转移；权重与包含归一化的 TorchScript 导出通过 |
| walk 短程 PPO | 32 环境 × 20 轮 × 24 步 | 训练、保存、导出、回放通过；不足以形成成熟步态 |

轮腿初始策略：`outputs/training/wheel_flat_initial/model_final.pt`；原键盘/ROS 场景使用同目录 `policy_jit.pt` 及 `policy_jit.json`。关节顺序在契约文件中显式保存，回放时按名字匹配。

固定指令评估分为静止、前进、转向、停车四段，策略自主输出，种子 42。指标为整个测试时段的平均值，不是多种子泛化结果：

| 策略 | 环境 × 步数 | 倾倒/越界提前重置 | 平均平面速度误差 | 平均直立度 | 最小机身高度 |
|---|---|---|---|---|---|
| wheel，300 轮 | 8 × 600（各 20s） | 0 | 0.02273m/s | 0.99957 | 0.376m |
| walk，20 轮 | 4 × 600（各 20s） | 0 | 0.21272m/s | 0.99816 | 0.317m |

直立度是重力投影的负 Z 分量，水平时为 1。速度误差是机身 XY 速度误差的范数，不包含 yaw 误差。数据保存在各训练目录 `evaluation/evaluation.json`；详细日志分别为 `outputs/go2w-wheel-evaluation.log` 与 `outputs/go2w-walk-evaluation.log`。

上述结果支持“平地轮腿初始策略已跑通”，不支持“复杂地形控制或成熟四足步态已完成”。walk 仍需更多训练、足端接触/滑移检查和步态评估。当前训练不包含推力恢复、复杂地形、质量/摩擦/延迟随机化或实机验证。

## 操作保护与边界

- ROS 速度指令 0.5 秒无更新时归零。
- 学习策略收到零命令时，轮子制动并恢复稳定站姿。
- R 或停止后重播会恢复机身、关节和速度状态；暂停继续不重置位置。
- 默认 ROS_DOMAIN_ID=42；仿真话题使用 `/go2w` 前缀。
- 大规模并行训练在单个进程内进行；避免同时启动多个训练 Kit 进程。

图形训练回放首次被 SIGKILL 中断；修复启动线程配置后复测通过：4 环境 × 90 步，提前重置 0 次，平均平面速度误差约 0.0305m/s。日志为 `outputs/go2w-gui-replay.log`。无窗口训练、评估和原仿真场景的导出策略回放均已通过。

训练启动曾在 Kit 创建后台进程时发生 OpenBLAS 线程退出崩溃（堆栈包含 blas_thread_shutdown/pthread_atfork）。启动脚本现固定 OPENBLAS_NUM_THREADS=1，并限制 OMP/MKL 等 CPU 线程数。修复后的本地地面检查已通过，日志 `outputs/go2w-local-ground-smoke.log`；未降低 GPU 并行环境数。相关线程设置参照 [OpenBLAS 用法说明](https://github.com/OpenMathLib/OpenBLAS/blob/develop/USAGE.md)，fork 处理见 [OpenBLAS 源码](https://github.com/OpenMathLib/OpenBLAS/blob/develop/driver/others/memory.c)。

2026-10-25 当次结束时曾启动交互窗口：ROS 2 + wheel_flat_initial 策略。原生 ROS 外部节点当时检查 8 项全部通过；结果 `outputs/ros2/verification-gui-policy.json`，超时后平面速度约 0.00430m/s。这是历史运行状态，不表示现在仍有窗口或进程运行；本次进程盘点与复验见阶段 0 记录。
