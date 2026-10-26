# 00 阶段0：环境盘点与最小链路复验

执行日期：2026-10-26（Asia/Shanghai）。本次运行标识：`stage0_20261026T151923`。范围仅为单机器人仿真和离线样本；阶段1尚未开始。

## 1. 证据与本次结论

全部证据在 [stage0_20261026T151923](../../outputs/sensor_pipeline/stage0_20261026T151923/)。先看 [汇总图](../../outputs/sensor_pipeline/stage0_20261026T151923/stage0_summary.png)、[验收表 CSV](../../outputs/sensor_pipeline/stage0_20261026T151923/acceptance.csv)、[可重算汇总 JSON](../../outputs/sensor_pipeline/stage0_20261026T151923/summary.json)。初建文档或上次训练的通过记录不算本次结果。

| 项目 | 本次结果 | 证据 |
|---|---|---|
| 现有跨进程 ROS 检查 | 8/8 通过；仿真移动 0.420m，超时后速度 0.00666m/s | `headless_recheck/verification.json` |
| 无窗口静止链路 | 12/12 通过，观察 30.00s | `headless_reliable/probe.json` |
| GUI 进程/渲染/ROS | 12/12 自动检查通过，观察 10.00s | `gui_capture/probe.json` |
| GUI 人工交互 | **未测试**；未人工观察窗口或操作键盘焦点 | CSV 的 NOT_TESTED 行 |
| 单机稳定性 | 最低机身高度 0.38867m，最低直立度 0.999983，观察段最大平面速度 0.01346m/s | `headless_reliable/stability.json` |
| 可靠 RGB 采集 | 447 回调 / 30.00s，约 14.90Hz | `probe.json`、RGB 样图 |
| 可靠 MCAP 样本 | 410,149,799 bytes（391.1MiB），包内时间跨度 29.533s；RGB/CameraInfo 均 443 条 | `headless_reliable/bag_info.txt`、`bag/metadata.yaml` |
| 实时因子 | 0.9944（仿真秒/墙钟秒） | 本次 probe |
| GPU / 进程资源 | 可靠无窗口 GPU 总占用峰值 3732MiB，Sim RSS 约 7018MiB；GUI RSS 峰值约 7292MiB | 各次 `resources.log`，含桌面 GPU 占用 |

![本次前置 RGB 样图](../../outputs/sensor_pipeline/stage0_20261026T151923/headless_reliable/rgb.png)

场景为已有 Go2-W 资产、平地、红蓝方块。未把传感器外壳或几何深度冒充测量；当前只有单路 RGB。原检查发送的运动命令只在 localhost 的隔离 ROS 域 97 内驱动此仿真节点；采样 probe 本身不发布运动指令。

## 2. 真实环境、路径与版本

- Ubuntu 24.04，内核和完整系统快照见 `inventory/hardware.txt`；RTX 5090 Laptop GPU，驱动 **595.99.02**，显存 **24463MiB**。
- 初始物理内存 66,750,468,096 bytes（约 62.17GiB），可用约 55.4GiB；初始 Linux 可用 182,504,210,432 bytes（约 169.97GiB）。整理时本地可用约 168.82GiB。共享盘初始可用约 200.5GiB，但不作为环境/缓存/训练盘。
- Sim 解释器 `/home/joy/robotics/isaac-env/bin/python`：Python 3.12.3、isaacsim **6.2.0.0**（运行日志显示 RC 运行时）、torch **2.11.0+cu130**、NumPy **2.3.1**。
- Lab 解释器 `/home/joy/robotics/isaaclab-env/bin/python`：isaaclab **3.0.0**、RSL-RL **5.5.1**，torch/NumPy 如上。官方源码 Git `e3f7d9ff1f5d7f8280de3ece9000b7117f66bcf4`、tag `v3.0.0`，源码工作树检查干净。
- 两套环境均本次执行小 GPU 张量检查并输出 `GPU_COMPUTE_OK`；**本次没有执行 Lab 物理训练或重训运动策略**。
- 系统 `/usr/bin/python3` 为 3.12.3，原有 NumPy **1.26.4**、OpenCV **4.6.0** 可导入，适合后续先做 CPU 匹配基线；未混入 Sim 的 NumPy 2.x 路径。
- 已有启动器 `/home/joy/robotics/bin/isaac-sim`、`isaac-lab` 均已读取；项目入口 `scripts/run_go2w.sh`、`run_go2w_ros2.sh`、`run_go2w_dual.sh`、`train_go2w.sh` 已读取真实参数。
- 原生 ROS 2 **Jazzy** 在 `/opt/ros/jazzy`，RMW 为 **rmw_fastrtps_cpp**。Sim 使用其内置 Jazzy 库，外部 probe/录包使用原生 ROS；完整包列表和 RMW 包版本在 `inventory_final/ros.txt`。MCAP/sqlite3 与 zstd 插件已安装，无需本阶段安装。
- 没有发现适用于工作区的 AGENTS.md（检查祖先及工作区文件）。本工作区非 Git 仓库，用文件 SHA-256 和源文件快照标识本次代码；不编造项目 commit。

## 3. 机器人、控制和现有测试入口

- 官方 URDF/网格：`models/go2w/go2w_description`；原作者仓库 `https://github.com/unitreerobotics/unitree_ros`，来源记录 commit `15ebe53f4cd77e7d745ae83b9d7ff90e685e8998`，BSD-3-Clause；已有源文件约 30,421,158 bytes。本次不重新下载，文件 SHA-256 见清单。
- 动力学 USD：`sim/assets/go2w/go2w_description/go2w_description.usda` 及 payloads；仿真路径 `/World/Go2W/Geometry/base`；网格只是外观/碰撞几何，URDF/USD 才包含关节/惯性/驱动信息。
- 本次选用已有轮腿策略 `outputs/training/wheel_slope/policy_jit.pt` 与 JSON 契约；实验纯腿 `walk_clearance` 可用但完整抬足未通过历史验收。本阶段未训练、未改已有权重。
- physics_dt=1/120s，rendering_dt=1/60s，策略目标 30Hz；站姿 hip=0°、thigh=45°、calf=-90°。腿部 USD stiffness=4、damping=0.14；求解迭代32/4。轮速映射中的0.05m是未标定的有效尺度，不作真实轮半径规格。
- 已有 `scripts/ros2/test_bridge.sh`、`verify_bridge.py` 本次实际复跑。`test_modes.sh`、`verify_modes.py`、交互 `--test` 三轮复位检查已盘点，但本阶段没有重跑模式/三轮操控验收，不沿用历史通过作为本次结果。

## 4. 相机、QoS、坐标和时间

相机 prim 为 `/World/Go2W/Geometry/base/FrontCamera`。真实脚本配置：分辨率640×480，渲染相机 frequency=30，水平视场80°，焦距0.018m，近/远裁剪0.02/100m；相对 base 平移(0.32,0,0.08)m、绕Y俯角约5°。ROS 发布代码另外以15Hz调度图像，不能把相机 frequency=30 写成 ROS RGB 实测30Hz。

| 数据 | 话题 / 说明 | 本次约墙钟频率 |
|---|---|---|
| RGB | `/go2w/front_camera/image_raw`，rgb8，921600 bytes/图像负载 | 14.90Hz |
| 内参 | `/go2w/front_camera/camera_info`，理想 plumb_bob、D=0 | 14.93Hz |
| 关节 | `/go2w/joint_states`，16关节的位置和速度 | 29.83Hz |
| 位姿 | `/go2w/odom`，**仿真真值** | 29.83Hz |
| TF | `odom→base_link` 动态；`base_link→front_camera_link→front_camera_optical_frame` 静态 | 动态约30Hz |
| 时间 | `/clock`，仿真时间；录包显式 `--use-sim-time` | 29.83Hz |

本次内参 fx=381.361170，fy=381.361170，cx=320.0，cy=240.0。完整值在 `info.json`。optical 静态旋转按 ROS 消息为 x=-0.5,y=0.5,z=-0.5,w=0.5，optical +Z 对应相机 link +X；已检查挂载平移与坐标链。**这是与配置的几何一致性核对，不是通过标定板估计内外参。**

图像/CameraInfo 的模拟发布时间匹配，clock递增已检验。但头时间由 `world.current_time` 在发布时生成，尚未核实渲染曝光时刻、读取延迟或新旧帧序号。因此不能声称已完成左右曝光同步、硬件同步、旋转扫描逐点同步或动态标定。

默认 best-effort 样本接收 311/448 条 RGB/Info 回调，约 10.36Hz；对应MCAP仅305条RGB/442条Info。受限基线保留在 `headless_sample_retry1/`。新增可选 `--ros-image-reliable`，发布端、probe及MCAP RGB订阅使用兼容 reliable QoS；可靠MCAP有443条RGB/443条Info。本次采样边界与发现预热仍可能差一个回调，不以计数相等证明每帧曝光无损。默认普通启动仍保留原 best-effort 行为；正式A/B要固定相同QoS与采样设置。

## 5. 本次失败、修复和恢复方法

1. 初次启动在线默认地面服务器超时，机器人未加载。保留 `failed_initial/sim-bridge.log` 和初始源清单。改为引用已有 `sim/assets/ground/default_ground_plane.usda`，保留原接触系数static/dynamic=0.5、restitution=0.8。地面纹理与原在线默认地面可能不同，后续A/B使用同一新场景；没有修改原资产或下载整个库。
2. 加入完成标记时，一次已经加载旧代码的进程没有新标记，harness拒绝成功；ROS8项当时已通过。该次 `headless/runner.log` 保留，未当作最终成功。新完整复跑为 `headless_recheck/`，正常结束和ROS8项均通过。
3. 采集wrapper首次 `set -u` 遇到原生ROS setup可选未定义变量而退出。保留 `headless_sample_runner.log` 和空目录；source阶段临时关闭nounset后恢复，重试放在新目录，不覆盖失败目录。
4. 默认best-effort原始图像接收率不足，保留其MCAP；可选可靠QoS另录30秒样本，没有修改系统socket配置、驱动或用户环境。

代码变更仅在项目脚本：本地地面引用、完成标记、可选RGB QoS、检查输出目录、单发布者保护；新增inventory/probe/capture/汇总脚本。未覆盖系统Python、两套Isaac环境、驱动、资产或既有权重；没有训练、新SDK/模型下载、付费调用或新的许可同意动作。Kit自身运行缓存仅在Linux本地。

原始脚本哈希在 `inventory/baseline_files.json`；`baseline_sources/` 按逆变更还原的4个原脚本逐个与初始SHA-256完全匹配，可用于受控恢复。最终源码在 `source_snapshot/`，哈希在 `final_code_manifest.json`。197个已有资产、权重及相关结果文件的校验均未改变，见 `asset_weight_integrity.json`。文档原文备份 `documentation_before.tar.gz`，迁移表 `document_renames.json`。恢复文档先从备份解包到临时目录检查，再按迁移表恢复需要文件；不要把备份直接覆盖新阶段记录。项目脚本如需回退，按基线和最终源码快照逐个恢复；关闭 `--ros-image-reliable` 即恢复旧QoS，无需重装。旧在线地面回退会重新依赖网络，不建议用于本地复现。

## 6. 存储、渲染与后续训练预算

实测可靠样本约 13.67MB/s，按当前设置线性外推约 45.83GiB/小时；这是30秒小样本估计，不是长期性能承诺。默认best-effort更小是接收缺口造成，不能当作等质量压缩收益。具体预算由 `storage_budget.csv` 重算，以下均为未压缩估计：

| 参数化情景（640×480） | 30秒GiB | 每小时GiB | 加25%规划余量GiB/小时 |
|---|---:|---:|---:|
| RGB_15Hz | 0.386 | 46.35 | 57.94 |
| RGB_30Hz | 0.772 | 92.70 | 115.87 |
| A_RGB10_Lidar_assumed | 0.325 | 38.95 | 48.68 |
| B_stereo10_Lidar_assumed | 0.582 | 69.84 | 87.31 |
| B_plus_float32_depth10 | 0.925 | 111.04 | 138.80 |

未来雷达预算假设100000点/s、24bytes/点；并非设备规格或已实现测量。双目情景为公共左RGB + 右RGB；如设备是独立RGB+双红外，还需第三路成像和RGB对齐，必须重算。float32深度是派生产物，应优先离线重算，不能把真值深度作为B组匹配深度。JPEG有损压缩不在主对照默认预算中。

保守安排：

- 先单环境640×480，维持目前约15Hz已验收流；加入右图/雷达时先降到**双方相同10Hz**做30–60秒容量/同步冒烟，再决定是否提频。每个新配置先限2GiB采样；开发数据总额先限30GiB，保留Linux至少80GiB给系统与Isaac缓存。当前170GiB空闲不足以无节制保留上百小时原始双目数据。
- 后续匹配/融合先用已有原生OpenCV无训练基线：20对小样本→最多200对开发样本；先不新建重复Isaac环境。CPU线程先≤8、RAM≤16GiB；新增独立依赖才建Linux新venv，单独记录版本/卸载路径。
- 渲染先单环境，GPU总使用预算≤12GiB、进程RAM≤24GiB；这是保守资源上限而非已测双目/雷达峰值。本次单RGB峰值约3.65GiB GPU、7.12GiB RSS，增加传感器必须重新测。
- **本阶段训练预算为0**。若固定匹配基线在开发集确实不能满足预定误差/覆盖率，下一阶段先提交训练对象、必要性、来源许可、场景不重叠的train/dev/test拆分与指标，再考虑原作者预训练匹配器或小规模微调。首轮上限可设单GPU、batch1、显存≤12GiB、RAM≤24GiB、1小时、训练数据及checkpoint≤10GiB；实际模型未选定，不能承诺此预算能训练任意模型。先冒烟再扩大，不训练VLM/LLM，不默认重训运动策略。
- A/B离线共享同一公共RGB与同一采集包，避免重复保存；先小开发集验证工具，冻结测试集后再定统计规模。两组定位/导航/语义模型固定。若未来用真值定位调通，必须标注GT定位；现有真值odom不算导航系统。

## 7. 缺失能力与下一阶段输入

**尚未完成：**真实左右图匹配、双目rig/曝光序号与标定估计、旋转雷达测量/逐点时间、深度融合、语义模型、SLAM或定位估计器、Nav2最小自主导航链路、A/B主实验。ROS包表未发现SLAM/Nav2，`ros2_ws/src`为空；已安装depthimage_to_laserscan或RTX扩展不能代替已接入雷达。打印目标或手工遥控不计自主导航。

阶段1输入：本次版本与资产清单、公共RGB视点/内参、可靠采集配置、已有资源预算、民用巡检工作距离/目标大小的参数化假设。具体设备型号、IMU/接口、同步机制未知时，先参数化并标注假设；不以仿真参数冒充产品规格。后续资产应逐项记录官方URL、许可、release/commit、大小和SHA-256，再决定最小下载。

**本阶段结束即停止。等待用户指定下一阶段，不自动开始选型下载、双目/雷达接入、SLAM/Nav2安装或训练。**

## 8. 共享快照与停止点

版本化单向导出位置：`/mnt/Shared/First1/1023Robot_Dog_Development/releases/stage0_20261026T151923_docs_v1`。仅含文档、图表、验收CSV与选定小结果，带文件哈希；不含MCAP、资产、环境或权重。导出器拒绝覆盖版本目录，原共享内容保留。
