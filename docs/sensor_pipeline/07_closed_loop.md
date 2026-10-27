# 07 已知地图定位、导航与受限巡检闭环

日期：2026-10-27（Asia/Shanghai）。最终固定协议目录为 `outputs/sensor_pipeline/stage7_20261027_2x2_v4/`；该轮已实际完成。范围是**单场景、平地、已知地图、人工登记红面板的仿真工程回归**。没有SLAM、Nav2、通用工业设备语义分类、LLM规划或实机控制。

## 1. 本机盘点与最小导航

本次检查了现有源码、ROS安装与阶段6验收：原来只有`nav_msgs`标准消息、速度控制、传感器发布和evaluation真值里程计；`/opt/ros/jazzy/share`没有Nav2/AMCL/SLAM包，项目没有可运行定位/导航实现。没有把ROS接口或历史截图当作自主导航。

新增已知地图最小导航：

- 物理房间墙、静态地标、碰撞障碍由独立Stage7场景创建；已知先验平面图范围x=[−1.8,4.2]m、y=[−1.6,1.7]m，0.05m栅格。不是用当前真值位姿生成定位输出，也不是SLAM建图。
- 定位器用实际旋转3D点云的**明确高度切片**（LiDAR局部z∈[−0.18,0.35]m），体素去重、结构地图最近邻匹配、截尾ICP估计SE(2)。已知初始区域手工给定[0.06,−0.04,0.025]，没有全局重定位；没有融合真值/轮速、IMU或双目来改善主结果定位。A/B始终使用同一定位器。
- A*在膨胀地图规划，路径反馈跟随输出原`/go2w/cmd_vel`；继续使用原Go2-W轮式速度控制与0.5s命令看门狗。最大请求速度0.13m/s、转速0.35rad/s，位置容差0.12m、航向容差0.18rad。
- 四角轮廓[±0.45,±0.30]m，地图膨胀半径0.55m；局部3D点云门控在前方0.78m、横向±0.38m内触发停车，持续1.5s即报告obstacle_stop，不盲目穿障或把未知深度当自由空间。它不是完整3D地形/悬空障碍安全导航。
- 新模式的`map→odom→base_link`由估计器发布；`base_link→相机/雷达/IMU`来自独立sensor_rig安装TF。`/go2w/navigation/odom`为平面**位姿里程计**，twist/协方差未提供估计，不把默认零值解释为精确速度。z固定约0.39m，没有完整6DoF姿态估计。

先以oracle真值定位运行开发验收，再单独以估计定位重验。oracle只读取`/evaluation/go2w/odom`作为开发控制位姿并明确标记；估计模式的该话题只进入`TruthEvaluator`记录评分，ICP/任务/控制函数均不接收它。主2×2全为estimated。

## 2. 真实接口与基础验收

|接口|作用|
|---|---|
|`/map`、`/go2w/navigation/costmap`|transient_local OccupancyGrid，先验占据与机器人膨胀图|
|`/go2w/navigation/footprint`|base_link坐标的PolygonStamped轮廓|
|`/go2w/navigation/path`|map坐标的实际A* Path|
|`/go2w/navigation/odom`、`/tf`|估计平面位姿和map/odom/base_link链|
|`/go2w/navigation/goal`|JSON `{id,x,y,yaw,timeout_s}`，目标范围/有限数校验|
|`/goal_pose`|标准PoseStamped目标，要求frame_id=map|
|`/go2w/navigation/get_plan`|标准nav_msgs/GetPlan，只规划不执行|
|`/go2w/navigation/cancel`|std_srvs/Trigger，清活动目标并停车；任务模式同时取消任务|
|`/go2w/navigation/status`|接受、结果、拒绝、失败原因等结构化事件|

这是**自建最小导航接口**，没有冒充Nav2 NavigateToPose action。未就绪GetPlan返回空路径；无clock的消息使用合法零Time，避免原生序列化崩溃。

实际开发目录：`outputs/sensor_pipeline/stage7_20261027_dev/oracle_01/`及`estimated_01/`。两种定位均真实完成：已知0.7m目标到达、通过ROS服务取消、0.2s目标超时、物理障碍停车、地图外不可达失败、失败后停稳等7项检查。估计定位首轮位置误差P50=0.00361m、P95=0.02017m，只覆盖这个房间/手工初始区域。独立ROS进程收到地图、代价地图、路径、轮廓、估计里程计、TF，并实际调用GetPlan；9项通过，见`interfaces_cross_process_v2.json`。不是只打印目标或遥控录像。

## 3. 人工任务、视觉模型与执行边界

先运行`rule_task_01`：人工登记设备`equipment_red_01`，属性red+inspection_panel，规则生成inspect→snapshot→wait(0.5s)。登记表**不向任务规划器提供设备世界坐标**，目标位置来自实际公共RGB掩码与几何证据。实际规则链已自主停靠拍照并等待。

随后引入一个实际本地视觉模型：torchvision **0.26.0 SqueezeNet1_1_Weights.IMAGENET1K_V1**，CPU float32/4线程。官方权重4,958,839字节，完整SHA-256：

`b8a52dc049b60e4b6ab68ad0df457362afab8b6304b2febdc1650a5dab4d7e7b`

原始来源：[官方权重](https://download.pytorch.org/models/squeezenet1_1-b8a52dc0.pth)、[官方模型文档](https://docs.pytorch.org/vision/0.26/models/generated/torchvision.models.squeezenet1_1.html)、[torchvision v0.26.0许可](https://github.com/pytorch/vision/blob/v0.26.0/LICENSE)。代码为BSD-3-Clause；预训练来自ImageNet，不能把代码许可当作所有数据/场景的工业认证。版本、大小、许可来源和SHA记录于`configs/models/stage7_squeezenet_SOURCE.json`。

模型输出512维外观特征和ImageNet类别分数；初次登记可见对象作为外观参考，到达后与该参考比较。开发两张图余弦0.869，既定阈值0.65冻结。开发ImageNet首类实际为**prayer rug**，因此类别输出只保存诊断，**不作为设备类别或身份判断**。设备属性仍为人工登记+固定红色分割；模型只用于外观一致性，没有声称理解真实工业设备。这是视觉特征模型+规则规划器，不是VLM/LLM。

模型在独立CPU worker运行，没有在线推理、登录或付费服务；输出只作为JSON数据。执行层固定允许检查指定设备、拍照、等待、补充观察；模型不生成任意Python、shell、ROS速度或电机命令。执行层另外检查对象登记/存在、证据时效、深度unknown/冲突、可达性、取消状态与非有限坐标。缺证据可有界重观测，不能绕过执行校验。

曾核查OpenAI原仓库CLIP ViT-B/32：官方大小353,976,522字节，原仓库MIT、commit `d05afc436d78f1c48dc0dbf8e5980a9d471f35f6`；官方端点约70KiB/s，下载中止。保留partial、未加载，并保存`unused_clip_manifest.json`。首次pip调用因环境无pip失败，随后uv只将4个CLIP依赖放到独立`/home/joy/robotics/stage7-clip-deps`，最终SqueezeNet未使用这些依赖。系统Python、Isaac环境、驱动、原权重均未覆盖；没有偷偷替换为在线模型。

## 4. 固定2×2与传感器时间

`stage7_protocol.json`固定A/B×direct/evidence四格，3个条件各1个种子，共12次独立Isaac运行。开发及基础设施调试曾看过这些场景，最终明确称**固定工程回归**，不是盲测统计或论文显著性结果。

- A：公共RGB+雷达用于对象几何；B：同一公共视点+左右SGBM估计深度+雷达。B语义不另用左右图，不读取真值深度。主定位、公共相机、视觉模型、导航/控制参数固定。
- direct按当前提案尝试执行一次；evidence在发现证据不满足条件时先请求补充观察。**两组都必须经过相同独立执行校验**，direct也不能用过期/未知证据开车。
- normal：随机纹理，seed72011；stale_evidence：新随机纹理，seed72037，明确人为加入1.4s延迟测试1.0s证据时效；repetitive：重复纹理，seed72057。延迟压力条件不是自然任务分布，不能用它直接证明一般任务性能改善。
- 所有组最多3次观察、35s任务仿真超时、20s单导航目标；初始观察+到达确认名义2次，额外观察=`max(0,次数−2)`。失败/超时保留，不给B无限重试，也不让B读取更多真值。
- A/B各自启动同起点/场景/种子，实际轨迹和后续RGB可以不同，保存在各run；不是用同一份离线包推算闭环成功率。

Stage7额外俯视相机暴露旧Camera各自采样计时相位漂移。修复只在Stage7：通过公开`Camera.set_frequency(-1)`读实际每个渲染帧，在每个真实新整圈末点之后选择三路相同`rendering_time`且因果差0–50ms的帧，名义上限10Hz；仍为640×480/10cm/理想零畸变，SGBM与冲突门限不变。**没有改写时间戳或宣称硬件触发同步**。Stage6默认入口不启用这个钩子。

对象观察只在停车等待后进行，使用实际静态安装外参和估计定位。没有动态真值去畸变。开发修复后102组同步帧、0次拒绝，实际到达后照片可读；旧定步长采样落在前圈后约83ms而被严格50ms拒绝的失败也保留。

## 5. 本次主结果

本次最终v4 **12/12运行正常完成，任务10/12完成**，无人工遥控介入；另外2次为过期证据的安全拒绝。两类规划器均使用雷达估计定位，不含oracle主结果。

|传感器/规划|完成任务|额外观察合计|人工介入|
|---|---:|---:|---:|
|A / 直接规划|2/3|0|0|
|A / 证据约束|3/3|1|0|
|B / 直接规划|2/3|0|0|
|B / 证据约束|3/3|1|0|

正常纹理及本轮2.9m重复纹理四组均完成。人工延迟1.4s条件中，A/B直接规划均被执行层以stale_evidence拒绝；A/B证据约束各补1次观察后完成。两组传感器的完成数相同，**本轮没有证明双目提升闭环任务完成数**。重复条件与阶段6的4m留出失败并非同一输入，不应据此宣称修复重复纹理深度。

成功任务仿真用时约8.0–10.2s，对应墙钟约12.3–15.95s；失败安全拒绝约2.27–2.30s仿真/3.35s墙钟。逐run wall与sim分开列在task_results.csv，均不含/包含启动的不同口径也单列；不声称硬实时。

24次实际停车观察均满足0–50ms因果配对，圈末点差P50/P95约0.0763ms；**这只是圈末点对曝光，不能代表整圈约100ms内每个点同步或硬件触发**。12次相机同步拒绝计数全0，模型输入SHA全部匹配。照片/观察在模型返回后的最大证据年龄为0.233s，小于1s。各run首次记录的真实起点存在约3.2mm/0.00188rad物理/采样差异，配置起点与seed相同，没有伪称逐像素输入完全相等。

主运行固定定位器各run位置误差P95约0.0036–0.0151m；这是同一雷达已知地图定位器的事后评分，不归因于双目。对象平面位置误差P50/P95约0.00552/0.01137m，仅当前人工对象场景。B的12次观察中7次选择stereo、5次选择lidar，本轮未出现深度冲突；保留阶段6的失败作为不同条件限制。

实际视觉CPU耗时P50/P95=13.07/14.94ms，worker实测峰值RSS约632.2MiB，GPU进程列表无该worker。每秒采样的最大仿真/导航单进程RSS约7625MiB，最大compute进程VRAM约3071MiB；不是瞬时峰值保证。主结果目录约76.5MiB（加入源码快照后略增），不含历史失败轮。

已实际生成：`task_results.csv`（逐次任务）、`observation_results.csv`（模型/几何证据）、`paired_ab.csv`（同条件A/B轨迹/观测差异）、`group_summary.csv`、`actual_trajectories.png`、`task_summary.png`、`failure_index.json`及两条真实失败WebM。`acceptance.json`全部固定参数/源码/模型输入/预算/因果配对检查通过；`post_checks.json`保留额外核对。安全执行13项边界检查通过，旧ROS控制8项回归通过。

## 6. 失败保留和证据

- `stage7_20261027_2x2_v1`：独立接口探针在首clock前调用GetPlan，暴露None时间戳原生序列化断言；故障不是导航成功，保留崩溃日志/冻结源码。启动前未生成足够帧时如实无视频。
- `..._v2`：跨run复用同名视觉job ID，旧响应被RGB SHA校验拒绝；保留appearance_unconfirmed及失败视频。改为run绝对路径SHA前缀，并原子提交request。
- `..._v3`：部分到点但相机相位错开、到达确认超时；保留失败照片/视频/1772次同步拒绝记录。
- `stage7_20261027_dev/camera_fix_task_01`：固定6帧取样有约83ms因果配对差，35s无合格观察，失败保留；`camera_fix_task_02`才通过因果新圈后选帧复验。
- 最终完整四格统一从v4重跑，没有只挑B或失败格额外重试；场景/模型/定位/阈值/预算均按固定协议，无根据结果调参。

每个run含`config.json`、`navigation/{events.jsonl,metadata.json,config.json,metrics.json,task.json,task_metrics.json,trajectory.csv}`，每次观察有公共/左右原图、NaN估计深度、雷达XYZ/圈末时间快照、掩码、证据JSON和实际模型输出；truth只在轨迹/评分命名空间。`publisher/`保存真实发送与采样事件；`actual_simulation.mp4`和报告的WebM来自**实际Isaac俯视RGB**，有嵌入仿真时间，4fps展示不等同墙钟性能录像。GUI人工交互没有运行。

## 7. 命令行复现

查看已完成结果：

```bash
cd /home/joy/桌面/RobotDog
RUN=outputs/sensor_pipeline/stage7_20261027_2x2_v4
xdg-open "$RUN/results.html"
column -s, -t "$RUN/group_summary.csv"
xdg-open "$RUN/actual_trajectories.png"
cat "$RUN/acceptance.json"
```

从新目录完整重跑（校验已有小模型，无环境安装；12次独立仿真后生成网页/失败视频）：

```bash
cd /home/joy/桌面/RobotDog
scripts/run_stage7_suite.sh "outputs/sensor_pipeline/stage7_$(date +%Y%m%dT%H%M%S)"
```

分别重验基础导航：

```bash
scripts/run_stage7.sh "outputs/sensor_pipeline/stage7_oracle_$(date +%Y%m%dT%H%M%S)" oracle validation
scripts/run_stage7.sh "outputs/sensor_pipeline/stage7_estimated_$(date +%Y%m%dT%H%M%S)" estimated validation
```

不启视觉模型的规则任务开发演示：

```bash
scripts/run_stage7.sh "outputs/sensor_pipeline/stage7_rules_$(date +%Y%m%dT%H%M%S)" estimated task
```

只重算主报告，不启动仿真：

```bash
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/sensor_pipeline/stage7_report.py \
  --suite outputs/sensor_pipeline/stage7_20261027_2x2_v4 \
  --dev outputs/sensor_pipeline/stage7_20261027_dev
```

运行中的单个独立开发导航可另开终端只读检查，勿手工干预固定实验：

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=101 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ros2 topic echo /go2w/navigation/status
ros2 topic echo /go2w/navigation/odom --once
ros2 service call /go2w/navigation/cancel std_srvs/srv/Trigger '{}'
```

最后一个命令会取消当前开发任务；它不是只读检查，不应在固定12次实验中调用。所有命令限制本机localhost/domain101，未连接真实机器狗。

## 8. 文件拓扑、恢复与边界

- `stage7_geometry.py`：已知地图、ICP、A*；`stage7_navigation.py`：目标/取消/控制闭环；`stage7_tasks.py`：结构化任务与独立执行验证。
- `stage7_sensors.py`：独立场景、真实测量、因果同步和俯视视频；`stage7_vision_worker.py`：固定CPU视觉数据输出。
- `stage7_suite.py`+两个shell入口：单run与四格独立启动；`stage7_probe.py`：跨进程接口；`stage7_checks.py`：恶意/缺失/过期/取消/未就绪边界；`stage7_report.py`：逐次证据重算、表图和失败网页。
- `configs/sensors/stage7_closed_loop.json`：机器人/地图/目标预算基础配置；`stage7_protocol.json`：条件、seed与2×2；`configs/models/...SOURCE.json`：唯一实际模型来源/许可/SHA。

恢复原控制只需使用原`run_go2w_ros2.sh`且不传`--closed-loop-config`。新目录彼此独立，旧资产/驱动/Python/策略未覆盖；独立未使用CLIP依赖目录可移到备份位置，运行最终SqueezeNet无需它。若删除/移动新权重或输出，先按SOURCE/manifest备份失败与选定数据；不进行共享盘双向同步。

已知地图、手工初始区域、单红面板和理想噪声条件限制结论。没有验证多对象设备身份、真实任务语言理解、全局重定位、复杂遮挡、坡面/跌落/悬空物安全、动态行走融合、真实传感器时钟或实机。接入SLAM/Nav2、通用检测/VLM、更多种子统计或运动策略训练均属后续阶段；当前不自动继续。

数据归档限制：阶段7没有录制完整ROSbag；选定观测的`raw_lidar.npz`仅保存XYZ与圈末时间，没有逐点时间/emitter字段，不能用它重做动态去畸变。阶段6完整原始PC2包仍保留。本阶段可重算的是任务事件、选定几何/外观输出与轨迹指标，不覆盖全流逐点运动分析。

最终代码另跑`estimated_final`：7项导航/失败停车检查再次通过，估计定位P50/P95=0.00315/0.01358m；最后一轮旧控制8项通过、命令过期后速度约0.00104m/s。当前主要是近直线导航，没有独立90°转弯任务、动态绕障重规划或全局重定位验收。

版本化共享导出：`/mnt/Shared/First1/1023Robot_Dog_Development/sensor_pipeline_stage7_20261027_v1`，对应Linux主run `stage7_20261027_2x2_v4`。共享目录只含文档/图表/汇总、两条主失败视频和选定照片/验收，不复制执行环境或双向覆盖。

全部阶段7尝试目录清单见主run的`all_attempts_inventory.csv`：开发/中止轮与主v4分开列，均保留日志，启动前无视频的情况明确为缺失。运行版本为Isaac Sim6.2.0、ROS2 Jazzy/FastDDS、本机Python3.12；一键入口与完整配置/来源随本次Git阶段提交保存。
