# 06 ROS传感器短包与同输入离线A/B

2026-10-27，阶段6完成，范围限于**静止单机器人、单对象几何/感知**。本次实际运行目录：`outputs/sensor_pipeline/stage6_20261027T_stage6_first/`。没有训练、SDK下载、实机通信、SLAM/Nav2闭环或自主任务成功率；GUI人工交互未运行。以前阶段只作代码/参数输入，不算本次验收。

## 1. 如何查看本次结果

```bash
cd /home/joy/桌面/RobotDog
RUN=outputs/sensor_pipeline/stage6_20261027T_stage6_first
xdg-open "$RUN/offline/recorded_sensor_views.png"
xdg-open "$RUN/offline/geometry_coverage.png"
xdg-open "$RUN/offline/sensor_projection_samples.png"
xdg-open "$RUN/offline/latency_resources.png"
column -s, -t "$RUN/offline/paired_summary.csv"
cat "$RUN/acceptance.json"
cat "$RUN/replay/verification.json"
```

`recorded_sensor_views.png`每行依次为公共RGB、左RGB、右RGB、SGBM估计Z，白色是无效/未知。它是**真实传感器输出组合图**，不是GUI截图。`geometry_coverage.png`中误差只统计已输出位置，B重复纹理场景没有误差柱意味着全unknown，绝不是零误差。`sensor_projection_samples.png`用绿点显示A关联到面板的雷达证据。

## 2. 输入、模式及目录拓扑

配置在测试采集前冻结：`configs/sensors/stage6_ros_ab.json`。2个开发场景（独立对象dev_panel_a/b）、2个留出场景（test_panel_c/d），使用不同位置/尺寸/纹理seed，按**整个场景/对象实例隔离**，不是相邻帧随机拆分。每场景固定6个观察偏移1.5–6.5s；单次33s仿真、约41.6s墙钟录包。场景为程序生成的简单红面板，没有宣称是多房间或真实巡检数据。公共RGB视点、相机参数、旋转配置、控制与权重均沿用前阶段。

- `scripts/go2w_teleop.py`：现有仿真/控制入口，新增显式可选`--sensor-pipeline-config/--sensor-pipeline-output`；不带参数仍走旧模式。
- `scripts/go2w_ros2.py`：复用ROS发布器，独立模式才将仿真odom/动态TF移到evaluation；原控制模式保留原话题。
- `scripts/sensor_pipeline/stage6_ros_sensors.py`：公共/左右同步帧、Stage3固定SGBM、Stage1旋转扫描、理想IMU和安装TF；仿真命令保持零速度。
- `scripts/sensor_pipeline/stage6_bag.py`：原生Jazzy录制、逐消息事件、实际回放及回退时钟清缓存；不运行第二个模拟时钟。
- `scripts/sensor_pipeline/stage6_offline.py`：从同包提取固定样本；A/B独立进程推理；完成后才单独读取evaluation评分。
- `scripts/sensor_pipeline/stage6_report.py`：从录包重查编码/内参/频率、资源和图表，保存代码快照与哈希。
- `scripts/run_stage6_record.sh`：新目录检查、磁盘检查、启动本任务录制/模拟器、清理自己启动的子进程、完成后真实回放。

每个run根目录有`metadata.json/config.json/commands.txt/source_manifest.json/source_snapshot/working_tree.patch`和完整日志；`publisher/`有配置快照、版本哈希、发送事件/计数；`record/`有MCAP、逐消息`events.jsonl/metrics.json`；`replay/`有实际播放器日志、全程指标、第二次短回放和验收；`offline/inputs/`为选定原始公共/左右图、录制的估计深度、原始点云、CameraInfo和静态安装外参。`offline/evaluation_only.json`单独保存评分真值，不放入inputs。`offline/A/`和`offline/B/`分别含**events/metadata/config/metrics**及证据点与同源掩码。`offline/per_sample_metrics.csv/paired_summary.csv`分别为逐次与配对汇总，`pairing.csv/scan_timing.csv/transport_rates.csv`可重算同步和频率。

## 3. ROS数据契约

|话题|编码/单位/无效值|frame_id/时间|
|---|---|---|
|`/go2w/front_camera/image_raw`|640×480 rgb8，1920字节行宽|`front_camera_optical_frame`，相机rendering_time曝光时间|
|`/go2w/stereo/left/image_raw`|同上，真实左右RGB匹配输入|`go2w_stereo_left_optical_frame`，与公共/右图完全相同stamp|
|`/go2w/stereo/right/image_raw`|同上|`go2w_stereo_right_optical_frame`|
|上述各目录`camera_info`|K/R/P/D，零畸变理想针孔；fx约381.361px；右P[3]=−fx×0.1m|与对应图像尺寸、stamp、frame一致|
|`/go2w/stereo/depth/image_raw`|**32FC1小端、公制光轴Z（米）**，2560字节行宽；NaN=未知，不能当自由空间|左校正光学坐标；本配置R校正为identity；stamp仍为输入曝光时刻|
|`/go2w/stereo/depth/camera_info`|SGBM校正P1的K，非真值Z|同左光学frame|
|`/go2w/lidar/points`|PointCloud2，米，32字节点布局；XYZ/相对扫描时间/emitter_id/逐点绝对sec+nsec|`go2w_lidar`，header为该圈最大有效逐点时间|
|`/go2w/imu/data`|角速度rad/s，specific force m/s²，理想无噪声；orientation_covariance[0]=−1表示**没有测量姿态**，其余0为未知协方差|`go2w_imu_link`，物理IMU读取时间，仿真约60Hz|
|`/tf_static`|一次5条安装变换，`T_base_sensor`将传感器点变换至base_link|base_link为父，传感器光学/测量frame为子，m|
|`/clock`|仿真时间；录包保存原始clock，回放不额外生成|仿真约30Hz|
|`/go2w/sensors/scene`|非真值JSON，场景/对象标识、split、曝光stamp、计算耗时|用于固定采样；对象身份为本单对象实验已知标签，不是多对象识别性能|
|`/evaluation/go2w/poses`、`/evaluation/go2w/odom`、`/evaluation/tf`|**仿真真值，仅独立评分**|世界为`simulation_world`，不接入A/B定位|

独立模式图像/信息/点云/IMU使用reliable、keep_last20；/clock与evaluation发布器depth10；/tf_static为reliable、transient_local、depth1。原生录制/回放验证订阅为reliable depth100；回放提供显式QoS覆盖文件。跨进程可靠订阅实际收齐，是传输证据；没有厂商硬件同步或DDS deadline保证。

雷达使用Stage1参数化64发射器×512列、10Hz、0.5–20m配置；只保留实际支持字段。`x/y/z float32`偏移0/4/8，`time_from_scan_start float64`偏移12，`emitter_id uint32`偏移20，`time_sec/time_nanosec uint32`偏移24/28。绝对点时间=`sec*10^9+nsec`，相对时间单位秒。**没有ring/intensity，emitter_id不冒充ring**。PC2无uint64字段类型，拆分秒/纳秒保留原始精度。本次每圈约99.939ms，header与有效末点完全一致，不是每帧瞬时360°点云；没有LaserScan或安全地图输出。

机身/雷达x前y左z上，相机光学x右y下z前，统一米。相机中心安装[0.32,0,0.08]m，左右y为±0.05m，相机俯5°；LiDAR[0,0,0.20]m，IMU[0,0,0.205]m。`T_camera_lidar=inv(T_base_camera)@T_base_lidar`。本次使用仿真配置安装外参作为基础条件，**不是实机标定**。厂家外观关闭以避免遮挡，测量仍是功能代理。

## 4. 实测传输、同步、存储与回放

|验收项|本次结果|
|---|---|
|短录包|墙钟41.639s，仿真约33s；MCAP zstd_fast **319,442,363字节（304.64MiB）**|
|原始序列化吞吐|1,357,141,498字节，31.08MiB/s；压缩落盘7.32MiB/s|
|公共/左右/估计深度+四路CameraInfo|各304条；**304/304组曝光stamp、尺寸和信息精确对应**，编码/内参检查通过|
|相机实际频率|曝光时间9.200Hz，墙钟7.301Hz；名义10Hz未完全达到，渲染采样间隔中位/P95约116.667ms，不能宣称稳定10Hz|
|雷达/IMU|329圈/1,978条；仿真10Hz/60Hz，墙钟7.90Hz/47.50Hz|
|发送→录制丢消息|有发送计数的13个话题全部一致，差值0；这是DDS计数检查，不掩盖源相机采样低于10Hz|
|LiDAR末点与RGB软件配对|303/304组≤50ms，绝对差P50 **33.251ms**、P95 **49.921ms**；首个启动边界帧无合格圈，明确保留失败|
|正式固定样本|4场景×6=24对，A/B同一RGB SHA、同一掩码SHA、同一圈点云，全部通过一致性检查|
|回放完整性|16个话题计数、canonical逐字段值/数组字节全部一致；MCAP原始CDR与接收流哈希也完全一致|
|回放时钟重置|完整回放后再播2秒，检测1次时钟回退并清空旧图像同步缓存；最大clock发布者1，没有`--clock`生成源|
|旧模式回归|原`test_bridge.sh`1800步，ROS跨进程8项通过；真值odom旧话题、TF、图像、外部速度指令及超时停车正常|
|GUI|未运行本阶段GUI人工交互；只检查实际相机输出图|

同stamp只验证理想同帧相机，雷达没有变成硬件同步。最近圈配对可能选到曝光后≤50ms的圈，属于**离线有限前视配对**，不能声称因果在线融合；动态去畸变未使用，也没有把真值姿态偷偷补入A/B。静止仿真仍有极小站姿残余运动，本结果不覆盖行走/转弯的动态误差。

首轮`replay_attempt01/`完整保留：各话题计数已一致，但比较重序列化CDR哈希出现失败。定位为未定义的CDR对齐填充，随后改为canonical字段比较，并额外对MCAP原始CDR做存储哈希检查；`replay/verification.json`为修复后同一包的实测复验。没有重新采漂亮样本、修改留出参数或删除失败。

## 5. 公平A/B与结果

A只读取**公共RGB+雷达+对应内参和安装TF**。B增加由录制左右RGB计算并录制的**SGBM估计深度**；B语义仍只读同一公共RGB，不用左右图另做语义，也不读任何几何真值深度。原始左右图仍保存在MCAP和样本中供重算。Stage3 SGBM定点/16、LR一致性、NaN规则不变；Stage5掩码阈值、前景层和10cm冲突门限不变。推理函数不读取evaluation目录，评分在两个进程退出后执行。

用同一冻结红色连通区域分割作为可解释语义基线，没有通用检测网络或人工标真值给B。雷达/双目点均投到公共视点，先过滤后方/越界、逐像素最近回波、按公共掩码关联，再做前景深度层和稳健中心。B模态一致时选择较低深度MAD一路，不假定独立误差作平均；两路差>10cm则unknown，无额外重试。每帧只尝试一次。

|场景/划分|A已知率|B已知率|A位置误差均值|B位置误差均值（仅已知）|B冲突|
|---|---:|---:|---:|---:|---:|
|dev_room_a 随机纹理/开发|6/6|6/6|1.605cm|1.605cm|0|
|dev_room_b 无纹理/开发|6/6|3/6|1.561cm|1.561cm|3|
|test_room_c 新随机纹理/留出|6/6|6/6|1.761cm|1.761cm|0|
|test_room_d 重复纹理/留出|6/6|0/6|0.569cm|**unknown，无有效误差**|6|

A总24/24已知，B总15/24；留出部分A12/12、B6/12。所有有输出的B选择雷达，所以其误差没有改善。A掩码支持覆盖率各场景约59.8/60.6/62.7/61.9%；B包含unknown记零后约59.8/30.3/62.7/0%。覆盖定义为保留点投影的3像素邻域与公共掩码相交比例，**不是无穷补全深度**。所选有效证据的对象关联purity本场景为1.0：仅在评分时检查点距目标平面<8cm且在扩张2cm边界内；单对象红平面实验过于简单，不能外推多对象关联/遮挡正确率。

本阶段结果**不支持双目加入后整体更好**：重复/无纹理误匹配可让保守冲突处理拒绝原本可用的雷达位置。保留这项失败，不用留出集调门限。若要改融合规则，必须在下一阶段新开发数据上确定，并重新建立未看过的测试集。

## 6. 延迟、内存与预算

A计算P50/P95 **6.23/8.01ms**；B几何处理 **32.67/43.10ms**，加该帧已录制SGBM耗时后，B完整计算成本P50/P95 **55.91/66.35ms**。SGBM发布端本次P50/P95为23.34/26.38ms。推理进程峰值RSS A **241.6MiB**、B **263.2MiB**，均CPU计算无CUDA上下文，新增算法VRAM为0；仿真进程峰值RSS **8287MiB**、VRAM **2881MiB**（1秒采样，不保证抓到瞬时峰值）。

这些是**离线串行CPU计算延迟**，包含样本读入/分割/几何，不是传感器曝光到机器人动作的端到端延迟；另附`sync_distribution.png`给采集时差。统一录制发布端先算SGBM再发图，A在该包中的到达时间也受该公共开销影响，不能用此包声称A/B各自独立在线流水线的实时时延。压缩/回放开销、≤50ms离线配对前视与调度另计。

按当前场景压缩约25.72GiB/墙钟小时；**场景纹理改变会改变压缩比**，不按此直接放大长期采集。当前约164GiB空闲；保留≥50GiB余量，下一步仍先5分钟分片（约2.2GiB/片），建议一轮≤30分钟并审核实际大小。新配置扩大前重做30–60秒预算。未进行训练；若将来训练感知，先写训练对象/数据/指标/预算，运动策略本阶段不重训。

## 7. 完整复现与回放

全新采集（自动录制+实际回放，拒绝覆盖已有目录）：

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/stage6_$(date +%Y%m%dT%H%M%S)"
scripts/run_stage6_record.sh "$RUN"
source /opt/ros/jazzy/setup.bash
/usr/bin/python3 scripts/sensor_pipeline/stage6_offline.py --run "$RUN" > "$RUN/offline.log" 2>&1
/usr/bin/python3 scripts/sensor_pipeline/stage6_report.py --run "$RUN" > "$RUN/report.log" 2>&1
```

离线推理会拒绝覆盖已有A/B结果目录；相同数据再次评分请先复制run到新分析目录或明确归档旧分析结果，不删除失败。仅重画/核对报告可以再次运行report，它不改变A/B推理结果。

只回放现有包（不要同时启动模拟器，不加`--clock`）：

```bash
cd /home/joy/桌面/RobotDog
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN=outputs/sensor_pipeline/stage6_20261027T_stage6_first
ros2 bag info "$RUN/record/bag"
ros2 bag play "$RUN/record/bag" --disable-keyboard-controls \
  --qos-profile-overrides-path "$RUN/replay/qos.yaml"
```

另一个终端同样source并设置domain97：`ros2 run rqt_image_view rqt_image_view`，选择公共、左、右或深度话题；`ros2 topic echo /go2w/stereo/left/camera_info --once`；`ros2 topic hz /go2w/lidar/points`；RViz固定frame用`base_link`可检查静态点云。此阶段`simulation_world→base_link`只在evaluation，不应为了显示把真值定位接回感知。回放频率受本次墙钟录制速度限制，与仿真时间频率分开。

独立再做完整+重置回放验收，用新目录避免覆盖：

```bash
ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1 /usr/bin/python3 scripts/sensor_pipeline/stage6_bag.py replay \
  --out "$RUN/replay_$(date +%Y%m%dT%H%M%S)"
```

原控制模式照旧：`scripts/run_go2w_ros2.sh`，不传sensor-pipeline参数；本次回归命令见`legacy_bridge/`记录。

## 8. 限制与下一阶段输入

已完成ROS发布、真实短录包/完整回放、静态同输入A/B及可重算证据。未完成动态可靠融合、估计定位/去畸变、真实传感器噪声/触发同步、厂家ring/intensity、通用语义、多对象关联、SLAM/Nav2闭环、导航安全性或运动策略训练。原相机源实际9.2Hz未达名义10Hz，暂不修改相机/控制基线；后续性能优化需独立复验。

恢复方法：原模式不带新参数；全部新功能显式可选，不改驱动、系统Python、现有Isaac环境/权重。依赖复用Isaac Python及原生Jazzy Python；没有新环境或软件安装。新录包/导出目录单独管理，源代码修改可按Git阶段提交回退。下一阶段应先在新开发场景分析B冲突、定义融合质量门控与新留出协议，再决定是否推进动态或导航；本次到此结束。

版本化共享导出：`/mnt/Shared/First1/1023Robot_Dog_Development/sensor_pipeline_stage6_20261027_v1/`。只含文档、18项选定图表/配置/汇总与SHA清单；原始MCAP仍在Linux本地，不进行双向同步。
