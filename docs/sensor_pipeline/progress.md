# 传感流水线进度

更新时间：2026-10-27（Asia/Shanghai）。**阶段7已知地图导航与固定2×2巡检工程回归已完成，等待下一阶段指令。**

首读：[00复现总览与文档索引](../00_复现总览与文档索引.md)。阶段3开发run：`stage3_20261026T2110_clean_dev`；最终留出run：`stage3_20261026T2118_clean_final`；阶段2：`stage2_20261026T172917`；阶段1：`stage1_20261026T154717`；阶段0：`stage0_20261026T151923`。以前阶段是输入/历史基线，不冒充本次结果。

## 阶段7已完成（本次实际执行）

- 本次真实盘点没有Nav2/SLAM/定位器；新增已知地图雷达ICP定位、膨胀栅格A*、速度反馈与本机ROS目标/取消/GetPlan接口。先oracle开发、后estimated独立验收；两者到点/取消/超时/障碍/不可达等7项通过，独立ROS接口9项通过，执行边界13项和旧控制8项通过。
- 先人工属性规则实际自主停靠拍照等待；再使用官方4,958,839字节SqueezeNet1_1 CPU视觉特征辅助外观核验。设备类别仍人工登记，ImageNet类别不当工业识别，没有LLM/在线付费推理。完整来源/许可/SHA已记录，CLIP慢速未完成下载及独立未用依赖保留。
- 最终 `stage7_20261027_2x2_v4` 完成12次独立仿真：A直接2/3、A证据3/3、B直接2/3、B证据3/3；总任务10/12完成，人工遥控0。2次失败均为过期证据安全拒绝；证据组在相同观察预算下各补1次观察完成。本轮没有双目任务完成数收益。
- 3种固定条件×四格，各仅1seed工程回归；1.4s延迟为人工压力条件，经过调试观察过场景，不能叫盲测、显著性结论或一般巡检成功率。原v1/v2/v3和两次相机开发失败均保留，不只重试B。
- 相机采用Stage7独立真实渲染帧/新整圈后因果选择，保持原光学/640×480/10cm/名义10Hz，实际24观察≤50ms、全部模型输入SHA匹配、12run同步拒绝0；不声称硬件同步。主定位统一estimated，真值仅评价。
- 实测视觉P50/P95=13.07/14.94ms、worker峰值RSS632MiB，无CUDA推理；最大采样仿真进程RSS7625MiB、compute VRAM3071MiB。已有环境/驱动/权重未覆盖。
- [阶段7详细结果与命令](07_closed_loop.md)、[结果网页/失败视频](../../outputs/sensor_pipeline/stage7_20261027_2x2_v4/results.html)、[逐次结果](../../outputs/sensor_pipeline/stage7_20261027_2x2_v4/task_results.csv)、[验收](../../outputs/sensor_pipeline/stage7_20261027_2x2_v4/acceptance.json)。

## 阶段6已完成（本次实际执行）

- 独立显式ROS传感器模式复用旧入口，公共RGB/左右RGB/CameraInfo/32FC1匹配深度、旋转PC2、理想IMU、安装TF与clock跨进程可用。新模式真值odom/位姿/动态TF隔离到evaluation；旧控制跨进程8项回归通过。
- 本次run `stage6_20261027T_stage6_first`：墙钟41.639s，MCAP319,442,363字节；304组图像/信息精确同stamp，329圈点云、1978条IMU，发送→录制差值0。相机实际仿真9.200Hz/墙钟7.301Hz，未达名义10Hz；303/304图像有≤50ms软件配对，首启动边界帧失败保留。
- 16话题真实完整回放逐字段/数组内容一致，MCAP原CDR存储哈希一致；再次短回放产生1次clock回退并清空缓存，始终1个clock源。首轮CDR对齐填充误判留在`replay_attempt01`。
- 测试采集前冻结2开发+2留出场景/对象，每个6次，共24配对，同一公共RGB/语义掩码/雷达SHA与stamp一致，无额外重试、无测试调参。A24/24已知，B15/24已知；B在无纹理/重复纹理出现9次冲突unknown，未显示整体收益。留出A12/12、B6/12；不是自主任务成功率。
- A CPU计算P50/P95=6.23/8.01ms；B计入录制SGBM为55.91/66.35ms，非端到端。推理RSS241.6/263.2MiB；仿真约8.09GiB RSS/2.81GiB VRAM。没有训练/环境安装/权重修改。
- [阶段6报告/查看与回放命令](06_offline_ab.md)、[实测验收](../../outputs/sensor_pipeline/stage6_20261027T_stage6_first/acceptance.json)、[配对汇总](../../outputs/sensor_pipeline/stage6_20261027T_stage6_first/offline/paired_summary.csv)、[传感图像](../../outputs/sensor_pipeline/stage6_20261027T_stage6_first/offline/recorded_sensor_views.png)。

## 阶段2已完成（本次实际执行）

- 从Isaac Sim 6.2官方S3按需本地化Gemini335L USD、OS0入口/10个REV6/7配置/网格/贴图，共约49 MiB；17项大小与SHA记录到`SOURCE.json`/manifest。未下载算法库、SDK或权重。
- 新增独立`/base/sensor_rig`装配：官方外观与理想双RGB、参数化64线RTX、理想IMU测量分开；供应商原件未改，旧演示默认路径不变。
- 离线闭包审计通过：17项哈希、14个USD层，0远程依赖、0缺失依赖；裸MDL模块由现有Isaac安装路径解析。装配为m/Z-up，基线95mm，无独立碰撞/刚体/质量。
- 最终无窗口运行实际得到左右640×480 RGB、12,084个有限RTX点、逐点timestamp/emitterId和有效IMU；0.48m自扫描圈内0点，日志无Error。
- 实际GUI运行并保存1440×900窗口截图；整机观察图、左右相机RGB和GUI截图分开记录。
- 旧默认入口不带sensor-rig参数回归180步正常结束，新增装配保持显式可选。
- 首轮缺OS0贴图、第二轮外壳悬浮均作为未完成尝试保留，第三轮才作为最终证据。

证据：[02资产/装配验收](02_assets.md)、[机器验收](../../outputs/sensor_pipeline/stage2_20261026T172917/final_acceptance.json)、[资产审计](../../outputs/sensor_pipeline/stage2_20261026T172917/asset_audit.json)、[样例组合图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_headless_03/stage2_contact_sheet.png)、[GUI截图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_gui_01/isaac_gui_window.png)。

## 阶段1已完成（本次实际执行）

- 修复滚动右前轮轴随腿前后摆动：保持名义站姿、轮式残差平滑/同侧合并/变化率限制；平地前进重放摆幅30.602→3.940 mm，下降87.1%。原权重未变，可用`--wheel-control legacy`回退。
- ROS跨进程8项与静止轮腿切换8项本次均通过；197个既有模型/资产/权重文件SHA复核无变化。
- 联网核查3个相机候选及2个传统旋转雷达候选，区分实机排序、开发假设与未知项；保留官方URL、日期、commit/release、许可与下载哈希。
- 实机研究优先Gemini335L+OS0 REV8 64线+雷达IMU；所有价格询价，未采购、未接受协议、未安装大型SDK。
- 本机Isaac6.2通用参数配置实际复跑成功：640×480左右RGB、12,954个RTX点、逐点时间/emitterId及有效物理IMU；只是API/测量烟测。
- 新增编号选型/修复文档、机器清单、参数配置、复算脚本、图/验收表与实际命令记录；7次失败尝试及中间修复轨迹保留。

## 阶段3已完成（本次实际执行）

- 新增可重跑同步双目采集与OpenCV StereoSGBM离线链路：640×480、10Hz、100mm基线理想RGB针孔。保留`FrontCamera`公共视点作为每个样本相同语义输入；未连实机、未训练LLM或运动策略、未下载预训练权重。
- 干净开发集8场景（1/2m×四类）用于候选调参；默认候选分数最高并冻结。最终留出8场景（3/5m×四类、新纹理seed 84017）在参数冻结后捕获。左右/公共RGB时间差0秒，所有图像640×480；Camera API内参/CameraInfo等价JSON、右P基线和正视差均通过复核。
- Isaac `distance_to_image_plane`几何深度单独写入`ground_truth/`，SGBM函数只接收左右RGB，匹配返回后才用真值评估。结果含左右校正图、定点/像素视差、公制光轴Z、LR拒绝与有效mask、误差图、极线图和逐场景CSV/JSON。无效Z写NaN，不作为自由空间。
- 留出集按理想纹理/遮挡/重复纹理/无纹理的coverage为0.4726/0.4774/0.2328/0.1593；MAE为0.1076/0.1050/1.4351/0.0311m。周期纹理错配显著，禁止把当前结果当作巡检安全深度输入。
- 首轮目录冲突、字段错误、相机矩阵遗漏、固定演示箱污染的试跑均留存并记为不合格；采用干净开发和新seed留出结果。详见[03_stereo.md](03_stereo.md)与[最终结果](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/analysis/results.json)。

## 阶段4已完成（本次实际执行）

- 使用阶段1 `generic_rotary_64.json` 与阶段2传感器装配，维持Go2-W、公共前置RGB和轮式控制基线；新增可选Stage4采集入口、配置、分析器、PointCloud2订阅检查脚本和独立启动器。供应商外观子树在采集时关闭以免遮挡公共RGB，传感器测量和物理控制仍启用。
- 最终有效run为 `stage4_20261026T214509_15`：Isaac Sim 6.2.0、RTX扩展3.4.0、约60Hz渲染、120Hz物理、10Hz完整圈累计，逐点跨度中位99.939ms；49圈输出。API的 `accumulateOutputs=True` 支持完整扫描语义，不能将其说成每个渲染帧同时采齐360°。缓冲区无方位角，因而方向/每点角度未独立验证。
- 静态0.3/0.5/1/2/3/5m靶面各3圈；最小距离0.5m，最近有效回波0.500007m，距离轴误差MAE约0.73–2.34cm。原始逐点 `timestamp` / `emitterId` 保留；无有效 `ring`、`intensity`、`objectId`、`azimuth`。命中点数不等同发射射线数。
- 低速0.08m/s设定持续2.5仿真秒，机身实际前进约0.2505m；每圈期间约移动1.08cm。输出时间着色原始运动点云，未去畸变、未使用IMU/位姿真值做运动补偿。
- Isaac理想IMU保留300个60Hz读取样本，物理更新率按step为120Hz；无噪声/偏置/漂移。IMU真值未参与去畸变。ROS 2 Jazzy独立订阅通过PointCloud2字段/长度/有限XYZ检查；不发布LaserScan。
- 机身网格可见/隐藏差异未观察到显著自扫变化；对象ID缺失、0.5m盲区限制了结论。公共RGB与雷达的视锥角几何交集平均12.65%，不是像素级遮挡检验。GUI未测试。
- 失败轮次 `01–14` 和日志均保留，最终证据只引用 `214509_15`。详见[04_lidar.md](04_lidar.md)、[最终采集汇总](../../outputs/sensor_pipeline/stage4_20261026T214509_15/capture_summary.json)、[逐圈时间](../../outputs/sensor_pipeline/stage4_20261026T214509_15/scan_timing.csv)、[点云图](../../outputs/sensor_pipeline/stage4_20261026T214509_15/motion_pointcloud_timestamp.png)及[ROS订阅验收](../../outputs/sensor_pipeline/stage4_20261026T214509_15/ros_pointcloud2_received.json)。

## 阶段5已完成（本次实际执行）

- 新增同场景公共RGB/左右RGB/通用旋转LiDAR采集器，按静止→直行→转弯→停车保存图像、逐点时间、逐帧真值位姿和IMU时钟。最终有效run为`stage5_20261026T223623_accepted`，四段观测、60圈扫描，转弯实际0.2564rad（约14.7°）。不连接实机，不重训策略或下载大模型。
- 明确`T_camera_lidar(t_p,t_e)=inverse(T_world_camera(t_e))·T_world_lidar(t_p)`；公共RGB光学+z向前。按实际每点时间插值LiDAR真值位姿到相机曝光，内参/零畸变投影后做画面过滤、同像素最近回波、前景深度层选择。该动态去畸变是Isaac真值oracle条件，非估计姿态验收。
- 首轮`stage5_20261026T222539_capture1`用图像SHA绑定的人工多边形核对几何，再用无训练红色连通区域分割；手绘/自动掩码IoU为0.965–0.977。最终run采用自动分割与阶段3固定SGBM参数，输出对象`lidar_only/stereo_only/hybrid` JSON/CSV及投影图。
- 静止、直行、转弯的LiDAR对象中心误差为1.80/2.51/4.79cm，双目为1.83/2.90/5.35cm。停车观察时两路中心差16.69cm，融合输出`unknown`/请求重观测；没有简单平均。该10cm冲突门限是在开发场景上从20cm修正，未进行独立留出验收。
- 60圈与最近公共RGB曝光的软件时间配对绝对差中位33.254ms。测试0/10/30/50ms算法时间偏移与估计外参1/3cm、0.5/1.5°扰动共48条，未改变观测真值。核心几何/冲突逻辑5项通过。完整记录见[05_fusion.md](05_fusion.md)与[最终对象表](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/object_geometry.csv)。

## 当前综合限制 / 未完成

- 阶段7是已知地图、人工初始区域、近直线低速停靠与人工登记单红面板；不是SLAM/Nav2、通用设备识别、语言规划或复杂3D安全导航。主结果全部估计定位，oracle仅开发。没有阶段7全流ROSbag/逐点归档或动态去畸变，只有选定XYZ/圈末时间快照与任务轨迹。

- 阶段6只覆盖静态几何/感知，B重复纹理场景全unknown；没有动态可靠融合或自主闭环。软件配对存在≤50ms离线前视，不是硬件同步；相机源低于名义10Hz。

- 阶段2实际GUI窗口和自动运行已检查，但没有把它称为人工W键交互验收；阶段1控制验证仍是等效前进指令重放和ROS跨进程检查。
- 速度仍需校准：命令0.15 m/s时平均实速约0.190 m/s；新控制器坡面/摩擦/扰动鲁棒性尚未复验，不能沿用旧50次记录。
- 阶段3是理想零畸变针孔仿真；当时CameraInfo只存JSON。阶段6已新增左右图/CameraInfo/匹配深度ROS发布，但仍没有真实设备标定、红外测量模型或滚动快门。
- 重复纹理深度错误很大，无纹理coverage较低；当前SGBM输出不可直接解释成自由空间或安全避障输入。
- 已有单红面板的离线双目/雷达对象级开发融合，阶段6新增理想IMU ROS发布和固定经典红色分割；阶段7已新增已知地图最小导航及预训练通用外观特征，但尚无通用多对象语义融合、SLAM/Nav2或工业设备语义模型。odom/动态TF仍是真值，阶段6独立模式隔离到evaluation；原模式仍用于调通。
- 335L外观虽为Isaac6.2官方认证资产，项目测量仍是功能代理；OS0官方资产只有REV6/7，不能冒充REV8。ScanBuffer读取API已deprecated，下一阶段需迁移并校验ring/时间。
- Ouster两份PDF网页可读，但Linux GET403；无本地文件和SHA，不伪造下载。首次启用MotionBVH约120秒启动，最终复跑约15.6秒，日志都保留。
- 阶段4仅验收Isaac参数化理想旋转点云链路。没有厂家级雷达/IMU测量模型或噪声、azimuth/ring/intensity/objectId，未验证实机运动畸变、时钟同步、RGB逐像素遮挡、动态自扫分类；PointCloud2的TF为仿真真值，未做SLAM/去畸变，也不生成LaserScan或安全地图。没有运行GUI人工验收。
- 阶段5融合依赖仿真真值外参与每点真值位姿，没有硬件同步、实机标定、可部署IMU/视觉里程计去畸变、运动目标或复杂遮挡验证；单红面板和已开发调参门限不代表留出集精度或安全导航。GUI人工检查未运行。阶段4关于“未做去畸变”的限制只适用于阶段4原始点云；阶段5新增的是离线oracle姿态补偿。

## 阶段0保留基线

已有原生Jazzy/FastDDS、Sim6.2/Lab3.0与5090Laptop环境；无窗口12项检查通过，30秒可靠RGB MCAP约410 MB/14.9 Hz；GUI渲染进程检查通过但无人工验收。详见[00_environment.md](00_environment.md)，不作为本次传感器同步/匹配验收。

## 证据与下一阶段输入

- [01选型/配置/官方来源](01_selection.md)、[01a滚动修复](01a_rolling_fix.md)
- [机器清单](../../configs/sensors/selection_manifest.json)、[仿真参数](../../configs/sensors/stage1_sim.json)
- [本次验收表](../../outputs/sensor_pipeline/stage1_20261026T154717/acceptance.csv)、[控制对照图](../../outputs/sensor_pipeline/stage1_20261026T154717/wheel_fix.png)
- [传感器样例图](../../outputs/sensor_pipeline/stage1_20261026T154717/sensor_smoke_final/sensor_preview.png)、[本次原始证据](../../outputs/sensor_pipeline/stage1_20261026T154717/)

下一阶段可在用户指示后建立独立纹理/遮挡/运动留出集与估计位姿来源，再评估ROS2闭环；未经处理不要将低置信/无效深度规划为自由空间。所有结果留在Linux `outputs/sensor_pipeline/`。当前阶段5结束，等待用户指定下一阶段。
