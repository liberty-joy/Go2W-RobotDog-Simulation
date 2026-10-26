# 传感流水线进度

更新时间：2026-10-26（Asia/Shanghai）。**阶段4已完成，等待下一阶段指令。**

首读：[00复现总览与文档索引](../00_复现总览与文档索引.md)。阶段3开发run：`stage3_20261026T2110_clean_dev`；最终留出run：`stage3_20261026T2118_clean_final`；阶段2：`stage2_20261026T172917`；阶段1：`stage1_20261026T154717`；阶段0：`stage0_20261026T151923`。以前阶段是输入/历史基线，不冒充本次结果。

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

## 当前综合限制 / 未完成

- 阶段2实际GUI窗口和自动运行已检查，但没有把它称为人工W键交互验收；阶段1控制验证仍是等效前进指令重放和ROS跨进程检查。
- 速度仍需校准：命令0.15 m/s时平均实速约0.190 m/s；新控制器坡面/摩擦/扰动鲁棒性尚未复验，不能沿用旧50次记录。
- 阶段3是理想零畸变针孔仿真和离线算法；CameraInfo值以JSON文件保存，没有把左右图/CameraInfo发布为ROS话题，也没有真实设备标定、红外测量模型或滚动快门。
- 重复纹理深度错误很大，无纹理coverage较低；当前SGBM输出不可直接解释成自由空间或安全避障输入。
- 尚无视觉/雷达/语义融合、ROS IMU发布、SLAM/Nav2闭环、固定语义模型。odom/动态TF仍是真值，仅用于调通/评估。
- 335L外观虽为Isaac6.2官方认证资产，项目测量仍是功能代理；OS0官方资产只有REV6/7，不能冒充REV8。ScanBuffer读取API已deprecated，下一阶段需迁移并校验ring/时间。
- Ouster两份PDF网页可读，但Linux GET403；无本地文件和SHA，不伪造下载。首次启用MotionBVH约120秒启动，最终复跑约15.6秒，日志都保留。
- 阶段4仅验收Isaac参数化理想旋转点云链路。没有厂家级雷达/IMU测量模型或噪声、azimuth/ring/intensity/objectId，未验证实机运动畸变、时钟同步、RGB逐像素遮挡、动态自扫分类；PointCloud2的TF为仿真真值，未做SLAM/去畸变，也不生成LaserScan或安全地图。没有运行GUI人工验收。

## 阶段0保留基线

已有原生Jazzy/FastDDS、Sim6.2/Lab3.0与5090Laptop环境；无窗口12项检查通过，30秒可靠RGB MCAP约410 MB/14.9 Hz；GUI渲染进程检查通过但无人工验收。详见[00_environment.md](00_environment.md)，不作为本次传感器同步/匹配验收。

## 证据与下一阶段输入

- [01选型/配置/官方来源](01_selection.md)、[01a滚动修复](01a_rolling_fix.md)
- [机器清单](../../configs/sensors/selection_manifest.json)、[仿真参数](../../configs/sensors/stage1_sim.json)
- [本次验收表](../../outputs/sensor_pipeline/stage1_20261026T154717/acceptance.csv)、[控制对照图](../../outputs/sensor_pipeline/stage1_20261026T154717/wheel_fix.png)
- [传感器样例图](../../outputs/sensor_pipeline/stage1_20261026T154717/sensor_smoke_final/sensor_preview.png)、[本次原始证据](../../outputs/sensor_pipeline/stage1_20261026T154717/)

下一阶段可在用户指示后基于当前有效mask与置信度推进视觉/雷达融合，再接ROS2闭环；未经处理不要将低置信/无效深度规划为自由空间。开发和最终留出结果留在Linux `outputs/sensor_pipeline/`。当前阶段4结束，等待用户指定下一阶段。
