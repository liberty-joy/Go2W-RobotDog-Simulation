# 传感流水线进度

更新时间：2026-10-26（Asia/Shanghai）。**阶段2已完成，等待下一阶段指令。**

首读：[00复现总览与文档索引](../00_复现总览与文档索引.md)。当前run：`stage2_20261026T172917`；阶段1：`stage1_20261026T154717`；阶段0：`stage0_20261026T151923`。以前阶段是输入/历史基线，不冒充本次结果。

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

## 本次限制 / 未完成

- 阶段2实际GUI窗口和自动运行已检查，但没有把它称为人工W键交互验收；阶段1控制验证仍是等效前进指令重放和ROS跨进程检查。
- 速度仍需校准：命令0.15 m/s时平均实速约0.190 m/s；新控制器坡面/摩擦/扰动鲁棒性尚未复验，不能沿用旧50次记录。
- 阶段2左右图和雷达/IMU虽在同一运行读取，但不等于同步曝光；同步/标定验收、真实IR/滚动快门/工厂beam模型未完成。
- 尚无左右匹配深度、融合、ROS雷达/IMU发布、SLAM/Nav2闭环、固定语义模型。odom/动态TF仍是真值，仅用于调通/评估。
- 335L外观虽为Isaac6.2官方认证资产，项目测量仍是功能代理；OS0官方资产只有REV6/7，不能冒充REV8。ScanBuffer读取API已deprecated，下一阶段需迁移并校验ring/时间。
- Ouster两份PDF网页可读，但Linux GET403；无本地文件和SHA，不伪造下载。首次启用MotionBVH约120秒启动，最终复跑约15.6秒，日志都保留。

## 阶段0保留基线

已有原生Jazzy/FastDDS、Sim6.2/Lab3.0与5090Laptop环境；无窗口12项检查通过，30秒可靠RGB MCAP约410 MB/14.9 Hz；GUI渲染进程检查通过但无人工验收。详见[00_environment.md](00_environment.md)，不作为本次传感器同步/匹配验收。

## 证据与下一阶段输入

- [01选型/配置/官方来源](01_selection.md)、[01a滚动修复](01a_rolling_fix.md)
- [机器清单](../../configs/sensors/selection_manifest.json)、[仿真参数](../../configs/sensors/stage1_sim.json)
- [本次验收表](../../outputs/sensor_pipeline/stage1_20261026T154717/acceptance.csv)、[控制对照图](../../outputs/sensor_pipeline/stage1_20261026T154717/wheel_fix.png)
- [传感器样例图](../../outputs/sensor_pipeline/stage1_20261026T154717/sensor_smoke_final/sensor_preview.png)、[本次原始证据](../../outputs/sensor_pipeline/stage1_20261026T154717/)

下一阶段读取阶段2闭包资产、配置、左右图和逐点时间字段，先同步与标定/离线匹配，不重训运动策略。保持A公共RGB+雷达，B再加真实左右匹配深度，真值仅评估；原始包留Linux，单向版本导出。真实设备参数未知时按显式假设推进。本阶段结束。
