# 01 传感器选型与可运行仿真配置

访问/执行日期：2026-10-26；本次证据 `outputs/sensor_pipeline/stage1_20261026T154717`。先读 [阶段0基线](00_environment.md)；本阶段不采购、不接受SDK协议、不安装候选SDK、不训练运动策略。以下“室内民用巡检、1–5 m目标”是开发假设，尚未由用户确认。

## 1. 决策与边界

**实机研究优先方案：Gemini 335L + Ouster OS0 REV8 64线 + 雷达内置6轴IMU。** 条件是后续核实实际固件的左右IR同步/标定导出、近距目标精度、相机和雷达时钟关联、完整订货号及安装供电。335L的全局快门、95 mm基线和标称理想距离覆盖开发假设；OS0的宽垂直视场利于近处地面/高处覆盖。这是候选排序，不是采购决定。

**成本导向备选：Gemini 335 + 同一雷达，优先借用/复用已有设备。** 335的50 mm基线及理想0.3–3 m使5 m匹配精度更需实测；目前没有同日同配置授权报价，不能断言总价更低。全部价格状态为**询价**，未编造金额。雷达也先参数化仿真，暂不采购。

**彩色双目对照：ZED 2i 2.1 mm广角、无偏振片（官方参考 ZED-221110）**。左右彩色可直接用于匹配，RGB语义与左图同光学系统，研究链路更直观；但滚动快门和无外部硬触发是机器人振动/多传感同步的限制，不能作为已同步硬件方案。OS1 REV8 64线是雷达备选：44°视场比OS0窄，相同线数下角采样更密，若对象高度窗口确定可优先；现阶段不替代近地覆盖假设。

**立即运行：通用理想双针孔RGB + 参数化RTX旋转64线 + 物理IMU**，见 `configs/sensors/stage1_sim.json`，不是任何产品的数字孪生，也不是实机采购参数。优先复用Isaac 6.2本机接口，不安装仅宣称兼容6.0/6.1的品牌扩展。后续阶段才验证同步/标定、左右匹配深度与ROS雷达链路。

## 2. 相机：官方核查比较

下列范围/FPS均为厂家模式或条件值，不代表实际三路同时满速、特定反光/黑色目标精度。IR“原始”指可访问左右图流，不保证未处理Bayer；是否已经去畸变必须随配置记录。

| 项目 | Orbbec Gemini 335 | Orbbec Gemini 335L | Stereolabs ZED 2i 广角2.1 mm |
|---|---|---|---|
| 光学/快门 | 双IR全局 + 独立RGB滚动 | 双IR全局 + 独立RGB全局 | 双彩色、电子同步滚动快门 |
| 基线 | 50 mm | 95 mm | 120 mm |
| FOV | 深度90°×65°±3°；RGB 86°×55°(16:9) | 深度90°×65°±3°；RGB94°×68°(16:10) | 110°H×70°V，120°D |
| 代表模式 | IR/深度1280×800@30；IR640×480@30/60/90；RGB1920×1080@30 | IR/深度1280×800@30；RGB1280×800@30；低分辨率高帧率依USB3模式表 | 每眼2208×1242@15；1920×1080@15/30；1280×720@15/30/60；672×376@15/30/60/100 |
| 距离条件 | 工作0.1–20 m+，目标反射率>10%；理想0.3–3 m | 工作0.17–20 m+，同条件；理想0.25–6 m | 广角深度0.3–20 m，理想0.3–12 m；场景纹理/照明影响匹配 |
| 左右流/标定 | ROS启用left_ir/right_ir；SDK流配置提供内参/畸变及跨流外参 | 同系列接口；实际固件/三路带宽待设备确认 | SDK LEFT/RIGHT_UNRECTIFIED与rectified流；raw/rectified calibration分开 |
| 同步/时钟 | 硬件时间戳、帧同步、外部/多机同步接口；与雷达PTP关联未验 | 同左 | 内部左右同步；无外部硬触发I/O；多机软件时间关联不等于曝光同步 |
| IMU | 6轴，gyro+accel 50–1000 Hz | 同左（官方合并表格同时适用两型号） | 内置IMU/环境传感器；本项目不作为首选融合IMU |
| 接口/供电/质量 | USB3/USB2 Type-C数据供电；均值<3 W/峰值<6.5 W；97 g | Type-C；均值<3 W/峰值<6 W(FW1.4.00)；133 g | USB3 Type-C、5 V USB、典型1.9 W；229 g |
| 尺寸/防护 | 90×25×30 mm、IP5X | 124×29×27 mm、IP65 | 175.3×30.3×43.1 mm、IP66 |

规格依据：[Gemini335产品页](https://www.orbbec.com/products/stereo-vision-camera/gemini-335/)、[335L产品页](https://www.orbbec.com/products/stereo-vision-camera/gemini-335l/)、[Orbbec USB系列数据表V1.4，2025-01-24](https://orbbec.com/wp-content/uploads/2024/04/Gemini-330-series-Datasheet-V1.4_20250124.pdf)、[ZED官方规格](https://docs.stereolabs.com/docs/products/cameras/zed/specifications)。数据表本地只作查证，未导出版权PDF到共享盘。

Orbbec ROS2官方启动文件明确开放 `enable_left_ir`/`enable_right_ir`（默认关闭）、`enable_frame_sync`、`time_domain`、`enable_sync_host_time`、去畸变开关及 `enable_laser`。标定读流的intrinsic/distortion/extrinsic接口可用；实际应保存原图对应内参而非混用D2C后的内参。**本阶段没有设备实测**；跨光学RGB对齐必须做IR→RGB外参投影、遮挡判断，不能直接拿IR像素坐标当RGB坐标。[官方ROS2仓库](https://github.com/orbbec/OrbbecSDK_ROS2)、[SDK v2](https://github.com/orbbec/OrbbecSDK_v2)。

ZED可取未校正左右图和raw calibration，也可取rectified图及其内参，必须成对使用。[官方标定说明](https://docs.stereolabs.com/docs/development/zed-sdk/modules/camera/camera-calibration)、[ROS2节点](https://docs.stereolabs.com/docs/integrations/ros-2/zed-stereo-node)。不启用偏振片作为室内默认，窄角4.6 mm近距下限1.5 m不覆盖1 m假设，故本轮不选。

## 3. 软件、许可及仿真支持

| 候选 | Linux/ROS2与计算 | 许可与本机兼容性 |
|---|---|---|
| Orbbec | 官方ROS2 v2支持Jazzy；机内MX6800产生厂商深度，传流/板载深度不要求NVIDIA GPU；后续自算匹配另预算 | ROS2 Apache-2.0；SDK源码MIT并含第三方许可，extensions二进制有仅用于Orbbec产品等限制，不能将整个SDK称为无条件开源；未安装/接受EULA。未找到明确声明支持Isaac Sim6.2的335/335L官方测量扩展/资产，记待证实；Isaac ROS集成不等于Isaac Sim资产 |
| ZED | SDK5.5 Linux支持Ubuntu24/22；CUDA GPU计算深度，官方当前要求compute capability≥7.5；本机GPU能力足够，SDK/驱动组合未实际验 | 当前ROS2 wrapper Apache-2.0；SDK专有许可；zed-isaac-sim扩展MIT，附带runtime另受SDK许可。官方v5.3.0说明6.0/6.1，未声明本机6.2，不能声称已兼容或一定不兼容 |
| Ouster | 官方ouster-ros ROS2分支支持Jazzy，CPU解包；NVIDIA RTX仿真需要本机RTX | 驱动/SDK BSD-3-Clause及各第三方许可。NVIDIA现有OS0/OS1资产是REV6/REV7配置，未验证REV8官方孪生 |

[ZED Linux要求](https://docs.stereolabs.com/docs/development/zed-sdk/linux)、[官方仿真插件](https://github.com/stereolabs/zed-isaac-sim)、[Ouster驱动](https://github.com/ouster-lidar/ouster-ros)、[NVIDIA非视觉传感资产](https://docs.isaacsim.omniverse.nvidia.com/latest/assets/usd_assets_nonvisual_sensors.html)。ZED插件的SDK接收路径可从左右图匹配深度；其SDK-free/IsaacLab渲染深度及直接StreamDepth路径是几何真值，不能替代B组匹配结果。

锁定的访问时分支commit、release、所读许可/启动文件大小与SHA-256在本次 `sources/repositories.json`、`selected_repo_files.json`，并写入机器清单。commit是访问时分支，不冒充release tag的commit。未镜像库、未下载权重/SDK。

## 4. 旋转3D雷达比较：不能混用硬件代次

| 项目 | Ouster OS0 REV8.0，64线候选 | Ouster OS1 REV8，64线候选 |
|---|---|---|
| 订货系列 | OS0-080-XXX-X-X-002-XX | OS1-080-XXX-X-X-002-XX |
| 旋转/FOV | 机械旋转CW，360°H×90°V(±45°) | 机械旋转CW，360°H×44°V(±22°)，不是旧代42.4° |
| 垂直/水平采样 | 系列32/64/128线；水平512/1024/2048/4096 | 同左；线数属于硬件配置，不能靠软件把32线变128线 |
| 扫描率组合 | 512/1024支持10/15/20/30/40 Hz；2048支持10/20；4096支持5/10 | 512/1024同左；2048还列15 Hz；4096 5/10；不能把全部分辨率与全部转速任意组合 |
| 最小/远距条件 | 最小可设0/0.3 m，默认0.5 m；10%反射率35 m@100 klx、1024×10 Hz、>90%检测 | 最小同左；同条件10%反射率90 m；本项目不需要此远距 |
| 点属性 | 距离/反射率/信号/NIR、channel/方位/时间戳；REV8有原生RGB能力 | 同左；A/B不额外引入雷达RGB语义，以保持公共相机RGB一致 |
| IMU | IAM-20685HP，6轴，640/1280/2560 Hz（默认2560） | 同左；不能套用REV7的IMU型号/100 Hz |
| 时钟 | IEEE1588 PTP/gPTP、PPS/NMEA；点时间分辨率<1 μs，时钟同步宣称<1 ms | 同左；PTP同步不是RGB曝光同时发生的证明 |
| 网络/电源/质量 | UDP Gigabit Ethernet；9–58 V（标称12/24 V）；10–20 W、典型15 W@1024×10；电源建议≥30 W连续能力；裸500 g/带帽580 g | 同左；实际线缆/支架/供电模块另计 |
| 机械 | 直径87 mm；无帽高58.35 mm、带帽74.2 mm | 同左 |

依据：[OS0 REV8 v4.0官方表](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os0.pdf)、[OS1 REV8 v4.0官方表](https://data.ouster.io/downloads/datasheets/datasheet-rev8-v4p0-os1.pdf)、[官方下载入口](https://ouster.com/downloads)。完整订货后缀、64线实际beam angles、固件版本、REV8专有字段在当前ROS2版本的完整传递需实物/厂商确认，未知不补写。

驱动应保留 `point_type=native` 的ring/时间字段；`xyz`或只保留xyzir会丢失必要信息，后续逐点时间单位/参考零点要实际验包。[官方驱动](https://github.com/ouster-lidar/ouster-ros)。本次PDF通过web读取成功；Linux下载GET两份均403，HEAD显示732485/747652字节，**没有本地文件、SHA为null**；失败记录保留，不伪造下载成功。安装目录自带REV7 JSON仅作通用配置的结构参考，无外观CAD下载。

ToF不产生左右匹配，本阶段不列作双目替代；Livox等非传统旋转扫描需要另建扫描/时间模型，本轮两款均为传统旋转候选，没有将非重复扫描伪装成机械旋转。

## 5. 可逆仿真选择、映射与公平对照

配置入口 `configs/sensors/stage1_sim.json`、`generic_rotary_64.json`。单环境、640×480、10 Hz理想左右RGB、80°水平视场、100 mm基线，左相机沿用公共RGB视点；右相机沿base的-Y偏移0.1 m。理想RTX 64×512/圈、10 Hz、垂直±45°、量程0.5–20 m；IMU在雷达中心随120 Hz物理步进读取。所有安装位姿是临时假设，不是CAD标定。

Isaac6.2把旧JSON创建路径迁移为OmniLidar USD schema，本项目适配器从本地参数创建Core/EmitterState，不修改Isaac安装目录。实际写入numberOfEmitters/Channels=64、patternFiringRateHz=5120、scanRate=10、64条均匀俯仰角及逐发射偏移；噪声理想化，非REV8厂家beam calibration。生成的点云来自RTX测量模型，不是外壳显示。保留点时间/发射器ID可用性检查，ID不能未经验证直接冒充ROS ring。

映射335L/335：将基线改95/50 mm，替换IR左右对应内参、畸变、RGB外参；当前RGB渲染不模拟850 nm主动投影/材质光谱，不能宣称复现真实IR双目性能。映射ZED：基线120 mm，按所选模式内参调整；当前理想瞬时快门不含真实滚动快门。映射OS1：V-FOV改±22°并使用实际beam angles；OS0也须以设备metadata替换均匀角与发射时间。IMU后续加入厂家噪声、轴定义、外参和时间偏移；本次理想orientation是真值调试量，不作为估计器输入。

A仅公共RGB+雷达；B使用相同公共RGB+雷达，再加左右图匹配深度。两组固定控制/场景/定位/导航/语义模型、QoS和种子。独立几何深度/真值位姿/语义标签只用于评估。当前odom仍为真值；匹配、同步验收、融合、SLAM/Nav2均未完成，不使用目标点打印替代导航。

## 6. 运行与本次验收证据

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/selection_$(date +%Y%m%dT%H%M%S)"
mkdir -p "$RUN"
./scripts/run_go2w.sh --headless --policy outputs/training/wheel_slope/policy_jit.pt \
  --steps 300 --sensor-selection configs/sensors/stage1_sim.json \
  --sensor-output "$RUN/sample" > "$RUN/sim.log" 2>&1
# 必须检查完成标记与文件；Kit退出码0本身不等于通过。
rg 'SELECTION_SENSOR_SMOKE_PASS|GO2W_SIMULATION_COMPLETE' "$RUN/sim.log"
cat "$RUN/sample/smoke.json"
```

当前试验结果、失败重试、轮抖修复与ROS回归见本次 `acceptance.json` 和 [01a滚动修复](01a_rolling_fix.md)。无窗口测量烟测与人工GUI交互分别记录；没有人工键盘验收声明。样图/点云仅证明传感器数据可读，不证明校准准确或曝光同步。

## 7. 预算与下一阶段所需输入

单卡单环境先10 Hz：640×480 RGB8每路9.216 MB/s，两路18.432 MB/s；未来Mono8左右+公共RGB约15.36 MB/s。64×512×10=327680理论点/s，按32字节/点约10.49 MB/s。两路RGB+点云约28.92 MB/s，60秒约1.74 GB（十进制、未含ROS/容器开销）；保守为60秒预留3 GB。以阶段0单RGB可靠30秒410 MB为已实测参考；本阶段短测不冒充新60秒录包实测。

先各30–60秒再测存储/GPU、时间戳与匹配效果；每轮原始样本上限10 GB、阶段2/3工作集先20 GB，保持Linux空闲≥80 GB。不开大训练；先CPU/OpenCV或已有环境的无训练匹配/融合基线。若后续确需训练，先写对象/数据/划分/指标/预算，单卡小样本≤15分钟冒烟，再单次≤2 GPU小时且可中止；此为上限建议，尚未批准执行或配置训练。共享盘只单向导出版本文档、选定图/表/日志，不自动同步缓存/环境/包。

用户以后提供（现在不阻塞参数化开发）：是否已有设备、完整型号/硬件修订/固件/镜头；预算与授权报价；对象尺寸/材质/照明/运动速度；1–5 m是否真实任务；相机同步线/雷达PTP与IMU选择；机器人安装空间、载重、可用电压/功率、网口USB带宽。下一阶段输入为本清单及参数配置，先做同步/标定/离线匹配验证；等待下一阶段指令。

## 8. 本次实际测量补充（不是下一阶段验收）

`sensor_smoke_retry7/smoke.json`：左右均480×640×3；12,952个有限XYZ点，逐点uint64时间跨度99,844,362 ns，emitterId有31种返回（上半视场无命中不等于未配置64束）；IMU有效，读数time=2.0 s、physics_step=240。两张最新图的render time均为1.85 s，早于IMU读取时刻2.0 s，**不能把同一次读取当同步采样**。目前通道ID未单独输出，返回emitterId还不是已验收ROS ring。

启用运动BVH须放在SimulationApp启动参数，运行时切换会产生Hydra警告与无完整样本。成功试验首次启动约120 s，在等待viewport处触发已有120秒faulthandler提示，随后正常完成300步；这个启动延迟保留为限制，未据此估算稳态实时率。旧IMU接口在6.2失效，改用本机experimental.physics.IMU/IMUSensor；Lidar用新OmniLidar authoring加仍可用但已deprecated的ScanBuffer读取。后续应迁移GenericModelOutput并验证通道/时间语义，不把这个短烟测当成熟导航传感链。

初次及retry1–retry6均失败，日志/failure.json原样保留；包括旧IMU导入、schema接口、JSON配置路径、数组设置、channelId必须1-based，以及运行时运动BVH问题。没有用重装隐藏这些失败。

最终版本复跑 `sensor_smoke_final.log` 正常结束（约15.6秒墙钟含启动），12,954点，左右RGB/有效IMU再次可读。首次120秒启动延迟未在最终复跑重复，但仍保留日志，不承诺所有启动都同样快。最终证据以 `sensor_smoke_final/` 为准；`sensor_smoke_retry7/` 是首次成功样本。重新生成样图：`python3 scripts/sensor_pipeline/visualize_selection.py --sample outputs/sensor_pipeline/stage1_20261026T154717/sensor_smoke_final`；重新生成验收表：`python3 scripts/sensor_pipeline/summarize_selection.py --run outputs/sensor_pipeline/stage1_20261026T154717`。
