# 04 旋转雷达、IMU与ROS 2 PointCloud2

状态：阶段4已完成。最终实测目录：`outputs/sensor_pipeline/stage4_20261026T214509_15/`。此前失败/过渡目录全部保留；只有该最终目录用于下文指标。

## 目标、基线与范围

- 沿用阶段1的 `configs/sensors/generic_rotary_64.json` 和阶段2传感器测量配置：Isaac RTX理想化参数扫描器，64个发射器、512列/圈、10Hz、0.5–20m、水平0–360°、垂直±45°、单回波、CW配置、理想量程且无传感器噪声。这是通用功能代理，不是Ouster OS0 REV8数字孪生，也不是厂家精度验收。
- Go2-W、公共前置 `FrontCamera` 和已有轮式策略/稳定器保持原基线。静态靶距离为0.3、0.5、1、2、3、5m；随后使用同一轮式策略接收0.08m/s前进设定、持续2.5仿真秒。该速度是本阶段测试输入，不是硬件规格。
- 项目装配中的Gemini/OS0外观子树在本次测量运行中关闭，避免外壳挡住公共RGB；传感器测量节点、机器人动力学网格与控制照常工作。机器人网格可见/隐藏对照用于自遮挡评估，碰撞、控制和场景靶保持不变。
- 仅仿真与本机ROS 2离线采集，没有实机连接或运动命令。无LLM、运动策略重训、SDK下载或模型权重下载。

## Isaac API与旋转扫描的判定

实测环境是本机 Isaac Sim 6.2.0，RTX LiDAR扩展为 `omni.sensors.nv.lidar 3.4.0+187cf21b`。本机 `isaacsim.sensors.experimental.rtx.Lidar` 源码说明：`accumulateOutputs=True` 会累计完整扫描后才输出；配置tick/scan rate为10Hz。本次渲染循环观测约60Hz，输出间隔中位数0.100000s；每个点云的逐点时间戳跨度中位数99.939ms。

因此，本次测到的是跨约100ms累计的一圈点云，不是渲染每一帧同时发射360°全部射线。低速移动时机器人基座前进约0.2505m/2.5s，实测中位前向速度约0.1080m/s；按实测速度和逐点时间跨度估算，一次完整扫描期间机器人移动约10.8mm。点云图用点时间着色，保留了扫描先后顺序。本阶段没有做去畸变，原始点云也没有用姿态真值改写。

边界：配置含0–360°旋转模式，但输出 `azimuth` 数组为空，所以不能从逐点方位角独立复算每个角度或证明CW旋转方向。完整圈的判定来自本机API的累计语义、配置和每圈约100ms时间跨度；这验证了本机的时间扫描模型，不构成硬件转速、雷达精度或运动补偿验收。每圈理论发射数为64×512=32,768；实际每输出只含12,608–20,029个有效命中点，未命中射线不在点云里。所有输出的最大唯一 `emitterId` 数达到64，但它不等价于已校准ring字段。

## 本次测量结果

静态误差按本次每点命中里，与已知靶面轴坐标相差不超过8cm的回波计算；用运行时LiDAR传感器位姿补偿安装原点相对机器人坐标原点约−5.4cm的偏置。该位姿来自Isaac仿真API，作为离线评估真值使用，不代表真实外参测量。CSV保留每距离的回波数、MAE/RMSE和偏差。

| 靶面距离 | 靶面回波数（3圈合计） | 靶面轴坐标MAE | RMSE |
|---:|---:|---:|---:|
| 0.3m | 16,600 | 0.0118m | 0.0176m |
| 0.5m | 29,333 | 0.0073m | 0.0124m |
| 1m | 24,130 | 0.0116m | 0.0165m |
| 2m | 18,309 | 0.0174m | 0.0228m |
| 3m | 14,408 | 0.0203m | 0.0259m |
| 5m | 8,609 | 0.0234m | 0.0290m |

靶面误差表是理想靶面上的仿真几何结果，不含实机标定、材料反射、光子噪声、运动补偿和厂家精度误差。0.3m靶面中心处于0.5m最小量程内；斜向打到靶面边缘的射线可能因实际斜距大于0.5m而回波，因此不能把“0.3m无效”误读成整个平面绝对无回波。有效最近回波约0.500007m。小于最小量程的无回波不代表自由空间。

### IMU

IMU来源为Isaac `IMUSensor` 物理传感API，装在阶段2配置的传感器支架原点；未启用厂家Ouster IMU，也没有噪声、偏置、标定误差、时钟偏差或温漂。API返回 `time`、`physics_step`、`linear_acceleration`、`angular_velocity`、`orientation`：加速度以米制stage的m/s²计，静止时测得重力比力约+9.81m/s²；角速度为rad/s；姿态是 `[w,x,y,z]` 全局四元数；加速度和角速度按IMU传感器局部轴输出。

物理步进计数与时间差得到120Hz更新；本脚本在60Hz渲染循环读取并保存，所以本次保留的300个离线样本时间间隔约16.667ms，不把60Hz读取率冒充120Hz发布记录。IMU姿态没有用于点云去畸变。用于靶距修正、相机/雷达视锥投影和ROS TF的是Isaac传感器/机器人仿真真值；这些都是评估/坐标联接条件，不是由真实IMU估计的姿态。

### 原始点云字段

每圈以 `raw/scan_NNNN.npz` 原样保存非空数组；单点字段从本次API读取为：

| 字段 | 本次数组 | 说明 |
|---|---|---|
| `data` | `float32[N,3]` | XYZ；本机API默认 `transformPoints=False`，输出传感器局部坐标。本次已用静态靶验证轴0正向对应机器人前方。 |
| `timestamp` | `uint64[N]` | 纳秒级逐点时间戳；用于确认扫描跨度。原始值在NPZ保留。 |
| `emitterId` | `uint32[N]` | 原始发射器标识，不改名成ring或通道号。 |

本次下列数组字段存在于buffer定义，但长度为0、不能提供：`azimuth`、`channelId`、`distance`、`echoId`、`elevation`、`hitNormal`、`intensity`、`materialId`、`objectId`、`radialVelocityMS`、`tickId`、`tickState`、`velocity`。全零XYZ占位点从有效点云和误差统计中排除；原始NPZ仍保留。`rendering_time`、`rendering_frame`、仿真时间、传感器/机身/相机位姿在 `capture_summary.json` 的每圈记录中保存。

## 自扫与相机共视

- 在1m静态靶场景下，保持碰撞和物理不变，只隐藏机器人可见网格后对比两圈。可见网格每圈中位有效点19,762，隐藏网格19,762.5，差−0.5点（−0.0025%）。用相对逐点时间+emitterId配对的射线约97.5%可匹配；两组配对距离MAE分别为0.076mm和0mm，P95分别为0.301mm和0mm，超过5cm变化的射线为0。当前场景未观察到显著机身网格遮挡变化。`objectId`为空，不能把单个回波标记为机身命中；0.5m近距盲区也使紧贴传感器的遮挡不可见，因此这不是“实机无自扫”的证明。
- 以公共RGB相机ROS光学坐标、运行时相机和LiDAR位姿投影静态点云，18圈中平均约12.65%的有效点落在80°水平视场及对应垂直视场内。这是几何视锥交集，不是逐像素RGB遮挡/语义对应验收。原公共RGB仍保存，未改相机外参；GUI截图和相机RGB是不同产物。

## ROS 2 PointCloud2

在真实数组字段确认后，本次ROS 2 Jazzy发布 `/go2w/lidar/points`，并由独立订阅器收到49条PointCloud2，校验字段布局、数据长度、点数和有限XYZ通过。header时间是扫描输出仿真时间，frame为 `go2w_lidar`；另发布由仿真传感器位姿计算的 `base_link -> go2w_lidar` TF。它是仿真真值TF，不是定位估计。

| ROS字段 | datatype / offset | 来源 |
|---|---|---|
| `x`,`y`,`z` | FLOAT32 / 0,4,8 | 原始 `data` XYZ |
| `time_from_scan_start` | FLOAT64 / 12 | 原始uint64纳秒转为从该圈最早回波开始的秒数；精确原始时间仍在NPZ |
| `emitter_id` | UINT32 / 20 | 原始 `emitterId` |

每点24字节。非有限/全零XYZ不会进入ROS消息，但保留在原始NPZ。没有伪造 `intensity`、`ring`、`channelId`，因为这些字段本次没有有效数组。未发布LaserScan：传感器是64线3D设备，没有指定2D平面或高度切片，不能把任意3D点云压平当安全地图。

## 复现与录制

完整仿真和自动离线分析（每次使用新目录；脚本默认用本机Isaac Sim 6.2和ROS2 Jazzy，域97且仅localhost发现）：

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/stage4_$(TZ=Asia/Shanghai date +%Y%m%dT%H%M%S)"
./scripts/run_stage4_lidar.sh "$RUN" --headless
```

在仿真运行前，另开终端用同一 `ROS_DOMAIN_ID=97` 订阅并检查一次PointCloud2（把`RUN`替换成终端1输出路径）：

```bash
cd /home/joy/桌面/RobotDog
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
python3 scripts/ros2/check_pointcloud.py --timeout 35 \\
  --output "outputs/sensor_pipeline/stage4_<run_id>/ros_pointcloud2_received.json"
```

录包时，在仿真开始前另开终端，录包放在Linux本机并用独立目录；结束仿真后按Ctrl+C正常收尾：

```bash
cd /home/joy/桌面/RobotDog
export ROS_DOMAIN_ID=97 ROS_LOCALHOST_ONLY=1
source /opt/ros/jazzy/setup.bash
BAG="outputs/sensor_pipeline/stage4_rosbag_$(TZ=Asia/Shanghai date +%Y%m%dT%H%M%S)"
ros2 bag record -o "$BAG" /go2w/lidar/points /tf /tf_static /clock \\
  /go2w/front_camera/image_raw /go2w/front_camera/camera_info
```

运行期间可在ROS终端查看话题/频率：`ros2 topic info /go2w/lidar/points --verbose`、`ros2 topic hz /go2w/lidar/points`。用户选择录包时，录包输出保留本机，不放共享盘。

## 输出目录与验收表

```text
outputs/sensor_pipeline/stage4_<run_id>/
  raw/scan_NNNN.npz                 原始非空XYZ/timestamp/emitterId数组
  capture_summary.json              采样时间、IMU、位姿、命中字段和ROS输出摘要
  imu_samples.json                  仿真IMU逐次读取样本（60Hz读取）
  motion_base_samples.json          低速测试的机身位姿/速度记录
  static_distance_error.csv         0.3/0.5/1/2/3/5m静态靶面误差
  scan_timing.csv                   每圈点数、frame time、时间跨度/间隔/emitterId覆盖
  camera_lidar_frustum_overlap.csv  静态LiDAR点落入公共RGB视锥比例
  self_scan_comparison.json         可见/隐藏网格同场景对照与射线配对误差
  motion_points_raw.npy              一圈原始移动XYZ样本
  motion_pointcloud_timestamp.png   原始移动点云按逐点时间着色
  pointcloud2_sample.bin/json       实际ROS PointCloud2字段布局/首条消息数据
  ros_pointcloud2_received.json     独立ROS2订阅校验
  public_rgb_after_motion.png       公共相机RGB样本，不是GUI截图
  isaac_capture.log, analysis.log   原始运行和离线分析日志
```

| 检查 | 结果 |
|---|---|
| 60Hz渲染、10Hz扫描输出 | 渲染60.0Hz；扫描输出间隔0.100000s；逐点跨度99.939ms |
| 输出圈数与命中点 | 49圈；每圈12,608–20,029点；唯一emitterId中位数62、最大64。点数是命中数，不是32768条射线的发射计数 |
| 静态靶 | 0.3/0.5/1/2/3/5m各3圈；靶面关联点MAE为1.18/0.73/1.16/1.74/2.03/2.34cm |
| 0.5m近距 | 最近有效返回0.500007m；0.3m靶中心落在盲区，斜射边缘仍可产生回波 |
| 低速移动 | 命令0.08m/s×2.5s，机身前进0.2505m；扫描期间估算移动约1.08cm；原始时间着色点云已保存，无去畸变 |
| IMU | 300个60Hz读取样本，physics_step推得120Hz更新；300/300有效；理想无噪声 |
| 自扫检查 | 网格可见/隐藏点数差−0.5，匹配射线中0条变化超过5cm；对象ID未提供，近距盲区限制仍存在 |
| 公共RGB视锥交集 | 18个静态圈，平均12.65%雷达有效点落入RGB角视锥；未做像素级遮挡比较 |
| ROS 2 PointCloud2 | 发布49条；独立订阅收到消息、布局/数据长度/有限XYZ检查通过；无ring/intensity |
| LaserScan | 未发布；无2D扫描面/指定切片 |
| GUI人工检查 | 本次无窗口运行，未测试 |

## 未完成、受限与重试

- 未验证厂家级OS0 REV8波束角/强度/反射率/转速误差、实机IMU噪声与时钟同步。`IsaacCreateRTXLidarScanBuffer`在当前Isaac版本仍可用但官方源码标为deprecated，日志提示未来改用`GenericModelOutput`；后续升级前需迁移并重新验证字段，不能沿用当前字段假设。
- 无 `azimuth`、`channelId`、`intensity`、`objectId`，所以方位角方向、每点ring、强度和单点自扫分类无法直接复算。ROS时间字段是相对圈起点秒，原始纳秒只在NPZ。
- 传感器坐标通过静态靶和API传感器位姿确认；ROS TF使用仿真真值。没有SLAM/状态估计；没有用IMU真值对运动点云去畸变。
- `stage4_20261026T212831_01`因输出目录被外层预建而未启动采集；`212851_02`渲染帧字段为tuple触发类型错误；`212917_03`漏建首个静态靶；`212938_04`和`213002_05`使用含小数点的USD prim名称失败。均保留日志，不计验收。
- `214127_12`使用了未加载Bundled Jazzy路径的通用启动器，Isaac内`rclpy`导入失败，独立订阅器没有收到消息；修正为`run_go2w_ros2.sh`后，`214216_13`证明了PC2收发，但ROS空命令覆盖了阶段测试速度；将测试速度覆盖移到ROS命令合并之后后，`214318_14`实测移动通过；将点时间字段重命名为无歧义的`time_from_scan_start`后，以`214509_15`作为最终验收。所有旧run均保留，不覆盖。

最终证据：[capture_summary.json](../../outputs/sensor_pipeline/stage4_20261026T214509_15/capture_summary.json)、[逐圈时间表](../../outputs/sensor_pipeline/stage4_20261026T214509_15/scan_timing.csv)、[静态误差CSV](../../outputs/sensor_pipeline/stage4_20261026T214509_15/static_distance_error.csv)、[独立ROS订阅校验](../../outputs/sensor_pipeline/stage4_20261026T214509_15/ros_pointcloud2_received.json)、[Isaac日志](../../outputs/sensor_pipeline/stage4_20261026T214509_15/isaac_capture.log)、[分析日志](../../outputs/sensor_pipeline/stage4_20261026T214509_15/analysis.log)。
