# 05 时空投影与对象级双目/雷达融合

状态：阶段5开发场景完成。最终可重跑采集/分析目录：`outputs/sensor_pipeline/stage5_20261026T223623_accepted/`。人工多边形几何检验目录：`stage5_20261026T222539_capture1/`，其图像SHA绑定标注见 `configs/sensors/stage5_manual_polygons_dev.json`。这些都是开发场景，尚无独立留出测试或实机标定。

## 输入、坐标与时间定义

- 沿用阶段1通用64发射器×512列、10Hz、0.5–20m的Isaac RTX旋转功能代理；沿用阶段3的公共 `FrontCamera` 和640×480、10Hz、10cm基线理想左右RGB。阶段2的厂家USD只提供外观，本次采集禁用外观子树以免挡住公共相机。原轮式控制器/权重不改；阶段5只增加静止、直行、转弯、停车的测试指令。未下载SDK或权重，不连接实机。
- 世界/机身与LiDAR采用米制，`+X`前、`+Y`左、`+Z`上。公共/左相机使用Isaac `camera_axes="ros"` 的光学坐标：`+x`图像向右、`+y`向下、`+z`沿光轴向前。实际保存姿态的静止帧数值验证：光学`+x`约对应世界`−Y`，`+y`约对应`−Z`，`+z`约对应`+X`。四元数保存为`[w,x,y,z]`，单位长度为m。
- `T_world_lidar(t)` 将LiDAR局部点转为世界点；`T_world_camera(t_e)` 将相机光学点转为世界点。因此这里的 `T_camera_lidar(t_p,t_e) = inverse(T_world_camera(t_e)) · T_world_lidar(t_p)`；`p_camera = T_camera_lidar · p_lidar`。时间` t_p`取原始 `uint64` 逐点纳秒转换的仿真秒，`t_e`取公共相机帧的 `rendering_time`，扫描结束取该圈最大逐点时间。相机位姿在约60Hz逐帧保存轨迹上插值到曝光时刻，LiDAR位姿插值到每个点的采集时刻。阶段5用的是Isaac真值位姿的 **oracle去畸变开发条件**，没有用理想IMU姿态冒充可部署状态估计。
- 运行时仿真真值外参是基础条件，没有完成实机外参标定。估计外参敏感性实验仅对读入点的估计LiDAR局部位姿额外施加`x=0/1/3cm`与绕局部`z=0/0.5/1.5°`扰动；保存的场景、图像和原始点云从未改变。

相机是零畸变理想针孔。公共相机点用实际API内参`K`和显式零畸变`D`经`cv2.projectPoints`投影；`Z≤0.05m`、非有限及画面外点被剔除。左右图用固定阶段3参数调用`cv2.stereoRectify`及StereoSGBM，视差按OpenCV定点`/16`转换，左右一致性与无效深度仍按阶段3规则处理；输出是公制光轴Z。左右/公共相机不是同一个光心：双目点先从左校正光学坐标转回左相机，再用曝光时刻外参转到公共RGB光学坐标。`evaluation_only_left_geometric_z_m.npy`只做独立评估留存，匹配和融合函数均不读取它。

## 同步、遮挡与对象接口

本次相机与扫描器只共享仿真时钟，没有配置硬件触发。每次观测对最近一圈扫描结束时刻与公共RGB曝光时刻做软件配对；允许的绝对差不超过50ms。最终4组被选帧的`相机−扫描末点`差分别为`−49.921、+0.079、−49.921、−16.557ms`。所有60圈与最近相机曝光的软件配对差绝对值中位33.254ms、P95约50.079ms；这不能称为硬件同步。去畸变另使用逐点时间和oracle位姿，解决一圈内机身运动，与硬件同步/软件配对属于不同问题。

目标是固定在世界`[2.3,0,0.55]m`的红色纹理面板。对象位置真值只在完成对象估计后计算误差。第一轮先目视公共RGB，按像素手绘4个矩形多边形并用图片SHA锁定；它们仅适用对应开发run。随后采用固定红色分量规则、形态闭合和最大连通轮廓作经典无训练分割，最终run使用该自动分割。首轮手绘与自动掩码IoU依次为`0.9767/0.9738/0.9648/0.9697`；对象中心差为`0.0004/0.0271/0.0005/0.0005m`，直行帧边界选择差较明显，保留在报告中。这里没有声称已接入通用检测模型。

LiDAR关联先做相机后方/画面外剔除，再按投影像素选最近回波；落入对象掩码后只取前方20%深度分位数附近12cm的层，拒绝更后方的背景回波。`valid_point_count`只计通过该前层的点；LiDAR覆盖率为这些点投影后以3像素半径支持的对象掩码面积比例，双目覆盖率为分割掩码中SGBM有效像素比例。不会把检测框内所有点无条件平均，也不把无效深度当自由空间。该近层规则和单面板场景是开发基线，尚未证明复杂遮挡鲁棒性。

统一对象JSON位于`object_geometry.json`，每段都有`lidar_only`、`stereo_only`、`hybrid`三条记录。字段包括：`object_id`、`position_m`、`frame_id`、`acquisition_time_s`、`depth_source`、`valid_point_count`、`coverage_fraction`、`quality`/`uncertainty_m`、`evidence`/`evidence_indices`、`conflict`及`request_reobservation`。这里`uncertainty_m`只是对象光轴深度MAD且下限1cm的质量代理，不是校准协方差，也不假定雷达与双目误差独立。两路均有效且中心差≤开发门限10cm时，`hybrid`只选择质量代理较小的一个位置并保留另一路作为一致性证据，不做简单平均；超过门限或证据不足则输出`unknown`。

## 最终开发运行结果

本次无窗口运行实际输出60圈，四段各1组公共RGB/左右RGB/原始LiDAR点云；转弯段机身累计偏航`0.2564rad`（约14.7°），路径长度约0.146m。直行段路径约0.101m。停车后仍有约0.012m微小路径与`−0.0188rad`姿态回弹，因此“停车”是控制设定为零的观察段，不把它当绝对静止标定。

| 阶段 | LiDAR点数 / 3px覆盖 | 双目有效像素 / 覆盖 | LiDAR中心误差 | 双目中心误差 | 融合结果 |
|---|---:|---:|---:|---:|---|
| 静止 | 513 / 58.7% | 34,234 / 100% | 1.80cm | 1.83cm | 选LiDAR，1.80cm |
| 直行 | 550 / 61.2% | 34,509 / 100% | 2.51cm | 2.90cm | 选双目，2.90cm |
| 转弯 | 572 / 54.8% | 42,798 / 100% | 4.79cm | 5.35cm | 选双目，5.35cm |
| 停车观察 | 476 / 42.0% | 40,588 / 81.3% | 13.25cm | 4.78cm | 两中心差16.69cm，`unknown`并请求重观测 |

表中误差是相对于面板表面中心的三维欧氏距离，**只用于评估**。融合并未保证比单传感器误差更低；尤其停车帧的冲突应保留。较强转弯前的`stage5_20261026T223357_final`只转了1.9°，未作为最终动态观察。`stage5_20261026T223450_turn_final`转到14.7°却因开发门限0.20m错误接受停车LiDAR；该run与结果原样保留，随后将门限收紧到0.10m并在全新`223623_accepted`目录重跑。此处门限是在开发场景调整的，不能将最终run称为独立测试集。

按每点时间的oracle位姿投影与把整圈当扫描结束时刻的简化投影相比，静止/直行/转弯/停车对象中心差分别为`0、1.10、4.82、0.11cm`。这说明转弯时忽略逐点时间会产生可见差异；它没有证明真实IMU/状态估计可达到oracle精度。

估计时间偏移只在分析侧施加`0/10/30/50ms`。LiDAR定位误差（m）按此顺序：静止`0.0180/0.0180/0.0180/0.0180`，直行`0.0251/0.0254/0.0260/0.0269`，转弯`0.0479/0.0458/0.0415/0.0362`，停车`0.1325/0.1326/0.1329/0.1331`。转弯误差非单调，因像素掩码、前层选择与局部运动同时改变，不能解释为偏移越大越好。仅改变估计外参时，四段误差范围分别为`0.0167–0.0391`、`0.0266–0.0477`、`0.0053–0.0557`、`0.0621–0.1307m`；具体48条开发扰动见`sensitivity.csv`，没有用保留测试集调参。

核心逻辑检查5项通过：光轴中心投影、相机后方剔除、越界剔除、同像素保留最近回波、冲突返回unknown。原始帧没有自然冲突的前三段正常保留有效输出；停车段是真实采集间的自然冲突。另保存一个仅对**估计结果**注入+0.5m双目Z偏置的合成冲突，算法也返回unknown；该合成注入没有修改传感器原始数据。

## 运行与查看

每次用新的输出目录，本机Isaac/Python直接运行；该命令不启动ROS实机接口：

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/stage5_$(TZ=Asia/Shanghai date +%Y%m%dT%H%M%S)"
./scripts/run_stage5_fusion.sh "$RUN" --headless
```

只用已保存观测重算表和投影图，不再启动仿真：

```bash
cd /home/joy/桌面/RobotDog
/home/joy/robotics/isaac-env/bin/python scripts/sensor_pipeline/analyze_stage5_fusion.py \
  --run outputs/sensor_pipeline/stage5_20261026T223623_accepted
```

分析器会写同名图/表；如要保留旧分析，先复制整个run为新分析目录或先另存旧分析文件。本次迭代的旧结果已保留在对应旧run，不静默删除。查看图形可运行：

```bash
xdg-open outputs/sensor_pipeline/stage5_20261026T223623_accepted/captures/static/projection_oracle_deskew.png
xdg-open outputs/sensor_pipeline/stage5_20261026T223623_accepted/captures/turn/projection_oracle_deskew.png
xdg-open outputs/sensor_pipeline/stage5_20261026T223623_accepted/sync_delta_histogram.png
```

每个`captures/<phase>/`含`public_rgb.png`、`left_rgb.png`、`right_rgb.png`、`lidar_raw.npz`、`capture.json`、自动分割掩码、SGBM光轴Z及oracle/不去畸变两张投影图。根目录含`trajectory.json`、`scan_clock.json`、`imu_clock.json`、`object_geometry.csv/json`、`frame_reports.json`、`sensitivity.csv/json`、`sync_differences.json`、`motion_observed.json`、`core_logic_checks.json`、`conflict_case.json`和完整`isaac_capture.log/analysis.log`。失败或不合格的开发run仍在`outputs/sensor_pipeline/`，不覆盖。

## 验收边界与下一阶段输入

- 已完成：单机器人同场景三路RGB与完整LiDAR扫描采集，真实逐点时间字段、相机内参与运行时外参方向核对，静态/直行/转弯/停车投影和对象接口，手绘掩码几何校验后改用无训练经典分割，冲突unknown与时间/外参开发扰动，日志/图/表可重算。
- 受限：动态点云到曝光时刻的位姿来自Isaac仿真真值，未接估计姿态/IMU积分；无硬件触发或真实镜头标定。当前对象是单个固定红色平面，未测试运动物体、多对象互遮挡、复杂材质或语义检测模型。公共RGB与双目仍是理想独立光学系统；没有真实设备RGB/IR对齐。本次无窗口检查，没有GUI人工交互验收。
- 下一阶段需要独立于开发场景的纹理、距离、运动与遮挡留出样本，估计位姿来源及对应误差预算，或用户确定真实设备后提供实测内外参/触发方式。未拿到这些输入前，动态融合只作为oracle开发链路，不作为安全地图或可靠自主导航观测。

最终证据：[对象表](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/object_geometry.csv)、[采集日志](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/isaac_capture.log)、[分析日志](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/analysis.log)、[同步分布](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/sync_delta_histogram.png)、[静态投影](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/captures/static/projection_oracle_deskew.png)、[转弯投影](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/captures/turn/projection_oracle_deskew.png)、[停车冲突](../../outputs/sensor_pipeline/stage5_20261026T223623_accepted/object_geometry.json)。
