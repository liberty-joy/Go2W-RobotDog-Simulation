# 03 同步双目、深度估计与留出评估

状态：阶段3已完成。干净开发运行`stage3_20261026T2110_clean_dev`，最终新seed留出运行`stage3_20261026T2118_clean_final`。最终运行使用修正后的相机投影矩阵，8/8留出样本通过同步与形状检查，正视差通过。

## 目标与假设

- 公共语义视点沿用 Go2-W 的 `FrontCamera`（640×480，原位 `(0.32, 0, 0.08)m`），每个样本另保存这一路公共RGB，A/B不因双目改变语义视点。
- 双目是两个Isaac理想RGB针孔相机，不是Gemini SDK、红外双目、真实成像噪声或厂家测量模型。分辨率640×480，10Hz，演示基线100mm，中心与公共RGB重合；左右相机分别置于机器人坐标 `+Y/-Y` 5cm。焦距18mm、水平视场80°，理想畸变系数全零。
- `K`由图像尺寸和80°水平视场计算，`fx=fy≈381.36 px`；主点通过Isaac `Camera.get_intrinsics_matrix()`实测API读取并与配置核对（中心为`width/2,height/2`）。外参为平行相机，OpenCV左到右变换`T=[-0.10,0,0]m`，预期左减右视差为正。CameraInfo等价JSON写出K/D/R/P、分辨率和frame_id；未伪造实机标定。
- 靶面与左右相机成像面平行，中心光轴Z分别为1、2、3、5m；前方箱体用于遮挡场景。机器人策略仍使用已有权重，只维持现有站姿，不训练运动策略。
- 1m/2m为开发集，3m/5m为留出集；四类（理想随机纹理、无纹理、周期重复纹理、前箱遮挡）均有独立场景。SGBM参数在开发集选定后冻结，留出结果只报告、不调参。

## 复现

```bash
cd "/home/joy/桌面/RobotDog"
RUN="outputs/sensor_pipeline/stage3_$(TZ=Asia/Shanghai date +%Y%m%dT%H%M%S)"
bash scripts/run_stage3_stereo.sh "$RUN" --headless
```

脚本拒绝覆盖已有目录，先以Isaac Sim本机Python启动Go2-W、公共RGB和双目相机；每个场景预热8步，要求左右和公共RGB时间戳在`1e-9s`内一致、图像都是640×480。采集完成后，使用同一Isaac环境内的OpenCV离线处理左右图并写入分析产物。若仿真在采集期间失败，原始日志/失败条目保留；不要复用失败目录。

单独重算离线结果（已有且非空的分析目录不会覆盖；可传`--analysis-dir`指定新的目录）：

```bash
/home/joy/robotics/isaac-env/bin/python scripts/sensor_pipeline/process_stage3_stereo.py \
  --run outputs/sensor_pipeline/stage3_<run_id> \
  --params configs/sensors/stage3_sgbm.json
```

## 算法与数据隔离

1. 仅由 `captures/<id>/left_rgb.png` 和 `right_rgb.png` 进入 `run_matcher`；左右按配置的K/D/R/T做 `stereoRectify` 与 `remap`。
2. OpenCV SGBM的signed int16视差含4位定点小数，先除以16一次；非正视差/特殊无效码剔除。
3. 反向匹配得到右视差，映射到 `x_right=round(x_left-d_left)` 并按1.5px阈值做左右一致性检查；越界、无效、被遮挡或不一致的像素不会进有效深度。`lr_rejected_mask`只表示由匹配结果推测的不一致区域，不等同于真实物体遮挡标签。
4. 有效深度按 `Z=fx_rect*B_rect/disparity` 计算，为公制光轴Z。无效深度写NaN，同时写 `valid_mask.npy`；不得填0或解释成自由空间。
5. Isaac `distance_to_image_plane` 几何深度注释器独立保存到 `ground_truth/<id>/`。SGBM处理函数返回后，评估器才读取它及几何前景遮挡mask。该真值不参与校正、视差、填洞或参数选择；包含近距地面遮挡，而非只用无限平面解析式。

## 输出结构

```text
outputs/sensor_pipeline/stage3_<run_id>/
  captures/<split>_<distance>m_<class>/
    left_rgb.png, right_rgb.png, public_rgb_common_semantics.png, capture.json
  ground_truth/<scenario>/
    left_optical_z_m.npy, geometric_occlusion_mask.npy
  calibration.json, camera_info_{public_rgb,left,right}.json, captures.json
  analysis/<scenario>/
    rectified_left.png, rectified_right.png, disparity_raw_fixed16.npy
    disparity_px.npy, depth_z_m.npy, valid_mask.npy, raw_valid_mask.npy
    lr_rejected_mask.npy, disparity_preview.png, depth_preview.png
    epipolar_disparity.png, depth_error.png, metrics.json
  analysis/per_scene.csv, split_metrics.csv, results.json
  isaac_capture.log, analysis.log
```

## 本次验收

运行ID、实际同步、视差方向、有效覆盖、误差和每轮失败在此补记；历史阶段结果不作为本阶段证据。

| 检查 | 要求 | 本次结果 |
|---|---|---|
| 640×480、左右/公共RGB同时间 | 左/右/公共RGB同一渲染时刻，时间差≤1e-9s | 干净开发8/8、最终留出8/8通过；时间差0s，shape均480×640×3 |
| CameraInfo等价JSON | 实测API内参、尺寸、K/D/R/P、frame_id一致 | 最终运行用`get_intrinsics_matrix()`校验三相机；D全零，右P Tx=`-fx*B=-38.1361`；非ROS话题/实机标定 |
| 基线/符号 | 中点重合、间距100mm、正视差 | 中点/间距检查通过；留出最近理想纹理场景正视差通过 |
| 四类场景开发与留出 | 逐场景CSV及split汇总 | 干净开发8场景+新seed留出8场景，四类分开 |
| 深度误差和覆盖率 | MAE/RMSE/覆盖率，遮挡分开 | 已输出；重复纹理误差大且覆盖低，作为能力限制保留 |
| GUI人工检查 | 若无GUI观察记录则不宣称人工交互通过 | 未执行（无窗口采集） |

试跑目录保留：`stage3_20261026T2102_01`输出目录冲突未采集；`stage3_20261026T2109_02`采图后因字段名错误处理失败；`stage3_20261026T2059_03`、`stage3_20261026T2106_final`、`stage3_20261026T2113_final`含固定演示红/蓝箱或初始相机信息问题，不作为最终开发/留出数据。所有失败日志保留。

干净开发集`stage3_20261026T2110_clean_dev`三候选（只读1/2m，选择分数为有效且绝对误差≤10cm的几何有效像素比例）：

| 候选 | 分数 | coverage | 有效像素MAE | 有效像素RMSE |
|---|---:|---:|---:|---:|
| baseline_default（冻结） | 0.2188 | 0.2510 | 0.1436m | 0.3898m |
| stricter_uniqueness | 0.1970 | 0.2238 | 0.1397m | 0.3863m |
| larger_support | 0.2120 | 0.2440 | 0.1521m | 0.4070m |

最终留出run`stage3_20261026T2118_clean_final`使用新纹理seed 84017、3/5m与冻结的baseline_default，未据测试误差改参：

| 场景 | coverage | MAE | RMSE | 平均处理时间 |
|---|---:|---:|---:|---:|
| 理想随机纹理 | 0.4726 | 0.1076m | 0.1598m | 26.0ms |
| 前景箱遮挡 | 0.4774 | 0.1050m | 0.1533m | 22.8ms |
| 周期重复纹理 | 0.2328 | 1.4351m | 2.4140m | 23.6ms |
| 无纹理 | 0.1593 | 0.0311m | 0.0641m | 22.9ms |

这是理想针孔/程序化纹理的离线结果，不是实机精度。重复纹理错误很大；无效像素保持NaN+mask，不能解释为自由空间。处理耗时含校正、双向SGBM和左右一致性，不包括ROS/导航传输，不作整机实时承诺。

## 失败条件和限制

- 任一图像缺失、分辨率错误、三路时间戳不一致、采集未完成，属于采集失败；不进入深度评估。
- 正视差符号检查失败意味着左右相机顺序/坐标映射有误；不可用绝对值掩盖。
- 纹理less低覆盖是算法输入缺少可辨纹理的限制，不用真值或插值伪装成成功深度。
- 周期纹理可能形成错误匹配；遮挡边界会触发LR不一致。左右一致性并不能识别全部错误匹配。
- 图像外观和目标几何是程序生成的理想代理；没有曝光噪声、镜头畸变、运动模糊、滚动快门、左右响应差异、双目相机实际内外参或实机SDK。其误差不能外推为硬件精度。
- 10Hz是本阶段采集预算设定，不是实时性能验收；处理耗时以每场景实测为准。

## 本次证据路径

- 最终开发集原始图/同步时间/几何真值/默认参数初始指标：[development run](../../outputs/sensor_pipeline/stage3_20261026T2110_clean_dev/)、[候选排名与冻结参数](../../outputs/sensor_pipeline/stage3_20261026T2110_clean_dev/tuning_dev/candidate_results.json)、[开发逐场景CSV](../../outputs/sensor_pipeline/stage3_20261026T2110_clean_dev/analysis/per_scene.csv)。
- 最终新seed留出集：[holdout run](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/)、[总结果JSON](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/analysis/results.json)、[逐场景CSV](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/analysis/per_scene.csv)、[最终Isaac采集日志](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/isaac_capture.log)、[离线处理日志](../../outputs/sensor_pipeline/stage3_20261026T2118_clean_final/analysis.log)。
- 示例目录`analysis/heldout_test_3m_ideal_textured/`包含左右校正图、极线与视差示意、深度估计和误差图、mask以及逐场景metrics JSON。其他15个干净样本均各有独立图和JSON。
- 干净开发/留出输出各约66/68MiB；旧的冲突、输入字段错误和被演示障碍污染的样本目录一并保留，禁止将它们当最终证据。

开发集候选比较（只读取1m/2m development样本，输出目录必须不存在；不读取heldout RGB/真值）：

```bash
/home/joy/robotics/isaac-env/bin/python scripts/sensor_pipeline/tune_stage3_sgbm.py \
  --run outputs/sensor_pipeline/stage3_20261026T2110_clean_dev \
  --output outputs/sensor_pipeline/stage3_20261026T2110_clean_dev/tuning_dev
```

候选参数冻结后，最终留出以新纹理seed 84017采3m/5m。最终逐场景CSV/JSON与图在run目录；处理只报告heldout split。测试集不参与候选排名。
