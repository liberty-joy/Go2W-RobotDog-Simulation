# 08 训练必要性、参数扫描与独立最终评估

执行日期：2026-10-30（Asia/Shanghai）。最终有效run：`outputs/sensor_pipeline/stage8_20261030_no_training_v2/`。**结论：当前不需要训练。** 本阶段真正执行的是经典StereoSGBM推理、36项参数扫描、独立验证和冻结后的最终评估，没有神经网络拟合或运动策略适配。这个结论不等于当前深度已经足够可靠。

## 1. 读完0–7后的实际瓶颈

|已有证据|实际瓶颈|是否能由当前训练直接解决|
|---|---|---|
|阶段0–2环境/资产已运行，传感器为功能代理|版本/API、真实设备参数/噪声/同步尚不完整|训练不能替代驱动、标定和测量模型|
|阶段3旧留出重复纹理MAE1.435m；本阶段新最终集仍严重失败|纹理缺失/周期歧义、匹配质量与覆盖率权衡|有潜在学习方向，但当前数据不足以证明迁移模型的稳定收益|
|阶段4/5扫描跨约100ms；有逐点时间但动态补偿曾用oracle|时间语义、真实姿态/去畸变与数据归档|不应通过网络掩盖坐标/时序错误|
|阶段6 A24/24、B15/24有效，9次模态冲突|错误深度与保守冲突门控导致unknown|先检查匹配/质量门控；不能把放宽门限叫学习收益|
|阶段7 A/B直接各2/3、证据各3/3，失败是证据过期|证据时效与重观察执行流程；没有双目完成数收益|不是重训Go2-W步态的依据|
|阶段7设备属性人工登记，ImageNet首类曾为prayer rug|缺少真实设备属性数据与任务域评估|尚不能凭类名错误就训练一个无独立数据验证的分类器|

历史文件逐项路径/SHA见`history_evidence.json`；不是把过去通过记录算作本次复验。本次重新验证的是匹配/参数/新静态样本，不重报ROS、导航或GUI通过。

三类训练候选均暂缓：轻量双目迁移没有已验证的成本收益；深度质量预测器缺足够独立场景的错误标签分布，现成LR检查/规则门控应先评价；小型属性识别缺真实设备类别数据。用相邻视频帧造大量patch并随机拆分，会掩盖场景泄漏。仿真数据可作为未来训练标签，但本次不据此声称实机泛化。

## 2. 预注册、划分与预算

运行前已写入[训练必要性协议](../../experiments/protocols/sensor_training.md)。候选数为3×2×3×2=36，运行前纠正了最初54的算术笔误，没有改变网格。

|用途|场景/对象范围|数量|
|---|---|---:|
|开发调参|只读阶段3干净开发1m/2m四类；不是历史测试|8|
|新独立验证|1.5m/2.75m，seed881031，宽高因子1.65/1.45，箱0.30m、偏移0.35m|8|
|新最终测试|3.5m/4.75m，seed881032，宽高因子1.55/1.35，箱0.25m、偏移0.30m|8|

各分区物理距离、尺寸/对象实例、seed隔离；每场景只采1组公共/左右图，不随机打散连续视频。无纹理/重复类不因换seed而自动变新纹理，所以还明确改变了几何/遮挡。它们仍属于同一程序生成器，不是跨真实设备/材质的强域泛化测试。

Stage3底层文件夹仍用`heldout_test_...`前缀；本阶段用途由独立run路径、`stage8_partition`配置和`validation/final_test`清单确定。验证真值只选晋升，测试真值只用于冻结后的评分。实际检查：最终图像文件创建时间晚于`final_selection.json.freeze_epoch_ns`，两个新分区各8/8同步、尺寸640×480。

本次使用已有Isaac Python3.12.3、OpenCV4.14.0、NumPy2.3.1；匹配固定4 CPU线程。开发先1候选×1场景冒烟，然后每候选每场景1次预热+2次测量，重复结果完全一致。开发扫描约26.53s，硬上限180s；各采集180s硬上限，无自动无限重试。全部自动流程在15分钟预算内完成。离线RAM规划≤4GiB，报告进程写decision时RSS高水位约76MiB，**不是整个渲染/扫描流程的实测峰值**；渲染GPU没有本次独立采样，不沿用历史数值。未安装环境/驱动/SDK。

## 3. 参数扫描与验证选择

固定网格：block_size={3,5,7}，num_disparities={64,128}，uniqueness_ratio={5,10,20}，LR阈值={1.0,1.5}px；P1=8b²、P2=32b²联动。其余沿用阶段3，视差/16一次、公制光轴Z、无效NaN，未改标定/投影/融合/控制。

原baseline：b5/n128/u10/LR1.5；开发得分最高挑战者：**b7/n64/u5/LR1.5**。逐场景等权主要指标是“有效且绝对误差≤10cm像素 / 真值有效像素”，坏接受是“已接受且误差>20cm像素 / 真值有效像素”。不能把低覆盖带来的低MAE冒充整体更好；同时报告总覆盖、条件MAE/RMSE、坏接受、遮挡和目标平面深度带诊断。深度带只作评估代理，不是真实语义mask，不进入匹配。

晋升要求开发好深度覆盖至少+1个百分点、坏接受不超过基线+1个百分点、延迟约束通过；独立验证也需好深度至少+1个百分点且坏接受增幅≤1个百分点。验证只比较开发选定的1个挑战者与baseline，不再次网格搜索。

|分区/方法|好深度覆盖≤10cm|总有效覆盖|坏接受>20cm|场景等权有效像素MAE|
|---|---:|---:|---:|---:|
|开发baseline|21.88%|25.10%|2.97%|0.163m|
|开发挑战者|39.14%|48.18%|8.82%|0.159m|
|新验证baseline|29.70%|35.73%|5.29%|0.217m|
|新验证挑战者|43.32%|59.59%|15.52%|0.343m|

**挑战者未晋升，最终主配置保持原baseline。** 它确实更快、覆盖更高，但验证坏接受大幅上升，不能只选漂亮指标。`development_selection.json`及`final_selection.json`记录全部选择依据；没有修改原`configs/sensors/stage3_sgbm.json`或阶段7运行配置。

## 4. 冻结后的新最终评估

在验证拒绝挑战者并冻结主配置后，才启动新测试采集；最终仅评估先前冻结的baseline和挑战者，不用测试选best。

|最终测试方法|好深度覆盖≤10cm|总有效覆盖|坏接受>20cm|有效像素MAE|CPU P50/P95|
|---|---:|---:|---:|---:|---:|
|baseline（最终主配置）|35.97%|42.99%|4.48%|0.259m|27.88/32.30ms|
|挑战者（仅诊断，不晋升）|57.01%|65.30%|5.23%|0.207m|20.20/22.34ms|

挑战者在这个新测试集部分指标更好，**仍不据此回选或覆盖主配置**；它在独立验证已失败。这说明少数程序场景间方差很大，需要更充分的独立场景/评价，而不是直接开始神经网络训练。每类仅2个距离样本，时延也仅8场景，不作显著性或一般实时性能承诺。

最终baseline四类：理想纹理MAE0.0426m/coverage49.57%，前箱遮挡0.0426m/48.56%，重复纹理**0.878m/44.91%**，无纹理0.0732m/28.91%。挑战者重复纹理仍约0.682m，问题没有被调参消除；低纹理的有效像素MAE较小也不能代表空洞区域可靠。

`failure_examples.png`按每类baseline误差最大的场景展示，灰色无效，色条截到0.5m但CSV保留全误差；没有只展示最漂亮远距样例。`parameter_scan.png`是**调参热图**，不是训练曲线；`final_comparison.png`和逐场景/逐类CSV可重算。

## 5. 训练/推理/调参的明确区分

- **真正训练：0步**。无网络/损失/优化器、无新训练权重、无best/last、无训练失败或loss曲线；不创建假的checkpoint。
- **新增传感神经网络推理：无**。只运行经典StereoSGBM匹配与调参；未下载轻量双目模型、质量网络或属性模型。既有SqueezeNet权重保持原样，本轮不运行它。
- **采集期间既有运动策略推理：有**，使用原wheel_slope导出策略保持站姿，权重冻结；不把已有JIT执行叫新训练。离线`metadata/decision.neural_inference=false`指匹配进程，不代表采集程序完全不执行既有运动策略。
- **调参：36项**，独立保存`selected_params.json`；**最终冻结：final_params.json**仍为原baseline。参数JSON不是神经权重。
- 本轮传感器没有新增质量/惯量，也无运动验收失败依据，未另立动力学适配实验。94项受保护策略/模型/控制/配置/关键资产SHA全部未变；用户已有`RobotDog.code-workspace`脏改动保持原样，不提交或覆盖。

## 6. 失败、版本和证据结构

首轮`stage8_20261030_no_training_v1`已完成开发扫描，随后1.5m场景把点号带入USD prim名，验证采集失败。日志和`failure_pipeline.json`保留；采集器仅把小数标识点号替为p，原整数命名不变，原matcher不变。完整重跑v2，没有静默删除失败。其他报告修订包括JSON快照换行比较、图示标题/无效颜色，分别有修订前后SHA记录，不改评分/选择规则。

```text
stage8_20261030_no_training_v2/
  metadata.json, events.jsonl, dirty_before.patch
  history_evidence.json, source_manifest.json, source_snapshot/
  protected_before.json, protected_after.json
  smoke.json, candidate_results.json/csv, development_per_scene.csv
  baseline_params.json, selected_params.json, development_selection.json
  validation_capture/           # 新验证原图/独立真值/CameraInfo/实际日志
  validation/                  # 两种冻结候选的估计、指标与输入SHA
  final_params.json, final_selection.json
  final_test_capture/           # 冻结后才采的新最终场景
  final_test/                   # 逐场景/逐类CSV、估计Z/视差/无效mask
  decision.json, acceptance.json, post_checks.json
  parameter_scan.png, final_comparison.png, failure_examples.png, results.html
```

`source_snapshot`是运行时快照，报告图示修订另存`report_only_amendment.json`；本次最终代码由Git提交标识。`metadata.json`记录父提交/解释器/库版本，`dirty_before.patch`保留启动时包括用户工作区文件在内的差异。历史0–7结果仅引用归因，不冒充本次重新验收。

## 7. 命令行查看与复现

查看本次结果：

```bash
cd /home/joy/桌面/RobotDog
RUN=outputs/sensor_pipeline/stage8_20261030_no_training_v2
xdg-open "$RUN/results.html"
xdg-open "$RUN/failure_examples.png"
column -s, -t "$RUN/final_test/per_class.csv"
cat "$RUN/final_selection.json"
```

完整新目录重跑（36项扫描、8验证、8最终，不训练）：

```bash
scripts/run_stage8.sh "outputs/sensor_pipeline/stage8_$(date +%Y%m%dT%H%M%S)"
```

只重画已有结果，不重算/改选参数：

```bash
/home/joy/robotics/isaac-env/bin/python scripts/sensor_pipeline/stage8_no_training.py report \
  --out outputs/sensor_pipeline/stage8_20261030_no_training_v2
```

最终匹配接口仍是 `process_stage3_stereo.run_matcher(left_bgr, right_bgr, calibration, params)`，只接收左右图、固定内外参和`final_params.json`。返回`depth_z_m`（float32米、光轴Z）、`valid_mask`、视差等；NaN表示未知。调用例（不读取真值）：

```python
import sys, json, cv2
from pathlib import Path
sys.path.insert(0, 'scripts/sensor_pipeline')
from process_stage3_stereo import load_image_pair, run_matcher
cv2.setNumThreads(4)
r = Path('outputs/sensor_pipeline/stage8_20261030_no_training_v2')
data = r / 'final_test_capture'
scene = json.loads((data / 'captures.json').read_text())['rows'][0]['scenario_id']
left, right = load_image_pair(data, scene)
result = run_matcher(left, right,
    json.loads((data / 'calibration.json').read_text()),
    json.loads((r / 'final_params.json').read_text()))
# result['depth_z_m'] 与 result['valid_mask'] 是估计，不是真值。
```

## 8. 最终评估交接与剩余问题

本阶段已进入并完成冻结参数的新静态场景最终评估。主配置继续用原SGBM；不自动部署挑战者、不重训运动策略、不把调参叫训练。下一阶段可直接读取`final_params.json`、`final_selection.json`、新最终CSV和旧阶段7固定导航配置，另行预注册更大范围/多种子评估；本轮没有重跑新的闭环任务，也不把像素覆盖率换算成自主任务成功率。

还缺：代表性真实IR/RGB噪声与同步、真实设备/材质/类别数据、更大独立场景、多对象/复杂遮挡验证、可部署动态去畸变及工业属性识别。重复纹理深度仍不可靠，NaN不能当自由空间；“当前不需要训练”只是当前证据不足以支持训练投入，不是宣称已经解决这些限制。若未来要训练，重新确认单一训练对象与协议，保留经典/未微调预训练基线，测试不选best。

自动流程首轮完成时间为15:50:30，启动15:49:33，约56.43秒（不包括工程实现/排错）。CPU时延是run_matcher内部校正/匹配，不含PNG读取、评分和仿真渲染；不与其他阶段不同口径时延直接比较。

版本化共享导出：`/mnt/Shared/First1/1023Robot_Dog_Development/sensor_pipeline_stage8_20261030_v1`，只含文档/协议/参数/图表/汇总与选择记录，原始图像/深度/历史失败留Linux本地。没有复制环境/缓存或双向覆盖。
