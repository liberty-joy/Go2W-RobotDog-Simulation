# 02 官方资产本地化与 Go2-W sensor_rig 装配

执行日期：2026-10-26（Asia/Shanghai）；本次 run：`outputs/sensor_pipeline/stage2_20261026T172917`。先读[阶段1选型](01_selection.md)。本阶段只完成资产追溯、独立装配和单场景测量冒烟；不做匹配深度、融合、训练、SLAM/Nav2或实机控制。

## 1. 本阶段实际选择与下载边界

| 分类 | 本阶段真正需要并实际处理的内容 | 结果 |
|---|---|---|
| 传感器USD/CAD外观资产 | Isaac Sim 6.2官方 Orbbec Gemini 335L USD；官方 Ouster OS0入口、网格与一张材质贴图 | 已本地化，供应商文件保持原样 |
| 测量模型/配置 | OS0入口实际引用的10个REV6/REV7配置；项目既有通用64线RTX配置和阶段2外参配置 | 已本地化/复用；最终测量明确为参数化代理 |
| 算法源码 | 本阶段无需双目匹配、融合或厂商SDK | 未下载 |
| 预训练权重 | 资产导入和传感器烟测不需要学习模型 | 未下载 |

官方6.2目录把 Gemini 335L 标为“Certified by Orbbec”，并列出IR左/右、RGB和深度相关相机配置；这只证明官方资产内容，不证明本项目已运行 Orbbec SDK 匹配算法。OS0官方6.2资产只提供REV6/REV7变体，本项目没有把它冒充REV8数字孪生。官方目录：[深度相机资产](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_depth_sensors.html)、[RTX雷达资产](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_nonvisual_sensors.html)、[传感器目录](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_sensors.html)，访问日2026-10-26。

下载前检查：根分区约167 GiB可用，`~/.cache/ov`约561 MiB；未把不透明缓存当官方来源。最终供应商目录约49 MiB，不下载多个SDK或模型仓库。精确URL、Isaac版本、许可说明、每个文件大小和SHA-256见 [SOURCE.json](../../sim/assets/sensors/SOURCE.json)，引用闭包和HTTP ETag见 [manifest.json](../../sim/assets/sensors/manifest.json)。官方S3不暴露Git commit，清单用“Isaac Sim 6.2目录版本+SHA-256”固定字节，不伪造提交号。资产仍受NVIDIA Omniverse/Isaac内容条款和原权利人条款约束，因此供应商原件不复制到Windows发布目录。

主要原件：

| 文件 | 字节 | SHA-256 |
|---|---:|---|
| `Orbbec/Gemini335L/orbbec_gemini_335L.usd` | 50,671,410 | `0ca83218…f59da` |
| `Ouster/OS0/OS0.usd` | 16,032 | `b11a455c…fe626` |
| `Ouster/meshes/os0_mesh.usd` | 240,923 | `36339a2e…84432` |
| `Ouster/meshes/textures/sensor_color.png` | 9,461 | `fd31f81c…60f7` |

## 2. 分层装配与真实性边界

项目层 [go2w_sensor_rig.usda](../../sim/assets/sensors/assemblies/go2w_sensor_rig.usda) 引用供应商原件，运行时挂到 `/World/Go2W/Geometry/base/sensor_rig`，没有改写 Go2-W 资产或供应商USD。结构如下：

```text
sensor_rig
├── Appearance
│   ├── Gemini335LMount/Gemini335L  -> 本地官方335L USD
│   ├── OS0Mount/OS0_REV7_Appearance -> 本地官方OS0外观，内嵌sensor停用
│   └── MountPlate                   -> 项目简单外观，无碰撞
└── Measurement
    ├── left/right camera -> 640×480@10Hz理想RGB针孔代理，基线95 mm
    ├── lidar             -> 64线×512、10Hz、0.5–20m参数化RTX代理
    └── imu               -> Isaac物理理想IMU
```

坐标为`+X前、+Y左、+Z上`。左右光心相对base为`(0.365,+0.0475,0.095)`与`(0.365,-0.0475,0.095)m`；雷达测量原点`(0,0,0.200)m`；IMU`(0,0,0.205)m`。官方335L根节点带约0.98975 m目录展示位移，项目引用层只归零该位移并保留官方朝向/尺度，供应商文件未改。OS0外壳底部落在板顶`z≈0.122m`，扫描原点在外壳顶部上方约4.5 mm。

装配层不写碰撞、刚体或独立质量，随Go2-W base运动；纯外观外壳不作为复杂碰撞体。审计装配边界约`x[-0.05,0.362]、y[-0.045,0.045]、z[0.0794,0.1955]m`，无碰撞/刚体/质量prim。运行时最终雷达最小回波0.8965 m，0.48 m自扫描保护圈内0点；这证明当前场景未被自身外壳完全挡住，不代表所有姿态/场景都不会自扫。

## 3. 可重跑命令

无窗口最终复现；每次使用不存在的新目录，失败结果也保留：

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/stage2_$(TZ=Asia/Shanghai date +%Y%m%dT%H%M%S)"
mkdir -p "$RUN/logs"
python3 scripts/sensor_pipeline/stage2_manifest.py --run "$RUN"
/home/joy/robotics/isaac-env/bin/python scripts/sensor_pipeline/audit_asset_rig.py \
  --output "$RUN/asset_audit.json"
./scripts/run_go2w_sensor_rig.sh "$RUN/smoke_headless" --headless \
  > "$RUN/logs/smoke_headless.log" 2>&1
rg 'STAGE2_SENSOR_RIG_PASS|GO2W_SIMULATION_COMPLETE|\[Error\]|Traceback' \
  "$RUN/logs/smoke_headless.log"
python3 scripts/sensor_pipeline/visualize_asset_rig.py --sample "$RUN/smoke_headless"
```

GUI与传感器图片分开。以下启动会自动跑300步并退出；在另一终端运行捕获命令：

```bash
# 终端1
./scripts/run_go2w_sensor_rig.sh "$RUN/smoke_gui" > "$RUN/logs/smoke_gui.log" 2>&1

# 终端2：等待标题出现并只截实际Isaac窗口，不是相机输出
python3 scripts/sensor_pipeline/capture_x11_window.py \
  --title 'Isaac Sim Python 6.2.0' --timeout 30 \
  --output "$RUN/smoke_gui/isaac_gui_window.png"
```

复现后必须检查日志完成标记和JSON；Kit退出码0本身不算通过。左右图的水平基线应使近物左右位置差大于远物，但本阶段只提供两路输入，`stereo_depth_computed=false`。雷达的非零XYZ、逐点timestamp/emitterId和IMU有效位才证明测量产生；只看到外壳不算测量成功。

## 4. 本次实际结果与失败保留

最终机器验收：[final_acceptance.json](../../outputs/sensor_pipeline/stage2_20261026T172917/final_acceptance.json)；离线审计：[asset_audit.json](../../outputs/sensor_pipeline/stage2_20261026T172917/asset_audit.json)。

| 项目 | 本次实测 | 状态 |
|---|---|---|
| 资产闭包 | 17项哈希通过；14个USD层；远程/缺失依赖均0；运行时MDL模块可解析 | 通过 |
| 尺度/外参 | metersPerUnit=1、Z-up、基线0.095 m | 通过 |
| 左/右图 | 均480×640×3；SHA不同；像素差异比例47.27% | 通过 |
| RTX雷达 | 12,084个有限XYZ点；timestamp/emitterId各12,084 | 通过 |
| 自扫/遮挡 | 最小回波0.8965 m；0.48m内0点 | 当前场景通过 |
| IMU | 有效 | 通过 |
| 动力学隔离 | sensor_rig下碰撞/刚体/质量均0 | 通过 |
| GUI | 实际Isaac窗口1440×900，Go2-W与两外壳可见 | 通过 |
| 旧默认入口回归 | 不带任何sensor-rig参数运行180步，正常打印`GO2W_SIMULATION_COMPLETE`且未创建阶段2传感器 | 通过 |
| 匹配深度/厂家测量 | 未运行 | 不属于阶段2，明确未完成 |

最终无窗口样例：[整机装配图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_headless_03/assembly_render.png)、[左图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_headless_03/left_rgb.png)、[右图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_headless_03/right_rgb.png)、[点云组合图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_headless_03/stage2_contact_sheet.png)。实际窗口：[GUI截图](../../outputs/sensor_pipeline/stage2_20261026T172917/smoke_gui_01/isaac_gui_window.png)。`assembly_render.png`是独立观察相机渲染，左右PNG是传感器输出，GUI截图是X11窗口，三者不能混称。

失败没有删除：`smoke_headless_01`虽产生13,103点，但OS0贴图缺失，标记`INCOMPLETE_ASSET_CLOSURE`；补图后`smoke_headless_02`产生13,080点，但外壳悬浮，标记`INCOMPLETE_VISUAL_QA`；修正安装高度后的`smoke_headless_03`才是最终无窗口证据。相应状态JSON和日志均保留。

兼容性回归日志为`logs/default_demo_regression.log`：旧入口不带`--sensor-rig-config`运行180步正常结束，证明阶段2仍是显式选择的独立入口，未把原演示强制改成新装配。

## 5. 回退、限制与下一阶段输入

本阶段没有安装包、SDK、Python环境或新权重，没有修改驱动/系统Python/Isaac安装目录/原策略。默认旧演示完全不传`--sensor-rig-config`即可保持原行为。若要回退阶段2，只需不启用新参数；删除项目引用层前应保留本run和清单，供应商原件可按SOURCE URL和SHA重新取得。

限制：官方335L外壳/内嵌相机存在不等于本阶段理想RGB双目的厂家真实性；官方OS0只有REV6/7，本阶段通用64线不是REV8工厂beam表；未模拟IR投影、光谱、滚动快门、厂家噪声/时钟；未验证曝光同步、标定精度、匹配深度、ROS发布、融合、SLAM/Nav2。ScanBuffer读取仍有官方deprecated警告，阶段3同步/标定工作应先迁移或并行验证GenericModelOutput，并保留字段语义对照。

阶段3输入：本地闭包资产、`stage2_rig.json`、最终左右PNG/点云时间字段、明确base外参和失败记录。先验证左右时间戳/内外参与标定靶几何，再做无需训练的左右匹配基线；几何真值仅评估，不替代匹配结果。当前阶段到此结束。

共享盘只导出版本化小快照：`python3 scripts/sensor_pipeline/export_stage2.py --run outputs/sensor_pipeline/stage2_20261026T172917 --destination /mnt/Shared/First1/1023Robot_Dog_Development/releases/stage2_20261026T172917_docs_v2`。导出器拒绝覆盖已有目录且不复制供应商原件或大数据。v1因遗漏项目装配层而保留为不完整快照，不能作为最终交付；v2包含项目自建USDA并仍排除`vendor/`。
