# 01a 滚动时右前轮前后抖动修复

日期2026-10-26；证据目录 `outputs/sensor_pipeline/stage1_20261026T154717`。本次没有重训/替换权重，也没有修改系统驱动或Isaac环境。

## 原因与修改

按W在滚动模式对应前进速度指令。原轮式策略带1.5 Hz步态相位与腿关节残差，右前大腿目标周期变化约0.115 rad，即使轮子始终正转，轮轴也随腿前后摆动；轮速目标/高阻尼又放大速度波动。不能将此现象误判成已测得轮电机反转。

`scripts/go2w_policy.py` 新增默认smooth轮式控制：保持既有名义站姿，滚动时不施加学习策略的抬腿残差；轮式相位固定；同侧前后轮残差合并并低通(alpha=.15)，轮速目标变化率限制4 rad/s²，轮关节阻尼从.08降至.02（USD单位）；松开指令仍立即置零制动。纯腿策略分支保持原逻辑。历史坡面/扰动50次验收不能自动沿用到这个新控制器。

保留 `--wheel-control legacy` 可恢复原控制行为；原源文件在本次 `before/scripts/`。环境/权重无需恢复。当前正在运行的旧窗口不会热更新：正常停止自己的仿真后重新执行原启动命令，默认即使用修复版。

## 本次量化结果

相同平地场景，站立2秒、0.15 m/s前进6秒、停车3秒；统计3–7.5秒稳态窗口。无窗口命令重放，不是人工按键试验。通过实际大腿/小腿角和模型连杆长，计算右前轮轴相对髋部前后位置：`x=-0.213*sin(thigh)-0.2264*sin(thigh+calf)`；不等于轮轴世界系位移。

| 指标 | 修复前 | 修复后 |
|---|---:|---:|
| FR轮轴前后峰峰值 | 30.602 mm | 3.940 mm |
| FR大腿目标峰峰值 | 0.11465 rad | 0 |
| FR轮转速峰峰值 | 11.226 rad/s | 2.393 rad/s |
| FR最小转速（前进窗口） | +1.860 rad/s | +0.783 rad/s |
| 平均机体前进速度 | 0.1377 m/s | 0.1897 m/s |

轮轴摆幅下降87.1%，没有消除全部接触/控制振动。0.15 m/s命令与实速仍有偏差，当前策略的有效轮半径/残差映射尚未重新标定；本次不宣称速度跟踪已校准。最终停车速度与ROS回归详见本次JSON。该结果仅覆盖单次平地直行/停车，不代表坡地、摩擦变化、纯腿切换或真人W键验收。

## 复现和回退

```bash
cd /home/joy/桌面/RobotDog
RUN="outputs/sensor_pipeline/wheel_$(date +%Y%m%dT%H%M%S)"
mkdir -p "$RUN"
./scripts/run_go2w.sh --headless --policy outputs/training/wheel_slope/policy_jit.pt \
  --wheel-control legacy --wheel-replay --diagnostics "$RUN/wheel_before.csv" > "$RUN/before.log" 2>&1
./scripts/run_go2w.sh --headless --policy outputs/training/wheel_slope/policy_jit.pt \
  --wheel-replay --diagnostics "$RUN/wheel_after_stance.csv" > "$RUN/after.log" 2>&1
rg GO2W_SIMULATION_COMPLETE "$RUN/before.log" "$RUN/after.log"
python3 scripts/sensor_pipeline/analyze_wheel.py --run "$RUN"
# 交互启动仍用已有脚本；需要比较旧逻辑时追加 --wheel-control legacy
./scripts/run_go2w_dual.sh
```

原始before/after CSV、第一轮仅平滑但未固定站姿的中间试验、全部日志均保留。`wheel_metrics.json`和`wheel_fix.png`可从CSV重算。运行记录/哈希在同目录；不删除失败，未观察GUI就不声称人工交互通过。

本次回归：`ros_recheck/verification.json` 8/8通过（跨进程cmd、超时停车、RGB、CameraInfo、16关节、TF、optical TF、clock）；`mode_recheck/mode-verification.json` 8/8通过（运动中拒绝切换、停车后进入walk/回到wheel、短距离直行及停车站姿）。测试域98且localhost；不连接实机。此模式检查只覆盖平地短距离，纯腿转弯/坡地未验。
