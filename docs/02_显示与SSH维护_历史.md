# 显示与 SSH 修复记录

> 文档导航：编号与复现入口见 [00_复现总览与文档索引](00_复现总览与文档索引.md)。本文保留历史记录或规划，不代表本次复验结果；本次结果以 [阶段0环境验收](sensor_pipeline/00_environment.md) 为准。

日期：2026-10-25

## 原因与处理

重启后 NVIDIA 内核模块缺失：`modinfo nvidia` 报 Module not found，`nvidia-smi` 无法连接驱动，DKMS 仅为 added 状态。当前 7.0.0-38 内核的头文件未安装，桌面只看到 None-1 基础显示输出。

已安装：
- linux-modules-nvidia-595-open-7.0.0-38-generic
- linux-modules-nvidia-595-open-generic-hwe-24.04（跟随内核升级）
- linux-headers-generic-hwe-24.04 及当前内核头文件

已完成 depmod、initramfs 和 GRUB 更新；nvidia 与 nvidia_drm 版本 595.99.02，签名为 Canonical Ltd. Kernel Module Signing。Secure Boot 保持开启。包管理器检查无未完成配置。

安装完成后尚需重启，显示恢复情况必须在重启后确认，不自动重启用户电脑。

## SSH 常驻

- ssh.service：enabled，active，监听 IPv4/IPv6 22 端口。
- 本机连接已获得 OpenSSH banner；UFW 当前 inactive。
- 当前 Wi-Fi IPv4 为 10.63.187.189，连接命令 `ssh joy@10.63.187.189`。该地址由网络分配，可能变化；校园网络可能限制设备互访。
- GNOME 接电和电池模式下自动睡眠均设置为 nothing。
- sleep.target、suspend.target、hibernate.target、hybrid-sleep.target 已 mask，重启后继续有效；电池供电也保持运行。
- 未修改 SSH 认证方式，未开放 root 登录，未设置公网端口转发。

## 重启后检查

```bash
nvidia-smi
systemctl is-active ssh
ip -brief address
```

修复脚本与日志：`/home/joy/robotics/maintenance/repair-display-ssh.sh` 和同目录 `repair-display-ssh.log`。

Ubuntu 官方说明：https://ubuntu.com/server/docs/nvidia-drivers-installation/
