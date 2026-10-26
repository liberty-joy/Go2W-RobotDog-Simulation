#!/usr/bin/env bash
# Optional, idempotent native Jazzy installer; run with sudo/pkexec.
set -euo pipefail
if [[ $EUID -ne 0 ]]; then
  echo 'Run with sudo bash scripts/ros2/install_jazzy.sh' >&2
  exit 1
fi
export DEBIAN_FRONTEND=noninteractive
if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | rg -q 'install ok installed'; then
  deb_file=$(mktemp /var/tmp/robotdog-ros2-source.XXXXXX.deb)
  curl -fL --retry 2 -o "$deb_file" https://github.com/ros-infrastructure/ros-apt-source/releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb
  dpkg -i "$deb_file"
  rm -f "$deb_file"
fi
original_sources=$(mktemp /var/tmp/robotdog-ubuntu-sources.XXXXXX)
cp /etc/apt/sources.list.d/ubuntu.sources "$original_sources"
restore_sources() {
  cp "$original_sources" /etc/apt/sources.list.d/ubuntu.sources
  rm -f "$original_sources"
}
trap restore_sources EXIT
sed -i -e 's|http://archive.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' -e 's|http://security.ubuntu.com/ubuntu/|https://mirrors.tuna.tsinghua.edu.cn/ubuntu/|g' /etc/apt/sources.list.d/ubuntu.sources
python3 - <<'PY'
from pathlib import Path
p=Path('/etc/apt/sources.list.d/ros2.sources').resolve()
s=p.read_text().replace('Types: deb deb-src','Types: deb').replace('http://packages.ros.org/ros2/ubuntu','https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu')
p.write_text(s)
PY
apt-get -o Acquire::Retries=2 update
apt-get -o DPkg::Lock::Timeout=600 -o Acquire::Retries=2 install -y ros-jazzy-desktop ros-jazzy-teleop-twist-keyboard ros-jazzy-rmw-fastrtps-cpp python3-colcon-common-extensions
printf '\nROBOTDOG_ROS2_INSTALL_OK\n'
