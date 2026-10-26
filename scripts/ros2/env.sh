#!/usr/bin/env bash
# Source this in external ROS terminals; use the same domain on other computers.
if [[ "${ROBOTDOG_ROS_RUNTIME:-auto}" != bundled && -f /opt/ros/jazzy/setup.bash ]]; then
  source /opt/ros/jazzy/setup.bash
else
  # Isaac Sim bundles a complete rclpy/message runtime; useful while apt installs.
  robotdog_ros_prefix=/home/joy/robotics/isaac-env/lib/python3.12/site-packages/isaacsim/exts/isaacsim.ros2.core/jazzy
  export ROS_DISTRO=jazzy
  export ROS_VERSION=2
  export ROS_PYTHON_VERSION=3
  export AMENT_PREFIX_PATH="$robotdog_ros_prefix${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
  export PYTHONPATH="$robotdog_ros_prefix/rclpy${PYTHONPATH:+:$PYTHONPATH}"
  export LD_LIBRARY_PATH="$robotdog_ros_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  case ":$PATH:" in
    *:/home/joy/桌面/RobotDog/scripts/ros2:*) ;;
    *) export PATH="/home/joy/桌面/RobotDog/scripts/ros2:$PATH" ;;
  esac
  unset robotdog_ros_prefix
fi
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
