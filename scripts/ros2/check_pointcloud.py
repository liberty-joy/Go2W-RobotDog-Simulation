#!/usr/bin/env python3
"""Receive one Go2-W PointCloud2 and verify its declared binary layout."""
import argparse
import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    rclpy.init()
    node = Node("go2w_pointcloud_layout_check")
    result = {"received": False}

    def callback(msg):
        fields = [{"name": f.name, "offset": int(f.offset), "datatype": int(f.datatype), "count": int(f.count)} for f in msg.fields]
        by_name = {f["name"]: f for f in fields}
        dtype = np.dtype({"names": ["x", "y", "z", "time_from_scan_start", "emitter_id"],
                          "formats": ["<f4", "<f4", "<f4", "<f8", "<u4"],
                          "offsets": [0, 4, 8, 12, 20], "itemsize": int(msg.point_step)})
        layout_ok = (msg.height == 1 and msg.point_step == 24 and msg.row_step == msg.point_step * msg.width
                     and len(msg.data) == msg.row_step and set(by_name) == {"x", "y", "z", "time_from_scan_start", "emitter_id"}
                     and [by_name[k]["offset"] for k in ("x", "y", "z", "time_from_scan_start", "emitter_id")] == [0, 4, 8, 12, 20])
        points = np.frombuffer(msg.data, dtype=dtype, count=msg.width)
        result.update({"received": True, "layout_ok": bool(layout_ok), "topic": "/go2w/lidar/points",
                       "frame_id": msg.header.frame_id, "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
                       "height": int(msg.height), "width": int(msg.width), "point_step": int(msg.point_step),
                       "row_step": int(msg.row_step), "data_length": len(msg.data), "fields": fields,
                       "xyz_finite": bool(np.isfinite(np.column_stack([points["x"], points["y"], points["z"]])).all()),
                       "time_from_scan_start_min_s": float(points["time_from_scan_start"].min()) if len(points) else None,
                       "time_from_scan_start_max_s": float(points["time_from_scan_start"].max()) if len(points) else None,
                       "emitter_id_unique_count": int(len(np.unique(points["emitter_id"]))) if len(points) else 0,
                       "ring_or_intensity_synthesized": False})

    sub = node.create_subscription(PointCloud2, "/go2w/lidar/points", callback, qos_profile_sensor_data)
    deadline = time.monotonic() + args.timeout
    while rclpy.ok() and not result["received"] and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    node.destroy_node()
    rclpy.shutdown()
    with open(args.output, "x") as stream:
        json.dump(result, stream, indent=2)
    print(json.dumps(result, indent=2))
    if not result["received"] or not result.get("layout_ok") or not result.get("xyz_finite"):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
