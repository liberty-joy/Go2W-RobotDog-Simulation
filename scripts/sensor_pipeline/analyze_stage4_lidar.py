"""Recompute Stage 4 range/timing summaries and raw motion point-cloud figures."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", type=Path, required=True)
    a = ap.parse_args()
    run = a.run.resolve()
    summary = json.loads((run / "capture_summary.json").read_text())
    scans = summary["scans"]
    static = [s for s in scans if str(s.get("phase", "")).startswith("static_") and "point_count" in s]
    target_distances = sorted({float(s["phase"].removeprefix("static_").removesuffix("m")) for s in static})
    loaded = {}
    for meta in scans:
        if "point_count" in meta:
            p = run / "raw" / f"scan_{meta['index']:04d}.npz"
            loaded[meta["index"]] = dict(np.load(p))
    # Infer which coordinate axis points along the known +X test plane from multiple static ranges.
    axis_scores = []
    for axis in range(3):
        for sign in (-1.0, 1.0):
            errors, counts = [], []
            for d in target_distances:
                group = [m for m in static if m["phase"] == f"static_{d:g}m"]
                pts = np.concatenate([loaded[m["index"]]["data"] for m in group if m["index"] in loaded]) if group else np.empty((0, 3))
                if len(pts):
                    delta = np.abs(sign * pts[:, axis] - d)
                    close = delta < 0.08
                    if close.any():
                        errors.extend(delta[close].tolist())
                        counts.append(int(close.sum()))
            axis_scores.append({"axis": axis, "sign": int(sign), "count": int(sum(counts)),
                                "mae": float(np.mean(errors)) if errors else None,
                                "score": float(sum(counts) - 100 * np.mean(errors)) if errors else -1})
    best = max(axis_scores, key=lambda x: x["score"])
    axis, sign = best["axis"], best["sign"]
    rows = []
    co_view_rows = []
    def rotation_from_wxyz(q):
        w, x, y, z = np.asarray(q, dtype=float) / max(np.linalg.norm(q), 1e-12)
        return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                         [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                         [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    for d in target_distances:
        group = [m for m in static if m["phase"] == f"static_{d:g}m"]
        pts = np.concatenate([loaded[m["index"]]["data"] for m in group if m["index"] in loaded]) if group else np.empty((0, 3))
        pose = group[len(group) // 2] if group else {}
        position = np.asarray(pose.get("sensor_position_world_m", [0, 0, 0]), dtype=float)
        q = np.asarray(pose.get("sensor_orientation_world_wxyz", [1, 0, 0, 0]), dtype=float)
        w, x, y, z = q / max(np.linalg.norm(q), 1e-12)
        rotation_world_from_sensor = rotation_from_wxyz(q)
        axis_projection = rotation_world_from_sensor[0, axis]
        expected_raw_coordinate = (d - position[0]) / axis_projection if abs(axis_projection) > 0.5 else d
        measured = sign * pts[:, axis] if len(pts) else np.empty(0)
        err = measured - expected_raw_coordinate if len(pts) else np.empty(0)
        nonzero = np.isfinite(pts).all(axis=1) & (np.linalg.norm(pts, axis=1) > 1e-6) if len(pts) else np.zeros(0, dtype=bool)
        target_mask = (np.abs(err) < .08) & nonzero
        target_err = err[target_mask]
        rows.append({"target_plane_distance_m": d, "expected_raw_axis_coordinate_m": float(sign * expected_raw_coordinate),
                     "sensor_world_x_m": float(position[0]), "axis_projection_world_x": float(axis_projection),
                     "scans": len(group), "all_hit_points": int(len(pts)), "zero_vector_invalid_points": int(np.count_nonzero(~nonzero)),
                     "target_plane_returns_within_8cm": int(target_mask.sum()),
                     "target_plane_fraction_of_hits": float(target_mask.mean()) if len(pts) else 0.0,
                     "plane_coordinate_mae_m": float(np.mean(np.abs(target_err))) if len(target_err) else None,
                     "plane_coordinate_rmse_m": float(np.sqrt(np.mean(target_err ** 2))) if len(target_err) else None,
                     "plane_coordinate_bias_m": float(np.mean(target_err)) if len(target_err) else None})
        # Use the actual per-scan sensor/camera transforms and the public RGB pinhole FoV.
        for meta in group:
            xyz = loaded[meta["index"]]["data"]
            valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-6)
            xyz = xyz[valid]
            if not len(xyz) or "public_camera_position_world_m" not in meta:
                continue
            r_lidar = rotation_from_wxyz(meta["sensor_orientation_world_wxyz"])
            p_world = xyz @ r_lidar.T + np.asarray(meta["sensor_position_world_m"])
            r_cam = rotation_from_wxyz(meta["public_camera_orientation_world_wxyz"])
            p_cam = (p_world - np.asarray(meta["public_camera_position_world_m"])) @ r_cam
            width, height, hfov = 640, 480, np.deg2rad(80.0)
            vfov = 2 * np.arctan(np.tan(hfov / 2) * height / width)
            in_view = (p_cam[:, 2] > 0.02) & (np.abs(p_cam[:, 0] / np.maximum(p_cam[:, 2], 1e-9)) <= np.tan(hfov/2)) & (np.abs(p_cam[:, 1] / np.maximum(p_cam[:, 2], 1e-9)) <= np.tan(vfov/2))
            co_view_rows.append({"target_plane_distance_m": d, "scan_index": meta["index"],
                                 "valid_lidar_points": int(len(xyz)), "inside_public_rgb_frustum": int(in_view.sum()),
                                 "frustum_fraction": float(in_view.mean()),
                                 "interpretation": "angular co-visibility only; camera pixel occlusion/visibility not tested"})
    with (run / "static_distance_error.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]) if rows else ["target_plane_distance_m"])
        w.writeheader(); w.writerows(rows)
    with (run / "scan_timing.csv").open("w", newline="") as f:
        fields = ["index", "phase", "simulation_time_s", "rendering_time", "rendering_frame", "point_count",
                  "timestamp_ns_min", "timestamp_ns_max", "timestamp_span_ms", "timestamp_unique",
                  "emitter_id_unique_count", "seconds_from_end_frame_to_latest_ray"]
        w = csv.DictWriter(f, fieldnames=fields); w.writeheader()
        prev = None
        for m in scans:
            if "point_count" not in m: continue
            lo, hi = m.get("timestamp_ns_min"), m.get("timestamp_ns_max")
            raw = loaded[m["index"]]
            stamps = raw.get("timestamp", np.empty(0, np.uint64))
            row = {k: m.get(k) for k in fields}
            row["timestamp_span_ms"] = (hi - lo) / 1e6 if lo is not None and hi is not None else None
            row["timestamp_unique"] = int(len(np.unique(stamps))) if len(stamps) else 0
            row["seconds_from_end_frame_to_latest_ray"] = m["simulation_time_s"] - hi / 1e9 if hi is not None else None
            w.writerow(row)
    with (run / "camera_lidar_frustum_overlap.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(co_view_rows[0]) if co_view_rows else ["target_plane_distance_m"])
        w.writeheader(); w.writerows(co_view_rows)
    selfscan_groups = {}
    for phase in ("selfscan_visible", "selfscan_hidden"):
        group = [m for m in scans if m.get("phase") == phase and "point_count" in m]
        counts, emitter_counts = [], []
        for meta in group:
            z = loaded[meta["index"]]
            xyz = z["data"]
            valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-6)
            counts.append(int(valid.sum()))
            e = z.get("emitterId", np.empty(0))
            emitter_counts.append(int(len(np.unique(e[valid]))) if len(e) == len(valid) else 0)
        selfscan_groups[phase] = {"scans": len(group), "valid_return_counts": counts,
                                  "median_valid_return_count": float(np.median(counts)) if counts else None,
                                  "median_emitter_ids_with_returns": float(np.median(emitter_counts)) if emitter_counts else None}
    vis_n = selfscan_groups["selfscan_visible"]["median_valid_return_count"]
    hid_n = selfscan_groups["selfscan_hidden"]["median_valid_return_count"]
    def ray_map(meta):
        raw = loaded[meta["index"]]
        xyz = raw["data"]
        stamps = raw.get("timestamp", np.empty(0, np.uint64))
        emitters = raw.get("emitterId", np.empty(0, np.uint32))
        valid = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-6)
        xyz, stamps, emitters = xyz[valid], stamps[valid], emitters[valid]
        if not len(xyz) or len(stamps) != len(xyz) or len(emitters) != len(xyz):
            return {}
        relative_us = np.rint((stamps.astype(np.float64) - float(stamps.min())) / 1000.0).astype(np.int64)
        keys = relative_us * 100 + emitters.astype(np.int64)
        return {int(k): float(r) for k, r in zip(keys, np.linalg.norm(xyz, axis=1))}
    ray_diffs = []
    visible_meta = [m for m in scans if m.get("phase") == "selfscan_visible" and "point_count" in m]
    hidden_meta = [m for m in scans if m.get("phase") == "selfscan_hidden" and "point_count" in m]
    for visible, hidden in zip(visible_meta, hidden_meta):
        vm, hm = ray_map(visible), ray_map(hidden)
        common = set(vm) & set(hm)
        delta = np.asarray([vm[k] - hm[k] for k in common], dtype=float)
        ray_diffs.append({"visible_scan_index": visible["index"], "hidden_scan_index": hidden["index"],
                          "matched_ray_keys": int(len(common)),
                          "fraction_of_smaller_return_set_matched": float(len(common) / max(1, min(len(vm), len(hm)))),
                          "matched_range_abs_mae_m": float(np.mean(np.abs(delta))) if len(delta) else None,
                          "matched_range_abs_p95_m": float(np.percentile(np.abs(delta), 95)) if len(delta) else None,
                          "matched_rays_changed_over_5cm": int(np.count_nonzero(np.abs(delta) > .05))})
    selfscan_comparison = {"groups": selfscan_groups,
                           "visible_minus_hidden_median_returns": (vis_n - hid_n) if vis_n is not None and hid_n is not None else None,
                           "visible_return_change_fraction": ((vis_n - hid_n) / hid_n) if vis_n is not None and hid_n else None,
                           "timestamp_emitter_matched_ray_differences": ray_diffs,
                           "interpretation": "counts compare same static scene with robot visual meshes visible/hidden; collision and control stay unchanged; no per-point objectId is available, so occlusion is detected indirectly, not labeled per hit"}
    (run / "self_scan_comparison.json").write_text(json.dumps(selfscan_comparison, indent=2))
    # Make a clearly raw, timestamp-colored plot of a full accumulated moving sweep.
    moving = [m for m in scans if m.get("phase") == "motion" and m.get("point_count", 0) > 100]
    chosen = moving[len(moving) // 2] if moving else None
    motion_plot = None
    if chosen:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        data = loaded[chosen["index"]]
        xyz = data["data"]
        stamps = data.get("timestamp", np.arange(len(xyz)))
        order = np.argsort(stamps)
        fig = plt.figure(figsize=(9, 7))
        ax = fig.add_subplot(111)
        c = (stamps[order] - stamps.min()).astype(np.float64) / 1e6
        sc = ax.scatter(xyz[order, 0], xyz[order, 1], c=c, s=1, cmap="viridis", rasterized=True)
        ax.set_xlabel("Raw point coordinate axis 0 (m)")
        ax.set_ylabel("Raw point coordinate axis 1 (m)")
        ax.set_title("Raw accumulated moving sweep, color = per-point timestamp (ms)")
        ax.set_aspect("equal", adjustable="datalim")
        fig.colorbar(sc, ax=ax, label="relative per-point timestamp (ms)")
        fig.tight_layout()
        motion_plot = str(run / "motion_pointcloud_timestamp.png")
        fig.savefig(motion_plot, dpi=160); plt.close(fig)
        np.save(run / "motion_points_raw.npy", xyz)
    # Evaluate the configured blind-zone sample without treating missing returns as free space.
    blind_scans = [m for m in scans if m.get("phase") == "static_0.3m" and m.get("point_count", 0) > 0]
    blind_near = []
    for m in blind_scans:
        xyz = loaded[m["index"]]["data"]
        ranges = np.linalg.norm(xyz, axis=1)
        ranges = ranges[np.isfinite(ranges) & (ranges > 1e-6)]
        if len(ranges): blind_near.append(float(ranges.min()))
    out = {
        "status": "recomputed from raw scan NPZ; no ground-truth pose or depth stream was used",
        "raw_point_coordinate_axis_inferred_from_known_planes": axis,
        "coordinate_frame": "sensor-local (transformPoints default false per installed Isaac scan-buffer OGN API); static expected raw plane coordinate uses per-scan sensor pose",
        "axis_sign": sign,
        "axis_selection_scores": axis_scores,
        "static_distance_rows": rows,
        "render_hz_observed": summary.get("render_rate_hz_observed"),
        "render_dt_median_s": summary.get("render_frame_dt_median_s"),
        "render_dt_p95_s": summary.get("render_frame_dt_p95_s"),
        "point_timestamp_span_ms_median": float(np.median([(m["timestamp_ns_max"] - m["timestamp_ns_min"]) / 1e6 for m in scans if m.get("timestamp_ns_min") is not None])) if scans else None,
        "scan_output_interval_s_median": float(np.median(np.diff([m["simulation_time_s"] for m in scans if "point_count" in m]))) if sum("point_count" in m for m in scans) > 1 else None,
        "blind_zone_0.3m": {"scans": len(blind_scans), "nearest_return_range_m": min(blind_near) if blind_near else None,
                             "interpretation": "near-range cutoff is 0.5m; zero-vector invalid records excluded; no return is not free-space evidence"},
        "camera_lidar_frustum_overlap": {"sample_count": len(co_view_rows),
                                          "mean_fraction": float(np.mean([r["frustum_fraction"] for r in co_view_rows])) if co_view_rows else None,
                                          "csv": "camera_lidar_frustum_overlap.csv",
                                          "limitation": "frustum geometry only; RGB visibility/occlusion is not pixel-compared"},
        "self_scan_comparison": selfscan_comparison,
        "motion_scan_count": len(moving),
        "motion_plot": motion_plot,
        "pointcloud2": summary.get("ros_pointcloud2", "not emitted; no ROS 2 output recorded"),
        "laserscan": "not emitted; the configured scanner is 3D and no explicit 2D height slice was authorized",
    }
    (run / "analysis.json").write_text(json.dumps(out, indent=2))
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
