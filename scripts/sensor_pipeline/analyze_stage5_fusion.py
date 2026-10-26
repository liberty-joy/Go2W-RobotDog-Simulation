"""Offline Stage 5 RGB/stereo/LiDAR object geometry with preserved evidence and failures."""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation

from fusion_geometry import (PoseTimeline, decide_hybrid, frontmost_per_pixel,
                             red_component_mask, project_optical, robust_center, world_to_camera)
from process_stage3_stereo import run_matcher


def save_json(path, value):
    path.write_text(json.dumps(value, indent=2, allow_nan=False))


def inspected_polygon_mask(annotation, phase, image_path, shape):
    if annotation is None: return None
    if annotation["run_id"] != image_path.parents[2].name:
        return None
    entry = annotation["public_rgb"][phase]
    digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
    if digest != entry["sha256"]:
        raise RuntimeError(f"manual annotation image hash mismatch for {phase}")
    mask = np.zeros(shape, np.uint8)
    polygon = np.asarray(entry["polygon_xy"], np.int32)
    cv2.fillPoly(mask, [polygon], 1)
    return mask


def core_logic_checks():
    K = np.array([[100., 0, 50], [0, 100, 50], [0, 0, 1.]])
    xyz = np.array([[0, 0, 2], [0, 0, -1], [10, 0, 1], [0, 0, 3.]], float)
    uv, valid = project_optical(xyz, K, np.zeros(5), (100, 100))
    front = frontmost_per_pixel(uv, xyz[:, 2], valid, (100, 100))
    a = {"position_m": [0, 0, 2], "uncertainty_m": .02, "modality": "lidar"}
    b = {"position_m": [0, 0, 2.6], "uncertainty_m": .02, "modality": "stereo"}
    _, mode, conflict, reobserve = decide_hybrid(a, b, .2)
    checks = {"optical_center": bool(np.allclose(uv[0], [50, 50])),
              "behind_rejected": bool(not valid[1]), "out_of_bounds_rejected": bool(not valid[2]),
              "z_buffer_keeps_nearest": bool(front.tolist() == [0]),
              "conflict_yields_unknown": bool(mode == "unknown" and conflict and reobserve)}
    if not all(checks.values()): raise RuntimeError("fusion core logic failed: " + str(checks))
    return checks


def project_lidar(raw, timeline, camera_pose, K, D, image_size, offset_s=0., translation_x_m=0.,
                  yaw_deg=0., deskew=True, readout_s=None):
    points = np.asarray(raw["data"], dtype=np.float64)
    time_s = np.asarray(raw["timestamp"], dtype=np.uint64).astype(np.float64) * 1e-9 + offset_s
    measured = np.isfinite(points).all(axis=1) & (np.linalg.norm(points, axis=1) > 1e-6)
    in_trajectory = (time_s >= timeline.times[0]) & (time_s <= timeline.times[-1])
    valid_time = measured & in_trajectory
    indices = np.flatnonzero(valid_time)
    if not len(indices):
        return {"points_optical": np.empty((0, 3)), "uv": np.empty((0, 2)), "valid": np.empty(0, bool),
                "raw_indices": indices, "time_rejected": int((measured & ~in_trajectory).sum())}
    pts = points[indices]
    if yaw_deg:
        pts = Rotation.from_euler("z", yaw_deg, degrees=True).apply(pts)
    pts[:, 0] += translation_x_m
    query = time_s[indices] if deskew else np.full(len(indices), readout_s)
    position, rotation = timeline.at(query)
    world = rotation.apply(pts) + position
    p_cam, r_cam = camera_pose
    optical = world_to_camera(world, p_cam, r_cam)
    uv, valid = project_optical(optical, K, D, image_size)
    return {"points_optical": optical, "uv": uv, "valid": valid, "raw_indices": indices,
            "time_rejected": int((measured & ~in_trajectory).sum()),
            "behind_count": int((optical[:, 2] <= .05).sum()), "image_valid_count": int(valid.sum())}


def lidar_object(projected, mask, config):
    height, width = mask.shape
    optical, uv, valid = projected["points_optical"], projected["uv"], projected["valid"]
    front = frontmost_per_pixel(uv, optical[:, 2], valid, (width, height))
    if not len(front): return None, {"frontmost": 0, "inside_mask": 0, "layer_kept": 0}
    pixels = np.rint(uv[front]).astype(int)
    pixels[:, 0] = np.clip(pixels[:, 0], 0, width-1)
    pixels[:, 1] = np.clip(pixels[:, 1], 0, height-1)
    inside = front[mask[pixels[:, 1], pixels[:, 0]] > 0]
    stats = {"frontmost": int(len(front)), "inside_mask": int(len(inside)), "layer_kept": 0}
    if not len(inside): return None, stats
    depth = optical[inside, 2]
    front_layer = float(np.quantile(depth, .2) + config["z_layer_tolerance_m"])
    chosen = inside[depth <= front_layer]
    stats["layer_kept"] = int(len(chosen))
    if len(chosen) < config["min_lidar_points"]: return None, stats
    center, geometry_spread = robust_center(optical[chosen])
    z = optical[chosen, 2]
    depth_mad = float(1.4826*np.median(np.abs(z-np.median(z))))
    if depth_mad > config["max_range_mad_m"]: return None, {**stats, "rejected_depth_mad_m": depth_mad}
    sample_pixels = np.rint(uv[chosen]).astype(int)
    hit_pixels = np.zeros((height, width), np.uint8)
    hit_pixels[np.clip(sample_pixels[:, 1],0,height-1), np.clip(sample_pixels[:, 0],0,width-1)] = 1
    supported = cv2.dilate(hit_pixels, np.ones((7,7),np.uint8))
    coverage = float(((supported>0)&(mask>0)).sum()/max(1,int(mask.sum())))
    estimate = {"modality": "lidar", "position_m": center.tolist(), "uncertainty_m": max(.01,depth_mad),
                "valid_point_count": int(len(chosen)), "coverage_fraction": coverage,
                "evidence_indices": projected["raw_indices"][chosen].astype(int).tolist()[:100]}
    return estimate, {**stats, "front_layer_z_m": front_layer, "supported_mask_fraction_3px": coverage,
                      "depth_mad_m":depth_mad,"object_geometry_spread_m":geometry_spread,
                      "chosen_indices": chosen}


def stereo_object(result, left_mask, left_pose, public_pose, config):
    depth = result["depth_z_m"]
    valid = (left_mask > 0) & (result["valid_mask"] > 0) & np.isfinite(depth)
    pixels = np.argwhere(valid)
    if len(pixels) < config["min_stereo_pixels"]: return None, {"valid_pixels": int(len(pixels))}
    # Dense pixels are correlated; a fixed stride avoids pretending they are independent samples.
    pixels = pixels[::max(1, len(pixels)//2000)]
    rows, cols = pixels[:, 0], pixels[:, 1]
    P = result["P1"]
    z = depth[rows, cols]
    rect_xyz = np.column_stack(((cols-P[0, 2])*z/P[0, 0], (rows-P[1, 2])*z/P[1, 1], z))
    # The ideal parallel pair has identity rectification rotation. Verify rather than assume it.
    if not np.allclose(result["R1"], np.eye(3), atol=1e-5):
        rect_xyz = result["R1"].T @ rect_xyz.T
        rect_xyz = rect_xyz.T
    left_p, left_r = left_pose
    world = left_r.apply(rect_xyz) + left_p
    public_p, public_r = public_pose
    optical = world_to_camera(world, public_p, public_r)
    center, geometry_spread = robust_center(optical)
    depth_mad = float(1.4826*np.median(np.abs(optical[:,2]-np.median(optical[:,2]))))
    coverage = float(valid.sum()/max(1, int(left_mask.sum())))
    estimate = {"modality": "stereo", "position_m": center.tolist(), "uncertainty_m": max(.01,depth_mad),
                "valid_point_count": int(valid.sum()), "coverage_fraction": coverage,
                "evidence_indices": [[int(y), int(x)] for y, x in pixels[:40]]}
    return estimate, {"valid_pixels": int(valid.sum()), "mask_pixels": int(left_mask.sum()),
                      "coverage": coverage, "sampled_for_center": int(len(pixels)),
                      "depth_mad_m":depth_mad,"object_geometry_spread_m":geometry_spread}


def object_record(phase, mode, estimate, exposure_s, config, evidence, conflict=False, reobserve=False):
    return {"object_id": config["object_id"], "phase": phase, "mode": mode,
            "status": "unknown" if estimate is None else "valid", "frame_id": "go2w_front_camera_optical_frame",
            "position_m": None if estimate is None else estimate["position_m"],
            "acquisition_time_s": exposure_s, "depth_source": mode,
            "valid_point_count": 0 if estimate is None else estimate["valid_point_count"],
            "coverage_fraction": 0. if estimate is None else estimate["coverage_fraction"],
            "quality": {"depth_spread_proxy_m": None if estimate is None else estimate["uncertainty_m"],
                        "interpretation": "robust optical-depth spread floor 1cm; not calibrated covariance or independent-sample standard error"},
            "uncertainty_m": None if estimate is None else estimate["uncertainty_m"],
            "evidence": evidence, "evidence_indices": [] if estimate is None else estimate["evidence_indices"],
            "conflict": bool(conflict), "request_reobservation": bool(reobserve),
            "reason": "cross_modal_disagreement" if conflict else ("insufficient_evidence" if estimate is None else None)}


def save_projection_figure(path, rgb, mask, projected, stats, title):
    uv, optical = projected["uv"], projected["points_optical"]
    front = frontmost_per_pixel(uv, optical[:, 2], projected["valid"], (rgb.shape[1], rgb.shape[0]))
    if len(front) > 4500: front = front[::max(1, len(front)//4500)]
    fig, ax = plt.subplots(figsize=(10, 7.5))
    ax.imshow(rgb)
    ax.contour(mask, levels=[.5], colors="yellow", linewidths=.8)
    if len(front):
        scatter = ax.scatter(uv[front, 0], uv[front, 1], c=optical[front, 2],
                             s=2, cmap="turbo", vmin=.5, vmax=5., alpha=.75)
        fig.colorbar(scatter, ax=ax, label="LiDAR optical Z (m)")
    ax.set_title(f"{title}: camera time projection; mask hits {stats['inside_mask']}, layer {stats['layer_kept']}")
    ax.set_xlim(0, rgb.shape[1]); ax.set_ylim(rgb.shape[0], 0)
    fig.tight_layout(); fig.savefig(path, dpi=130); plt.close(fig)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--run", type=Path, required=True)
    args = p.parse_args()
    run = args.run.resolve()
    config = json.loads((run/"config_snapshot.json").read_text())
    root = Path(__file__).resolve().parents[2]
    annotations_path = root/"configs/sensors/stage5_manual_polygons_dev.json"
    annotation = json.loads(annotations_path.read_text()) if annotations_path.exists() else None
    params = json.loads((root/config["sgbm_config"]).read_text())
    trajectory = json.loads((run/"trajectory.json").read_text())
    lidar_path = PoseTimeline(trajectory, "lidar")
    public_path = PoseTimeline(trajectory, "public")
    left_path = PoseTimeline(trajectory, "left")
    checks = core_logic_checks()
    save_json(run/"core_logic_checks.json", checks)
    rows, sensitivity, frame_reports = [], [], []
    for phase in ("static", "straight", "turn", "parked"):
        folder = run/"captures"/phase
        meta = json.loads((folder/"capture.json").read_text())
        raw = np.load(folder/"lidar_raw.npz")
        exposure = float(meta["camera_exposure_render_time_s"])
        public_pose = (public_path.at(exposure)[0][0], public_path.at(exposure)[1][0])
        left_pose = (left_path.at(exposure)[0][0], left_path.at(exposure)[1][0])
        rgb = np.asarray(cv2.cvtColor(cv2.imread(str(folder/"public_rgb.png")), cv2.COLOR_BGR2RGB))
        left_rgb = cv2.imread(str(folder/"left_rgb.png"))
        right_rgb = cv2.imread(str(folder/"right_rgb.png"))
        mask = red_component_mask(rgb, config["manual_red_mask"])
        if int(mask.sum()) < config["manual_red_mask"]["min_pixels"]:
            raise RuntimeError(f"{phase}: handcrafted object mask too small ({mask.sum()})")
        cv2.imwrite(str(folder/"auto_red_component_mask.png"), mask*255)
        inspected = inspected_polygon_mask(annotation, phase, folder/"public_rgb.png", mask.shape)
        if inspected is not None:
            cv2.imwrite(str(folder/"manual_polygon_mask.png"), inspected*255)
        calib = {"K_left": meta["intrinsics"]["left"], "K_right": meta["intrinsics"]["right"],
                 "D_left": [0.]*5, "D_right": [0.]*5, "R_left_to_right": np.eye(3).tolist(),
                 "T_left_to_right_m": [-.1, 0., 0.]}
        stereo = run_matcher(left_rgb, right_rgb, calib, params)
        # Reconstruct rectification rotation for interpreting returned rectified optical Z.
        R1, _, _, _, _, _, _ = cv2.stereoRectify(np.asarray(calib["K_left"]), np.zeros(5),
            np.asarray(calib["K_right"]), np.zeros(5), (640,480), np.eye(3), np.array([-.1,0,0]), flags=0, alpha=0)
        stereo["R1"] = R1
        left_mask = red_component_mask(cv2.cvtColor(stereo["rect_left"], cv2.COLOR_BGR2RGB), config["manual_red_mask"])
        cv2.imwrite(str(folder/"left_rectified_mask.png"), left_mask*255)
        stereo_est, stereo_stats = stereo_object(stereo, left_mask, left_pose, public_pose, config["association"])
        np.save(folder/"stereo_depth_z_m.npy", stereo["depth_z_m"])
        projected = project_lidar(raw, lidar_path, public_pose, meta["intrinsics"]["public"], [0.]*5, (640,480),
                                  readout_s=meta["simulation_time_s"])
        lidar_est, lidar_stats = lidar_object(projected, mask, config["association"])
        save_projection_figure(folder/"projection_oracle_deskew.png", rgb, mask, projected, lidar_stats, phase+" automatic segment")
        inspected_stats, inspected_error_m, manual_auto_iou = None, None, None
        if inspected is not None:
            inspected_est, inspected_stats = lidar_object(projected, inspected, config["association"])
            save_projection_figure(folder/"projection_manual_polygon.png", rgb, inspected, projected, inspected_stats,
                                   phase+" inspected polygon")
            if inspected_est is not None and lidar_est is not None:
                inspected_error_m = float(np.linalg.norm(np.asarray(inspected_est["position_m"])-np.asarray(lidar_est["position_m"])))
            manual_auto_iou = float(((inspected>0)&(mask>0)).sum()/max(1,((inspected>0)|(mask>0)).sum()))
        raw_projected = project_lidar(raw, lidar_path, public_pose, meta["intrinsics"]["public"], [0.]*5,
                                      (640,480), deskew=False, readout_s=meta["simulation_time_s"])
        no_deskew_est, no_deskew_stats = lidar_object(raw_projected, mask, config["association"])
        save_projection_figure(folder/"projection_no_deskew.png", rgb, mask, raw_projected, no_deskew_stats, phase+" raw pose")
        truth_world = np.asarray(config["object_surface_center_world_m"], float)
        truth_optical = world_to_camera(truth_world[None,:], *public_pose)[0]
        evidence = {"capture": str((folder/"capture.json").relative_to(run)),
                    "lidar_raw": str((folder/"lidar_raw.npz").relative_to(run)),
                    "public_rgb": str((folder/"public_rgb.png").relative_to(run)),
                    "stereo_depth": str((folder/"stereo_depth_z_m.npy").relative_to(run)),
                    "segmentation_mask": str((folder/"auto_red_component_mask.png").relative_to(run)),
                    "inspected_polygon_mask": None if inspected is None else str((folder/"manual_polygon_mask.png").relative_to(run))}
        candidates = {"lidar_only": lidar_est, "stereo_only": stereo_est}
        hybrid, source, conflict, reobserve = decide_hybrid(lidar_est, stereo_est,
                                      config["association"]["max_lidar_stereo_disagreement_m"])
        modality_disagreement = None if lidar_est is None or stereo_est is None else float(np.linalg.norm(
            np.asarray(lidar_est["position_m"])-np.asarray(stereo_est["position_m"])))
        candidates["hybrid"] = hybrid
        for mode, estimate in candidates.items():
            record = object_record(phase, source if mode=="hybrid" else mode, estimate, exposure, config,
                                   evidence, conflict if mode=="hybrid" else False,
                                   reobserve if mode=="hybrid" else estimate is None)
            record["requested_mode"] = mode
            record["cross_modal_disagreement_m"] = modality_disagreement
            record["conflict_threshold_m"] = config["association"]["max_lidar_stereo_disagreement_m"]
            record["truth_position_for_evaluation_m"] = truth_optical.tolist()
            record["position_error_m"] = None if estimate is None else float(np.linalg.norm(np.asarray(estimate["position_m"])-truth_optical))
            rows.append(record)
        for offset_ms in config["sensitivity"]["time_offsets_ms"]:
            for axis, dx, yaw in (("time_only",0.,0.),):
                variant = project_lidar(raw, lidar_path, public_pose, meta["intrinsics"]["public"], [0.]*5, (640,480),
                                        offset_s=offset_ms/1000, translation_x_m=dx, yaw_deg=yaw,
                                        readout_s=meta["simulation_time_s"])
                estimate, stats = lidar_object(variant, mask, config["association"])
                sensitivity.append({"phase":phase,"type":axis,"time_offset_ms":offset_ms,"translation_x_m":dx,
                    "yaw_deg":yaw,"valid_points":0 if estimate is None else estimate["valid_point_count"],
                    "error_m":None if estimate is None else float(np.linalg.norm(np.asarray(estimate["position_m"])-truth_optical)),
                    "time_rejected":variant["time_rejected"]})
        for dx in config["sensitivity"]["estimated_extrinsic_translation_x_m"]:
            for yaw in config["sensitivity"]["estimated_extrinsic_yaw_deg"]:
                if dx == 0 and yaw == 0: continue
                variant = project_lidar(raw, lidar_path, public_pose, meta["intrinsics"]["public"], [0.]*5,
                                        (640,480), translation_x_m=dx, yaw_deg=yaw,
                                        readout_s=meta["simulation_time_s"])
                estimate, stats = lidar_object(variant, mask, config["association"])
                sensitivity.append({"phase":phase,"type":"estimated_extrinsic_only","time_offset_ms":0,
                    "translation_x_m":dx,"yaw_deg":yaw,"valid_points":0 if estimate is None else estimate["valid_point_count"],
                    "error_m":None if estimate is None else float(np.linalg.norm(np.asarray(estimate["position_m"])-truth_optical)),
                    "time_rejected":variant["time_rejected"]})
        frame_reports.append({"phase":phase,"camera_minus_scan_end_s":meta["camera_minus_scan_end_s"],
            "mask_pixels":int(mask.sum()),"lidar":{k:v for k,v in lidar_stats.items() if k!="chosen_indices"},
            "stereo":stereo_stats,"stereo_runtime_s":stereo["elapsed_s"],
            "cross_modal_disagreement_m":modality_disagreement,
            "inspected_polygon_lidar":None if inspected_stats is None else {k:v for k,v in inspected_stats.items() if k!="chosen_indices"},
            "inspected_vs_auto_mask_iou":manual_auto_iou,
            "inspected_vs_auto_object_center_delta_m":inspected_error_m,
            "no_deskew_lidar":{k:v for k,v in no_deskew_stats.items() if k!="chosen_indices"},
            "oracle_vs_no_deskew_center_delta_m":None if lidar_est is None or no_deskew_est is None else
                float(np.linalg.norm(np.asarray(lidar_est["position_m"])-np.asarray(no_deskew_est["position_m"])))})
    # Deliberately injected conflict checks behavior without changing either captured sensor stream.
    baseline_lidar = next((r for r in rows if r["phase"]=="static" and r["requested_mode"]=="lidar_only" and r["status"]=="valid"), None)
    if baseline_lidar is None: raise RuntimeError("no static lidar object geometry for conflict check")
    original = {"position_m":baseline_lidar["position_m"], "uncertainty_m":baseline_lidar["uncertainty_m"], "modality":"lidar"}
    injected = {"position_m":(np.asarray(original["position_m"])+[0,0,.5]).tolist(), "uncertainty_m":.02, "modality":"stereo"}
    _, mode, conflict, reobserve = decide_hybrid(original, injected, config["association"]["max_lidar_stereo_disagreement_m"])
    conflict_case = {"status":"synthetic_estimate_injection_for_logic_check", "injected_stereo_bias_z_m":.5,
                     "decision":mode,"conflict":conflict,"request_reobservation":reobserve,
                     "original_sensor_observations_unchanged":True}
    if not (mode=="unknown" and conflict and reobserve): raise RuntimeError("conflict gate failed")
    save_json(run/"conflict_case.json", conflict_case)
    scan_clock = json.loads((run/"scan_clock.json").read_text())
    camera_ticks = sorted({float(r["public_rendering_time_s"]) for r in trajectory if r["public_rendering_time_s"] is not None})
    timing = []
    for scan in scan_clock:
        closest = min(camera_ticks, key=lambda x:abs(x-scan["scan_end_s"]))
        timing.append({"scan_end_s":scan["scan_end_s"],"nearest_public_exposure_s":closest,
                       "software_pair_delta_ms":1000*(closest-scan["scan_end_s"]),"phase":scan["phase"]})
    save_json(run/"object_geometry.json", rows)
    save_json(run/"frame_reports.json", frame_reports)
    save_json(run/"sensitivity.json", sensitivity)
    save_json(run/"sync_differences.json", timing)
    with (run/"object_geometry.csv").open("w", newline="") as f:
        fields = ["phase","requested_mode","status","depth_source","position_error_m","valid_point_count","coverage_fraction","uncertainty_m","conflict","request_reobservation"]
        writer=csv.DictWriter(f,fieldnames=fields); writer.writeheader()
        writer.writerows([{k:r[k] for k in fields} for r in rows])
    with (run/"sensitivity.csv").open("w", newline="") as f:
        writer=csv.DictWriter(f,fieldnames=list(sensitivity[0])); writer.writeheader(); writer.writerows(sensitivity)
    fig, ax = plt.subplots(figsize=(8,4))
    ax.hist([x["software_pair_delta_ms"] for x in timing], bins=12, color="#2874a6")
    ax.set_xlabel("nearest public exposure minus scan end (ms)");ax.set_ylabel("full scans")
    ax.set_title("Software pairing offsets; not hardware synchronization")
    fig.tight_layout();fig.savefig(run/"sync_delta_histogram.png",dpi=130);plt.close(fig)
    motion_observed = {}
    for phase in ("static", "straight", "turn", "parked"):
        subset = [r for r in trajectory if r.get("phase") == phase]
        if len(subset) < 2: continue
        positions = np.asarray([r["base"]["position_world_m"] for r in subset], float)
        q = np.asarray([r["base"]["orientation_world_wxyz"] for r in subset], float)
        yaws = np.unwrap(Rotation.from_quat(q[:,[1,2,3,0]]).as_euler("xyz")[:,2])
        motion_observed[phase] = {"duration_s":subset[-1]["time_s"]-subset[0]["time_s"],
            "base_displacement_world_m":(positions[-1]-positions[0]).tolist(),
            "base_path_length_xy_m":float(np.linalg.norm(np.diff(positions[:,:2],axis=0),axis=1).sum()),
            "yaw_change_rad":float(yaws[-1]-yaws[0])}
    save_json(run/"motion_observed.json",motion_observed)
    summary={"status":"analysis_complete","phase_count":4,"object_rows":len(rows),
             "sync_delta_ms_median_abs":float(np.median(np.abs([x["software_pair_delta_ms"] for x in timing]))),
             "retained_scan_count":len(timing),"core_logic_checks":checks,
             "actual_conflicts":[r["phase"] for r in rows if r["requested_mode"]=="hybrid" and r["conflict"]],
             "injected_conflict":conflict_case,
             "sensitivity_runs":len(sensitivity),"motion_deskew":"Isaac truth pose oracle for development, not an estimated IMU pose"}
    summary["motion_observed"] = motion_observed
    summary["manual_polygon_validation"] = "completed on image-SHA-pinned development frames" if all(r["inspected_polygon_lidar"] is not None for r in frame_reports) else "not available for this run; automatic segmentation only"
    save_json(run/"analysis_summary.json",summary)
    print("STAGE5_ANALYSIS_COMPLETE", json.dumps(summary),flush=True)


if __name__=="__main__": main()
