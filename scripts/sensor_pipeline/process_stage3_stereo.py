"""Rectify and match Stage 3 RGB stereo captures; ground truth is loaded only afterward for evaluation."""
import argparse
import csv
import json
import time
from pathlib import Path

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def load_image_pair(run_dir, scenario_id):
    """Matcher input boundary: deliberately reads only the two rendered RGB images."""
    folder = run_dir / "captures" / scenario_id
    left = cv2.imread(str(folder / "left_rgb.png"), cv2.IMREAD_COLOR)
    right = cv2.imread(str(folder / "right_rgb.png"), cv2.IMREAD_COLOR)
    if left is None or right is None or left.shape != right.shape:
        raise RuntimeError(f"missing or shape-mismatched stereo images for {scenario_id}")
    return left, right


def run_matcher(left_bgr, right_bgr, calibration, params):
    """Returns rectified pair, disparity (pixels), optical-axis Z (m), and explicit masks."""
    h, w = left_bgr.shape[:2]
    K1 = np.asarray(calibration["K_left"], np.float64)
    K2 = np.asarray(calibration["K_right"], np.float64)
    D1, D2 = np.asarray(calibration["D_left"], np.float64), np.asarray(calibration["D_right"], np.float64)
    R, T = np.asarray(calibration["R_left_to_right"], np.float64), np.asarray(calibration["T_left_to_right_m"], np.float64)
    start = time.perf_counter()
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        K1, D1, K2, D2, (w, h), R, T, flags=0, alpha=0
    )
    map1x, map1y = cv2.initUndistortRectifyMap(K1, D1, R1, P1, (w, h), cv2.CV_32FC1)
    map2x, map2y = cv2.initUndistortRectifyMap(K2, D2, R2, P2, (w, h), cv2.CV_32FC1)
    rect_left = cv2.remap(left_bgr, map1x, map1y, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_bgr, map2x, map2y, cv2.INTER_LINEAR)
    gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)
    block = int(params["block_size"])
    channels = 1
    p1 = int(params.get("P1", 8 * channels * block * block))
    p2 = int(params.get("P2", 32 * channels * block * block))
    ndisp = int(params["num_disparities"])
    matcher = cv2.StereoSGBM_create(
        minDisparity=int(params["min_disparity"]), numDisparities=ndisp, blockSize=block,
        P1=p1, P2=p2, disp12MaxDiff=int(params["disp12_max_diff"]),
        preFilterCap=int(params["pre_filter_cap"]), uniquenessRatio=int(params["uniqueness_ratio"]),
        speckleWindowSize=int(params["speckle_window_size"]), speckleRange=int(params["speckle_range"]),
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    right_matcher = cv2.StereoSGBM_create(
        minDisparity=-ndisp, numDisparities=ndisp, blockSize=block,
        P1=p1, P2=p2, disp12MaxDiff=int(params["disp12_max_diff"]),
        preFilterCap=int(params["pre_filter_cap"]), uniquenessRatio=int(params["uniqueness_ratio"]),
        speckleWindowSize=int(params["speckle_window_size"]), speckleRange=int(params["speckle_range"]),
        mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY,
    )
    raw_left = matcher.compute(gray_left, gray_right).astype(np.int16)
    raw_right = right_matcher.compute(gray_right, gray_left).astype(np.int16)
    # OpenCV StereoSGBM encodes disparity in signed 4-bit fixed point: divide by 16 exactly once.
    disp_left = raw_left.astype(np.float32) / 16.0
    disp_right = raw_right.astype(np.float32) / 16.0
    min_disp = int(params["min_disparity"])
    raw_valid = (raw_left > (min_disp - 1) * 16) & (disp_left > 0.0)
    x = np.broadcast_to(np.arange(w, dtype=np.float32)[None, :], (h, w))
    xr = np.rint(x - disp_left).astype(np.int32)
    in_bounds = (xr >= 0) & (xr < w)
    xr_safe = np.clip(xr, 0, w - 1)
    rows = np.broadcast_to(np.arange(h, dtype=np.int32)[:, None], (h, w))
    sampled_right = disp_right[rows, xr_safe]
    raw_right_valid = raw_right[rows, xr_safe] > (-ndisp - 1) * 16
    lr_error = np.abs(disp_left + sampled_right)
    lr_consistent = in_bounds & raw_right_valid & (lr_error <= float(params["lr_tolerance_px"]))
    lr_rejected = raw_valid & ~lr_consistent
    valid = raw_valid & lr_consistent
    fx_rect = float(P1[0, 0])
    baseline_rect = abs(float(P2[0, 3]) / float(P2[0, 0]))
    depth = np.full((h, w), np.nan, dtype=np.float32)
    depth[valid] = fx_rect * baseline_rect / disp_left[valid]
    valid &= (depth >= float(params["min_depth_m"])) & (depth <= float(params["max_depth_m"]))
    depth[~valid] = np.nan
    elapsed = time.perf_counter() - start
    return {"rect_left": rect_left, "rect_right": rect_right, "disparity_px": disp_left,
            "depth_z_m": depth, "valid_mask": valid.astype(np.uint8),
            "raw_valid_mask": raw_valid.astype(np.uint8), "lr_rejected_mask": lr_rejected.astype(np.uint8),
            "lr_error_px": lr_error.astype(np.float32), "P1": P1, "P2": P2, "Q": Q,
            "roi1": tuple(map(int, roi1)), "roi2": tuple(map(int, roi2)),
            "fx_rect_px": fx_rect, "baseline_rect_m": baseline_rect,
            "elapsed_s": elapsed, "raw_left": raw_left, "raw_right": raw_right}


def masked_metrics(depth, truth, valid_mask, geom_occlusion):
    gt_valid = np.isfinite(truth) & (truth > 0)
    accepted = (valid_mask > 0) & gt_valid & np.isfinite(depth)
    errors = depth[accepted] - truth[accepted]
    result = {"ground_truth_pixels": int(gt_valid.sum()), "accepted_pixels": int(accepted.sum()),
              "coverage": float(accepted.sum() / max(1, gt_valid.sum())),
              "mae_m": float(np.mean(np.abs(errors))) if errors.size else None,
              "rmse_m": float(np.sqrt(np.mean(errors ** 2))) if errors.size else None,
              "median_abs_error_m": float(np.median(np.abs(errors))) if errors.size else None,
              "abs_rel": float(np.mean(np.abs(errors) / truth[accepted])) if errors.size else None,
              "within_5cm_fraction": float(np.mean(np.abs(errors) <= .05)) if errors.size else None}
    for label, region in (("occluded", geom_occlusion > 0), ("non_occluded", geom_occlusion == 0)):
        region_gt = gt_valid & region
        region_ok = accepted & region
        e = depth[region_ok] - truth[region_ok]
        result[label + "_gt_pixels"] = int(region_gt.sum())
        result[label + "_coverage"] = float(region_ok.sum() / max(1, region_gt.sum()))
        result[label + "_mae_m"] = float(np.mean(np.abs(e))) if e.size else None
    return result


def save_preview(folder, result, truth, geom_occ, metrics):
    rect_l = cv2.cvtColor(result["rect_left"], cv2.COLOR_BGR2RGB)
    rect_r = cv2.cvtColor(result["rect_right"], cv2.COLOR_BGR2RGB)
    fig, axes = plt.subplots(1, 2, figsize=(14, 5), constrained_layout=True)
    axes[0].imshow(np.concatenate([rect_l, rect_r], axis=1)); axes[0].set_title("Rectified L | R; epipolar rows")
    axes[0].axis("off")
    for row in [60, 120, 180, 240, 300, 360, 420]:
        axes[0].axhline(row, color="#ff3030", alpha=.65, linewidth=.7)
    disp = result["disparity_px"].copy(); disp[result["valid_mask"] == 0] = np.nan
    im = axes[1].imshow(disp, cmap="turbo", vmin=0, vmax=64); axes[1].set_title("SGBM disparity (px), LR-checked")
    fig.colorbar(im, ax=axes[1], shrink=.8)
    fig.savefig(folder / "epipolar_disparity.png", dpi=140); plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), constrained_layout=True)
    estimated = result["depth_z_m"]
    depth_im = axes[0].imshow(estimated, cmap="viridis", vmin=0.5, vmax=5.5)
    axes[0].set_title("Estimated optical Z (m; invalid=transparent)"); fig.colorbar(depth_im, ax=axes[0], shrink=.75)
    error = np.full(truth.shape, np.nan, dtype=np.float32)
    accepted = (result["valid_mask"] > 0) & np.isfinite(truth)
    error[accepted] = np.abs(estimated[accepted] - truth[accepted])
    err_im = axes[1].imshow(error, cmap="magma", vmin=0, vmax=.25); axes[1].set_title("Absolute error (m)"); fig.colorbar(err_im, ax=axes[1], shrink=.75)
    axes[2].imshow(geom_occ, cmap="gray", vmin=0, vmax=1); axes[2].set_title("Evaluation-only geometric occlusion")
    for ax in axes: ax.axis("off")
    fig.suptitle(f"coverage={metrics['coverage']:.3f}; MAE={metrics['mae_m']}")
    fig.savefig(folder / "depth_error.png", dpi=140); plt.close(fig)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--params", type=Path, default=Path("configs/sensors/stage3_sgbm.json"))
    parser.add_argument("--analysis-dir", default="analysis", help="new analysis subdirectory; an existing empty dir from a failed attempt is reusable")
    parser.add_argument("--split", choices=["all", "development", "heldout_test"], default="all")
    args = parser.parse_args()
    run_dir = args.run.resolve()
    capture_index = json.loads((run_dir / "captures.json").read_text())
    calibration = json.loads((run_dir / "calibration.json").read_text())
    params = json.loads(args.params.read_text())
    out_root = run_dir / args.analysis_dir
    if out_root.exists():
        if any(out_root.iterdir()):
            raise FileExistsError(f"refusing to overwrite non-empty analysis directory: {out_root}")
    else:
        out_root.mkdir(parents=True)
    rows = []
    selected_rows = [row for row in capture_index["rows"] if args.split == "all" or row["split"] == args.split]
    if not selected_rows:
        raise RuntimeError(f"no capture records for requested split {args.split}")
    for capture in selected_rows:
        scenario_id = capture["scenario_id"]
        # Estimate path: only the left/right RGB pair enters this function. GT is opened below for scoring.
        left, right = load_image_pair(run_dir, scenario_id)
        result = run_matcher(left, right, calibration, params)
        eval_dir = run_dir / capture["truth_path"]
        truth = np.load(eval_dir / "left_optical_z_m.npy")
        geom_occ = np.load(eval_dir / "geometric_occlusion_mask.npy")
        metrics = masked_metrics(result["depth_z_m"], truth, result["valid_mask"], geom_occ)
        folder = out_root / scenario_id
        folder.mkdir()
        for key in ("disparity_px", "depth_z_m", "valid_mask", "raw_valid_mask", "lr_rejected_mask", "lr_error_px"):
            np.save(folder / f"{key}.npy", result[key])
        np.save(folder / "disparity_raw_fixed16.npy", result["raw_left"])
        cv2.imwrite(str(folder / "rectified_left.png"), result["rect_left"])
        cv2.imwrite(str(folder / "rectified_right.png"), result["rect_right"])
        disp_png = np.zeros(left.shape[:2], np.uint8)
        valid = result["valid_mask"] > 0
        if valid.any():
            disp_png[valid] = np.clip(result["disparity_px"][valid] / params["display_max_disparity_px"] * 255, 0, 255).astype(np.uint8)
        cv2.imwrite(str(folder / "disparity_preview.png"), cv2.applyColorMap(disp_png, cv2.COLORMAP_TURBO))
        depth_png = np.zeros(left.shape[:2], np.uint8)
        if valid.any():
            depth_png[valid] = np.clip((result["depth_z_m"][valid] - params["min_depth_m"]) /
                                       (params["max_depth_m"] - params["min_depth_m"]) * 255, 0, 255).astype(np.uint8)
        cv2.imwrite(str(folder / "depth_preview.png"), cv2.applyColorMap(depth_png, cv2.COLORMAP_VIRIDIS))
        save_preview(folder, result, truth, geom_occ, metrics)
        row = {**capture, **metrics, "algorithm": "OpenCV StereoSGBM", "opencv_version": cv2.__version__,
               "compute_elapsed_s": result["elapsed_s"], "fx_rect_px": result["fx_rect_px"],
               "baseline_rect_m": result["baseline_rect_m"], "median_positive_disparity_px":
               float(np.median(result["disparity_px"][valid])) if valid.any() else None,
               "lr_rejected_fraction_of_raw": float((result["lr_rejected_mask"] > 0).sum() /
                   max(1, (result["raw_valid_mask"] > 0).sum())), "invalid_depth_encoding": "NaN + valid_mask.npy; never free-space/zero"}
        (folder / "metrics.json").write_text(json.dumps(row, indent=2))
        rows.append(row)

    fields = list(dict.fromkeys(key for row in rows for key in row))
    with (out_root / "per_scene.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    grouped = {}
    for row in rows:
        key = (row["split"], row["scene_class"])
        grouped.setdefault(key, []).append(row)
    summary = []
    for (split, scene_class), group in sorted(grouped.items()):
        valid_gt = sum(r["ground_truth_pixels"] for r in group)
        accepted = sum(r["accepted_pixels"] for r in group)
        abs_err_sq = sum((r["rmse_m"] or 0) ** 2 * r["accepted_pixels"] for r in group)
        maes = [(r["mae_m"], r["accepted_pixels"]) for r in group if r["mae_m"] is not None]
        summary.append({"split": split, "scene_class": scene_class, "scenes": len(group),
                        "coverage": accepted / max(1, valid_gt),
                        "weighted_mae_m": sum(x*w for x,w in maes) / max(1, sum(w for _,w in maes)) if maes else None,
                        "weighted_rmse_m": float(np.sqrt(abs_err_sq / max(1, accepted))) if accepted else None,
                        "mean_runtime_s": float(np.mean([r["compute_elapsed_s"] for r in group]))})
    with (out_root / "split_metrics.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary[0])); writer.writeheader(); writer.writerows(summary)
    sign_candidates = [r for r in rows if r["scene_class"] == "ideal_textured" and r["median_positive_disparity_px"] is not None]
    sign_scene = min(sign_candidates, key=lambda row: row["distance_m"]) if sign_candidates else None
    sign_ok = bool(sign_scene and sign_scene["median_positive_disparity_px"] is not None and sign_scene["median_positive_disparity_px"] > 0)
    result_json = {"status": "ANALYSIS_COMPLETE", "matcher_input": "left_rgb.png + right_rgb.png only",
                   "ground_truth_read_after_matching": True, "baseline_sign_check": sign_ok,
                   "positive_disparity_scene": sign_scene, "params": params, "opencv_version": cv2.__version__,
                   "opencv_build": cv2.getBuildInformation().splitlines()[:30], "scene_metrics": rows,
                   "split_metrics": summary, "split_processed": args.split,
                   "failure_policy": "capture/sync errors are retained; invalid disparity/depth is masked, not treated as free space"}
    (out_root / "results.json").write_text(json.dumps(result_json, indent=2))
    print(json.dumps({"status": result_json["status"], "scenes": len(rows), "baseline_sign_check": sign_ok,
                      "split_metrics": summary}, indent=2))


if __name__ == "__main__":
    main()
