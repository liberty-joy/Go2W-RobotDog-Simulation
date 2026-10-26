"""Compare a fixed candidate set using development captures only."""
import argparse
import csv
import json
from pathlib import Path

import numpy as np

from process_stage3_stereo import load_image_pair, run_matcher


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--run", type=Path, required=True)
    parser.add_argument("--candidates", type=Path, default=Path("configs/sensors/stage3_sgbm_candidates.json"))
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    root, output = args.run.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    records = json.loads((root / "captures.json").read_text())["rows"]
    dev = [row for row in records if row["split"] == "development"]
    if not dev:
        raise RuntimeError("development split is empty; tuner will not open held-out captures")
    calibration = json.loads((root / "calibration.json").read_text())
    candidates = json.loads(args.candidates.read_text())
    results = []
    for candidate in candidates["candidates"]:
        name = candidate["name"]
        params = {key: value for key, value in candidate.items() if key != "name"}
        n_gt = n_accepted = n_within10 = 0
        abs_sum = sq_sum = runtime_sum = 0.0
        classes = {}
        for record in dev:
            left, right = load_image_pair(root, record["scenario_id"])
            estimate = run_matcher(left, right, calibration, params)
            # Ground truth is opened after matching and only for development scoring.
            truth = np.load(root / record["truth_path"] / "left_optical_z_m.npy")
            valid = (estimate["valid_mask"] > 0) & np.isfinite(truth)
            error = estimate["depth_z_m"][valid] - truth[valid]
            gt_count, accepted = int(np.isfinite(truth).sum()), int(valid.sum())
            within = int((np.abs(error) <= .10).sum())
            n_gt += gt_count; n_accepted += accepted; n_within10 += within
            abs_sum += float(np.abs(error).sum()); sq_sum += float(np.square(error).sum())
            runtime_sum += estimate["elapsed_s"]
            group = classes.setdefault(record["scene_class"], {"scenes":0,"ground_truth_pixels":0,
                "accepted_pixels":0,"within_10cm_pixels":0,"abs_error_sum_m":0.0})
            group["scenes"] += 1; group["ground_truth_pixels"] += gt_count
            group["accepted_pixels"] += accepted; group["within_10cm_pixels"] += within
            group["abs_error_sum_m"] += float(np.abs(error).sum())
        results.append({"candidate":name,"dev_score_within_10cm":n_within10/max(1,n_gt),
            "development_coverage":n_accepted/max(1,n_gt),"accepted_pixel_mae_m":abs_sum/max(1,n_accepted),
            "accepted_pixel_rmse_m":float(np.sqrt(sq_sum/max(1,n_accepted))) if n_accepted else None,
            "mean_runtime_s_per_scene":runtime_sum/len(dev),"per_class":classes,"params":params})
    best = max(results,key=lambda result:(result["dev_score_within_10cm"],result["development_coverage"],-result["accepted_pixel_mae_m"]))
    selected = {**best["params"],"method":"OpenCV StereoSGBM; selected on development set only",
        "selected_candidate":best["candidate"],"selection_metric":candidates["selection_metric"],
        "heldout_test_used_for_tuning":False,"opencv_disparity_storage":"signed int16 fixed point; divide by 16 exactly once",
        "development_source_run":str(root)}
    (output/"candidate_results.json").write_text(json.dumps({"development_only":True,
        "development_scenarios":[row["scenario_id"] for row in dev],"heldout_scenarios_opened":0,
        "selection_metric":candidates["selection_metric"],"candidates":results,"selected":best["candidate"]},indent=2))
    fields=["candidate","dev_score_within_10cm","development_coverage","accepted_pixel_mae_m","accepted_pixel_rmse_m","mean_runtime_s_per_scene"]
    with (output/"candidate_results.csv").open("w",newline="") as stream:
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows([{key:row[key] for key in fields} for row in results])
    (output/"selected_sgbm.json").write_text(json.dumps(selected,indent=2))
    print(json.dumps({"selected_candidate":best["candidate"],"candidate_summary":[{key:row[key] for key in
        ("candidate","dev_score_within_10cm","development_coverage","accepted_pixel_mae_m","accepted_pixel_rmse_m")}
        for row in results],"heldout_test_used_for_tuning":False},indent=2))


if __name__ == "__main__":
    main()
