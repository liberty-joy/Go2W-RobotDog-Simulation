"""Explicit metric transforms, timestamp interpolation, projection and object gating."""
import numpy as np
import cv2
from scipy.spatial.transform import Rotation, Slerp


def pose_arrays(record):
    p = np.asarray(record["position_world_m"], dtype=np.float64)
    w, x, y, z = record["orientation_world_wxyz"]
    return p, Rotation.from_quat([x, y, z, w])


class PoseTimeline:
    def __init__(self, rows, key):
        self.times = np.asarray([r["time_s"] for r in rows], dtype=np.float64)
        self.positions = np.asarray([r[key]["position_world_m"] for r in rows], dtype=np.float64)
        quaternion = np.asarray([r[key]["orientation_world_wxyz"] for r in rows], dtype=np.float64)
        if np.any(np.diff(self.times) <= 0):
            raise ValueError("pose timeline must have strictly increasing simulation time")
        self.rotations = Slerp(self.times, Rotation.from_quat(quaternion[:, [1, 2, 3, 0]]))

    def at(self, query_s):
        t = np.atleast_1d(np.asarray(query_s, dtype=np.float64))
        if not np.all(np.isfinite(t)) or np.any(t < self.times[0]) or np.any(t > self.times[-1]):
            raise ValueError("requested pose time outside recorded trajectory")
        position = np.stack([np.interp(t, self.times, self.positions[:, i]) for i in range(3)], axis=1)
        return position, self.rotations(t)


def world_to_camera(points_world_m, camera_position_world_m, camera_rotation_world):
    """T_camera_world maps metric world XYZ into ROS optical (+x right,+y down,+z forward)."""
    return camera_rotation_world.inv().apply(np.asarray(points_world_m) - camera_position_world_m)


def project_optical(points_optical_m, K, D, image_size):
    """Project optical XYZ, rejecting behind-camera and out-of-image points."""
    xyz = np.asarray(points_optical_m, dtype=np.float64)
    width, height = image_size
    valid = np.isfinite(xyz).all(axis=1) & (xyz[:, 2] > 0.05)
    uv = np.full((len(xyz), 2), np.nan, dtype=np.float64)
    if valid.any():
        pts = xyz[valid]
        uv[valid] = cv2.projectPoints(pts, np.zeros(3), np.zeros(3),
                                      np.asarray(K, np.float64), np.asarray(D, np.float64))[0].reshape(-1, 2)
    valid &= np.isfinite(uv).all(axis=1) & (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return uv, valid


def frontmost_per_pixel(uv, optical_z_m, valid, image_size):
    """Keep nearest measured return per rounded image pixel; other depths are occluded."""
    width, height = image_size
    indices = np.flatnonzero(valid)
    if not len(indices): return indices
    pixel = np.rint(uv[indices]).astype(np.int64)
    pixel[:, 0] = np.clip(pixel[:, 0], 0, width - 1)
    pixel[:, 1] = np.clip(pixel[:, 1], 0, height - 1)
    key = pixel[:, 1] * width + pixel[:, 0]
    order = np.lexsort((optical_z_m[indices], key))
    sorted_key = key[order]
    first = np.r_[True, sorted_key[1:] != sorted_key[:-1]]
    return indices[order[first]]


def red_component_mask(rgb, rule):
    """Classical red-component segmentation with hand-set thresholds; no learned detector."""
    r, g, b = [rgb[:, :, i].astype(np.float32) for i in range(3)]
    binary = ((r >= rule["r_min"]) & (r >= rule["r_over_g"] * np.maximum(g, 1)) &
              (r >= rule["r_over_b"] * np.maximum(b, 1))).astype(np.uint8)
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = np.zeros_like(binary)
    if contours:
        cv2.drawContours(out, [max(contours, key=cv2.contourArea)], -1, 1, thickness=-1)
    return out


def robust_center(points):
    pts = np.asarray(points, np.float64)
    if not len(pts): return None, None
    center = np.median(pts, axis=0)
    radial = np.linalg.norm(pts - center, axis=1)
    mad = 1.4826 * np.median(np.abs(radial - np.median(radial)))
    return center, float(max(0.005, mad))


def decide_hybrid(lidar, stereo, max_disagreement_m):
    """Correlated modalities select one estimate; conflicting estimates become unknown."""
    if lidar is None and stereo is None: return None, "unknown", False, True
    if lidar is None: return stereo, "stereo_only", False, False
    if stereo is None: return lidar, "lidar_only", False, False
    disagreement = float(np.linalg.norm(np.asarray(lidar["position_m"]) - np.asarray(stereo["position_m"])))
    if disagreement > max_disagreement_m: return None, "unknown", True, True
    choice = min((lidar, stereo), key=lambda x: x["uncertainty_m"])
    return choice, "hybrid_selected_" + choice["modality"], False, False
