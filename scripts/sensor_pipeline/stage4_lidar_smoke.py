"""Stage 4 raw RTX lidar capture; preserves only fields actually returned by Isaac."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


class Stage4LidarSmoke:
    def __init__(self, config_path, output_path, root, stage, base_path, public_camera):
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.rtx")
        enable_extension("isaacsim.sensors.experimental.rtx")
        enable_extension("isaacsim.sensors.experimental.physics")
        from isaacsim.sensors.experimental.rtx import Lidar
        from isaacsim.sensors.rtx import LidarRtx
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        from pxr import Gf, UsdGeom, UsdPhysics

        self.UsdGeom, self.UsdPhysics, self.Gf = UsdGeom, UsdPhysics, Gf
        self.config_path = Path(config_path).resolve()
        self.config = json.loads(self.config_path.read_text())
        self.rig_config = json.loads((Path(root) / self.config["rig_config"]).read_text())
        self.profile_path = Path(root) / self.config["source_profile"]
        self.profile = json.loads(self.profile_path.read_text())["profile"]
        self.out = Path(output_path).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        (self.out / "raw").mkdir(exist_ok=True)
        self.root, self.stage, self.base_path = Path(root), stage, base_path
        self.camera = public_camera
        self.done = False
        self.phase = "settle"
        self.phase_start = None
        self.target_index = -1
        self.current_target_name = None
        self.target_scans = 0
        self.motion_started = None
        self.frame_count = 0
        self.lidar_samples = []
        self.new_ros_scan = None
        self.ros_pointcloud_enabled = False
        self.ros_pointcloud_published = 0
        self.pc2_sample_saved = False
        self.imu_samples = []
        self.motion_samples = []
        self.selfscan_visibility_saved = {}
        self.selfscan_summary = {"status": "not_started", "hidden_scans": [], "visible_scans": []}
        self.selfscan_started = False
        self.scan_seen = set()
        self.frame_times = []
        self.axis = None
        self.scan_path = base_path + "/sensor_rig/Measurement/Stage4Lidar"
        rig_path = base_path + "/sensor_rig"
        rig_usd = (Path(root) / self.rig_config["rig_usd"]).resolve()
        stage.DefinePrim(rig_path, "Xform").GetReferences().AddReference(str(rig_usd))
        appearance = stage.GetPrimAtPath(rig_path + "/Appearance")
        if appearance and appearance.IsActive():
            appearance.SetActive(False)
        self.appearance_disabled_for_baseline = True

        attrs = self._make_lidar_attributes(self.profile)
        Lidar.create(self.scan_path, tick_rate=float(self.profile["scanRateBaseHz"]),
                     aux_output_level="FULL", translations=[self.rig_config["lidar"]["translation_base_m"]],
                     attributes={k: v for k, v in attrs.items() if not isinstance(v, list)})
        prim = stage.GetPrimAtPath(self.scan_path)
        for key, value in attrs.items():
            if isinstance(value, list):
                attribute = prim.GetAttribute(key)
                if not attribute.IsValid():
                    raise RuntimeError("Missing lidar attribute " + key)
                attribute.Set(value)
        self.lidar = LidarRtx(prim_path=self.scan_path)
        self.lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer", outputTimestamp=True, outputEmitterId=True)
        self.lidar.initialize()
        imu_path = base_path + "/sensor_rig/Measurement/Stage4Imu"
        self.imu = IMUSensor(IMU.create(imu_path, translations=[self.rig_config["imu"]["translation_base_m"]]))

        self.targets = stage.DefinePrim("/World/Stage4LidarTargets", "Xform")
        self._disable_demo_boxes()
        self._write_provenance()
        self.last_frame = {}

    @staticmethod
    def _make_lidar_attributes(profile):
        attrs = {
            "omni:sensor:Core:scanType": profile["scanType"].upper(),
            "omni:sensor:Core:rotationDirection": profile["rotationDirection"],
            "omni:sensor:Core:numberOfEmitters": profile["numberOfEmitters"],
            "omni:sensor:Core:numberOfChannels": profile["numberOfEmitters"],
            "omni:sensor:Core:patternFiringRateHz": int(profile["reportRateBaseHz"]),
            "omni:sensor:Core:scanRateBaseHz": int(profile["scanRateBaseHz"]),
            "omni:sensor:Core:nearRangeM": profile["nearRangeM"],
            "omni:sensor:Core:farRangeM": profile["farRangeM"],
            "omni:sensor:Core:azimuthErrorStd": 0.0,
            "omni:sensor:Core:elevationErrorStd": 0.0,
            "omni:sensor:Core:rangeAccuracyM": 0.0,
            "omni:sensor:Core:accumulateOutputs": True,
        }
        for key, value in profile["emitterStates"][0].items():
            attrs["omni:sensor:Core:emitterState:s001:" + key] = value
        attrs["omni:sensor:Core:emitterState:s001:channelId"] = list(range(1, profile["numberOfEmitters"] + 1))
        attrs["omni:sensor:Core:emitterState:s001:bank"] = []
        return attrs

    def _write_provenance(self):
        import hashlib
        files = {}
        for label, path in (("profile", self.profile_path), ("rig_config", self.root / self.config["rig_config"]),
                            ("rig_usd", self.root / self.rig_config["rig_usd"])):
            files[label] = {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        (self.out / "provenance.json").write_text(json.dumps({
            "isaac": "local Isaac Sim 6.2; source API read from installed environment",
            "config": self.config, "profile": self.profile, "files": files,
            "sensor_prim": self.scan_path, "coordinates": "raw annotator data preserved; coordinate frame determined from returned transform and static plane geometry",
        }, indent=2))

    def _disable_demo_boxes(self):
        for name in ("RedBox", "BlueBox"):
            prim = self.stage.GetPrimAtPath("/World/" + name)
            if prim and prim.IsActive():
                prim.SetActive(False)

    def _set_robot_visuals_hidden(self, hidden):
        from pxr import UsdGeom
        if hidden:
            for prim in self.stage.Traverse():
                path = str(prim.GetPath())
                if not path.startswith("/World/Go2W/Geometry/"):
                    continue
                if "/FrontCamera" in path or "/sensor_rig/Measurement" in path:
                    continue
                if not prim.IsA(UsdGeom.Gprim):
                    continue
                imageable = UsdGeom.Imageable(prim)
                attr = imageable.GetVisibilityAttr()
                if not attr.IsValid():
                    continue
                self.selfscan_visibility_saved[path] = attr.Get()
                attr.Set(UsdGeom.Tokens.invisible)
        else:
            for path, value in self.selfscan_visibility_saved.items():
                prim = self.stage.GetPrimAtPath(path)
                if prim and prim.IsValid():
                    UsdGeom.Imageable(prim).GetVisibilityAttr().Set(value)
            self.selfscan_visibility_saved.clear()

    def _add_target(self, distance, label):
        p = "/World/Stage4LidarTargets/target_" + label
        box = self.UsdGeom.Cube.Define(self.stage, p)
        sx, sy, sz = self.config["target_dimensions_m"]
        box.CreateSizeAttr(1.0)
        self.UsdGeom.XformCommonAPI(box).SetScale(self.Gf.Vec3f(sx, sy, sz))
        # Put the front face at the configured distance from the lidar origin.
        self.UsdGeom.XformCommonAPI(box).SetTranslate(self.Gf.Vec3d(distance + sx / 2, 0, 1.0))
        self.UsdPhysics.CollisionAPI.Apply(box.GetPrim())
        return box.GetPrim()

    @staticmethod
    def _target_name(distance):
        return f"d{int(round(float(distance) * 100)):03d}cm"

    @staticmethod
    def _arrays(scan):
        return {k: np.asarray(v) for k, v in scan.items() if isinstance(v, np.ndarray)}

    def _capture_scan(self, simulation_time, phase, robot):
        frame = self.lidar.get_current_frame()
        scan = frame.get("IsaacCreateRTXLidarScanBuffer", {})
        if not isinstance(scan, dict) or "data" not in scan:
            return
        arrays = self._arrays(scan)
        xyz = arrays.get("data", np.empty((0, 3), np.float32))
        if xyz.ndim != 2 or xyz.shape[1] != 3 or not len(xyz):
            return
        timestamp = arrays.get("timestamp", np.empty((0,), np.uint64))
        emitter = arrays.get("emitterId", np.empty((0,), np.uint32))
        signature = (len(xyz), int(timestamp[0]) if len(timestamp) else 0,
                     int(timestamp[-1]) if len(timestamp) else 0)
        # Rendering frame advances every 1/60 s but scan signature only changes when RTX publishes.
        scan_sig = signature[:3]
        if scan_sig in self.scan_seen:
            return
        self.scan_seen.add(scan_sig)
        sensor_position, sensor_orientation = self.lidar.get_world_pose()
        base_position, base_orientation = robot.get_world_pose()
        # Camera's ROS camera axes are +Z forward, +Y up; preserve this convention explicitly.
        camera_position, camera_orientation = self.camera.get_world_pose(camera_axes="ros")
        index = len(self.lidar_samples)
        arrays_out = {k: v for k, v in arrays.items() if v.size}
        np.savez_compressed(self.out / "raw" / f"scan_{index:04d}.npz", **arrays_out)
        phase_label = phase
        if phase == "static" and self.target_index >= 0:
            distance = self.config["static_target_distances_m"][self.target_index]
            phase_label = f"static_{distance:g}m"
        metadata = {
            "index": index, "phase": phase_label, "simulation_time_s": float(simulation_time),
            "rendering_time": frame.get("rendering_time"),
            "rendering_frame": list(frame.get("rendering_frame")) if isinstance(frame.get("rendering_frame"), tuple) else frame.get("rendering_frame"),
            "sensor_position_world_m": np.asarray(sensor_position).tolist(),
            "sensor_orientation_world_wxyz": np.asarray(sensor_orientation).tolist(),
            "base_position_world_m": np.asarray(base_position).tolist(),
            "base_orientation_world_wxyz": np.asarray(base_orientation).tolist(),
            "public_camera_position_world_m": np.asarray(camera_position).tolist(),
            "public_camera_orientation_world_wxyz": np.asarray(camera_orientation).tolist(),
            "scan_field_names": list(scan.keys()),
            "point_count": int(len(xyz)), "array_fields": {k: {"shape": list(v.shape), "dtype": str(v.dtype),
                "nonempty": bool(v.size)} for k, v in arrays.items()},
            "non_array_frame_fields": {k: str(type(v).__name__) for k, v in frame.items()
                if k != "IsaacCreateRTXLidarScanBuffer"},
            "transform": np.asarray(scan["transform"]).tolist() if "transform" in scan else None,
            "timestamp_ns_min": int(timestamp.min()) if len(timestamp) else None,
            "timestamp_ns_max": int(timestamp.max()) if len(timestamp) else None,
            "emitter_id_min": int(emitter.min()) if len(emitter) else None,
            "emitter_id_max": int(emitter.max()) if len(emitter) else None,
            "emitter_id_unique_count": int(len(np.unique(emitter))) if len(emitter) else 0,
        }
        self.lidar_samples.append(metadata)
        self.new_ros_scan = (metadata, arrays_out)
        self.last_frame = {"xyz": xyz.copy(), "phase": phase, "scan": arrays}
        if phase in ("selfscan_hidden", "selfscan_visible"):
            entry = {"index": index, "point_count": int(len(xyz)),
                     "emitter_id_unique_count": int(len(np.unique(emitter))) if len(emitter) else 0,
                     "timestamp_ns_min": int(timestamp.min()) if len(timestamp) else None,
                     "timestamp_ns_max": int(timestamp.max()) if len(timestamp) else None}
            self.selfscan_summary["hidden_scans" if phase == "selfscan_hidden" else "visible_scans"].append(entry)

    def _record_imu(self, simulation_time):
        reading = self.imu.get_data()
        if not reading:
            return
        sample = {k: np.asarray(v).tolist() for k, v in reading.items()}
        sample["simulation_time_s"] = float(simulation_time)
        sample["is_valid"] = bool(self.imu.get_sensor_reading().is_valid)
        self.imu_samples.append(sample)

    def update(self, simulation_time, robot):
        if self.done:
            return 0.0
        t = float(simulation_time)
        self.frame_count += 1
        self.frame_times.append(t)
        self._record_imu(t)
        if self.phase == "motion":
            position, orientation = robot.get_world_pose()
            self.motion_samples.append({"time_s": t, "position_world_m": np.asarray(position).tolist(),
                                        "orientation_world_wxyz": np.asarray(orientation).tolist(),
                                        "linear_velocity_world_m_s": np.asarray(robot.get_linear_velocity()).tolist()})
        phase = self.phase
        self._capture_scan(t, phase, robot)
        if self.phase == "settle":
            if t >= self.config["settle_time_s"]:
                self.target_index = 0
                self.current_target_name = self._target_name(self.config["static_target_distances_m"][0])
                self._add_target(self.config["static_target_distances_m"][0], self.current_target_name)
                self.phase = "static"
                self.phase_start = t
                self.target_scans = len(self.lidar_samples)
            return 0.0
        if self.phase == "static":
            dists = self.config["static_target_distances_m"]
            count = len(self.lidar_samples) - self.target_scans
            if count >= self.config["capture_full_scans_per_target"]:
                if self.target_index == 2 and not self.selfscan_started:
                    self.selfscan_started = True
                    self.selfscan_summary["status"] = "visible_mesh versus hidden_mesh comparison at fixed 1m target; visual meshes only are toggled, physics/contact remains unchanged"
                    self._set_robot_visuals_hidden(True)
                    self.phase = "selfscan_hidden"
                    self.target_scans = len(self.lidar_samples)
                    return 0.0
                self.target_index += 1
                if self.target_index >= len(dists):
                    self.phase = "motion"
                    self.motion_started = t
                    self.phase_start = t
                    return float(self.config["motion_speed_m_s"])
                old = self.stage.GetPrimAtPath("/World/Stage4LidarTargets/target_" + self.current_target_name)
                if old and old.IsValid(): old.SetActive(False)
                label = self._target_name(dists[self.target_index])
                self.current_target_name = label
                self._add_target(dists[self.target_index], label)
                self.phase_start = t
                self.target_scans = len(self.lidar_samples)
            elif t - self.phase_start >= self.config["settle_time_s"] and count == 0:
                # Keep the failed target interval visible in metadata; move on to avoid a hung run.
                self.lidar_samples.append({"phase": "static", "expected_distance_m": dists[self.target_index], "failure": "no complete scan during target interval"})
                self.target_index += 1
                if self.target_index >= len(dists):
                    self.phase = "motion"; self.motion_started = t; self.phase_start = t
                    return float(self.config["motion_speed_m_s"])
                old = self.stage.GetPrimAtPath("/World/Stage4LidarTargets/target_" + self.current_target_name)
                if old and old.IsValid(): old.SetActive(False)
                d = dists[self.target_index]
                self.current_target_name = self._target_name(d)
                self._add_target(d, self.current_target_name)
                self.phase_start = t; self.target_scans = len(self.lidar_samples)
            return 0.0
        if self.phase == "selfscan_hidden":
            if len(self.lidar_samples) - self.target_scans >= 2:
                self._set_robot_visuals_hidden(False)
                self.phase = "selfscan_visible"
                self.target_scans = len(self.lidar_samples)
            return 0.0
        if self.phase == "selfscan_visible":
            if len(self.lidar_samples) - self.target_scans >= 2:
                self.selfscan_summary["comparison_complete"] = True
                self.target_index = 3
                self.current_target_name = self._target_name(self.config["static_target_distances_m"][self.target_index])
                old = self.stage.GetPrimAtPath("/World/Stage4LidarTargets/target_" + self._target_name(1.0))
                if old and old.IsValid(): old.SetActive(False)
                self._add_target(self.config["static_target_distances_m"][self.target_index], self.current_target_name)
                self.phase = "static"
                self.phase_start = t
                self.target_scans = len(self.lidar_samples)
            return 0.0
        if self.phase == "motion":
            if t - self.motion_started >= self.config["motion_duration_s"]:
                self.phase = "done"
                self.done = True
                self._write_results()
                return 0.0
            return float(self.config["motion_speed_m_s"])
        return 0.0

    def _write_results(self):
        (self.out / "raw").mkdir(exist_ok=True)
        starts = np.asarray([float(x) for x in self.frame_times], dtype=np.float64)
        intervals = np.diff(starts)
        imu_t = np.asarray([s.get("time") for s in self.imu_samples if isinstance(s.get("time"), (float, int))])
        imu_steps = np.asarray([s.get("physics_step") for s in self.imu_samples if isinstance(s.get("physics_step"), (float, int))])
        motion_positions = np.asarray([s["position_world_m"] for s in self.motion_samples], dtype=float)
        motion_velocities = np.asarray([s["linear_velocity_world_m_s"] for s in self.motion_samples], dtype=float)
        (self.out / "imu_samples.json").write_text(json.dumps(self.imu_samples, indent=2))
        (self.out / "motion_base_samples.json").write_text(json.dumps(self.motion_samples, indent=2))
        summary = {
            "status": "capture complete; metrics are recomputed offline from preserved NPZ files",
            "simulation_frame_count": self.frame_count,
            "render_rate_hz_observed": float(1.0 / np.median(intervals)) if len(intervals) else None,
            "render_frame_dt_median_s": float(np.median(intervals)) if len(intervals) else None,
            "render_frame_dt_p95_s": float(np.percentile(intervals, 95)) if len(intervals) else None,
            "scan_output_count": len([s for s in self.lidar_samples if "point_count" in s]),
            "scans": self.lidar_samples,
            "imu_samples_count": len(self.imu_samples),
            "imu_reported_time_range_s": [float(imu_t.min()), float(imu_t.max())] if len(imu_t) else None,
            "imu_sample_dt_median_s": float(np.median(np.diff(imu_t))) if len(imu_t) > 1 else None,
            "imu_physics_step_rate_hz": float(np.median(np.diff(imu_steps)) / np.median(np.diff(imu_t))) if len(imu_steps) > 1 and np.median(np.diff(imu_t)) > 0 else None,
            "imu_invalid_count": sum(not s.get("is_valid", False) for s in self.imu_samples),
            "imu_first": self.imu_samples[0] if self.imu_samples else None,
            "imu_last": self.imu_samples[-1] if self.imu_samples else None,
            "imu_model": "Isaac physics IMUSensor, ideal/noise-free; no Ouster hardware noise, calibration, clock error or firmware model",
            "imu_units_and_axes": "linear_acceleration in stage length units/s^2 (m/s^2 on this meter stage), angular_velocity in rad/s physics convention, orientation [w,x,y,z] unit quaternion; acceleration/gyro are reported on sensor axes and attitude is world orientation; no noise model",
            "motion": {"requested_command_m_s": self.config["motion_speed_m_s"], "duration_s": self.config["motion_duration_s"],
                       "sample_count": int(len(self.motion_samples)),
                       "base_displacement_world_m": (motion_positions[-1] - motion_positions[0]).tolist() if len(motion_positions) > 1 else None,
                       "median_base_velocity_world_m_s": np.median(motion_velocities, axis=0).tolist() if len(motion_velocities) else None},
            "sensor_appearance_disabled": self.appearance_disabled_for_baseline,
            "self_scan_test": self.selfscan_summary,
            "ros_pointcloud2": "not emitted in this stage: fields/frame semantics are only adapted after raw output validation",
            "laserscan": "not emitted: this is 3D 64-emitter point data, no selected 2D slice",
            "gui_screenshot": "not tested by headless run",
        }
        (self.out / "capture_summary.json").write_text(json.dumps(summary, indent=2))
        rgb = self.camera.get_rgb()
        if rgb is not None and np.asarray(rgb).size:
            Image.fromarray(np.asarray(rgb, dtype=np.uint8)).save(self.out / "public_rgb_after_motion.png")

    def record_pointcloud2_message(self, msg):
        (self.out / "pointcloud2_sample.bin").write_bytes(bytes(msg.data))
        (self.out / "pointcloud2_sample.json").write_text(json.dumps({
            "topic": "/go2w/lidar/points", "frame_id": msg.header.frame_id,
            "stamp": {"sec": int(msg.header.stamp.sec), "nanosec": int(msg.header.stamp.nanosec)},
            "height": int(msg.height), "width": int(msg.width), "point_step": int(msg.point_step),
            "row_step": int(msg.row_step), "is_bigendian": bool(msg.is_bigendian), "is_dense": bool(msg.is_dense),
            "fields": [{"name": f.name, "offset": int(f.offset), "datatype": int(f.datatype), "count": int(f.count)} for f in msg.fields],
            "timestamp_conversion": "per-point uint64 nanoseconds converted to float64 seconds from earliest return in this scan; raw uint64 retained in NPZ",
            "emitter_id_semantics": "raw emitterId uint32; not a calibrated ring/channel index",
            "omitted_fields": ["intensity", "ring", "channelId"],
            "filtered_records": "nonfinite XYZ and all-zero XYZ records excluded from PointCloud2; raw NPZ remains unchanged",
        }, indent=2))

    def finalize_ros_summary(self):
        path = self.out / "capture_summary.json"
        summary = json.loads(path.read_text())
        summary["ros_pointcloud2"] = {
            "enabled": self.ros_pointcloud_enabled, "topic": "/go2w/lidar/points" if self.ros_pointcloud_enabled else None,
            "published_messages": self.ros_pointcloud_published,
            "sample_message": "pointcloud2_sample.json" if self.pc2_sample_saved else None,
            "field_schema": ["x FLOAT32", "y FLOAT32", "z FLOAT32", "time_from_scan_start FLOAT64 seconds", "emitter_id UINT32"],
            "ring_or_intensity": "not present; not fabricated",
        }
        path.write_text(json.dumps(summary, indent=2))

    def finish(self):
        if not self.done:
            (self.out / "failure.json").write_text(json.dumps({"phase": self.phase, "last_time_s": self.frame_times[-1] if self.frame_times else None,
                                                                  "partial_scans": self.lidar_samples}, indent=2))
            raise RuntimeError("Stage 4 capture stopped before its scheduled completion")
