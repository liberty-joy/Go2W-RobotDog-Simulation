"""One-scene RGB/stereo/rotary-LiDAR capture with explicit simulation timestamps."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image

from sensor_pipeline.stage4_lidar_smoke import Stage4LidarSmoke


class Stage5FusionCapture:
    PHASES = ("static", "straight", "turn", "parked")

    def __init__(self, config_path, output_path, root, stage, base_path, public_camera):
        from isaacsim.core.utils.extensions import enable_extension
        enable_extension("isaacsim.sensors.rtx")
        enable_extension("isaacsim.sensors.experimental.rtx")
        enable_extension("isaacsim.sensors.experimental.physics")
        from isaacsim.sensors.experimental.rtx import Lidar
        from isaacsim.sensors.rtx import LidarRtx
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, UsdGeom, Vt

        self.config_path = Path(config_path).resolve()
        self.config = json.loads(self.config_path.read_text())
        self.root = Path(root)
        self.out = Path(output_path).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        if any((self.out / name).exists() for name in ("captures", "capture_summary.json")):
            raise FileExistsError("Stage 5 output already contains capture data")
        (self.out / "captures").mkdir()
        self.stage, self.base_path, self.public = stage, base_path, public_camera
        self.rig_config = json.loads((self.root / self.config["rig_config"]).read_text())
        stereo = json.loads((self.root / self.config["stereo_config"]).read_text())["stereo"]
        profile = json.loads((self.root / self.config["lidar_profile"]).read_text())["profile"]
        self.stereo_config, self.profile = stereo, profile
        rig_path = base_path + "/sensor_rig"
        stage.DefinePrim(rig_path, "Xform").GetReferences().AddReference(str((self.root / self.rig_config["rig_usd"]).resolve()))
        appearance = stage.GetPrimAtPath(rig_path + "/Appearance")
        if appearance and appearance.IsActive(): appearance.SetActive(False)

        self.lidar_path = rig_path + "/Measurement/Stage5Lidar"
        attrs = Stage4LidarSmoke._make_lidar_attributes(profile)
        Lidar.create(self.lidar_path, tick_rate=float(profile["scanRateBaseHz"]),
                     aux_output_level="FULL", translations=[self.rig_config["lidar"]["translation_base_m"]],
                     attributes={k: v for k, v in attrs.items() if not isinstance(v, list)})
        prim = stage.GetPrimAtPath(self.lidar_path)
        for key, value in attrs.items():
            if isinstance(value, list): prim.GetAttribute(key).Set(value)
        self.lidar = LidarRtx(prim_path=self.lidar_path)
        self.lidar.attach_annotator("IsaacCreateRTXLidarScanBuffer", outputTimestamp=True, outputEmitterId=True)
        self.lidar.initialize()
        imu_path = rig_path + "/Measurement/Stage5Imu"
        self.imu = IMUSensor(IMU.create(imu_path, translations=[self.rig_config["imu"]["translation_base_m"]]))

        self.left = Camera(prim_path=rig_path + "/Stage5Left", resolution=(stereo["width"], stereo["height"]),
                           frequency=stereo["frequency_hz"], translation=np.asarray(stereo["left_translation_base_m"]))
        self.right = Camera(prim_path=rig_path + "/Stage5Right", resolution=(stereo["width"], stereo["height"]),
                            frequency=stereo["frequency_hz"], translation=np.asarray(stereo["right_translation_base_m"]))
        half_pitch = math.radians(stereo["pitch_down_deg"]) / 2
        orient = np.array([math.cos(half_pitch), 0.0, math.sin(half_pitch), 0.0])
        for cam, trans in ((self.left, stereo["left_translation_base_m"]), (self.right, stereo["right_translation_base_m"])):
            cam.initialize()
            cam.set_local_pose(translation=np.asarray(trans), orientation=orient, camera_axes="world")
            cam.set_focal_length(stereo["focal_length_m"])
            cam.set_horizontal_aperture(2 * stereo["focal_length_m"] * math.tan(math.radians(stereo["horizontal_fov_deg"]) / 2))
            cam.set_clipping_range(0.05, 100.0)
        self.left.attach_annotator("distance_to_image_plane")
        self.K = {}
        for name, cam in (("public", self.public), ("left", self.left), ("right", self.right)):
            self.K[name] = np.asarray(cam.get_intrinsics_matrix(device="cpu"), dtype=float).tolist()
        if not np.allclose(self.K["left"], self.K["right"], atol=1e-4):
            raise RuntimeError("Stage 5 stereo intrinsics differ")

        for name in ("RedBox", "BlueBox"):
            old = stage.GetPrimAtPath("/World/" + name)
            if old and old.IsActive(): old.SetActive(False)
        self._create_panel(Gf, UsdGeom, Vt)
        self.phase = "warmup"
        self.phase_start_s = 0.0
        self.captured = {}
        self.trajectory = []
        self.scan_clock = []
        self.imu_clock = []
        self.seen_scan = set()
        self.done = False
        self.results_written = False
        (self.out / "config_snapshot.json").write_text(json.dumps(self.config, indent=2))

    def _create_panel(self, Gf, UsdGeom, Vt):
        center = self.config["object_surface_center_world_m"]
        width, height = self.config["object_width_m"], self.config["object_height_m"]
        ny, nz = 24, 20
        points = [Gf.Vec3f(0, -width/2 + width*i/ny, -height/2 + height*j/nz)
                  for j in range(nz+1) for i in range(ny+1)]
        rng = np.random.default_rng(self.config["seed"])
        counts, indices, colors = [], [], []
        for j in range(nz):
            for i in range(ny):
                a = j*(ny+1)+i
                b, c, d = a+1, a+ny+1, a+ny+2
                red = float(rng.uniform(0.5, 0.95))
                green, blue = float(rng.uniform(0.025, 0.12)), float(rng.uniform(0.02, 0.10))
                for tri in ((a,d,c), (a,b,d)):
                    counts.append(3)
                    indices.extend(tri)
                    colors.append(Gf.Vec3f(red, green, blue))
        mesh = UsdGeom.Mesh.Define(self.stage, "/World/Stage5RedPanel")
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform).Set(Vt.Vec3fArray(colors))
        UsdGeom.XformCommonAPI(mesh.GetPrim()).SetTranslate(Gf.Vec3d(*map(float, center)))

    @staticmethod
    def _pose(sensor, camera=False):
        position, quaternion = sensor.get_world_pose(camera_axes="ros") if camera else sensor.get_world_pose()
        return {"position_world_m": np.asarray(position, dtype=float).tolist(),
                "orientation_world_wxyz": np.asarray(quaternion, dtype=float).tolist()}

    def command(self):
        c = self.config["command"]
        if self.phase == "straight": return float(c["straight_v_m_s"]), 0.0
        if self.phase == "turn": return float(c["turn_v_m_s"]), float(c["turn_yaw_rad_s"])
        return 0.0, 0.0

    def _new_scan(self):
        frame = self.lidar.get_current_frame()
        scan = frame.get("IsaacCreateRTXLidarScanBuffer", {})
        if not isinstance(scan, dict): return None
        xyz, timestamps, emitter = (np.asarray(scan.get(key, [])) for key in ("data", "timestamp", "emitterId"))
        if xyz.ndim != 2 or xyz.shape[1] != 3 or len(xyz) == 0:
            return None
        if len(timestamps) != len(xyz) or len(emitter) != len(xyz):
            raise RuntimeError("Stage 5 requires real per-point timestamp and emitterId")
        signature = (int(timestamps[0]), int(timestamps[-1]), int(len(xyz)))
        if signature in self.seen_scan: return None
        self.seen_scan.add(signature)
        return {"data": xyz, "timestamp": timestamps, "emitterId": emitter}, frame, scan

    def _capture(self, t, arrays, lidar_frame, scan):
        images = {}
        times = {}
        for name, cam in (("public", self.public), ("left", self.left), ("right", self.right)):
            rgb = cam.get_rgb()
            if rgb is None or np.asarray(rgb).shape != (480, 640, 3): return False
            images[name] = np.asarray(rgb, dtype=np.uint8)
            times[name] = cam.get_current_frame().get("rendering_time")
        if any(v is None for v in times.values()) or max(times.values())-min(times.values()) > 1e-9:
            return False
        exposure_s = float(times["public"])
        scan_end_s = int(np.max(arrays["timestamp"])) * 1e-9
        pair_delta_s = exposure_s - scan_end_s
        if abs(pair_delta_s) > self.config["pair_max_abs_delta_s"]:
            return False
        folder = self.out / "captures" / self.phase
        folder.mkdir(exist_ok=False)
        for name, image in images.items(): Image.fromarray(image).save(folder / f"{name}_rgb.png")
        np.savez_compressed(folder / "lidar_raw.npz", **arrays)
        truth = self.left.get_depth(device="cpu")
        if truth is not None and np.asarray(truth).shape[:2] == (480, 640):
            np.save(folder / "evaluation_only_left_geometric_z_m.npy", np.asarray(truth, dtype=np.float32))
        data = {"phase": self.phase, "simulation_time_s": float(t), "camera_exposure_render_time_s": exposure_s,
                "camera_times_s": times, "scan_output_rendering_time_s": lidar_frame.get("rendering_time"),
                "scan_start_s": int(np.min(arrays["timestamp"])) * 1e-9,
                "scan_end_s": scan_end_s, "camera_minus_scan_end_s": pair_delta_s,
                "scan_points": int(len(arrays["data"])), "lidar_pose_at_readout": self._pose(self.lidar),
                "public_pose_at_readout": self._pose(self.public, camera=True),
                "left_pose_at_readout": self._pose(self.left, camera=True),
                "right_pose_at_readout": self._pose(self.right, camera=True),
                "intrinsics": self.K,
                "raw_fields": {k: {"shape": list(np.asarray(v).shape), "dtype": str(np.asarray(v).dtype)}
                               for k, v in scan.items() if isinstance(v, np.ndarray)},
                "object_truth_surface_center_world_m": self.config["object_surface_center_world_m"],
                "truth_usage": "object position and evaluation-only depth are excluded from estimator inputs"}
        (folder / "capture.json").write_text(json.dumps(data, indent=2))
        self.captured[self.phase] = data
        print("STAGE5_CAPTURED", self.phase, "delta_s", pair_delta_s, "points", len(arrays["data"]), flush=True)
        return True

    def update(self, simulation_time, robot):
        if self.done: return
        t = float(simulation_time)
        self.trajectory.append({"time_s": t, "phase": self.phase, "base": self._pose(robot), "lidar": self._pose(self.lidar),
                                "public": self._pose(self.public, camera=True), "left": self._pose(self.left, camera=True),
                                "public_rendering_time_s": self.public.get_current_frame().get("rendering_time"),
                                "left_rendering_time_s": self.left.get_current_frame().get("rendering_time"),
                                "right_rendering_time_s": self.right.get_current_frame().get("rendering_time")})
        reading = self.imu.get_data()
        if reading:
            self.imu_clock.append({"time_s": float(reading.get("time", t)),
                                   "physics_step": float(reading.get("physics_step", 0)),
                                   "valid": bool(self.imu.get_sensor_reading().is_valid)})
        item = self._new_scan()
        if item is not None:
            arrays, frame, scan = item
            self.scan_clock.append({"scan_start_s": int(np.min(arrays["timestamp"]))*1e-9,
                                    "scan_end_s": int(np.max(arrays["timestamp"]))*1e-9,
                                    "output_rendering_time_s": frame.get("rendering_time"),
                                    "point_count": int(len(arrays["data"])), "phase": self.phase})
            if self.phase in self.PHASES and self.phase not in self.captured:
                if t-self.phase_start_s >= self.config["capture_after_phase_s"][self.phase]:
                    self._capture(t, arrays, frame, scan)
        if self.phase == "warmup":
            if t >= self.config["warmup_s"]:
                self.phase, self.phase_start_s = "static", t
            return
        duration = self.config["phase_duration_s"][self.phase]
        if t-self.phase_start_s < duration or self.phase not in self.captured: return
        idx = self.PHASES.index(self.phase)
        if idx + 1 < len(self.PHASES):
            self.phase, self.phase_start_s = self.PHASES[idx+1], t
        else:
            self.done = True
            self.finish()

    def finish(self):
        if self.results_written: return
        if not self.done or len(self.captured) != len(self.PHASES):
            (self.out / "failure.json").write_text(json.dumps({"phase": self.phase,
                "captured": list(self.captured), "scan_count": len(self.scan_clock)}, indent=2))
            raise RuntimeError("Stage 5 capture incomplete")
        (self.out / "trajectory.json").write_text(json.dumps(self.trajectory))
        (self.out / "scan_clock.json").write_text(json.dumps(self.scan_clock, indent=2))
        (self.out / "imu_clock.json").write_text(json.dumps(self.imu_clock))
        (self.out / "capture_summary.json").write_text(json.dumps({
            "status": "capture_complete", "phases": list(self.captured), "frames": len(self.trajectory),
            "full_scan_outputs": len(self.scan_clock), "imu_read_samples": len(self.imu_clock),
            "camera_model": "ideal zero-distortion global shutter; exposure time is camera rendering_time",
            "lidar_model": "Stage 1 generic rotary 64x512/10Hz functional proxy; real point timestamps retained",
            "motion_pose": "Isaac world truth saved at render cadence for explicit oracle deskew experiments",
            "operator_mask": "hand-authored red color threshold in config, not a trained detector",
            "gui": "not tested by headless capture"}, indent=2))
        self.results_written = True
        print("STAGE5_CAPTURE_PASS", len(self.captured), "phases", flush=True)
