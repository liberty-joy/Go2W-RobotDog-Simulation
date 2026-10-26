"""Stage 2 local sensor-rig import and measurement smoke for Isaac Sim 6.2."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


class AssetRigSmoke:
    def __init__(self, config, out, root, stage, base_path):
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.sensors.rtx")
        enable_extension("isaacsim.sensors.experimental.rtx")
        enable_extension("isaacsim.sensors.experimental.physics")

        from isaacsim.sensors.camera import Camera
        from isaacsim.sensors.experimental.physics import IMU, IMUSensor
        from isaacsim.sensors.experimental.rtx import Lidar
        from isaacsim.sensors.rtx import LidarRtx
        from isaacsim.core.utils.viewports import set_camera_view

        self.config_path = Path(config).resolve()
        self.config = json.loads(self.config_path.read_text())
        self.out = Path(out).resolve()
        self.out.mkdir(parents=True, exist_ok=False)
        self.stage = stage
        self.base_path = base_path
        self.rig_path = base_path + "/sensor_rig"
        self.done = False
        self.last_status = {}

        rig_file = (Path(root) / self.config["rig_usd"]).resolve()
        rig = stage.DefinePrim(self.rig_path, "Xform")
        rig.GetReferences().AddReference(str(rig_file))

        stereo = self.config["stereo"]
        left_t = np.asarray(stereo["left_translation_base_m"], dtype=float)
        right_t = np.asarray(stereo["right_translation_base_m"], dtype=float)
        actual_baseline = float(np.linalg.norm(left_t - right_t))
        if not np.isclose(actual_baseline, stereo["baseline_m"], atol=1e-9):
            raise ValueError(f"Configured translations give baseline {actual_baseline}, expected {stereo['baseline_m']}")

        measurement = self.rig_path + "/Measurement"
        pitch = math.radians(stereo["pitch_down_deg"]) / 2
        orientation = np.array([math.cos(pitch), 0, math.sin(pitch), 0])
        self.left = Camera(
            prim_path=measurement + "/left_camera",
            resolution=(stereo["width"], stereo["height"]),
            frequency=stereo["frequency_hz"],
            translation=left_t,
        )
        self.right = Camera(
            prim_path=measurement + "/right_camera",
            resolution=(stereo["width"], stereo["height"]),
            frequency=stereo["frequency_hz"],
            translation=right_t,
        )
        for camera, translation in [(self.left, left_t), (self.right, right_t)]:
            camera.initialize()
            camera.set_local_pose(translation=translation, orientation=orientation, camera_axes="world")
            camera.set_focal_length(0.018)
            camera.set_horizontal_aperture(
                2 * 0.018 * math.tan(math.radians(stereo["horizontal_fov_deg"] / 2))
            )
            camera.set_clipping_range(0.02, 100)

        capture = self.config["capture"]
        self.overview = Camera(
            prim_path="/World/Stage2AssemblyCamera",
            resolution=(960, 600),
            frequency=stereo["frequency_hz"],
        )
        self.overview.initialize()
        set_camera_view(
            eye=capture["overview_eye_world_m"],
            target=capture["overview_target_world_m"],
            camera_prim_path="/World/Stage2AssemblyCamera",
        )

        lidar_cfg = self.config["lidar"]
        profile = json.loads((Path(root) / lidar_cfg["profile"]).read_text())["profile"]
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
        emitter = profile["emitterStates"][0]
        for key, value in emitter.items():
            attrs["omni:sensor:Core:emitterState:s001:" + key] = value
        attrs["omni:sensor:Core:emitterState:s001:channelId"] = list(
            range(1, profile["numberOfEmitters"] + 1)
        )
        attrs["omni:sensor:Core:emitterState:s001:bank"] = []
        lidar_path = measurement + "/lidar"
        Lidar.create(
            lidar_path,
            tick_rate=profile["scanRateBaseHz"],
            aux_output_level="FULL",
            translations=[lidar_cfg["translation_base_m"]],
            attributes={key: value for key, value in attrs.items() if not isinstance(value, list)},
        )
        lidar_prim = stage.GetPrimAtPath(lidar_path)
        for key, value in attrs.items():
            if isinstance(value, list):
                attribute = lidar_prim.GetAttribute(key)
                if not attribute.IsValid():
                    raise RuntimeError(f"Missing authored lidar attribute: {key}")
                attribute.Set(value)
        self.lidar = LidarRtx(prim_path=lidar_path)
        self.lidar.attach_annotator(
            "IsaacCreateRTXLidarScanBuffer", outputTimestamp=True, outputEmitterId=True
        )
        self.lidar.initialize()

        imu_path = measurement + "/imu"
        self.imu = IMUSensor(IMU.create(imu_path, translations=[self.config["imu"]["translation_base_m"]]))
        self.profile = profile
        self.authored_lidar_attributes = attrs

    def update(self, simulation_time):
        if self.done or simulation_time < self.config["capture"]["sample_after_sim_s"]:
            return
        left = self.left.get_rgb()
        right = self.right.get_rgb()
        frame = self.lidar.get_current_frame()
        scan = frame.get("IsaacCreateRTXLidarScanBuffer", {})
        xyz = scan.get("data", np.empty((0, 3))) if isinstance(scan, dict) else np.empty((0, 3))
        reading = self.imu.get_data()
        imu_valid = bool(self.imu.get_sensor_reading().is_valid)
        self.last_status = {
            "left": None if left is None else list(left.shape),
            "right": None if right is None else list(right.shape),
            "points": int(len(xyz)),
            "imu_valid": imu_valid,
        }
        if left is None or right is None or not left.size or not right.size or not len(xyz) or not imu_valid:
            return

        Image.fromarray(np.asarray(left, dtype=np.uint8)).save(self.out / "left_rgb.png")
        Image.fromarray(np.asarray(right, dtype=np.uint8)).save(self.out / "right_rgb.png")
        overview = self.overview.get_rgb()
        if overview is not None and overview.size:
            Image.fromarray(np.asarray(overview, dtype=np.uint8)).save(self.out / "assembly_render.png")
        np.save(self.out / "lidar_xyz.npy", xyz)
        arrays = {}
        for name, value in scan.items():
            if isinstance(value, np.ndarray):
                arrays[name] = {"shape": list(value.shape), "dtype": str(value.dtype)}
                if name in {"timestamp", "emitterId"}:
                    np.save(self.out / f"lidar_{name}.npy", value)

        from pxr import Usd, UsdGeom, UsdPhysics

        collision_prims = []
        for prim in self.stage.Traverse():
            if str(prim.GetPath()).startswith(self.rig_path) and prim.HasAPI(UsdPhysics.CollisionAPI):
                collision_prims.append(str(prim.GetPath()))
        bound = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
        ).ComputeWorldBound(self.stage.GetPrimAtPath(self.rig_path)).ComputeAlignedRange()
        appearance_prims = [
            str(prim.GetPath())
            for prim in self.stage.Traverse()
            if str(prim.GetPath()).startswith(self.rig_path + "/Appearance/")
        ]
        distances = np.linalg.norm(xyz, axis=1)
        result = {
            "simulation_time": simulation_time,
            "rig_prim": self.rig_path,
            "rig_usd": self.config["rig_usd"],
            "left_shape": list(left.shape),
            "right_shape": list(right.shape),
            "left_rendering_time": self.left.get_current_frame().get("rendering_time"),
            "right_rendering_time": self.right.get_current_frame().get("rendering_time"),
            "point_count": int(len(xyz)),
            "point_distance_min_m": float(np.min(distances)),
            "points_inside_self_scan_guard": int(np.count_nonzero(distances < self.config["lidar"]["self_scan_guard_m"])),
            "lidar_fields": arrays,
            "imu_valid": imu_valid,
            "imu": {key: np.asarray(value).tolist() for key, value in reading.items()},
            "collision_prims_under_rig": collision_prims,
            "appearance_prim_count": len(appearance_prims),
            "rig_world_bounds_m": {
                "min": list(bound.GetMin()),
                "max": list(bound.GetMax()),
            },
            "assembly_render_saved": bool(overview is not None and overview.size),
            "gui_screenshot": "NOT_TESTED; assembly_render.png is a sensor-independent stage render, not a GUI screenshot",
            "vendor_measurement_used": False,
            "stereo_depth_computed": False,
            "scope": self.config["capture"]["scope"],
            "config": self.config,
        }
        (self.out / "smoke.json").write_text(json.dumps(result, indent=2))
        self.done = True
        print("STAGE2_SENSOR_RIG_PASS", len(xyz), flush=True)

    def finish(self):
        if not self.done:
            (self.out / "failure.json").write_text(json.dumps(self.last_status, indent=2))
            raise RuntimeError("Stage 2 sensor rig produced no complete sample")
