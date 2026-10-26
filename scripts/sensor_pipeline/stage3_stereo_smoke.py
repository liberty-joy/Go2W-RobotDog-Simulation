"""Render synchronized Stage 3 stereo pairs and a separate analytic evaluation truth."""
import json
import math
from pathlib import Path

import numpy as np
from PIL import Image


class Stage3StereoSmoke:
    def __init__(self, config_path, output_path, root, stage, base_path, public_camera):
        from isaacsim.sensors.camera import Camera
        from pxr import Gf, UsdGeom, Vt

        self.Gf, self.UsdGeom, self.Vt = Gf, UsdGeom, Vt
        self.config_path = Path(config_path).resolve()
        self.config = json.loads(self.config_path.read_text())
        self.out = Path(output_path).resolve()
        self.out.mkdir(parents=True, exist_ok=True)
        protected = ["calibration.json", "captures.json", "captures", "ground_truth", "camera_info_left.json"]
        existing = [name for name in protected if (self.out / name).exists()]
        if existing:
            raise FileExistsError(f"refusing to overwrite existing Stage 3 outputs: {existing}")
        self.stage, self.base_path, self.public = stage, base_path, public_camera
        s, cap = self.config["stereo"], self.config["capture"]
        left_t = np.asarray(s["left_translation_base_m"], dtype=float)
        right_t = np.asarray(s["right_translation_base_m"], dtype=float)
        baseline = float(np.linalg.norm(left_t - right_t))
        if not np.isclose(baseline, s["baseline_m"], atol=1e-9):
            raise ValueError(f"left/right translations imply {baseline} m, configured {s['baseline_m']} m")
        if not np.allclose((left_t + right_t) / 2, s["cyclopean_translation_base_m"], atol=1e-9):
            raise ValueError("stereo midpoint must equal the existing public RGB camera center")
        if s["width"] != 640 or s["height"] != 480:
            raise ValueError("Stage 3 acceptance baseline is fixed at 640x480")
        expected_K = self._camera_matrix(s)
        self.left = Camera(prim_path=base_path + "/sensor_rig/Stage3Left", resolution=(s["width"], s["height"]),
                           frequency=s["frequency_hz"], translation=left_t)
        self.right = Camera(prim_path=base_path + "/sensor_rig/Stage3Right", resolution=(s["width"], s["height"]),
                            frequency=s["frequency_hz"], translation=right_t)
        pitch = math.radians(s["pitch_down_deg"]) / 2
        orient = np.array([math.cos(pitch), 0.0, math.sin(pitch), 0.0])
        for camera, translation in ((self.left, left_t), (self.right, right_t)):
            camera.initialize()
            camera.set_local_pose(translation=translation, orientation=orient, camera_axes="world")
            camera.set_focal_length(s["focal_length_m"])
            camera.set_horizontal_aperture(2 * s["focal_length_m"] * math.tan(math.radians(s["horizontal_fov_deg"] / 2)))
            camera.set_clipping_range(0.05, 100.0)
        # Isaac's geometric distance-to-image-plane annotator is a separate evaluation stream.
        # It is saved under ground_truth/ and is never passed into StereoSGBM.
        self.left.attach_annotator("distance_to_image_plane")
        self.camera_intrinsics = {}
        for name, sensor in (("public_rgb", public_camera), ("left", self.left), ("right", self.right)):
            actual_K = np.asarray(sensor.get_intrinsics_matrix(device="cpu"), dtype=np.float64)
            if not np.allclose(actual_K, expected_K, atol=1e-4):
                raise RuntimeError(f"{name} CameraInfo/intrinsics mismatch: expected {expected_K.tolist()}, got {actual_K.tolist()}")
            self.camera_intrinsics[name] = actual_K.tolist()
        self.K = np.asarray(self.camera_intrinsics["left"], dtype=np.float64)
        self.disabled_demo_obstacles = []
        for obstacle_name in ("RedBox", "BlueBox"):
            obstacle = stage.GetPrimAtPath("/World/" + obstacle_name)
            if obstacle and obstacle.IsActive():
                obstacle.SetActive(False)
                self.disabled_demo_obstacles.append("/World/" + obstacle_name)
        # Keep the legacy FrontCamera's exact pose and projection; only its cadence changes to the paired capture rate.
        self.targets = stage.DefinePrim(base_path + "/Stage3Targets", "Xform")
        self.scenarios = []
        for distance in cap["distances_optical_z_m"]:
            split = "development" if distance in cap["development_distances_m"] else "heldout_test"
            for scene_class in cap["scene_classes"]:
                self.scenarios.append({"scenario_id": f"{split}_{distance:g}m_{scene_class}",
                                       "split": split, "distance_m": float(distance), "scene_class": scene_class})
        self.index = 0
        self.wait_frames = 0
        self.started = False
        self.done = False
        self.last_status = {}
        self.rows = []
        self._write_metadata()

    @staticmethod
    def _camera_matrix(stereo):
        width, height = stereo["width"], stereo["height"]
        fov = math.radians(stereo["horizontal_fov_deg"])
        fx = width / (2 * math.tan(fov / 2))
        fy = fx
        # Isaac Camera.get_intrinsics_matrix uses image center at width/2,height/2.
        return np.array([[fx, 0, width / 2], [0, fy, height / 2], [0, 0, 1]], dtype=np.float64)

    def _write_metadata(self):
        s = self.config["stereo"]
        fx, fy = float(self.K[0, 0]), float(self.K[1, 1])
        p = {
            "schema_version": 1,
            "resolution": [s["width"], s["height"]],
            "distortion_model": "none",
            "D": s["distortion_coefficients"],
            "K": self.K.tolist(),
            "R": np.eye(3).tolist(),
            "P": [[fx, 0, self.K[0, 2], 0], [0, fy, self.K[1, 2], 0], [0, 0, 1, 0]],
            "frame_id": s["camera_info_frame_ids"]["public_rgb"],
            "source": "Isaac Camera.get_intrinsics_matrix checked against ideal configuration; not measured physical calibration",
            "runtime_camera_intrinsics": self.camera_intrinsics,
        }
        for name, xyz in (("public_rgb", s["cyclopean_translation_base_m"]), ("left", s["left_translation_base_m"]), ("right", s["right_translation_base_m"])):
            entry = dict(p)
            entry["frame_id"] = s["camera_info_frame_ids"][name]
            entry["translation_base_m"] = xyz
            entry["baseline_m"] = s["baseline_m"] if name == "right" else 0.0
            if name == "right":
                entry["P"] = [[fx, 0, self.K[0, 2], -fx * s["baseline_m"]],
                               [0, fy, self.K[1, 2], 0], [0, 0, 1, 0]]
            (self.out / f"camera_info_{name}.json").write_text(json.dumps(entry, indent=2))
        (self.out / "calibration.json").write_text(json.dumps({
            "model": "parallel ideal pinhole pair", "resolution": [s["width"], s["height"]],
            "K_left": self.K.tolist(), "K_right": self.K.tolist(), "D_left": s["distortion_coefficients"],
            "D_right": s["distortion_coefficients"], "R_left_to_right": np.eye(3).tolist(),
            "T_left_to_right_m": [-s["baseline_m"], 0, 0], "baseline_m": s["baseline_m"],
            "P_left": p["P"],
            "P_right": [[fx, 0, self.K[0, 2], -fx * s["baseline_m"]],
                        [0, fy, self.K[1, 2], 0], [0, 0, 1, 0]],
            "opencv_convention": "x_right = R*x_left + T; left camera center is physically +Y; expected left-minus-right image disparity is positive",
            "distortion": "zero by ideal simulation configuration; no physical lens calibration is claimed",
            "public_rgb_center": s["cyclopean_translation_base_m"],
            "left_translation_base_m": s["left_translation_base_m"],
            "right_translation_base_m": s["right_translation_base_m"], "K": self.K.tolist()
        }, indent=2))
        capture_config = dict(self.config)
        capture_config["capture_runtime"] = {
            "disabled_demo_obstacles": self.disabled_demo_obstacles,
            "geometric_depth_annotator": "Isaac distance_to_image_plane; evaluation-only",
            "stereo_inputs": ["left_rgb.png", "right_rgb.png"],
        }
        (self.out / "capture_config.json").write_text(json.dumps(capture_config, indent=2))

    def _author_scene(self, scenario, robot):
        from pxr import Gf, UsdGeom, Vt
        d = scenario["distance_m"]
        cap = self.config["capture"]
        root_path = str(self.targets.GetPath()) + "/" + scenario["scenario_id"]
        root = self.stage.DefinePrim(root_path, "Xform")
        distance = float(d)
        center = self.config["stereo"]["cyclopean_translation_base_m"]
        pitch = math.radians(self.config["stereo"]["pitch_down_deg"])
        forward = np.array([math.cos(pitch), 0, -math.sin(pitch)])
        plane_center = np.asarray(center, dtype=float) + distance * forward
        width = max(cap["plane_min_extent_m"], distance * cap["plane_width_factor"])
        height = max(cap["plane_min_extent_m"], distance * cap["plane_height_factor"])
        nx, ny = 48, 36
        points = []
        for j in range(ny + 1):
            z = -height / 2 + height * j / ny
            for i in range(nx + 1):
                y = -width / 2 + width * i / nx
                points.append(Gf.Vec3f(0, y, z))
        counts, indices, colors = [], [], []
        rng = np.random.default_rng(int(cap["texture_seed"]) + int(d * 100) + sum(map(ord, scenario["scene_class"])))
        base = rng.uniform(0.12, 0.92, size=(ny, nx, 3))
        for j in range(ny):
            for i in range(nx):
                a = j * (nx + 1) + i
                b, c, e = a + 1, a + nx + 1, a + nx + 2
                # Two triangles per texel; use the same flat color on both faces.
                if scenario["scene_class"] == "textureless":
                    col = np.array([0.58, 0.60, 0.62])
                elif scenario["scene_class"] == "repetitive":
                    col = np.array([0.82, 0.82, 0.82]) if ((i % 4) + (j % 4)) % 2 else np.array([0.16, 0.16, 0.16])
                else:
                    col = base[j, i]
                for tri in ((a, e, c), (a, b, e)):
                    counts.append(3); indices.extend(tri); colors.append(Gf.Vec3f(*map(float, col)))
        mesh_path = root_path + "/TargetPlane"
        mesh = UsdGeom.Mesh.Define(self.stage, mesh_path)
        mesh.CreatePointsAttr(Vt.Vec3fArray(points))
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray(counts))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreateDisplayColorPrimvar(UsdGeom.Tokens.uniform).Set(Vt.Vec3fArray(colors))
        mesh_xf = UsdGeom.XformCommonAPI(mesh.GetPrim())
        mesh_xf.SetTranslate(Gf.Vec3d(*map(float, plane_center)))
        mesh_xf.SetRotate(Gf.Vec3f(0, self.config["stereo"]["pitch_down_deg"], 0))
        if scenario["scene_class"] == "occlusion":
            cube_path = root_path + "/ForegroundBox"
            cube = UsdGeom.Cube.Define(self.stage, cube_path)
            cube.CreateSizeAttr(cap["box_size_m"])
            cube.CreateDisplayColorAttr([Gf.Vec3f(0.85, 0.12, 0.08)])
            offset = cap["occluder_center_offset_from_plane_m"]
            box_center = plane_center - forward * offset
            cube_xf = UsdGeom.XformCommonAPI(cube.GetPrim())
            cube_xf.SetTranslate(Gf.Vec3d(*map(float, box_center)))
            cube_xf.SetRotate(Gf.Vec3f(0, self.config["stereo"]["pitch_down_deg"], 0))
        return root, plane_center

    def _set_scene_visible(self, scenario, visible):
        from pxr import UsdGeom
        prim = self.stage.GetPrimAtPath(str(self.targets.GetPath()) + "/" + scenario["scenario_id"])
        if prim:
            UsdGeom.Imageable(prim).GetVisibilityAttr().Set(UsdGeom.Tokens.inherited if visible else UsdGeom.Tokens.invisible)

    @staticmethod
    def _rgb(camera):
        image = camera.get_rgb()
        if image is None or not image.size:
            return None
        image = np.asarray(image)
        return image[:, :, :3].astype(np.uint8, copy=False)

    @staticmethod
    def _time(camera):
        frame = camera.get_current_frame()
        return frame.get("rendering_time"), frame.get("simulation_time")

    def _ground_truth(self, scenario, geometric_depth):
        h, w = self.config["stereo"]["height"], self.config["stereo"]["width"]
        d = scenario["distance_m"]
        truth = np.asarray(geometric_depth, dtype=np.float32).squeeze()
        if truth.shape != (h, w):
            raise RuntimeError(f"geometric depth shape {truth.shape} != {(h, w)}")
        truth[(~np.isfinite(truth)) | (truth <= 0)] = np.nan
        # Foreground depth nearer than the known plane marks geometric occlusion;
        # this also correctly accounts for the floor clipping the plane at close range.
        geom_occluded = ((np.isfinite(truth)) & (truth < d - 0.02)).astype(np.uint8)
        truth_path = self.out / "ground_truth" / scenario["scenario_id"]
        truth_path.mkdir(parents=True, exist_ok=True)
        np.save(truth_path / "left_optical_z_m.npy", truth)
        np.save(truth_path / "geometric_occlusion_mask.npy", geom_occluded)
        return truth_path

    def update(self, simulation_time, robot):
        if self.done:
            return
        if self.index >= len(self.scenarios):
            self.done = True
            return
        scenario = self.scenarios[self.index]
        if not self.started:
            self._author_scene(scenario, robot)
            self.started = True
            self.wait_frames = 0
            return
        self.wait_frames += 1
        if self.wait_frames < self.config["capture"]["warmup_frames_per_scene"]:
            return
        left, right, public = self._rgb(self.left), self._rgb(self.right), self._rgb(self.public)
        lt, ls = self._time(self.left)
        rt, rs = self._time(self.right)
        pt, ps = self._time(self.public)
        self.last_status = {"scenario_id": scenario["scenario_id"], "left_rendering_time": lt,
                            "right_rendering_time": rt, "public_rendering_time": pt,
                            "left_shape": None if left is None else list(left.shape),
                            "right_shape": None if right is None else list(right.shape),
                            "public_shape": None if public is None else list(public.shape),
                            "pair_wait_frames": self.wait_frames}
        times = [lt, rt, pt]
        same_time = all(t is not None for t in times) and max(float(t) for t in times) - min(float(t) for t in times) <= self.config["capture"]["timestamp_tolerance_s"]
        good_shapes = all(im is not None and im.shape == (self.config["stereo"]["height"], self.config["stereo"]["width"], 3)
                          for im in (left, right, public))
        if not same_time or not good_shapes:
            if self.wait_frames <= self.config["capture"]["max_pair_wait_frames"]:
                return
            self._write_failure(scenario, "no synchronized three-view frame with expected shape")
            raise RuntimeError(f"Stage 3 frame synchronization/shape failed: {self.last_status}")
        scenario_dir = self.out / "captures" / scenario["scenario_id"]
        scenario_dir.mkdir(parents=True, exist_ok=False)
        Image.fromarray(left).save(scenario_dir / "left_rgb.png")
        Image.fromarray(right).save(scenario_dir / "right_rgb.png")
        Image.fromarray(public).save(scenario_dir / "public_rgb_common_semantics.png")
        gt_depth = self.left.get_depth(device="cpu")
        if gt_depth is None:
            self._write_failure(scenario, "evaluation-only geometric depth annotator returned no data")
            raise RuntimeError(f"missing evaluation-only geometric depth for {scenario['scenario_id']}")
        truth_path = self._ground_truth(scenario, gt_depth)
        row = {**scenario, "simulation_time_s": float(simulation_time),
               "left_rendering_time_s": float(lt), "right_rendering_time_s": float(rt),
               "public_rendering_time_s": float(pt), "left_simulation_time_s": ls,
               "right_simulation_time_s": rs, "public_simulation_time_s": ps,
               "timestamp_delta_s": max(map(float, times)) - min(map(float, times)),
               "left_shape": list(left.shape), "right_shape": list(right.shape), "public_shape": list(public.shape),
               "baseline_m": self.config["stereo"]["baseline_m"], "truth_path": str(truth_path.relative_to(self.out)),
               "pair_status": "PASS", "ground_truth_source": "Isaac distance_to_image_plane geometric annotator; evaluation-only separate directory"}
        (scenario_dir / "capture.json").write_text(json.dumps(row, indent=2))
        self.rows.append(row)
        self._set_scene_visible(scenario, False)
        self.index += 1
        self.started = False
        self.wait_frames = 0
        if self.index == len(self.scenarios):
            self.done = True
            (self.out / "captures.json").write_text(json.dumps({"status": "CAPTURE_PASS", "rows": self.rows}, indent=2))
            print(f"STAGE3_STEREO_CAPTURE_PASS {len(self.rows)} synchronized scenarios", flush=True)
        else:
            print(f"STAGE3_CAPTURED {scenario['scenario_id']} ({self.index}/{len(self.scenarios)})", flush=True)

    def _write_failure(self, scenario, reason):
        path = self.out / "failures.jsonl"
        with path.open("a") as f:
            f.write(json.dumps({"scenario": scenario, "reason": reason, "status": self.last_status}) + "\n")

    def finish(self):
        if not self.done:
            self._write_failure(self.scenarios[self.index], "simulation exited before all scenarios were captured")
            raise RuntimeError(f"Stage 3 capture incomplete: {self.index}/{len(self.scenarios)}; see failures.jsonl")
