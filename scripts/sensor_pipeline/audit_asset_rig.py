"""Offline Stage 2 provenance, dependency, assembly and extrinsic audit."""
import argparse
import hashlib
import json
import sys
from pathlib import Path

from pxr import Sdf, Usd, UsdGeom, UsdPhysics


parser = argparse.ArgumentParser()
parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
root = args.root.resolve()
output = args.output.resolve()
source_path = root / "sim/assets/sensors/SOURCE.json"
source = json.loads(source_path.read_text())

hash_checks = []
for item in source["resources"]:
    path = root / item["path"]
    actual = hashlib.sha256(path.read_bytes()).hexdigest() if path.is_file() else None
    hash_checks.append(
        {
            "path": item["path"],
            "exists": path.is_file(),
            "expected_sha256": item["sha256"],
            "actual_sha256": actual,
            "pass": actual == item["sha256"],
        }
    )

usd_dependency_checks = []
asset_attribute_checks = []
remote_dependencies = []
missing_dependencies = []
vendor_root = root / "sim/assets/sensors/vendor/nvidia_isaac_6_2"
rig_path = root / "sim/assets/sensors/assemblies/go2w_sensor_rig.usda"
asset_layers = sorted(vendor_root.rglob("*.usd*")) + [rig_path]
for path in asset_layers:
    layer = Sdf.Layer.FindOrOpen(str(path))
    references = [] if layer is None else list(layer.GetExternalReferences())
    resolved = []
    for reference in references:
        if "://" in reference or reference.startswith("omniverse:"):
            remote_dependencies.append({"layer": str(path.relative_to(root)), "reference": reference})
            resolved.append({"reference": reference, "kind": "remote", "exists": False})
            continue
        dependency = (path.parent / reference).resolve()
        exists = dependency.is_file()
        if not exists:
            missing_dependencies.append(
                {"layer": str(path.relative_to(root)), "reference": reference, "resolved": str(dependency)}
            )
        resolved.append(
            {
                "reference": reference,
                "kind": "local",
                "resolved": str(dependency.relative_to(root)) if dependency.is_relative_to(root) else str(dependency),
                "exists": exists,
            }
        )
    usd_dependency_checks.append(
        {
            "path": str(path.relative_to(root)),
            "opened": layer is not None,
            "references": resolved,
        }
    )
    usd_stage = None if layer is None else Usd.Stage.Open(layer)
    if usd_stage is not None:
        for prim in usd_stage.Traverse():
            for attribute in prim.GetAttributes():
                value = attribute.Get()
                values = [value] if isinstance(value, Sdf.AssetPath) else value if isinstance(value, (list, tuple)) else []
                for asset_value in values:
                    if not isinstance(asset_value, Sdf.AssetPath) or not asset_value.path:
                        continue
                    reference = asset_value.path
                    stack = attribute.GetPropertyStack(Usd.TimeCode.Default())
                    authoring_layer = Path(stack[0].layer.realPath).resolve() if stack and stack[0].layer.realPath else path
                    if attribute.GetName() == "info:mdl:sourceAsset" and "/" not in reference and not reference.startswith("."):
                        candidates = [
                            base / reference
                            for base in Path(sys.prefix).glob("lib/python*/site-packages/isaacsim/kit/mdl/core/Base")
                        ]
                        installed = next((candidate for candidate in candidates if candidate.is_file()), None)
                        check = {
                            "attribute": str(attribute.GetPath()),
                            "reference": reference,
                            "kind": "installed_mdl_module",
                            "resolved": str(installed) if installed else None,
                            "exists": installed is not None,
                        }
                        asset_attribute_checks.append(check)
                        if installed is None:
                            missing_dependencies.append(check)
                        continue
                    if "://" in reference or reference.startswith("omniverse:"):
                        remote_dependencies.append(
                            {"attribute": str(attribute.GetPath()), "reference": reference}
                        )
                        asset_attribute_checks.append(
                            {"attribute": str(attribute.GetPath()), "reference": reference, "kind": "remote", "exists": False}
                        )
                        continue
                    dependency = (authoring_layer.parent / reference).resolve()
                    exists = dependency.is_file()
                    check = {
                        "attribute": str(attribute.GetPath()),
                        "authoring_layer": str(authoring_layer.relative_to(root)) if authoring_layer.is_relative_to(root) else str(authoring_layer),
                        "reference": reference,
                        "resolved": str(dependency.relative_to(root)) if dependency.is_relative_to(root) else str(dependency),
                        "kind": "local",
                        "exists": exists,
                    }
                    asset_attribute_checks.append(check)
                    if not exists:
                        missing_dependencies.append(check)

stage = Usd.Stage.Open(str(rig_path), load=Usd.Stage.LoadAll)
if stage is None:
    raise RuntimeError(f"Cannot open assembly: {rig_path}")
rig = stage.GetDefaultPrim()
collision_prims = []
rigid_body_prims = []
mass_prims = []
camera_prims = []
for prim in stage.Traverse():
    path = str(prim.GetPath())
    if prim.HasAPI(UsdPhysics.CollisionAPI):
        collision_prims.append(path)
    if prim.HasAPI(UsdPhysics.RigidBodyAPI):
        rigid_body_prims.append(path)
    if prim.HasAPI(UsdPhysics.MassAPI):
        mass_prims.append(path)
    if prim.IsA(UsdGeom.Camera):
        camera_prims.append(path)

bounds = UsdGeom.BBoxCache(
    Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render]
).ComputeWorldBound(rig).ComputeAlignedRange()
config = json.loads((root / "configs/sensors/stage2_rig.json").read_text())
left = config["stereo"]["left_translation_base_m"]
right = config["stereo"]["right_translation_base_m"]
baseline = sum((a - b) ** 2 for a, b in zip(left, right)) ** 0.5

result = {
    "status": "PASS"
    if all(item["pass"] for item in hash_checks)
    and not remote_dependencies
    and not missing_dependencies
    else "FAIL",
    "source_manifest": str(source_path.relative_to(root)),
    "hash_checks": hash_checks,
    "usd_dependency_checks": usd_dependency_checks,
    "asset_attribute_checks": asset_attribute_checks,
    "dependency_policy": "Relative USD/texture assets must be localized; bare MDL sourceAsset names must resolve in the installed Isaac MDL module path.",
    "remote_dependencies": remote_dependencies,
    "missing_dependencies": missing_dependencies,
    "assembly": {
        "path": str(rig_path.relative_to(root)),
        "default_prim": str(rig.GetPath()),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "up_axis": UsdGeom.GetStageUpAxis(stage),
        "bounds_m": {"min": list(bounds.GetMin()), "max": list(bounds.GetMax())},
        "collision_prims": collision_prims,
        "rigid_body_prims": rigid_body_prims,
        "mass_prims": mass_prims,
        "embedded_camera_prims": camera_prims,
        "dynamics_policy": "No independent collision/rigid body/mass; follows the Go2-W base.",
    },
    "extrinsics": {
        "coordinate_convention": config["coordinate_convention"],
        "left_translation_base_m": left,
        "right_translation_base_m": right,
        "computed_baseline_m": baseline,
        "configured_baseline_m": config["stereo"]["baseline_m"],
        "baseline_pass": abs(baseline - config["stereo"]["baseline_m"]) < 1e-9,
        "lidar_translation_base_m": config["lidar"]["translation_base_m"],
        "imu_translation_base_m": config["imu"]["translation_base_m"],
    },
}
output.parent.mkdir(parents=True, exist_ok=True)
output.write_text(json.dumps(result, indent=2, ensure_ascii=False))
print(json.dumps({"status": result["status"], "files": len(hash_checks), "usd_layers": len(usd_dependency_checks)}))
