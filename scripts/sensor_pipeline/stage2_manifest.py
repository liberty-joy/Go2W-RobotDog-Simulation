"""Create Stage 2 SOURCE/manifest from local immutable files and explicit source records."""
import argparse
import hashlib
import json
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[2])
parser.add_argument("--run", type=Path, required=True)
args = parser.parse_args()
root = args.root.resolve()
run = args.run.resolve()
assets = root / "sim/assets/sensors"

official_root = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.2/Isaac/Sensors"
records = []
for path in sorted((assets / "vendor/nvidia_isaac_6_2").rglob("*")):
    if not path.is_file():
        continue
    relative_vendor = path.relative_to(assets / "vendor/nvidia_isaac_6_2").as_posix()
    if relative_vendor == "Orbbec/Gemini335L/orbbec_gemini_335L.usd":
        category = "sensor USD/CAD appearance + embedded camera configuration"
    elif relative_vendor == "Ouster/OS0/OS0.usd":
        category = "sensor USD assembly + variant index"
    elif path.suffix == ".usda":
        category = "measurement model/config"
    else:
        category = "sensor USD/CAD appearance asset"
    records.append(
        {
            "category": category,
            "path": path.relative_to(root).as_posix(),
            "source_url": official_root + "/" + relative_vendor,
            "source_version": "Isaac Sim 6.2 official asset catalog; no Git commit exposed; local bytes pinned by SHA-256",
            "license": "NVIDIA Omniverse/Isaac content terms; vendor certification does not transfer hardware truth to derived proxies",
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "vendor_original_unchanged": True,
        }
    )

project_files = [
    assets / "assemblies/go2w_sensor_rig.usda",
    root / "configs/sensors/stage2_rig.json",
    root / "configs/sensors/generic_rotary_64.json",
]
for path in project_files:
    records.append(
        {
            "category": "assembly layer" if path.suffix in {".usd", ".usda"} else "measurement model/config",
            "path": path.relative_to(root).as_posix(),
            "source_url": None,
            "source_version": "project-authored Stage 2",
            "license": "project-local; upstream referenced assets retain their own terms",
            "bytes": path.stat().st_size,
            "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "vendor_original_unchanged": None,
        }
    )

source = {
    "schema_version": 1,
    "access_date": "2026-10-26",
    "isaac_version": "6.2.0.0-rc.25+release.10547.7dfc9ea2.gl",
    "asset_root": "sim/assets/sensors",
    "categories": {
        "sensor_usd_cad_appearance": "Gemini335L certified camera USD; OS0 family exterior and REV6/REV7 family USD",
        "measurement_models_configs": "Gemini asset contains NVIDIA depth sensor metadata; Stage 2 smoke deliberately uses explicit ideal stereo proxy and generic rotary64 config",
        "algorithm_source": "none downloaded in Stage 2; existing project import/smoke code only",
        "pretrained_weights": "none downloaded; asset import and measurement smoke require no learned model",
    },
    "truth_boundaries": [
        "Orbbec Gemini335L vendor USD is preserved and referenced as appearance/digital-twin evidence.",
        "OS0 USD only exposes REV6/REV7; the target REV8 64-channel behavior remains a parameterized proxy.",
        "Two camera prims or vendor depth output do not prove SDK stereo matching.",
        "The project proxy does not model active IR, rolling shutter, factory beam calibration or hardware clocks.",
    ],
    "resources": records,
}
(assets / "SOURCE.json").write_text(json.dumps(source, indent=2, ensure_ascii=False))
manifest = {
    "schema_version": 1,
    "generated_for_run": run.name,
    "files": {
        record["path"]: {"bytes": record["bytes"], "sha256": record["sha256"]}
        for record in records
    },
    "relative_references": {
        "sim/assets/sensors/assemblies/go2w_sensor_rig.usda": [
            "../vendor/nvidia_isaac_6_2/Orbbec/Gemini335L/orbbec_gemini_335L.usd",
            "../vendor/nvidia_isaac_6_2/Ouster/OS0/OS0.usd",
        ],
        "sim/assets/sensors/vendor/nvidia_isaac_6_2/Ouster/OS0/OS0.usd": [
            "../meshes/os0_mesh.usd",
        ] + [path.name for path in sorted((assets / "vendor/nvidia_isaac_6_2/Ouster/OS0").glob("*.usda"))],
        "sim/assets/sensors/vendor/nvidia_isaac_6_2/Orbbec/Gemini335L/orbbec_gemini_335L.usd": [],
    },
    "http_headers_observed": {
        "access_date": "2026-10-26",
        "common_last_modified": "Wed, 07 Oct 2026 01:11:50-54 GMT for the five probed objects",
        "probed": {
            "sim/assets/sensors/vendor/nvidia_isaac_6_2/Orbbec/Gemini335L/orbbec_gemini_335L.usd": {"content_length": 50671410, "etag": "69a92c31c42f466a4a6cdf92a9b2bf56-7"},
            "sim/assets/sensors/vendor/nvidia_isaac_6_2/Ouster/OS0/OS0.usd": {"content_length": 16032, "etag": "d5bc3a14e75cc4baab084f6d6e4ef452"},
            "sim/assets/sensors/vendor/nvidia_isaac_6_2/Ouster/meshes/os0_mesh.usd": {"content_length": 240923, "etag": "75dec7a3250a0767b89be9734284aa0c"},
            "sim/assets/sensors/vendor/nvidia_isaac_6_2/Ouster/OS0/OS0_REV7_128ch10hz512res.usda": {"content_length": 14965, "etag": "934eedcd3ccecf5b80752a172f7be09d"},
            "sim/assets/sensors/vendor/nvidia_isaac_6_2/Ouster/meshes/textures/sensor_color.png": {"content_length": 9461, "etag": "6b2b5bc2b69388035fe9b82b155dc75a"},
        }
    },
    "remote_dependencies_expected": [],
}
(assets / "manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
(run / "asset_manifest.json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False))
print(json.dumps({"resources": len(records), "bytes": sum(item["bytes"] for item in records)}))
