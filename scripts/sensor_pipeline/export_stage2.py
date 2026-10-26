"""One-way versioned Stage 2 document/result export; excludes vendor assets and raw XWD."""
import argparse
import hashlib
import json
import os
import re
import shutil
from pathlib import Path


parser = argparse.ArgumentParser()
parser.add_argument("--run", type=Path, required=True)
parser.add_argument("--destination", type=Path, required=True)
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
run = args.run.resolve()
destination = args.destination.resolve()
if not run.is_relative_to(root / "outputs/sensor_pipeline"):
    raise ValueError("--run must be inside outputs/sensor_pipeline")
destination.mkdir(parents=True, exist_ok=False)

files = [root / "README.md", root / "docs/document_renames.json"]
files += [path for path in (root / "docs").rglob("*.md") if path.is_file() and not path.is_symlink()]
files += list((root / "configs/sensors").glob("*.json"))
files += [root / "sim/assets/sensors/SOURCE.json", root / "sim/assets/sensors/manifest.json"]
files += list((root / "sim/assets/sensors/assemblies").glob("*.usda"))
files += [path for path in run.iterdir() if path.is_file() and path.suffix in {".json", ".csv", ".png", ".log", ".txt"}]
for name, suffixes in {
    "smoke_headless_01": {".json"},
    "smoke_headless_02": {".json"},
    "smoke_headless_03": {".json", ".png"},
    "smoke_gui_01": {".json", ".png"},
    "logs": {".log", ".txt", ".sha256"},
}.items():
    folder = run / name
    if folder.is_dir():
        files += [path for path in folder.iterdir() if path.is_file() and path.suffix in suffixes]

renames = {
    str((root / item["old"]).absolute()): (root / item["canonical"]).resolve()
    for item in json.loads((root / "docs/document_renames.json").read_text())
}
for source in files:
    target = destination / source.relative_to(root)
    target.parent.mkdir(parents=True, exist_ok=True)
    if source.suffix == ".md":
        def rewrite(match):
            address = match.group(1)
            if "://" in address or address.startswith("#"):
                return match.group(0)
            raw, separator, anchor = address.partition("#")
            absolute = Path(os.path.abspath(source.parent / raw.strip("<>")))
            resolved = renames.get(str(absolute), absolute.resolve())
            if not resolved.is_relative_to(root):
                return match.group(0)
            relative = os.path.relpath(destination / resolved.relative_to(root), target.parent)
            return "](" + relative + ("#" + anchor if separator else "") + ")"
        target.write_text(re.sub(r"\]\(([^)]+)\)", rewrite, source.read_text()))
    else:
        shutil.copy2(source, target)

(destination / "LOCAL_REPRODUCTION.md").write_text(
    "# 本机版本导出\n\n"
    "这是阶段2文档和选定小结果的只读快照，不是可独立运行的软件包。完整代码、官方供应商资产与原始数据留在Ubuntu "
    "`/home/joy/桌面/RobotDog`；所有复现命令从该目录执行。首读 `docs/00_复现总览与文档索引.md`，阶段2见 "
    "`docs/sensor_pipeline/02_assets.md`。未复制供应商USD、缓存、环境、权重、SDK或原始XWD；不要反向覆盖Linux工作区。\n"
)
manifest = {
    str(path.relative_to(destination)): {
        "bytes": path.stat().st_size,
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
    }
    for path in destination.rglob("*")
    if path.is_file()
}
(destination / "export_manifest.json").write_text(
    json.dumps(manifest, indent=2, ensure_ascii=False)
)
print(
    json.dumps(
        {
            "destination": str(destination),
            "files": len(manifest),
            "bytes": sum(item["bytes"] for item in manifest.values()),
            "one_way": True,
            "vendor_assets_exported": False,
            "project_assembly_layers_exported": True,
        },
        ensure_ascii=False,
    )
)
