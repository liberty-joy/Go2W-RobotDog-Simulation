"""Export a new, versioned document/result snapshot; never copy MCAP or weights."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil

p=argparse.ArgumentParser()
p.add_argument('--run',type=Path,required=True)
p.add_argument('--destination',type=Path,required=True)
a=p.parse_args();root=Path(__file__).resolve().parents[2];run=a.run.resolve();dest=a.destination.resolve()
assert run.is_relative_to(root/'outputs/sensor_pipeline')
dest.mkdir(parents=True,exist_ok=False)
files=[root/'README.md',root/'docs/document_renames.json']
files += [f for f in (root/'docs').rglob('*.md') if f.is_file() and not f.is_symlink()]
files += [run/name for name in ['run.json','summary.json','acceptance.csv','storage_budget.csv','stage0_summary.png','document_renames.json','asset_weight_integrity.json','final_code_manifest.json'] if (run/name).exists()]
for sub in ['headless_recheck','headless_reliable','gui_capture']:
    files += [f for f in (run/sub).glob('*') if f.is_file() and f.suffix in ['.png','.json','.txt','.log','.yaml'] and f.name!='stability.json']
renames={str((root/r['old']).absolute()):(root/r['canonical']).resolve() for r in json.loads((root/'docs/document_renames.json').read_text())}
for src in files:
    target=dest/src.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True)
    if src.suffix=='.md':
        def rewrite(match):
            address=match.group(1)
            if '://' in address or address.startswith('#'):return match.group(0)
            raw,sep,anchor=address.partition('#')
            absolute=Path(os.path.abspath(src.parent/raw.strip('<>')))
            resolved=renames.get(str(absolute),absolute.resolve())
            if not resolved.is_relative_to(root):return match.group(0)
            relative=os.path.relpath(dest/resolved.relative_to(root),target.parent)
            return ']('+relative+('#'+anchor if sep else '')+')'
        target.write_text(re.sub(r'\]\(([^)]+)\)',rewrite,src.read_text()))
    else:shutil.copy2(src,target)
(dest/'00_共享快照说明.md').write_text(f'''# 00 共享快照说明

版本：{run.name}。此目录是Linux工作区单向导出的文档与选定结果，不是可在Windows运行的完整工程。

先读 [复现总览](docs/00_复现总览与文档索引.md) 和 [阶段0验收](docs/sensor_pipeline/00_environment.md)。命令在Ubuntu的 `/home/joy/桌面/RobotDog` 执行。

原始MCAP、解释器、环境、缓存、资产和策略权重留在Linux；本快照不包含它们。旧共享文件没有自动覆盖，也没有双向同步。目录内 `EXPORT_MANIFEST.json` 提供文件SHA-256。
''')
manifest={str(f.relative_to(dest)):{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in dest.rglob('*') if f.is_file()}
(dest/'EXPORT_MANIFEST.json').write_text(json.dumps({'source_run':str(run),'files':manifest},indent=2,ensure_ascii=False))
print(json.dumps({'export':str(dest),'files':len(manifest),'bytes':sum(f['bytes'] for f in manifest.values()),'raw_bags_exported':False},indent=2,ensure_ascii=False))
