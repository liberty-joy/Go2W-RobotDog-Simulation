"""One-way versioned stage1 document/result export; no SDKs, bags, source PDFs or weights."""
import argparse,hashlib,json,os,re,shutil
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);p.add_argument('--destination',type=Path,required=True);a=p.parse_args()
root=Path(__file__).resolve().parents[2];run=a.run.resolve();dest=a.destination.resolve()
assert run.is_relative_to(root/'outputs/sensor_pipeline')
dest.mkdir(parents=True,exist_ok=False)
files=[root/'README.md',root/'docs/document_renames.json']
files += [f for f in (root/'docs').rglob('*.md') if f.is_file() and not f.is_symlink()]
files += list((root/'configs/sensors').glob('*.json'))
files += [f for f in run.iterdir() if f.is_file() and f.suffix in ['.json','.csv','.png','.log','.txt']]
for sub in ['sensor_smoke_final','ros_recheck','mode_recheck','sources']:
 files += [f for f in (run/sub).iterdir() if f.is_file() and f.suffix in (['.json'] if sub=='sources' else ['.json','.png','.log'])]
renames={str((root/r['old']).absolute()):(root/r['canonical']).resolve() for r in json.loads((root/'docs/document_renames.json').read_text())}
for src in files:
 target=dest/src.relative_to(root);target.parent.mkdir(parents=True,exist_ok=True)
 if src.suffix=='.md':
  def rewrite(m):
   address=m.group(1)
   if '://' in address or address.startswith('#'):return m.group(0)
   raw,sep,anchor=address.partition('#');absolute=Path(os.path.abspath(src.parent/raw.strip('<>')));resolved=renames.get(str(absolute),absolute.resolve())
   if not resolved.is_relative_to(root):return m.group(0)
   return ']('+os.path.relpath(dest/resolved.relative_to(root),target.parent)+('#'+anchor if sep else '')+')'
  target.write_text(re.sub(r'\]\(([^)]+)\)',rewrite,src.read_text()))
 else:shutil.copy2(src,target)
(dest/'LOCAL_REPRODUCTION.md').write_text('# 本机版本导出\n\n这是文档和选定结果快照，不是可独立运行的软件包。完整代码/原始数据在Ubuntu `/home/joy/桌面/RobotDog`；命令从该目录执行。首读 docs/00_复现总览与文档索引.md，阶段1见 docs/sensor_pipeline/01_selection.md。历史阶段链接指向Linux完整工作区；未复制历史包、权重、环境、SDK或版权来源PDF。不要反向覆盖Linux工作区。\n')
manifest={str(f.relative_to(dest)):{'bytes':f.stat().st_size,'sha256':hashlib.sha256(f.read_bytes()).hexdigest()} for f in dest.rglob('*') if f.is_file()}
(dest/'export_manifest.json').write_text(json.dumps(manifest,indent=2,ensure_ascii=False))
print(json.dumps({'destination':str(dest),'files':len(manifest),'bytes':sum(v['bytes'] for v in manifest.values()),'one_way':True},ensure_ascii=False))
