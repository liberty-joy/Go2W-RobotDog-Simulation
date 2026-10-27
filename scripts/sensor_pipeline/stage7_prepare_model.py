"""Prepare only the small official model actually used in Stage7, with pinned SHA."""
import json,hashlib,shutil,datetime
from pathlib import Path
import requests
source=json.loads(Path('configs/models/stage7_squeezenet_SOURCE.json').read_text());item=source['files'][0];dest=Path('models/perception/stage7_squeezenet');dest.mkdir(parents=True,exist_ok=True);path=dest/item['path']
if path.exists():
 assert hashlib.sha256(path.read_bytes()).hexdigest()==item['sha256'],'Existing model mismatch; refusing overwrite'
 print('Existing pinned SqueezeNet verified:',path)
else:
 assert shutil.disk_usage(dest).free>1024**3,'Need at least 1GiB free'
 head=requests.head(item['url'],timeout=20);head.raise_for_status();size=int(head.headers.get('Content-Length',item['bytes']));assert size==item['bytes']
 temp=path.with_suffix('.partial');sha=hashlib.sha256();total=0
 with requests.get(item['url'],stream=True,timeout=(15,60)) as r:
  r.raise_for_status()
  with temp.open('wb') as f:
   for chunk in r.iter_content(1024*1024):f.write(chunk);sha.update(chunk);total+=len(chunk)
 assert total==item['bytes'] and sha.hexdigest()==item['sha256'],'Incomplete/untrusted download retained as partial'
 temp.replace(path);license=requests.get(source['license_url'],timeout=20);license.raise_for_status();(dest/'LICENSE').write_text(license.text);(dest/'SOURCE.json').write_text(json.dumps(source,indent=2));print('Downloaded and SHA verified:',path,total)
