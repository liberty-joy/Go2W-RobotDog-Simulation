"""Minimal pinned original CLIP source + one public checkpoint; never touches vendor originals."""
import argparse,json,hashlib,shutil,datetime
from pathlib import Path
import requests
p=argparse.ArgumentParser();p.add_argument('--preflight',type=Path,required=True);a=p.parse_args();pre=json.loads(a.preflight.read_text());dest=Path('third_party/stage7_clip');model=Path('models/perception/stage7_clip');dest.mkdir(parents=True,exist_ok=True);model.mkdir(parents=True,exist_ok=True)
assert shutil.disk_usage(model).free>5*1024**3
files=[];commit=pre['official_commit']
items=[(f'https://raw.githubusercontent.com/openai/CLIP/{commit}/{name}',dest/name) for name in ['LICENSE','clip/__init__.py','clip/clip.py','clip/model.py','clip/simple_tokenizer.py','clip/bpe_simple_vocab_16e6.txt.gz']]+[(pre['url'],model/'ViT-B-32.pt')]
for url,path in items:
 path.parent.mkdir(parents=True,exist_ok=True)
 if path.exists():raise FileExistsError('Refusing existing vendor item '+str(path))
 temp=path.with_name(path.name+'.partial');h=hashlib.sha256();size=0
 try:
  with requests.get(url,stream=True,timeout=(15,60)) as response:
   response.raise_for_status();expected=response.headers.get('Content-Length');print('DOWNLOAD',url,'content_length',expected,flush=True)
   with temp.open('wb') as out:
    for chunk in response.iter_content(1024*1024):
     if chunk:out.write(chunk);h.update(chunk);size+=len(chunk)
  if path.suffix=='.pt' and h.hexdigest()!=pre['sha256_from_official_url']:raise ValueError('Official checkpoint SHA mismatch')
  temp.rename(path);files.append({'path':str(path),'url':url,'bytes':size,'sha256':h.hexdigest(),'commit':commit,'license':'MIT; original LICENSE included'})
 except Exception as e:
  (model/'DOWNLOAD_FAILED.json').write_text(json.dumps({'url':url,'error':str(e),'partial_bytes':size},indent=2));raise
manifest={'date':datetime.datetime.now().astimezone().isoformat(),'source':'OpenAI original CLIP repository, pinned commit','checkpoint':'ViT-B/32','files':files,'device':'CPU; no remote inference service; no login/payment','scope':'generic zero-shot image/text similarity, not validated industrial equipment recognition','preflight':pre}
(model/'SOURCE.json').write_text(json.dumps(manifest,indent=2));(dest/'SOURCE.json').write_text(json.dumps(manifest,indent=2));print('DOWNLOAD_COMPLETE',sum(f['bytes'] for f in files),flush=True)
