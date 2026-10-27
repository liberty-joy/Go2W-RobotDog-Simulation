"""CPU-only fixed visual feature worker. Outputs data, never executable actions."""
import argparse,json,time,hashlib,os
from pathlib import Path
import torch,torchvision
from PIL import Image
p=argparse.ArgumentParser();p.add_argument('--io',type=Path,required=True);a=p.parse_args();a.io.mkdir(parents=True,exist_ok=True)
torch.set_num_threads(4);weight=Path('models/perception/stage7_squeezenet/squeezenet1_1-b8a52dc0.pth');sha=hashlib.sha256(weight.read_bytes()).hexdigest();assert sha.startswith('b8a52dc0')
from torchvision.models import squeezenet1_1,SqueezeNet1_1_Weights
meta=SqueezeNet1_1_Weights.IMAGENET1K_V1;model=squeezenet1_1(weights=None);model.load_state_dict(torch.load(weight,map_location='cpu',weights_only=True));model.eval();preprocess=meta.transforms();categories=meta.meta['categories']
with torch.inference_mode():model(torch.zeros(1,3,224,224))
(a.io/'ready.json').write_text(json.dumps({'model':'SqueezeNet1_1 ImageNet1K V1','torchvision':torchvision.__version__,'torch':torch.__version__,'checkpoint_sha256':sha,'device':'CPU','CUDA_context':False,'pid':os.getpid(),'identity_source':'manual registry; features only establish appearance consistency'}))
try:
 while not (a.io/'stop').exists():
  for request in sorted(a.io.glob('*.request.json')):
   response=request.with_name(request.name.replace('.request.json','.response.json'))
   if response.exists():continue
   start=time.perf_counter()
   try:
    data=json.loads(request.read_text());image=Image.open(data['crop_path']).convert('RGB');x=preprocess(image).unsqueeze(0)
    with torch.inference_mode():
     features=model.features(x).mean(dim=(-2,-1))[0];features=features/features.norm().clamp_min(1e-9);scores=model(x)[0].softmax(0);vals,idx=scores.topk(5)
    result={'status':'ok','embedding':features.tolist(),'top5':[{'imagenet_label':categories[int(i)],'probability':float(v)} for v,i in zip(vals,idx)],'inference_ms':(time.perf_counter()-start)*1000,'source_rgb_sha256':data['source_rgb_sha256'],'model':'squeezenet1_1-imagenet1k-v1','checkpoint_sha256':sha,'scope':'generic appearance feature; not equipment identity classifier'}
   except Exception as e:result={'status':'error','error':str(e)}
   temporary=response.with_suffix('.tmp');temporary.write_text(json.dumps(result));temporary.replace(response)
  time.sleep(.025)
except KeyboardInterrupt:pass
