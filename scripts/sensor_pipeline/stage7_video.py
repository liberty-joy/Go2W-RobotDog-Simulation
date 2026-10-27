"""Encode actual rendered frames using installed OpenCV; no system codec install."""
import argparse,cv2,json
from pathlib import Path
p=argparse.ArgumentParser();p.add_argument('--run',type=Path,required=True);a=p.parse_args();frames=sorted((a.run/'publisher/video_frames').glob('*.jpg'))
if not frames:raise RuntimeError('No actual rendered video frames')
im=cv2.imread(str(frames[0]));h,w=im.shape[:2];v=cv2.VideoWriter(str(a.run/'actual_simulation.mp4'),cv2.VideoWriter_fourcc(*'mp4v'),4.,(w,h))
if not v.isOpened():raise RuntimeError('Installed OpenCV mp4v encoder unavailable')
for frame in frames:v.write(cv2.imread(str(frame)))
v.release();check=cv2.VideoCapture(str(a.run/'actual_simulation.mp4'));count=int(check.get(cv2.CAP_PROP_FRAME_COUNT));check.release();assert count==len(frames)
(a.run/'video_metadata.json').write_text(json.dumps({'frames':count,'fps':4,'source':'actual Isaac overview RGB with embedded simulation time','GUI_screenshot':False,'time_scale':'4fps presentation; actual simulation timestamps embedded; not wall-time performance video'},indent=2))
print('VIDEO_PASS',count)
