"""Read-only environment inventory. Writes only to a new evidence directory."""
import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from datetime import datetime
from zoneinfo import ZoneInfo

p=argparse.ArgumentParser();p.add_argument('--out',type=Path,required=True);a=p.parse_args()
a.out.mkdir(parents=True,exist_ok=False)
root=Path(__file__).resolve().parents[2]
commands={
 'hardware':'nvidia-smi; free -b; df -B1 /home/joy/桌面/RobotDog /mnt/Shared; uname -a; cat /etc/os-release',
 'processes':'ps -eo pid,etime,pcpu,rss,args',
 'ros':'source scripts/ros2/env.sh; printenv ROS_DISTRO RMW_IMPLEMENTATION ROS_DOMAIN_ID; ros2 pkg list; dpkg-query -W ros-jazzy-rmw-fastrtps-cpp ros-jazzy-rosbag2-storage-mcap',
 'launchers':'cat /home/joy/robotics/bin/isaac-sim /home/joy/robotics/bin/isaac-lab /home/joy/robotics/bin/launch_sim.py; cat scripts/run_go2w.sh scripts/run_go2w_ros2.sh scripts/run_go2w_dual.sh scripts/train_go2w.sh',
 'source_version':'git -C /home/joy/robotics/IsaacLab-source rev-parse HEAD; git -C /home/joy/robotics/IsaacLab-source describe --tags --always; git -C /home/joy/robotics/IsaacLab-source status --porcelain',
 'native_python':"/usr/bin/python3 -c 'import sys,numpy,cv2; print(sys.executable,sys.version); print(numpy.__version__,cv2.__version__)'",
}
for name,cmd in commands.items():
    result=subprocess.run(['bash','-c',cmd],cwd=root,capture_output=True,text=True)
    (a.out/f'{name}.txt').write_text(f'$ {cmd}\nexit={result.returncode}\n'+result.stdout+result.stderr)
for env in ['isaac-env','isaaclab-env']:
    code="import sys,json,importlib.metadata as m; names=['isaacsim','isaaclab','torch','numpy','rsl-rl-lib']; print(json.dumps({'executable':sys.executable,'python':sys.version,'packages':{n:next((d.version for d in m.distributions() if d.metadata['Name'].lower()==n),None) for n in names}},indent=2))"
    result=subprocess.run([f'/home/joy/robotics/{env}/bin/python','-c',code],capture_output=True,text=True)
    (a.out/f'{env}.json').write_text(result.stdout+result.stderr)
    result=subprocess.run([f'/home/joy/robotics/{env}/bin/python','/home/joy/robotics/checks/check_gpu.py'],capture_output=True,text=True)
    (a.out/f'{env}_gpu.txt').write_text(f'exit={result.returncode}\n'+result.stdout+result.stderr)
files=[path for folder in ['scripts','training/go2w','models/go2w','sim/assets','outputs/training']
       for path in (root/folder).rglob('*') if path.is_file() and '__pycache__' not in str(path)
       and (folder!='outputs/training' or path.suffix in ('.pt','.json'))]
manifest={str(path.relative_to(root)):{'bytes':path.stat().st_size,'sha256':hashlib.sha256(path.read_bytes()).hexdigest()} for path in files}
(a.out/'files.json').write_text(json.dumps(manifest,indent=2))
folders={path.name:{'files':sum(1 for f in path.rglob('*') if f.is_file()),'immediate_children':sorted(f.name for f in path.iterdir())}
         for path in root.iterdir() if path.is_dir() and path.name!='outputs'}
(a.out/'folders.json').write_text(json.dumps(folders,indent=2,ensure_ascii=False))
(a.out/'time.json').write_text(json.dumps({'local':datetime.now(ZoneInfo('Asia/Shanghai')).isoformat(),'new_license_acceptance':False,'package_changes':False},indent=2))
print(a.out)
