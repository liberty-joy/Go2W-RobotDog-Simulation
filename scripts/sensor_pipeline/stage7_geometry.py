"""Known-map SE(2) scan matching and occupied/inflated grid planning, without truth inputs."""
import heapq,math
import numpy as np
from scipy.spatial import cKDTree
from scipy.ndimage import distance_transform_edt

def angle(x):return (x+np.pi)%(2*np.pi)-np.pi
def rot2(a):return np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
class KnownMap:
 def __init__(self,cfg):
  self.cfg=cfg;self.bounds=np.array(cfg['map_bounds_m']);self.res=cfg['resolution_m'];x0,y0,x1,y1=self.bounds;self.w=int(round((x1-x0)/self.res))+1;self.h=int(round((y1-y0)/self.res))+1
  self.occupied=np.zeros((self.h,self.w),bool);self.occupied[[0,-1],:]=True;self.occupied[:,[0,-1]]=True;points=[]
  # Exact inside surfaces of 12cm thick walls, matched to configured physical geometry.
  for y in [y0+.06,y1-.06]:points.extend([[x,y] for x in np.arange(x0+.06,x1-.06,.03)])
  for x in [x0+.06,x1-.06]:points.extend([[x,y] for y in np.arange(y0+.06,y1-.06,.03)])
  for b in cfg['static_boxes']:
   cx,cy,_=b['center'];sx,sy,_=b['size'];a,c=cx-sx/2,cx+sx/2;d,e=cy-sy/2,cy+sy/2
   for x in [a,c]:points.extend([[x,y] for y in np.arange(d,e+.01,.025)])
   for y in [d,e]:points.extend([[x,y] for x in np.arange(a,c+.01,.025)])
   lo=self.cell([a,d]);hi=self.cell([c,e]);self.occupied[lo[1]:hi[1]+1,lo[0]:hi[0]+1]=True
  self.cloud=np.asarray(points);self.tree=cKDTree(self.cloud);self.inflated=distance_transform_edt(~self.occupied)*self.res<=cfg['inflation_radius_m']
 def cell(self,p):return np.rint((np.asarray(p)-self.bounds[:2])/self.res).astype(int)
 def xy(self,c):return self.bounds[:2]+np.asarray(c)*self.res
 def free(self,c):x,y=c;return 0<=x<self.w and 0<=y<self.h and not self.inflated[y,x]
 def plan(self,start,goal):
  s=tuple(self.cell(start));g=tuple(self.cell(goal))
  if not self.free(s) or not self.free(g):return None
  queue=[(0.,s)];parent={s:None};cost={s:0.}
  while queue:
   _,cur=heapq.heappop(queue)
   if cur==g:
    cells=[]
    while cur is not None:cells.append(cur);cur=parent[cur]
    return np.asarray([self.xy(c) for c in cells[::-1]])
   for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,-1),(1,-1),(-1,1)]:
    nxt=(cur[0]+dx,cur[1]+dy)
    if not self.free(nxt):continue
    if dx and dy and (not self.free((cur[0]+dx,cur[1])) or not self.free((cur[0],cur[1]+dy))):continue
    nc=cost[cur]+math.hypot(dx,dy)
    if nc<cost.get(nxt,float('inf')):cost[nxt]=nc;parent[nxt]=cur;heapq.heappush(queue,(nc+np.linalg.norm(np.subtract(g,nxt)),nxt))
  return None
 def localize(self,points,initial):
  # Explicit horizontal height slice only for planar localization, not a flattened 3D safety map.
  p=np.asarray(points);valid=np.isfinite(p).all(axis=1)&(p[:,2]>-.18)&(p[:,2]<.35)&(np.linalg.norm(p[:,:2],axis=1)>.65);p=p[valid,:2]
  if not len(p):return None,{'reason':'no_height_slice_points'}
  _,idx=np.unique(np.round(p/.055).astype(int),axis=0,return_index=True);p=p[idx];pose=np.array(initial,float);rmse=999.;n=0
  for _ in range(15):
   world=p@rot2(pose[2]).T+pose[:2];d,j=self.tree.query(world);good=d<self.cfg['icp_max_correspondence_m'];n=int(good.sum())
   if n<self.cfg['icp_min_points']:return None,{'reason':'insufficient_map_correspondences','count':n}
   # Robust trimmed fit excludes equipment/dynamic returns that are not on the known structural map.
   threshold=min(self.cfg['icp_max_correspondence_m'],float(np.quantile(d[good],.85))+.005);good&=d<=threshold
   a=world[good];b=self.cloud[j[good]];ac=a.mean(0);bc=b.mean(0);u,sv,vh=np.linalg.svd((a-ac).T@(b-bc));R=vh.T@u.T
   if np.linalg.det(R)<0:vh[-1]*=-1;R=vh.T@u.T
   dt=bc-R@ac;dyaw=math.atan2(R[1,0],R[0,0]);pose[:2]=R@pose[:2]+dt;pose[2]=angle(pose[2]+dyaw);rmse=float(np.sqrt(np.mean(d[good]**2)))
   if np.linalg.norm(dt)<.0002 and abs(dyaw)<.0002:break
  if rmse>self.cfg['icp_max_rmse_m']:return None,{'reason':'map_fit_rejected','rmse_m':rmse,'count':n}
  return pose,{'rmse_m':rmse,'count':n,'source':'3D lidar explicit height slice -> known map ICP','truth_pose_used':False}
