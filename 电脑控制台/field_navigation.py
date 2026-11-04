"""Fixed rectangular field frame and automatic two-sided obstacle bypass.

Centimetres/degrees. No hardware access. Unseen field interior is a planning
hypothesis; every emitted short movement also requires current laser clearance.
"""
from dataclasses import dataclass,field
import math,heapq,statistics,time
from local_navigation import ScanTracker,Match,Snapshot,Rectangle,cloud_points,world_points,rotate,wrap,NavigationWorker

def fit_field(points,expected=(330,260)):
 raw=[p[:2] for p in cloud_points(points) if p[3]>=35];best=None;bestscore=0
 if len(raw)<40:return None
 def search(angles):
  nonlocal best,bestscore
  for angle in angles:
   pts=[rotate(x,y,angle) for x,y in raw];axes=[]
   for axis in (0,1):
    hist={}
    for p in pts:
     k=round(p[axis]/4);hist[k]=hist.get(k,0)+1
    pair=[]
    for sign in (-1,1):
     keys=sorted((k for k in hist if k*sign>=4),key=lambda k:hist[k],reverse=True);chosen=[]
     for k in keys:
      if all(abs(k-j)>3 for j in chosen):chosen.append(k)
      if len(chosen)==4:break
     pair.append([(k*4,[p for p in pts if abs(p[axis]-k*4)<=6]) for k in chosen])
    axes.append(pair)
   for xmin,left in axes[0][0]:
    for xmax,right in axes[0][1]:
     w=xmax-xmin
     if not 80<=w<=600 or (expected and all(abs(w-v)>v*.25 for v in expected)):continue
     for ymin,bottom in axes[1][0]:
      for ymax,top in axes[1][1]:
       h=ymax-ymin
       if not 80<=h<=600:continue
       pairs=[expected,expected[::-1]] if expected else []
       matches=[(ew,eh) for ew,eh in pairs if abs(w-ew)<=ew*.25 and abs(h-eh)<=eh*.25]
       if expected and not matches:continue
       # A closed perimeter should enclose most returns. An interior crossbar
       # must not become a boundary while long sidewalls continue beyond it.
       enclosed=sum(xmin-8<=x<=xmax+8 and ymin-8<=y<=ymax+8 for x,y in pts)/len(pts)
       if enclosed<.80:continue
       line_sets=[([p for p in left if ymin-6<=p[1]<=ymax+6],1,ymin,ymax,xmin),([p for p in right if ymin-6<=p[1]<=ymax+6],1,ymin,ymax,xmax),([p for p in bottom if xmin-6<=p[0]<=xmax+6],0,xmin,xmax,ymin),([p for p in top if xmin-6<=p[0]<=xmax+6],0,xmin,xmax,ymax)]
       coverage=[];errors=[];valid=True
       for ps,axis,lo,hi,wall in line_sets:
        vals=sorted(p[axis] for p in ps);bins={max(0,min(11,int((v-lo)/(hi-lo)*12))) for v in vals}
        if len(vals)<6 or max(vals,default=0)-min(vals,default=0)<max(45,.20*(hi-lo)) or len(bins)<3:valid=False;break
        coverage.append(min(1,(max(vals)-min(vals))/(hi-lo)));errors.extend(abs(p[1-axis]-wall) for p in ps)
       if not valid or coverage[0]+coverage[1]<.9 or coverage[2]+coverage[3]<.9:continue
       weighted=(coverage[0]*h+coverage[1]*h+coverage[2]*w+coverage[3]*w)/(2*w+2*h)
       if weighted<.55:continue
       residual=sum(errors)/len(errors)
       if residual>3.5:continue
       corners=0
       for xi,yi in [(0,0),(1,0),(1,1),(0,1)]:
        xc=xmax if xi else xmin;yc=ymax if yi else ymin
        if min(abs(p[1]-yc) for p in line_sets[xi][0])<max(15,h*.15) and min(abs(p[0]-xc) for p in line_sets[2+yi][0])<max(15,w*.15):corners+=1
       prior=min((abs(w-ew)/ew+abs(h-eh)/eh for ew,eh in matches),default=0)
       score=weighted*(1+.025*corners)*math.sqrt(w*h)/(1+.5*residual+prior)
       if score>bestscore:bestscore=score;best=Rectangle(angle,xmin,xmax,ymin,ymax,weighted,corners,len(errors))
 search(range(-44,45,2))
 if best:search([best.angle+i*.5 for i in range(-4,5)])
 return best

@dataclass
class Field:
 width:float;height:float;heading:float=0

class FieldTracker(ScanTracker):
 def __init__(self):
  super().__init__();self.frame=None;self.pending=False;self.expected=(330,260);self.candidate=None;self.anchor=None;self.votes=0;self.cal_yaw=None
 def request(self,expected):
  if expected and not all(math.isfinite(v) and 80<=v<=600 for v in expected):raise ValueError('边长参考范围80～600cm')
  self.__init__();self.pending=True;self.expected=expected
 def make_lines(self,points,pose):
  lines=super().make_lines(points,pose)
  if self.frame:
   f=self.frame
   lines=[p for p in lines if -8<=p[0]<=f.width+8 and -8<=p[1]<=f.height+8 and min(abs(p[0]),abs(p[0]-f.width),abs(p[1]),abs(p[1]-f.height))<=9]
  return lines
 def wall_yaw(self,points,imu_yaw):
  """Anchor slow gyro drift to the fixed four walls, allowing translation."""
  predicted=wrap(imu_yaw-self.origin_yaw);x,y,_=self.pose;f=self.frame;best=(1e9,predicted)
  raw=cloud_points(points)
  for delta in (i*.5 for i in range(-6,7)):
   angle=wrap(predicted+delta);axes=[[],[]]
   for bx,by,*_ in raw:
    px,py=rotate(bx,by,angle);px+=x;py+=y
    if not -40<=px<=f.width+40 or not -40<=py<=f.height+40:continue
    choices=[(abs(px),0,-px),(abs(px-f.width),0,f.width-px),(abs(py),1,-py),(abs(py-f.height),1,f.height-py)]
    distance,axis,error=min(choices)
    if distance<35:axes[axis].append(error)
   if min(map(len,axes))<10:continue
   residual=0
   for errors in axes:
    median=statistics.median(errors);deviations=sorted(abs(e-median) for e in errors);kept=deviations[:max(8,int(len(deviations)*.8))];residual+=sum(kept)/len(kept)
   score=residual+.015*abs(delta)
   if score<best[0]:best=(score,angle)
  return best[1] if best[0]<8 else predicted
 def update(self,points,imu_yaw,now):
  if not self.frame:
   if not self.pending:return self.fail('请先点击“静止建立场地坐标”')
   if self.cal_yaw is None:self.cal_yaw=imu_yaw
   if abs(wrap(imu_yaw-self.cal_yaw))>1:self.votes=0;self.cal_yaw=imu_yaw;return self.fail('建系期间有转动，请保持静止')
   r=fit_field(points,self.expected)
   if not r:self.votes=0;return self.fail('四边尚未匹配：核对边长参考、扫描高度和遮挡')
   prev=self.anchor if self.votes else None;self.candidate=r
   similar=prev and abs(r.angle-prev.angle)<2 and abs(r.xmin-prev.xmin)<6 and abs(r.ymin-prev.ymin)<6 and abs((r.xmax-r.xmin)-(prev.xmax-prev.xmin))<12 and abs((r.ymax-r.ymin)-(prev.ymax-prev.ymin))<12
   self.votes=self.votes+1 if similar else 1
   if not similar:self.anchor=r
   if self.votes<3:return self.fail(f'已识别四边，静止确认 {self.votes}/3')
   self.frame=Field(r.xmax-r.xmin,r.ymax-r.ymin,r.angle);self.pose=(-r.xmin,-r.ymin,r.angle)
   self.origin_yaw=imu_yaw-r.angle;self.key_pose=self.pose[:2];self.lines=self.make_lines(points,self.pose);self.last_time=now;self.good_time=now;self.stable=3;self.pending=False
   if len(self.lines)<24:self.frame=None;self.pending=True;return self.fail('边框配准支持不足，继续静止采集')
   errors=[min(abs(x),abs(x-self.frame.width),abs(y),abs(y-self.frame.height)) for x,y in world_points(points,self.pose)]
   errors=[e for e in errors if e<=6];rms=math.sqrt(sum(e*e for e in errors)/max(1,len(errors)))
   self.quality=Match(True,*self.pose[:2],rms=rms,support=len(self.lines),reason='场地建系成功：左下原点，+X右，+Y上');return self.quality
  if now-self.last_time<=1.0:self.origin_yaw=imu_yaw-self.wall_yaw(points,imu_yaw)
  result=super().update(points,imu_yaw,now)
  if result.ok:
   x,y,_=self.pose
   if not -5<=x<=self.frame.width+5 or not -5<=y<=self.frame.height+5:return self.fail('定位超出已校准边框，请停车重新建系')
   result.reason='边框定位有效'
  return result

class FieldPlanner:
 def __init__(self,points,pose,frame,memory=None):
  from local_navigation import LocalPlanner
  self.pose=pose;self.frame=frame
  # Boundary is exact geometry, not a collection of inflated interior objects.
  interior=[]
  for a,r in points:
   x,y=rotate(0,r,pose[2]-a);x+=pose[0];y+=pose[1]
   if 8<x<frame.width-8 and 8<y<frame.height-8:interior.append((a,r))
  self.local=LocalPlanner(interior,pose);self.local.bins={int(a//2)%180:r for a,r in points if 5<=r<=500}
  self.blocked=self.local.blocked;self.cache={}
  if memory:self.blocked.update(memory)
  a=math.radians(pose[2]);self.mx=abs(math.cos(a))*25+abs(math.sin(a))*32;self.my=abs(math.sin(a))*25+abs(math.cos(a))*32
 def cell(self,p):return round(p[0]/10),round(p[1]/10)
 def free(self,p):
  x,y=p[0]*10,p[1]*10
  return p not in self.blocked and self.mx<=x<=self.frame.width-self.mx and self.my<=y<=self.frame.height-self.my
 def short_clear(self,x,y):return self.free(self.cell((x,y))) and self.local.visible(self.cell((x,y)))
 def search(self,target,gate=None,side=0,start_at=None):
  start=self.cell(self.pose if start_at is None else start_at);goal=self.cell(target)
  if not self.free(start):return [],'车辆太靠近障碍或边界'
  if not self.free(goal):return [],'目标落在障碍或边框余量内'
  heap=[(0,0,start)];cost={start:0};parent={}
  while heap:
   _,g,p=heapq.heappop(heap)
   if g!=cost.get(p):continue
   if p==goal:
    path=[p]
    while path[-1]!=start:path.append(parent[path[-1]])
    return [(x*10,y*10) for x,y in reversed(path)]+[target],'路径有效'
   for dx,dy in [(1,0),(0,1),(-1,0),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)]:
    q=(p[0]+dx,p[1]+dy)
    if not self.free(q) or (dx and dy and (not self.free((p[0]+dx,p[1])) or not self.free((p[0],p[1]+dy)))):continue
    if gate:
     ux,uy,station=gate;rx,ry=q[0]*10-self.pose[0],q[1]*10-self.pose[1];u=rx*ux+ry*uy;v=-rx*uy+ry*ux
     if abs(u-station)<=8 and side*v<5:continue
    ng=g+(math.sqrt(2) if dx and dy else 1)
    if ng<cost.get(q,1e9):cost[q]=ng;parent[q]=p;heapq.heappush(heap,(ng+math.hypot(goal[0]-q[0],goal[1]-q[1]),ng,q))
  return [],'场内没有可通路径'
 def plan(self,target,preferred=0):
  dx,dy=target[0]-self.pose[0],target[1]-self.pose[1];distance=math.hypot(dx,dy);ux,uy=dx/max(distance,1),dy/max(distance,1)
  hits=[]
  for u in range(10,int(distance),5):
   if self.cell((self.pose[0]+u*ux,self.pose[1]+u*uy)) in self.blocked:hits.append(u)
   elif hits:break
  if not hits:
   path,why=self.search(target);return path,why,'直达',0
  gate=(ux,uy,(hits[0]+hits[-1])/2);candidates=[]
  for side in (1,-1):
   path,why=self.search(target,gate,side)
   if path:
    length=sum(math.dist(a,b) for a,b in zip(path,path[1:]));candidates.append((length*(.90 if side==preferred else 1),side,path))
  if not candidates:
   path,why=self.search(target)
   return path,why,'重新选路' if path else '无路等待',0
  _,side,path=min(candidates,key=lambda x:x[0]);return path,'左侧绕行' if side==1 else '右侧绕行','左侧绕行' if side==1 else '右侧绕行',side

 def swept_clear(self,mask,distance,escape=False):
  """Check the whole moving body, rather than a single centre ray/cell.

  Escaping may leave an already occupied safety margin, but may never deepen
  an existing overlap or enter another object's margin. No rotation escape.
  """
  dx=int(bool(mask&8))-int(bool(mask&4));dy=int(bool(mask&1))-int(bool(mask&2))
  norm=math.hypot(dx,dy)
  if not norm or mask&48:return False
  dx/=norm;dy/=norm
  pts=[rotate(0,r,-index*2) for index,r in self.local.bins.items()]
  initial=[max(abs(x)-12,abs(y)-19) for x,y in pts]
  if not escape and any(v<=0 for v in initial):return False
  for step in range(1,math.ceil(distance)+1):
   distance_step=min(step,distance);sx=dx*distance_step;sy=dy*distance_step
   wx,wy=rotate(sx,sy,self.pose[2]);x,y=self.pose[0]+wx,self.pose[1]+wy
   mx=abs(math.cos(math.radians(self.pose[2])))*12+abs(math.sin(math.radians(self.pose[2])))*19
   my=abs(math.sin(math.radians(self.pose[2])))*12+abs(math.cos(math.radians(self.pose[2])))*19
   if not mx<=x<=self.frame.width-mx or not my<=y<=self.frame.height-my:return False
   for (px,py),before in zip(pts,initial):
    after=max(abs(px-sx)-12,abs(py-sy)-19)
    if before>0 and after<=0:return False
    if before<=0 and (not escape or after<before-.05):return False
  if escape and any(before<=0 and max(abs(px-dx*distance)-12,abs(py-dy*distance)-19)<before+.5 for (px,py),before in zip(pts,initial)):return False
  # Require a fresh visible return beyond the exit in the movement direction.
  a=math.degrees(math.atan2(dx,dy))%360;i=int(a//2)%180
  return all(self.local.bins.get((i+k)%180,0)>distance+20 for k in (-1,0,1))

 def escape_route(self,target,allowed,previous=0,minimum_distance=25):
  """Find a checked exit from the blocked start, then an ordinary route."""
  candidates=[];exits=[]
  for mask in (1,2,4,8,5,9,6,10):
   if not allowed(mask):continue
   dx=int(bool(mask&8))-int(bool(mask&4));dy=int(bool(mask&1))-int(bool(mask&2));norm=math.hypot(dx,dy)
   for distance in (15,25,35,45,55,65):
    if distance<minimum_distance:continue
    wx,wy=rotate(dx*distance/norm,dy*distance/norm,self.pose[2]);exit_point=(self.pose[0]+wx,self.pose[1]+wy)
    if not self.free(self.cell(exit_point)):continue
    if not self.swept_clear(mask,distance,escape=True):break
    tail,why=self.search(target,start_at=exit_point)
    if not tail:
     # A temporarily obscured goal must not trap a car already in a near
     # margin. Exit to checked free space first, then observe/replan there.
     if not self.free(self.cell(self.pose)):
      exits.append((distance+math.dist(exit_point,target)*.05,mask,exit_point,[exit_point]))
     continue
    length=distance+sum(math.dist(a,b) for a,b in zip(tail,tail[1:]))
    candidates.append((length-(12 if mask==previous else 0),mask,exit_point,tail));break
  if not candidates:candidates=exits
  if not candidates:return None
  _,mask,exit_point,tail=min(candidates,key=lambda v:v[0])
  return mask,exit_point,[self.pose[:2],exit_point]+tail

def route_waypoint(path,target):
 if len(path)<2:return target
 first=path[1];direction=(round(first[0]-path[0][0],3),round(first[1]-path[0][1],3))
 for a,b in zip(path[1:],path[2:]):
  if (round(b[0]-a[0],3),round(b[1]-a[1],3))!=direction:return a
 return path[-1]

@dataclass
class FieldSnapshot(Snapshot):
 frame:object=None;route_mode:str='等待建系';calibrating:bool=False;duration:float=.38;tracking_reason:str='';heading_error:float=0

class FieldNavigator:
 def __init__(self):self.tracker=FieldTracker();self.target=None;self.active=False;self.snapshot=FieldSnapshot();self.last_sequence=None;self.occupied={};self.arrivals=0;self.started=0;self.side=0;self.last_stamp=0;self.terminal_reason='';self.terminal_arrived=False;self.correcting=False;self.lost_since=None
 def feedback(self,data,stamp):
  previous=getattr(self,'feedback_stamp',stamp)
  actual=abs(data.get('applied_forward',0))+abs(data.get('applied_left',0))
  if actual and data.get('manual',False):self.applied_time=getattr(self,'applied_time',0)+max(0,min(.2,stamp-previous))
  self.feedback_data=dict(data);self.feedback_stamp=stamp
 def allowed(self,mask,stamp):
  blocked=getattr(self,'feedback_data',{}).get('blocked',0)
  if hasattr(self,'feedback_stamp') and stamp-self.feedback_stamp>.6:return False
  if mask&blocked&15:return False
  return not any(mask&bit and stamp<until for bit,until in getattr(self,'avoid_dirs',{}).items())
 def reset(self):self.__init__()
 def calibrate(self,expected=(330,260)):
  self.stop();self.tracker.request(expected);self.target=None;self.occupied={};self.last_sequence=None;self.side=0;self.terminal_reason='';self.terminal_arrived=False
 def stop(self,reason=None):
  was_active=self.active
  self.active=False;self.snapshot.mask=0;self.snapshot.until=0
  if reason:self.terminal_reason=reason
  elif was_active and not self.terminal_reason:self.terminal_reason='用户停止'
  if self.tracker.pending:self.tracker.pending=False;self.tracker.votes=0
 def select(self,target):
  f=self.tracker.frame
  if not f:raise ValueError('请先建立矩形场地坐标')
  if not all(math.isfinite(v) for v in target) or not 25<=target[0]<=f.width-25 or not 32<=target[1]<=f.height-32:raise ValueError('目标应位于框内并留出车身余量')
  self.stop();self.target=tuple(target);self.side=0;self.arrivals=0;self.terminal_reason='';self.terminal_arrived=False
 def start(self,now):
  if not self.target or not self.snapshot.frame or not self.snapshot.valid or self.snapshot.stable<3 or now-self.snapshot.stamp>.6:raise ValueError('建系或定位未就绪，不能出发')
  self.active=True;self.started=now;self.arrivals=0;self.terminal_reason='';self.terminal_arrived=False;self.correcting=False;self.lost_since=None;self.progress_pose=None;self.progress_stamp=now;self.avoid_dirs={};self.escape_mask=0;self.applied_time=0;self.route_commit=None
 def update(self,points,imu_yaw,stamp,sequence):
  if self.last_sequence is not None and not 0<((sequence-self.last_sequence)&65535)<32768:match=self.tracker.fail('点云重复或倒序')
  else:match=self.tracker.update(points,imu_yaw,stamp);self.last_sequence=sequence
  pose=self.tracker.pose;frame=self.tracker.frame
  snap=FieldSnapshot(pose=pose,valid=match.ok,stable=self.tracker.stable,reason=match.reason,stamp=stamp,sequence=sequence,rms=match.rms,support=match.support,frame=frame,rectangle=self.tracker.candidate,calibrating=self.tracker.pending,tracking_reason=match.reason)
  snap.route_mode='待出发' if frame else '建系中' if self.tracker.pending else '等待建系'
  planner=None
  if match.ok and frame:
   planner=FieldPlanner(points,pose,frame)
   self.occupied={p:t for p,t in self.occupied.items() if p in planner.blocked or not planner.local.visible(p)}
   self.occupied.update({p:stamp for p in planner.blocked});planner.blocked.update(self.occupied)
  if self.active:
   if not match.ok:
    if self.lost_since is None:self.lost_since=stamp
    snap.reason=match.reason+'；停车等待定位恢复';snap.route_mode='定位等待'
    if stamp-self.lost_since>.9:self.stop('持续定位失效，已停止，请检查回波后重新建系');snap.reason=self.terminal_reason
   elif self.tracker.stable<3:snap.reason=f'定位恢复确认{self.tracker.stable}/3，保持停车';snap.route_mode='定位恢复'
   elif stamp-self.started>120:self.stop('任务超过120秒，已停止');snap.reason=self.terminal_reason
   elif math.dist(self.target,pose[:2])<=12:
    self.arrivals+=1;snap.reason='目标附近，等待静止确认';snap.route_mode='到达确认'
    if self.arrivals>=3:self.terminal_arrived=True;self.stop('已到达目标');snap.arrived=True;snap.reason=self.terminal_reason
   else:
    self.lost_since=None
    # A commanded move without scan-matched displacement is not progress.
    # Penalise that direction temporarily, then test a different checked exit.
    progress=getattr(self,'progress_pose',None)
    if progress is None or math.dist(progress,pose[:2])>2:
     self.progress_pose=pose[:2];self.progress_stamp=stamp;self.applied_time=0
    elif self.snapshot.mask and stamp-self.progress_stamp>2.5 and getattr(self,'applied_time',0)>1.2:
     self.avoid_dirs=getattr(self,'avoid_dirs',{})
     for bit in (1,2,4,8):
      if self.snapshot.mask&bit:self.avoid_dirs[bit]=stamp+3
     self.progress_stamp=stamp;self.escape_mask=0;self.applied_time=0;self.route_commit=None
    self.arrivals=0;path,why,mode,side=planner.plan(self.target,self.side);snap.path=path;snap.reason=why;snap.route_mode=mode
    mask=0;duration=.38;error=wrap(pose[2]-frame.heading);snap.heading_error=error
    allowed=lambda keys:self.allowed(keys,stamp)
    if path:
     if side:self.side=side
     waypoint=route_waypoint(path,self.target)
     rx,fy=rotate(waypoint[0]-pose[0],waypoint[1]-pose[1],-pose[2]);scale=max(abs(rx),abs(fy),1)
     mask=(1 if fy>.35*scale else 2 if fy<-.35*scale else 0)|(8 if rx>.35*scale else 4 if rx<-.35*scale else 0)
     alternatives=[mask]
     if mask&3 and mask&12:alternatives+=([mask&3,mask&12] if abs(fy)>=abs(rx) else [mask&12,mask&3])
     velocity=max(25,self.tracker.velocity)
     # Include startup before any measured velocity exists; otherwise a fast
     # first movement can enter the inflation band before the next scan.
     # A corner 10cm away is not permission to travel 32cm past it. Check
     # the actual first leg, and shorten its command rather than vetoing it.
     sweep=min(45,max(80,self.tracker.velocity)*.40,max(12,math.dist(waypoint,pose[:2])))
     mask=next((m for m in alternatives if allowed(m)),0)
     error=wrap(pose[2]-frame.heading)
     snap.heading_error=error
     if abs(error)>8:self.correcting=True
     elif abs(error)<3:self.correcting=False
     if self.correcting:
      # Same immediate blocking policy as Task3: no extra PC rotation cone.
      mask=16 if error<0 else 32
      snap.route_mode='航向纠正';snap.reason=f'先纠正航向，偏差{error:+.1f}°'
     # Renew every scan, rather than forcing a pause on every scan period.
     duration=min(.22,max(.12,abs(error)/60*.55)) if mask in (16,32) else min(.38,max(.12,sweep/max(80,self.tracker.velocity)*.8))
     distance=math.dist(self.target,pose[:2])
     if distance<35 and mask not in (16,32):duration=min(duration,max(.12,(distance-10)/max(35,velocity)*.5))
    if not mask:
     recovery=planner.escape_route(self.target,allowed,getattr(self,'escape_mask',0),max(35,max(80,self.tracker.velocity)*.45))
     if recovery:
      mask,exit_point,escape_path=recovery;self.escape_mask=mask;snap.path=escape_path
      snap.reason='受阻重新选路，先退出近障区再继续原目标';snap.route_mode='退出近障区'
      duration=.32
     else:
      blocked=getattr(self,'feedback_data',{}).get('blocked',0)
      snap.reason=('任务3方向受阻，正在重新选路' if blocked else '当前规划暂无可通出口，正在重新选路');snap.route_mode='重新选路'
    else:self.escape_mask=0
    # Keep the chosen translation until its checked exit/waypoint is reached.
    # A newly occupied exit, Task3 board blocking or stale feedback invalidates it.
    commit=getattr(self,'route_commit',None)
    if commit:
     keys,endpoint,old_path=commit
     remaining=math.dist(pose[:2],endpoint)
     dx=int(bool(keys&8))-int(bool(keys&4));dy=int(bool(keys&1))-int(bool(keys&2));norm=math.hypot(dx,dy)
     bx,by=rotate(endpoint[0]-pose[0],endpoint[1]-pose[1],-pose[2])
     along=(bx*dx+by*dy)/max(norm,1);cross=abs(bx*dy-by*dx)/max(norm,1)
     if remaining>8 and along>3 and cross<12 and allowed(keys) and planner.free(planner.cell(endpoint)):
      mask=keys;duration=.32;snap.path=[pose[:2],endpoint]+old_path
      snap.route_mode='沿选定通道绕行';snap.reason='通道仍有效，保持方向继续行驶'
     else:self.route_commit=None
    if not getattr(self,'route_commit',None) and mask and not mask&48:
     endpoint=exit_point if snap.route_mode=='退出近障区' else waypoint
     if math.dist(pose[:2],endpoint)>15:self.route_commit=(mask,endpoint,list(snap.path[1:]))
    snap.mask=mask;snap.duration=duration;snap.until=stamp+duration if mask else 0
  if not self.active and self.terminal_reason:
   snap.reason=self.terminal_reason;snap.route_mode='已到达' if self.terminal_arrived else '已停止';snap.arrived=self.terminal_arrived
  snap.running=self.active;self.snapshot=snap;self.last_stamp=stamp;return snap

class FieldWorker(NavigationWorker):
 def run(self):
  import queue
  nav=FieldNavigator()
  while not self.closed.is_set():
   try:
    while True:
     op,value=self.commands.get_nowait()
     if op=='reset':nav.reset()
     elif op=='calibrate':nav.calibrate(value)
     elif op=='stop':nav.stop()
     elif op=='target':nav.select(value)
     elif op=='start':nav.start(time.monotonic())
     elif op=='feedback':nav.feedback(*value)
   except queue.Empty:pass
   except Exception as exc:nav.stop();self.events.put(('local_error',str(exc)))
   try:args=self.inputs.get(timeout=.05)
   except queue.Empty:continue
   try:self.events.put(('local_nav',nav.update(*args)))
   except Exception as exc:nav.stop();self.events.put(('local_error','场地定位/规划异常：'+str(exc)))
