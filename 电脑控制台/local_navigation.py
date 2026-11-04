"""Metric local navigation using scan-to-line translation and IMU yaw.

Pure standard-library model. Units are centimetres/degrees. It never opens a
port. Rectangles are optional visual evidence, never a fabricated pose source.
"""
from dataclasses import dataclass,field
import math,heapq,time,threading,queue

def wrap(a):return (a+180)%360-180
def rotate(x,y,a):
 a=math.radians(a);c,s=math.cos(a),math.sin(a);return x*c-y*s,x*s+y*c
def cloud_points(points):
 return [(*rotate(0,r,-a),a,r) for a,r in points if 5<=r<=500]
def world_points(points,pose):
 x,y,yaw=pose
 return [(x+u,y+v) for r,l,*_ in cloud_points(points) for u,v in [rotate(r,l,yaw)]]

@dataclass
class Match:
 ok:bool; x:float=0; y:float=0; rms:float=999; support:int=0; condition:float=0; reason:str=''

class ScanTracker:
 def __init__(self):self.reset()
 def reset(self):
  self.pose=(0.,0.,0.);self.origin_yaw=None;self.lines=[];self.key_pose=(0,0);self.stable=0;self.good_time=0.;self.quality=Match(False,reason='等待雷达和IMU');self.last_time=0.;self.velocity=0.
 def make_lines(self,points,pose):
  data=cloud_points(points);out=[]
  for p,q in zip(data,data[1:]):
   if not 0<q[2]-p[2]<=6:continue
   a=rotate(p[0],p[1],pose[2]);b=rotate(q[0],q[1],pose[2]);dx,dy=b[0]-a[0],b[1]-a[1];length=math.hypot(dx,dy)
   if not 2<=length<=28:continue
   nx,ny=-dy/length,dx/length
   out.append((pose[0]+(a[0]+b[0])/2,pose[1]+(a[1]+b[1])/2,nx,ny))
  return out
 def update(self,points,imu_yaw,now):
  if not math.isfinite(imu_yaw) or len(points)<45:return self.fail('有效点不足，暂停定位')
  if self.origin_yaw is None:
   self.origin_yaw=imu_yaw;self.pose=(0.,0.,0.);self.lines=self.make_lines(points,self.pose);self.last_time=now
   if len(self.lines)<24:self.origin_yaw=None;return self.fail('稳定边缘不足，保持静止')
   self.key_pose=(0,0);self.stable=1;self.good_time=now;self.quality=Match(True,support=len(self.lines),reason='建立局部参考，继续静止等待');return self.quality
  if now-self.last_time>1.0:return self.fail('雷达定位中断，请停车后重置参考')
  yaw=wrap(imu_yaw-self.origin_yaw);pts=[rotate(p[0],p[1],yaw) for p in cloud_points(points)]
  x,y,_=self.pose;oldx,oldy=x,y
  # Use spatial lookup to bound nearest-line work on the UI-independent worker.
  buckets={}
  for line in self.lines:buckets.setdefault((math.floor(line[0]/25),math.floor(line[1]/25)),[]).append(line)
  condition=0;rms=999;support=0
  for iteration in range(7):
   rows=[]
   for u,v in pts:
    px,py=x+u,y+v;ix,iy=math.floor(px/25),math.floor(py/25);best=None;bestd=30**2
    for gx in range(ix-1,ix+2):
     for gy in range(iy-1,iy+2):
      for line in buckets.get((gx,gy),()):
       d=(px-line[0])**2+(py-line[1])**2
       if d<bestd:best,bestd=line,d
    if best:
     a,b,nx,ny=best;error=(px-a)*nx+(py-b)*ny
     if abs(error)<=18:rows.append((abs(error),nx,ny,error))
   # Keep evidence from both wall-normal directions while trimming outliers.
   # Global trimming can drop the less numerous perpendicular wall during a
   # faster scan-to-scan displacement and falsely report a single-wall scene.
   groups=([row for row in rows if abs(row[1])>=abs(row[2])],[row for row in rows if abs(row[1])<abs(row[2])])
   rows=[]
   for group in groups:
    group.sort();rows.extend(group[:max(8,int(len(group)*.8))])
   support=len(rows)
   if support<24:return self.fail('回波匹配不足，暂停并保留原目标')
   aa=sum(v[1]**2 for v in rows);bb=sum(v[1]*v[2] for v in rows);cc=sum(v[2]**2 for v in rows)
   bx=-sum(v[1]*v[3] for v in rows);by=-sum(v[2]*v[3] for v in rows);det=aa*cc-bb*bb;condition=det/max(1,(aa+cc)**2)
   if condition<.025:return self.fail('只有单方向边缘，位置无法可靠确定')
   dx,dy=(bx*cc-by*bb)/det,(by*aa-bx*bb)/det
   if math.hypot(dx,dy)>25:return self.fail('扫描位移不可靠，已暂停')
   x+=dx;y+=dy;rms=math.sqrt(sum(v[3]**2 for v in rows)/support)
   if math.hypot(dx,dy)<.05:break
  dt=max(.02,now-self.last_time);distance=math.hypot(x-oldx,y-oldy)
  if rms>5 or distance>max(12,150*dt):return self.fail('匹配误差或位移过大，已暂停')
  if abs(x)>700 or abs(y)>700:return self.fail('超出局部定位范围，请停车重置参考')
  self.velocity=distance/dt;self.pose=(x,y,yaw);self.last_time=now;self.good_time=now;self.stable=min(5,self.stable+1)
  self.quality=Match(True,x,y,rms,support,condition,'局部定位有效')
  if math.hypot(x-self.key_pose[0],y-self.key_pose[1])>45:
   self.lines=self.make_lines(points,self.pose);self.key_pose=(x,y)
  return self.quality
 def fail(self,reason):
  self.stable=0;self.quality=Match(False,reason=reason);return self.quality

@dataclass
class Rectangle:
 angle:float; xmin:float;xmax:float;ymin:float;ymax:float;coverage:float;corners:int;support:int
 def vertices(self):return [rotate(x,y,-self.angle) for x,y in [(self.xmin,self.ymin),(self.xmax,self.ymin),(self.xmax,self.ymax),(self.xmin,self.ymax)]]

def fit_rectangle(points):
 """Require perimeter coverage and corner evidence, not just four strong peaks."""
 raw=[p[:2] for p in cloud_points(points)];best=None;bestscore=0
 if len(raw)<60:return None
 def search(angles):
  nonlocal best,bestscore
  for angle in angles:
   pts=[rotate(x,y,angle) for x,y in raw];lines=[]
   for axis in (0,1):
    hist={}
    for pt in pts:
     k=round(pt[axis]/4);hist[k]=hist.get(k,0)+1
    sides=[]
    for sign in (-1,1):
     keys=sorted((k for k in hist if k*sign>=4),key=lambda k:hist[k],reverse=True);chosen=[]
     for k in keys:
      if all(abs(k-j)>2 for j in chosen):chosen.append(k)
      if len(chosen)==3:break
     sides.append([(4*k,[pt for pt in pts if abs(pt[axis]-4*k)<=5]) for k in chosen])
    lines.append(sides)
   for xmin,l in lines[0][0]:
    for xmax,r in lines[0][1]:
     w=xmax-xmin
     if not 80<=w<=320:continue
     for ymin,b in lines[1][0]:
      for ymax,t in lines[1][1]:
       h=ymax-ymin
       if not 80<=h<=320:continue
       edges=[([p[1] for p in l if ymin-5<=p[1]<=ymax+5],ymin,ymax),([p[1] for p in r if ymin-5<=p[1]<=ymax+5],ymin,ymax),([p[0] for p in b if xmin-5<=p[0]<=xmax+5],xmin,xmax),([p[0] for p in t if xmin-5<=p[0]<=xmax+5],xmin,xmax)]
       cover=[];valid=True
       for vals,low,high in edges:
        bins={min(7,max(0,int((v-low)/(high-low)*8))) for v in vals}
        ordered=sorted(vals)
        if len(vals)<7 or len(bins)<5 or max(vals)-min(vals)<.60*(high-low) or max((b-a for a,b in zip(ordered,ordered[1:])),default=999)>.22*(high-low):valid=False;break
        cover.append(len(bins)/8)
       if not valid:continue
       # Both adjoining sides must support the same physical corner.
       count=0
       for xi,yi in [(0,0),(1,0),(1,1),(0,1)]:
        vertical=edges[xi][0];horizontal=edges[2+yi][0];xc=xmax if xi else xmin;yc=ymax if yi else ymin
        if min(abs(v-yc) for v in vertical)<=max(15,h*.15) and min(abs(v-xc) for v in horizontal)<=max(15,w*.15):count+=1
       if count<3:continue
       errors=[abs(p[0]-xmin) for p in l if ymin-5<=p[1]<=ymax+5]+[abs(p[0]-xmax) for p in r if ymin-5<=p[1]<=ymax+5]+[abs(p[1]-ymin) for p in b if xmin-5<=p[0]<=xmax+5]+[abs(p[1]-ymax) for p in t if xmin-5<=p[0]<=xmax+5]
       residual=sum(errors)/max(1,len(errors))
       score=w*h*min(cover)*(1+.05*count)/(1+.35*residual)
       if score>bestscore:bestscore=score;best=Rectangle(angle,xmin,xmax,ymin,ymax,min(cover),count,sum(len(e[0]) for e in edges))
 search(range(-44,45,2))
 if best:search([best.angle+i*.5 for i in range(-3,4)])
 return best

class LocalPlanner:
 CELL=10
 def __init__(self,points,pose,known=None):
  self.pose=pose;self.bins={int(a//2)%180:r for a,r in points if 5<=r<=500};self.blocked=set();self.cache={};self.known=known if known is not None else set()
  # Chassis 15cm fore/aft, 8cm sideways plus 10cm margin and grid diagonal.
  for x,y in world_points(points,pose):
   ix,iy=round(x/10),round(y/10)
   for dx in range(-4,5):
    for dy in range(-4,5):
     bx,by=rotate((ix+dx)*10-x,(iy+dy)*10-y,-pose[2])
     if abs(bx)<=25 and abs(by)<=32:self.blocked.add((ix+dx,iy+dy))
 def cell(self,p):return round(p[0]/10),round(p[1]/10)
 def visible(self,p):
  x,y=rotate(p[0]*10-self.pose[0],p[1]*10-self.pose[1],-self.pose[2]);d=math.hypot(x,y)
  if d<8:ok=True
  else:
   a=math.degrees(math.atan2(x,y))%360;i=int(a//2)%180;r0=self.bins.get(i);r1=self.bins.get((i+1)%180)
   ok=r0 is not None and r1 is not None and d+8<min(r0,r1)
  return ok
 def free(self,p):
  if p not in self.cache:self.cache[p]=p not in self.blocked and (p in self.known or self.visible(p))
  return self.cache[p]
 def observe(self):
  ix,iy=self.cell(self.pose);extent=min(50,math.ceil(max(self.bins.values(),default=0)/10))
  for x in range(ix-extent,ix+extent+1):
   for y in range(iy-extent,iy+extent+1):
    p=(x,y)
    if p not in self.blocked and self.visible(p):self.known.add(p)
 def plan(self,target):
  start=self.cell(self.pose);goal=self.cell(target)
  if not self.free(start):return [],'车身周围余量不足，请先遥控移到开阔处'
  if not self.free(goal):return [],'目标处于障碍膨胀区或未知区域，请选择可见空地'
  open_=[(0,0,start)];cost={start:0};parent={};visited=0
  while open_ and visited<12000:
   _,g,node=heapq.heappop(open_)
   if g!=cost.get(node):continue
   visited+=1
   if node==goal:
    path=[node]
    while path[-1]!=start:path.append(parent[path[-1]])
    path.reverse();return [(x*10,y*10) for x,y in path]+[target],'路径有效'
   for dx,dy in [(1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)]:
    nxt=(node[0]+dx,node[1]+dy)
    if not self.free(nxt) or (dx and dy and (not self.free((node[0]+dx,node[1])) or not self.free((node[0],node[1]+dy)))):continue
    ng=g+(math.sqrt(2) if dx and dy else 1)
    if ng<cost.get(nxt,1e30):cost[nxt]=ng;parent[nxt]=node;heapq.heappush(open_,(ng+math.hypot(goal[0]-nxt[0],goal[1]-nxt[1]),ng,nxt))
  return [],'当前可见空地内没有可通路径，等待或更换目标'

@dataclass
class Snapshot:
 pose:tuple=(0.,0.,0.);valid:bool=False;stable:int=0;reason:str='等待扫描';stamp:float=0;sequence:int=-1
 path:list=field(default_factory=list);rectangle:object=None;mask:int=0;until:float=0;arrived:bool=False;rms:float=999;support:int=0;running:bool=False

class Navigator:
 def __init__(self):self.tracker=ScanTracker();self.target=None;self.active=False;self.snapshot=Snapshot();self.arrivals=0;self.rectangle=None;self.last_fit=0;self.started=0;self.known=set();self.occupied={};self.last_sequence=None
 def reset(self):self.__init__()
 def stop(self):self.active=False;self.snapshot.mask=0;self.snapshot.until=0
 def select(self,target):
  if not all(math.isfinite(v) for v in target) or math.hypot(*target)>700:raise ValueError('目标超出局部参考范围')
  self.stop();self.target=tuple(target);self.arrivals=0
 def start(self,now):
  s=self.snapshot
  if not self.target:raise ValueError('先在图上选一个空地目标')
  if not s.valid or s.stable<3 or now-s.stamp>.6:raise ValueError('请静止等待可靠雷达定位，或重置局部参考')
  self.active=True;self.started=now;self.arrivals=0
 def update(self,points,imu_yaw,stamp,sequence):
  if self.last_sequence is not None and not 0<(sequence-self.last_sequence)&65535<32768:
   match=self.tracker.fail('点云序号重复或倒序，已停止')
  else:match=self.tracker.update(points,imu_yaw,stamp);self.last_sequence=sequence
  pose=self.tracker.pose
  if stamp-self.last_fit>2:
   self.rectangle=fit_rectangle(points);self.last_fit=stamp
  snap=Snapshot(pose,match.ok,self.tracker.stable,match.reason,stamp,sequence,rectangle=self.rectangle,rms=match.rms,support=match.support)
  planner=None
  if match.ok:
   planner=LocalPlanner(points,pose,self.known)
   self.occupied={p:t for p,t in self.occupied.items() if stamp-t<2 and (p in planner.blocked or not planner.visible(p))}
   self.occupied.update({p:stamp for p in planner.blocked})
   planner.blocked.update(self.occupied);planner.observe()
  if self.active:
   if not match.ok or self.tracker.stable<3:snap.reason=match.reason+'；已暂停，需重新出发';self.stop()
   elif stamp-self.started>90:snap.reason='任务超过90秒，已停止';self.stop()
   else:
    dist=math.hypot(self.target[0]-pose[0],self.target[1]-pose[1])
    if dist<=12:
     self.arrivals+=1;snap.reason='到达目标，等待静止确认'
     if self.arrivals>=3:snap.arrived=True;snap.reason='已到达目标（12cm容差）';self.stop()
    else:
     self.arrivals=0;path,reason=planner.plan(self.target);snap.path=path;snap.reason=reason
     if path:
      waypoint=next((p for p in path[1:] if math.hypot(p[0]-pose[0],p[1]-pose[1])>=12),self.target)
      rx,fy=rotate(waypoint[0]-pose[0],waypoint[1]-pose[1],-pose[2]);scale=max(abs(rx),abs(fy),.01)
      mask=(1 if fy>.35*scale else 2 if fy<-.35*scale else 0)|(8 if rx>.35*scale else 4 if rx<-.35*scale else 0)
      # Quantised WASD output must itself be collision checked. A straight
      # shortcut between grid waypoints can otherwise cut an inflated corner.
      alternatives=[mask]
      if mask&3 and mask&12:
       alternatives+=([mask&3,mask&12] if abs(fy)>=abs(rx) else [mask&12,mask&3])
      def clear_step(keys):
       dx=int(bool(keys&8))-int(bool(keys&4));dy=int(bool(keys&1))-int(bool(keys&2));norm=max(1,math.hypot(dx,dy))
       for step in (2,4,6,8,10):
        wx,wy=rotate(dx*step/norm,dy*step/norm,pose[2])
        if not planner.free(planner.cell((pose[0]+wx,pose[1]+wy))):return False
       return True
      mask=next((keys for keys in alternatives if clear_step(keys)),0)
      if not mask:snap.reason='即将运动方向的车身通道不够宽，停车重规划'
      if abs(pose[2])>20:snap.reason='朝向偏差超过20°，请停车重新对准';self.stop();mask=0
      elif abs(pose[2])>4:mask=16 if pose[2]<0 else 32
      snap.mask=mask;snap.until=stamp+min(.06 if mask in (16,32) else .18,8/max(35,self.tracker.velocity))
     # No route leaves the task paused with zero output; fresh scans replan.
  snap.running=self.active;self.snapshot=snap;return snap

class NavigationWorker:
 """Single owner of the model; latest scan wins, no stale job backlog."""
 def __init__(self,events):
  self.events=events;self.inputs=queue.Queue(maxsize=1);self.commands=queue.Queue();self.closed=threading.Event();self.thread=None
 def start(self):
  if self.thread is None:self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
 def scan(self,*args):
  self.start()
  try:self.inputs.put_nowait(args)
  except queue.Full:
   try:self.inputs.get_nowait()
   except queue.Empty:pass
   self.inputs.put_nowait(args)
 def command(self,op,value=None):self.commands.put((op,value));self.start()
 def run(self):
  nav=Navigator()
  while not self.closed.is_set():
   try:
    while True:
     op,value=self.commands.get_nowait()
     if op=='reset':nav.reset()
     elif op=='stop':nav.stop()
     elif op=='target':nav.select(value)
     elif op=='start':nav.start(time.monotonic())
   except queue.Empty:pass
   except Exception as exc:nav.stop();self.events.put(('local_error',str(exc)))
   try:args=self.inputs.get(timeout=.05)
   except queue.Empty:continue
   try:self.events.put(('local_nav',nav.update(*args)))
   except Exception as exc:nav.stop();self.events.put(('local_error','定位/规划异常，已停：'+str(exc)))
 def close(self):self.closed.set()
