"""97F4: explicit PC point-to-point navigation, no rectangle calibration gate."""
import argparse,math,time,tkinter as tk
from tkinter import ttk,messagebox
from legacy_console import App as LegacyApp,BG,SIDE,CARD,LINE,TEXT,MUTED,ACCENT,BLUE,RED,AMBER,SMALL,FONT
from protocol import *
from local_navigation import NavigationWorker,Snapshot,rotate

class App(LegacyApp):
 def __init__(self,root,demo=False,enumerate_ports=True):
  self.local=Snapshot();self.local_run=False;self.local_goal=None;self.local_started=0;self.last_att_time=0;self.worker=None
  self.local_log=[];self.show_frame=tk.BooleanVar(root,value=True)
  super().__init__(root,demo,enumerate_ports)
  self.worker=NavigationWorker(self.events)
  self.root.title('麦轮任务控制台 · 97F4 · 自由点选导航'+(' · 离线演示' if demo else ''))
  self.nav_buttons['navigation'].configure(text='04  自由点选导航')
 def _radar(self):
  super()._radar()
  side=next(w for w in self.pages['radar'].winfo_children() if isinstance(w,ttk.Notebook))
  nav=self.root.nametowidget(side.tabs()[1]);side.tab(nav,text='任务4 · 自由导航')
  removed={b for b,_ in self.actions if str(b).startswith(str(nav)+'.')}
  self.actions=[(b,r) for b,r in self.actions if b not in removed]
  for child in nav.winfo_children():child.destroy()
  self.local_panel(nav)
  toolbar=self.radar_canvas.master.winfo_children()[2]
  for widget in toolbar.winfo_children():
   if isinstance(widget,ttk.Combobox) and str(widget.cget('textvariable'))==str(self.radar_view):widget.configure(values=['车体雷达','局部地图'])
 def _navigation(self):
  p=self.pages['navigation'];p.grid_columnconfigure(0,weight=3);p.grid_columnconfigure(1,weight=2);p.grid_rowconfigure(0,weight=1)
  c=self.card(p,'局部地图与固定目标','无需柱子或四边框。参考原点为建立局部参考时的小车，向右+X、向前+Y。')
  c.grid(row=0,column=0,sticky='nsew',padx=(0,12));canvas=self._world(c,390);canvas._local_map=True;canvas.bind('<Button-1>',lambda e:self.map_click(canvas,e))
  self.goal_x=tk.StringVar(value='0');self.goal_y=tk.StringVar(value='100');self.points=[]
  right=tk.Frame(p,bg=BG);right.grid(row=0,column=1,sticky='nsew');self.local_panel(right,form=True)
 def local_panel(self,parent,form=False):
  card=self.card(parent,'任务4 · 免建系到点','电脑持续连接，点击空地设固定目标。');card.pack(fill='both',expand=True)
  self.value_label(card,'local_state','等待定位',font=SMALL,color=BLUE,wraplength=310,justify='left').pack(anchor='w',pady=8)
  self.value_label(card,'local_pose','X —  Y —',font=('Consolas',12),color=ACCENT).pack(anchor='w')
  self.checkbutton(card,'启用点击即出发',self.click_go,self.arm_click).pack(fill='x')
  if form:
   f=tk.Frame(card,bg=CARD);f.pack(fill='x',pady=6)
   for i,(name,var) in enumerate([('目标X / cm',self.goal_x),('目标Y / cm',self.goal_y)]):
    self.label(f,name,size=9,color=MUTED).grid(row=i,column=0,pady=5)
    ttk.Spinbox(f,from_=-700,to=700,increment=5,textvariable=var,width=8).grid(row=i,column=1,padx=6)
   self.button(card,'仅选择目标',lambda:self.goal(False),role='local_goal').pack(fill='x',pady=4)
  self.value_label(card,'local_target','尚未选择目标',font=SMALL,color=AMBER,wraplength=310).pack(anchor='w',pady=7)
  self.button(card,'前往已选目标',lambda:self.goal(True),role='local_go',primary=True).pack(fill='x',pady=4)
  self.button(card,'停车后重置局部参考',self.reset_local,role='local_reset').pack(fill='x',pady=(7,3))
  self.checkbutton(card,'显示候选边框（可选）',self.show_frame,self.redraw_worlds).pack(fill='x')
  self.value_label(card,'frame_evidence','边框只作辅助，不是出发条件',font=SMALL,color=MUTED,wraplength=310,justify='left').pack(anchor='w',pady=3)
  self.label(card,'首次连接静止约1秒；无需校准边框。\nEsc停止。定位丢失后需重新出发。',size=9,color=MUTED,wraplength=310,justify='left').pack(anchor='w',pady=6)
 def available(self,role):
  if role in ('local_goal','local_go','local_reset'):
   if self.demo:return not self.local_run
   ready=self.available('manual') and not self.pending and not self.local_run
   if role=='local_reset':return ready and not self.task.get('active')
   return ready and self.local.valid and self.local.stable>=3 and time.monotonic()-self.local.stamp<.6
  if self.local_run and role not in ('manual',):return False
  return super().available(role)
 def arm_click(self):
  if self.click_go.get() and not self.available('local_go'):
   self.click_go.set(False);self.note('先连接97F4，保持静止等待有效局部定位。',True)
  elif self.click_go.get():self.manual_armed.set(False);self.link.set_drive(0);self.note('点击即出发已启用；请点击可见空地。')
 def reset_local(self):
  if not self.available('local_reset'):return
  self.stop_local();self.local=Snapshot();self.local_goal=None
  if self.worker:self.worker.command('reset')
  self.note('局部参考已重置，保持静止等待新扫描。');self.render()
 def goal(self,go):
  if not self.available('local_go' if go else 'local_goal'):
   self.note('无线、雷达或局部定位尚未就绪，请保持静止。',True);return
  try:
   x,y=float(self.goal_x.get()),float(self.goal_y.get())
   if not all(math.isfinite(v) and abs(v)<=700 for v in (x,y)):raise ValueError('目标请输入±700cm内的有效坐标')
  except ValueError as exc:messagebox.showerror('局部目标',str(exc));return
  self.stop_local();self.local_goal=(x,y)
  if self.worker:self.worker.command('target',self.local_goal)
  self.manual_armed.set(False);self.link.set_drive(0)
  if go:
   self.local_started=time.monotonic();self.local_run=True
   if self.worker and not self.demo:self.worker.command('start')
   self.note('开始场地导航：目标固定，自动比较左右绕行。')
  else:self.note('目标已选中，尚未出发。')
  self.render()
 def map_click(self,c,event):
  if not self.available('local_goal'):self.note('等待局部定位就绪后再选目标。',True);return
  if not hasattr(c,'_local_transform'):return
  cx,cy,scale,body=c._local_transform
  x,y=(event.x-cx)/scale,(cy-event.y)/scale
  if body:
   x,y=rotate(x,y,self.local.pose[2]);x+=self.local.pose[0];y+=self.local.pose[1]
  self.goal_x.set(str(round(x/5)*5));self.goal_y.set(str(round(y/5)*5));self.goal(self.click_go.get())
 def stop_local(self):
  self.local_run=False;self.local_started=0;self.link.set_drive(0)
  if self.worker:self.worker.command('stop')
 def stop(self):self.stop_local();super().stop()
 def arm_manual(self):self.stop_local();super().arm_manual()
 def show(self,key):
  if self.local_run and key not in ('radar','navigation'):self.stop_local()
  super().show(key)
 def apply_speed(self):self.stop_local();super().apply_speed()
 def apply_obstacle(self):self.stop_local();super().apply_obstacle()
 def send(self,commands):
  if any(op in (CALIBRATE,GOAL,GO,POINT,ROUTE) for op,*_ in commands):
   self.note('自由导航由电脑规划，不发送旧版边框导航指令。',True);return
  if self.local_run:self.stop_local()
  super().send(commands)
 def on_link_lost(self):
  self.stop_local();self.local=Snapshot();self.local_goal=None
  if self.worker:self.worker.command('reset')
  super().on_link_lost()
 def _poll_drive_once(self):
  if not self.local_run:return super()._poll_drive_once()
  now=time.monotonic();s=self.local
  ready=self.available('manual') and not self.pending and not self.speed_focus_pending
  if not ready or not self.held_input.foreground() or now-s.stamp>.6:
   self.stop_local();self.note('场地导航已停：窗口失焦或实时定位/连接条件失效。',True);return
  mask=s.mask if s.valid and s.stamp>=self.local_started and now<s.until else 0
  self.link.set_drive(mask)
  self.variable('held_state').set('电脑正在执行到点导航；手动遥控未启用')
 def ingest(self,kind,value):
  if kind=='local_nav':
   if value.stamp>=self.local.stamp:
    self.local=value
    if self.local_run and value.stamp>=self.local_started and not value.running:
     self.stop_local();self.note(value.reason,not value.arrived)
    self.local_log.append(dict(time=value.stamp,pose=value.pose,valid=value.valid,reason=value.reason,mask=value.mask,rms=value.rms));self.local_log=self.local_log[-2000:]
   return
  if kind=='local_error':self.stop_local();self.note(value,True);return
  if kind=='attitude':self.last_att_time=time.monotonic()
  super().ingest(kind,value)
  if kind=='scan' and self.worker and not self.demo and self.connected and self.remote_ready:
   if self.att.get('ready') and time.monotonic()-self.last_att_time<.6:
    self.worker.scan(value['points'],self.att['Y'],self.cloud_time,value['sequence'])
 def render(self):
  super().render();s=self.local
  self.variable('local_state').set(s.reason+f' · 匹配{s.support}点 / 残差{s.rms:.1f}cm' if s.valid else s.reason)
  self.variable('local_pose').set(f'X {s.pose[0]:+.1f}  Y {s.pose[1]:+.1f} cm' if s.valid else 'X —  Y —')
  self.variable('position').set(f'{s.pose[0]:+.1f}, {s.pose[1]:+.1f}' if s.valid else '等待局部定位')
  self.variable('local_target').set(f'目标 ({self.local_goal[0]:g}, {self.local_goal[1]:g}) cm' if self.local_goal else '尚未选择目标')
  r=s.rectangle
  self.variable('frame_evidence').set(f'候选框 {r.xmax-r.xmin:.0f}×{r.ymax-r.ymin:.0f}cm · 至少{r.coverage:.0%}边长分段覆盖 · {r.corners}/4角点\n仅辅助观察，请核对实物。' if r else '未确认连续闭合四边；不影响局部导航')
  self.variable('box').set('局部参考 · 无边框出发门槛')
  self.variable('nav_flags').set('电脑雷达配准定位 · FPGA轮速闭环')
  self.variable('nav_reason').set(s.reason)
  if self.local_run:self.variable('task_state').set('自由导航中')
 def radar_tab_changed(self,tabs,nav_tab):
  self.stop_local();super().radar_tab_changed(tabs,nav_tab)
 def draw_world(self,c):
  c.delete('all');w=max(100,c.winfo_width());h=max(100,c.winfo_height());cx,cy=w/2,h/2
  radius=int(self.radar_radius.get()) if self.radar_radius.get() in ('100','200','300','500') else 300
  scale=max(.05,min(w,h)/2-40)/radius
  body=getattr(c,'_body_radar',False) and self.radar_view.get()=='车体雷达'
  c._local_transform=(cx,cy,scale,body);c._body_display=body
  pose=self.local.pose
  def xy(x,y,world=True):
   if body and world:x,y=rotate(x-pose[0],y-pose[1],-pose[2])
   return cx+x*scale,cy-y*scale
  step=50 if radius<=300 else 100
  for d in range(step,radius+1,step):
   c.create_oval(cx-d*scale,cy-d*scale,cx+d*scale,cy+d*scale,outline=LINE)
   c.create_text(cx+d*scale,cy+12,text=str(d),fill=MUTED,font=('Consolas',8))
  c.create_line(cx-radius*scale,cy,cx+radius*scale,cy,fill=LINE);c.create_line(cx,cy-radius*scale,cx,cy+radius*scale,fill=LINE)
  fresh=self.cloud and time.monotonic()-self.cloud_time<.7
  if fresh:
   for angle,distance in self.cloud['points']:
    x,y=rotate(0,distance,-angle)
    if not body:x,y=rotate(x,y,pose[2]);x+=pose[0];y+=pose[1]
    if math.hypot(x,y)>radius:continue
    sx,sy=xy(x,y,world=False);c.create_oval(sx-2,sy-2,sx+2,sy+2,fill=ACCENT,outline='')
  if body:
   for bit,poly in self.obstacle_polygons():
    coords=[]
    for forward,left in poly:coords.extend(xy(-left,forward,False))
    color=RED if self.task.get('blocked',0)&(1<<bit) else AMBER
    c.create_polygon(*coords,outline=color,fill=color,stipple='gray25',tags='obstacle_zone')
  if self.show_frame.get() and fresh and self.local.rectangle:
   vertices=[]
   for x,y in self.local.rectangle.vertices():
    x,y=rotate(x,y,pose[2]);vertices.append(xy(x+pose[0],y+pose[1]))
   c.create_line(*[v for p in vertices+[vertices[0]] for v in p],fill=BLUE,width=2,dash=(6,4),tags='candidate_frame')
  if self.local.path:
   coords=[v for p in self.local.path for v in xy(*p)]
   if len(coords)>=4:c.create_line(*coords,fill=AMBER,width=2,tags='local_path')
  if self.local_goal:
   sx,sy=xy(*self.local_goal);c.create_line(sx-7,sy,sx+7,sy,fill=RED,width=2);c.create_line(sx,sy-7,sx,sy+7,fill=RED,width=2)
   c.create_text(sx,sy-15,text='固定目标',fill=RED,font=SMALL)
  sx,sy=xy(pose[0],pose[1]);dx,dy=rotate(0,20,0 if body else pose[2]);c.create_line(sx,sy,sx+dx*scale,sy-dy*scale,fill=BLUE,width=4,arrow='last')
  c.create_text(cx,15,text=('车体雷达 · 车头↑' if body else '局部地图 · 原点=参考建立时的小车')+f' · {radius}cm半径',fill=MUTED,font=SMALL)
  c.create_text(cx,h-15,text='点空地设目标 · 橙线为规划路径 · 蓝虚线为候选边框',fill=MUTED,font=SMALL)
 def seed_demo(self):
  super().seed_demo();self.local=Snapshot((0,0,0),True,5,'离线演示：局部定位',time.monotonic(),1,rms=1,support=100);self.render()
 def close(self):
  self.stop_local()
  if self.worker:self.worker.close()
  super().close()

def main():
 p=argparse.ArgumentParser();p.add_argument('--demo',action='store_true');p.add_argument('--smoke',action='store_true');args=p.parse_args()
 root=tk.Tk();a=App(root,args.demo,not args.smoke and not args.demo)
 if args.smoke:
  root.withdraw();root.update_idletasks()
  for key in a.pages:a.show(key);root.update_idletasks()
  assert a.link.thread is None and a.worker.thread is None
  for ident in root.tk.splitlist(root.tk.call('after','info')):root.after_cancel(ident)
  a.worker.close();root.destroy();print('PASS GUI: six pages, no serial, no navigation worker or automatic movement')
 else:root.mainloop()
if __name__=='__main__':main()
