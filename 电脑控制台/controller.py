"""Fixed-field navigation client, compatible with unchanged 97F4 firmware."""
import argparse,math,time,copy,tkinter as tk
from pathlib import Path
from tkinter import ttk,messagebox
from free_console import App as FreeApp
from legacy_console import BG,SIDE,CARD,LINE,TEXT,MUTED,ACCENT,BLUE,RED,AMBER,SMALL,FONT
from field_navigation import FieldWorker,FieldSnapshot,Field
from local_navigation import rotate
from legacy_console import App as SensorApp
from session_trace import SessionTrace

class App(FreeApp):
 def __init__(self,root,demo=False,enumerate_ports=True):
  self.trace=None;self.last_stop='';self.decision=None;self.pulse_deadline=None;self.decision_received=0.;self.drive_ack_text='等待';self.last_scan_record=0
  self.expected_w=tk.StringVar(root,value='330');self.expected_h=tk.StringVar(root,value='260');self.use_size=tk.BooleanVar(root,value=True);self.field_tabs=[];self.had_frame=False
  super().__init__(root,demo,enumerate_ports)
  self.worker.close();self.worker=FieldWorker(self.events)
  self.root.title('麦轮任务控制台 · 97F4 / R6 · 矩形建系与自动绕行'+(' · 离线演示' if demo else ''))
  self.nav_buttons['navigation'].configure(text='04  场地坐标导航');self.radar_view.set('场地坐标')
  if not demo:
   try:self.trace=SessionTrace(Path(__file__).parent/'运行记录');self.note('自动诊断日志：'+str(self.trace.path))
   except OSError as exc:self.note('诊断日志无法写入：'+str(exc),True)
 def _radar(self):
  super()._radar()
  side=next(w for w in self.pages['radar'].winfo_children() if isinstance(w,ttk.Notebook));side.tab(side.tabs()[1],text='任务4 · 建系绕行')
  manual=self.root.nametowidget(side.tabs()[0]).winfo_children()[0]
  before=next(w for w in manual.winfo_children() if isinstance(w,tk.Label) and str(w.cget('textvariable'))==str(self.variable('held_state')))
  keys=tk.Frame(manual,bg=CARD);keys.pack(fill='x',pady=(0,7),before=before)
  self.key_indicators={};self.key_display_mask=None
  for column,(bit,text) in enumerate([(1,'W ↑'),(4,'A ←'),(2,'S ↓'),(8,'D →')]):
   keys.grid_columnconfigure(column,weight=1,uniform='held_key')
   label=tk.Label(keys,text=text,bg=SIDE,fg=MUTED,font=('Segoe UI',11,'bold'),pady=4,bd=1,relief='solid',takefocus=False)
   label.grid(row=0,column=column,sticky='ew',padx=2,pady=2);self.key_indicators[bit]=label
  for column,(bit,text) in enumerate([(32,'侧键1 · 顺时针'),(16,'侧键2 · 逆时针')]):
   label=tk.Label(keys,text=text,bg=SIDE,fg=MUTED,font=SMALL,pady=4,bd=1,relief='solid',takefocus=False)
   label.grid(row=1,column=column*2,columnspan=2,sticky='ew',padx=2,pady=2);self.key_indicators[bit]=label
  self.value_label(keys,'pressed_keys','当前按键：未按下',font=SMALL,color=ACCENT).grid(row=2,column=0,columnspan=4,sticky='w',pady=(3,0))
  for widget in self.radar_canvas.master.winfo_children()[2].winfo_children():
   if isinstance(widget,ttk.Combobox) and str(widget.cget('textvariable'))==str(self.radar_view):widget.configure(values=['车体雷达','场地坐标'])
 def _navigation(self):
  p=self.pages['navigation'];p.grid_columnconfigure(0,weight=3);p.grid_columnconfigure(1,weight=2);p.grid_rowconfigure(0,weight=1)
  card=self.card(p,'矩形场地与固定目标','静止建系：左下角原点，+X向右、+Y向上。路径遇障自动比较左右绕行。')
  card.grid(row=0,column=0,sticky='nsew',padx=(0,12));c=self._world(card,390);c._local_map=True;c.bind('<Button-1>',lambda e:self.map_click(c,e))
  self.goal_x=tk.StringVar(value='100');self.goal_y=tk.StringVar(value='100');self.points=[]
  panel=tk.Frame(p,bg=BG);panel.grid(row=0,column=1,sticky='nsew');self.local_panel(panel,True)
 def local_panel(self,parent,form=False):
  card=self.card(parent,'任务4 · 矩形场地导航');card.pack(fill='both',expand=True)
  self.value_label(card,'local_state','请先静止建系',font=SMALL,color=BLUE,wraplength=310,justify='left').pack(anchor='w',pady=5)
  self.value_label(card,'local_pose','X —  Y —',font=('Consolas',12),color=ACCENT).pack(anchor='w',pady=3)
  tabs=ttk.Notebook(card,style='Mission.TNotebook');tabs.pack(fill='both',expand=True,pady=(8,0))
  goal=tk.Frame(tabs,bg=CARD,padx=5,pady=6);setup=tk.Frame(tabs,bg=CARD,padx=5,pady=6);tabs.add(goal,text='目标与绕行');tabs.add(setup,text='建系设置');tabs.select(setup);self.field_tabs.append((tabs,goal))
  self.checkbutton(goal,'启用点击即出发',self.click_go,self.arm_click).pack(fill='x')
  if form:
   f=tk.Frame(goal,bg=CARD);f.pack(fill='x')
   for i,(label,var) in enumerate([('目标X / cm',self.goal_x),('目标Y / cm',self.goal_y)]):
    self.label(f,label,size=9,color=MUTED).grid(row=i,column=0,pady=5)
    ttk.Spinbox(f,from_=0,to=600,increment=5,textvariable=var,width=8).grid(row=i,column=1,padx=6)
   self.button(goal,'仅选择目标',lambda:self.goal(False),role='local_goal').pack(fill='x',pady=5)
  self.value_label(goal,'local_target','尚未选择目标',font=SMALL,color=AMBER,wraplength=310).pack(anchor='w',pady=8)
  self.button(goal,'前往目标 / 继续任务',lambda:self.goal(True),role='local_go',primary=True).pack(fill='x',pady=5)
  self.value_label(goal,'route_choice','直达 / 左侧绕行 / 右侧绕行',font=FONT,color=ACCENT,wraplength=300).pack(anchor='w',pady=10)
  self.value_label(goal,'nav_execution','请求：停止 · 板端确认：等待',font=SMALL,color=MUTED,wraplength=300,justify='left').pack(anchor='w',pady=3)
  self.label(goal,'障碍遮挡目标时仍在框内规划。\n会选择可通一侧，绕过后回到原目标。\n确实无路、定位失效时停车。Esc取消。',size=9,color=MUTED,wraplength=300,justify='left').pack(anchor='w',pady=8)
  form_frame=tk.Frame(setup,bg=CARD);form_frame.pack(fill='x',pady=7)
  for i,(label,var) in enumerate([('边长A参考/cm',self.expected_w),('边长B参考/cm',self.expected_h)]):
   self.label(form_frame,label,size=9,color=MUTED).grid(row=i,column=0,pady=5)
   ttk.Spinbox(form_frame,from_=80,to=600,increment=10,textvariable=var,width=7).grid(row=i,column=1,padx=5)
  self.checkbutton(setup,'参考边长匹配（±25%，可交换）',self.use_size).pack(fill='x')
  self.button(setup,'静止建立场地坐标',self.calibrate_field,role='field_calibrate',primary=True).pack(fill='x',pady=7)
  self.label(setup,'车头大致朝场地+Y，保持静止。\n三帧一致后建系；不需要四根柱子。\n允许角点缺口，四条边都须有线段证据。',size=9,color=MUTED,wraplength=300,justify='left').pack(anchor='w',pady=7)
  self.value_label(setup,'frame_evidence','尚未建系',font=SMALL,color=BLUE,wraplength=300,justify='left').pack(anchor='w',pady=5)
 def available(self,role):
  if role=='field_calibrate':
   return not self.local_run and (self.demo or (self.available('manual') and not self.pending and not self.task.get('active')))
  if role in ('local_goal','local_go') and not getattr(self.local,'frame',None):return False
  return super().available(role)
 def calibrate_field(self):
  if not self.available('field_calibrate'):return
  try:
   expected=tuple(float(v.get()) for v in (self.expected_w,self.expected_h)) if self.use_size.get() else None
   if expected and not all(math.isfinite(v) and 80<=v<=600 for v in expected):raise ValueError('边长参考请输入80～600cm')
  except ValueError as exc:messagebox.showerror('场地边长',str(exc));return
  self.stop_local();self.last_stop='';self.local_goal=None;self.click_go.set(False);self.manual_armed.set(False);self.local=FieldSnapshot(reason='静止采集四边框…',calibrating=True);self.had_frame=False
  self._record('calibrate',dict(expected=expected))
  if self.demo:
   w,h=expected or (330,260);self.local=FieldSnapshot(pose=(w/2,h/2,0),valid=True,stable=5,stamp=time.monotonic(),frame=Field(w,h,0),reason='离线演示：场地坐标有效',rms=1,support=100)
  elif self.worker:self.worker.command('calibrate',expected)
  self.note('开始静止建系，保持车不动；不会发起运动。');self.render()
 def reset_local(self):self.calibrate_field()
 def goal(self,go):
  f=getattr(self.local,'frame',None)
  if not f:self.note('先建立场地坐标，再选择框内目标。',True);return
  try:
   x,y=float(self.goal_x.get()),float(self.goal_y.get())
   if not all(math.isfinite(v) for v in (x,y)) or not (25<=x<=f.width-25 and 32<=y<=f.height-32):raise ValueError(f'目标范围：X 25～{f.width-25:.0f}cm，Y 32～{f.height-32:.0f}cm')
  except ValueError as exc:messagebox.showerror('框内目标',str(exc));return
  old_started=self.local_started;old_goal=self.local_goal;super().goal(go)
  if self.local_started!=old_started or self.local_goal!=old_goal:self.last_stop='';self.decision=None;self.pulse_deadline=None;self._record('goal',dict(target=self.local_goal,start=go))
 def _record(self,kind,data):
  if self.trace:
   try:self.trace.write(kind,data)
   except OSError:self.trace.close();self.trace=None
 def note(self,text,error=False):
  self._record('message',dict(text=str(text),error=error))
  if str(text).startswith('DBG '):return
  super().note(text,error)
 def stop_local(self,reason=None):
  was_running=self.local_run;super().stop_local();self.decision=None;self.pulse_deadline=None
  if reason:self.last_stop=reason
  elif was_running and not self.last_stop:self.last_stop='任务已取消，等待再次出发'
  if was_running or reason:self._record('stop',dict(reason=self.last_stop,target=self.local_goal))
 def stop(self):
  self.stop_local('用户停止');super().stop()
 def show(self,key):
  # Looking at telemetry/logs or switching windows must not cancel autonomy.
  # SensorApp still disarms manual keyboard control outside the radar page.
  SensorApp.show(self,key)
 def on_link_lost(self):
  if self.local_run:self.last_stop='无线连接中断，任务已停止'
  super().on_link_lost()
 def _poll_drive_once(self):
  self.update_key_display()
  if not self.local_run:return super()._poll_drive_once()
  now=time.monotonic();s=self.local
  if getattr(self.held_input,'emergency_stop',lambda:False)():self.stop();return
  if not self.available('manual') or now-s.stamp>.6:
   self.stop_local('实时遥测、扫描或IMU条件失效，已停止');return
  if self.pending or self.speed_focus_pending:self.link.set_drive(0);return
  d=self.decision;mask=0
  if d and s.valid and d.stamp>=self.local_started and now-d.stamp<.55:
   if self.pulse_deadline is None:
    # Permit the first write, but do not wait indefinitely for serial output.
    if now-self.decision_received<.20:mask=d.mask
   elif now<self.pulse_deadline:mask=d.mask
  # Do not let the FPGA silently remove one component of a diagonal.
  # Release the entire blocked request; the worker selects a free route.
  if mask & self.task.get('blocked',0) & 15:mask=0
  self.link.set_drive(mask)
  names=self.mask_name(mask);t=self.task
  blocks=self.mask_name(t.get('blocked',0)&15) if t.get('blocked',0)&15 else '无'
  self.variable('nav_execution').set(f'请求：{names} · 板端确认：{self.drive_ack_text}\n任务3方向阻挡：{blocks}\n实际目标 F{t.get("applied_forward",0)} L{t.get("applied_left",0)} U{t.get("applied_turn",0)}')
 def update_key_display(self):
  if not hasattr(self,'key_indicators') or not hasattr(self,'held_input'):return
  mask=self.held_input.mask()
  if mask==self.key_display_mask:return
  self.key_display_mask=mask
  for bit,label in self.key_indicators.items():label.configure(bg=ACCENT if mask&bit else SIDE,fg=BG if mask&bit else MUTED)
  names=[name for bit,name in [(1,'W'),(4,'A'),(2,'S'),(8,'D'),(32,'侧键1'),(16,'侧键2')] if mask&bit]
  self.variable('pressed_keys').set('当前按键：'+(' + '.join(names) if names else '未按下'))
 def mask_name(self,mask):
  return '+'.join(n for i,n in enumerate(['前','后','左','右','逆转','顺转']) if mask&(1<<i)) or '停止'
 def ingest(self,kind,value):
  now=time.monotonic()
  if kind in ('drive_sent','drive_ack'):
   self._record(kind,value)
   if kind=='drive_sent' and self.local_run and self.decision and value['mask']==self.decision.mask and value['mask'] and self.pulse_deadline is None and now-self.decision.stamp<.55:
    duration=max(.12,min(.38,getattr(self.decision,'duration',self.decision.until-self.decision.stamp)))
    self.pulse_deadline=min(value['time']+duration,self.decision.stamp+.55)
   if kind=='drive_ack':
    self.drive_ack_text='已接受' if value['status']==0 else f'拒绝{value["status"]}'
    if self.local_run and value['mask'] and value['status']!=0:self.stop_local('FPGA拒绝运动指令：'+str(value['status']))
   return
  if kind in ('task','attitude','wheels','pid','sensors','local_nav','local_error'):self._record(kind,value)
  if kind=='scan' and now-self.last_scan_record>=1:self._record('scan',value);self.last_scan_record=now
  super().ingest(kind,value)
  if kind=='task' and self.worker:self.worker.command('feedback',(dict(value),now))
  if kind=='local_nav':
   if self.local is not value:return
   if self.local_run and value.running and value.valid and value.stamp>=self.local_started and now-value.stamp<.35:
    self.decision=value;self.decision_received=now;self.pulse_deadline=None
   else:self.decision=None;self.pulse_deadline=None
   if not value.running and value.route_mode in ('已停止','已到达'):
    if not self.last_stop or self.last_stop=='任务已取消，等待再次出发' or value.arrived:self.last_stop=value.reason
  elif kind=='local_error':self.last_stop=str(value)
 def arm_click(self):
  if self.click_go.get() and not self.available('local_go'):self.click_go.set(False);self.note('先静止建立场地坐标，等待定位有效。',True)
  elif self.click_go.get():self.manual_armed.set(False);self.link.set_drive(0);self.note('点击框内目标即出发，途中自动选择左右绕行。')
 def render(self):
  super().render();s=self.local;f=getattr(s,'frame',None)
  self.variable('frame_evidence').set(f'已建系 {f.width:.0f} × {f.height:.0f} cm\n左下原点；请核对蓝色框是否对应实物' if f else s.reason)
  self.variable('route_choice').set(self.last_stop if self.last_stop and not self.local_run else getattr(s,'route_mode','等待建系'))
  if getattr(s,'tracking_reason',''):
   self.variable('local_state').set(s.tracking_reason+f' · 匹配{s.support}点 / 残差{s.rms:.1f}cm' if s.valid else s.tracking_reason)
  self.variable('box').set(f'场地 {f.width:.0f}×{f.height:.0f} cm' if f else '等待矩形建系')
  self.variable('nav_flags').set('场地固定坐标 · 自动左右绕行' if f else '请先静止建立场地坐标')
  if f and not self.had_frame:
   self.had_frame=True
   for tabs,goal in self.field_tabs:tabs.select(goal)
 def draw_world(self,c):
  f=getattr(self.local,'frame',None);body=getattr(c,'_body_radar',False) and self.radar_view.get()=='车体雷达'
  if not f or body:
   old=self.local;self.local=copy.copy(old);self.local.rectangle=None
   try:super().draw_world(c)
   finally:self.local=old
   if not f:
    c.create_text(max(c.winfo_width(),100)/2,40,text='请先静止建立矩形场地坐标',fill=AMBER,font=SMALL);return
   cx,cy,scale,_=c._local_transform
   def xy(x,y):
    x,y=rotate(x-self.local.pose[0],y-self.local.pose[1],-self.local.pose[2]);return cx+x*scale,cy-y*scale
   vertices=[xy(x,y) for x,y in [(0,0),(f.width,0),(f.width,f.height),(0,f.height),(0,0)]]
   c.create_line(*[v for p in vertices for v in p],fill=BLUE,width=2,tags='field_frame');return
  c.delete('all');w=max(100,c.winfo_width());h=max(100,c.winfo_height());scale=max(.05,min((w-84)/f.width,(h-84)/f.height));ox=(w-f.width*scale)/2;oy=(h+f.height*scale)/2
  c._local_transform=(ox,oy,scale,False);c._body_display=False
  def xy(x,y):return ox+x*scale,oy-y*scale
  for x in range(0,int(f.width)+1,50):
   a,b=xy(x,0);u,v=xy(x,f.height);c.create_line(a,b,u,v,fill=LINE);c.create_text(a,b+15,text=str(x),fill=MUTED,font=('Consolas',8))
  for y in range(0,int(f.height)+1,50):
   a,b=xy(0,y);u,v=xy(f.width,y);c.create_line(a,b,u,v,fill=LINE);c.create_text(a-20,b,text=str(y),fill=MUTED,font=('Consolas',8))
  a,b=xy(0,0);u,v=xy(f.width,f.height);c.create_rectangle(a,v,u,b,outline=BLUE,width=2,tags='field_frame')
  pose=self.local.pose
  if self.cloud and time.monotonic()-self.cloud_time<.7:
   for angle,r in self.cloud['points']:
    x,y=rotate(0,r,pose[2]-angle);x+=pose[0];y+=pose[1]
    if -10<=x<=f.width+10 and -10<=y<=f.height+10:
     sx,sy=xy(x,y);c.create_oval(sx-2,sy-2,sx+2,sy+2,fill=ACCENT,outline='')
  if self.local.path:c.create_line(*[v for p in self.local.path for v in xy(*p)],fill=AMBER,width=2,tags='field_path')
  if self.local_goal:
   sx,sy=xy(*self.local_goal);c.create_line(sx-7,sy,sx+7,sy,fill=RED,width=2);c.create_line(sx,sy-7,sx,sy+7,fill=RED,width=2);c.create_text(sx,sy-16,text='目标',fill=RED,font=SMALL)
  sx,sy=xy(*pose[:2]);dx,dy=rotate(0,20,pose[2]);c.create_line(sx,sy,sx+dx*scale,sy-dy*scale,fill=BLUE,width=4,arrow='last')
  c.create_text(w/2,15,text=f'场地 {f.width:.0f}×{f.height:.0f}cm · 左下原点 · +X右 / +Y上',fill=MUTED,font=SMALL)
 def seed_demo(self):
  super().seed_demo();self.local=FieldSnapshot(pose=(165,130,0),valid=True,stable=5,stamp=time.monotonic(),frame=Field(330,260,0),reason='离线演示：场地坐标有效',rms=1,support=100);self.render()
 def seed_cloud(self):
  f=getattr(self.local,'frame',None) or Field(330,260);pose=self.local.pose if getattr(self.local,'frame',None) else (165,130,0);bins=[]
  for angle in range(0,360,2):
   dx,dy=rotate(0,1,pose[2]-angle);distances=[]
   for edge,coord,d in [(0,pose[0],dx),(f.width,pose[0],dx),(0,pose[1],dy),(f.height,pose[1],dy)]:
    if abs(d)>1e-9 and (edge-coord)/d>0:distances.append((edge-coord)/d)
   bins.append(max(1,min(255,round(min(distances)/2))))
  self.cloud=dict(sequence=1,bins=bins,points=[(i*2,v*2) for i,v in enumerate(bins)]);self.cloud_time=time.monotonic()
 def close(self):
  self.stop_local('控制台关闭');super().close()
  if self.trace:self.trace.close()

def main():
 p=argparse.ArgumentParser();p.add_argument('--demo',action='store_true');p.add_argument('--smoke',action='store_true');args=p.parse_args()
 root=tk.Tk();app=App(root,args.demo,not args.smoke and not args.demo)
 if args.smoke:
  root.withdraw();root.update_idletasks()
  for key in app.pages:app.show(key);root.update_idletasks()
  assert app.link.thread is None and app.worker.thread is None
  for ident in root.tk.splitlist(root.tk.call('after','info')):root.after_cancel(ident)
  app.worker.close();root.destroy();print('PASS field GUI: six pages, no serial or automatic movement')
 else:root.mainloop()
if __name__=='__main__':main()
