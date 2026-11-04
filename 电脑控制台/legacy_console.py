"""Local desktop console. No automatic serial connection or vehicle movement."""
import argparse,math,queue,time,tkinter as tk
from tkinter import ttk,messagebox,filedialog
from collections import deque
from datetime import datetime
from pathlib import Path
from protocol import *
from transport import Link
from inputs import HeldInput

BG='#0B1220';SIDE='#101A2C';CARD='#152238';LINE='#283952';TEXT='#ECF3FF';MUTED='#99AAC4'
ACCENT='#48DFC1';BLUE='#62ACFF';RED='#FF6978';AMBER='#F2C36A';PURPLE='#B4A2FF'
FONT=('Microsoft YaHei UI',10);SMALL=('Microsoft YaHei UI',9);MONO=('Consolas',17,'bold')
WHEEL_NAMES={'LF':'左前','RF':'右前','LB':'左后','RB':'右后'}
WHEEL_COLORS={'LF':ACCENT,'RF':BLUE,'LB':AMBER,'RB':PURPLE}

class BaseApp:
    def __init__(self,root,demo=False,enumerate_ports=True):
        self.root=root;self.demo=demo;self.events=queue.Queue();self.link=Link(self.events)
        self.connected=False;self.remote_ready=False;self.pending=False;self.task={};self.att={};self.wheels={};self.sensors={};self.imu={};self.pid={}
        self.last_task=0;self.last_rx=0;self.current='overview';self.actions=[];self.vars={};self.logs=deque(maxlen=1200)
        self.history={k:deque(maxlen=90) for k in WHEEL_NAMES};self.worlds=[];self.motion_names=['前进4秒','后退4秒','左横移4秒','右横移4秒','原地左转90°','原地右转90°']
        self.demo_started=time.monotonic();self.demo_until=0
        root.title('麦轮任务控制台 · 97F4'+(' · 离线演示' if demo else ''))
        root.geometry('1280x860');root.minsize(1120,760);root.configure(bg=BG)
        style=ttk.Style(root);style.theme_use('clam')
        style.configure('TCombobox',fieldbackground=SIDE,background=LINE,foreground=TEXT,arrowcolor=MUTED,bordercolor=LINE,selectbackground=LINE,selectforeground=TEXT,padding=5)
        style.map('TCombobox',fieldbackground=[('readonly',SIDE)],foreground=[('readonly',TEXT)])
        style.configure('TSpinbox',fieldbackground=SIDE,background=LINE,foreground=TEXT,arrowcolor=MUTED,bordercolor=LINE,padding=5)
        style.configure('Mission.TNotebook',background=BG,borderwidth=0)
        style.configure('Mission.TNotebook.Tab',background=SIDE,foreground=MUTED,padding=(16,10),font=FONT)
        style.map('Mission.TNotebook.Tab',background=[('selected',CARD)],foreground=[('selected',ACCENT)])
        root.option_add('*TCombobox*Listbox.background',SIDE);root.option_add('*TCombobox*Listbox.foreground',TEXT)
        root.grid_rowconfigure(1,weight=1);root.grid_columnconfigure(1,weight=1)
        self._header();self._sidebar();self._body()
        self.footer=tk.Label(root,text='离线演示 · 画面使用模拟数据，不打开串口' if demo else '选择逐飞无线USB端，连接并握手后再选择任务。',bg=SIDE,fg=MUTED,font=SMALL,anchor='w',padx=18,pady=8)
        self.footer.grid(row=2,column=0,columnspan=2,sticky='ew')
        root.bind('<Escape>',lambda event:self.stop());root.protocol('WM_DELETE_WINDOW',self.close)
        if demo:self.seed_demo();root.after(200,self.demo_tick)
        elif enumerate_ports:self.refresh_ports()
        self.show('overview');self.update_controls();root.after(60,self.poll)
        self.note('离线演示已开启，所有数值为模拟数据。' if demo else '程序已启动，未连接设备。')

    def label(self,parent,text='',size=10,color=TEXT,bold=False,**kw):
        return tk.Label(parent,text=text,bg=parent.cget('bg'),fg=color,font=('Microsoft YaHei UI',size,'bold' if bold else 'normal'),**kw)
    def variable(self,key,default='—'):
        if key not in self.vars:self.vars[key]=tk.StringVar(value=default)
        return self.vars[key]
    def value_label(self,parent,key,default='—',font=MONO,color=TEXT,**kw):
        return tk.Label(parent,textvariable=self.variable(key,default),bg=parent.cget('bg'),fg=color,font=font,**kw)
    def card(self,parent,title=None,subtitle=None):
        frame=tk.Frame(parent,bg=CARD,highlightbackground=LINE,highlightthickness=1,padx=16,pady=13)
        if title:self.label(frame,title,size=11,bold=True).pack(anchor='w')
        if subtitle:self.label(frame,subtitle,size=9,color=MUTED,wraplength=600,justify='left').pack(anchor='w',pady=(3,9))
        return frame
    def button(self,parent,text,command,role=None,primary=False,danger=False):
        color=RED if danger else ACCENT if primary else LINE
        b=tk.Button(parent,text=text,command=command,bg=color,fg=BG if primary else TEXT,activebackground=color,activeforeground=TEXT,
                    disabledforeground='#67758A',relief='flat',bd=0,padx=13,pady=9,font=FONT,cursor='hand2',highlightthickness=0)
        if role:self.actions.append((b,role))
        return b
    def _header(self):
        header=tk.Frame(self.root,bg=BG,padx=20,pady=16);header.grid(row=0,column=0,columnspan=2,sticky='ew');header.grid_columnconfigure(1,weight=1)
        brand=tk.Frame(header,bg=BG);brand.grid(row=0,column=0,sticky='w')
        self.label(brand,'麦轮任务控制台',size=21,bold=True).pack(anchor='w')
        self.label(brand,'FPGA  /  WIRELESS MISSION CONTROL',size=8,color=ACCENT).pack(anchor='w',pady=(3,0))
        connection=tk.Frame(header,bg=BG);connection.grid(row=0,column=1,sticky='e',padx=18)
        self.port=ttk.Combobox(connection,width=27,state='readonly',font=SMALL);self.port.grid(row=0,column=0,columnspan=2,pady=(0,6))
        self.refresh_button=self.button(connection,'刷新设备',self.refresh_ports);self.refresh_button.grid(row=1,column=0,sticky='ew',padx=(0,5))
        self.connect_button=self.button(connection,'连接无线',self.toggle,primary=True);self.connect_button.grid(row=1,column=1,sticky='ew')
        side=tk.Frame(header,bg=BG);side.grid(row=0,column=2,sticky='e')
        self.chip=tk.Label(side,text='离线演示' if self.demo else '未连接',font=FONT,bg=LINE,fg=PURPLE if self.demo else MUTED,padx=13,pady=4);self.chip.pack(fill='x')
        self.stop_button=self.button(side,'■  停止任务  /  Esc',self.stop,danger=True);self.stop_button.pack(pady=(7,0),fill='x')
    def _sidebar(self):
        sidebar=tk.Frame(self.root,bg=SIDE,width=200,padx=12,pady=14);sidebar.grid(row=1,column=0,sticky='ns');sidebar.grid_propagate(False)
        self.label(sidebar,'任务面板',size=9,color=MUTED).pack(anchor='w',padx=10,pady=(0,12))
        self.nav_buttons={}
        for key,title in [('overview','01  总览与反馈'),('motion','02  运动测试'),('radar','03  雷达任务'),('navigation','04  坐标导航'),('settings','05  航向与显示'),('logs','06  通信记录')]:
            b=self.button(sidebar,title,lambda k=key:self.show(k));b.pack(fill='x',pady=4);self.nav_buttons[key]=b
        speed=tk.Frame(sidebar,bg=CARD,padx=11,pady=13);speed.pack(side='bottom',fill='x',pady=8)
        self.label(speed,'平移速度倍率',size=11,bold=True).pack(anchor='w')
        self.speed_choice=tk.StringVar(value='100')
        self.speed_combo=ttk.Combobox(speed,values=['25','50','75','100','125','150'],textvariable=self.speed_choice,state='readonly',width=10,font=FONT)
        self.speed_combo.pack(fill='x',pady=(10,6))
        self.button(speed,'应用倍率  %',self.apply_speed,role='idle',primary=True).pack(fill='x')
        self.value_label(speed,'speed_actual','当前：待遥测',font=SMALL,color=ACCENT,anchor='w').pack(fill='x',pady=(9,4))
        self.label(speed,'100%为新版平移基准。\n各档平移目标再乘1.75。\n旋转输出110；停止后调倍率。',size=8,color=MUTED,justify='left',wraplength=158).pack(anchor='w')
    def _body(self):
        self.body=tk.Frame(self.root,bg=BG,padx=18,pady=8);self.body.grid(row=1,column=1,sticky='nsew');self.body.grid_rowconfigure(1,weight=1);self.body.grid_columnconfigure(0,weight=1)
        top=tk.Frame(self.body,bg=BG);top.grid(row=0,column=0,sticky='ew',pady=(0,13));top.grid_columnconfigure(0,weight=1)
        self.page_title=self.label(top,'总览与反馈',size=18,bold=True);self.page_title.grid(row=0,column=0,sticky='w')
        self.value_label(top,'firmware','等待97F4固件',font=SMALL,color=MUTED).grid(row=0,column=1,sticky='e')
        self.pages={k:tk.Frame(self.body,bg=BG) for k in ['overview','motion','radar','navigation','settings','logs']}
        for f in self.pages.values():f.grid(row=1,column=0,sticky='nsew')
        self._overview();self._motion();self._radar();self._navigation();self._settings();self._logs()
    def _metric(self,parent,title,key,note=None):
        c=self.card(parent);self.label(c,title,size=9,color=MUTED).pack(anchor='w')
        self.value_label(c,key).pack(anchor='w',pady=(6,2))
        if note:self.label(c,note,size=8,color=MUTED).pack(anchor='w')
        return c
    def _world(self,parent,height=300):
        canvas=tk.Canvas(parent,bg=SIDE,highlightthickness=0,height=height)
        canvas.pack(fill='both',expand=True,pady=(9,0));canvas.bind('<Configure>',lambda e:self.draw_world(canvas));self.worlds.append(canvas)
        return canvas
    def _overview(self):
        page=self.pages['overview'];page.grid_columnconfigure(0,weight=3);page.grid_columnconfigure(1,weight=2);page.grid_rowconfigure(1,weight=1)
        strip=tk.Frame(page,bg=BG);strip.grid(row=0,column=0,columnspan=2,sticky='ew',pady=(0,13))
        for i,(title,key,note) in enumerate([('当前位置','position','厘米 · 场地固定坐标'),('IMU航向角','yaw','度 · 车头左偏为正'),('雷达测距','radar_state','基于有效完整扫描'),('任务状态','task_state','状态来自FPGA遥测')]):
            strip.grid_columnconfigure(i,weight=1,uniform='m');self._metric(strip,title,key,note).grid(row=0,column=i,sticky='nsew',padx=(0,9 if i<3 else 0))
        world=self.card(page,'场地与目标','矩形场地坐标、车辆位置和绕行路径');world.grid(row=1,column=0,sticky='nsew',padx=(0,13));self._world(world)
        right=tk.Frame(page,bg=BG);right.grid(row=1,column=1,sticky='nsew');right.grid_columnconfigure(0,weight=1);right.grid_rowconfigure(1,weight=1)
        attitude=self.card(right,'姿态反馈');attitude.grid(row=0,column=0,sticky='ew',pady=(0,12))
        self.value_label(attitude,'angles','Roll —   Pitch —',font=('Consolas',12),color=BLUE).pack(anchor='w',pady=8)
        self.compass=tk.Canvas(attitude,height=125,bg=CARD,highlightthickness=0);self.compass.pack(fill='x');self.compass.bind('<Configure>',lambda e:self.draw_compass())
        wheels=self.card(right,'四轮编码器','速度单位：编码器计数 / 10ms');wheels.grid(row=1,column=0,sticky='nsew')
        for k in WHEEL_NAMES:
            row=tk.Frame(wheels,bg=CARD);row.pack(fill='x',pady=5)
            self.label(row,WHEEL_NAMES[k],color=WHEEL_COLORS[k],bold=True).pack(side='left')
            self.value_label(row,'wheel_'+k,'—',font=('Consolas',11),color=TEXT).pack(side='right')
        self.value_label(wheels,'sensor_state','IMU待就绪 · TOF未接入',font=SMALL,color=MUTED,wraplength=310,justify='left').pack(anchor='w',pady=(12,0))
    def _motion(self):
        page=self.pages['motion'];page.grid_columnconfigure(0,weight=1);page.grid_rowconfigure(2,weight=1)
        self.label(page,'四向平移保持车头方向；每次平移约4秒，原地转向到角后结束。',color=MUTED).grid(row=0,column=0,sticky='w',pady=(0,12))
        grid=tk.Frame(page,bg=BG);grid.grid(row=1,column=0,sticky='ew')
        labels=['↑  前进4秒','↓  后退4秒','←  左横移4秒','→  右横移4秒','↺  左转90°','↻  右转90°']
        for i,label in enumerate(labels):
            grid.grid_columnconfigure(i%3,weight=1,uniform='move')
            card=self.card(grid);card.grid(row=i//3,column=i%3,sticky='nsew',padx=(0,10 if i%3<2 else 0),pady=(0,11))
            self.button(card,label,lambda n=i:self.send([(MOTION,n,0,0)]),role='motion',primary=i<4).pack(fill='x')
            self.label(card,'平移 · 随速度倍率调整' if i<4 else '原地旋转 · 新版输出110',size=9,color=MUTED).pack(pady=(9,0))
        trend=self.card(page,'四轮速度反馈','LF 左前   RF 右前   LB 左后   RB 右后');trend.grid(row=2,column=0,sticky='nsew')
        self.trend=tk.Canvas(trend,bg=SIDE,highlightthickness=0,height=180);self.trend.pack(fill='both',expand=True,pady=(9,0));self.trend.bind('<Configure>',lambda e:self.draw_trend())
    def _radar(self):
        page=self.pages['radar'];page.grid_columnconfigure(0,weight=1)
        strip=tk.Frame(page,bg=BG);strip.grid(row=0,column=0,sticky='ew')
        for i,(title,key) in enumerate([('前方 F','front'),('后方 B','back'),('左侧 L','left'),('右侧 R','right')]):
            strip.grid_columnconfigure(i,weight=1,uniform='range');self._metric(strip,title,key,'厘米 · 雷达中心起算').grid(row=0,column=i,sticky='nsew',padx=(0,10 if i<3 else 0))
        task=self.card(page,'任务3 · 接近障碍，减速停车','用无线启动后可离开车身附近，避免按键时被雷达识别。');task.grid(row=1,column=0,sticky='ew',pady=13)
        buttons=tk.Frame(task,bg=CARD);buttons.pack(fill='x')
        self.button(buttons,'启动减速停车测试',lambda:self.send([(APPROACH,0,0,0)]),role='approach',primary=True).pack(side='left',padx=(0,10))
        self.button(buttons,'TFT实时雷达',lambda:self.send([(PAGE,5,0,0)]),role='idle').pack(side='left',padx=(0,10))
        self.button(buttons,'进入坐标导航 →',lambda:self.show('navigation')).pack(side='left')
        self.value_label(task,'obstacle_flags','RANGE —   NEAR —   SLOW —   STOP —',font=('Consolas',12),color=AMBER).pack(anchor='w',pady=(15,4))
        self.value_label(task,'front_limit','前进速度目标上限：—',font=FONT,color=TEXT).pack(anchor='w')
        threshold=self.card(page,'距离与速度分段','阈值保持不变，速度分段随平移倍率同比例调整。');threshold.grid(row=2,column=0,sticky='ew')
        for i,(distance,key) in enumerate([('≥70cm','stage0'),('55～70cm','stage1'),('40～55cm','stage2'),('27～40cm','stage3'),('≤27cm','stage4')]):
            threshold.grid_columnconfigure(i,weight=1)
            col=tk.Frame(threshold,bg=CARD);col.pack(side='left',expand=True,fill='x',pady=(10,2))
            self.label(col,distance,size=10,color=MUTED).pack();self.value_label(col,key,font=MONO,color=RED if i==4 else ACCENT).pack(pady=6)
        notes=self.card(page,'如何理解这一页');notes.grid(row=3,column=0,sticky='ew',pady=(13,0))
        self.label(notes,'LIVE DISTANCES：四向距离。APPROACH：向前接近并减速，最长约8秒。\nBYPASS：使用导航页建系、设目标、规划绕行。NEAR会检查任意方向27cm内的回波。\nSLOW / STOP记录本次测试过程；速度数字是计数/10ms，不是PWM百分比。',color=MUTED,size=9,justify='left',wraplength=850).pack(anchor='w')
    def _number(self,parent,text,variable,row,col=0,step=5,width=8):
        self.label(parent,text,size=9,color=MUTED).grid(row=row,column=col,sticky='w',padx=(0,6),pady=5)
        ttk.Spinbox(parent,from_=0,to=6553.5,increment=step,textvariable=variable,width=width,font=FONT).grid(row=row,column=col+1,sticky='ew',padx=(0,9))
    def _navigation(self):
        page=self.pages['navigation'];page.grid_columnconfigure(0,weight=3);page.grid_columnconfigure(1,weight=2);page.grid_rowconfigure(0,weight=1)
        map_card=self.card(page,'场地坐标','先将车头朝+Y，静止建系；点击图内可选择5cm网格目标。');map_card.grid(row=0,column=0,sticky='nsew',padx=(0,13))
        canvas=self._world(map_card,height=390);canvas.bind('<Button-1>',lambda e,c=canvas:self.map_click(c,e))
        self.value_label(map_card,'box','BOX — × — cm',font=FONT,color=BLUE).pack(anchor='w',pady=(9,0))
        self.value_label(map_card,'nav_flags','等待有效边框和定位',font=SMALL,color=MUTED,wraplength=460,justify='left').pack(anchor='w',pady=(5,0))
        right=ttk.Notebook(page,style='Mission.TNotebook');right.grid(row=0,column=1,sticky='nsew')
        single_tab=tk.Frame(right,bg=BG);points_tab=tk.Frame(right,bg=BG)
        right.add(single_tab,text='单点与建系');right.add(points_tab,text='四点顺序')
        goal=self.card(single_tab,'单点任务');goal.pack(fill='both',expand=True)
        self.button(goal,'①  静止校准边框',lambda:self.send([(CALIBRATE,0,0,0)]),role='calibrate').pack(fill='x',pady=(4,9))
        form=tk.Frame(goal,bg=CARD);form.pack(fill='x');form.grid_columnconfigure(1,weight=1)
        self.goal_x=tk.StringVar(value='100');self.goal_y=tk.StringVar(value='100')
        self._number(form,'目标 X / cm',self.goal_x,0);self._number(form,'目标 Y / cm',self.goal_y,1)
        self.button(goal,'②  上传目标',lambda:self.goal(False),role='goal').pack(fill='x',pady=(8,6))
        self.button(goal,'③  上传并出发',lambda:self.goal(True),role='go',primary=True).pack(fill='x')
        points=self.card(points_tab,'四点顺序任务');points.pack(fill='both',expand=True)
        form=tk.Frame(points,bg=CARD);form.pack(fill='x');self.points=[]
        for i,(x,y) in enumerate([(50,50),(150,50),(150,150),(50,150)]):
            xv=tk.StringVar(value=str(x));yv=tk.StringVar(value=str(y));self.points.append((xv,yv))
            self.label(form,str(i+1),size=9,color=ACCENT).grid(row=i,column=0,padx=(0,7))
            ttk.Spinbox(form,from_=0,to=300,increment=5,textvariable=xv,width=6,font=FONT).grid(row=i,column=1,pady=4,padx=3)
            ttk.Spinbox(form,from_=0,to=300,increment=5,textvariable=yv,width=6,font=FONT).grid(row=i,column=2,pady=4,padx=3)
        self.label(points,'左列X，右列Y，单位cm',size=8,color=MUTED).pack(anchor='w',pady=4)
        self.button(points,'上传四点',lambda:self.route(False),role='goal').pack(fill='x',pady=(4,6))
        self.button(points,'上传四点并出发',lambda:self.route(True),role='go',primary=True).pack(fill='x')
        self.label(points,'先空场单点，再验证四点与绕障。\n示意连线表示目标顺序。',size=8,color=MUTED,justify='left').pack(anchor='w',pady=(9,0))
        for variable in [self.goal_x,self.goal_y]+[v for pair in self.points for v in pair]:variable.trace_add('write',lambda *args:self.redraw_worlds())
    def _settings(self):
        page=self.pages['settings'];page.grid_columnconfigure(0,weight=1);page.grid_columnconfigure(1,weight=1)
        gains=self.card(page,'航向参数','P/D为板上原始参数值，只在本次上电有效。');gains.grid(row=0,column=0,sticky='nsew',padx=(0,13))
        form=tk.Frame(gains,bg=CARD);form.pack(fill='x');self.kp=tk.StringVar(value='320');self.kd=tk.StringVar(value='2560')
        self._number(form,'航向 P',self.kp,0,step=8);self._number(form,'航向 D',self.kd,1,step=256)
        self.button(gains,'应用航向参数',self.gains,role='idle',primary=True).pack(fill='x',pady=(13,0))
        self.label(gains,'P范围0～1280，D范围0～16384。\n平移倍率不改变航向环参数和原地转速（110）。',size=9,color=MUTED,justify='left',wraplength=420).pack(anchor='w',pady=(12,0))
        raw=self.card(page,'原始IMU与控制状态');raw.grid(row=0,column=1,sticky='nsew')
        self.value_label(raw,'raw_imu','AX —   AY —   AZ —\nGX —   GY —   GZ —',font=('Consolas',12),color=BLUE,justify='left').pack(anchor='w',pady=10)
        self.value_label(raw,'pid','RUN —   FAULT —',font=('Consolas',11),color=MUTED).pack(anchor='w')
        display=self.card(page,'TFT画面管理','远程切换板上屏幕；摄像头图像仍在TFT显示。');display.grid(row=1,column=0,columnspan=2,sticky='ew',pady=(13,0))
        grid=tk.Frame(display,bg=CARD);grid.pack(fill='x')
        for i,name in enumerate(PAGES):
            grid.grid_columnconfigure(i%3,weight=1,uniform='display')
            self.button(grid,name,lambda n=i:self.send([(PAGE,n,0,0)]),role='idle').grid(row=i//3,column=i%3,sticky='ew',padx=5,pady=6)
        self.button(display,'返回板上主菜单',lambda:self.send([(MENU,0,0,0)]),role='idle').pack(anchor='w',pady=(9,0))
    def _logs(self):
        page=self.pages['logs'];page.grid_columnconfigure(0,weight=1);page.grid_rowconfigure(1,weight=1)
        tools=tk.Frame(page,bg=BG);tools.grid(row=0,column=0,sticky='ew',pady=(0,10))
        self.label(tools,'指令确认、连接事件与原始状态',color=MUTED).pack(side='left')
        self.button(tools,'导出记录',self.export_log).pack(side='right')
        frame=tk.Frame(page,bg=CARD);frame.grid(row=1,column=0,sticky='nsew');frame.grid_rowconfigure(0,weight=1);frame.grid_columnconfigure(0,weight=1)
        self.log_widget=tk.Text(frame,bg=SIDE,fg=TEXT,insertbackground=TEXT,relief='flat',font=('Consolas',10),wrap='word',state='disabled',padx=12,pady=12)
        self.log_widget.grid(row=0,column=0,sticky='nsew');scroll=ttk.Scrollbar(frame,command=self.log_widget.yview);scroll.grid(row=0,column=1,sticky='ns');self.log_widget.configure(yscrollcommand=scroll.set)

    def show(self,key):
        self.current=key;self.pages[key].tkraise()
        titles={'overview':'总览与反馈','motion':'运动测试','radar':'雷达任务','navigation':'坐标导航','settings':'航向与显示','logs':'通信记录'}
        self.page_title.configure(text=titles[key])
        for k,b in self.nav_buttons.items():b.configure(bg=CARD if k==key else SIDE,fg=ACCENT if k==key else MUTED)
        self.root.after_idle(self.redraw_worlds);self.root.after_idle(self.draw_trend)
    def refresh_ports(self):
        if self.demo:self.port['values']=['离线演示 · 不使用串口'];self.port.current(0);return
        from serial.tools import list_ports
        ports=list(list_ports.comports());self.ports={f'{p.device} · {p.description}':p.device for p in ports};self.port['values']=list(self.ports)
        native=next((label for label in self.ports if self.ports[label] in [p.device for p in ports if p.vid==0x0483 and p.pid==0x5740]),None)
        if native:self.port.set(native)
        elif self.port.get() not in self.ports:self.port.set('')
    def toggle(self):
        if self.demo:return
        if self.connected:self.connect_button.configure(state='disabled');self.link.close_event.set()
        elif self.port.get() in getattr(self,'ports',{}):
            try:self.link.connect(self.ports[self.port.get()]);self.connect_button.configure(state='disabled')
            except ValueError as exc:self.note(str(exc),error=True)
        else:self.note('请先选择逐飞USB端串口。',error=True)
    def available(self,role):
        if self.demo:return True
        fresh=time.monotonic()-self.last_task<1.5
        base=self.connected and self.remote_ready and not self.pending and fresh and self.task.get('firmware')==FIRMWARE and not self.task.get('active',False)
        if not base:return False
        imu=self.att.get('ready',False) and self.sensors.get('IMU_INIT',False) and self.task.get('imu_fresh',False)
        if role=='motion':return imu
        if role in ('approach','calibrate'):return imu and self.task.get('radar',False) and not self.task.get('near',False)
        if role=='goal':return self.task.get('calibrated',False)
        if role=='go':return self.task.get('can_go',False)
        return True
    def update_controls(self):
        for button,role in self.actions:button.configure(state='normal' if self.available(role) else 'disabled')
        self.stop_button.configure(state='normal' if self.connected or self.demo else 'disabled')
        self.refresh_button.configure(state='disabled' if self.connected or self.demo else 'normal')
        if self.demo:self.connect_button.configure(state='disabled');self.port.configure(state='disabled');self.refresh_ports();self.chip.configure(text='离线演示 · 模拟数据',fg=PURPLE,bg=LINE)
        elif self.connected and self.remote_ready:
            stale=time.monotonic()-self.last_task>1.5
            self.chip.configure(text='遥测待更新' if stale else '任务运行中' if self.task.get('active') else '已连接 · 就绪',fg=AMBER if stale or self.task.get('active') else ACCENT,bg=LINE)
        else:self.chip.configure(text='等待握手' if self.connected else '未连接',fg=AMBER if self.connected else MUTED,bg=LINE)
    def send(self,commands):
        if self.demo:
            self.note('演示指令：'+str(commands));self.preview_commands(commands);return
        if not self.connected or not self.remote_ready:return
        self.pending=True;self.update_controls();self.link.submit(commands)
    def stop(self):
        if self.demo:self.task.update(active=False,demo=7,mission=7);self.demo_until=0;self.note('离线演示：停止任务');self.render();return
        if self.connected:self.link.stop_event.set();self.note('正在发送停止指令。')
    def apply_speed(self):
        try:self.send([(SPEED,speed_level(self.speed_choice.get()),0,0)])
        except ValueError as exc:messagebox.showerror('速度倍率',str(exc))
    def checked_xy(self,xv,yv):
        x,y=cm_to_mm(xv.get()),cm_to_mm(yv.get())
        if self.task.get('calibrated'):
            mx=(round(self.task.get('width_cm',200)*10)-295)//50*50;my=(round(self.task.get('height_cm',200)*10)-295)//50*50
            if not 300<=x<=mx or not 300<=y<=my:raise ValueError(f'当前目标范围：X 30～{mx/10:g}cm，Y 30～{my/10:g}cm')
        return x,y
    def goal(self,go):
        try:
            x,y=self.checked_xy(self.goal_x,self.goal_y);self.send([(GOAL,x,y,0)]+([(GO,0,0,0)] if go else []))
        except ValueError as exc:messagebox.showerror('目标坐标',str(exc))
    def route(self,go):
        try:
            commands=[]
            for i,(xv,yv) in enumerate(self.points):x,y=self.checked_xy(xv,yv);commands.append((POINT,i,x,y))
            self.send(commands+([(ROUTE,0,0,0)] if go else []))
        except ValueError as exc:messagebox.showerror('四点坐标',str(exc))
    def gains(self):
        try:
            p,d=int(self.kp.get()),int(self.kd.get())
            if not 0<=p<=1280 or not 0<=d<=16384:raise ValueError('P范围0～1280，D范围0～16384')
            self.send([(GAINS,p,d,0)])
        except ValueError as exc:messagebox.showerror('航向参数',str(exc))
    def note(self,text,error=False):
        text=str(text);entry=f'{datetime.now():%H:%M:%S}  {text}';self.logs.append(entry)
        if hasattr(self,'footer'):self.footer.configure(text=text,fg=RED if error else MUTED)
        if hasattr(self,'log_widget'):
            self.log_widget.configure(state='normal');self.log_widget.insert('end',entry+'\n')
            if int(self.log_widget.index('end-1c').split('.')[0])>1200:self.log_widget.delete('1.0','101.0')
            self.log_widget.see('end');self.log_widget.configure(state='disabled')
    def export_log(self):
        name=filedialog.asksaveasfilename(title='导出通信记录',defaultextension='.txt',initialfile=f'小车通信_{datetime.now():%Y%m%d_%H%M%S}.txt',filetypes=[('文本记录','*.txt')])
        if name:Path(name).write_text('\n'.join(self.logs),encoding='utf-8');self.note('记录已保存：'+name)
    def poll(self):
        dirty=False
        for _ in range(400):
            try:kind,value=self.events.get_nowait()
            except queue.Empty:break
            if kind=='connection':
                self.connected=value;self.connect_button.configure(state='normal',text='断开连接' if value else '连接无线');self.port.configure(state='disabled' if value else 'readonly')
                self.note('串口已打开，正在核对97F4固件。' if value else '连接已关闭。')
                if not value:
                    self.remote_ready=False
                    if hasattr(self,'on_link_lost'):self.on_link_lost()
            elif kind=='ready':self.remote_ready=value;self.note('97F4握手成功，等待选择任务。' if value else '任务指令尚未启用。')
            elif kind=='pending':self.pending=value
            elif kind=='message':self.note(value,error=True)
            elif kind=='ack':
                self.note(f'ACK {value["seq"]:02X} · '+STATUS.get(value['status'],'未知返回码'),error=value['status']!=0)
                if hasattr(self,'on_command_ack'):self.on_command_ack(value)
            elif kind=='sent':
                self.note(f'发送 #{value["seq"]:02X} · OP {value["op"]}')
                if hasattr(self,'on_command_sent'):self.on_command_sent(value)
            else:
                self.ingest(kind,value)
                if kind not in ('drive_sent','drive_ack'):dirty=True
        if dirty:self.render()
        self.update_controls();self.root.after(60,self.poll)
    def ingest(self,kind,value):
        self.last_rx=time.monotonic()
        if kind=='task':self.task=value;self.last_task=time.monotonic()
        elif kind=='attitude':self.att=value
        elif kind=='wheels':
            self.wheels=value
            for k in WHEEL_NAMES:
                if k in value:self.history[k].append(value[k]['speed'])
        elif kind=='sensors':self.sensors=value
        elif kind=='imu':self.imu=value
        elif kind=='pid':self.pid=value
        elif kind=='text' and value:self.note(value)
    def render(self):
        t=self.task;a=self.att
        self.variable('firmware').set('97F4 · 模拟数据' if self.demo else f'{t["firmware"]:04X} · 实时遥测' if t.get('firmware') else '等待97F4固件')
        self.variable('position').set(f'{t["x_cm"]:.1f}, {t["y_cm"]:.1f}' if t.get('calibrated') else '等待建系')
        self.variable('yaw').set(f'{a["Y"]:+.2f}°' if 'Y' in a else '—')
        self.variable('angles').set(f'Roll {a.get("R",0):+.2f}°   Pitch {a.get("P",0):+.2f}°' if a else 'Roll —   Pitch —')
        self.variable('radar_state').set('有效扫描' if t.get('radar') else '等待数据')
        status=t.get('demo',0) if t.get('demo_running') or t.get('group')==2 else t.get('mission',0)
        if t.get('group')==3:status=t.get('demo',0)
        self.variable('task_state').set(TASK_STATUS.get(status,'等待状态'))
        percent=t.get('speed_percent')
        self.variable('speed_actual').set(f'当前：{percent}%' if percent else '当前：待遥测')
        for k in WHEEL_NAMES:
            row=self.wheels.get(k);self.variable('wheel_'+k).set(f'V {row["speed"]:+5d}  C {row["count"]:+7d}' if row else '—')
        self.variable('sensor_state').set(('IMU已就绪' if self.sensors.get('IMU_INIT') and a.get('ready') else 'IMU等待就绪')+' · TOF未接入')
        for name in ['front','back','left','right']:
            val=t.get(name+'_cm');self.variable(name).set(f'{val:.1f}' if val is not None and t.get('radar') else '—')
        self.variable('obstacle_flags').set('   '.join(f'{name} {int(bool(t.get(key)))}' for name,key in [('RANGE','radar'),('NEAR','near'),('SLOW','slow_seen'),('STOP','stop_seen')]) if t else 'RANGE —   NEAR —   SLOW —   STOP —')
        factor=(percent or 100)//25
        for i,base in enumerate([80,60,40,20,0]):self.variable('stage'+str(i)).set(str(base*factor//4))
        distance=t.get('front_cm',0)*10
        base=0 if distance<=270 else 20 if distance<400 else 40 if distance<550 else 60 if distance<700 else 80
        self.variable('front_limit').set(f'前进速度目标上限：{base*factor//4 if t.get("radar") else 0} 计数/10ms')
        self.variable('box').set(f'BOX {t.get("width_cm",0):.1f} × {t.get("height_cm",0):.1f} cm' if t.get('calibrated') else 'BOX 等待边框建系')
        self.variable('nav_flags').set(f'建系 {int(bool(t.get("calibrated")))}   定位 {int(bool(t.get("pose")))}   可出发 {int(bool(t.get("can_go")))}\n导航：{TASK_STATUS.get(t.get("nav",0),"等待状态")}   点 {t.get("waypoint",1)}/4')
        self.variable('raw_imu').set('   '.join(f'{k} {self.imu.get(k,"—")}' for k in ['AX','AY','AZ'])+'\n'+'   '.join(f'{k} {self.imu.get(k,"—")}' for k in ['GX','GY','GZ']))
        self.variable('pid').set(f'RUN {self.pid.get("run","—")}   FAULT {self.pid.get("fault","—")}')
        self.redraw_worlds();self.draw_compass();self.draw_trend()
    def redraw_worlds(self):
        for c in self.worlds:
            if c.winfo_exists():self.draw_world(c)
    def draw_world(self,c):
        c.delete('all');w=max(c.winfo_width(),100);h=max(c.winfo_height(),100);pad=42
        measured=self.task.get('calibrated') and self.task.get('width_cm',0)>0 and self.task.get('height_cm',0)>0
        bw=self.task.get('width_cm',200) if measured else 200;bh=self.task.get('height_cm',200) if measured else 200
        scale=max(.1,min((w-pad*2)/bw,(h-pad*2)/bh));ox=(w-bw*scale)/2;oy=(h+bh*scale)/2
        c._world_transform=(ox,oy,scale,bw,bh)
        def pos(x,y):return ox+x*scale,oy-y*scale
        for v in range(0,int(max(bw,bh))+1,25):
            if v<=bw:x,y=pos(v,0);c.create_line(x,oy,x,oy-bh*scale,fill=LINE);c.create_text(x,oy+15,text=str(v),fill=MUTED,font=('Consolas',8))
            if v<=bh:x,y=pos(0,v);c.create_line(ox,y,ox+bw*scale,y,fill=LINE);c.create_text(ox-18,y,text=str(v),fill=MUTED,font=('Consolas',8))
        c.create_rectangle(ox,oy-bh*scale,ox+bw*scale,oy,outline=BLUE if measured else MUTED,width=2,dash=() if measured else (5,5))
        c.create_text(ox+6,oy+28,text='O  → +X / cm',fill=ACCENT,anchor='w',font=('Consolas',9));c.create_text(ox-24,oy-bh*scale-14,text='+Y',fill=ACCENT,font=('Consolas',9))
        if not measured:c.create_text(w/2,h/2,text='未建系 · 等待实测边框',fill=MUTED,font=('Microsoft YaHei UI',12));return
        points=[]
        for i,(xv,yv) in enumerate(getattr(self,'points',[])):
            try:x,y=float(xv.get()),float(yv.get())
            except ValueError:continue
            if not math.isfinite(x+y) or not (0<=x<=bw and 0<=y<=bh):continue
            sx,sy=pos(x,y);points.append((sx,sy));c.create_oval(sx-4,sy-4,sx+4,sy+4,fill=MUTED,outline='');c.create_text(sx+10,sy-9,text=str(i+1),fill=MUTED,font=('Consolas',10))
        if len(points)>1:c.create_line(*[coord for p in points for coord in p],fill=LINE,dash=(4,5),width=2)
        try:gx,gy=float(self.goal_x.get()),float(self.goal_y.get())
        except (ValueError,AttributeError):gx=gy=-1
        if math.isfinite(gx+gy) and 0<=gx<=bw and 0<=gy<=bh:
            sx,sy=pos(gx,gy);c.create_oval(sx-9,sy-9,sx+9,sy+9,outline=AMBER,width=2);c.create_line(sx-13,sy,sx+13,sy,fill=AMBER);c.create_line(sx,sy-13,sx,sy+13,fill=AMBER)
        if self.task.get('pose'):
            sx,sy=pos(self.task.get('x_cm',0),self.task.get('y_cm',0));angle=math.radians(self.task.get('yaw_deg',90))
            c.create_oval(sx-11,sy-11,sx+11,sy+11,fill=ACCENT,outline=BG,width=2)
            c.create_line(sx,sy,sx+25*math.cos(angle),sy-25*math.sin(angle),fill=ACCENT,width=3,arrow='last')
            c.create_text(sx,sy+23,text='车辆',fill=TEXT,font=SMALL)
    def map_click(self,c,event):
        if not self.task.get('calibrated') or self.task.get('active'):return
        ox,oy,scale,bw,bh=getattr(c,'_world_transform',(0,0,1,200,200))
        x=round((event.x-ox)/scale/5)*5;y=round((oy-event.y)/scale/5)*5
        mx=math.floor((bw-29.5)/5)*5;my=math.floor((bh-29.5)/5)*5
        self.goal_x.set(str(max(30,min(mx,x))));self.goal_y.set(str(max(30,min(my,y))));self.note('目标已选中；上传并出发后才执行。')
    def draw_compass(self):
        if not hasattr(self,'compass'):return
        c=self.compass;c.delete('all');w=max(c.winfo_width(),120);cx=w/2;cy=64;r=45
        c.create_oval(cx-r,cy-r,cx+r,cy+r,outline=LINE,width=2);c.create_text(cx,cy-r-10,text='0°',fill=MUTED,font=('Consolas',9))
        if 'Y' not in self.att:c.create_text(cx,cy,text='—',fill=MUTED);return
        angle=math.radians(self.att['Y']);c.create_line(cx,cy,cx-r*.8*math.sin(angle),cy-r*.8*math.cos(angle),fill=ACCENT,width=4,arrow='last')
        c.create_oval(cx-4,cy-4,cx+4,cy+4,fill=TEXT,outline='')
    def draw_trend(self):
        if not hasattr(self,'trend'):return
        c=self.trend;c.delete('all');w=max(c.winfo_width(),100);h=max(c.winfo_height(),120);pad=35
        maximum=max(150,max((abs(v) for data in self.history.values() for v in data),default=0));mid=h/2
        for value in [-maximum,0,maximum]:
            y=mid-value*(h-2*pad)/(2*maximum);c.create_line(pad,y,w-12,y,fill=LINE);c.create_text(pad-6,y,text=str(value),fill=MUTED,anchor='e',font=('Consolas',8))
        for k,data in self.history.items():
            if len(data)<2:continue
            coords=[]
            for i,value in enumerate(data):coords.extend([pad+i*(w-pad-15)/89,mid-value*(h-2*pad)/(2*maximum)])
            c.create_line(*coords,fill=WHEEL_COLORS[k],width=2)
        if not any(self.history.values()):c.create_text(w/2,h/2,text='等待编码器速度数据',fill=MUTED,font=FONT)
    def seed_demo(self):
        self.connected=False;self.remote_ready=False
        self.task=dict(active=False,calibrated=True,radar=True,pose=True,can_go=True,heading_ready=True,demo_running=False,mission_running=False,
                       demo=0,mission=0,nav=3,waypoint=1,route=False,group=0,x_cm=73,y_cm=86,yaw_deg=90,
                       goal_x_cm=100,goal_y_cm=100,front_cm=114,back_cm=86,left_cm=73,right_cm=127,width_cm=200,height_cm=200,
                       near=False,slow_seen=False,stop_seen=False,imu_fresh=True,speed_percent=100,firmware=FIRMWARE)
        self.att=dict(R=.38,P=-.72,Y=.16,ready=True);self.sensors=dict(IMU_INIT=True,IMU_NEW=True,TOF_INIT=False,TOF_NEW=False)
        self.imu=dict(AX=12,AY=-28,AZ=4094,GX=1,GY=-1,GZ=0);self.pid=dict(run='0000',fault='0000');self.render()
    def preview_commands(self,commands):
        for op,a,b,c in commands:
            if op==SPEED:self.task['speed_percent']=a*25
            elif op==GOAL:self.task.update(goal_x_cm=a/10,goal_y_cm=b/10)
            elif op in (MOTION,APPROACH,GO,ROUTE):
                self.task.update(active=True,demo=4,mission=4,demo_running=op in (MOTION,APPROACH),mission_running=op in (GO,ROUTE));self.demo_until=time.monotonic()+3
            elif op==CALIBRATE:self.task['calibrated']=True
        self.render()
    def demo_tick(self):
        if not self.demo:return
        elapsed=time.monotonic()-self.demo_started
        self.att['Y']=.16+.12*math.sin(elapsed/2)
        for i,k in enumerate(WHEEL_NAMES):
            speed=round(210+3*math.sin(elapsed*2+i)) if self.task.get('active') else 0
            count=self.wheels.get(k,{}).get('count',[1328,1322,1334,1325][i])+speed
            self.wheels[k]=dict(count=count,speed=speed);self.history[k].append(speed)
        if self.demo_until and time.monotonic()>self.demo_until:self.task.update(active=False,demo=5,mission=5,demo_running=False,mission_running=False);self.demo_until=0
        self.render();self.root.after(180,self.demo_tick)
    def close(self):
        self.link.close_event.set();self.root.withdraw();deadline=time.monotonic()+1
        def finish():
            if not self.link.thread or not self.link.thread.is_alive() or time.monotonic()>=deadline:self.root.destroy()
            else:self.root.after(20,finish)
        finish()

class App(BaseApp):
    def __init__(self,root,demo=False,enumerate_ports=True):
        self.cloud={};self.cloud_time=0;self.previous_mask=0
        self.radar_radius=tk.StringVar(root,value='300');self.radar_view=tk.StringVar(root,value='车体雷达')
        self.obstacle_fb=tk.StringVar(root,value='40');self.obstacle_lr=tk.StringVar(root,value='25')
        self.obstacle_cone=tk.StringVar(root,value='中 ±20.6°');self.calibration_seq=None
        self.speed_focus_pending=False;self.speed_seq=None;self.release_before_drive=False
        self.manual_armed=tk.BooleanVar(root,value=False);self.click_go=tk.BooleanVar(root,value=False)
        super().__init__(root,demo,enumerate_ports)
        self.held_input=HeldInput(root);root.after(20,self.poll_drive)
    def checkbutton(self,parent,text,var,command=None):
        return tk.Checkbutton(parent,text=text,variable=var,command=command,bg=parent.cget('bg'),fg=TEXT,
          activebackground=parent.cget('bg'),activeforeground=ACCENT,selectcolor=SIDE,font=FONT,anchor='w',padx=0,pady=8)
    def _radar(self):
        page=self.pages['radar'];page.grid_columnconfigure(0,weight=3);page.grid_columnconfigure(1,weight=2);page.grid_rowconfigure(1,weight=1)
        ranges=tk.Frame(page,bg=BG);ranges.grid(row=0,column=0,columnspan=2,sticky='ew',pady=(0,12))
        for i,(title,key) in enumerate([('前进方向','front'),('后退方向','back'),('左移方向','left'),('右移方向','right')]):
            ranges.grid_columnconfigure(i,weight=1,uniform='r');self._metric(ranges,title,key,'只检查该方向正向窄扇区').grid(row=0,column=i,sticky='ew',padx=(0,8 if i<3 else 0))
        view=self.card(page,'实时雷达点云','彩色区域是任务3限行范围；距离从雷达中心起算。');view.grid(row=1,column=0,sticky='nsew',padx=(0,12))
        toolbar=tk.Frame(view,bg=CARD);toolbar.pack(fill='x',pady=(3,4))
        ttk.Combobox(toolbar,textvariable=self.radar_view,values=['车体雷达','场地坐标'],state='readonly',width=10,font=SMALL).pack(side='left')
        self.label(toolbar,'显示半径/cm',size=9,color=MUTED).pack(side='left',padx=(10,5))
        ttk.Combobox(toolbar,textvariable=self.radar_radius,values=['100','200','300','500'],state='readonly',width=6,font=SMALL).pack(side='left')
        for item in toolbar.winfo_children():
            if isinstance(item,ttk.Combobox):item.bind('<<ComboboxSelected>>',lambda e:self.finish_view_change())
        self.radar_view.trace_add('write',lambda *a:self.redraw_worlds());self.radar_radius.trace_add('write',lambda *a:self.redraw_worlds())
        self.radar_canvas=self._world(view,height=320);self.radar_canvas._body_radar=True
        self.radar_canvas.bind('<Button-1>',lambda e:self.map_click(self.radar_canvas,e))
        self.value_label(view,'scan_state','等待点云',font=SMALL,color=BLUE).pack(anchor='w',pady=(8,0))
        side=ttk.Notebook(page,style='Mission.TNotebook');side.grid(row=1,column=1,sticky='nsew')
        manual_tab=tk.Frame(side,bg=BG);nav_tab=tk.Frame(side,bg=BG)
        side.add(manual_tab,text='任务3 · 遥控');side.add(nav_tab,text='任务4 · 建系导航')
        side.bind('<<NotebookTabChanged>>',lambda e:self.radar_tab_changed(side,nav_tab))
        manual=self.card(manual_tab,'任务3 · 按住遥控');manual.pack(fill='both',expand=True)
        self.manual_check=self.checkbutton(manual,'启用 WASD / 鼠标侧键',self.manual_armed,self.arm_manual);self.manual_check.pack(fill='x')
        self.label(manual,'W/S前后，A/D横移，可组合斜走\n侧键1顺时针，侧键2逆时针；松键停',size=9,color=MUTED,justify='left',wraplength=320).pack(anchor='w',pady=(3,9))
        self.value_label(manual,'held_state','未启用遥控',font=SMALL,color=ACCENT,wraplength=320,justify='left').pack(anchor='w')
        self.value_label(manual,'blocked_axes','方向阻挡：等待数据',font=SMALL,color=AMBER,wraplength=320,justify='left').pack(anchor='w',pady=(8,0))
        self.value_label(manual,'drive_actual','实际目标：—',font=SMALL,color=MUTED,wraplength=320).pack(anchor='w',pady=(8,0))
        config=tk.Frame(manual,bg=CARD);config.pack(fill='x',pady=(12,3))
        self.label(config,'前后/cm',size=9,color=MUTED).grid(row=0,column=0,padx=(0,4))
        ttk.Spinbox(config,from_=20,to=80,increment=5,textvariable=self.obstacle_fb,width=4,font=FONT).grid(row=0,column=1)
        self.label(config,'左右/cm',size=9,color=MUTED).grid(row=0,column=2,padx=(9,4))
        ttk.Spinbox(config,from_=15,to=60,increment=5,textvariable=self.obstacle_lr,width=4,font=FONT).grid(row=0,column=3)
        self.label(config,'半角',size=9,color=MUTED).grid(row=1,column=0,pady=7)
        ttk.Combobox(config,textvariable=self.obstacle_cone,values=['窄 ±10.6°','中 ±20.6°','宽 ±29.4°'],state='readonly',width=17,font=SMALL).grid(row=1,column=1,columnspan=3,sticky='ew')
        self.button(manual,'应用阻挡范围',self.apply_obstacle,role='idle',primary=True).pack(fill='x',pady=(3,7))
        self.value_label(manual,'obstacle_actual','等待实际参数',font=SMALL,color=AMBER,wraplength=320,justify='left').pack(anchor='w')
        nav=self.card(nav_tab,'任务4 · 静止建系与点击导航');nav.pack(fill='both',expand=True)
        self.label(nav,'车停在框内、车头粗略对准+Y。\n四边需在雷达扫描高度内可见。\n建系本身不动车；不必把车放在原点。',size=9,color=MUTED,justify='left',wraplength=320).pack(anchor='w',pady=(4,8))
        self.button(nav,'①  静止建系（不动车）',self.calibrate_frame,role='calibrate').pack(fill='x',pady=(4,3))
        self.value_label(nav,'fit_state','尚未建系',font=SMALL,color=BLUE,wraplength=320,justify='left').pack(anchor='w',pady=(9,5))
        self.value_label(nav,'nav_reason','等待状态',font=SMALL,color=AMBER,wraplength=320,justify='left').pack(anchor='w',pady=(3,6))
        self.checkbutton(nav,'②  启用点击即出发',self.click_go,self.arm_click).pack(fill='x')
        self.label(nav,'校准时车头对准+Y；定位有效后，\n在点云图或导航图内点击目标。\n未启用时，点击只选点。',size=9,color=MUTED,justify='left',wraplength=320).pack(anchor='w',pady=(2,8))
        self.button(nav,'目标 / 四点详细设置 →',lambda:self.show('navigation')).pack(fill='x')
    def _navigation(self):
        super()._navigation()
        self.checkbutton(self.worlds[-1].master,'启用点击即出发（否则只选点）',self.click_go,self.arm_click).pack(anchor='w',pady=(5,0))
        self.value_label(self.worlds[-1].master,'nav_reason','等待状态',font=SMALL,color=AMBER,wraplength=450,justify='left').pack(anchor='w',pady=(4,0))
    def radar_tab_changed(self,tabs,nav_tab):
        if tabs.select()==str(nav_tab):self.manual_armed.set(False);self.link.set_drive(0)
        else:self.click_go.set(False)
    def finish_view_change(self):
        self.release_before_drive=True
        if self.manual_armed.get() and self.held_input.foreground():self.root.after_idle(self.root.focus_set)
    def available(self,role):
        if role=='calibrate':
            if self.demo:return True
            return self.available('manual') and not self.pending and not self.task.get('active')
        if role=='manual':
            if self.demo:return True
            t=self.task
            return self.connected and self.remote_ready and time.monotonic()-self.last_task<.6 and t.get('firmware')==FIRMWARE and (not t.get('active') or t.get('manual')) and t.get('manual_range_fresh') and t.get('imu_fresh') and self.att.get('ready') and self.sensors.get('IMU_INIT')
        return super().available(role)
    def arm_manual(self):
        if self.manual_armed.get() and not self.available('manual'):
            self.manual_armed.set(False);self.note('遥控需要无线连接、有效雷达和IMU，且不能有其他任务运行。',True)
        if self.manual_armed.get():self.click_go.set(False);self.root.focus_set();self.note('遥控已启用：按住即动，松开即停。')
        else:self.link.set_drive(0)
    def arm_click(self):
        if self.click_go.get() and not self.available('go'):
            self.click_go.set(False);self.note('；'.join(navigation_blockers(self.task,self.att)),True)
        elif self.click_go.get():
            self.manual_armed.set(False);self.link.set_drive(0);self.radar_view.set('场地坐标');self.note('点击即出发已启用；在场地图内点击一个目标。')
    def show(self,key):
        if key!='radar' and self.manual_armed.get():self.manual_armed.set(False);self.link.set_drive(0)
        super().show(key)
    def poll_drive(self):
        try:self._poll_drive_once()
        except Exception as exc:
            self.link.set_drive(0);self.manual_armed.set(False);self.note('遥控输入暂时异常，已停止输出：'+str(exc),True)
        finally:
            try:self.root.after(20,self.poll_drive)
            except tk.TclError:pass
    def _poll_drive_once(self):
        armed=self.manual_armed.get()
        if armed and (self.current!='radar' or (not self.demo and (not self.held_input.foreground() or not self.available('manual')))):
            self.manual_armed.set(False);armed=False;self.note('遥控已解除；需要时重新启用。')
        mask=self.held_input.mask() if armed and not self.demo else 0
        if self.pending or self.speed_focus_pending:
            mask=0;self.release_before_drive=True
        elif self.release_before_drive:
            if mask==0:self.release_before_drive=False
            mask=0
        self.link.set_drive(mask)
        if mask!=self.previous_mask:
            self.previous_mask=mask
            names=[name for i,name in enumerate(['W','S','A','D','逆时针','顺时针']) if mask&(1<<i)]
            self.variable('held_state').set('按住：'+' + '.join(names) if names else '已启用 · 松键停车' if armed else '未启用遥控')
        elif not armed:self.variable('held_state').set('未启用遥控')
        if armed and (self.pending or self.speed_focus_pending):self.variable('held_state').set('等待参数确认，暂时停车；遥控仍已启用')
        elif armed and self.release_before_drive:self.variable('held_state').set('请先松开全部按键，再按WASD或侧键继续')
        elif armed and mask==0:self.variable('held_state').set('已启用 · 松键停车，可再次按键')
    def apply_speed(self):
        try:level=speed_level(self.speed_choice.get())
        except ValueError as exc:messagebox.showerror('速度倍率',str(exc));return
        if not self.demo and (not self.connected or not self.remote_ready):return
        self.link.set_drive(0);self.release_before_drive=True;self.speed_focus_pending=not self.demo;self.speed_seq=None
        self.send([(DRIVE,0,0,0),(SPEED,level,0,0)])
        if self.demo:self.root.focus_set()
    def on_command_sent(self,value):
        if self.speed_focus_pending and value['op'] in (SPEED,OBSTACLE):self.speed_seq=value['seq']
        if value['op']==CALIBRATE:self.calibration_seq=value['seq']
    def on_command_ack(self,value):
        if self.speed_focus_pending and value['seq']==self.speed_seq:
            self.speed_focus_pending=False;self.speed_seq=None;self.release_before_drive=True
            if self.manual_armed.get() and self.held_input.foreground():self.root.focus_set()
        if value['seq']==self.calibration_seq:
            self.calibration_seq=None
            if value['status']==0:self.radar_view.set('场地坐标');self.note('建系请求已确认：请保持静止，等待四边拟合及定位稳定。')
    def calibrate_frame(self):
        if not self.available('calibrate'):return
        self.manual_armed.set(False);self.click_go.set(False);self.link.set_drive(0)
        self.send([(DRIVE,0,0,0),(CALIBRATE,0,0,0)])
        if self.demo:self.radar_view.set('场地坐标')
    def apply_obstacle(self):
        try:
            fb=cm_to_mm(self.obstacle_fb.get());lr=cm_to_mm(self.obstacle_lr.get())
            choices=['窄 ±10.6°','中 ±20.6°','宽 ±29.4°'];mode=choices.index(self.obstacle_cone.get())+1
            if not 200<=fb<=800 or not 150<=lr<=600:raise ValueError('前后20～80cm，左右15～60cm')
        except ValueError as exc:messagebox.showerror('阻挡范围',str(exc));return
        if not self.demo and (not self.connected or not self.remote_ready):return
        self.link.set_drive(0);self.release_before_drive=True;self.speed_focus_pending=not self.demo;self.speed_seq=None
        self.send([(DRIVE,0,0,0),(OBSTACLE,fb,lr,mode)])
    def on_link_lost(self):
        self.speed_focus_pending=False;self.speed_seq=None;self.release_before_drive=True
    def stop(self):
        self.manual_armed.set(False);self.click_go.set(False);self.link.set_drive(0);super().stop()
    def close(self):
        self.link.set_drive(0);super().close()
    def ingest(self,kind,value):
        if kind=='scan':self.cloud=value;self.cloud_time=time.monotonic();return
        super().ingest(kind,value)
    def render(self):
        super().render()
        if self.task.get('manual'):self.variable('task_state').set('手动遥控中')
        self.variable('radar_state').set('有效扫描' if self.task.get('manual_range_fresh') else '等待数据')
        mask=self.task.get('blocked');names=[] if mask is None else [n for i,n in enumerate(['前','后','左','右']) if mask&(1<<i)]
        for i,key in enumerate(['front','back','left','right']):
            self.variable(key).set('等待扫描' if not self.task.get('manual_range_fresh') or mask is None else '受阻' if mask&(1<<i) else '可通行')
        self.variable('blocked_axes').set('方向阻挡：'+('、'.join(names) if names else '无') if mask is not None else '方向阻挡：等待数据')
        t=self.task
        self.variable('drive_actual').set(f'实际目标 F {t.get("applied_forward",0):+d} / L {t.get("applied_left",0):+d} / 转向 {t.get("applied_turn",0):+d}')
        live=self.cloud and time.monotonic()-self.cloud_time<.7
        self.variable('scan_state').set(f'点云 #{self.cloud.get("sequence",0)} · {len(self.cloud.get("points",[]))}/180点 · '+('模拟数据' if self.demo else '实时') if live else '点云待更新 · 不显示过期回波')
        fb=t.get('stop_fb_cm',40);lr=t.get('stop_lr_cm',25);mode=t.get('cone_mode',2)
        self.variable('obstacle_actual').set(f'已生效：前后{fb:g}cm / 左右{lr:g}cm\n半角 ±{CONE_ANGLES.get(mode,20.56):.1f}°，从雷达中心起算' if 'cone_mode' in t else '等待实际阻挡参数，未显示为已应用')
        reason=FIT_REASONS.get(t.get('fit_reason',0),'等待边框状态')
        self.variable('fit_state').set(reason+f'\n边框：{t.get("width_cm",0):.1f} × {t.get("height_cm",0):.1f}cm；拟合分数{t.get("fit_quality",0)}')
        blockers=navigation_blockers(t,self.att)
        self.variable('nav_reason').set('可出发：切到场地坐标图后点选目标' if t.get('can_go') and not blockers else '未就绪：\n'+'\n'.join(blockers))
    def draw_world(self,c):
        fresh=self.cloud and time.monotonic()-self.cloud_time<.7
        body=getattr(c,'_body_radar',False) and (self.radar_view.get()=='车体雷达' or not (self.task.get('calibrated') and self.task.get('pose')))
        c._body_display=bool(body)
        if body:
            limit=int(self.radar_radius.get()) if self.radar_radius.get() in ('100','200','300','500') else 300
            c.delete('all');w=max(c.winfo_width(),100);h=max(c.winfo_height(),100);cx=w/2;cy=h/2;radius=max(20,min(w,h)/2-32);scale=radius/limit
            c._body_transform=(cx,cy,scale,limit)
            for d in range(50 if limit<=300 else 100,limit+1,50 if limit<=300 else 100):
                q=d*scale;c.create_oval(cx-q,cy-q,cx+q,cy+q,outline=LINE);c.create_text(cx+q,cy+10,text=str(d),fill=MUTED,font=('Consolas',8))
            c.create_line(cx,25,cx,h-25,fill=LINE);c.create_line(25,cy,w-25,cy,fill=LINE)
            if 'cone_mode' in self.task:
                for bit,points in self.obstacle_polygons():
                    coords=[]
                    for f,l in points:coords.extend((cx-l*scale,cy-f*scale))
                    color=RED if self.task.get('blocked',0)&(1<<bit) else AMBER
                    c.create_polygon(*coords,fill=color,stipple='gray25',outline=color,width=1,tags='obstacle_zone')
            if fresh:
                for angle,distance in self.cloud['points']:
                    if distance>limit:continue
                    f,l=body_point(angle,distance);x=cx-l*scale;y=cy-f*scale
                    c.create_oval(x-2,y-2,x+2,y+2,fill=ACCENT,outline='')
            c.create_polygon(cx,cy-12,cx-8,cy+8,cx+8,cy+8,fill=BLUE)
            c.create_text(cx,15,text=f'车头 ↑ · 车体坐标 · {limit}cm半径',fill=MUTED,font=SMALL)
            c.create_text(cx,h-15,text='任务3：区域只限制对应方向；任务4需切场地坐标',fill=AMBER,font=SMALL);return
        super().draw_world(c)
        if fresh and self.task.get('pose') and self.task.get('calibrated'):
            ox,oy,scale,bw,bh=c._world_transform
            for angle,distance in self.cloud['points']:
                x,y=world_point(angle,distance,self.task['x_cm'],self.task['y_cm'],self.task['yaw_deg'])
                if 0<=x<=bw and 0<=y<=bh:
                    sx=ox+x*scale;sy=oy-y*scale;c.create_oval(sx-2,sy-2,sx+2,sy+2,fill=ACCENT,outline='')
    def map_click(self,c,event):
        if self.manual_armed.get():self.root.focus_set();self.release_before_drive=True
        if getattr(c,'_body_display',False):
            self.note('这是车体雷达图；请先静止建系，再切换场地坐标图设目标。',True);return
        if not self.task.get('pose') or not self.task.get('calibrated'):
            self.note('当前是车体雷达图；请先静止校准边框并取得有效定位。',True);return
        if self.task.get('active'):self.note('任务运行中，先停止再选目标。',True);return
        if not hasattr(c,'_world_transform'):return
        ox,oy,scale,bw,bh=c._world_transform;x=(event.x-ox)/scale;y=(oy-event.y)/scale
        if not (30<=x<=bw-29.5 and 30<=y<=bh-29.5):self.note('目标应在边框内，并离边界至少约30cm。',True);return
        super().map_click(c,event)
        if self.click_go.get():
            if self.available('go') and self.cloud and time.monotonic()-self.cloud_time<.7:self.goal(True)
            else:self.note('目标已选择；定位、点云或可出发条件未就绪，未发起运动。',True)
    def seed_demo(self):
        super().seed_demo();self.task.update(manual=False,manual_range_fresh=True,drive_lease=False,blocked=0,applied_forward=0,applied_left=0,applied_turn=0,
            stop_fb_cm=40,stop_lr_cm=25,cone_mode=2,fit_quality=180,fit_reason=9,map_near=False,nav_range_near=False,heading_fault=False,wheel_fault=False)
        self.seed_cloud();self.render()
    def seed_cloud(self):
        bins=[]
        for i in range(180):
            a=math.radians(90-i*2);dx=math.cos(a);dy=math.sin(a);candidates=[]
            if abs(dx)>.00001:candidates.append(((200 if dx>0 else 0)-73)/dx)
            if abs(dy)>.00001:candidates.append(((200 if dy>0 else 0)-86)/dy)
            bins.append(max(1,min(255,int(min(v for v in candidates if v>0)/2))))
        self.cloud=dict(sequence=1,bins=bins,points=[(i*2,v*2) for i,v in enumerate(bins)]);self.cloud_time=time.monotonic()
    def demo_tick(self):
        self.seed_cloud();super().demo_tick()
    def obstacle_polygons(self):
        fb=self.task.get('stop_fb_cm',40);lr=self.task.get('stop_lr_cm',25)
        tan={1:3/16,2:3/8,3:9/16}.get(self.task.get('cone_mode',2),3/8)
        return [(0,[(0,0),(fb,fb*tan),(fb,-fb*tan)]),(1,[(0,0),(-fb,-fb*tan),(-fb,fb*tan)]),
                (2,[(0,0),(-lr*tan,lr),(lr*tan,lr)]),(3,[(0,0),(lr*tan,-lr),(-lr*tan,-lr)])]
    def preview_commands(self,commands):
        super().preview_commands(commands)
        for op,a,b,c in commands:
            if op==OBSTACLE:self.task.update(stop_fb_cm=a/10,stop_lr_cm=b/10,cone_mode=c)
        self.render()

def main():
    ap=argparse.ArgumentParser();ap.add_argument('--demo',action='store_true');ap.add_argument('--smoke',action='store_true');args=ap.parse_args()
    root=tk.Tk();app=App(root,demo=args.demo,enumerate_ports=not args.smoke and not args.demo)
    if args.smoke:
        root.withdraw();root.update_idletasks();assert app.link.thread is None and not app.connected
        for page in app.pages:app.show(page);root.update_idletasks()
        root.destroy();print('PASS GUI: all six pages render, no serial connection and no automatic commands')
    else:root.mainloop()

if __name__=='__main__':main()

