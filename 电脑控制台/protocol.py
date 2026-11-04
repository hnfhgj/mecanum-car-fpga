"""Pure protocol/model helpers. Importing never opens serial or starts a task."""
import re
import struct
import math
from collections import deque
from decimal import Decimal,InvalidOperation

STOP,PAGE,MOTION,APPROACH,CALIBRATE,GOAL,GO,POINT,ROUTE,GAINS,MENU,HEARTBEAT=range(12)
HELLO,SPEED,INFO=12,13,14
FIRMWARE=0x97F4
DRIVE=15
OBSTACLE=16
CONE_ANGLES={1:10.62,2:20.56,3:29.36}
FIT_REASONS={0:'尚未建系',1:'等待完整扫描，保持静止',2:'正在拟合四边框，保持静止',3:'四边框未识别：检查四边可见及边长80～300cm',4:'校准期间发生转动，请保持静止后重试',5:'扫描点数超过缓冲容量',6:'有效测距点不足120个',7:'雷达数据中断或校验异常',9:'边框定位有效'}
STATUS={0:'指令已接受',1:'任务运行中，请先停止再修改',2:'参数或坐标超出允许范围',3:'传感器或定位尚未就绪',4:'板上按键优先',5:'固件不支持此指令',6:'指令序号冲突，请重新操作'}
TASK_STATUS={0:'空闲',1:'等待准备',2:'未就绪',3:'可以出发',4:'运行中',5:'已结束，核对结果',6:'条件失效 / 超时',7:'已停止',8:'近障停车'}
PAGES=['编码器与姿态','航向参数','原始IMU','控制诊断','摄像头','实时雷达']

def encode(seq,op,a=0,b=0,c=0):
    if not 0<=seq<=255 or not 0<=op<=255 or any(not isinstance(v,int) or not 0<=v<=65535 for v in (a,b,c)):
        raise ValueError('指令参数必须为范围内整数')
    body=struct.pack('>BBBHHH',1,seq,op,a,b,c)
    return b'\xa5\x5a'+body+bytes([sum(body)&255])+b'\r\n'

def cm_to_mm(value):
    try:number=Decimal(str(value))*10
    except InvalidOperation:raise ValueError('坐标请输入数字')
    if not number.is_finite() or number!=number.to_integral_value() or not 0<=number<=65535:
        raise ValueError('坐标精度为1毫米，请输入有效范围内的厘米数值')
    return int(number)

def speed_level(percent):
    try:value=int(percent)
    except (ValueError,TypeError):raise ValueError('速度倍率必须为25%～150%，每档25%')
    if str(value)!=str(percent).strip() or value not in range(25,151,25):raise ValueError('速度倍率必须为25%～150%，每档25%')
    return value//25

def _legacy_parse_line(line):
    line=line.strip()
    match=re.fullmatch(r'@([0-9A-Fa-f]{2}):([0-9A-Fa-f]{2})',line)
    if match:return 'ack',(int(match[1],16),int(match[2],16))
    if re.fullmatch(r'IO HEX=(?:[0-9A-Fa-f]{64}|[0-9A-Fa-f]{42})',line):
        data=bytes.fromhex(line[7:])
        if data[:3] not in (b'BT\x01',b'BT\x02'):return 'text',line
        if (data[:3]==b'BT\x02')!=(len(data)==32):return 'text',line
        flags=data[3];x,y,yaw,tx,ty,front,back=struct.unpack('>hhhHHHH',data[7:21])
        result=dict(active=bool(flags&2),calibrated=bool(flags&32),radar=bool(flags&16),pose=bool(flags&8),can_go=bool(flags&4),
                    demo_running=bool(flags&128),mission_running=bool(flags&64),heading_ready=bool(flags&1),
                    demo=data[4]>>4,mission=data[4]&15,waypoint=1+((data[5]>>2)&3),route=bool(data[5]&16),group=data[5]>>5,
                    x_cm=x/10,y_cm=y/10,yaw_deg=yaw/100,goal_x_cm=tx/10,goal_y_cm=ty/10,front_cm=front/10,back_cm=back/10,
                    speed_percent=((data[6]>>4)&7)*25 if data[:3]==b'BT\x02' else None)
        if len(data)==32:
            left,right,width,height=struct.unpack('>HHHH',data[21:29]);aux=data[29]
            result.update(left_cm=left/10,right_cm=right/10,width_cm=width/10,height_cm=height/10,
                          nav=aux>>4,slow_seen=bool(aux&8),stop_seen=bool(aux&4),near=bool(aux&2),imu_fresh=bool(aux&1),
                          firmware=int.from_bytes(data[30:32],'big'))
        return 'task',result
    if line.startswith('ATT '):
        values=dict(re.findall(r'([RPY])=([+-]\d+)',line));return 'attitude',{**{k:int(v)/100 for k,v in values.items()},'ready':'OK=1' in line}
    if line.startswith('LF='):
        rows=re.findall(r'(LF|RF|LB|RB)=([+-]\d+) V=([+-]\d+)',line)
        if len(rows)==4:return 'wheels',{k:dict(count=int(c),speed=int(s)) for k,c,s in rows}
    if line.startswith('IMU_INIT='):
        return 'sensors',{k:bool(int(v)) for k,v in re.findall(r'(IMU_INIT|IMU_NEW|TOF_INIT|TOF_NEW)=([01])',line)}
    if line.startswith('AX='):
        return 'imu',{k:int(v) for k,v in re.findall(r'([AG][XYZ])=([+-]\d+)',line)}
    if line.startswith('PID '):
        m=re.search(r'RUN=([01]{4}) FAULT=([01]{4})',line)
        if m:return 'pid',dict(run=m[1],fault=m[2])
    return 'text',line

def _bt03_parse_line(line):
    line=line.strip()
    match=re.fullmatch(r'SCAN HEX=([0-9A-Fa-f]{368})',line)
    if match:
        data=bytes.fromhex(match[1])
        if sum(data[:-2])&65535!=int.from_bytes(data[-2:],'big'):return 'text','点云校验失败，已丢弃'
        return 'scan',dict(sequence=int.from_bytes(data[:2],'big'),bins=list(data[2:-2]),
                           points=[(i*2,value*2) for i,value in enumerate(data[2:-2]) if value])
    match=re.fullmatch(r'IO HEX=([0-9A-Fa-f]{78})',line)
    if match:
        data=bytes.fromhex(match[1])
        if data[:3]!=b'BT\x03':return 'text',line
        kind,result=_legacy_parse_line('IO HEX='+(b'BT\x02'+data[3:32]).hex())
        f,l,u=struct.unpack('>hhh',data[33:39]);flags=data[32]
        result.update(manual=bool(flags&128),manual_range_fresh=bool(flags&64),drive_lease=bool(flags&32),blocked=flags&15,
                      applied_forward=f,applied_left=l,applied_turn=u)
        return kind,result
    return _legacy_parse_line(line)

def parse_line(line):
    match=re.fullmatch(r'IO HEX=([0-9A-Fa-f]{96})',line.strip())
    if match:
        data=bytes.fromhex(match[1])
        if data[:3]!=b'BT\x04':return 'text',line
        kind,t=_bt03_parse_line('IO HEX='+(b'BT\x03'+data[3:39]).hex())
        t.update(stop_fb_cm=int.from_bytes(data[39:41],'big')/10,stop_lr_cm=int.from_bytes(data[41:43],'big')/10,
                 cone_mode=data[43],fit_quality=int.from_bytes(data[44:46],'big'),fit_reason=data[46],
                 map_near=bool(data[47]&1),nav_range_near=bool(data[47]&2),heading_fault=bool(data[47]&4),wheel_fault=bool(data[47]&8))
        return kind,t
    return _bt03_parse_line(line)

def navigation_blockers(task,attitude):
    reasons=[]
    if task.get('active'):reasons.append('任务正在运行，先停止')
    if not task.get('calibrated'):reasons.append('未建系：'+FIT_REASONS.get(task.get('fit_reason',0),'边框未识别'))
    elif not task.get('pose'):reasons.append('场地定位暂未有效：'+FIT_REASONS.get(task.get('fit_reason',0),'保持四边可见'))
    if not task.get('imu_fresh') or not attitude.get('ready'):reasons.append('IMU未就绪或数据过期')
    if task.get('calibrated') and not task.get('heading_ready'):reasons.append('航向参考初始化中，请保持静止')
    if task.get('heading_fault'):reasons.append('导航航向数据异常，请检查IMU')
    if task.get('wheel_fault'):reasons.append('轮速控制状态异常')
    if task.get('near') or task.get('nav_range_near') or task.get('map_near'):reasons.append('车身附近27cm内有导航近障，先用任务3移到开阔位置')
    if not task.get('radar'):reasons.append('导航四向测距未就绪，检查四边回波')
    if not reasons and not task.get('can_go'):reasons.append('等待连续定位稳定，或检查目标是否距边界至少30cm')
    return reasons

def body_point(angle_deg,range_cm):
    a=math.radians(angle_deg)
    return range_cm*math.cos(a),-range_cm*math.sin(a)

def world_point(angle_deg,range_cm,x_cm,y_cm,yaw_deg):
    f,l=body_point(angle_deg,range_cm);a=math.radians(yaw_deg)
    return x_cm+f*math.cos(a)-l*math.sin(a),y_cm+f*math.sin(a)+l*math.cos(a)

class CommandQueue:
    def __init__(self):self.items=deque();self.pending=None;self.seq=0
    def add(self,commands):
        if self.items or self.pending:raise ValueError('上一组指令尚未确认，请等待或停止')
        self.items.extend(commands)
    def emergency_stop(self):self.items.clear();self.pending=None;self.items.append((STOP,0,0,0))
    def next_packet(self,now):
        if self.pending or not self.items:return None
        self.seq=(self.seq+1)&255;packet=encode(self.seq,*self.items.popleft())
        self.pending=dict(seq=self.seq,packet=packet,last=now,retries=0);return packet
    def acknowledge(self,seq,status):
        if not self.pending or seq!=self.pending['seq']:return False
        self.pending=None
        if status:self.items.clear()
        return True
    def retry(self,now):
        if not self.pending or now-self.pending['last']<1:return None
        if self.pending['retries']>=2:self.items.clear();self.pending=None;raise TimeoutError('指令确认超时，已取消后续操作')
        self.pending['last']=now;self.pending['retries']+=1;return self.pending['packet']
