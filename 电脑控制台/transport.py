"""Serial worker. Connect/INFO never starts movement; only explicit submissions do."""
import queue,secrets,threading,time
from protocol import *

class Link:
    def __init__(self,events,serial_factory=None):
        self.events=events;self.factory=serial_factory;self.actions=queue.Queue()
        self.drive_state=(0,0.0);self.stop_event=threading.Event();self.close_event=threading.Event();self.thread=None
    def connect(self,port):
        if self.thread and self.thread.is_alive():raise ValueError('连接正在使用中')
        self.close_event.clear();self.stop_event.clear()
        while not self.actions.empty():self.actions.get_nowait()
        self.thread=threading.Thread(target=self.run,args=(port,),daemon=True);self.thread.start()
    def submit(self,commands):self.actions.put(commands)
    def set_drive(self,mask):
        if not isinstance(mask,int) or not 0<=mask<=63:raise ValueError('无效遥控按键状态')
        self.drive_state=(mask,time.monotonic())
    def run(self,port):
        import serial
        device=None;commands=CommandQueue();commands.seq=secrets.randbelow(256)
        try:
            device=self.factory() if self.factory else serial.Serial()
            device.port=port;device.baudrate=115200;device.timeout=0;device.write_timeout=.3
            device.dtr=False;device.rts=False;device.xonxoff=False;device.rtscts=False;device.dsrdtr=False;device.open()
            commands.add([(INFO,FIRMWARE,0,0)]);ready=False;last_heartbeat=0;last_task=time.monotonic();buffer=bytearray()
            self.drive_state=(0,0.0);last_drive=0;last_mask=0;task_state={};drive_packets={}
            self.events.put(('connection',True));self.events.put(('ready',False))
            def write(packet):
                if device.write(packet)!=len(packet):raise IOError('串口未完整发送')
            while not self.close_event.is_set():
                now=time.monotonic()
                if self.stop_event.is_set():
                    self.stop_event.clear()
                    self.drive_state=(0,0.0);last_mask=0
                    while not self.actions.empty():self.actions.get_nowait()
                    commands.emergency_stop()
                try:
                    request=self.actions.get_nowait()
                    if not ready:raise ValueError('尚未完成97F4固件握手，不能执行任务')
                    commands.add(request)
                except queue.Empty:pass
                except ValueError as exc:self.events.put(('message',str(exc)));self.events.put(('pending',bool(commands.pending or commands.items)))
                if now-last_heartbeat>=.5:write(encode(0,HEARTBEAT));last_heartbeat=now
                packet=commands.next_packet(now)
                if packet:
                    write(packet)
                    if packet[4]==DRIVE and packet[5:7]==b'\x00\x00':last_mask=0;self.drive_state=(0,now)
                    self.events.put(('pending',True));self.events.put(('sent',dict(seq=packet[3],op=packet[4])))
                retry=commands.retry(now)
                if retry:write(retry)
                mask,refreshed=self.drive_state
                if now-refreshed>.15 or not ready or now-last_task>.6 or not task_state.get('manual_range_fresh') or not task_state.get('imu_fresh'):mask=0
                if ready and ((last_mask!=0 and mask==0) or (mask and not commands.pending and now-last_drive>=.06)):
                    commands.seq=(commands.seq+1)&255
                    write(encode(commands.seq,DRIVE,mask));last_drive=now;last_mask=mask
                    drive_packets[commands.seq]=(mask,now)
                    if len(drive_packets)>64:drive_packets.pop(next(iter(drive_packets)))
                    self.events.put(('drive_sent',dict(seq=commands.seq,mask=mask,time=now)))
                block=device.read(min(8192,max(1,device.in_waiting)))
                if block:buffer.extend(block)
                if len(buffer)>32768:raise IOError('接收流缺少完整行，请检查波特率或固件')
                while b'\n' in buffer:
                    line,_,tail=buffer.partition(b'\n');buffer=bytearray(tail)
                    text=line.decode('ascii',errors='replace').strip()
                    if not text:continue
                    kind,value=parse_line(text)
                    if kind=='ack':
                        seq,status=value;info=commands.pending and commands.pending['packet'][4]==INFO
                        if commands.acknowledge(seq,status):
                            if info and status==0:ready=True;last_task=now;self.events.put(('ready',True))
                            elif info:raise IOError('固件握手未通过，请使用配套97F4烧录文件')
                            self.events.put(('ack',dict(seq=seq,status=status)));self.events.put(('pending',bool(commands.items)))
                        elif seq in drive_packets:
                            mask,sent_time=drive_packets.pop(seq)
                            self.events.put(('drive_ack',dict(seq=seq,status=status,mask=mask,time=now,sent_time=sent_time)))
                    else:
                        if kind=='task' and value.get('firmware')==FIRMWARE:last_task=now;task_state=value
                        self.events.put((kind,value))
                if ready and now-last_task>2.5:raise IOError('有效97F4遥测超过2.5秒未更新，已断开并尝试停止任务')
                self.close_event.wait(.01)
        except Exception as exc:self.events.put(('message',str(exc)))
        finally:
            if device and getattr(device,'is_open',False):
                try:device.write(encode((commands.seq+1)&255,STOP))
                except Exception:pass
                device.close()
            self.events.put(('pending',False));self.events.put(('ready',False));self.events.put(('connection',False))
