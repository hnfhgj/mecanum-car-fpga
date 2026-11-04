import queue,threading,time,unittest
from protocol import *
from transport import Link

FIXTURE='425403F1546042FF9C01F4232803E803E807D003E805DC07D009C409C43997F4E1005AFFC40050'
def scan_line(bins=None):
 bins=bins if bins is not None else list(range(1,181));data=bytes.fromhex('1234')+bytes(bins)
 return 'SCAN HEX='+(data+(sum(data)&65535).to_bytes(2,'big')).hex().upper()

class ProtocolTests(unittest.TestCase):
 def test_frame(self):
  self.assertEqual(encode(35,INFO,FIRMWARE),bytes.fromhex('A55A01230E97F400000000BD0D0A'))
  p=encode(44,DRIVE,5);self.assertEqual(p[4],15);self.assertEqual(p[6],5);self.assertEqual(sum(p[2:11])&255,p[11])
 def test_invalid_parameters(self):
  for value in [-1,65536,1.2]:
   with self.assertRaises(ValueError):encode(1,GOAL,value)
 def test_units(self):
  self.assertEqual(cm_to_mm('123.4'),1234)
  for v in ['nan','Infinity','1.23','-1','abc']:
   with self.assertRaises(ValueError):cm_to_mm(v)
 def test_speed(self):
  self.assertEqual([speed_level(str(v)) for v in range(25,151,25)],list(range(1,7)))
 def test_telemetry(self):
  kind,t=parse_line('IO HEX='+FIXTURE)
  self.assertEqual(kind,'task');self.assertEqual(t['firmware'],FIRMWARE)
  self.assertEqual((t['x_cm'],t['y_cm'],t['yaw_deg']),(-10,50,90))
  self.assertEqual((t['manual'],t['manual_range_fresh'],t['drive_lease'],t['blocked']),(True,True,True,1))
  self.assertEqual((t['applied_forward'],t['applied_left'],t['applied_turn']),(90,-60,80))
 def test_scan(self):
  kind,s=parse_line(scan_line());self.assertEqual(kind,'scan');self.assertEqual(s['sequence'],0x1234)
  self.assertEqual(len(s['points']),180);self.assertEqual(s['points'][90],(180,182))
 def test_scan_checksum(self):
  line=scan_line();self.assertEqual(parse_line(line[:-1]+'0')[0],'text')
 def test_scan_missing_bins(self):
  bins=[0]*180;bins[45]=100;kind,s=parse_line(scan_line(bins));self.assertEqual(s['points'],[(90,200)])
 def test_body_axes(self):
  f,l=body_point(90,100);self.assertAlmostEqual(f,0);self.assertAlmostEqual(l,-100)
  self.assertAlmostEqual(body_point(0,100)[0],100)
 def test_world_axes(self):
  x,y=world_point(0,100,20,30,90);self.assertAlmostEqual(x,20);self.assertAlmostEqual(y,130)
  x,y=world_point(90,100,20,30,90);self.assertAlmostEqual(x,120);self.assertAlmostEqual(y,30)
 def test_go_not_sent_after_reject(self):
  q=CommandQueue();q.add([(GOAL,500,600,0),(GO,0,0,0)]);p=q.next_packet(0)
  q.acknowledge(p[3],2);self.assertFalse(q.items)
 def test_stop_cancels_pending(self):
  q=CommandQueue();q.add([(GO,0,0,0)]);q.next_packet(0);q.emergency_stop();self.assertEqual(q.next_packet(1)[4],STOP)
 def test_retries(self):
  q=CommandQueue();q.add([(GO,0,0,0)]);p=q.next_packet(0);self.assertEqual(q.retry(1),p);self.assertEqual(q.retry(2),p)
  with self.assertRaises(TimeoutError):q.retry(3)
 def test_legacy_cannot_claim_new_capability(self):
  kind,t=parse_line('IO HEX=425402'+FIXTURE[6:60]+'97F0')
  self.assertNotEqual(t.get('firmware') if isinstance(t,dict) else None,FIRMWARE)

class FakeSerial:
 def __init__(self,status=0):self.status=status;self.is_open=False;self.received=bytearray();self.writes=[];self.lock=threading.Lock()
 def open(self):self.is_open=True
 @property
 def in_waiting(self):return len(self.received)
 def write(self,data):
  self.writes.append(data)
  if data[4]!=HEARTBEAT:
   self.received.extend(f'@{data[3]:02X}:{self.status:02X}\r\n'.encode())
   if data[4]==INFO and self.status==0:self.received.extend(('IO HEX='+FIXTURE+'\r\n').encode())
  return len(data)
 def read(self,n):
  data=bytes(self.received[:n]);del self.received[:n];return data
 def close(self):self.is_open=False

class WorkerTests(unittest.TestCase):
 def start(self,status=0):
  q=queue.Queue();dev=FakeSerial(status);link=Link(q,lambda:dev);link.connect('OFFLINE_FAKE')
  if status==0:
   deadline=time.monotonic()+1
   while time.monotonic()<deadline:
    try:kind,value=q.get(timeout=.1)
    except queue.Empty:continue
    if kind=='task':break
   else:self.fail('No task telemetry')
  return q,dev,link
 def finish(self,link):link.close_event.set();link.thread.join(1);self.assertFalse(link.thread.is_alive())
 def test_no_automatic_movement(self):
  q,dev,link=self.start()
  try:self.assertFalse(any(p[4] in (MOTION,GO,ROUTE,APPROACH,DRIVE) for p in dev.writes))
  finally:self.finish(link)
  self.assertEqual(dev.writes[-1][4],STOP)
 def test_old_firmware_rejected(self):
  q,dev,link=self.start(5);link.thread.join(1);self.assertFalse(dev.is_open)
  self.assertFalse(any(p[4]==DRIVE for p in dev.writes))
 def test_live_state_and_release(self):
  q,dev,link=self.start()
  try:
   end=time.monotonic()+.22
   while time.monotonic()<end:link.set_drive(5);time.sleep(.02)
   link.set_drive(0);time.sleep(.04)
   frames=[p for p in dev.writes if p[4]==DRIVE]
   self.assertGreaterEqual(len(frames),3);self.assertEqual(frames[-1][6],0);self.assertTrue(any(p[6]==5 for p in frames))
   self.assertEqual(len({p[3] for p in frames}),len(frames))
  finally:self.finish(link)
 def test_ui_freeze_releases_state(self):
  q,dev,link=self.start()
  try:
   link.set_drive(1);time.sleep(.26)
   frames=[p for p in dev.writes if p[4]==DRIVE]
   self.assertTrue(any(p[6]==1 for p in frames));self.assertEqual(frames[-1][6],0)
  finally:self.finish(link)
 def test_invalid_state(self):
  link=Link(queue.Queue())
  for m in [-1,64,'w']:
   with self.assertRaises(ValueError):link.set_drive(m)
 def test_speed_waits_for_release_ack_without_duplicate_zero(self):
  q,dev,link=self.start()
  try:
   link.set_drive(1);time.sleep(.08);offset=len(dev.writes)
   link.set_drive(0);link.submit([(DRIVE,0,0,0),(SPEED,5,0,0)]);time.sleep(.15)
   frames=[p for p in dev.writes[offset:] if p[4]!=HEARTBEAT]
   self.assertEqual([p[4] for p in frames],[DRIVE,SPEED]);self.assertEqual(frames[0][6],0)
   self.assertNotEqual(frames[0][3],frames[1][3])
  finally:self.finish(link)

if __name__=='__main__':unittest.main(verbosity=2)
