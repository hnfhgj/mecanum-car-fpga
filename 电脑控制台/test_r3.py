import unittest,time,types,tempfile,json,queue,tkinter as tk
from pathlib import Path
from unittest.mock import Mock
from controller import App
from field_navigation import FieldSnapshot,Field
from test_field import ready,field_scan,POSE
import test_gui as gui_base
from test_console import FakeSerial
from protocol import DRIVE,INFO
from session_trace import SessionTrace

class TimingTests(unittest.TestCase):
 setUp=gui_base.FieldGuiTests.setUp
 tearDown=gui_base.FieldGuiTests.tearDown
 ready=gui_base.FieldGuiTests.ready
 # Inherits the established GUI regressions, plus exact dead-pulse cases.
 def decision(self,mask=32,age=.09,duration=.12):
  a=self.ready();now=time.monotonic();a.local_run=True;a.local_started=now-.5
  a.local.stamp=now-.4
  s=FieldSnapshot(pose=(165,80,0),frame=Field(330,260),valid=True,stable=5,stamp=now-age,sequence=10,mask=mask,until=now-age+.06,duration=duration,running=True)
  a.ingest('local_nav',s);return a,now,s
 def test_expired_scan_based_rotation_still_gets_first_serial_write(self):
  a,now,s=self.decision();self.assertLess(s.until,now);a._poll_drive_once();self.assertEqual(a.link.drive_state[0],32)
 def test_timer_starts_on_actual_write_not_on_scan(self):
  a,now,s=self.decision();a.ingest('drive_sent',dict(mask=32,time=now,seq=5))
  self.assertAlmostEqual(a.pulse_deadline,now+.12,delta=.01);a._poll_drive_once();self.assertEqual(a.link.drive_state[0],32)
 def test_repeated_sends_do_not_extend_single_pulse(self):
  a,now,s=self.decision();a.ingest('drive_sent',dict(mask=32,time=now,seq=5));end=a.pulse_deadline
  a.ingest('drive_sent',dict(mask=32,time=now+.06,seq=6));self.assertEqual(a.pulse_deadline,end)
 def test_expired_actual_pulse_releases(self):
  a,now,s=self.decision();a.pulse_deadline=now-.01;a._poll_drive_once();self.assertEqual(a.link.drive_state[0],0)
 def test_wrong_mask_write_cannot_arm_pulse(self):
  a,now,s=self.decision();a.ingest('drive_sent',dict(mask=4,time=now,seq=5));self.assertIsNone(a.pulse_deadline)
 def test_global_escape_cancels_background_automatic_task(self):
  a,_,_=self.decision();a.held_input.foreground=lambda:False;a.held_input.emergency_stop=lambda:True
  a._poll_drive_once();self.assertFalse(a.local_run);self.assertEqual(a.link.drive_state[0],0)
 def test_manual_still_stops_on_focus_loss(self):
  a=self.ready();a.manual_armed.set(True);a.held_input.foreground=lambda:False;a._poll_drive_once();self.assertFalse(a.manual_armed.get())
 def test_telemetry_and_logs_pages_do_not_cancel_auto(self):
  a,_,_=self.decision();a.show('logs');self.assertTrue(a.local_run);a.show('overview');self.assertTrue(a.local_run)
 def test_nack_is_visible_and_stops(self):
  a,_,_=self.decision();a.ingest('drive_ack',dict(mask=32,status=3,seq=5,time=time.monotonic()))
  self.assertFalse(a.local_run);self.assertIn('拒绝',a.last_stop)
 def test_specific_rejection_not_overwritten_by_generic_worker_stop(self):
  a,now,s=self.decision();a.ingest('drive_ack',dict(mask=32,status=3,seq=5,time=now))
  stopped=FieldSnapshot(frame=Field(330,260),valid=True,stable=5,stamp=now+.01,route_mode='已停止',reason='用户停止')
  a.ingest('local_nav',stopped);self.assertIn('拒绝',a.last_stop)
 def test_stop_reason_survives_later_valid_frames(self):
  a,now,s=self.decision();s.running=False;s.route_mode='已停止';s.reason='测试停止原因';s.stamp=now;a.ingest('local_nav',s)
  s2=FieldSnapshot(frame=Field(330,260),valid=True,stable=5,stamp=now+.01,route_mode='待出发');a.ingest('local_nav',s2);a.render()
  self.assertEqual(a.variable('route_choice').get(),'测试停止原因')

class ModelRecoveryTests(unittest.TestCase):
 def test_heading_over_twenty_corrects_instead_of_canceling(self):
  n=ready();n.select((165,210));n.start(10.9);s=n.update(field_scan((165,80,35)),35,11.08,4)
  self.assertTrue(s.valid,s.reason);self.assertTrue(s.running);self.assertEqual(s.route_mode,'航向纠正');self.assertEqual(s.mask,32);self.assertGreaterEqual(s.duration,.12)
 def test_stop_reason_is_latched_across_good_tracking(self):
  n=ready();n.select((165,210));n.start(10.9);n.stop('用户停止测试')
  for i in range(3):s=n.update(field_scan(),0,11.08+i*.27,4+i)
  self.assertEqual(s.route_mode,'已停止');self.assertEqual(s.reason,'用户停止测试');self.assertEqual(s.mask,0)
 def test_single_bad_scan_pauses_then_resumes_original_goal(self):
  n=ready();n.select((165,210));n.start(10.9);s=n.update([],0,11.08,4)
  self.assertTrue(s.running);self.assertEqual(s.mask,0)
  for i in range(3):s=n.update(field_scan(),0,11.35+i*.27,5+i)
  self.assertTrue(s.running);self.assertNotEqual(s.mask,0);self.assertEqual(n.target,(165,210))

class TraceTests(unittest.TestCase):
 def test_machine_readable_scan_command_and_stop_records(self):
  test_root=Path(__file__).resolve().parent/'test_work';test_root.mkdir(exist_ok=True)
  with tempfile.TemporaryDirectory(dir=test_root) as d:
   self.assertTrue(Path(d).resolve().is_relative_to(test_root.resolve()))
   log=SessionTrace(d);log.write('local_nav',FieldSnapshot(reason='测试'));log.write('stop',dict(reason='用户停止'));log.close()
   rows=[json.loads(s) for s in log.path.read_text(encoding='utf-8').splitlines()]
   self.assertEqual([r['event'] for r in rows],['session','local_nav','stop']);self.assertEqual(rows[1]['data']['reason'],'测试')

class EndToEndTests(unittest.TestCase):
 def test_delayed_gui_rotation_reaches_fake_serial_and_ack(self):
  root=tk.Tk();root.withdraw();a=App(root,demo=True,enumerate_ports=False);a.worker.close();a.demo=False
  device=FakeSerial();writes=[];original=device.write
  def write(data):writes.append((time.monotonic(),bytes(data)));return original(data)
  device.write=write;a.link.factory=lambda:device;a.held_input=types.SimpleNamespace(foreground=lambda:False,mask=lambda:0,emergency_stop=lambda:False)
  try:
   a.link.connect('OFFLINE_FAKE');end=time.monotonic()+1
   while time.monotonic()<end:
    root.update();time.sleep(.005)
    if a.remote_ready and a.task:break
   self.assertTrue(a.remote_ready)
   now=time.monotonic();a.local_run=True;a.local_started=now-.4
   s=FieldSnapshot(pose=(165,80,8),frame=Field(330,260),valid=True,stable=5,stamp=now-.09,sequence=99,mask=32,until=now-.03,duration=.12,running=True)
   a.ingest('local_nav',s);end=time.monotonic()+.30
   while time.monotonic()<end:root.update();time.sleep(.005)
   commands=[(t,p[6]) for t,p in writes if p[4]==DRIVE]
   self.assertTrue(any(mask==32 for _,mask in commands),commands)
   self.assertEqual(commands[-1][1],0);self.assertTrue(a.local_run);self.assertEqual(a.drive_ack_text,'已接受')
   first=next(t for t,mask in commands if mask==32);release=next(t for t,mask in commands if t>first and mask==0)
   self.assertGreaterEqual(release-first,.10)
  finally:
   a.worker.close();a.link.close_event.set();a.link.thread.join(1)
   for ident in root.tk.splitlist(root.tk.call('after','info')):root.after_cancel(ident)
   root.destroy()

if __name__=='__main__':unittest.main(verbosity=2)
