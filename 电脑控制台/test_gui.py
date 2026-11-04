import unittest,tkinter as tk,time,types
from unittest.mock import Mock,patch
from controller import App
from field_navigation import FieldSnapshot,Field
from protocol import SPEED,CALIBRATE,GO

class FieldGuiTests(unittest.TestCase):
 def setUp(self):
  self.root=tk.Tk();self.root.withdraw();self.a=App(self.root,demo=True,enumerate_ports=False);self.root.update_idletasks()
  self.assertIsNone(self.a.worker.thread);self.a.worker=types.SimpleNamespace(command=Mock(),close=Mock(),thread=None)
 def tearDown(self):
  self.assertIsNone(self.a.link.thread)
  for ident in self.root.tk.splitlist(self.root.tk.call('after','info')):self.root.after_cancel(ident)
  self.root.destroy()
 def ready(self):
  a=self.a;a.demo=False;a.connected=True;a.remote_ready=True;a.last_task=time.monotonic();a.current='radar'
  a.task.update(calibrated=False,pose=False,can_go=False,near=True,radar=False,active=False,manual=False)
  a.local=FieldSnapshot(pose=(165,80,0),valid=True,stable=5,stamp=time.monotonic(),frame=Field(330,260),reason='边框定位有效',rms=1,support=100)
  a.held_input=types.SimpleNamespace(foreground=lambda:True,mask=lambda:1);return a
 def test_reference_matches_user_image_scale(self):
  self.assertEqual((self.a.expected_w.get(),self.a.expected_h.get()),('330','260'))
 def test_no_frame_cannot_start(self):
  a=self.ready();a.local.frame=None;self.assertFalse(a.available('local_go'));a.goal(True);self.assertFalse(a.local_run)
 def test_calibration_not_gated_on_old_fpga_frame(self):
  a=self.ready();a.local=FieldSnapshot();self.assertTrue(a.available('field_calibrate'));a.calibrate_field()
  self.assertEqual(a.worker.command.call_args_list[-1].args,('calibrate',(330.,260.)));self.assertFalse(a.local_run)
 def test_bad_size_not_sent(self):
  a=self.ready();a.expected_w.set('nan')
  with patch('controller.messagebox.showerror') as err:a.calibrate_field();err.assert_called_once()
  a.worker.command.assert_not_called()
 def test_calibration_cancels_click_arm_and_goal(self):
  a=self.ready();a.click_go.set(True);a.local_goal=(100,100);a.calibrate_field();self.assertFalse(a.click_go.get());self.assertIsNone(a.local_goal)
 def test_goal_only_uses_pc_worker(self):
  a=self.ready();a.send=Mock();a.goal(True);self.assertTrue(a.local_run);a.send.assert_not_called()
  self.assertEqual(a.worker.command.call_args_list[-1].args,('start',))
 def test_goal_outside_frame_rejected(self):
  a=self.ready();a.goal_x.set('500')
  with patch('controller.messagebox.showerror') as err:a.goal(True);err.assert_called_once()
  self.assertFalse(a.local_run)
 def test_field_map_click_preserves_fixed_coordinates(self):
  a=self.ready();c=a.worlds[-1];a.draw_world(c);ox,oy,scale,_=c._local_transform
  a.map_click(c,types.SimpleNamespace(x=ox+120*scale,y=oy-170*scale));self.assertEqual(a.local_goal,(120,170));self.assertFalse(a.local_run)
 def test_actual_frame_outline_drawn(self):
  a=self.ready();c=a.worlds[-1];a.draw_world(c);self.assertEqual(len(c.find_withtag('field_frame')),1)
 def test_continuous_decision_survives_old_180ms_cutoff(self):
  a=self.ready();now=time.monotonic();a.local_run=True;a.local_started=now-.3;a.local.stamp=now-.22;a.local.mask=1;a.local.until=now+.16
  a.local.running=True;a.ingest('local_nav',a.local);a.ingest('drive_sent',dict(mask=1,time=now,seq=1))
  a._poll_drive_once();self.assertEqual(a.link.drive_state[0],1)
 def test_old_decision_before_restart_cannot_move(self):
  a=self.ready();now=time.monotonic();a.local_run=True;a.local_started=now;a.local.stamp=now-.1;a.local.mask=1;a.local.until=now+.28
  a._poll_drive_once();self.assertEqual(a.link.drive_state[0],0)
 def test_stale_scan_cancels(self):
  a=self.ready();a.local_run=True;a.local.stamp=time.monotonic()-1;a._poll_drive_once();self.assertFalse(a.local_run)
 def test_focus_loss_does_not_cancel_automatic_navigation(self):
  a=self.ready();a.local_run=True;a.held_input.foreground=lambda:False;a._poll_drive_once();self.assertTrue(a.local_run)
 def test_speed_apply_keeps_manual_and_waits_for_release(self):
  a=self.ready();a.manual_armed.set(True);a.root.focus_set=Mock();a.apply_speed();a.on_command_sent(dict(seq=10,op=SPEED));a.on_command_ack(dict(seq=10,status=0));a.pending=False
  self.assertTrue(a.manual_armed.get());a.root.focus_set.assert_called_once();a._poll_drive_once();self.assertEqual(a.link.drive_state[0],0)
  a.held_input.mask=lambda:0;a._poll_drive_once();a.held_input.mask=lambda:1;a._poll_drive_once();self.assertEqual(a.link.drive_state[0],1)
 def test_legacy_fpga_commands_still_not_used(self):
  a=self.ready();a.link.submit=Mock();a.send([(CALIBRATE,0,0,0)]);a.send([(GO,0,0,0)]);a.link.submit.assert_not_called()
 def test_stop_cancels_both_modes(self):
  a=self.ready();a.local_run=True;a.manual_armed.set(True);a.stop();self.assertFalse(a.local_run);self.assertFalse(a.manual_armed.get());self.assertEqual(a.link.drive_state[0],0)
 def test_success_switches_to_goal_tab_without_start(self):
  a=self.ready();a.had_frame=False;a.render()
  for tabs,goal in a.field_tabs:self.assertEqual(tabs.select(),str(goal))
  self.assertFalse(a.local_run)

if __name__=='__main__':unittest.main(verbosity=2)
