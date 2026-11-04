import unittest,math,json
from pathlib import Path
import test_r3
from unittest.mock import patch
from field_navigation import FieldPlanner,Field,route_waypoint
from test_field import ready,field_scan,POSE

class ProtectedRecoveryTests(unittest.TestCase):
 def test_blocked_inflated_start_has_checked_side_exit(self):
  # Object is inside the old 32cm front margin but outside the physical body.
  p=FieldPlanner(field_scan(obstacles=[(155,175,105,120)]),POSE,Field(330,260))
  self.assertFalse(p.free(p.cell(POSE)))
  self.assertFalse(p.search((165,210))[0])
  result=p.escape_route((165,210),lambda mask:not mask&1)
  self.assertIsNotNone(result)
  mask,exit_point,path=result
  self.assertFalse(mask&1);self.assertTrue(p.free(p.cell(exit_point)));self.assertEqual(path[-1],(165,210))
 def test_swept_body_rejects_off_centre_object(self):
  p=FieldPlanner(field_scan(obstacles=[(174,179,105,120)]),POSE,Field(330,260))
  self.assertFalse(p.swept_clear(1,32))
 def test_close_left_contact_can_leave_right_but_not_push_left(self):
  p=FieldPlanner(field_scan(obstacles=[(145,156,50,110)]),POSE,Field(330,260))
  self.assertTrue(p.swept_clear(8,25,escape=True))
  self.assertFalse(p.swept_clear(4,25,escape=True))
 def test_all_directions_blocked_never_force_an_exit(self):
  p=FieldPlanner(field_scan(),POSE,Field(330,260))
  self.assertIsNone(p.escape_route((165,210),lambda mask:False))
 def test_board_blocked_forward_selects_side_with_same_goal(self):
  n=ready();n.select((165,210));n.start(10.9);n.feedback({'blocked':1},11.08)
  s=n.update(field_scan(),0,11.08,4)
  self.assertTrue(s.running);self.assertNotEqual(s.mask,0);self.assertFalse(s.mask&1)
  self.assertEqual(n.target,(165,210));self.assertEqual(s.route_mode,'退出近障区')
 def test_stale_feedback_cannot_initiate_escape(self):
  n=ready();n.select((165,210));n.start(10.9);n.feedback({'blocked':1},10)
  s=n.update(field_scan(),0,11.08,4);self.assertEqual(s.mask,0)
 def test_close_object_replans_without_cancelling_goal(self):
  n=ready();n.select((165,210));n.start(10.9);n.feedback({'blocked':1},11.08)
  s=n.update(field_scan(obstacles=[(155,175,105,120)]),0,11.08,4)
  self.assertTrue(s.valid,s.reason);self.assertTrue(s.running);self.assertNotEqual(s.mask,0,s.reason)
  self.assertFalse(s.mask&1);self.assertEqual(n.target,(165,210))
 def test_no_displacement_penalises_direction_and_tries_another(self):
  n=ready();n.select((165,210));n.start(10.9)
  for i in range(12):
   stamp=11.08+i*.27;n.feedback({'blocked':0,'manual':True,'applied_forward':210},stamp)
   s=n.update(field_scan(),0,stamp,4+i)
  self.assertTrue(n.active);self.assertTrue(n.avoid_dirs);self.assertFalse(s.mask&1,s.reason)
 def test_rotation_uses_task3_policy_without_extra_pc_veto(self):
  n=ready();n.select((165,210));n.start(10.9)
  n.correcting=True
  with patch.object(FieldPlanner,'plan',return_value=([(165,80),(165,210)],'test','test',0)):
   s=n.update(field_scan((165,80,10),[(145,156,50,110)]),10,11.08,4)
  self.assertTrue(s.running);self.assertEqual(s.mask,32,s.reason)
 def test_waypoint_does_not_jump_over_grid_corner(self):
  self.assertEqual(route_waypoint([(0,0),(0,10),(10,20),(20,30)],(20,30)),(0,10))
 def test_recorded_near_wall_stall_has_right_exit(self):
  f=json.loads(Path(__file__).with_name('near_obstacle_replay.json').read_text(encoding='utf-8'))
  p=FieldPlanner(f['points'],f['pose'],Field(**f['frame']))
  result=p.escape_route(f['goal'],lambda mask:not mask&f['blocked'])
  self.assertIsNotNone(result)
  mask,exit_point,path=result;self.assertFalse(mask&f['blocked'])
  self.assertTrue(p.free(p.cell(exit_point)))
 def test_protected_model_bypasses_close_front_object_and_finishes(self):
  for speed in (40,60,80):
   with self.subTest(speed=speed):
    n=ready();n.select((165,210));n.start(10.9);now=11.08;pose=list(POSE);seen=set()
    for i in range(180):
     pts=field_scan(tuple(pose),[(155,175,105,120)]);blocked=0
     for bit,centre,limit in [(1,0,40),(2,180,40),(4,270,25),(8,90,25)]:
      ranges=[r for a,r in pts if abs((a-centre+180)%360-180)<=11]
      if ranges and min(ranges)<limit:blocked|=bit
     n.feedback({'blocked':blocked},now);s=n.update(pts,0,now,4+i);seen.add(s.route_mode)
     if s.arrived:break
     self.assertTrue(s.running,s.reason);self.assertFalse(s.mask&blocked&15)
     dx=int(bool(s.mask&8))-int(bool(s.mask&4));dy=int(bool(s.mask&1))-int(bool(s.mask&2))
     norm=max(1,math.hypot(dx,dy));travel=speed*min(.27,max(0,s.until-now))
     pose[0]+=dx*travel/norm;pose[1]+=dy*travel/norm;now+=.27
     self.assertFalse(147<pose[0]<183 and 90<pose[1]<135,'Physical chassis hit object')
    self.assertTrue(s.arrived,s.reason);self.assertIn('退出近障区',seen)

class ProtectedGuiTests(unittest.TestCase):
 setUp=test_r3.TimingTests.setUp
 tearDown=test_r3.TimingTests.tearDown
 ready=test_r3.TimingTests.ready
 decision=test_r3.TimingTests.decision
 def test_new_board_block_releases_entire_diagonal_without_cancelling(self):
  a,now,s=self.decision(mask=9);a.task['blocked']=1
  a._poll_drive_once();self.assertEqual(a.link.drive_state[0],0);self.assertTrue(a.local_run)
 def test_free_side_command_is_sent_while_front_blocked(self):
  a,now,s=self.decision(mask=8);a.task['blocked']=1
  a._poll_drive_once();self.assertEqual(a.link.drive_state[0],8);self.assertTrue(a.local_run)

if __name__=='__main__':unittest.main(verbosity=2)
