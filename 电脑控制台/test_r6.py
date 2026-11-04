import unittest,json
from pathlib import Path
from unittest.mock import patch
from field_navigation import FieldNavigator,FieldTracker,Field,FieldPlanner,Match
from test_field import ready,field_scan

class ContinuousNavigationTests(unittest.TestCase):
 def test_valid_route_has_no_extra_pc_short_or_swept_veto(self):
  n=ready();n.select((165,210));n.start(10.9);n.feedback({'blocked':0},11.08)
  with patch.object(FieldPlanner,'short_clear',side_effect=AssertionError('extra stop')):
   with patch.object(FieldPlanner,'swept_clear',side_effect=AssertionError('extra stop')):
    s=n.update(field_scan(),0,11.08,4)
  self.assertEqual(s.mask,1)
 def test_three_recorded_false_stops_produce_movement(self):
  rows=json.loads(Path(__file__).with_name('r5_false_stop_replay.json').read_text(encoding='utf-8'))
  self.assertEqual(len(rows),3)
  for row in rows:
   n=FieldNavigator();n.tracker.frame=Field(**row['frame']);n.tracker.pose=row['pose'];n.tracker.stable=5
   n.target=row['goal'];n.active=True;n.started=9;n.avoid_dirs={}
   with patch.object(n.tracker,'update',return_value=Match(True,'replay',1,100)):
    s=n.update(row['points'],row['pose'][2],10,1)
   self.assertNotEqual(s.mask,0,s.reason)
 def test_intention_without_actual_drive_does_not_blacklist_free_directions(self):
  n=ready();n.select((165,210));n.start(10.9);masks=[]
  for i in range(20):
   stamp=11.08+i*.27;n.feedback({'blocked':0,'manual':False,'applied_forward':0,'applied_left':0},stamp)
   s=n.update(field_scan(),0,stamp,4+i);masks.append(s.mask)
  self.assertFalse(n.avoid_dirs);self.assertEqual(set(masks),{1});self.assertGreaterEqual(s.duration,.3)
 def test_selected_exit_is_held_instead_of_flipping_with_each_plan(self):
  n=ready();n.select((165,210));n.start(10.9)
  with patch.object(FieldPlanner,'plan',return_value=([], 'test', 'test',0)):
   for i in range(6):
    mask=4 if i%2==0 else 8;ep=(125,80) if mask==4 else (205,80)
    with patch.object(FieldPlanner,'escape_route',return_value=(mask,ep,[(165,80),ep,(165,210)])):
     s=n.update(field_scan(),0,11.08+i*.27,4+i)
    self.assertEqual(s.mask,4);self.assertGreaterEqual(s.duration,.3)
 def test_new_board_obstacle_invalidates_committed_direction(self):
  n=ready();n.select((165,210));n.start(10.9);n.route_commit=(4,(125,80),[(125,80),(165,210)])
  n.feedback({'blocked':4},11.08);s=n.update(field_scan(),0,11.08,4)
  self.assertFalse(s.mask&4);self.assertNotEqual(s.mask,0)
 def test_new_raw_obstacle_invalidates_committed_direction(self):
  n=ready();n.select((165,210));n.start(10.9);n.route_commit=(4,(125,80),[(125,80),(165,210)])
  s=n.update(field_scan(obstacles=[(130,140,70,90)]),0,11.08,4)
  self.assertFalse(s.mask&4);self.assertNotEqual(s.mask,0,s.reason)
 def test_stale_feedback_never_renews_committed_motion(self):
  n=ready();n.select((165,210));n.start(10.9);n.route_commit=(4,(125,80),[])
  n.feedback({'blocked':0},10);s=n.update(field_scan(),0,11.08,4)
  self.assertEqual(s.mask,0)

if __name__=='__main__':unittest.main(verbosity=2)
