import unittest,math,json
from pathlib import Path
from field_navigation import *
from test_local import scene

BOX=(0,330,0,260);POSE=(165,80,0)
def field_scan(pose=POSE,obstacles=()):return scene(pose,BOX,obstacles)
def ready():
 n=FieldNavigator();n.calibrate((330,260))
 for i in range(4):n.update(field_scan(),0,10+i*.27,i)
 return n

class FieldFitTests(unittest.TestCase):
 def test_recorded_inner_crossbar_not_outer_and_reference_can_swap(self):
  pts=json.loads(Path(__file__).with_name('recorded_field_scan.json').read_text(encoding='utf-8'))
  for expected in ((330,260),(260,330),None):
   r=fit_field(pts,expected);self.assertIsNotNone(r)
   self.assertAlmostEqual(r.xmax-r.xmin,252,delta=8);self.assertAlmostEqual(r.ymax-r.ymin,336,delta=8)
 def test_clear_large_rectangle_over_old_three_metre_limit(self):
  r=fit_field(field_scan(),(330,260));self.assertIsNotNone(r)
  self.assertAlmostEqual(r.xmax-r.xmin,330,delta=8);self.assertAlmostEqual(r.ymax-r.ymin,260,delta=8)
 def test_image_style_partial_edges_missing_corners(self):
  points=[]
  for a,r in field_scan((165,130,14)):
   x,y=rotate(0,r,14-a);x+=165;y+=130
   edge=min(range(4),key=lambda i:[abs(x),abs(x-330),abs(y),abs(y-260)][i])
   keep=[90<=y<=220,True,90<=x<=180,x>=20][edge]
   if keep:points.append((a,r))
  r=fit_field(points,(330,260));self.assertIsNotNone(r)
  self.assertAlmostEqual(r.xmax-r.xmin,330,delta=12);self.assertAlmostEqual(r.angle,14,delta=2)
 def test_small_inner_rectangle_not_used_for_large_field(self):
  p=scene(box=(-50,50,-50,50));self.assertIsNone(fit_field(p,(330,260)))
 def test_missing_entire_edge_rejected(self):
  p=[(a,r) for a,r in field_scan() if not 44<a<136]
  self.assertIsNone(fit_field(p,(330,260)))
 def test_rotation_size_and_translation(self):
  r=fit_field(field_scan((110,90,12)),(330,260));self.assertIsNotNone(r)
  self.assertAlmostEqual(-r.xmin,110,delta=6);self.assertAlmostEqual(-r.ymin,90,delta=6);self.assertAlmostEqual(r.angle,12,delta=2)

class FieldTrackerTests(unittest.TestCase):
 def test_does_not_calibrate_without_button(self):
  n=FieldNavigator();s=n.update(field_scan(),0,10,0);self.assertIsNone(s.frame);self.assertEqual(s.mask,0)
 def test_three_static_scans_establish_fixed_origin(self):
  n=ready();self.assertIsNotNone(n.tracker.frame);self.assertTrue(n.snapshot.valid)
  self.assertAlmostEqual(n.tracker.pose[0],165,delta=6);self.assertAlmostEqual(n.tracker.pose[1],80,delta=6)
 def test_movement_during_calibration_not_accepted(self):
  n=FieldNavigator();n.calibrate((330,260));n.update(field_scan(),0,10,0);s=n.update(field_scan(),3,10.27,1)
  self.assertIsNone(s.frame);self.assertIn('转动',s.reason)
 def test_translation_during_calibration_not_treated_as_static(self):
  n=FieldNavigator();n.calibrate((330,260))
  for i in range(3):s=n.update(field_scan((165+5*i,80,0)),0,10+i*.27,i)
  self.assertIsNone(s.frame)
 def test_tracks_movement_in_fixed_field(self):
  n=ready()
  for i,p in enumerate([(165,85,0),(170,90,1),(180,95,2)]):s=n.update(field_scan(p),p[2],11.08+i*.27,4+i)
  self.assertTrue(s.valid,s.reason);self.assertAlmostEqual(s.pose[0],180,delta=5);self.assertAlmostEqual(s.pose[1],95,delta=5)
 def test_bad_goal_rejected(self):
  n=ready()
  for p in [(500,100),(100,255),(float('nan'),50)]:
   with self.assertRaises(ValueError):n.select(p)
 def test_interior_objects_not_tracking_reference(self):
  n=ready();s=n.update(field_scan(obstacles=[(150,180,135,155)]),0,11.08,4)
  self.assertTrue(s.valid,s.reason);self.assertLess(math.dist(s.pose[:2],POSE[:2]),4)
 def test_slow_gyro_drift_corrected_by_field_edges(self):
  n=ready()
  for i in range(12):s=n.update(field_scan(),(i+1)*.3,11.08+i*.27,4+i)
  self.assertTrue(s.valid,s.reason);self.assertLess(abs(s.pose[2]),1.5);self.assertLess(math.dist(s.pose[:2],POSE[:2]),4)

class FieldPlanTests(unittest.TestCase):
 def planner(self,obstacles):return FieldPlanner(field_scan(obstacles=obstacles),POSE,Field(330,260))
 def test_occluded_target_has_both_side_routes(self):
  p=self.planner([(155,175,125,145)]);target=(165,210)
  self.assertFalse(p.local.visible(p.cell(target)))
  path,why,mode,side=p.plan(target);self.assertTrue(path,why);self.assertIn(side,(-1,1));self.assertIn('绕行',mode)
  # Both explicit alternatives are evaluated on the same fixed map.
  self.assertTrue(p.search(target,(0,1,55),1)[0]);self.assertTrue(p.search(target,(0,1,55),-1)[0])
 def test_left_blocked_selects_right(self):
  path,why,mode,side=self.planner([(0,175,125,145)]).plan((165,210));self.assertTrue(path,why);self.assertEqual(side,-1)
 def test_right_blocked_selects_left(self):
  path,why,mode,side=self.planner([(155,330,125,145)]).plan((165,210));self.assertTrue(path,why);self.assertEqual(side,1)
 def test_full_barrier_is_genuinely_no_route(self):
  path,why,mode,side=self.planner([(0,330,125,145)]).plan((165,210));self.assertFalse(path);self.assertEqual(mode,'无路等待')
 def test_open_field_direct(self):
  path,why,mode,side=self.planner([]).plan((165,210));self.assertTrue(path);self.assertEqual(mode,'直达')
 def test_cannot_leave_calibrated_rectangle(self):
  path,why,mode,side=self.planner([]).plan((400,100));self.assertFalse(path)

class FieldControlTests(unittest.TestCase):
 def test_stop_cancels_pending_calibration(self):
  n=FieldNavigator();n.calibrate((330,260));n.update(field_scan(),0,10,0);n.stop()
  for i in range(3):s=n.update(field_scan(),0,10.27+i*.27,1+i)
  self.assertIsNone(s.frame);self.assertFalse(n.tracker.pending);self.assertEqual(s.mask,0)
 def test_continuous_open_cruise_outlasts_scan_period(self):
  n=ready();n.select((165,210));n.start(10.9);s=n.update(field_scan(),0,11.08,4)
  self.assertNotEqual(s.mask,0);self.assertGreater(s.until-s.stamp,.27)
 def test_duplicate_data_stops(self):
  n=ready();n.select((165,210));n.start(10.9);n.update(field_scan(),0,11.08,4);s=n.update(field_scan(),0,11.35,4)
  self.assertTrue(n.active);self.assertEqual(s.mask,0);self.assertEqual(s.route_mode,'定位等待')
  s=n.update(field_scan(),0,12.5,4);self.assertFalse(n.active);self.assertEqual(s.route_mode,'已停止')
 def test_closed_loop_obstacle_behind_goal_line_bypassed(self):
  for speed in (40,60,80):
   with self.subTest(cm_per_second=speed):
    n=ready();n.select((165,210));n.start(10.9);now=11.08;pose=list(POSE);max_excursion=0;seen=set()
    for i in range(130):
     s=n.update(field_scan(tuple(pose),[(155,175,125,145)]),0,now,4+i);seen.add(s.route_mode)
     if s.arrived:break
     self.assertTrue(n.active,s.reason)
     dx=int(bool(s.mask&8))-int(bool(s.mask&4));dy=int(bool(s.mask&1))-int(bool(s.mask&2));norm=max(1,math.hypot(dx,dy));travel=speed*min(.27,max(0,s.until-now))
     pose[0]+=dx*travel/norm;pose[1]+=dy*travel/norm;max_excursion=max(max_excursion,abs(pose[0]-165));now+=.27
     self.assertFalse(147<pose[0]<183 and 110<pose[1]<160,'Physical chassis intersects obstacle')
    self.assertTrue(s.arrived,s.reason);self.assertGreater(max_excursion,30);self.assertTrue({'左侧绕行','右侧绕行'}&seen);self.assertLess(math.dist(pose[:2],(165,210)),12)

if __name__=='__main__':unittest.main(verbosity=2)
