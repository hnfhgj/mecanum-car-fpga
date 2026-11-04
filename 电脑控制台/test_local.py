import unittest,math,time
from local_navigation import *

def scene(pose=(0,0,0),box=(-150,150,-150,150),obstacles=()):
 """Raycast a rectangular room and optional interior axis-aligned objects."""
 x,y,yaw=pose;points=[]
 for a in range(0,360,2):
  dx,dy=rotate(0,1,yaw-a);dist=500
  xmin,xmax,ymin,ymax=box
  for wall,c,d,other,lo,hi in [(xmin,x,dx,(y,dy),ymin,ymax),(xmax,x,dx,(y,dy),ymin,ymax),(ymin,y,dy,(x,dx),xmin,xmax),(ymax,y,dy,(x,dx),xmin,xmax)]:
   if abs(d)>1e-9:
    t=(wall-c)/d
    if t>0 and lo-1e-8<=other[0]+t*other[1]<=hi+1e-8:dist=min(dist,t)
  for ax,bx,ay,by in obstacles:
   low,high=0,500
   for coord,delta,lo,hi in [(x,dx,ax,bx),(y,dy,ay,by)]:
    if abs(delta)<1e-9:
     if not lo<=coord<=hi:low,high=1,0
    else:
     t0,t1=(lo-coord)/delta,(hi-coord)/delta;low=max(low,min(t0,t1));high=min(high,max(t0,t1))
   if 0<low<=high:dist=min(dist,low)
  points.append((a,max(6,round(dist/2)*2)))
 return points

class TrackingTests(unittest.TestCase):
 def test_known_translation_and_imu_rotation(self):
  t=ScanTracker();t.update(scene(),20,10)
  for i,p in enumerate([(3,2,1),(6,4,2),(12,8,3)]):
   m=t.update(scene(p),20+p[2],10+.27*(i+1));self.assertTrue(m.ok,m.reason)
  self.assertAlmostEqual(t.pose[0],12,delta=2);self.assertAlmostEqual(t.pose[1],8,delta=2);self.assertEqual(t.pose[2],3)
 def test_static_quantized_scan_does_not_walk(self):
  t=ScanTracker()
  for i in range(20):self.assertTrue(t.update(scene(),40,10+i*.27).ok)
  self.assertLess(math.hypot(*t.pose[:2]),1)
 def test_longitudinal_single_wall_unobservable(self):
  points=[(a,100/max(.1,math.cos(math.radians(a)))) for a in range(0,50,2)]
  t=ScanTracker();self.assertFalse(t.update(points,0,10).ok)
 def test_disconnect_never_reanchors_silently(self):
  t=ScanTracker();t.update(scene(),0,10);m=t.update(scene(),0,12)
  self.assertFalse(m.ok);self.assertIn('重置',m.reason)
 def test_large_unexplained_jump_rejected(self):
  t=ScanTracker();t.update(scene(),0,10);m=t.update(scene((75,65,0)),0,10.1)
  self.assertFalse(m.ok)
 def test_repeated_straight_scan_rejected_without_false_pose(self):
  points=[(a,100/max(.01,math.cos(math.radians(a)))) for a in range(-60,62,2)]
  t=ScanTracker();t.update(points,0,10);m=t.update(points,0,10.27);self.assertFalse(m.ok)

class FrameTests(unittest.TestCase):
 def test_rotated_clear_frame(self):
  r=fit_rectangle(scene((0,0,13),box=(-100,100,-100,100)));self.assertIsNotNone(r)
  self.assertAlmostEqual(r.angle,13,delta=2)
 def test_clear_four_edges(self):
  r=fit_rectangle(scene(box=(-100,100,-80,120)));self.assertIsNotNone(r)
  self.assertAlmostEqual(r.xmax-r.xmin,200,delta=10);self.assertAlmostEqual(r.ymax-r.ymin,200,delta=10)
  self.assertGreaterEqual(r.corners,3);self.assertGreaterEqual(r.coverage,.625)
 def test_rectangular_not_square(self):
  r=fit_rectangle(scene(box=(-150,150,-90,90)));self.assertIsNotNone(r)
  self.assertAlmostEqual(max(r.xmax-r.xmin,r.ymax-r.ymin),300,delta=12)
  self.assertAlmostEqual(min(r.xmax-r.xmin,r.ymax-r.ymin),180,delta=12)
 def test_four_short_dense_clusters_not_a_frame(self):
  points=[]
  for cx,cy in [(-60,0),(60,0),(0,60),(0,-60)]:
   for i in range(25):
    x,y=cx+(i%5-2),cy+(i//5-2);points.append((math.degrees(math.atan2(x,y))%360,math.hypot(x,y)))
  self.assertIsNone(fit_rectangle(points))
 def test_missing_edge_rejected(self):
  p=[(a,r) for a,r in scene() if not 44<=a<=136]
  self.assertIsNone(fit_rectangle(p))
 def test_internal_objects_do_not_replace_outer_frame(self):
  r=fit_rectangle(scene(obstacles=[(60,65,30,35),(-65,-60,30,35)]));self.assertIsNotNone(r)
  self.assertGreater(r.xmax-r.xmin,270);self.assertGreater(r.ymax-r.ymin,270)

class PlanningTests(unittest.TestCase):
 def test_empty_route(self):
  p=LocalPlanner(scene(),(0,0,0));route,reason=p.plan((0,90));self.assertTrue(route,reason)
  self.assertEqual(route[-1],(0,90))
 def test_visible_obstacle_requires_detour(self):
  pts=scene(obstacles=[(-12,12,45,65)]);planner=LocalPlanner(pts,(0,0,0));route,reason=planner.plan((45,100))
  self.assertTrue(route,reason);self.assertGreater(max(abs(x) for x,y in route),35)
  self.assertTrue(all(planner.free(planner.cell(p)) for p in route))
 def test_wall_target_rejected(self):
  route,reason=LocalPlanner(scene(),(0,0,0)).plan((0,145));self.assertFalse(route)
 def test_unseen_goal_not_assumed_free(self):
  route,reason=LocalPlanner(scene(),(0,0,0)).plan((0,300));self.assertFalse(route)
 def test_diagonal_does_not_cut_blocked_corner(self):
  p=LocalPlanner(scene(),(0,0,0));p.blocked.update({(1,0),(0,1)});route,_=p.plan((30,30))
  self.assertTrue(route);self.assertNotEqual(route[1],(10,10))

class NavigatorTests(unittest.TestCase):
 def ready(self):
  n=Navigator()
  for i in range(4):n.update(scene(),0,10+i*.27,i)
  return n
 def test_connection_never_starts_motion(self):
  n=self.ready();self.assertFalse(n.active);self.assertEqual(n.snapshot.mask,0)
 def test_no_rectangle_required_to_start(self):
  n=self.ready();n.rectangle=None;n.select((0,90));n.start(10.9)
  s=n.update(scene(),0,11.08,4);self.assertTrue(s.running);self.assertEqual(s.mask,1)
 def test_target_stays_fixed_as_car_moves(self):
  n=self.ready();n.select((0,90));n.start(10.9);n.update(scene((0,8,0)),0,11.08,4)
  self.assertEqual(n.target,(0,90));self.assertGreater(n.tracker.pose[1],5)
 def test_arrival_stops_and_requires_stability(self):
  n=self.ready();n.select((0,5));n.start(10.9)
  for i in range(3):s=n.update(scene(),0,11.08+i*.27,4+i)
  self.assertTrue(s.arrived);self.assertFalse(n.active);self.assertEqual(s.mask,0)
 def test_failure_cancels_does_not_resume(self):
  n=self.ready();n.select((0,90));n.start(10.9);s=n.update([],0,11.08,4)
  self.assertFalse(n.active);self.assertEqual(s.mask,0)
  self.assertEqual(n.update(scene(),0,11.35,5).mask,0)
 def test_stop_immediate(self):
  n=self.ready();n.select((0,90));n.start(10.9);n.update(scene(),0,11.08,4);n.stop()
  self.assertFalse(n.active);self.assertEqual(n.snapshot.mask,0)
 def test_stale_start_rejected(self):
  n=self.ready();n.select((0,90))
  with self.assertRaises(ValueError):n.start(20)
 def test_bad_goal_rejected(self):
  n=self.ready()
  for target in [(float('nan'),0),(800,0)]:
   with self.assertRaises(ValueError):n.select(target)
 def test_duplicate_scan_cannot_refresh_motion(self):
  n=self.ready();n.select((0,90));n.start(10.9);n.update(scene(),0,11.08,4)
  s=n.update(scene(),0,11.35,4);self.assertFalse(s.valid);self.assertFalse(n.active);self.assertEqual(s.mask,0)
 def test_scan_sequence_wrap_is_valid(self):
  n=Navigator();n.update(scene(),0,10,65535);s=n.update(scene(),0,10.27,0);self.assertTrue(s.valid)
 def test_closed_loop_bypasses_new_obstacle_then_arrives(self):
  n=self.ready();n.select((0,100));n.start(10.9);now=11.08;pose=[0.,0.,0.];excursion=0
  for i in range(120):
   s=n.update(scene(tuple(pose),obstacles=[(-10,10,40,60)]),0,now,4+i)
   if s.arrived:break
   self.assertTrue(n.active,s.reason)
   dx=int(bool(s.mask&8))-int(bool(s.mask&4));dy=int(bool(s.mask&1))-int(bool(s.mask&2));norm=max(1,math.hypot(dx,dy))
   distance=40*max(0,s.until-now);pose[0]+=dx*distance/norm;pose[1]+=dy*distance/norm
   excursion=max(excursion,abs(pose[0]));now+=.27
   self.assertFalse(-18<pose[0]<18 and 25<pose[1]<75,'Chassis entered physical obstacle')
  self.assertTrue(s.arrived);self.assertGreater(excursion,30);self.assertLess(math.hypot(pose[0],pose[1]-100),12)
  self.assertEqual(n.target,(0,100));self.assertEqual(s.mask,0)

if __name__=='__main__':unittest.main(verbosity=2)
