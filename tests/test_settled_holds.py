import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController as C


def node():
    risk=[0.];cancelled=[]
    grid=NS(frame='map',inflation_overlap=lambda *a:risk[0])
    n=NS(p={'footprint_padding':.12,'minimum_separation':.4,'primary_minimum_separation':.6},
        poses={'tb3':(0.,0.,0.),'tb1':(10.,0.,0.)},polygons={r:[(-.2,-.2),(-.2,.2),(.2,.2),(.2,-.2)] for r in ('tb3','tb1')},
        grids={'/tb3/local_costmap/costmap':grid},in_frame=lambda r,p,f:p,
        held_auxiliary_slots={},obstacle_recoveries={},clearance_maneuver=None,state='BOUNDARY_TRAVELLING',
        cancel=cancelled.append,get_logger=lambda:NS(info=lambda *a:None),set_state=lambda *a:None)
    C.lock_auxiliary(n,'tb3',(0.,0.,0.))
    return n,risk,cancelled


def test_hold_survives_target_reservations_and_position_estimate_drift():
    n,risk,cancelled=node();n.feasible=lambda *a,**kw:False
    n.poses['tb3']=(.2,0.,.1)
    n.obstacle_recoveries={'tb3':{'mode':'escape'}}
    C.refresh_auxiliary_holds(n)
    assert 'tb3' in n.held_auxiliary_slots and cancelled==['tb3']
    assert not n.obstacle_recoveries


def test_new_local_obstacle_releases_only_the_affected_auxiliary():
    n,risk,_=node();n.held_auxiliary_slots['tb4']=(5.,5.,0.);n.polygons['tb4']=n.polygons['tb3']
    risk[0]=.2
    C.refresh_auxiliary_holds(n)
    assert 'tb3' not in n.held_auxiliary_slots and 'tb4' in n.held_auxiliary_slots


def test_missing_costmap_does_not_authorize_motion_from_a_settled_position():
    n,risk,_=node();n.grids={}
    C.refresh_auxiliary_holds(n)
    assert 'tb3' in n.held_auxiliary_slots


def test_actual_peer_entering_physical_clearance_releases_hold():
    n,risk,_=node();n.poses['tb1']=(.9,0.,0.)
    C.refresh_auxiliary_holds(n)
    assert 'tb3' not in n.held_auxiliary_slots


def test_all_arrived_auxiliaries_stay_locked_despite_later_pose_estimate_drift():
    from formation_geometry import rectangle
    primary=(10.,0.,0.);targets=rectangle(primary,4.2,3.4);sent=[]
    n=NS(p={'rectangle_half_length':4.2,'rectangle_half_width':3.4,'arrival_tolerance':.15,'yaw_tolerance':.2618,'primary_yaw_tolerance':.05},
        poses={'tb1':primary,**targets},primary=primary,boundary_heading=0.,flexible_slots={},flexible_final_targets=None,
        feasible=lambda *a,**kw:True,choose=lambda r,t,res:t,send=lambda r,t:sent.append(r),cancel=lambda *a:None,
        cancel_all=lambda:None,idle_actions=lambda:True,set_state=lambda *a:None)
    n.arrived=lambda r,t:C.arrived(n,r,t)
    C.flexible_tick(n)
    assert set(n.held_auxiliary_slots)==set(targets)
    sent.clear();n.poses['tb1']=(10.2,0.,0.)
    for r,t in targets.items():n.poses[r]=(t[0]+.2,t[1],t[2])
    C.flexible_tick(n)
    assert n.primary_goal_reached and set(n.held_auxiliary_slots)==set(targets)
    assert not set(sent)&set(targets)
