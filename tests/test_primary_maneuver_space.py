import sys,time,math
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController as C
from formation_geometry import ROBOTS

BOX=[(-.2,-.2),(-.2,.2),(.2,.2),(.2,-.2)]

def node(tight=True):
    cancelled=[];sent=[];logs=[];states=[]
    grid=NS(frame='map',footprint_free=lambda pose,poly,padding,*args: not tight or padding<.5)
    free=NS(frame='map',footprint_free=lambda *a:True)
    poses={'tb1':(0.,0.,0.),'tb2':(2.,0.,0.),'tb3':(-2.,0.,0.),'tb4':(0.,2.,0.),'tb5':(0.,-2.,0.)}
    n=NS(p={'primary_space_enabled':True,'primary_space_margin':1.2,'primary_space_probe_padding':.6,
        'primary_space_release_delay':5.,'primary_space_search_radius':3.,'footprint_padding':.12,
        'primary_minimum_separation':.6,'minimum_separation':.4,'collision_horizon':2.,'yield_recovery_delay':3.,'maximum_target_cost':70},
        poses=poses,polygons={r:BOX for r in ROBOTS},radii={r:math.hypot(.2,.2) for r in ROBOTS},
        grids={'/map':grid,**{f'/{r}/{scope}_costmap/costmap':free for r in ROBOTS for scope in ('local','global')}},
        clock_value=10.,primary=(10.,0.,0.),state='BOUNDARY_TRAVELLING',slots={r:{'handle':object(),'progress_sim':10.} for r in ROBOTS},
        primary_space_active=False,primary_space_targets={},primary_space_clear_since=None,clearance_maneuver=None,
        obstacle_recoveries={},obstacle_since={},flexible_final_targets={'old':(0.,0.,0.)},local_blocked=set(),unresolved_pairs=set(),
        cancel=cancelled.append,send=lambda r,t:sent.append((r,t)),set_state=lambda *a:states.append(a),
        in_frame=lambda r,p,f:p,arrived=lambda r,t:math.dist(poses[r][:2],t[:2])<.15,
        get_logger=lambda:NS(info=logs.append))
    n.feasible=lambda *a,**kw:C.feasible(n,*a,**kw)
    return n,cancelled,sent,logs


def test_tight_map_cancels_only_auxiliary_actions_without_releasing_ownership():
    n,cancelled,sent,logs=node();owners={r:s['handle'] for r,s in n.slots.items()}
    assert C.update_primary_space(n,False)
    assert cancelled==list(ROBOTS[1:]) and n.flexible_final_targets is None
    assert all(n.slots[r]['handle'] is owners[r] for r in ROBOTS)
    assert C.update_primary_space(n,False)
    assert cancelled==list(ROBOTS[1:]) # No repeated cancellation churn.


def test_open_map_and_unstalled_primary_do_not_trigger_spreading():
    n,*_=node(False)
    assert not C.update_primary_space(n,False)
    n.slots['tb1']['progress_sim']=0.
    assert C.update_primary_space(n,False) # Stalled while surrounded by peers.


def test_release_waits_for_continuously_clear_space():
    n,*_=node();assert C.update_primary_space(n,False)
    n.grids['/map']=NS(frame='map',footprint_free=lambda *a:True)
    n.clock_value=11.;n.slots['tb1']['progress_sim']=11.
    assert C.update_primary_space(n,False)
    n.clock_value=15.;n.slots['tb1']['progress_sim']=15.
    assert C.update_primary_space(n,False)
    n.clock_value=16.;n.slots['tb1']['progress_sim']=16.
    assert not C.update_primary_space(n,False)
    assert n.primary_space_targets=={} and n.flexible_final_targets is None


@pytest.mark.parametrize('robot',ROBOTS[1:])
def test_parking_goals_gain_space_without_crossing_the_primary_or_static_hazards(robot):
    n,*_=node();n.primary_space_active=True
    target=C.primary_space_target(n,robot,{'tb1':n.poses['tb1']})
    assert target is not None and math.dist(target[:2],(0.,0.))>4.
    lane=C.primary_corridor(n,lookahead=6.)
    assert C.primary_corridor_gap(n,robot,target,lane)>=.6
    n.grids['/map']=NS(frame='map',footprint_free=lambda *a:False)
    assert C.primary_space_target(n,robot,{'tb1':n.poses['tb1']}) is None


def test_existing_roomy_parking_position_is_held_without_chasing_larger_distance():
    n,*_=node();n.primary_space_active=True;n.poses['tb2']=(-6.,0.,0.)
    assert C.primary_space_target(n,'tb2',{'tb1':n.poses['tb1']})==n.poses['tb2']


def test_gate_blocks_tightening_space_but_allows_checked_reverse_escape():
    n,*_=node();n.primary_space_active=True
    assert not C.primary_space_command_safe(n,'tb2',(2.,0.,math.pi),(.2,0.),n.poses['tb1'])
    assert C.primary_space_command_safe(n,'tb2',(2.,0.,math.pi),(-.2,.1),n.poses['tb1'])
    assert C.primary_space_command_safe(n,'tb2',(2.,0.,0.),(0.,.2),n.poses['tb1']) # Turning to depart still passes physical collision gates.
    assert C.primary_space_command_safe(n,'tb2',(6.,0.,math.pi),(.2,0.),n.poses['tb1'])


def test_no_safe_retreat_holds_instead_of_returning_to_nearby_formation_slot():
    n,cancelled,sent,logs=node();n.primary_space_active=True
    n.grids['/map']=NS(frame='map',footprint_free=lambda *a:False)
    C.primary_space_tick(n)
    assert sent==[('tb1',n.primary)] and cancelled==list(ROBOTS[1:])
    assert all(n.targets[r]==n.poses[r] for r in ROBOTS[1:])


def test_fair_search_gives_each_auxiliary_a_stable_outward_goal():
    n,cancelled,sent,logs=node();n.primary_space_active=True
    for _ in range(4):C.primary_space_tick(n)
    assert set(n.primary_space_targets)==set(ROBOTS[1:])
    saved=dict(n.primary_space_targets)
    C.primary_space_tick(n)
    assert n.primary_space_targets==saved


def test_a_robot_parked_further_ahead_still_moves_off_the_future_primary_lane():
    n,*_=node();n.primary_space_active=True;n.poses['tb2']=(4.,0.,0.)
    target=C.primary_space_target(n,'tb2',{'tb1':n.poses['tb1']})
    assert target is not None and abs(target[1])>.5


def test_partial_retreat_is_revisited_after_arrival_to_gain_more_room():
    n,*_=node();n.primary_space_active=True;n.primary_space_targets={'tb3':(-2.1,0.,0.)}
    n.poses['tb3']=(-2.1,0.,0.);n.primary_space_search_cursor=1
    C.primary_space_tick(n)
    assert n.primary_space_targets['tb3'][0]<-2.4


def test_parallel_motion_inside_reserved_lane_does_not_fake_outward_progress():
    from formation_geometry import polyline_distance,route_preview
    route=route_preview((0.,0.,0.),[(0.,0.,0.),(4.,0.,0.)],4.)
    assert polyline_distance((1.,.5,0.),route)==pytest.approx(.5)
    assert polyline_distance((1.6,.5,0.),route)==pytest.approx(.5)


def test_only_selected_outward_partial_retreat_bypasses_the_preferred_extra_buffer():
    n,*_=node();n.primary_space_active=True;n.poses['tb4']=(0.,1.,0.)
    target=(0.,2.,0.)
    assert not C.feasible(n,'tb4',target)
    n.primary_space_targets['tb4']=target
    assert C.feasible(n,'tb4',target)
    n.primary_space_targets['tb4']=(0.,.5,0.)
    assert not C.feasible(n,'tb4',(0.,.5,0.))


def test_primary_local_escape_keeps_command_ownership_while_followers_give_room():
    n,cancelled,sent,logs=node();n.primary_space_active=True
    n.obstacle_recoveries={'tb1':{'target':None}}
    C.primary_space_tick(n)
    assert all(robot!='tb1' for robot,target in sent)
    n.obstacle_recoveries['tb1']['target']=(-1.,0.,0.)
    C.primary_space_tick(n)
    assert ('tb1',(-1.,0.,0.)) in sent


def test_normal_clearance_maneuvers_do_not_compete_with_outward_parking_targets():
    n,cancelled,sent,logs=node();n.primary_space_active=True
    n.unresolved_pairs={('tb1','tb2')};n.clearance_target=lambda *a:pytest.fail('competing clearance plan')
    C.clearance_tick(n)
    assert n.clearance_maneuver is None
