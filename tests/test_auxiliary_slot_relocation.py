import sys,math
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController as C


def node():
    logs=[]
    n=NS(p={'flexible_boundary':True,'alternative_radius':1.5,'candidate_step':.3,'retry_delay':8.,'maximum_retries':3},
        state='BOUNDARY_TRAVELLING',primary_space_active=False,flexible_final_targets={'tb4':(4.,3.,.2)},
        slots={'tb4':{'target':(4.,3.,.2),'planning_target':(4.,3.,.2),'retries':0,'handle':None,'pending':False}},
        targets={'tb4':(4.,3.,.2)},yielding={},local_blocked=set(),obstacle_recoveries={},
        feasible=lambda r,p,res:True,get_logger=lambda:NS(info=logs.append,warning=logs.append),
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=0)),cancel=lambda r:None)
    return n,logs


def test_occupied_slot_moves_to_closest_free_candidate_with_original_heading():
    n,_=node();preferred=(4.,3.,.2)
    n.feasible=lambda r,p,res:math.dist(p[:2],preferred[:2])>=.29
    selected=C.choose(n,'tb4',preferred,{})
    assert .29<=math.dist(selected[:2],preferred[:2])<=.31
    assert selected[2]==preferred[2]


def test_no_path_rejects_slot_before_repeated_retries_and_tries_nearby():
    n,logs=node();preferred=n.targets['tb4']
    C.failure(n,'tb4','No feasible Nav2 path')
    assert n.flexible_final_targets is None and not n.obstacle_recoveries
    assert n.slots['tb4']['retries']==0 and n.slots['tb4']['cooldown']==0.
    selected=C.choose(n,'tb4',preferred,{})
    assert .2<math.dist(selected[:2],preferred[:2])<=.31
    assert logs


def test_shifted_slot_is_stable_when_original_obstacle_disappears():
    n,_=node();preferred=(4.,3.,.2)
    n.feasible=lambda r,p,res:math.dist(p[:2],preferred[:2])>.2
    selected=C.choose(n,'tb4',preferred,{})
    n.feasible=lambda *a:True
    assert C.choose(n,'tb4',preferred,{})==selected


def test_repeated_rejections_stay_within_original_neighborhood_and_exhaust_safely():
    n,_=node();preferred=(4.,3.,.2);count=0
    while True:
        selected=C.choose(n,'tb4',preferred,{})
        if selected is None:break
        assert math.dist(selected[:2],preferred[:2])<=1.500001
        n.slots['tb4'].update(target=selected,planning_target=selected)
        assert C.reject_auxiliary_slot(n,'tb4','No feasible Nav2 path')
        count+=1
        assert count<200
    assert count>3


def test_relocation_does_not_override_primary_or_outward_parking_recovery():
    n,_=node()
    assert not C.reject_auxiliary_slot(n,'tb1','No path')
    n.primary_space_active=True
    assert not C.reject_auxiliary_slot(n,'tb4','No outward route')
    assert n.flexible_final_targets is not None


def test_frozen_slot_is_revalidated_when_a_new_obstacle_appears():
    from formation_geometry import rectangle
    targets=rectangle((10.,0.,0.),4.2,3.4)
    changed=targets['tb4'];calls=[]
    n=NS(poses={'tb1':(10.,0.,0.),**targets},primary=(10.,0.,0.),boundary_heading=0.,
        p={'rectangle_half_length':4.2,'rectangle_half_width':3.4,'arrival_tolerance':.15},
        flexible_final_targets=dict(targets),flexible_slots={},arrived=lambda r,t:r=='tb1',
        feasible=lambda r,t,res:not(r=='tb4' and t==changed),
        choose=lambda r,t,res:(calls.append(r) or (t[0],t[1]+.3,t[2]) if r=='tb4' else t),
        set_state=lambda *a:None,send=lambda *a:None,cancel=lambda *a:None)
    C.flexible_tick(n)
    assert 'tb4' in calls and n.flexible_final_targets['tb4']!=changed
    assert n.targets['tb1']==n.primary


def test_stable_shifted_position_still_uses_requested_final_heading():
    n,_=node();preferred=(4.,3.,.2)
    n.feasible=lambda r,p,res:math.dist(p[:2],preferred[:2])>.2
    selected=C.choose(n,'tb4',preferred,{})
    updated=C.choose(n,'tb4',(4.,3.,.25),{})
    assert updated[:2]==selected[:2] and updated[2]==.25


def test_settled_auxiliary_does_not_chase_a_new_nearby_candidate():
    n,_=node();held=(4.3,3.,.2);n.held_auxiliary_slots={'tb4':held}
    assert C.choose(n,'tb4',(4.,3.,.2),{})==held
    n.feasible=lambda *a:False
    assert C.choose(n,'tb4',(4.,3.,.2),{})==held
    assert 'tb4' in n.held_auxiliary_slots


def test_clearance_moves_unfinished_robot_before_settled_neighbor():
    n,_=node();requested=[]
    n.clock_value=10.;n.p['yield_recovery_delay']=3.
    n.clearance_maneuver=None;n.conflict_since={('tb3','tb4'):0.}
    n.unresolved_pairs={('tb3','tb4')};n.held_auxiliary_slots={'tb3':(4.,3.,0.)}
    n.clearance_target=lambda moving,held:(requested.append((moving,held)) or (6.,3.,0.))
    C.clearance_tick(n)
    assert requested==[('tb4','tb3')]
    assert 'tb3' in n.held_auxiliary_slots
    assert n.clearance_maneuver['robot']=='tb4'


def test_moving_slot_prefers_clear_footprint_over_nearer_inflated_candidate():
    import numpy as np
    n,_=node();n.polygons={'tb4':[]};n.in_frame=lambda r,p,f:p
    n.p.update(footprint_padding=.12,preferred_slot_cost=0)
    grid=NS(frame='map',footprint_values=lambda p,*a:np.array([40 if p[0]<4.2 else 0]))
    n.grids={'/tb4/global_costmap/costmap':grid}
    selected=C.choose(n,'tb4',(4.,3.,.2),{})
    assert selected[0]>=4.2 and C.auxiliary_slot_cost(n,'tb4',selected)==0
    # A settled robot stays locked even if a clearer nearby candidate exists.
    n.held_auxiliary_slots={'tb4':(4.,3.,.2)}
    assert C.choose(n,'tb4',(4.,3.,.2),{})==(4.,3.,.2)
