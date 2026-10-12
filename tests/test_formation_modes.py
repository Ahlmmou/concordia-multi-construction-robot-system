import math
import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import formation_slots, layout_clearance, ROBOTS, trail_target
from fleet_mission_controller import FleetMissionController

D=dict(rectangle_half_length=4.2,rectangle_half_width=3.4,
       compact_half_length=2.8,compact_half_width=2.2,transit_spacing=3.,
       wall_workspace_depth=3.,wall_row_spacing=1.8,wall_half_width=2.2)
POLY={r:([[-1.2,-.85],[-1.2,.85],[1.4,.85],[1.4,-.85]] if r=='tb1' else
         [[-.3,-.3],[-.3,.3],[.475,.3],[.475,-.3]]) for r in ROBOTS}


def test_all_modes_preserve_footprint_clearance_when_rotated():
    for mode in ('OPEN_RECTANGLE','COMPACT_RECTANGLE','SINGLE_FILE','WALL_WORKSPACE'):
        for angle in (0.,.7,math.pi/2):
            leader=(4.*math.cos(angle),4.*math.sin(angle),angle)
            slots=formation_slots(mode,leader,D,(0.,0.,angle))
            assert layout_clearance({'tb1':leader,**slots},POLY,.4,.6)==''


def test_compact_configuration_cannot_override_physical_clearance():
    slots=formation_slots('COMPACT_RECTANGLE',(0.,0.,0.),{**D,'compact_half_length':.5,'compact_half_width':.3})
    assert 'footprint clearance' in layout_clearance({'tb1':(0.,0.,0.),**slots},POLY,.4,.6)


def test_single_file_trail_retains_bend():
    trail=[(0.,0.,0.),(5.,0.,0.),(5.,5.,math.pi/2)]
    assert trail_target(trail,6.)==(4.,0.,0.)
    assert trail_target(trail,3.)==(5.,2.,math.pi/2)


def test_wall_workspace_stays_on_accessible_side_outside_reserved_depth():
    slots=formation_slots('WALL_WORKSPACE',(2.,1.,0.),D,(0.,0.,0.))
    assert all(p[0]>=5. and abs(p[1]-1.)==2.2 for p in slots.values())


def test_wall_wrong_side_rejected():
    import pytest
    with pytest.raises(ValueError,match='accessible side'):
        formation_slots('WALL_WORKSPACE',(-2.,1.,0.),D,(0.,0.,0.))


def test_cancellation_ack_does_not_release_action_ownership():
    h=object();node=NS(slots={'tb2':{'handle':h,'cancelling':True}},get_logger=lambda:NS(warning=lambda s:None))
    FleetMissionController.cancelled(node,NS(result=lambda:NS(goals_canceling=[1])),'tb2')
    assert node.slots['tb2']['handle'] is h
    assert node.slots['tb2']['cancelling']


def test_trail_waypoint_visits_corner_before_advancing_down_next_leg():
    from formation_geometry import trail_waypoint
    trail=[(0.,0.,0.),(5.,0.,0.),(5.,5.,math.pi/2)]
    target,cursor=trail_waypoint(trail,(4.8,0.,0.),2.,1.,0.)
    assert target[:2]==(5.,0.)
    target,cursor=trail_waypoint(trail,(5.,.3,math.pi/2),2.,1.,cursor)
    assert target[:2]==(5.,1.2999999999999998)
    assert cursor>=5.


def test_trail_waypoint_never_advances_into_leader_spacing():
    from formation_geometry import trail_waypoint
    target,cursor=trail_waypoint([(0.,0.,0.),(10.,0.,0.)],(8.,0.,0.),3.,1.,0.)
    assert target[:2]==(7.,0.)
    assert cursor==7.


def test_trail_waypoint_advances_when_exactly_at_corner():
    from formation_geometry import trail_waypoint
    target,cursor=trail_waypoint([(0.,0.,0.),(5.,0.,0.),(5.,5.,math.pi/2)],(5.,0.,0.),2.,1.,5.)
    assert target[:2]==(5.,1.)


def transition_node():
    import time
    leader=(0.,0.,0.);targets=formation_slots('COMPACT_RECTANGLE',leader,D)
    node=NS(transition_started=time.monotonic(),transition_state='CANCELLING',p={**D,'transition_timeout':600.,'minimum_separation':.4,'primary_minimum_separation':.6,'arrival_tolerance':.4,'wall_x':0.,'wall_y':0.,'wall_normal_yaw':0.},
        clearance_maneuver=None,unresolved_pairs=set(),blocked=set(),yielding={},conflict_since={},clock_value=100.,
        mode_targets=None,mode_anchor=None,poses={'tb1':leader,**targets},requested_mode='COMPACT_RECTANGLE',active_mode='OPEN_RECTANGLE',
        open_dimensions=(4.2,3.4),polygons=POLY,targets={},transition_robot=None,
        idle_actions=lambda:False,feasible=lambda *a,**k:True,arrived=lambda *a:True,
        cancel_all=lambda:None,mode_blocked=lambda reason:None,set_state=lambda *a:None,
        slots={r:{'pending':False,'handle':None} for r in ROBOTS})
    node.complete_mode=lambda:FleetMissionController.complete_mode(node)
    return node


def test_transition_waits_for_owned_action_results():
    node=transition_node()
    FleetMissionController.mode_tick(node)
    assert node.transition_state=='CANCELLING'
    assert node.mode_targets is None
    assert node.active_mode=='OPEN_RECTANGLE'


def test_transition_cannot_complete_from_goal_sending_or_action_success_only():
    node=transition_node();node.transition_state='ASSEMBLING';node.mode_targets=formation_slots('COMPACT_RECTANGLE',node.poses['tb1'],D);node.mode_anchor=node.poses['tb1']
    node.arrived=lambda *a:False;node.idle_actions=lambda:True;node.send=lambda *a:None
    FleetMissionController.mode_tick(node)
    assert node.active_mode=='OPEN_RECTANGLE'
    assert node.transition_state=='ASSEMBLING'


def test_transition_needs_pose_arrival_and_idle_actions_to_complete():
    node=transition_node();node.transition_state='ASSEMBLING';node.mode_targets=formation_slots('COMPACT_RECTANGLE',node.poses['tb1'],D);node.mode_anchor=node.poses['tb1']
    FleetMissionController.mode_tick(node)
    assert node.active_mode=='OPEN_RECTANGLE'
    node.idle_actions=lambda:True
    FleetMissionController.mode_tick(node)
    assert node.active_mode=='COMPACT_RECTANGLE'
    assert node.transition_state=='COMPLETE'


def test_unfittable_exact_targets_block_before_any_navigation():
    node=transition_node();node.idle_actions=lambda:True;node.feasible=lambda *a,**k:False
    reasons=[];node.mode_blocked=reasons.append
    FleetMissionController.mode_tick(node)
    assert reasons and 'cannot fit' in reasons[0]
    assert node.active_mode=='OPEN_RECTANGLE'


def test_transition_recovery_moves_only_current_auxiliary_to_checked_waypoint():
    node=transition_node();node.transition_state='ASSEMBLING';node.mode_anchor=node.poses['tb1'];node.mode_targets=formation_slots('COMPACT_RECTANGLE',node.mode_anchor,D)
    node.transition_robot='tb2';node.unresolved_pairs={('tb2','tb1')};node.conflict_since={('tb2','tb1'):0.};node.p['yield_recovery_delay']=3.
    checked=[];cancelled=[];sent=[]
    node.clearance_target=lambda r,p:checked.append((r,p)) or (4.,4.,0.)
    node.cancel=cancelled.append;node.send=lambda r,t:sent.append((r,t));node.arrived=lambda *a:False
    FleetMissionController.mode_tick(node)
    assert checked==[('tb2','tb1')] and cancelled==['tb2'] and sent==[]
    FleetMissionController.mode_tick(node)
    assert sent==[('tb2',(4.,4.,0.))]
    assert node.active_mode=='OPEN_RECTANGLE'


def test_transition_recovers_when_yielding_to_a_stationary_priority_peer():
    node=transition_node();node.transition_state='ASSEMBLING';node.mode_anchor=node.poses['tb1'];node.mode_targets=formation_slots('COMPACT_RECTANGLE',node.mode_anchor,D)
    node.transition_robot='tb4';node.blocked={'tb4'};node.yielding={'tb4':'tb3'};node.p['yield_recovery_delay']=3.;node.conflict_since={('tb3','tb4'):0.}
    node.clearance_target=lambda r,p:(6.,-4.,0.);node.cancel=lambda r:None
    FleetMissionController.mode_tick(node)
    assert node.clearance_maneuver=={'robot':'tb4','peer':'tb3','target':(6.,-4.,0.)}


def test_dense_actual_trail_samples_do_not_reduce_waypoint_below_arrival_tolerance():
    from formation_geometry import trail_waypoint
    trail=[(i*.1,0.,0.) for i in range(101)]
    target,cursor=trail_waypoint(trail,(1.,0.,0.),3.,.8,0.)
    assert abs(target[0]-1.8)<1e-8
    assert cursor==1.


def test_arrival_tolerance_can_advance_past_a_trail_corner():
    from formation_geometry import trail_waypoint
    trail=[(0.,0.,0.),(5.,0.,0.),(5.,5.,math.pi/2)]
    _,completed=trail_waypoint(trail,(5.,0.,0.),0.,0.,4.7)
    target,cursor=trail_waypoint(trail,(4.7,0.,0.),2.,.8,completed)
    assert target[1]>.4
    assert cursor==5.


def test_single_file_default_route_is_horizontal_before_turning_to_slot():
    from formation_geometry import horizontal_first_routes
    route=next(horizontal_first_routes((4.7,5.,0.),(-2.5,1.6,0.)))
    assert route[0][:2]==(-2.5,5.)
    assert route[1]==(-2.5,1.6,0.)


def test_horizontal_assembly_clears_rear_robot_before_front_robot_passes():
    from formation_geometry import horizontal_first_routes,stationary_route_clear,rectangle
    poses={'tb1':(.5,1.6,0.),**rectangle((.5,1.6,0.),4.2,3.4)}
    slots=formation_slots('SINGLE_FILE',poses['tb1'],D)
    front=next(horizontal_first_routes(poses['tb2'],slots['tb2']))
    rear=next(horizontal_first_routes(poses['tb5'],slots['tb5']))
    assert not stationary_route_clear(poses['tb2'],front,'tb2',poses,POLY,.4,.6)
    assert stationary_route_clear(poses['tb5'],rear,'tb5',poses,POLY,.4,.6)
    poses['tb5']=slots['tb5']
    assert stationary_route_clear(poses['tb2'],front,'tb2',poses,POLY,.4,.6)


def test_horizontal_route_rejects_a_map_obstacle():
    from formation_geometry import horizontal_first_routes,stationary_route_clear
    start=(4.,5.,0.);target=(-2.,1.6,0.)
    route=next(horizontal_first_routes(start,target))
    assert not stationary_route_clear(start,route,'tb2',{'tb2':start},POLY,.4,.6,free=lambda p:abs(p[0]-1.)>.2)


def test_blocked_horizontal_route_has_only_one_outward_clearance_step():
    from formation_geometry import horizontal_first_routes
    routes=list(horizontal_first_routes((0.,2.,0.),(-8.,1.6,0.),clearance_step=1.))
    assert len(routes)==2
    assert routes[1][0][:2]==(0.,3.)
    assert routes[1][1][:2]==(-8.,3.)


def test_horizontal_clearance_does_not_wait_for_an_unnecessary_turn_at_elbow():
    from formation_geometry import distance,wrap
    poses={r:(i*3.,0.,0.) for i,r in enumerate(ROBOTS)};poses['tb5']=(0.,5.,0.)
    final={**poses,'tb5':(0.,1.6,0.)};sent=[]
    node=NS(poses=poses,mode_targets=final,transition_robot='tb5',assembly_route=[(0.,5.,math.pi),final['tb5']],
        p={'arrival_tolerance':.4},slots={'tb5':{'target':None,'status':'active'}},
        arrived=lambda r,t:distance(poses[r],t)<.4 and abs(wrap(poses[r][2]-t[2]))<.45,
        cancel=lambda r:None,idle_actions=lambda:True,set_state=lambda *a:None,send=lambda r,t:sent.append((r,t)))
    FleetMissionController.horizontal_assembly_tick(node)
    assert sent==[('tb5',final['tb5'])]
    assert node.assembly_stage=='APPROACH_SLOT'


def test_legacy_restore_waits_for_owned_actions_and_restores_original_config():
    node=transition_node();node.requested_mode='LEGACY'
    node.legacy_parameters={'rectangle_half_length':4.2,'rectangle_half_width':3.4,
                            'flexible_boundary':True,'maintain_boundary':True}
    node.p.update(rectangle_half_length=2.8,rectangle_half_width=2.2,flexible_boundary=False)
    FleetMissionController.mode_tick(node)
    assert node.active_mode=='OPEN_RECTANGLE'
    assert node.p['flexible_boundary'] is False
    node.idle_actions=lambda:True
    FleetMissionController.mode_tick(node)
    assert node.active_mode=='LEGACY'
    assert node.p['flexible_boundary'] is True
    assert node.p['rectangle_half_length']==4.2
    assert node.phase=='IDLE'
    assert node.transition_state=='COMPLETE'


def test_original_transit_assembles_work_rectangle_on_primary_arrival():
    events=[]
    node=NS(primary=(10.,0.,0.),arrived=lambda *a:True,
            cancel_all=lambda:events.append('cancel'),set_state=lambda *args:events.append(args))
    FleetMissionController.legacy_transit_tick(node,(10.,0.,0.))
    assert events[0]=='cancel'
    assert node.phase=='FORMING_RECTANGLE'
    assert node.rectangle_targets is None


def test_original_transit_keeps_feasible_alternative_targets():
    trail=[(0.,0.,0.),(20.,0.,0.)];sent=[]
    node=NS(primary=(30.,0.,0.),trail=trail,p={'transit_spacing':3.},
            arrived=lambda *a:False,feasible=lambda *a:False,
            choose=lambda r,target,reserved:(target[0],target[1]+1.,target[2]),
            send=lambda r,target:sent.append((r,target)))
    FleetMissionController.legacy_transit_tick(node,(20.,0.,0.))
    assert len(sent)==5
    assert node.targets['tb1']==node.primary
    assert all(node.targets[r][1]==1. for r in ROBOTS[1:])


def test_transient_trail_obstruction_waits_without_abandoning_mission():
    cancelled=[];states=[]
    node=NS(primary=(30.,0.,0.),clock_value=10.,trail_blocked_since={},
            p={'yield_recovery_delay':3.},trail_recovering={},trail_detours={},poses={'tb2':(-3.,0.,0.)},
            recover_auxiliary=lambda r,reason:cancelled.append('recover '+r),
            feasible=lambda robot,*a,**k:robot!='tb2',cancel=lambda r:cancelled.append(r),
            cancel_all=lambda:cancelled.append('all'),set_state=lambda *a:states.append(a))
    result=FleetMissionController.trail_navigation_targets(node,(0.,0.,0.),{'tb2':(-2.,0.,0.),'tb3':(-6.,0.,0.)})
    assert result=={'tb1':node.primary,'tb3':(-6.,0.,0.)}
    assert cancelled==['tb2']
    assert states[-1][0]=='TRAVELLING'
    node.clock_value=131.
    assert FleetMissionController.trail_navigation_targets(node,(0.,0.,0.),{'tb2':(-2.,0.,0.)})=={'tb1':node.primary}
    assert 'recover tb2' in cancelled
    assert 'all' not in cancelled
    assert states[-1][0]=='TRAVELLING'


def test_trail_retry_clears_timer_when_waypoint_becomes_free():
    node=NS(primary=(30.,0.,0.),clock_value=20.,trail_blocked_since={'tb2':10.},trail_recovering={},
            feasible=lambda *a,**k:True,set_state=lambda *a:None)
    target=(-2.,0.,0.)
    assert FleetMissionController.trail_navigation_targets(node,(0.,0.,0.),{'tb2':target})['tb2']==target
    assert not node.trail_blocked_since


def test_single_file_assembly_dispatches_all_free_robots_in_one_tick():
    from formation_geometry import distance
    poses={r:(-4.*i,0.,0.) for i,r in enumerate(ROBOTS)}
    targets={r:(poses[r][0]+1.,0.,0.) for r in ROBOTS[1:]};sent=[]
    node=NS(poses=poses,mode_targets=targets,assembly_routes={},assembly_blocked_since={},blocked=set(),yielding={},polygons=POLY,
            p={'single_file_assembly_axis_yaw':0.,'single_file_clearance_step':1.,
               'minimum_separation':.4,'primary_minimum_separation':.6,'arrival_tolerance':.4},
            slots={r:{'pending':False,'handle':None,'target':None,'status':'idle'} for r in ROBOTS[1:]},
            feasible=lambda *a,**k:True,arrived=lambda r,t:distance(poses[r],t)<.4,
            cancel=lambda r:None,send=lambda r,t:sent.append(r),set_state=lambda *a:None,
            complete_mode=lambda:None)
    FleetMissionController.concurrent_line_assembly_tick(node)
    assert set(sent)==set(ROBOTS[1:])


def test_assembly_rerouting_considers_both_sides_and_larger_detours():
    from formation_geometry import assembly_reroute_candidates
    routes=list(assembly_reroute_candidates((0.,1.,0.),(-3.,0.,0.)))
    assert any(route[0][1]<1. for route in routes)
    assert any(route[0][1]>1. for route in routes)
    assert any(abs(route[0][1]-1.)==3. for route in routes)
    assert all(route[-1]==(-3.,0.,0.) for route in routes)


def test_actual_footprint_route_can_clear_hilti_when_bounding_discs_overlap():
    from formation_geometry import stationary_route_clear
    start=(-2.75,0.,0.);target=(-3.,-.3,0.)
    assert stationary_route_clear(start,[target],'tb2',{'tb1':(0.,0.,0.),'tb2':start},POLY,.4,.6)
    assert not stationary_route_clear(start,[(-1.,0.,0.)],'tb2',{'tb1':(0.,0.,0.),'tb2':start},POLY,.4,.6)


def test_recovering_auxiliary_gets_detour_while_primary_and_other_followers_continue():
    poses={r:(-4.*i,0.,0.) for i,r in enumerate(ROBOTS)};detour=(-4.,2.,0.)
    node=NS(primary=(30.,0.,0.),poses=poses,clock_value=20.,trail_recovering={'tb2':'obstruction'},
        trail_detours={},trail_blocked_since={},slots={'tb2':{'pending':False,'handle':None}},
        clearance_target=lambda *a:detour,feasible=lambda *a,**k:True,
        set_state=lambda *a:None)
    targets=FleetMissionController.trail_navigation_targets(node,poses['tb1'],{'tb2':(-3.,0.,0.),'tb3':(-7.,0.,0.)})
    assert targets=={'tb1':node.primary,'tb2':detour,'tb3':(-7.,0.,0.)}
    assert node.trail_detours['tb2']==detour


def test_detour_completion_returns_robot_to_its_trail_slot():
    normal=(-3.,0.,0.);detour=(-4.,2.,0.);cancelled=[]
    node=NS(primary=(30.,0.,0.),poses={'tb2':detour},clock_value=20.,
        trail_recovering={'tb2':'obstruction'},trail_detours={'tb2':detour},trail_blocked_since={},
        slots={'tb2':{'pending':False,'handle':None}},arrived=lambda *a:True,
        cancel=lambda r:cancelled.append(r),feasible=lambda *a,**k:True,set_state=lambda *a:None)
    assert FleetMissionController.trail_navigation_targets(node,(0.,0.,0.),{'tb2':normal})['tb2']==normal
    assert cancelled==['tb2']
    assert not node.trail_recovering and not node.trail_detours


def test_short_backtracking_noise_does_not_create_a_false_turn_waypoint():
    from formation_geometry import trail_waypoint
    trail=[(-5.,0.,0.),(0.,0.,0.),(-.12,.05,3.),(.1,0.,0.),(10.,0.,0.)]
    point,_=trail_waypoint(trail,(-.1,.02,0.),3.,2.5,4.95)
    assert point[0]>1.
    assert abs(point[2])<.2


def test_single_file_lag_holds_translation_but_allows_collision_checked_turning():
    from formation_geometry import follower_lag_gate
    poses={r:(10.*i,0.,0.) for i,r in enumerate(ROBOTS)}
    commands={r:(.6,.3) for r in ROBOTS}
    assert follower_lag_gate(poses,commands,5.,allow_turning=True)['tb1']==(0.,.3)
    assert follower_lag_gate(poses,commands,5.)['tb1']==(0.,0.)
    assert follower_lag_gate(poses,commands,5.,recovering={'tb2'},allow_turning=True)['tb1']==(.6,.3)
