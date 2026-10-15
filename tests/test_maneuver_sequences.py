import sys,math
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import maneuver_sequence_search,distance,advance
from fleet_mission_controller import FleetMissionController as C


def finish(generator):
    while True:
        try:next(generator)
        except StopIteration as result:return result.value


def test_search_can_find_forward_left_then_right_instead_of_one_fixed_arc():
    start=(0.,0.,0.);goal=(1.,0.,0.)
    controls=[(.12,.5),(.12,-.5),(-.12,.5),(-.12,-.5)]
    def safe(p,c,offset):
        return c[0]>0 and ((offset==0 and c[1]>0) or (offset>0 and c[1]<0))
    sequence=finish(maneuver_sequence_search(start,goal,controls,2.,safe,lambda p:distance(p,goal),depth=2))
    assert sequence==[(.12,.5),(.12,-.5)]


def test_search_can_change_gear_and_cannot_select_an_unchecked_segment():
    checked=[]
    def safe(p,c,offset):
        checked.append((offset,c))
        return (offset==0 and c[0]<0) or (offset>0 and c[0]>0 and c[1]>0)
    sequence=finish(maneuver_sequence_search((0.,0.,0.),(0.,1.,0.),[(.12,.5),(.12,-.5),(-.12,.5)],2.,safe,lambda p:distance(p,(0.,1.)),depth=2))
    assert sequence[0][0]<0 and sequence[1][0]>0
    assert all((i*2.,c) in checked for i,c in enumerate(sequence))
    assert finish(maneuver_sequence_search((0.,0.,0.),(1.,0.,0.),[(.1,.5)],2.,lambda *a:False,lambda p:0))==[]


def test_sequence_commits_left_then_right_and_stops_if_second_segment_is_unsafe(monkeypatch):
    r={'sequence':[(.12,.5),(.12,-.5)],'sequence_index':0,'segment_started':0.}
    n=NS(clock_value=0.,p={'collision_horizon':2.},obstacle_recoveries={'tb4':r})
    monkeypatch.setattr(C,'recovery_segment_safe',lambda *a:True)
    assert C.maneuver_sequence_command(n,'tb4',{'tb4':(0.,0.,0.)},{})==(.12,.5)
    n.clock_value=2.1
    assert C.maneuver_sequence_command(n,'tb4',{'tb4':(.2,.1,.9)},{})==(.12,-.5)
    monkeypatch.setattr(C,'recovery_segment_safe',lambda *a:False)
    assert C.maneuver_sequence_command(n,'tb4',{'tb4':(.2,.1,.9)},{})==(0.,0.)
    assert 'sequence' not in r


def test_auxiliary_heading_accepts_fifteen_degrees_while_hilti_stays_precise():
    import yaml
    root=Path(__file__).resolve().parents[1]
    p=yaml.safe_load((root/'config/fleet_mission.yaml').read_text())['fleet_mission_controller']['ros__parameters']
    n=NS(p=p,poses={'tb4':(0.,0.,math.radians(14.)),'tb1':(0.,0.,math.radians(14.))})
    assert C.arrived(n,'tb4',(0.,0.,0.))
    assert not C.arrived(n,'tb1',(0.,0.,0.))
    n.poses['tb4']=(0.,0.,math.radians(16.))
    assert not C.arrived(n,'tb4',(0.,0.,0.))
    nav=yaml.safe_load((root/'params/nav2_params.yaml').read_text())
    assert nav['controller_server']['ros__parameters']['goal_checker']['yaw_goal_tolerance']==p['yaw_tolerance']


def test_segment_uses_oriented_physical_peer_clearance_without_reducing_limits():
    box=[(-.2,-.2),(-.2,.2),(.2,.2),(.2,-.2)]
    free=NS(frame='map',footprint_free=lambda *a:True)
    n=NS(p={'collision_horizon':2.,'footprint_padding':.12,'minimum_separation':.4,'primary_minimum_separation':.6},
        polygons={'tb4':box,'tb1':box},radii={'tb4':math.hypot(.2,.2)},state='BOUNDARY_TRAVELLING',primary=None,
        obstacle_recoveries={'tb4':{}},grids={'/map':free,'/tb4/local_costmap/costmap':free,'/tb4/global_costmap/costmap':free},
        in_frame=lambda r,p,f:p)
    poses={'tb4':(0.,0.,0.),'tb1':(1.1,0.,0.)}
    assert C.recovery_segment_safe(n,'tb4',poses['tb4'],(-.12,.5),poses,{})
    assert not C.recovery_segment_safe(n,'tb4',poses['tb4'],(.12,0.),poses,{})
    poses['tb1']=(.95,0.,0.)
    # Existing peer escape policy permits increasing an already short gap.
    assert C.recovery_segment_safe(n,'tb4',poses['tb4'],(-.12,.5),poses,{})
    assert not C.recovery_segment_safe(n,'tb4',poses['tb4'],(.12,0.),poses,{})


def test_no_progress_turning_loop_is_not_selected():
    sequence=finish(maneuver_sequence_search((0.,0.,0.),(1.,0.,0.),[(0.,.5),(0.,-.5)],2.,lambda *a:True,lambda p:distance(p,(1.,0.)),depth=3))
    assert sequence==[]


def test_parking_recovery_backs_through_a_long_arc_before_changing_gear():
    start=(0.,0.,0.);checked=[]
    def safe(p,c,offset):checked.append((c,offset));return True
    sequence=finish(maneuver_sequence_search(start,(-2.,0.,0.),[(-.12,.15),(-.12,-.15),(.12,.15)],2.,safe,
        lambda p:distance(p,(-2.,0.)),parking_distance=.65,parking_rate=.15))
    assert sequence and sequence[0][0]<0
    reverse_distance=0.
    for command in sequence:
        if command[0]>=0:break
        reverse_distance+=abs(command[0])*2.
    assert reverse_distance>=.65
    assert all(abs(c[1])<=.3 for c in sequence)
    assert all((c,i*2.) in checked for i,c in enumerate(sequence))
