import sys,math
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController as C
from formation_geometry import ROBOTS

HILTI=[(-1.2,-.85),(-1.2,.85),(1.4,.85),(1.4,-.85)]
BOX=[(-.3,-.3),(-.3,.3),(.475,.3),(.475,-.3)]

def node(free=lambda p:True):
    cancelled=[];states=[]
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)};poses['tb1']=(0.,0.,0.)
    polygons={r:HILTI if r=='tb1' else BOX for r in ROBOTS}
    grid=NS(frame='map',footprint_free=lambda p,*a:free(p))
    n=NS(p={'collision_horizon':2.,'minimum_separation':.4,'primary_minimum_separation':.6,'footprint_padding':.12,
        'obstacle_escape_speed':.12,'rolling_turn_rate':.15,'primary_maneuver_delay':.5,'primary_maneuver_distance':.6,
        'primary_maneuver_angle':.35,'primary_maneuver_timeout':12.,'obstacle_recovery_delay':3.,
        'obstacle_release_padding':.2,'maximum_retries':3},
        poses=poses,polygons=polygons,radii={r:max(math.hypot(x,y) for x,y in poly) for r,poly in polygons.items()},
        primary=(10.,0.,0.),clock_value=0.,slots={r:dict(pending=False,handle=None,retries=0,cooldown=0.) for r in ROBOTS},
        grids={t:grid for t in ['/map','/tb1/global_costmap/costmap','/tb1/local_costmap/costmap']},
        obstacle_recoveries={'tb1':{'target':None,'mode':'primary_maneuver','start_pose':(0.,0.,0.),'started':0.}},
        obstacle_since={},local_blocked=set(),blocked=set(),unresolved_pairs=set(),yielding={},commands={},
        in_frame=lambda r,p,f:p,cancel=cancelled.append,cancel_all=lambda:cancelled.append('all'),
        arrived=lambda *a:False,set_state=lambda *a:states.append(a),get_logger=lambda:NS(info=lambda *a:None))
    return n,cancelled,states


def test_blocked_forward_motion_checks_reverse_before_forward_or_stationary_turn():
    n,*_=node();n.poses['tb4']=(2.5,0.,0.)
    command=C.primary_maneuver_command(n,n.poses,{})
    assert command[0]<0 and C.primary_maneuver_safe(n,command,n.poses,{})


def test_reverse_arc_is_available_when_straight_reverse_is_obstructed():
    n,*_=node(free=lambda p:not (p[0]<-.1 and abs(p[1])<.005))
    command=C.primary_maneuver_command(n,n.poses,{})
    assert command[0]<0 and command[1]!=0


def test_forward_or_turn_alternatives_are_checked_when_every_reverse_is_blocked():
    n,*_=node(free=lambda p:p[0]>=-.001)
    command=C.primary_maneuver_command(n,n.poses,{})
    assert command[0]>0
    n,*_=node(free=lambda p:abs(p[0])<.001 and abs(p[1])<.001)
    command=C.primary_maneuver_command(n,n.poses,{})
    assert command==(0.,0.)


def test_no_free_maneuver_or_missing_maps_never_force_movement():
    n,*_=node(free=lambda p:False)
    assert C.primary_maneuver_command(n,n.poses,{})==(0.,0.)
    assert 'No verified' in n.obstacle_recoveries['tb1']['state']
    n,*_=node();n.grids.pop('/map')
    assert C.primary_maneuver_command(n,n.poses,{})==(0.,0.)


def test_reverse_preserves_clearance_to_stationary_and_moving_rear_peers():
    n,*_=node();n.poses['tb3']=(-3.,0.,0.)
    assert C.primary_maneuver_safe(n,(-.12,0.),n.poses,{})
    assert not C.primary_maneuver_safe(n,(-.12,0.),n.poses,{'tb3':(.8,0.)})
    n.poses['tb3']=(-2.,0.,0.)
    assert not C.primary_maneuver_safe(n,(-.12,0.),n.poses,{})


@pytest.mark.parametrize('pending,handle',[(True,None),(False,object())])
def test_no_recovery_command_until_owned_navigation_action_has_finished_cancelling(pending,handle):
    n,*_=node();n.slots['tb1'].update(pending=pending,handle=handle)
    assert C.primary_maneuver_command(n,n.poses,{})==(0.,0.)
    assert C.obstacle_escape_command(n,'tb1',n.poses)==(0.,0.)


@pytest.mark.parametrize('pose,elapsed', [((-.59,0.,0.),0.),((0.,0.,.34),0.),((0.,0.,0.),12.)])
def test_local_maneuver_obeys_distance_angle_and_time_bounds(pose,elapsed):
    n,*_=node();n.poses['tb1']=pose;n.clock_value=elapsed
    assert C.primary_maneuver_command(n,n.poses,{})==(0.,0.)


def test_peer_block_triggers_recovery_without_releasing_action_ownership():
    n,cancelled,states=node();n.obstacle_recoveries={};owner=object();n.slots['tb1']['handle']=owner
    n.unresolved_pairs={('tb1','tb4')}
    C.obstacle_tick(n)
    assert not cancelled
    n.clock_value=.51;C.obstacle_tick(n)
    assert cancelled==['tb1'] and not states
    assert n.slots['tb1']['handle'] is owner
    assert n.obstacle_recoveries['tb1']['mode']=='primary_maneuver'
    assert n.primary_maneuver_attempts==1


def test_finished_maneuver_discards_stale_velocity_and_replans_original_goal():
    n,*_=node();goal=n.primary;n.poses['tb1']=(-.59,0.,0.);n.commands['tb1']=object()
    n.slots['tb1']['target']=(5.,0.,0.)
    C.obstacle_tick(n)
    assert n.primary==goal and 'tb1' not in n.obstacle_recoveries
    assert 'tb1' not in n.commands and n.slots['tb1']['target'] is None
    assert n.slots['tb1']['last_send']==0.


def test_repeated_maneuvers_are_bounded_until_actual_goal_progress_is_regained():
    n,cancelled,states=node();n.obstacle_recoveries={};n.primary_maneuver_attempts=3
    n.primary_maneuver_goal_distance=10.;n.unresolved_pairs={('tb1','tb4')};n.obstacle_since={'tb1':0.};n.clock_value=4.
    C.obstacle_tick(n)
    assert 'all' in cancelled and states[-1][0]=='WAITING'
    assert 'bounded reverse/turn' in states[-1][1]


def test_existing_inflation_escape_can_back_out_without_crossing_lethal_cells():
    import numpy as np
    from formation_geometry import Grid
    n,*_=node()
    g=Grid.__new__(Grid);g.frame='map';g.origin=(-5.,-5.);g.angle=0.;g.resolution=.05
    g.cells=np.zeros((200,200),dtype=np.int8);g.cells[:,130:]=99;g.cells[:,136:]=100
    n.grids['/tb1/global_costmap/costmap']=g
    assert not g.footprint_free(n.poses['tb1'],HILTI,.145,98)
    assert C.primary_maneuver_safe(n,(-.12,0.),n.poses,{})
    assert not C.primary_maneuver_safe(n,(.12,0.),n.poses,{})
    g.cells[80:120,70:100]=100
    assert not C.primary_maneuver_safe(n,(-.12,0.),n.poses,{})


def test_recovery_budget_is_not_reset_by_backtracking_away_from_the_goal():
    n,cancelled,states=node();n.obstacle_recoveries={};n.primary_maneuver_attempts=1;n.primary_maneuver_goal_distance=10.
    n.poses['tb1']=(-.6,0.,0.);n.unresolved_pairs={('tb1','tb4')};n.obstacle_since={'tb1':0.};n.clock_value=1.
    C.obstacle_tick(n)
    assert n.primary_maneuver_attempts==2 and n.primary_maneuver_goal_distance==10.
    n.obstacle_recoveries={};n.unresolved_pairs=set();n.poses['tb1']=(.4,0.,0.)
    C.obstacle_tick(n)
    assert n.primary_maneuver_attempts==0


def test_reverse_continues_until_full_forward_turn_fits(monkeypatch):
    n,*_=node();n.p['primary_maneuver_distance']=1.8;n.p['primary_maneuver_angle']=1.2
    n.primary=(5.,4.,0.);n.poses['tb1']=(-.3,0.,0.)
    n.obstacle_recoveries['tb1']['command']=(-.12,0.)
    checks=[]
    def safe(self,command,poses,peers,horizon=None):
        checks.append(horizon)
        return command[0]<0 or (horizon is not None and poses['tb1'][0]<-.7)
    monkeypatch.setattr(C,'primary_maneuver_safe',safe)
    assert C.primary_maneuver_command(n,n.poses,{})[0]<0
    assert any(h is not None and h>n.p['collision_horizon'] for h in checks)
    n.poses['tb1']=(-.8,0.,0.)
    command=C.primary_maneuver_command(n,n.poses,{})
    assert command[0]>0 and command[1]>0
    assert n.obstacle_recoveries['tb1']['phase']=='forward'


def test_replans_only_after_checked_forward_alignment_and_progress():
    n,*_=node();n.p['primary_maneuver_distance']=1.8
    recovery=n.obstacle_recoveries['tb1'];recovery.update(command=(-.12,0.))
    n.poses['tb1']=(-.3,0.,0.)
    assert C.primary_maneuver_command(n,n.poses,{})[0]>0
    assert not recovery.get('forward_ready')
    n.poses['tb1']=(-.13,0.,0.)
    assert C.primary_maneuver_command(n,n.poses,{})==(0.,0.)
    assert recovery['forward_ready']
    C.obstacle_tick(n)
    assert 'tb1' not in n.obstacle_recoveries
