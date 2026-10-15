import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController as Controller
from formation_geometry import ROBOTS


def node():
    cancelled=[];states=[]
    n=NS(p={'flexible_boundary':True,'footprint_padding':.12,'retry_delay':8.,'maximum_retries':3},
        state='BOUNDARY_TRAVELLING',requested_mode='LEGACY',clock_value=20.,
        poses={r:(i*10.,0.,0.) for i,r in enumerate(ROBOTS)},
        slots={r:dict(pending=False,handle=None,cancelling=False,retries=2,cooldown=0.) for r in ROBOTS},
        obstacle_recoveries={},grids={},polygons={r:[] for r in ROBOTS},yielding={},local_blocked=set(),
        cancel=cancelled.append,cancel_all=lambda:cancelled.append('all'),set_state=lambda *x:states.append(x),
        in_frame=lambda r,p,f:p,get_clock=lambda:NS(now=lambda:NS(nanoseconds=20_000_000_000)),
        get_logger=lambda:NS(info=lambda *x:None,warning=lambda *x:None))
    n.flexible_recovery=lambda r,reason:Controller.flexible_recovery(n,r,reason)
    return n,cancelled,states


def test_auxiliary_retry_limit_starts_own_recovery_without_stopping_fleet():
    n,cancelled,states=node()
    Controller.failure(n,'tb3','No feasible Nav2 path')
    assert cancelled==['tb3'] and not states
    assert n.state=='BOUNDARY_TRAVELLING' and n.slots['tb3']['retries']==0
    assert set(n.obstacle_recoveries)=={'tb3'}


def test_recovery_keeps_owned_action_until_cancellation_result():
    n,cancelled,states=node();h=object();n.slots['tb3']['handle']=h
    n.cancel=lambda r:Controller.cancel(n,r)
    h=NS(cancel_goal_async=lambda:NS(add_done_callback=lambda cb:None))
    n.slots['tb3']['handle']=h
    assert n.flexible_recovery('tb3','actual motion stalled')
    assert n.slots['tb3']['handle'] is h and n.slots['tb3']['cancelling']
    assert not states


@pytest.mark.parametrize('robot,flexible,state',[('tb1',True,'BOUNDARY_TRAVELLING'),('tb3',False,'BOUNDARY_TRAVELLING'),('tb3',True,'STOPPED')])
def test_independent_recovery_does_not_override_other_workflows(robot,flexible,state):
    n,cancelled,states=node();n.p['flexible_boundary']=flexible;n.state=state
    assert not n.flexible_recovery(robot,'stall')
    assert not cancelled and not n.obstacle_recoveries


def test_moving_corner_reserves_current_primary_body_instead_of_destination():
    n,_,_=node();n.targets={'tb1':(4.2,-3.4,0.),'tb4':(4.2,-3.4,0.)}
    assert Controller.navigation_reservations(n,'tb4')=={'tb1':n.poses['tb1'],'tb4':n.targets['tb4']}
    assert Controller.navigation_reservations(n,'tb1')=={}
    n.p['flexible_boundary']=False
    assert Controller.navigation_reservations(n,'tb1')==n.targets


def test_relaxed_future_reservation_still_rejects_actual_peer_overlap():
    n,_,_=node();box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    n.polygons={r:box for r in ROBOTS};n.p.update(minimum_separation=.4,primary_minimum_separation=.6,maximum_target_cost=70)
    n.targets={'tb1':n.poses['tb4'],'tb4':n.poses['tb4']}
    free=NS(frame='map',footprint_free=lambda *x:True)
    n.grids={'/map':free,'/tb1/global_costmap/costmap':free}
    assert not Controller.feasible(n,'tb1',n.targets['tb1'],Controller.navigation_reservations(n,'tb1'))


@pytest.mark.parametrize('pending',[False,True])
def test_watchdog_recovers_only_stalled_auxiliary_and_keeps_flexible_tick_running(pending):
    import time
    n,cancelled,states=node();continued=[]
    n.p.update(action_response_timeout=1.,progress_timeout=1.,auto_form_rectangle=False)
    n.phase='BOUNDARY_TRAVELLING';n.primary=(40.,0.,0.);n.pending_primary=None
    for slot in n.slots.values():slot.update(progress_pose=(0.,0.,0.),progress_sim=0.)
    n.slots['tb3'].update(handle=None if pending else object(),pending=pending,
        pending_since=time.monotonic()-10,progress_pose=n.poses['tb3'])
    n.refresh_poses=lambda:None;n.ready=lambda:(True,'');n.obstacle_tick=lambda:None
    n.publish_markers=lambda:None;n.clearance_tick=lambda:None;n.flexible_tick=lambda:continued.append(True)
    Controller.tick(n)
    assert set(n.obstacle_recoveries)=={'tb3'} and cancelled==['tb3']
    assert continued==[True] and not states
    if pending:assert n.slots['tb3']['cancel_when_accepted']



def test_coincident_future_goals_do_not_deadlock_physically_clear_flexible_travel():
    n,_,_=node();box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    n.polygons={r:box for r in ROBOTS};n.p.update(minimum_separation=.4,primary_minimum_separation=.6,maximum_target_cost=70)
    goal=(4.2,-3.4,0.);n.targets={'tb1':goal,'tb4':goal}
    free=NS(frame='map',footprint_free=lambda *x:True)
    n.grids={'/map':free,'/tb1/global_costmap/costmap':free,'/tb4/global_costmap/costmap':free}
    assert not Controller.feasible(n,'tb1',goal,n.targets)
    assert not Controller.feasible(n,'tb4',goal,n.targets)
    assert Controller.feasible(n,'tb1',goal,Controller.navigation_reservations(n,'tb1'))
    assert Controller.feasible(n,'tb4',goal,Controller.navigation_reservations(n,'tb4'))



@pytest.mark.parametrize('near,expected',[(False,.2),(True,0.)])
def test_healthy_primary_continues_during_auxiliary_recovery_unless_actual_clearance_conflicts(near,expected):
    import time,math
    from geometry_msgs.msg import Twist
    n,_,_=node();box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    n.poses['tb3']=(1.,0.,0.) if near else (20.,0.,0.)
    n.polygons={r:box for r in ROBOTS};n.radii={r:math.hypot(.3,.3) for r in ROBOTS}
    n.p.update(command_timeout=2.,leader_speed=.6,auxiliary_speed=.8,angular_speed=.5,
        minimum_separation=.4,primary_minimum_separation=.6,collision_horizon=2.,yield_recovery_delay=3.)
    command=Twist();command.linear.x=.2
    n.commands={'tb1':(command,time.monotonic())};n.manual=False;n.ready=lambda:(True,'')
    n.targets={'tb1':(50.,0.,0.)};n.slots['tb1']['handle']=object()
    n.obstacle_recoveries={'tb3':{'target':None}};n.obstacle_escape_command=lambda *x:(0.,0.)
    n.clearance_maneuver=None;n.arrived=lambda *x:False
    n.grids={f'/{r}/local_costmap/costmap':NS(frame='map',footprint_free=lambda *x:True) for r in ROBOTS}
    published={};n.cmd_pubs={r:NS(publish=lambda m,r=r:published.update({r:(m.linear.x,m.angular.z)})) for r in ROBOTS}
    n.grids.setdefault('/map',NS(frame='map',footprint_free=lambda *a:True))
    Controller.gate(n)
    assert published['tb1'][0]==expected
    assert published['tb3']==(0.,0.)
