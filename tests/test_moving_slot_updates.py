import sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from action_msgs.msg import GoalStatus
from fleet_mission_controller import FleetMissionController as Controller


class Future:
    def __init__(self,value):self.value=value;self.callbacks=[]
    def result(self):return self.value
    def add_done_callback(self,callback):self.callbacks.append(callback)


def controller():
    cancels=[];sends=[];plans=[]
    owner=NS(cancel_goal_async=lambda:cancels.append(True) or Future(NS(goals_canceling=[1])))
    slot=dict(handle=owner,pending=False,cancelling=False,target=(0.,0.,0.),last_send=10.,
        cooldown=0.,retries=0,status='active',plan_token=0)
    n=NS(slots={'tb2':slot},p={'flexible_boundary':True,'update_distance':.8,'update_angle':.5,
        'minimum_goal_interval':4.,'maximum_retries':3},state='BOUNDARY_TRAVELLING',requested_mode='LEGACY',
        obstacle_recoveries={},clearance_maneuver=None,targets={'tb2':(2.,0.,0.)},mission_number=1,
        commands={'tb2':object()},clock_value=20.,poses={'tb2':(0.,0.,0.)},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=20_000_000_000)),arrived=lambda *x:False,
        ready=lambda:(True,''),feasible=lambda *x:True,navigation_reservations=lambda r:{},
        pose_message=lambda p:__import__('geometry_msgs.msg',fromlist=['PoseStamped']).PoseStamped(),
        get_logger=lambda:NS(info=lambda *x:None,warning=lambda *x:None,error=lambda *x:None))
    def planned(goal):
        f=Future(None);plans.append(f);return f
    def sent(goal):
        f=Future(None);sends.append(f);return f
    n.planners={'tb2':NS(send_goal_async=planned)};n.nav={'tb2':NS(send_goal_async=sent)}
    for method in ['cancel','cancelled','start_plan','plan_accepted','plan_done','plan_error','dispatch_navigation','nav_accepted','nav_done']:
        setattr(n,method,lambda *a,_method=method,**kw:getattr(Controller,_method)(n,*a,**kw))
    n.failure=lambda *x:pytest.fail('unexpected navigation failure '+str(x))
    return n,slot,owner,cancels,sends,plans


def plan_result(status=GoalStatus.STATUS_SUCCEEDED):
    return Future(NS(status=status,result=NS(path=NS(poses=[1]))))


def prepare(n,slot,owner,target=(2.,0.,0.)):
    Controller.send(n,'tb2',target)
    Controller.plan_done(n,plan_result(), 'tb2',target,slot['plan_token'],1,owner)


def test_new_path_is_requested_while_existing_navigation_keeps_ownership():
    n,s,owner,cancels,sends,plans=controller()
    Controller.send(n,'tb2',(2.,0.,0.))
    assert s['handle'] is owner and s['pending'] and not s['cancelling']
    assert len(plans)==1 and not cancels and not sends
    Controller.send(n,'tb2',(3.,0.,0.))
    assert len(plans)==1


def test_prepared_update_waits_for_old_result_then_dispatches_immediately():
    n,s,owner,cancels,sends,plans=controller();prepare(n,s,owner)
    assert s['handle'] is owner and s['cancelling'] and not sends
    Controller.nav_done(n,Future(NS(status=GoalStatus.STATUS_CANCELED)),'tb2',owner)
    assert s['handle'] is None and s['pending'] and s['status']=='sending'
    assert len(sends)==1 and s['target']==(2.,0.,0.) and s['updates_switched']==1
    assert 'tb2' not in n.commands


@pytest.mark.parametrize('stage',['planning','prepared'])
def test_operator_stop_invalidates_update_and_cancels_owned_action(stage):
    n,s,owner,cancels,sends,plans=controller()
    Controller.send(n,'tb2',(2.,0.,0.))
    if stage=='prepared':Controller.plan_done(n,plan_result(),'tb2',(2.,0.,0.),s['plan_token'],1,owner)
    n.state='STOPPED';Controller.cancel(n,'tb2')
    assert s['cancelling'] and cancels
    Controller.plan_done(n,plan_result(),'tb2',(2.,0.,0.),s['plan_token'],1,owner)
    Controller.nav_done(n,Future(NS(status=GoalStatus.STATUS_CANCELED)),'tb2',owner)
    assert not sends and 'next_target' not in s


def test_failed_update_path_keeps_old_action_and_does_not_consume_retries():
    n,s,owner,cancels,sends,plans=controller();Controller.send(n,'tb2',(2.,0.,0.))
    Controller.plan_done(n,plan_result(GoalStatus.STATUS_ABORTED),'tb2',(2.,0.,0.),s['plan_token'],1,owner)
    assert s['handle'] is owner and not s['pending'] and s['retries']==0 and not cancels
    Controller.send(n,'tb2',(2.,0.,0.))
    assert len(plans)==1 # Failed updates retain the per-robot planning interval.


@pytest.mark.parametrize('change',['target','mission','recovery','owner','obstacle'])
def test_stale_or_unsafe_prepared_paths_never_cancel_current_motion(change):
    n,s,owner,cancels,sends,plans=controller();Controller.send(n,'tb2',(2.,0.,0.))
    if change=='target':n.targets['tb2']=(4.,0.,0.)
    if change=='mission':n.mission_number=2
    if change=='recovery':n.obstacle_recoveries['tb2']={'target':None}
    if change=='owner':s['handle']=object()
    if change=='obstacle':n.feasible=lambda *x:False
    Controller.plan_done(n,plan_result(),'tb2',(2.,0.,0.),s['plan_token'],1,owner)
    assert not cancels and not sends and not s['pending']


@pytest.mark.parametrize('change',['stale_data','obstacle','target','recovery'])
def test_post_cancellation_recheck_prevents_unsafe_handoff(change):
    n,s,owner,cancels,sends,plans=controller();prepare(n,s,owner)
    if change=='stale_data':n.ready=lambda:(False,'stale')
    if change=='obstacle':n.feasible=lambda *x:False
    if change=='target':n.targets['tb2']=(5.,0.,0.)
    if change=='recovery':n.obstacle_recoveries['tb2']={'target':None}
    Controller.nav_done(n,Future(NS(status=GoalStatus.STATUS_CANCELED)),'tb2',owner)
    assert not sends and s['handle'] is None


def test_late_old_plan_does_not_clear_new_pending_request():
    n,s,owner,cancels,sends,plans=controller();s.update(pending=True,plan_token=2)
    Controller.plan_done(n,plan_result(),'tb2',(2.,0.,0.),1,1,owner)
    assert s['pending'] and not cancels


def test_unrelated_cancel_drops_prepared_target_and_preserves_action_ownership():
    n,s,owner,cancels,sends,plans=controller();prepare(n,s,owner)
    Controller.cancel(n,'tb2')
    assert s['handle'] is owner and 'next_target' not in s



def test_planning_transport_error_preserves_existing_action():
    n,s,owner,cancels,sends,plans=controller()
    def broken(goal):raise RuntimeError('transport unavailable')
    n.planners['tb2'].send_goal_async=broken
    Controller.send(n,'tb2',(2.,0.,0.))
    assert s['handle'] is owner and not s['pending'] and not cancels


@pytest.mark.parametrize('cancel_pending,expected',[(False,.2),(True,0.)])
def test_motor_gate_keeps_safe_owned_motion_during_planning_but_stops_for_cancellation(cancel_pending,expected):
    import time,math
    from geometry_msgs.msg import Twist
    from formation_geometry import ROBOTS
    n,s,owner,cancels,sends,plans=controller();Controller.send(n,'tb2',(2.,0.,0.))
    for r in ROBOTS:
        if r!='tb2':n.slots[r]=dict(handle=None,pending=False,cancelling=False)
    n.poses={r:(i*10.,0.,0.) for i,r in enumerate(ROBOTS)}
    box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    n.polygons={r:box for r in ROBOTS};n.radii={r:math.hypot(.3,.3) for r in ROBOTS}
    n.p.update(command_timeout=2.,leader_speed=.6,auxiliary_speed=.8,angular_speed=.5,
        footprint_padding=.12,minimum_separation=.4,primary_minimum_separation=.6,collision_horizon=2.,yield_recovery_delay=3.)
    cmd=Twist();cmd.linear.x=.2;n.commands={'tb2':(cmd,time.monotonic())};n.manual=False
    n.in_frame=lambda r,p,f:p
    n.grids={f'/{r}/local_costmap/costmap':NS(frame='map',footprint_free=lambda *x:True) for r in ROBOTS}
    n.targets={'tb2':(30.,0.,0.)}
    published={};n.cmd_pubs={r:NS(publish=lambda m,r=r:published.update({r:(m.linear.x,m.angular.z)})) for r in ROBOTS}
    if cancel_pending:Controller.cancel(n,'tb2')
    Controller.gate(n)
    assert published['tb2'][0]==expected
