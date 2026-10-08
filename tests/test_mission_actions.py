import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController


def supervisor(target=(1.,2.,0.),handle=None):
    slot=dict(target=target,handle=handle,pending=False,cancelling=False,cooldown=0.,last_send=10.,status='active',retries=0)
    calls=[]
    return NS(slots={'tb2':slot},p={'update_distance':.8,'update_angle':.5,'minimum_goal_interval':4.,'maximum_retries':3},arrived=lambda r,t:False,get_clock=lambda:NS(now=lambda:NS(nanoseconds=20_000_000_000)),cancel=lambda r:calls.append(r)),calls


def test_unchanged_goal_does_not_cancel_active_navigation():
    node,calls=supervisor(handle=object())
    FleetMissionController.send(node,'tb2',(1.1,2.,.1))
    assert calls==[]


def test_significantly_changed_target_cancels_once_before_resending():
    node,calls=supervisor(handle=object())
    FleetMissionController.send(node,'tb2',(2.,2.,0.))
    assert calls==['tb2']
    node.slots['tb2']['cancelling']=True
    FleetMissionController.send(node,'tb2',(2.,2.,0.))
    assert calls==['tb2']


def test_actual_arrival_stops_owned_action():
    node,calls=supervisor(handle=object());node.arrived=lambda r,t:True
    FleetMissionController.send(node,'tb2',(1.,2.,0.))
    assert calls==['tb2']


def test_pending_planner_is_not_started_twice():
    node,calls=supervisor();node.slots['tb2']['pending']=True
    FleetMissionController.send(node,'tb2',(2.,2.,0.))
    assert calls==[]


def boundary_supervisor(state='BOUNDARY_TRAVELLING', blocked=False):
    from formation_geometry import rectangle
    leader=(0.,0.,0.);poses={'tb1':leader,**rectangle(leader,4.2,3.4)};sent=[];states=[]
    node=NS(poses=poses,primary=(2.,0.,0.),boundary_heading=None,boundary_waypoint=None,state=state,
        p={'boundary_step':1.,'rectangle_half_length':4.2,'rectangle_half_width':3.4,'yaw_tolerance':.45},
        targets={},slots={r:dict(target=None,status='idle',handle=None,pending=False) for r in poses},
        feasible=lambda r,p,reserved,peers: not blocked,send=lambda r,t:sent.append((r,t)),
        arrived=lambda r,t:abs(poses[r][0]-t[0])<.01 and abs(poses[r][1]-t[1])<.01,
        idle_actions=lambda:True,cancel_all=lambda:sent.append(('cancel',None)),
        set_state=lambda s,r:states.append((s,r)))
    return node,sent,states


def test_boundary_translates_all_four_corners_with_leader():
    node,sent,states=boundary_supervisor()
    FleetMissionController.boundary_tick(node)
    assert len(sent)==5
    assert node.targets['tb1']==(1.,0.,0.)
    for r in ('tb2','tb3','tb4','tb5'):
        assert node.targets[r][0]-node.poses[r][0]==1.
        assert node.targets[r][1]==node.poses[r][1]


def test_boundary_obstruction_stops_whole_group():
    node,sent,states=boundary_supervisor(blocked=True)
    FleetMissionController.boundary_tick(node)
    assert sent==[('cancel',None)]
    assert states[0][0]=='WAITING'


def test_initial_boundary_assembly_keeps_leader_stationary():
    node,sent,states=boundary_supervisor(state='FORMING_BOUNDARY')
    node.poses['tb2']=(0.,8.,0.)
    FleetMissionController.boundary_tick(node)
    assert [r for r,t in sent]==['tb2']
    assert node.targets['tb1']==node.poses['tb1']


def test_boundary_rotation_uses_small_steps():
    node,sent,states=boundary_supervisor()
    node.primary=(0.5,0.,1.57)
    FleetMissionController.boundary_tick(node)
    assert node.boundary_waypoint[2]==0.1
    assert node.targets['tb1'][2]==1.57


def test_initial_boundary_preserves_corner_axes_despite_leader_yaw():
    node,sent,states=boundary_supervisor(state='FORMING_BOUNDARY')
    node.poses['tb1']=(0.,0.,0.12)
    FleetMissionController.boundary_tick(node)
    assert node.boundary_heading==0.
    assert node.targets['tb2']==(4.2,3.4,0.)
    assert node.targets['tb4']==(4.2,-3.4,0.)


def test_final_boundary_orientation_is_independent_of_leader_steering():
    node,sent,states=boundary_supervisor()
    node.poses['tb1']=(0.,0.,-1.57);node.primary=(.5,0.,0.)
    FleetMissionController.boundary_tick(node)
    assert node.targets['tb1'][2]==0.
    assert node.targets['tb2'][1]==3.4


def test_sideways_stage_faces_travel_direction_without_rotating_rectangle():
    import math
    node,sent,states=boundary_supervisor()
    node.primary=(0.,-3.,-math.pi/2)
    FleetMissionController.boundary_tick(node)
    assert node.targets['tb1']==(0.,-1.,-math.pi/2)
    assert node.targets['tb2']==(4.2,2.4,-math.pi/2)
    assert node.boundary_waypoint[2]==0.
