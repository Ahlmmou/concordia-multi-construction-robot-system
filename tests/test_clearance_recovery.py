import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController
from formation_geometry import ROBOTS,rectangle,footprint_clearance


def test_clearance_waypoint_improves_separation_and_rejects_blocked_map():
    box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses.update(tb1=(0.,0.,0.),tb4=(1.1,0.,0.))
    node=NS(poses=poses,polygons={r:box for r in ROBOTS},targets={'tb4':(-2.,0.,0.)},
        p={'minimum_separation':.4,'yield_clearance_distance':1.5},feasible=lambda *a,**kw:True)
    target=FleetMissionController.clearance_target(node,'tb4','tb1')
    assert target is not None
    assert footprint_clearance(target,box,poses['tb1'],box)>=.8-1e-6
    node.feasible=lambda *a,**kw:False
    assert FleetMissionController.clearance_target(node,'tb4','tb1') is None


def test_clearance_maneuver_waits_then_automatically_releases_peer():
    cancelled=[]
    node=NS(clock_value=0.,clearance_maneuver=None,conflict_since={},unresolved_pairs={('tb4','tb1')},
        p={'yield_recovery_delay':3.},slots={'tb4':{'retries':2,'cooldown':10}},
        clearance_target=lambda *a:(2.,0.,0.),cancel=cancelled.append,arrived=lambda *a:False,
        get_logger=lambda:NS(info=lambda *a:None))
    FleetMissionController.clearance_tick(node)
    assert node.clearance_maneuver is None
    node.clock_value=3.1
    FleetMissionController.clearance_tick(node)
    assert node.clearance_maneuver['peer']=='tb1' and cancelled==['tb4']
    node.arrived=lambda *a:True
    FleetMissionController.clearance_tick(node)
    assert node.clearance_maneuver is None and node.conflict_since=={}


def test_clearance_target_is_frozen_and_peer_goal_is_not_reissued():
    poses={'tb1':(0.,0.,0.),**rectangle((0.,0.,0.),4.2,3.4)};sent=[];cancelled=[]
    waypoint=(5.,-3.,0.)
    node=NS(poses=poses,primary=(20.,0.,0.),boundary_heading=0.,flexible_slots={},flexible_final_targets=None,
        clearance_maneuver={'robot':'tb4','peer':'tb1','target':waypoint},
        p={'rectangle_half_length':4.2,'rectangle_half_width':3.4,'arrival_tolerance':.4},
        arrived=lambda *a:False,set_state=lambda *a:None,choose=lambda r,t,reserved:t,
        cancel=cancelled.append,send=lambda r,t:sent.append((r,t)))
    FleetMissionController.flexible_tick(node)
    assert ('tb4',waypoint) in sent and all(r!='tb1' for r,t in sent)
    assert node.targets['tb1']==node.primary and not cancelled


def test_nav2_timeout_while_yielding_does_not_pause_the_fleet():
    states=[];cancelled=[]
    slot={'retries':2}
    node=NS(slots={'tb1':slot},p={'retry_delay':8.,'maximum_retries':3},yielding={'tb1':'tb5'},
        get_clock=lambda:NS(now=lambda:NS(nanoseconds=0)),get_logger=lambda:NS(info=lambda *a:None,warning=lambda *a:None),
        cancel_all=lambda:cancelled.append(True),set_state=lambda *a:states.append(a))
    FleetMissionController.failure(node,'tb1','Nav2 progress timeout')
    assert slot['retries']==0 and slot['cooldown']==0 and not cancelled and not states


def test_stalled_auxiliary_near_hilti_triggers_clearance_for_auxiliary():
    import math,time
    from geometry_msgs.msg import Twist
    box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses.update(tb1=(0.,0.,0.),tb4=(1.5,0.,0.))
    turn=Twist();turn.angular.z=.3
    published={}
    node=NS(clock_value=10.,ready=lambda:(True,''),state='BOUNDARY_TRAVELLING',manual=False,
        p={'command_timeout':2.,'leader_speed':.6,'auxiliary_speed':.8,'angular_speed':.5,
           'flexible_boundary':True,'footprint_padding':.1,'minimum_separation':.4,
           'collision_horizon':2.,'yield_recovery_delay':3.},
        commands={'tb4':(turn,time.monotonic())},
        slots={r:{'handle':object() if r=='tb4' else None,'progress_sim':0.} for r in ROBOTS},
        poses=poses,polygons={r:box for r in ROBOTS},radii={r:math.hypot(.3,.3) for r in ROBOTS},
        targets={'tb4':(3.,0.,0.)},clearance_maneuver=None,
        grids={f'/{r}/local_costmap/costmap':NS(frame='map',footprint_free=lambda *a:True) for r in ROBOTS},
        in_frame=lambda r,p,f:p,arrived=lambda *a:False,
        cmd_pubs={r:NS(publish=lambda m,r=r:published.update({r:(m.linear.x,m.angular.z)})) for r in ROBOTS})
    # Use a peer within the near-peer detection radius but with safe turn clearance.
    node.radii['tb1']=.8
    node.grids.setdefault('/map',NS(frame='map',footprint_free=lambda *a:True))
    FleetMissionController.gate(node)
    assert ('tb1','tb4') in node.unresolved_pairs
    assert node.yielding['tb4']=='tb1' and published['tb1']==(0.,0.)
    assert published['tb4']==(0.,0.)


def test_gate_uses_physical_poses_when_tf_lag_overstates_clearance():
    import math,time
    from geometry_msgs.msg import Twist
    box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses.update(tb1=(0.,0.,0.),tb4=(1.7,0.,0.))
    physical=dict(poses);physical['tb4']=(1.4,0.,0.)
    command=Twist();command.linear.x=-.15
    published={}
    node=NS(clock_value=10.,ready=lambda:(True,''),state='BOUNDARY_TRAVELLING',manual=False,
        p={'command_timeout':2.,'leader_speed':.6,'auxiliary_speed':.8,'angular_speed':.5,
           'flexible_boundary':True,'footprint_padding':.1,'minimum_separation':.4,
           'primary_minimum_separation':.6,'collision_horizon':2.,'yield_recovery_delay':3.,
           'verify_gazebo_localization':True,'clock_timeout':5.},
        commands={'tb4':(command,time.monotonic())},
        slots={r:{'handle':object() if r=='tb4' else None,'progress_sim':10.} for r in ROBOTS},
        poses=poses,physical_poses=physical,physical_stamp=time.monotonic(),
        polygons={r:box for r in ROBOTS},radii={r:math.hypot(.3,.3) for r in ROBOTS},
        targets={'tb4':(0.,0.,0.)},clearance_maneuver=None,
        grids={f'/{r}/local_costmap/costmap':NS(frame='map',footprint_free=lambda *a:True) for r in ROBOTS},
        in_frame=lambda r,p,f:p,arrived=lambda *a:False,
        cmd_pubs={r:NS(publish=lambda m,r=r:published.update({r:(m.linear.x,m.angular.z)})) for r in ROBOTS})
    node.grids.setdefault('/map',NS(frame='map',footprint_free=lambda *a:True))
    FleetMissionController.gate(node)
    assert published['tb4']==(0.,0.) and 'tb4' in node.blocked
    node.p['verify_gazebo_localization']=False
    node.grids.setdefault('/map',NS(frame='map',footprint_free=lambda *a:True))
    FleetMissionController.gate(node)
    assert published['tb4'][0]==-.15


def test_hilti_priority_moves_blocking_auxiliary_aside_instead_of_detouring_hilti():
    cancelled=[];chosen=[]
    node=NS(clock_value=10.,clearance_maneuver=None,conflict_since={('tb1','tb3'):0.},
        unresolved_pairs={('tb1','tb3')},p={'yield_recovery_delay':3.},
        slots={'tb3':{'retries':2,'cooldown':9.}},arrived=lambda *a:False,
        clearance_target=lambda robot,peer:chosen.append((robot,peer)) or (1.,3.,0.),
        cancel=cancelled.append,get_logger=lambda:NS(info=lambda *a:None))
    FleetMissionController.clearance_tick(node)
    assert chosen==[('tb3','tb1')] and cancelled==['tb3']
    assert node.clearance_maneuver=={'robot':'tb3','peer':'tb1','target':(1.,3.,0.)}
