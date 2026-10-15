import sys,math,time
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import ROBOTS,obstacle_detour_candidates,route_preview
from fleet_mission_controller import FleetMissionController as Controller

BOX=[(-.1,-.1),(-.1,.1),(.1,.1),(.1,-.1)]

def node(free=lambda p:True):
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)}
    poses.update(tb2=(0.,.9,0.),tb3=(0.,-.9,0.))
    grid=NS(frame='map',footprint_free=lambda p,*args:free(p))
    n=NS(poses=poses,targets={'tb2':(4.,.9,0.),'tb3':(4.,-.9,0.)},
        polygons={r:BOX for r in ROBOTS},p={'minimum_separation':.4,'primary_minimum_separation':.6,'footprint_padding':.12},
        grids={topic:grid for r in ROBOTS for topic in ['/map',f'/{r}/global_costmap/costmap',f'/{r}/local_costmap/costmap']},
        feasible=lambda *a,**kw:True,in_frame=lambda r,p,f:p)
    return n


def test_waypoints_include_both_sides_and_reversing():
    candidates=list(obstacle_detour_candidates((0.,0.,0.),(5.,0.,0.)))
    assert any(p[1]>0 for p in candidates) and any(p[1]<0 for p in candidates)
    assert any(p[0]<0 for p in candidates)


def test_followers_can_take_opposite_sides_of_the_same_obstacle():
    # Wall immediately ahead; each robot's outer flank is clear. A rear wall
    # rules out simply backing up, so the two independently choose opposite sides.
    n=node(lambda p:not (p[0]<-.1 or (p[0]>.1 and abs(p[1])<1.4)))
    upper=Controller.obstacle_detour_target(n,'tb2',n.poses)
    lower=Controller.obstacle_detour_target(n,'tb3',n.poses)
    assert upper is not None and upper[1]>.9
    assert lower is not None and lower[1]<-.9


def test_blocked_maps_or_close_peers_never_force_a_side():
    n=node(lambda p:False)
    assert Controller.obstacle_detour_target(n,'tb2',n.poses) is None
    n=node();n.poses['tb4']=n.poses['tb2']
    assert Controller.obstacle_detour_target(n,'tb2',n.poses) is None


def test_reservations_follow_near_curved_path_and_expire():
    n=node();n.state='BOUNDARY_TRAVELLING';n.primary=(5.,0.,0.);n.poses['tb1']=(0.,0.,0.)
    n.primary_route=[(0.,0.,0.),(0.,1.,math.pi/2),(5.,1.,0.),n.primary];n.primary_route_stamp=time.monotonic()
    corridor=Controller.primary_corridor(n)
    assert any(p[1]>.9 and p[0]>.5 for p in corridor)
    assert not any(p[0]>1.01 for p in corridor)
    n.primary_route_stamp=0.
    assert all(abs(p[1])<1e-6 for p in Controller.primary_corridor(n))


def test_auxiliary_target_in_hilti_route_is_rejected_but_outer_flank_is_clear():
    n=node();n.state='BOUNDARY_TRAVELLING';n.primary=(5.,0.,0.);n.poses['tb1']=(0.,0.,0.)
    n.p['maximum_target_cost']=70
    assert not Controller.feasible(n,'tb2',(1.5,0.,0.),peers=False)
    assert Controller.feasible(n,'tb2',(1.5,2.,0.),peers=False)
    n.primary=n.poses['tb1']
    assert Controller.feasible(n,'tb2',(1.5,0.,0.),peers=False)


def test_route_preview_stops_at_lookahead_and_samples_turns():
    preview=route_preview((0.,0.,0.),[(0.,0.,0.),(0.,10.,math.pi/2)],2.)
    assert preview[-1]==(0.,2.,math.pi/2)
    assert any(p[0]==p[1]==0 and 0<p[2]<math.pi/2 for p in preview)


def test_auxiliaries_cannot_enter_route_but_can_escape_outwards():
    n=node();n.state='BOUNDARY_TRAVELLING';n.primary=(5.,0.,0.);n.poses['tb1']=(0.,0.,0.)
    n.p['collision_horizon']=2.
    corridor=Controller.primary_corridor(n)
    assert not Controller.auxiliary_corridor_safe(n,'tb2',(1.,1.,-math.pi/2),(.6,0.),corridor)
    assert Controller.auxiliary_corridor_safe(n,'tb2',(1.,.5,math.pi/2),(.3,0.),corridor)
    assert not Controller.auxiliary_corridor_safe(n,'tb2',(1.,.5,0.),(.3,0.),corridor)


def test_auxiliary_detour_never_crosses_hilti_corridor_to_reach_opposite_side():
    n=node();n.state='BOUNDARY_TRAVELLING';n.primary=(5.,0.,0.);n.poses['tb1']=(0.,0.,0.)
    n.poses['tb2']=(1.,.9,0.);n.targets['tb2']=(1.,-2.,0.)
    target=Controller.obstacle_detour_target(n,'tb2',n.poses)
    assert target is not None and target[1]>=.9
