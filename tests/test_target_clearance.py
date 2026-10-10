import sys
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController
from formation_geometry import ROBOTS,rolling_turn_candidates


def node():
    box=[[-.3,-.3],[-.3,.3],[.475,.3],[.475,-.3]]
    polygons={r:box for r in ROBOTS};polygons['tb1']=[[-1.2,-.85],[-1.2,.85],[1.4,.85],[1.4,-.85]]
    g=NS(frame='map',footprint_free=lambda *a:True)
    return NS(polygons=polygons,poses={'tb1':(0.,0.,0.),'tb3':(0.,3.,0.)},
        grids={'/map':g,'/tb1/global_costmap/costmap':g},in_frame=lambda r,p,f:p,
        p={'footprint_padding':.12,'maximum_target_cost':70,'minimum_separation':.4,'primary_minimum_separation':.6})


def test_reserved_position_uses_oriented_body_gap_instead_of_large_circle():
    n=node()
    target=(31.325109481811523,-6.217083930969238,2.573962858178192)
    reserved={'tb3':(31.20722635380427,-3.5689532583861165,-1.2824562656295462)}
    n.poses={}
    assert FleetMissionController.feasible(n,'tb1',target,reserved)


def test_real_gap_violations_are_still_rejected():
    n=node();n.poses['tb3']=(0.,1.6,0.)
    assert not FleetMissionController.feasible(n,'tb1',(0.,0.,0.))
    n.poses={}
    assert not FleetMissionController.feasible(n,'tb1',(0.,0.,0.),{'tb3':(0.,1.6,0.)})


def test_slow_turning_command_can_become_a_wide_arc():
    assert rolling_turn_candidates((-.094,.5),speed=.22,turn_rate=.15)[0]==(-.22,.15)
    assert rolling_turn_candidates((0.,-.5),speed=.22,turn_rate=.15)[0]==(.22,-.15)
