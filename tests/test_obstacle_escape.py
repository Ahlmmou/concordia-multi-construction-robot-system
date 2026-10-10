import sys
from pathlib import Path
import numpy as np
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import Grid,inflation_escape
from fleet_mission_controller import FleetMissionController

BOX=[(-.2,-.2),(-.2,.2),(.2,.2),(.2,-.2)]

def grid():
    g=Grid.__new__(Grid);g.frame='map';g.origin=(-2.,-2.);g.angle=0.;g.resolution=.05;g.cells=np.zeros((80,80),dtype=np.int8)
    # A wall beyond the true body, with its inflation intersecting padding.
    g.cells[:,48:]=99;g.cells[:,52:]=100
    return g


def test_buffer_escape_backs_away_without_crossing_wall():
    g=grid();pose=(.12,0.,0.)
    assert not g.footprint_free(pose,BOX,.12,98)
    command=inflation_escape(g,pose,BOX,[(.12,0.),(-.12,0.)])
    assert command==(-.12,0.)


def test_unknown_and_actual_obstacle_under_body_never_allow_escape():
    for value in (-1,100):
        g=grid();g.cells[36:45,39:47]=value
        assert inflation_escape(g,(.12,0.,0.),BOX,[(-.12,0.)]) is None


def test_escape_rejects_new_lethal_obstacle_along_the_route():
    g=grid();g.cells[37:44,31:33]=100
    assert inflation_escape(g,(.12,0.,0.),BOX,[(-.2,0.)]) is None


def test_peer_and_static_map_verifier_can_reject_an_otherwise_good_escape():
    assert inflation_escape(grid(),(.12,0.,0.),BOX,[(-.12,0.)],verify=lambda p:False) is None


def test_recovery_cancels_only_blocked_robot_and_releases_when_clear():
    g=grid();calls=[];slot={'pending':False,'handle':None}
    node=NS(obstacle_recoveries={},obstacle_since={},local_blocked={'tb2'},yielding={},clock_value=0.,
        slots={'tb2':slot},grids={'/tb2/local_costmap/costmap':g},poses={'tb2':(.12,0.,0.)},polygons={'tb2':BOX},
        p={'obstacle_recovery_delay':3.,'footprint_padding':.12,'obstacle_release_padding':.20},in_frame=lambda r,p,f:p,
        cancel=calls.append,get_logger=lambda:NS(info=lambda *a:None),arrived=lambda *a:True)
    FleetMissionController.obstacle_tick(node)
    assert not calls
    node.clock_value=3.1;FleetMissionController.obstacle_tick(node)
    assert calls==['tb2'] and node.obstacle_recoveries['tb2']['mode']=='escape'
    node.poses['tb2']=(-.2,0.,0.);node.local_blocked=set()
    FleetMissionController.obstacle_tick(node)
    assert not node.obstacle_recoveries and slot['target'] is None and slot['cooldown']==0


def test_small_inflation_overlap_can_clear_without_absolute_threshold_deadlock():
    g=grid();pose=(.07,0.,0.)
    assert inflation_escape(g,pose,BOX,[(-.12,0.)])==(-.12,0.)


def test_obstacle_approach_slows_down_instead_of_stopping_when_safe():
    from formation_geometry import local_obstacle_command
    g=grid();pose=(-.3,0.,0.)
    result=local_obstacle_command(g,pose,(.8,0.),BOX,.12)
    assert result is not None and 0.<result[0]<.8
    assert local_obstacle_command(g,(.12,0.,0.),(.8,0.),BOX,.12) is None


def test_safe_command_is_preserved_and_unknown_remains_blocked():
    from formation_geometry import local_obstacle_command
    g=grid();pose=(-.8,0.,0.)
    assert local_obstacle_command(g,pose,(.12,0.),BOX,.12)==(.12,0.)
    g.cells[:]=-1
    assert local_obstacle_command(g,pose,(.12,0.),BOX,.12) is None


def test_reverse_can_follow_buffer_edge_until_it_reaches_clear_space():
    g=grid();g.cells[:]=0;g.cells[48:,:]=99;g.cells[52:,:]=100
    assert inflation_escape(g,(0.,.12,0.),BOX,[(-.12,0.)])==(-.12,0.)
    g.cells[37:48,30:33]=100
    assert inflation_escape(g,(0.,.12,0.),BOX,[(-.2,0.)]) is None
