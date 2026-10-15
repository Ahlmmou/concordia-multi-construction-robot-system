import sys,math
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from formation_geometry import ROBOTS,rolling_turn_candidates
from fleet_mission_controller import FleetMissionController


def node(free=lambda p:True):
    poses={r:(i*10.,20.,0.) for i,r in enumerate(ROBOTS)};poses['tb1']=(0.,0.,0.)
    commands={r:(0.,0.) for r in ROBOTS}
    box=[(-.3,-.3),(-.3,.3),(.3,.3),(.3,-.3)]
    grid=NS(frame='map',footprint_free=lambda p,*a:free(p))
    n=NS(p={'rolling_turn_speed':.18,'rolling_turn_rate':.2,'minimum_separation':.4,
        'primary_minimum_separation':.6,'collision_horizon':2.,'footprint_padding':.12},
        polygons={r:box for r in ROBOTS},grids={t:grid for t in ('/map','/tb1/global_costmap/costmap','/tb1/local_costmap/costmap')},
        in_frame=lambda r,p,f:p)
    return n,commands,poses


def test_gentle_arc_prefers_forward_and_preserves_steering_direction():
    assert rolling_turn_candidates((0.,-.5))==[(.18,-.2),(-.18,-.2)]
    n,c,p=node()
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(.18,.2)


def test_reverse_intent_is_preserved_and_normal_driving_is_untouched():
    assert rolling_turn_candidates((-.04,.3))[0]==(-.18,.2)
    assert rolling_turn_candidates((.3,.4))==[]
    assert rolling_turn_candidates((0.,0.))==[]
    n,c,p=node()
    assert FleetMissionController.rolling_turn_command(n,(.3,.4),c,p)==(.3,.4)


def test_blocked_forward_arc_uses_checked_reverse_arc():
    n,c,p=node(free=lambda pose:pose[0]<.1)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(-.18,.2)


def test_no_free_arc_keeps_navigation_command_without_forcing_motion():
    n,c,p=node(free=lambda pose:abs(pose[0])<.1)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(0.,.5)
    n.grids.clear()
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(0.,.5)


def test_primary_buffer_is_enforced_during_rolling_turn():
    n,c,p=node();p['tb4']=(1.25,0.,0.)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(-.18,.2)
    p['tb3']=(-1.25,0.,0.)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(0.,.5)


def test_reverse_arc_can_win_even_when_forward_is_free():
    n,c,p=node();n.primary=(-5.,2.,1.)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(-.18,.2)
    n.primary=(5.,2.,1.)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(.18,.2)


def test_goal_preference_cannot_select_an_unsafe_reverse_arc():
    n,c,p=node(free=lambda pose:pose[0]>-.1);n.primary=(-5.,2.,1.)
    assert FleetMissionController.rolling_turn_command(n,(0.,.5),c,p)==(.18,.2)
