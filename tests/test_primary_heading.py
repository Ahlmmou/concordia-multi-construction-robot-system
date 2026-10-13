import math,sys
from pathlib import Path
from types import SimpleNamespace as NS
import pytest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController


def node(heading):
    return NS(poses={'tb1':(0.,0.,heading),'tb2':(0.,0.,heading)},
        p={'arrival_tolerance':.4,'yaw_tolerance':.45,'primary_yaw_tolerance':.10})


@pytest.mark.parametrize('angle,arrived',[(0.,True),(.09,True),(-.09,True),(.11,False),(-.11,False),(.25,False)])
def test_primary_does_not_finish_until_arrow_heading_is_met(angle,arrived):
    assert FleetMissionController.arrived(node(angle),'tb1',(0.,0.,0.)) is arrived


def test_auxiliary_heading_tolerance_is_preserved():
    assert FleetMissionController.arrived(node(.25),'tb2',(0.,0.,0.))


def test_heading_difference_wraps_across_pi():
    assert FleetMissionController.arrived(node(math.pi-.04),'tb1',(0.,0.,-math.pi+.04))
    assert not FleetMissionController.arrived(node(math.pi-.08),'tb1',(0.,0.,-math.pi+.08))


def test_matching_heading_does_not_bypass_position_check():
    assert not FleetMissionController.arrived(node(0.),'tb1',(1.,0.,0.))
