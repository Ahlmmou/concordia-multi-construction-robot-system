import math
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import pytest
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_mission_controller import FleetMissionController
ROOT=Path(__file__).resolve().parents[1]
PARAMS=yaml.safe_load((ROOT/'config/fleet_mission.yaml').read_text())['fleet_mission_controller']['ros__parameters']

@pytest.mark.parametrize('robot',['tb1','tb2'])
def test_arrival_requires_both_precise_position_and_heading(robot):
    limit=PARAMS['primary_yaw_tolerance'] if robot=='tb1' else PARAMS['yaw_tolerance']
    node=NS(p=PARAMS,poses={robot:(.14,0.,limit-.001)})
    assert FleetMissionController.arrived(node,robot,(0.,0.,0.))
    node.poses[robot]=(.16,0.,0.)
    assert not FleetMissionController.arrived(node,robot,(0.,0.,0.))
    node.poses[robot]=(0.,0.,limit+.001)
    assert not FleetMissionController.arrived(node,robot,(0.,0.,0.))
    node.poses[robot]=(0.,0.,math.pi-.01)
    assert FleetMissionController.arrived(node,robot,(0.,0.,-math.pi+.01))

@pytest.mark.parametrize('filename',['nav2_params.yaml','nav2_params_hilti.yaml'])
def test_nav2_finishes_inside_supervisor_tolerances(filename):
    params=yaml.safe_load((ROOT/'params'/filename).read_text())['controller_server']['ros__parameters']
    checker=params['goal_checker']
    assert params['FollowPath']['xy_goal_tolerance']==checker['xy_goal_tolerance']<PARAMS['arrival_tolerance']
    assert checker['yaw_goal_tolerance']<min(PARAMS['yaw_tolerance'],PARAMS['primary_yaw_tolerance'])
    assert checker['stateful'] is False
