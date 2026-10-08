import sys,json
from pathlib import Path
from types import SimpleNamespace as NS
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from fleet_live_status import FleetStatus
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray,Marker
from rcl_interfaces.msg import Log


def panel():
    return NS(state={'mission':4},requested_goal=None,request_number=0,request_mission=0,
              goal_confirmed=False,goal_rejected=None,goal=(1.,2.),notice=None)


def submit(node):
    msg=PoseStamped();msg.header.frame_id='map';msg.pose.position.x=8.;msg.pose.position.y=3.
    FleetStatus.goal_requested(node,msg)


def test_new_request_is_visible_before_phase_or_marker_changes():
    node=panel();submit(node)
    assert node.requested_goal==(8.,3.) and node.request_number==1
    assert node.goal==(1.,2.) and not node.goal_confirmed


def test_repeated_goal_in_same_phase_is_still_a_new_submission():
    node=panel();submit(node);submit(node)
    assert node.request_number==2


def test_mission_confirmation_updates_distance_without_waiting_for_marker():
    node=panel();submit(node)
    FleetStatus.status(node,String(data=json.dumps({'mission':5,'state':'FORMING_BOUNDARY'})))
    assert node.goal_confirmed and node.goal==(8.,3.)
    old=Marker();old.ns='fleet_mission';old.id=1;old.pose.position.x=1.;old.pose.position.y=2.
    FleetStatus.markers(node,MarkerArray(markers=[old]))
    assert node.goal==(8.,3.)


def test_rejected_submission_keeps_previous_active_goal():
    node=panel();submit(node)
    FleetStatus.log(node,Log(name='fleet_mission_controller',level=40,msg='Primary goal is unknown or occupied'))
    assert node.goal_rejected and not node.goal_confirmed and node.goal==(1.,2.)
    submit(node)
    assert node.goal_rejected is None


def test_normal_controller_log_does_not_freeze_live_updates():
    node=panel()
    FleetStatus.log(node,Log(name='fleet_mission_controller',level=20,msg='FORMING_BOUNDARY: new goal'))
    assert node.notice is None
