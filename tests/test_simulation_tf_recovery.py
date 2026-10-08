from types import SimpleNamespace as NS
import sys
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
from fleet_mission_controller import FleetMissionController
from formation_geometry import ROBOTS


def test_static_map_edge_is_restored_after_tf_buffer_clock_clear():
    buf=Buffer()
    fixed=TransformStamped();fixed.header.frame_id='map';fixed.child_frame_id='odom';fixed.transform.rotation.w=1.
    moving=TransformStamped();moving.header.frame_id='odom';moving.child_frame_id='base_footprint';moving.header.stamp.sec=10;moving.transform.rotation.w=1.;moving.transform.translation.x=3.
    buf.set_transform_static(fixed,'test');buf.clear()
    buf.set_transform(moving,'test')
    node=NS(buffers={'tb1':buf},static_transforms={r:{} for r in ROBOTS},odom_frames={'tb1':'base_footprint'},poses={},pose_stamp={})
    node.static_transforms['tb1']['odom']=fixed
    FleetMissionController.refresh_poses(node)
    assert node.poses['tb1']==(3.,0.,0.)
    assert node.pose_stamp['tb1']==10.
