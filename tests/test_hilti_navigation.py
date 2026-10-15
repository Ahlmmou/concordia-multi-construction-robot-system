from pathlib import Path
from types import SimpleNamespace as NS
import sys
import xml.etree.ElementTree as ET
import pytest
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'scripts'))
from fleet_mission_controller import FleetMissionController

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('mode', ['LEGACY', 'SINGLE_FILE', 'COMPACT_RECTANGLE'])
def test_primary_uses_recovery_without_spin_in_every_mode(monkeypatch, mode):
    import ament_index_python.packages
    monkeypatch.setattr(ament_index_python.packages, 'get_package_share_directory', lambda _: str(ROOT))
    sent = []
    future = NS(add_done_callback=lambda callback: None)
    node = NS(slots={'tb1': {}}, requested_mode=mode, commands={'tb1': object()},
              pose_message=lambda target: __import__('geometry_msgs.msg', fromlist=['PoseStamped']).PoseStamped(),
              get_clock=lambda: NS(now=lambda: NS(nanoseconds=0)),
              nav={'tb1': NS(send_goal_async=lambda goal: sent.append(goal) or future)},
              nav_accepted=lambda *args, **kwargs: None)
    FleetMissionController.dispatch_navigation(node, 'tb1', (1., 0., 0.))
    tree = ET.parse(sent[0].behavior_tree)
    assert tree.find('.//Spin') is None
    assert tree.find('.//BackUp') is None
    assert tree.find('.//Wait') is not None
    assert tree.find('.//ClearEntireCostmap') is not None
    assert 'tb1' not in node.commands
