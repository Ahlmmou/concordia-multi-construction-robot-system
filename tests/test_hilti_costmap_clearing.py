"""Keep free-space clearing consistent with the simulated Hilti laser."""
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]

@pytest.mark.parametrize('scope,layer', [('global', 'obstacle'), ('local', 'voxel')])
def test_hilti_clears_full_sensor_range_without_expanding_obstacle_marking(scope, layer):
    sensor = ET.parse(ROOT / 'models/Hilti_4/Hilti_4.sdf').find('.//sensor[@name="front_laser"]')
    sensor_range = float(sensor.findtext('ray/range/max'))
    params = yaml.safe_load((ROOT / 'params/nav2_params_hilti.yaml').read_text())
    costmap = params[f'{scope}_costmap'][f'{scope}_costmap']['ros__parameters']
    scan = costmap[f'{layer}_layer']['scan']
    assert scan['clearing'] and scan['marking'] and scan['inf_is_valid']
    assert scan['raytrace_max_range'] == sensor_range
    assert 0 <= scan['raytrace_min_range'] <= scan['obstacle_min_range']
    assert scan['obstacle_max_range'] == 2.5 < sensor_range
    assert costmap['inflation_layer']['inflation_radius'] == 1.8
