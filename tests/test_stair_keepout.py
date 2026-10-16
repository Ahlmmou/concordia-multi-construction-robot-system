from pathlib import Path
from types import SimpleNamespace as NS
import sys,math
import numpy as np
from PIL import Image
import pytest,yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from generate_stair_keepout import build
from generate_aligned_formation_map import build as build_aligned
from fleet_mission_controller import FleetMissionController as Controller
from formation_geometry import ROBOTS,Grid
ROOT=Path(__file__).resolve().parents[1]


def image_cell(metadata,image,x,y):
    ox,oy,_=metadata['origin'];r=metadata['resolution']
    return image[image.shape[0]-1-math.floor((y-oy)/r),math.floor((x-ox)/r)]


def test_stair_interior_and_doorway_are_permanently_occupied_without_changing_outside():
    metadata,source,result,output,image,area=build(ROOT/'config/stair_keepout.yaml')
    assert 24<area<27
    for x,y in [(41.9142866,-.682685),(40.,-3.),(42.,1.),(37.8,-1.2)]:
        assert image_cell(metadata,result,x,y)==0
    assert image_cell(metadata,source,41.9142866,-.682685)>200
    assert image_cell(metadata,result,37.,-1.2)==image_cell(metadata,source,37.,-1.2)>200
    assert np.array_equal(result,np.array(Image.open(image)))
    rows,cols=np.where(result!=source);r=metadata['resolution'];ox,oy,_=metadata['origin']
    xs=ox+(cols+.5)*r;ys=oy+(source.shape[0]-rows-.5)*r
    assert xs.min()>37.6 and xs.max()<43.5 and ys.min()>-4. and ys.max()<1.5
    assert np.all(result[source<90]==source[source<90])


def test_open_enclosure_is_rejected_instead_of_filling_unrelated_floor(tmp_path):
    spec=yaml.safe_load((ROOT/'config/stair_keepout.yaml').read_text())
    spec['source_map']=str(ROOT/'config/EVfloor9_outer_only.yaml')
    spec['doorway']=[[35.,-.67],[35.,-1.8]]
    temporary=tmp_path/'spec.yaml';temporary.write_text(yaml.safe_dump(spec))
    with pytest.raises(ValueError,match='leaks'):build(temporary)


def node_with_protected_map(map_name='EVfloor9_stairs_keepout.yaml'):
    metadata=yaml.safe_load((ROOT/'config'/map_name).read_text())
    pixels=np.array(Image.open(ROOT/'config'/metadata['image']))
    grid=Grid.__new__(Grid);grid.frame='map';grid.origin=tuple(metadata['origin'][:2]);grid.angle=0.;grid.resolution=metadata['resolution']
    grid.cells=np.flipud(np.where(pixels<90,100,np.where(pixels>205,0,-1))).astype(np.int8)
    p=yaml.safe_load((ROOT/'config/fleet_mission.yaml').read_text())['fleet_mission_controller']['ros__parameters']
    import ast
    polygons={r:ast.literal_eval(p['hilti_footprint'] if r=='tb1' else p['auxiliary_footprint']) for r in ROBOTS}
    free=NS(frame='map',footprint_free=lambda *a:True)
    return NS(p=p,polygons=polygons,grids={'/map':grid,**{f'/{r}/global_costmap/costmap':free for r in ROBOTS}},poses={},targets={},in_frame=lambda r,p,f:p)


@pytest.fixture(params=['EVfloor9_stairs_keepout.yaml', 'EVfloor9_formation_aligned.yaml'])
def protected_map(request):
    return node_with_protected_map(request.param)


def test_aligned_map_keeps_reference_walls_and_all_existing_stair_exclusion_cells():
    metadata,aligned,result=build_aligned(ROOT/'config')
    old_metadata,source,protected,*_=build(ROOT/'config/stair_keepout.yaml')
    assert np.all(result[source!=protected]==0)
    assert np.all(result[aligned==0]==0)
    assert np.array_equal(result,np.array(Image.open(ROOT/'config'/metadata['image'])))
    assert metadata==yaml.safe_load((ROOT/'config/EVfloor9_formation_aligned.yaml').read_text())
    assert all(metadata[k]==old_metadata[k] for k in ('origin','resolution','negate','occupied_thresh','free_thresh'))


@pytest.mark.parametrize('robot',ROBOTS)
def test_every_robot_rejects_stair_goal_even_with_completely_clear_lidar_costmaps(robot,protected_map):
    n=protected_map
    assert not Controller.feasible(n,robot,(41.9142866,-.682685,0.),peers=False)
    assert Controller.feasible(n,robot,(35.,-1.2,0.),peers=False)


@pytest.mark.parametrize('robot',ROBOTS)
def test_static_velocity_gate_blocks_entry_through_open_door_including_reverse(robot,protected_map):
    n=protected_map
    # A small footprint fits the doorway geometrically; the mapped hazard still
    # rejects entry. Use both signed velocities to test reverse maneuvers too.
    n.polygons[robot]=[(-.1,-.1),(-.1,.1),(.1,.1),(.1,-.1)]
    assert not Controller.static_motion_safe(n,robot,(37.2,-1.2,0.),(.5,0.))
    assert not Controller.static_motion_safe(n,robot,(37.2,-1.2,math.pi),(-.5,0.))
    assert not Controller.static_motion_safe(n,robot,(37.2,-1.3,0.),(.5,.2))
    assert not Controller.static_motion_safe(n,robot,(37.2,-1.3,math.pi),(-.5,-.2))
    assert Controller.static_motion_safe(n,robot,(36.8,-1.2,math.pi),(.2,0.))
    n.grids.pop('/map')
    assert not Controller.static_motion_safe(n,robot,(36.8,-1.2,0.),(.1,0.))


def test_auxiliary_reverse_overlap_recovery_cannot_enter_stair_exclusion(protected_map):
    n=protected_map;robot='tb4'
    n.polygons[robot]=[(-.1,-.1),(-.1,.1),(.1,.1),(.1,-.1)]
    n.radii={robot:.15};n.obstacle_recoveries={robot:{'mode':'escape'}}
    n.grids[f'/{robot}/global_costmap/costmap']=n.grids['/map']
    n.grids[f'/{robot}/local_costmap/costmap']=NS(frame='map',footprint_free=lambda *a:True)
    assert not Controller.mapped_retreat_safe(n,robot,(37.2,-1.2,math.pi),(-.5,0.))
    assert not Controller.static_motion_safe(n,robot,(37.2,-1.2,math.pi),(-.5,0.))
