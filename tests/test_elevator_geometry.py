from collections import Counter
from pathlib import Path
import math
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image, ImageDraw
import pytest
from scipy.ndimage import distance_transform_edt
import yaml


ROOT = Path(__file__).resolve().parents[1]
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def pose(text):
    x,y,z,roll,pitch,yaw = map(float,text.split())
    assert roll==pitch==0
    c,s=math.cos(yaw),math.sin(yaw)
    return np.array([[c,-s,0,x],[s,c,0,y],[0,0,1,z],[0,0,0,1]])


@pytest.fixture(scope='module')
def exported_walls():
    model=ET.parse(ROOT/'models/floor_plan/floor_plan.sdf')
    link=model.find('.//link[@name="elevator_walls"]')
    world=ET.parse(ROOT/'worlds/ev_9.world')
    include=next(e for e in world.findall('.//include') if e.findtext('uri')=='model://floor_plan')
    transform=pose(include.findtext('pose'))@pose(link.findtext('pose'))
    dae=ET.parse(ROOT/'models/floor_plan/meshes/EVfloor9_elevators.dae')
    geometries={g.get('id'):g for g in dae.findall('.//c:geometry',NS)}
    walls=[]
    for node in dae.findall('.//c:library_visual_scenes/c:visual_scene/c:node',NS):
        instance=node.find('c:instance_geometry',NS)
        if instance is None:continue
        mesh=geometries[instance.get('url')[1:]].find('c:mesh',NS)
        source=mesh.find('c:vertices/c:input[@semantic="POSITION"]',NS).get('source')[1:]
        vertices=np.fromstring(mesh.find(f'c:source[@id="{source}"]/c:float_array',NS).text,sep=' ').reshape(-1,3)
        matrix=np.fromstring(node.findtext('c:matrix',namespaces=NS),sep=' ').reshape(4,4)
        vertices=(transform@matrix@np.c_[vertices,np.ones(len(vertices))].T).T[:,:3]
        indices=[]
        for triangles in mesh.findall('c:triangles',NS):
            inputs=triangles.findall('c:input',NS)
            stride=max(int(i.get('offset')) for i in inputs)+1
            offset=int(next(i for i in inputs if i.get('semantic')=='VERTEX').get('offset'))
            indices.extend(np.fromstring(triangles.findtext('c:p',namespaces=NS),sep=' ',dtype=int)[offset::stride].reshape(-1,3))
        walls.append((node.get('name'),vertices,np.array(indices)))
    return walls


def test_visual_and_collision_share_export_and_preserve_outer_geometry():
    model=ET.parse(ROOT/'models/floor_plan/floor_plan.sdf')
    floor=model.find('.//link[@name="floor"]')
    elevators=model.find('.//link[@name="elevator_walls"]')
    assert elevators.findtext('pose')==floor.findtext('pose')
    for kind in ('visual','collision'):
        assert floor.findtext(f'{kind}/geometry/mesh/uri')=='model://floor_plan/meshes/EVfloor9_outer_only.dae'
        assert elevators.findtext(f'{kind}/geometry/mesh/uri')=='model://floor_plan/meshes/EVfloor9_elevators.dae'


def test_eighteen_exported_wall_segments_are_closed_and_confined_to_banks(exported_walls):
    assert len(exported_walls)==18
    for name,vertices,triangles in exported_walls:
        lo,hi=vertices.min(0),vertices.max(0)
        assert lo[2]==pytest.approx(0,abs=1e-5)
        assert hi[2]==pytest.approx(3.67,abs=1e-4)
        assert 26.59<lo[0]<hi[0]<35.73
        if 'north' in name:assert .24<lo[1]<hi[1]<3.41
        else:assert -6.10<lo[1]<hi[1]<-2.59
        edges=Counter()
        for triangle in triangles:
            points=[tuple(np.round(v,5)) for v in vertices[triangle]]
            for i,j in ((0,1),(1,2),(2,0)):
                edges[tuple(sorted((points[i],points[j])))]+=1
        assert all(count==2 for count in edges.values())


def test_physical_wall_projection_tracks_map_and_does_not_close_passage(exported_walls):
    metadata=yaml.safe_load((ROOT/'config/EVfloor9_formation_aligned.yaml').read_text())
    layer=np.array(Image.open(ROOT/'config/EVfloor9_elevator_walls.png').convert('RGBA'))
    height,width=layer.shape[:2];res=metadata['resolution'];ox,oy,_=metadata['origin']
    image=Image.new('L',(width,height));draw=ImageDraw.Draw(image)
    for _,vertices,triangles in exported_walls:
        for triangle in vertices[triangles]:
            xy=np.c_[(triangle[:,0]-ox)/res,height-(triangle[:,1]-oy)/res]
            draw.polygon([tuple(p) for p in xy.round().astype(int)],fill=255)
    projection=np.array(image)>0
    mapped=(layer[:,:,3]>=128)&(layer[:,:,:3].max(2)<90)
    distances=distance_transform_edt(~mapped)*res
    # Original detailed-plan/Gazebo differences fit within three map cells
    # for 95% of the physical projection. This does not alter robot clearance.
    assert np.quantile(distances[projection],.95)<3*res
    assert not np.any(projection[1660:1750,2500:2800])
