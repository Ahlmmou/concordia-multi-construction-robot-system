from pathlib import Path
import numpy as np
from PIL import Image
import yaml
import xml.etree.ElementTree as ET

ROOT=Path(__file__).resolve().parents[1]


def test_generated_world_and_map_share_box_coordinates():
    spec=yaml.safe_load((ROOT/'config/ev9_formation_test.yaml').read_text())
    walls=yaml.safe_load((ROOT/'config/formation_test_walls.yaml').read_text())['walls']
    image=np.asarray(Image.open(ROOT/'config/ev9_formation_test.png'))[::-1]
    models={m.attrib['name']:m for m in ET.parse(ROOT/'worlds/ev9_formation_test.world').getroot().find('world').findall('model')}
    res=spec['resolution'];ox,oy,_=spec['origin']
    for wall in walls:
        model=models[wall['name']]
        assert model.find('static').text=='true'
        assert list(map(float,model.find('pose').text.split()))[:2]==[wall['x'],wall['y']]
        assert list(map(float,model.find('link/collision/geometry/box/size').text.split()))[:2]==[wall['length_x'],wall['length_y']]
        ix=int((wall['x']-ox)/res);iy=int((wall['y']-oy)/res)
        assert image[iy,ix]==0
    # A safe route centre in the straight and bent portions remains map free.
    for x,y in ((16.,1.6),(25.,1.6),(32.,1.6),(32.,8.),(32.,15.),(4.,8.)):
        assert image[int((y-oy)/res),int((x-ox)/res)]==255


def test_scenario_preserves_original_ev9_occupied_map_cells():
    original=np.asarray(Image.open(ROOT/'config/EVfloor9_outer_only.pgm'))
    scenario=np.asarray(Image.open(ROOT/'config/ev9_formation_test.png'))
    assert np.all(scenario[original==0]==0)
