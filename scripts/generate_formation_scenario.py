#!/usr/bin/env python3
"""Generate an EV9-preserving world/map pair with measured test walls.

Map rasterization and SDF collisions share the same box specifications.
The source EV9 assets are read only. Run before building the package.
"""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from PIL import Image
import yaml

ROOT=Path(__file__).resolve().parents[1]
# name, centre x/y, length along x/y. Interior passage width is 6 m.
WALLS=[('formation_corridor_south',25.5,-1.5,19.,.2),
       ('formation_corridor_north',22.5,4.7,13.,.2),
       ('formation_corridor_east',35.1,9.7,.2,22.4),
       ('formation_corridor_west',28.9,11.3,.2,13.4),
       ('formation_workspace_wall',2.,12.,20.,.2)]


def main():
    spec=yaml.safe_load((ROOT/'config/EVfloor9_outer_only.yaml').read_text())
    cells=np.array(Image.open(ROOT/'config'/spec['image']).convert('L'))
    res=spec['resolution'];ox,oy,_=spec['origin'];h,w=cells.shape
    tree=ET.parse(ROOT/'worlds/ev_9.world');world=tree.getroot().find('world')
    for name,x,y,lx,ly in WALLS:
        model=ET.SubElement(world,'model',name=name);ET.SubElement(model,'static').text='true'
        ET.SubElement(model,'pose').text=f'{x} {y} 1 0 0 0'
        link=ET.SubElement(model,'link',name='wall')
        for kind in ('collision','visual'):
            item=ET.SubElement(link,kind,name='box');geom=ET.SubElement(item,'geometry')
            box=ET.SubElement(geom,'box');ET.SubElement(box,'size').text=f'{lx} {ly} 2'
        # Occupy every map cell intersecting the physical box.
        ix0=max(0,int(np.floor((x-lx/2-ox)/res)));ix1=min(w,int(np.ceil((x+lx/2-ox)/res)))
        iy0=max(0,int(np.floor((y-ly/2-oy)/res)));iy1=min(h,int(np.ceil((y+ly/2-oy)/res)))
        cells[h-iy1:h-iy0,ix0:ix1]=0
    ET.indent(tree);tree.write(ROOT/'worlds/ev9_formation_test.world',encoding='unicode')
    Image.fromarray(cells).save(ROOT/'config/ev9_formation_test.png')
    spec['image']='./ev9_formation_test.png'
    (ROOT/'config/ev9_formation_test.yaml').write_text(yaml.safe_dump(spec,sort_keys=False))
    (ROOT/'config/formation_test_walls.yaml').write_text(yaml.safe_dump({'walls':[dict(name=n,x=x,y=y,length_x=lx,length_y=ly) for n,x,y,lx,ly in WALLS]},sort_keys=False))
    print('Generated world/map with identical test-wall coordinates; EV9 source assets preserved.')


if __name__=='__main__':main()
