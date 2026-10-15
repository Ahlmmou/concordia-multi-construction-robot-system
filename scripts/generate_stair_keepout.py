#!/usr/bin/env python3
"""Fill the enclosed stairwell in a derived map; preserve the source map."""
import argparse
from pathlib import Path
import math
import numpy as np
from PIL import Image
from scipy.ndimage import label
import yaml


def build(spec_path):
    spec_path=Path(spec_path);spec=yaml.safe_load(spec_path.read_text())
    root=spec_path.parent
    source_yaml=root/spec['source_map']
    metadata=yaml.safe_load(source_yaml.read_text())
    source=np.array(Image.open(source_yaml.parent/metadata['image']).convert('L'))
    result=source.copy();height,width=source.shape
    resolution=metadata['resolution'];ox,oy,yaw=metadata['origin']
    if abs(yaw)>1e-9 or metadata['negate']!=0:raise ValueError('Expected unrotated, non-negated source map')
    xmin,ymin,xmax,ymax=spec['search_bounds']
    c0=max(0,math.floor((xmin-ox)/resolution));c1=min(width,math.ceil((xmax-ox)/resolution))
    r0=max(0,height-math.ceil((ymax-oy)/resolution));r1=min(height,height-math.floor((ymin-oy)/resolution))
    xx,yy=np.meshgrid(ox+(np.arange(c0,c1)+.5)*resolution,oy+(height-np.arange(r0,r1)-.5)*resolution)
    a,b=np.array(spec['doorway'],float);delta=b-a;length=np.linalg.norm(delta)
    if length<=0:raise ValueError('Doorway endpoints must differ')
    along=((xx-a[0])*delta[0]+(yy-a[1])*delta[1])/length
    perpendicular=abs((xx-a[0])*delta[1]-(yy-a[1])*delta[0])/length
    allowance=resolution*math.sqrt(2)/2
    door=(along>=-spec['door_extension']-allowance)&(along<=length+spec['door_extension']+allowance)&(perpendicular<=spec['door_thickness']/2+allowance)
    occupied=(1-source[r0:r1,c0:c1]/255.)>metadata['occupied_thresh']
    regions,_=label(~(occupied|door))
    x,y=spec['interior_reference'];row=height-1-math.floor((y-oy)/resolution)-r0;col=math.floor((x-ox)/resolution)-c0
    if not (0<=row<regions.shape[0] and 0<=col<regions.shape[1]) or regions[row,col]==0:raise ValueError('Interior reference is not a free cell in the search area')
    interior=regions==regions[row,col]
    area=float(interior.sum())*resolution**2
    if any(np.any(edge) for edge in (interior[0],interior[-1],interior[:,0],interior[:,-1])):raise ValueError('Enclosure leaks; refusing to fill outside the stairwell')
    if not 1.<area<spec['maximum_area']:raise ValueError('Unexpected stairwell area: '+str(area))
    result[r0:r1,c0:c1][interior|door]=0
    output=root/spec['output_map'];image=output.with_suffix('.png')
    return metadata,source,result,output,image,area


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--spec',default=str(Path(__file__).resolve().parents[1]/'config/stair_keepout.yaml'));args=parser.parse_args()
    metadata,source,result,output,image,area=build(args.spec)
    Image.fromarray(result).save(image)
    metadata['image']=image.name;output.write_text(yaml.safe_dump(metadata,sort_keys=False))
    print(f'Protected stairwell: {area:.2f} m²; {np.count_nonzero(source!=result)} cells changed; {output}')

if __name__=='__main__':main()
