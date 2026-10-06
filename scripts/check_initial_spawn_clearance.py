#!/usr/bin/env python3
"""Read-only spawn clearance check for the EV9 world (no ROS commands)."""
import argparse
import xml.etree.ElementTree as E
import numpy as np, yaml, json, math
from PIL import Image
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from pathlib import Path
R=Path(__file__).resolve().parents[1]
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--margin',type=float,help='Override configured clearance margin in metres')
parser.add_argument('--poses-json',help='Gazebo model state JSON from the live read-only audit')
parser.add_argument('--output',default='/tmp/step1_clearance.json')
args=parser.parse_args()
if args.margin is not None and args.margin < 0:parser.error('--margin must be nonnegative')
r=E.parse(R/'models/floor_plan/meshes/EVfloor9_outer_only.dae').getroot(); ns={'c':r.tag.split('}')[0][1:]}; geometries={}
for g in r.findall('c:library_geometries/c:geometry',ns):
 mesh=g.find('c:mesh',ns); sources={s.attrib['id']:np.fromstring(s.find('c:float_array',ns).text,sep=' ').reshape(-1,int(s.find('c:technique_common/c:accessor',ns).attrib['stride'])) for s in mesh.findall('c:source',ns)}
 vertices={v.attrib['id']:v.find('c:input',ns).attrib['source'][1:] for v in mesh.findall('c:vertices',ns)}; tris=[]
 for tag in ['triangles','polylist']:
  for p in mesh.findall('c:'+tag,ns):
   ins=p.findall('c:input',ns);stride=max(int(i.attrib.get('offset',0)) for i in ins)+1; vin=next(i for i in ins if i.attrib['semantic']=='VERTEX');idx=np.fromstring(p.find('c:p',ns).text,sep=' ',dtype=int).reshape(-1,stride)[:,int(vin.attrib.get('offset',0))]; pts=sources[vertices[vin.attrib['source'][1:]]]
   counts=[3]*(len(idx)//3) if tag=='triangles' else np.fromstring(p.find('c:vcount',ns).text,sep=' ',dtype=int);k=0
   for count in counts:
    for j in range(1,count-1):tris.append(pts[idx[[k,k+j,k+j+1]]])
    k+=count
 geometries[g.attrib['id']]=np.array(tris)
tri=[];names=[]
def sdf_pose(text):
 values=np.fromstring(text,sep=' '); transform=np.eye(4)
 transform[:3,:3]=Rotation.from_euler('xyz',values[3:]).as_matrix()
 transform[:3,3]=values[:3]; return transform
world=E.parse(R/'worlds/ev_9.world').getroot().find('world')
floor_include=next(e for e in world.findall('include') if e.findtext('uri')=='model://floor_plan')
floor=E.parse(R/'models/floor_plan/floor_plan.sdf').getroot().find('model/link')
assert floor.findtext('collision/geometry/mesh/uri')=='model://floor_plan/meshes/EVfloor9_outer_only.dae'
W=sdf_pose(floor_include.findtext('pose','0 0 0 0 0 0'))@sdf_pose(floor.findtext('pose','0 0 0 0 0 0'))
for node in r.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):
 inst=node.find('c:instance_geometry',ns)
 if inst is None:continue
 M=np.fromstring(node.find('c:matrix',ns).text,sep=' ').reshape(4,4);v=geometries[inst.attrib['url'][1:]];h=np.concatenate([v,np.ones((*v.shape[:-1],1))],axis=-1);v=(h@(W@M).T)[...,:3];v=v[(v[:,:,2].max(axis=1)>.05)&(v[:,:,2].min(axis=1)<3.3)]
 tri.extend(v[:,:,:2]);names.extend([node.attrib['name']]*len(v))
tri=np.array(tri);A=tri;B=np.roll(A,-1,axis=1);D=B-A;L=(D*D).sum(axis=2)
def geomdist(p):
 t=np.clip(((p-A)*D).sum(axis=2)/np.maximum(L,1e-20),0,1);ds=np.linalg.norm(p-(A+t[:,:,None]*D),axis=2).min(axis=1)
 cross=D[:,:,0]*(p-A)[:,:,1]-D[:,:,1]*(p-A)[:,:,0];area=np.abs(np.cross(A[:,1]-A[:,0],A[:,2]-A[:,0]));inside=((cross>=-1e-10).all(axis=1)|(cross<=1e-10).all(axis=1))&(area>1e-10);ds[inside]=0;i=ds.argmin();return float(ds[i]),names[i]
spec=yaml.safe_load((R/'config/EVfloor9_outer_only.yaml').read_text());im=np.array(Image.open(R/'config/EVfloor9_outer_only.pgm'))[::-1];prob=im/255 if spec['negate'] else (255-im)/255;occ=prob>=spec['free_thresh'];iy,ix=np.where(occ);assert spec['origin'][2]==0.0;res=spec['resolution'];origin=np.array(spec['origin'][:2]);pts=origin+(np.array([ix,iy]).T+.5)*res;tree=cKDTree(pts)
def mapdist(p):return float(tree.query(p)[0]-res/2**.5)
poses=np.array([[.7,1.1],[4.9,4.5],[-3.5,-2.3],[4.9,-2.3],[-3.5,4.5]])
import ast
radii=[]
for name in ['tb1','tb2','tb3','tb4','tb5']:
 param=yaml.safe_load((R/('params/nav2_params_hilti.yaml' if name=='tb1' else 'params/nav2_params.yaml')).read_text())
 footprint=ast.literal_eval(param['global_costmap']['global_costmap']['ros__parameters']['footprint'])
 radii.append(max(math.hypot(*p) for p in footprint))
radii=np.array(radii)
def check(xy):return [{'name':f'tb{i+1}','xy':p.tolist(),'geometry_centre_distance':geomdist(p)[0],'nearest_wall':geomdist(p)[1],'map_centre_distance':mapdist(p),'rotation_radius':float(radii[i]),'geometry_turn_clearance':geomdist(p)[0]-radii[i],'map_turn_clearance':mapdist(p)-radii[i]} for i,p in enumerate(xy)]

cfg=yaml.safe_load((R/'config/fleet_spawn.yaml').read_text());new=np.array([[r['x_pose'],r['y_pose']] for r in cfg['robots']]);required=(cfg['clearance_margin'] if args.margin is None else args.margin)+cfg['departure_distance']+cfg['footprint_padding']
if args.poses_json:
 states=json.loads(Path(args.poses_json).read_text())
 new=np.array([states[r['name']]['pose']['position'][:2] for r in cfg['robots']])
checks=check(new)
for item in checks:item['passes']=bool(min(item['geometry_turn_clearance'],item['map_turn_clearance'])>=required)
pairs=[{'robots':[f'tb{i+1}',f'tb{j+1}'],'turn_envelope_gap':float(np.linalg.norm(new[i]-new[j])-radii[i]-radii[j]),'passes':bool(np.linalg.norm(new[i]-new[j])-radii[i]-radii[j]>=2*required)} for i in range(5) for j in range(i+1,5)]
report={'original':check(poses),'translation':[-.2,.5],'required_clearance':required,'revised':checks,'pairs':pairs,'all_pass':all(x['passes'] for x in checks+pairs)}
print(json.dumps(report,indent=2));Path(args.output).write_text(json.dumps(report,indent=2))
raise SystemExit(0 if report['all_pass'] else 1)
