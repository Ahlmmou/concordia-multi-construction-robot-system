"""Run with Blender 3.6 --background --python scripts/restore_elevator_mesh.py.

Restore only the elevator-bank wall segments from the archived dev mesh.
Keep the original Blender project and the existing Gazebo outer mesh intact.
"""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import bmesh
import bpy
from mathutils import Matrix, Vector


ROOT = Path(__file__).resolve().parents[1]
MESHES = ROOT / 'models/floor_plan/meshes'
NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}
# World XY bounds trim adjoining corridor walls to the two elevator banks.
BANKS = {
    'north': ((26.60, 0.25, 35.70, 3.40),
              ['134292', '133254', '132984', '132941', '132083',
               '128653', '128623', '128586', '128558', '128469']),
    'south': ((26.76, -6.09, 35.72, -2.60),
              ['133136', '133052', '128834', '128807', '128776',
               '127944', '127805', '127654']),
}


def floor_transform():
    world = ET.parse(ROOT / 'worlds/ev_9.world').getroot()
    include = next(e for e in world.findall('.//include')
                   if e.findtext('uri') == 'model://floor_plan')
    link = ET.parse(ROOT / 'models/floor_plan/floor_plan.sdf').find('.//link[@name="floor"]')
    def pose(text):
        x,y,z,roll,pitch,yaw = map(float, text.split())
        if abs(roll)+abs(pitch)>1e-9:
            raise ValueError('Expected planar floor placement')
        return Matrix.Translation((x,y,z)) @ Matrix.Rotation(yaw,4,'Z')
    return pose(include.findtext('pose')) @ pose(link.findtext('pose'))


def source_mesh(node, geometries, world_from_floor):
    geometry = geometries[node.find('c:instance_geometry',NS).get('url')[1:]]
    mesh = geometry.find('c:mesh',NS)
    position = mesh.find('c:vertices/c:input[@semantic="POSITION"]',NS).get('source')[1:]
    values = list(map(float,mesh.find(f'c:source[@id="{position}"]/c:float_array',NS).text.split()))
    matrix_values = list(map(float,node.find('c:matrix',NS).text.split()))
    transform = world_from_floor @ Matrix([matrix_values[i:i+4] for i in range(0,16,4)])
    vertices = [transform @ Vector(values[i:i+3]) for i in range(0,len(values),3)]
    faces = []
    for triangles in mesh.findall('c:triangles',NS):
        inputs = triangles.findall('c:input',NS)
        stride = max(int(e.get('offset')) for e in inputs)+1
        offset = int(next(e for e in inputs if e.get('semantic')=='VERTEX').get('offset'))
        indices = list(map(int,triangles.findtext('c:p',namespaces=NS).split()))[offset::stride]
        faces.extend(tuple(indices[i:i+3]) for i in range(0,len(indices),3))
    return vertices,faces


def main():
    source = MESHES / 'EVfloor9_L.dae'
    digest = hashlib.sha256(source.read_bytes()).hexdigest()
    if digest != '43fd9b2503fbe6ef447d996998496b8bf02ac0f4bf01e9c48f47917555ab40be':
        raise ValueError('Source mesh differs from the verified GitHub dev revision')
    bpy.ops.wm.open_mainfile(filepath=str(MESHES/'EVfloor9_outer_only.blend'))
    original = {obj.name: (obj.matrix_world.copy(), obj.data) for obj in bpy.context.scene.objects}
    world_from_floor = floor_transform()
    floor_from_world = world_from_floor.inverted()
    root = ET.parse(source).getroot()
    geometries = {g.get('id'):g for g in root.findall('.//c:geometry',NS)}
    nodes = root.findall('.//c:library_visual_scenes/c:visual_scene/c:node',NS)
    collection = bpy.data.collections.new('Elevator walls restored from GitHub dev')
    bpy.context.scene.collection.children.link(collection)
    material = bpy.data.materials.new('Elevator wall concrete')
    material.diffuse_color = (.6,.6,.6,1)
    audit = []
    for bank,(bounds,ids) in BANKS.items():
        xmin,ymin,xmax,ymax = bounds
        for wall_id in ids:
            node = next(n for n in nodes if f'___{wall_id}_' in n.get('id',''))
            vertices,faces = source_mesh(node,geometries,world_from_floor)
            mesh = bpy.data.meshes.new(f'Elevator_{bank}_{wall_id}')
            mesh.from_pydata(vertices,[],faces)
            bm = bmesh.new();bm.from_mesh(mesh)
            bmesh.ops.remove_doubles(bm,verts=list(bm.verts),dist=1e-6)
            # The south bank's left edge continues into a small service shaft.
            top = -2.854 if wall_id=='127654' else ymax
            planes = [((xmin,0,0),(-1,0,0)),((xmax,0,0),(1,0,0)),
                      ((0,ymin,0),(0,-1,0)),((0,top,0),(0,1,0))]
            for point,normal in planes:
                bmesh.ops.bisect_plane(bm,geom=list(bm.verts)+list(bm.edges)+list(bm.faces),
                    dist=1e-7,plane_co=point,plane_no=normal,clear_outer=True)
            bmesh.ops.holes_fill(bm,edges=[e for e in bm.edges if e.is_boundary],sides=0)
            bmesh.ops.recalc_face_normals(bm,faces=list(bm.faces))
            if not bm.faces:
                raise ValueError(f'Empty restored wall {wall_id}')
            audit.append({'bank':bank,'source_wall':wall_id,'vertices':len(bm.verts),
                          'faces':len(bm.faces),'closed':all(e.is_manifold for e in bm.edges),
                          'world_bounds':[[min(v.co[i] for v in bm.verts) for i in range(3)],
                                          [max(v.co[i] for v in bm.verts) for i in range(3)]]})
            for vertex in bm.verts:
                vertex.co = floor_from_world @ vertex.co
            bm.to_mesh(mesh);bm.free();mesh.update()
            obj = bpy.data.objects.new(mesh.name,mesh);collection.objects.link(obj)
            obj.data.materials.append(material)
    for name,(matrix,data) in original.items():
        assert bpy.data.objects[name].matrix_world==matrix and bpy.data.objects[name].data==data
    bpy.ops.object.select_all(action='DESELECT')
    for obj in collection.objects:obj.select_set(True)
    bpy.context.view_layer.objects.active = next(iter(collection.objects))
    bpy.context.scene['elevator_source_commit'] = 'a8a587547c581289ee31312957f66c8b1bdf311e'
    bpy.context.scene['elevator_source_sha256'] = digest
    bpy.ops.wm.save_as_mainfile(filepath=str(MESHES/'EVfloor9_with_elevators.blend'))
    bpy.ops.wm.collada_export(filepath=str(MESHES/'EVfloor9_elevators.dae'),
        selected=True,apply_modifiers=True,triangulate=True,use_object_instantiation=False)
    (ROOT/'docs/validation/gimp_alignment/elevator_mesh_audit.json').write_text(
        json.dumps({'original_objects_preserved':len(original),'walls':audit},indent=2)+'\n')
    print('RESTORED ELEVATORS',len(audit),'closed:',all(w['closed'] for w in audit))


if __name__=='__main__':
    main()
