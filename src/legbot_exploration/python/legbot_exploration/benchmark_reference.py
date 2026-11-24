"""Independent support-area reference from static SDF collision geometry.

The reference is a point-robot grid, not a footprint safety certificate. It is
never supplied to the exploration planner. Unsupported geometry fails explicitly.
"""
from collections import defaultdict, deque
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

REFERENCE_VERSION=2

def pose_matrix(text):
    x,y,z,r,p,yaw=map(float,(text or '0 0 0 0 0 0').split())
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(yaw),math.sin(yaw)
    m=np.eye(4);m[:3,3]=(x,y,z)
    m[:3,:3]=((cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr),
               (sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr),(-sp,cp*sr,cp*cr))
    return m


def dae_triangles(path,transform):
    root=ET.parse(path).getroot();ns={'c':root.tag.split('}')[0][1:]};geometries={}
    if root.findtext('c:asset/c:up_axis',namespaces=ns)!='Z_UP':
        raise ValueError('reference builder currently requires COLLADA Z_UP')
    if abs(float(root.find('c:asset/c:unit',ns).get('meter','1'))-1)>1e-9:
        raise ValueError('reference builder requires metre-scaled COLLADA')
    for item in root.findall('c:library_geometries/c:geometry',ns):
        mesh=item.find('c:mesh',ns);sources={}
        for source in mesh.findall('c:source',ns):
            stride=int(source.find('c:technique_common/c:accessor',ns).get('stride'))
            sources[source.get('id')]=np.fromstring(source.findtext('c:float_array',namespaces=ns),sep=' ').reshape(-1,stride)
        vertices={v.get('id'):v.find('c:input',ns).get('source')[1:] for v in mesh.findall('c:vertices',ns)}
        triangles=[]
        if mesh.findall('c:polylist',ns):raise ValueError('triangulate COLLADA polylist before evaluation')
        for primitive in mesh.findall('c:triangles',ns):
            inputs=primitive.findall('c:input',ns)
            vertex=next(v for v in inputs if v.get('semantic')=='VERTEX')
            stride=max(int(v.get('offset')) for v in inputs)+1
            indices=np.fromstring(primitive.findtext('c:p',namespaces=ns),sep=' ',dtype=int)
            indices=indices.reshape(-1,stride)[:,int(vertex.get('offset'))].reshape(-1,3)
            triangles.append(sources[vertices[vertex.get('source')[1:]]][indices,:3])
        geometries[item.get('id')]=np.concatenate(triangles) if triangles else np.empty((0,3,3))
    def visit(node,parent):
        m=parent.copy()
        for child in node:
            tag=child.tag.split('}')[-1]
            if tag=='matrix':m=m@np.fromstring(child.text,sep=' ').reshape(4,4)
            elif tag in ('rotate','translate','scale'):raise ValueError('reference requires matrix COLLADA transforms')
        for instance in node.findall('c:instance_geometry',ns):
            t=geometries[instance.get('url')[1:]]
            result=t@m[:3,:3].T+m[:3,3]
            if np.linalg.det(m[:3,:3])<0:result=result[:,[0,2,1]]
            yield result
        for sub in node.findall('c:node',ns):yield from visit(sub,m)
    for node in root.findall('c:library_visual_scenes/c:visual_scene/c:node',ns):
        yield from visit(node,transform)


def collision_meshes(world,model_roots,dependencies):
    def resolve(uri,base):
        if uri.startswith('model://'):
            for root in model_roots:
                candidate=Path(root)/uri[8:]
                if candidate.exists():return candidate
            raise ValueError('unresolved model URI: '+uri)
        candidate=(base/uri.removeprefix('file://')).resolve()
        if not candidate.exists():raise ValueError('missing collision asset: '+str(candidate))
        return candidate
    def load(path,parent):
        path=Path(path).resolve();dependencies.add(path)
        root=ET.parse(path).getroot();container=root.find('world')
        if container is None:container=root
        yield from contents(container,path.parent,parent)
    def contents(container,base,parent):
        for include in container.findall('include'):
            path=resolve(include.findtext('uri'),base)
            if path.is_dir():path=path/'model.sdf'
            yield from load(path,parent@pose_matrix(include.findtext('pose')))
        for model in container.findall('model'):
            m=parent@pose_matrix(model.findtext('pose'))
            for link in model.findall('link'):
                frame=m@pose_matrix(link.findtext('pose'))
                for collision in link.findall('collision'):
                    t=frame@pose_matrix(collision.findtext('pose'));g=collision.find('geometry')
                    if g.find('mesh') is not None:
                        asset=resolve(g.findtext('mesh/uri'),base);dependencies.add(asset.resolve())
                        scale=np.diag([*map(float,(g.findtext('mesh/scale') or '1 1 1').split()),1.])
                        if asset.suffix.lower()!='.dae':raise ValueError('only COLLADA collision meshes supported')
                        # A collision mesh can split one closed body across
                        # COLLADA material geometries. Pair its vertical hits
                        # together; pairing each material separately can turn
                        # several floor slabs into one solid spanning rooms.
                        pieces=list(dae_triangles(asset,t@scale))
                        if pieces:yield np.concatenate(pieces)
                    elif g.find('box') is not None:
                        x,y,z=np.asarray(list(map(float,g.findtext('box/size').split())))/2
                        v=np.asarray([(-x,-y,-z),(x,-y,-z),(x,y,-z),(-x,y,-z),
                                      (-x,-y,z),(x,-y,z),(x,y,z),(-x,y,z)])
                        faces=[[0,2,1],[0,3,2],[4,5,6],[4,6,7],[0,1,5],[0,5,4],
                               [1,2,6],[1,6,5],[2,3,7],[2,7,6],[3,0,4],[3,4,7]]
                        yield (v@t[:3,:3].T+t[:3,3])[np.asarray(faces)]
                    elif g.find('plane') is not None:
                        normal=np.asarray(list(map(float,(g.findtext('plane/normal') or '0 0 1').split())))
                        if not np.allclose(normal,[0,0,1]):raise ValueError('reference requires local +Z plane normal')
                        x,y=np.asarray(list(map(float,g.findtext('plane/size').split())))/2
                        v=np.asarray([(-x,-y,0),(x,-y,0),(x,y,0),(-x,y,0)])
                        yield (v@t[:3,:3].T+t[:3,3])[np.asarray([[0,1,2],[0,2,3]])]
                    elif g.find('cylinder') is not None:
                        radius=float(g.findtext('cylinder/radius'));half=float(g.findtext('cylinder/length'))/2
                        angles=np.arange(64)*2*np.pi/64
                        rim=np.column_stack((radius*np.cos(angles),radius*np.sin(angles)))
                        v=np.vstack((np.column_stack((rim,np.full(64,-half))),
                                     np.column_stack((rim,np.full(64,half))),[[0,0,-half],[0,0,half]]))
                        faces=[]
                        for i in range(64):
                            j=(i+1)%64
                            faces.extend([[128,j,i],[129,i+64,j+64],[i,j,j+64],[i,j+64,i+64]])
                        yield (v@t[:3,:3].T+t[:3,3])[np.asarray(faces)]
                    else:raise ValueError('unsupported collision geometry in '+collision.get('name',''))
            yield from contents(model,base,m)
    yield from load(Path(world),np.eye(4))


def build_reference(world,model_roots,spawn=(-6.75,9.6,0.),resolution=.1,step_max=.16,clearance=.5):
    supports=defaultdict(list);solids=defaultdict(list);dependencies=set()
    for triangles in collision_meshes(world,model_roots,dependencies):
        hits=defaultdict(list)
        for t in triangles:
            normal=np.cross(t[1]-t[0],t[2]-t[0])
            if abs(normal[2])<1e-10:continue
            lo=np.ceil(t[:,:2].min(axis=0)/resolution-1e-7).astype(int)
            hi=np.floor(t[:,:2].max(axis=0)/resolution+1e-7).astype(int)
            if np.prod(hi-lo+1)>2000000:raise ValueError('reference triangle exceeds grid safety limit')
            x,y=np.meshgrid(np.arange(lo[0],hi[0]+1),np.arange(lo[1],hi[1]+1),indexing='ij')
            if not x.size:continue
            dx=x.ravel()*resolution-t[0,0];dy=y.ravel()*resolution-t[0,1]
            a,b=t[1]-t[0],t[2]-t[0];det=a[0]*b[1]-a[1]*b[0]
            u=(dx*b[1]-dy*b[0])/det;v=(a[0]*dy-a[1]*dx)/det
            valid=(u>=-1e-7)&(v>=-1e-7)&(u+v<=1+1e-7)
            z=t[0,2]+u*a[2]+v*b[2]
            sign=1 if normal[2]>0 else -1
            walkable=sign>0 and normal[2]/np.linalg.norm(normal)>=math.cos(.4)
            for ix,iy,height in zip(x.ravel()[valid],y.ravel()[valid],z[valid]):
                key=int(ix),int(iy);hits[key].append((float(height),sign))
                if walkable:supports[key].append(float(height))
        for xy,records in hits.items():
            records=sorted(set((round(z,5),sign) for z,sign in records));lower=None
            for z,sign in records:
                if sign<0:lower=z if lower is None else min(lower,z)
                elif lower is not None:solids[xy].append((lower,z));lower=None
    usable={}
    for xy,heights in supports.items():
        heights=sorted(heights);unique=[]
        for z in heights:
            if unique and z-unique[-1]<.015:continue
            if any(low<z+clearance-1e-5 and high>z+.015 for low,high in solids[xy]):continue
            unique.append(z)
        if unique:usable[xy]=unique
    seed_xy=tuple(np.rint(np.asarray(spawn[:2])/resolution).astype(int))
    choices=[(xy,i,z) for xy,heights in usable.items() for i,z in enumerate(heights)
             if math.dist(xy,seed_xy)*resolution<=.5 and abs(z-spawn[2])<=step_max]
    if not choices:raise ValueError('spawn is outside reference support')
    xy,i,_=min(choices,key=lambda a:math.dist(a[0],seed_xy)+abs(a[2]-spawn[2]))
    reached={(xy,i)};queue=deque(reached)
    while queue:
        xy,i=queue.popleft();z=usable[xy][i]
        for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
            neighbor=xy[0]+dx,xy[1]+dy
            for j,height in enumerate(usable.get(neighbor,())):
                key=neighbor,j
                if key not in reached and abs(height-z)<=step_max+1e-6:
                    reached.add(key);queue.append(key)
    xyz=np.asarray([(x*resolution,y*resolution,usable[x,y][i]) for (x,y),i in sorted(reached)],dtype=np.float32)
    metadata=dict(builder_version=REFERENCE_VERSION,
        definition='reachable static collision support surface; point-robot, 4-neighbor, projected XY area',
        resolution=resolution,step_max=step_max,minimum_clearance=clearance,spawn=list(spawn),
        cells=len(xyz),area_m2=len(xyz)*resolution**2,
        assets={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(dependencies)})
    return xyz,metadata


def save_reference(path,xyz,metadata):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    np.savez_compressed(path,xyz=xyz,resolution=metadata['resolution'],metadata=json.dumps(metadata))
