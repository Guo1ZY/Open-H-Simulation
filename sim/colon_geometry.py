"""C3VDv2 sigmoid1 surface: mm->m, rigid placement and hollow collision shell.
Source: Golhar et al., JHU doi:10.7281/T1/JC64MK, CC BY 4.0.
"""
from pathlib import Path
import json
import numpy as np
import trimesh
import xml.etree.ElementTree as E
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
P=Path(__file__).resolve().parent
SOURCE=P.parent/'assets/c3vdv2/source/c1_sigmoid1.obj'

def boundary_loops(mesh):
 edges,count=np.unique(mesh.edges_sorted,axis=0,return_counts=True);edges=edges[count==1]
 ids=np.unique(edges);ix={v:i for i,v in enumerate(ids)}
 a=np.array([ix[i] for i in edges[:,0]]);b=np.array([ix[i] for i in edges[:,1]])
 graph=coo_matrix((np.ones(len(a)*2),(np.r_[a,b],np.r_[b,a])),shape=(len(ids),len(ids))).tocsr()
 _,labels=connected_components(graph)
 return [mesh.vertices[ids[labels==i]] for i in np.unique(labels)]

def prepare():
 out=P/'colon';out.mkdir(exist_ok=True)
 if (out/'geometry.json').exists():return json.loads((out/'geometry.json').read_text())
 mesh=trimesh.load(SOURCE,force='mesh');raw_faces=len(mesh.faces)
 loops=boundary_loops(mesh);assert len(loops)==2, 'Expected two open ends'
 inlet=min(loops,key=lambda x:x[:,0].mean());outlet=max(loops,key=lambda x:x[:,0].mean())
 center=inlet.mean(axis=0);_,_,vh=np.linalg.svd(inlet-center);inward=vh[-1]
 if inward@(outlet.mean(axis=0)-center)<0:inward=-inward
 # Local x points inward; roll the outlet into the horizontal plane.
 lateral=outlet.mean(axis=0)-center;lateral-=inward*(lateral@inward);lateral/=np.linalg.norm(lateral)
 up=np.cross(inward,lateral);frame=np.stack([inward,lateral,up])
 mesh.vertices=(mesh.vertices-center)@frame.T*.001
 mesh.fix_normals(multibody=True)
 if mesh.volume<0:mesh.invert()
 visual=mesh.simplify_quadric_decimation(face_count=45000);collision=mesh.simplify_quadric_decimation(face_count=3500)
 # Keep original open surfaces. No filled caps or enclosing convex hull.
 visual.export(out/'visual.obj');inner=visual.copy();inner.invert();inner.export(out/'interior.obj');np.savez(out/'collision.npz',vertices=collision.vertices,faces=collision.faces,normals=collision.vertex_normals)
 report=dict(source="assets/c3vdv2/source/c1_sigmoid1.obj",license='CC BY 4.0',scale_to_meters=.001,source_faces=raw_faces,visual_faces=len(visual.faces),collision_faces=len(collision.faces),source_inlet_center=center.tolist(),source_inward_normal=inward.tolist(),source_to_local_rotation=frame.tolist(),local_bounds_m=mesh.bounds.tolist(),outlet_local_m=((outlet.mean(axis=0)-center)@frame.T*.001).tolist(),inlet_radius_range_m=[float(np.linalg.norm((inlet-center)@frame.T*.001,axis=1).min()),float(np.linalg.norm((inlet-center)@frame.T*.001,axis=1).max())],wall_thickness_m=.002,description='Original open lumen surface; rigid uncalibrated wall, simplified visual and separate outward-extruded triangular prism colliders; no original RGB texture supplied.')
 (out/'geometry.json').write_text(json.dumps(report,indent=2));print('Prepared colon:',report,flush=True)
 return report

def add_colon(root,world,tip,rotation):
 report=prepare();asset=root.find('asset')
 # Tube advances along world +X in the V9 layout. Start tip 15 mm inside inlet.
 entry=tip-np.array([.015,0,0]);fmt=lambda v:' '.join(f'{x:.9g}' for x in v)
 E.SubElement(asset,'mesh',name='c3vd_sigmoid_visual',file='../../sim/colon/visual.obj')
 E.SubElement(asset,'material',name='mucosa',rgba='.72 .32 .29 1',specular='.25',shininess='.35')
 E.SubElement(world,'geom',name='lumen_surface',type='mesh',mesh='c3vd_sigmoid_visual',material='mucosa',pos=fmt(entry),contype='0',conaffinity='0',group='4')
 E.SubElement(asset,'mesh',name='c3vd_sigmoid_interior',file='../../sim/colon/interior.obj')
 E.SubElement(world,'geom',name='lumen_interior',type='mesh',mesh='c3vd_sigmoid_interior',material='mucosa',pos=fmt(entry),contype='0',conaffinity='0',group='5')
 data=np.load(P/'colon/collision.npz');vertices=data['vertices'];normals=data['normals'];faces=data['faces']
 for i,face in enumerate(faces):
  v=vertices[face];outer=v+.002*normals[face]
  if np.linalg.norm(np.cross(v[1]-v[0],v[2]-v[0]))<1e-12:continue
  E.SubElement(asset,'mesh',name=f'wall_mesh_{i}',vertex=fmt(np.vstack((v,outer)).ravel()))
  E.SubElement(world,'geom',name=f'wall_{i}',type='mesh',mesh=f'wall_mesh_{i}',pos=fmt(entry),rgba='.72 .32 .29 1',group='3',contype='2',conaffinity='1',friction='.12 .005 .0001',condim='3',solref='.012 1')
 report['world_entry_m']=entry.tolist();(P/'colon/placement.json').write_text(json.dumps(report,indent=2))
