"""Generate a preliminary colon phantom and articulated scope scene (SI units)."""
from pathlib import Path
import copy, xml.etree.ElementTree as E, math
import numpy as np, mujoco
from layout import DRIVE_HOME, SUPPORT_HOME
P=Path(__file__).resolve().parent; parent=P.parent
root=E.parse(parent/'ur5e/ur5e.xml').getroot();root.set('model','openh_v2');root.find('compiler').set('meshdir','../ur5e/assets')
option=root.find('option');option.set('timestep','0.002');option.set('gravity','0 0 0');option.set('integrator','implicitfast')
visual=E.SubElement(root,'visual');E.SubElement(visual,'map',znear='.0002',zfar='10');E.SubElement(visual,'rgba',haze='.8 .85 .9 1');E.SubElement(root.find('asset'),'texture',name='sky',type='skybox',builtin='gradient',rgb1='.72 .80 .87',rgb2='.95 .96 .97',width='256',height='256')
world=root.find('worldbody');arm=world.find('body');other=copy.deepcopy(arm)
for e in other.iter():
 if e.get('name'):e.set('name','support_'+e.get('name'))
other.set('pos','-.8 0 0');world.append(other)
act=root.find('actuator')
for e in list(act):
 n=copy.deepcopy(e);n.set('name','support_'+n.get('name'));n.set('joint','support_'+n.get('joint'));act.append(n)
root.remove(root.find('keyframe'))
# Enable tube-versus-robot collision using the UR5e collision proxies (group 3).
# Do not enable robot-vs-robot contacts: this version only protects the tube.
for e in world.iter('geom'):
 if e.get('class') in ('collision','eef_collision'):
  e.set('contype','8');e.set('conaffinity','1')
 else:
  e.set('contype','0');e.set('conaffinity','0')
wrist=next(e for e in arm.iter('body') if e.get('name')=='wrist_3_link')
# Upward-facing flange supports a compact transverse gripper directly.
clamp=E.SubElement(wrist,'body',name='clamp',pos='0 .13 0',quat='.70710678 0 -.70710678 0')
E.SubElement(clamp,'geom',name='fixture_mount',type='box',pos='0 -.024 0',size='.035 .006 .030',rgba='.25 .42 .55 1',mass='.05',contype='8',conaffinity='1')
for x in (-.027,.027):
 for z in (-.022,.022):
  E.SubElement(clamp,'geom',type='cylinder',pos=f'{x} -.016 {z}',size='.003 .002',quat='.70710678 .70710678 0 0',rgba='.65 .68 .70 1',mass='.001',contype='0',conaffinity='0')
for side,label in [(-1,'left'),(1,'right')]:
 jaw=E.SubElement(clamp,'body',name=f'jaw_{label}',pos=f'{side*.020} 0 0')
 E.SubElement(jaw,'joint',name=f'jaw_{label}_slide',type='slide',axis=f'{-side} 0 0',range='0 .014',damping='.5',armature='.001')
 E.SubElement(jaw,'geom',name=f'jaw_body_{label}',type='box',pos='0 0 0',size='.007 .019 .023',rgba='.70 .76 .79 1',mass='.025',contype='0',conaffinity='0')
 E.SubElement(jaw,'geom',name=f'jaw_pad_{label}',type='box',pos=f'{-side*.008} 0 0',size='.0015 .014 .020',rgba='.96 .52 .12 1',mass='.003',contype='4',conaffinity='1',friction='.5 .01 .001',condim='3')
 E.SubElement(act,'position',name=f'jaw_{label}_motor',joint=f'jaw_{label}_slide',kp='120',ctrlrange='0 .007',forcerange='-1 1')
node=clamp
# Segment lengths and stiffness are guessed values, to be identified experimentally.
N=12; length=.018
for i in range(N):
 node=E.SubElement(node,'body',name=f'scope_{i}',pos='0 0 -.050' if i==0 else f'0 0 {length}')
 if i==0:
  E.SubElement(node,'joint',name='scope_slide',type='slide',axis='0 0 1',range='-.16 .16',damping='.002',armature='.00005')
  E.SubElement(node,'site',name='scope_proximal_site',pos='0 0 0',size='.002',rgba='1 1 1 0')
 for axis in ('x','y'):
  E.SubElement(node,'joint',name=f'bend_{axis}_{i}',type='hinge',axis='1 0 0' if axis=='x' else '0 1 0',range='-.35 .35',damping='.016',stiffness='.12' if i<8 else '.045',armature='.00001')
 E.SubElement(node,'geom',name=f'shaft_{i}',type='capsule',fromto=f'0 0 0 0 0 {length}',size='.006',density='550',rgba='.14 .16 .18 1',contype='1',conaffinity='14',friction='.12 .005 .0001',condim='3')
 if i>=8:
  for axis in ('x','y'):
   E.SubElement(act,'position',name=f'drive_{axis}_{i}',joint=f'bend_{axis}_{i}',kp='.08',ctrlrange='-.20 .20',forcerange='-.04 .04')
E.SubElement(node,'site',name='tip',pos=f'0 0 {length}',size='.002',rgba='1 .3 .1 1')
E.SubElement(node,'camera',name='scope_camera',pos=f'0 0 {length+.002}',xyaxes='1 0 0 0 -1 0',fovy='90')
E.SubElement(node,'light',pos=f'0 0 {length+.003}',dir='0 0 1',diffuse='.9 .8 .7',cutoff='65',exponent='4')
E.SubElement(world,'light',pos='0 0 2',dir='0 0 -1',diffuse='.7 .7 .7')
E.SubElement(world,'geom',type='plane',size='2 2 .1',rgba='.72 .76 .80 1',contype='0',conaffinity='0')
# Compile once to locate the camera and orient a synthetic phantom relative to it.
E.ElementTree(root).write(P/'scene_tmp.xml',encoding='unicode')
m=mujoco.MjModel.from_xml_path(str(P/'scene_tmp.xml'));d=mujoco.MjData(m)
J=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint'];home=[-1.5708,-1.5708,1.5708,-1.5708,-3.14159,0]
for prefix,home in (('',DRIVE_HOME),('support_',SUPPORT_HOME)):
 for n,v in zip(J,home): d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,prefix+n)]]=v
mujoco.mj_forward(m,d);tid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tip');c=d.site_xpos[tid].copy(); rot=d.site_xmat[tid].reshape(3,3);x=rot[:,0].copy();y=rot[:,1].copy();z=rot[:,2].copy()
from colon_geometry import add_colon
add_colon(root,world,c,rot)
E.ElementTree(root).write(P/'scene_base.xml',encoding='unicode');(P/'scene_tmp.xml').unlink()
print('Wrote',P/'scene_base.xml')
