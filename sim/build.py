"""Derive a two-arm concept scene from v2, keeping motor/handle layout explicit."""
from pathlib import Path
import xml.etree.ElementTree as E
import numpy as np, mujoco
from layout import DRIVE_HOME, SUPPORT_HOME
P=Path(__file__).resolve().parent
root=E.parse(P/'scene_base.xml').getroot();root.set('model','open_h_two_arm_concept')
w=root.find('worldbody');act=root.find('actuator')
support=next(e for e in w.iter('body') if e.get('name')=='support_wrist_3_link')
handle=E.SubElement(support,'body',name='scope_handle',pos='0 .1 0',quat='.70710678 -.70710678 0 0')
E.SubElement(handle,'geom',name='handle_grip',type='cylinder',size='.021 .055',pos='0 0 .055',rgba='.12 .14 .16 1',mass='.16',contype='0',conaffinity='0')
E.SubElement(handle,'geom',name='handle_nose',type='cylinder',size='.014 .012',pos='0 0 .122',rgba='.23 .24 .25 1',mass='.015',contype='0',conaffinity='0')
E.SubElement(handle,'site',name='handle_exit',pos='0 0 .134',size='.003',rgba='1 0 0 0')
for axis,offset,color in [('x',.035,'.14 .31 .58 1'),('y',.075,'.19 .48 .70 1')]:
 E.SubElement(handle,'geom',name=f'{axis}_motor_case',type='box',pos=f'0 .034 {offset}',size='.012 .014 .018',rgba=color,mass='.06',contype='0',conaffinity='0')
 rotor=E.SubElement(handle,'body',name=f'{axis}_motor_rotor',pos=f'0 .016 {offset}')
 E.SubElement(rotor,'joint',name=f'{axis}_motor_joint',type='hinge',axis='0 1 0',range='-.8 .8',damping='.04',armature='.001')
 E.SubElement(rotor,'geom',type='cylinder',name=f'{axis}_motor_shaft',size='.004 .008',quat='.70710678 .70710678 0 0',rgba='.8 .8 .8 1',mass='.008',contype='0',conaffinity='0')
 E.SubElement(rotor,'geom',type='box',pos='0 .009 0',size='.011 .002 .003',rgba='.95 .6 .1 1',mass='.003',contype='0',conaffinity='0')
 E.SubElement(act,'position',name=f'{axis}_motor',joint=f'{axis}_motor_joint',kp='.2',ctrlrange='-.65 .65',forcerange='-.08 .08')
clamp=next(e for e in w.iter('body') if e.get('name')=='clamp')
E.SubElement(clamp,'site',name='clamp_entry',pos='0 0 -.012',size='.003',rgba='1 0 0 0')
# Determine both attachment points at the same robot home pose used at runtime.
tmp=P/'bridge_layout_tmp.xml'
E.ElementTree(root).write(tmp,encoding='unicode')
m=mujoco.MjModel.from_xml_path(str(tmp));d=mujoco.MjData(m)
J=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
for prefix,home in (('',DRIVE_HOME),('support_',SUPPORT_HOME)):
 for name,value in zip(J,home):
  d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,prefix+name)]]=value
mujoco.mj_forward(m,d)
hid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'scope_handle')
a=d.site_xpos[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'handle_exit')].copy()
b=d.site_xpos[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'scope_proximal_site')].copy()
rot=d.xmat[hid].reshape(3,3);origin=d.xpos[hid].copy()
# Rest curve has slack; hinges represent bending compliance, while each segment is inextensible.
N=16
points=[]
for i in range(N+1):
 t=i/N; world=(1-t)*a+t*b+np.array([0.,0.,-.13*np.sin(np.pi*t)**2])
 points.append(rot.T@(world-origin))
fmt=lambda v:' '.join(f'{x:.9g}' for x in v)
node=handle
for i in range(N):
 node=E.SubElement(node,'body',name=f'proximal_segment_{i}',pos=fmt(points[i] if i==0 else points[i]-points[i-1]))
 for axis,vec in [('x','1 0 0'),('y','0 1 0'),('z','0 0 1')]:
  E.SubElement(node,'joint',name=f'proximal_{axis}_{i}',type='hinge',axis=vec,damping='.015',stiffness='.008',armature='.00005')
 E.SubElement(node,'geom',name=f'proximal_shaft_{i}',type='capsule',fromto='0 0 0 '+fmt(points[i+1]-points[i]),size='.006',density='550',rgba='.08 .09 .10 1',contype='1',conaffinity='14',friction='.12 .005 .0001',condim='3')
E.SubElement(node,'site',name='proximal_chain_end',pos=fmt(points[-1]-points[-2]),size='.002',rgba='1 1 1 0')
eq=root.find('equality')
if eq is None:eq=E.SubElement(root,'equality')
E.SubElement(eq,'connect',name='tube_connection',site1='proximal_chain_end',site2='scope_proximal_site',solref='.025 1')
E.ElementTree(root).write(P/'scene.xml',encoding='unicode');tmp.unlink();print('Built',P/'scene.xml')
