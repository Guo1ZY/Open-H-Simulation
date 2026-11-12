"""C3VDv2 integration checks: open lumen, wall obstruction, dynamic steering."""
from pathlib import Path
import csv,json,xml.etree.ElementTree as ET
import numpy as np
import mujoco
from layout import DRIVE_HOME,SUPPORT_HOME,track_clamp
P=Path(__file__).resolve().parent
J=['shoulder_pan_joint','shoulder_lift_joint','elbow_joint','wrist_1_joint','wrist_2_joint','wrist_3_joint']
def ident(m,kind,name):return mujoco.mj_name2id(m,kind,name)
def setup(m):
 d=mujoco.MjData(m)
 js=[ident(m,mujoco.mjtObj.mjOBJ_JOINT,n) for n in J];q=np.array([m.jnt_qposadr[j] for j in js]);v=np.array([m.jnt_dofadr[j] for j in js])
 sq=[m.jnt_qposadr[ident(m,mujoco.mjtObj.mjOBJ_JOINT,'support_'+n)] for n in J]
 d.qpos[q]=DRIVE_HOME;d.qpos[sq]=SUPPORT_HOME;d.ctrl[:6]=DRIVE_HOME;d.ctrl[6:12]=SUPPORT_HOME
 jq=[m.jnt_qposadr[ident(m,mujoco.mjtObj.mjOBJ_JOINT,'jaw_'+a+'_slide')] for a in ('left','right')];jc=[ident(m,mujoco.mjtObj.mjOBJ_ACTUATOR,'jaw_'+a+'_motor') for a in ('left','right')]
 d.qpos[jq]=.004;d.ctrl[jc]=.007;mujoco.mj_forward(m,d)
 return d,q,v

def run():
 placement=json.loads((P/'colon/placement.json').read_text());entry=np.array(placement['world_entry_m'])
 # Independent 6 mm sphere probe verifies the imported hollow collider, with no simulation changes.
 root=ET.parse(P/'scene.xml').getroot();root.find('compiler').set('meshdir',str((P.parent/'ur5e/assets').resolve()))
 b=ET.SubElement(root.find('worldbody'),'body',name='verification_probe');ET.SubElement(b,'freejoint',name='probe_free');ET.SubElement(b,'geom',name='verification_probe_geom',type='sphere',size='.006',contype='1',conaffinity='2')
 pm=mujoco.MjModel.from_xml_string(ET.tostring(root,encoding='unicode'));pd,_,_=setup(pm)
 pb=ident(pm,mujoco.mjtObj.mjOBJ_BODY,'verification_probe');pg=ident(pm,mujoco.mjtObj.mjOBJ_GEOM,'verification_probe_geom');probe_q=pm.jnt_qposadr[ident(pm,mujoco.mjtObj.mjOBJ_JOINT,'probe_free')]
 probes=[]
 for s in (-.02,-.01,0,.005,.01,.015,.025,.035,.045,.05,.055,.06):
  pd.qpos[probe_q:probe_q+3]=entry+np.array([s,0.,0.]);mujoco.mj_forward(pm,pd)
  contacts=[c for c in pd.contact if pg in (c.geom1,c.geom2)]
  probes.append(dict(depth_m=s,contacts=len(contacts),penetration_m=max([0.]+[-float(c.dist) for c in contacts])))
 print('Probe scan:',probes,flush=True)
 assert all(r['contacts']==0 for r in probes if r['depth_m']<=.015),'Inlet blocked or too narrow'
 assert any(r['contacts']>0 for r in probes if .05<=r['depth_m']<=.06),'Expected curved wall is missing'
 # Use the same actuator / Cartesian controller as the keyboard viewer.
 m=mujoco.MjModel.from_xml_path(str(P/'scene.xml'));d,q,v=setup(m);ik=mujoco.MjData(m)
 clamp=ident(m,mujoco.mjtObj.mjOBJ_BODY,'clamp');tip=ident(m,mujoco.mjtObj.mjOBJ_SITE,'tip');start=d.xpos[clamp].copy();R=d.xmat[clamp].reshape(3,3).copy();axis=R[:,2]
 names=[mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,i) or '' for i in range(m.ngeom)];walls=set(i for i,n in enumerate(names) if n.startswith('wall_'));tubes=set(i for i,n in enumerate(names) if n.startswith(('shaft_','proximal_shaft_')));robots=set(np.where(m.geom_contype==8)[0])
 ends=[ident(m,mujoco.mjtObj.mjOBJ_SITE,n) for n in ('proximal_chain_end','scope_proximal_site')]
 motors=[ident(m,mujoco.mjtObj.mjOBJ_ACTUATOR,a+'_motor') for a in ('x','y')];mq=[m.jnt_qposadr[ident(m,mujoco.mjtObj.mjOBJ_JOINT,a+'_motor_joint')] for a in ('x','y')]
 steer=[ident(m,mujoco.mjtObj.mjOBJ_ACTUATOR,f'drive_{a}_{i}') for i in range(8,12) for a in ('x','y')]
 initial_wall=sum((c.geom1 in tubes and c.geom2 in walls) or (c.geom2 in tubes and c.geom1 in walls) for c in d.contact)
 assert initial_wall==0,'Default pose starts inside wall'
 rows=[];max_pen=wall_force=max_gap=0.;wall_contacts=robot_contacts=0;previous_distance=previous_steer=0.
 phases=[('settle',.4,0.,0.),('advance',2.,.025,0.),('turn_advance',2.,.05,.6),('hold',.4,.05,.6),('retract_turn',3.,.005,0.),('rest',.5,.005,0.)]
 for phase,duration,goal,turn in phases:
  ticks=round(duration/.02)
  for k in range(ticks):
   t=(k+1)/ticks;distance=previous_distance+(goal-previous_distance)*t;angle=previous_steer+(turn-previous_steer)*t
   track_clamp(m,d,ik,q,v,clamp,start+axis*distance,R);d.ctrl[motors[0]]=angle;d.ctrl[motors[1]]=0
   tick_wall=tick_robot=0;tick_pen=tick_force=0.
   for _ in range(10):
    for i in range(4):d.ctrl[steer[2*i]]=d.qpos[mq[1]]/4;d.ctrl[steer[2*i+1]]=d.qpos[mq[0]]/4
    mujoco.mj_step(m,d)
    for j,c in enumerate(d.contact):
     a,b=int(c.geom1),int(c.geom2)
     if (a in walls and b in tubes) or (b in walls and a in tubes):
      tick_wall+=1;tick_pen=max(tick_pen,-float(c.dist));f=np.zeros(6);mujoco.mj_contactForce(m,d,j,f);tick_force=max(tick_force,float(f[0]))
     if (a in robots and b in tubes) or (b in robots and a in tubes):tick_robot+=1
   mujoco.mj_forward(m,d);gap=float(np.linalg.norm(d.site_xpos[ends[0]]-d.site_xpos[ends[1]]));max_gap=max(max_gap,gap);max_pen=max(max_pen,tick_pen);wall_force=max(wall_force,tick_force);wall_contacts+=tick_wall;robot_contacts+=tick_robot
   rows.append(dict(time_s=float(d.time),phase=phase,command_m=distance,travel_m=float((d.xpos[clamp]-start)@axis),steer_rad=angle,tip_x_m=float(d.site_xpos[tip,0]),tip_y_m=float(d.site_xpos[tip,1]),tip_z_m=float(d.site_xpos[tip,2]),wall_contacts=tick_wall,wall_max_force_N=tick_force,wall_penetration_m=tick_pen,robot_contacts=tick_robot,connection_gap_m=gap))
   assert np.all(np.isfinite(d.qpos)), 'Non-finite state'
  previous_distance=goal;previous_steer=turn
  print(phase,rows[-1],flush=True)
 report=dict(source=placement['source'],geometry=placement,probe_scan=probes,initial_wall_contacts=int(initial_wall),simulation_seconds=float(d.time),max_wall_penetration_m=max_pen,max_wall_single_contact_force_N=wall_force,wall_contacts_all_steps=wall_contacts,robot_contacts_all_steps=robot_contacts,max_connection_gap_m=max_gap,notes='Rigid hollow shell. Probe verifies open entry and expected curved-wall obstruction. Dynamic path tests limited forward/back and steering; not full-segment navigation.')
 out=P/'verification';out.mkdir(exist_ok=True)
 with (out/'colon_states.csv').open('w',newline='') as f:w=csv.DictWriter(f,fieldnames=rows[0]);w.writeheader();w.writerows(rows)
 (out/'colon_report.json').write_text(json.dumps(report,indent=2))
 print(json.dumps({k:v for k,v in report.items() if k not in ('geometry','probe_scan')},indent=2))
 assert robot_contacts==0,'Tube encountered robot'
 assert max_pen<.002,'Excess wall penetration'
 assert max_gap<.003,'Tube attachment opened'
 return report
if __name__=='__main__':run()
