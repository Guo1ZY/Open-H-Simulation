"""Common SI-unit layout for the compact transverse gripper."""
import numpy as np
DRIVE_HOME=np.array([-np.pi/2,-np.pi/2,np.pi/2,-np.pi/2,np.pi/2,0.])
SUPPORT_HOME=np.array([-1.5707963267948968,-1.5288646806738675,1.2454669122383613,-1.4965473916805756,-np.pi,-0.2091488333211852])
STROKE=(-.06,.08)

def track_clamp(m,d,ik_data,q,v,clamp,position,rotation):
 """Fixed-orientation Cartesian target, achieved through UR joint actuators."""
 import mujoco
 ik_data.qpos[:]=d.qpos;ik_data.qvel[:]=0
 for _ in range(6):
  mujoco.mj_kinematics(m,ik_data);mujoco.mj_comPos(m,ik_data)
  R=ik_data.xmat[clamp].reshape(3,3)
  error=np.r_[position-ik_data.xpos[clamp],.5*sum(np.cross(R[:,i],rotation[:,i]) for i in range(3))]
  jp=np.zeros((3,m.nv));jr=np.zeros((3,m.nv));mujoco.mj_jacBody(m,ik_data,jp,jr,clamp)
  jac=np.vstack((jp[:,v],jr[:,v]));dq=jac.T@np.linalg.solve(jac@jac.T+1e-5*np.eye(6),error)
  ik_data.qpos[q]+=np.clip(dq,-.025,.025)
 d.ctrl[:6]=ik_data.qpos[q];d.ctrl[6:12]=SUPPORT_HOME
