"""Two-arm scope concept: support arm holds motorized handle, drive arm translates clamp."""
import argparse,csv,json,sys,time
from pathlib import Path
import glfw,mujoco,numpy as np
P=Path(__file__).resolve().parent
from common import png,J,ids,dead
from layout import DRIVE_HOME, SUPPORT_HOME, STROKE, track_clamp

def keyboard_command(pressed, focused):
 enable=focused and pressed(glfw.KEY_SPACE)
 if not enable:return False,0.,0.,0.
 return True, float(pressed(glfw.KEY_W))-float(pressed(glfw.KEY_S)), float(pressed(glfw.KEY_RIGHT))-float(pressed(glfw.KEY_LEFT)), float(pressed(glfw.KEY_DOWN))-float(pressed(glfw.KEY_UP))

def main():
 ap=argparse.ArgumentParser();ap.add_argument('--mode',choices=['keyboard','xbox','demo','griptest'],default='keyboard');ap.add_argument('--seconds',type=float,default=120);ap.add_argument('--no-video',action='store_true');ap.add_argument('--screenshot',type=str,default=None,help='Save first native composite frame');args=ap.parse_args()
 if not glfw.init():raise SystemExit('GLFW failed to initialize')
 window=glfw.create_window(1440,800,'Open-H V10 - C3VDv2 sigmoid + dual UR5',None,None)
 if not window:glfw.terminate();raise SystemExit('Could not create native window')
 glfw.make_context_current(window);glfw.swap_interval(1)
 m=mujoco.MjModel.from_xml_path(str(P/'scene.xml'));d=mujoco.MjData(m);ik_data=mujoco.MjData(m)
 joints=ids(m,mujoco.mjtObj.mjOBJ_JOINT,J);q=np.array([m.jnt_qposadr[i] for i in joints]);v=np.array([m.jnt_dofadr[i] for i in joints]);other=ids(m,mujoco.mjtObj.mjOBJ_JOINT,['support_'+x for x in J]);sq=np.array([m.jnt_qposadr[i] for i in other]);sv=np.array([m.jnt_dofadr[i] for i in other]);d.qpos[q]=DRIVE_HOME;d.qpos[sq]=SUPPORT_HOME;mujoco.mj_forward(m,d)
 clamp=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_BODY,'clamp');tip=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'tip');axis=d.xmat[clamp].reshape(3,3)[:,2].copy();start=d.xpos[clamp].copy();start_R=d.xmat[clamp].reshape(3,3).copy();steer=[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_ACTUATOR,f'drive_{a}_{i}') for i in range(8,12) for a in ('x','y')];bends=[m.jnt_qposadr[mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,f'bend_{a}_{i}')] for i in range(12) for a in ('x','y')]
 cam=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(m,cam);cam.lookat[:]=[-.36,.42,.44];cam.distance=2.10;cam.azimuth=100;cam.elevation=-18
 motor_joints=ids(m,mujoco.mjtObj.mjOBJ_JOINT,['x_motor_joint','y_motor_joint']);motor_q=[m.jnt_qposadr[j] for j in motor_joints];motor_ctrl=ids(m,mujoco.mjtObj.mjOBJ_ACTUATOR,['x_motor','y_motor']);ends=ids(m,mujoco.mjtObj.mjOBJ_SITE,['handle_exit','scope_proximal_site'])
 tube_eq=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_EQUALITY,'tube_connection')
 proximal_joints=ids(m,mujoco.mjtObj.mjOBJ_JOINT,[f'proximal_{a}_{i}' for i in range(16) for a in ('x','y','z')]);proximal_q=[m.jnt_qposadr[j] for j in proximal_joints]
 jaw_ids=ids(m,mujoco.mjtObj.mjOBJ_JOINT,['jaw_left_slide','jaw_right_slide']);jaw_q=[m.jnt_qposadr[j] for j in jaw_ids];jaw_motor=ids(m,mujoco.mjtObj.mjOBJ_ACTUATOR,['jaw_left_motor','jaw_right_motor']);slide_id=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_JOINT,'scope_slide');slide_q=m.jnt_qposadr[slide_id];slide_v=m.jnt_dofadr[slide_id]
 d.qpos[jaw_q]=.004;d.ctrl[jaw_motor]=.007;mujoco.mj_forward(m,d)
 handle_cam=mujoco.MjvCamera();mujoco.mjv_defaultFreeCamera(m,handle_cam);handle_cam.lookat[:]=d.site_xpos[ends[0]];handle_cam.distance=.40;handle_cam.azimuth=125;handle_cam.elevation=-20
 scope=mujoco.MjvCamera();scope.type=mujoco.mjtCamera.mjCAMERA_FIXED;scope.fixedcamid=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_CAMERA,'scope_camera')
 opt=mujoco.MjvOption();opt.geomgroup[4]=1;scope_opt=mujoco.MjvOption();scope_opt.geomgroup[4]=0;scope_opt.geomgroup[5]=1;scene=mujoco.MjvScene(m,maxgeom=5000);ctx=mujoco.MjrContext(m,mujoco.mjtFontScale.mjFONTSCALE_150)
 # Mouse interactions affect only the external 3D camera, never the scope camera.
 mouse={'last':None,'button':None}
 def on_mouse(w,button,action,mods):
  x,y=glfw.get_cursor_pos(w);ww,hh=glfw.get_window_size(w)
  if action==glfw.PRESS and x < ww*.60 and button in (glfw.MOUSE_BUTTON_LEFT,glfw.MOUSE_BUTTON_RIGHT,glfw.MOUSE_BUTTON_MIDDLE):mouse.update(last=(x,y),button=button)
  elif action==glfw.RELEASE and mouse['button']==button:mouse.update(last=None,button=None)
 def on_cursor(w,x,y):
  if mouse['last'] is None:return
  px,py=mouse['last'];mouse['last']=(x,y);ww,hh=glfw.get_window_size(w)
  if not hh:return
  action=mujoco.mjtMouse.mjMOUSE_ROTATE_V if mouse['button']==glfw.MOUSE_BUTTON_LEFT else mujoco.mjtMouse.mjMOUSE_MOVE_V
  mujoco.mjv_moveCamera(m,action,(x-px)/hh,(y-py)/hh,cam)
 def on_scroll(w,xoff,yoff):
  x,_=glfw.get_cursor_pos(w);ww,_=glfw.get_window_size(w)
  if x<ww*.60:mujoco.mjv_moveCamera(m,mujoco.mjtMouse.mjMOUSE_ZOOM,0,-.06*yoff,cam)
 glfw.set_mouse_button_callback(window,on_mouse);glfw.set_cursor_pos_callback(window,on_cursor);glfw.set_scroll_callback(window,on_scroll)
 # Offscreen renderer is used only for saved scope frames, separate from visible window.
 renderer=mujoco.Renderer(m,480,640) if not args.no_video else None
 glfw.make_context_current(window)
 out=P/'recordings_native'/time.strftime('%Y%m%d_%H%M%S');out.mkdir(parents=True,exist_ok=False)
 if renderer:(out/'frames').mkdir()
 (out/'metadata.json').write_text(json.dumps({'mode':args.mode,'simulated':True,'interface':'native MuJoCo GLFW window','arm':'position actuators track IK targets; support arm position controlled','scope':'12 distal segments plus 16 dynamic proximal segments, connected by a compliant equality constraint','phantom':'C3VDv2 c1_sigmoid1 (CC BY 4.0), rigid triangular prism wall; no tissue deformation','rates':'50 Hz states and 30 fps per simulation second; check wall_elapsed_s','input': 'keyboard space+arrows+W/S' if args.mode=='keyboard' else 'standard gamepad left stick; LT/RT; hold LB','motor_coupling':'two motor actuator states mapped to distal joint targets, no physical cable transmission','gripper':'dynamic jaws and free axial shaft slide with MuJoCo contact and friction; experimental uncalibrated model','collision':'tube versus simplified robot collision proxies enabled; arm-arm collisions disabled','warning':'not calibrated; not real hardware data'},indent=2))
 headers=['sim_time_s','wall_elapsed_s','enable','advance_input','steer_x','steer_y','insertion_command_m','clamp_progress_m','steer_x_command_rad','steer_y_command_rad','tip_x_m','tip_y_m','tip_z_m','shaft_wall_contacts','contact_force_N','frame_index']+[f'drive_q{i}' for i in range(6)]+[f'support_q{i}' for i in range(6)]+[f'bend_{i}' for i in range(24)]+['motor_x_command_rad','motor_y_command_rad','motor_x_measured_rad','motor_y_measured_rad','gripper_close_command','jaw_left_m','jaw_right_m','grip_engaged','shaft_slide_m','jaw_shaft_contacts','pad_normal_force_N','pad_tangential_force_N','shaft_slide_velocity_mps','jaw_target_m','tube_connection_force_N','proximal_max_joint_angle_rad','tube_connection_gap_m','tube_robot_contacts','tube_robot_max_penetration_m','clamp_lateral_error_m','clamp_rotation_error_rad']
 distance=x_cmd=y_cmd=0.;frame_idx=-1;nextframe=0.;t0=time.monotonic();last=t0;sim_t=0.;step_index=0;state=None;detail_view=0;c_was_pressed=False;g_was_pressed=False;grip_close=True;jaw_target=.007;key_was={glfw.KEY_LEFT_BRACKET:False,glfw.KEY_RIGHT_BRACKET:False}
 print('Native window: drag left side to orbit; right drag to pan; scroll to zoom. Keyboard: hold SPACE, arrows steer, W/S advance/retract; ESC closes.',flush=True)
 with (out/'states.csv').open('w',newline='') as f:
  wr=csv.writer(f);wr.writerow(headers)
  try:
   while not glfw.window_should_close(window) and step_index < int(args.seconds/.02):
    glfw.poll_events()
    if glfw.get_key(window,glfw.KEY_ESCAPE)==glfw.PRESS:break
    c_pressed=glfw.get_key(window,glfw.KEY_C)==glfw.PRESS
    if c_pressed and not c_was_pressed:detail_view=(detail_view+1)%3
    c_was_pressed=c_pressed
    g_pressed=glfw.get_key(window,glfw.KEY_G)==glfw.PRESS
    if g_pressed and not g_was_pressed and args.mode in ('keyboard','xbox'):grip_close=not grip_close
    g_was_pressed=g_pressed
    for key,delta in ((glfw.KEY_LEFT_BRACKET,-.00025),(glfw.KEY_RIGHT_BRACKET,.00025)):
     down=glfw.get_key(window,key)==glfw.PRESS
     if down and not key_was[key]:jaw_target=float(np.clip(jaw_target+delta,.004,.007))
     key_was[key]=down
    if args.mode=='keyboard':
     focused=bool(glfw.get_window_attrib(window,glfw.FOCUSED)); pressed=lambda key:glfw.get_key(window,key)==glfw.PRESS
     enable,forward,sx,sy=keyboard_command(pressed,focused)
    elif args.mode=='xbox':
     jid=next((j for j in range(glfw.JOYSTICK_1,glfw.JOYSTICK_LAST+1) if glfw.joystick_is_gamepad(j)),None)
     state=glfw.get_gamepad_state(jid) if jid is not None else None
     enable=bool(state and state.buttons[glfw.GAMEPAD_BUTTON_LEFT_BUMPER]);sx=dead(state.axes[glfw.GAMEPAD_AXIS_LEFT_X]) if enable else 0.;sy=dead(state.axes[glfw.GAMEPAD_AXIS_LEFT_Y]) if enable else 0.;forward=((state.axes[glfw.GAMEPAD_AXIS_RIGHT_TRIGGER]+1)/2-(state.axes[glfw.GAMEPAD_AXIS_LEFT_TRIGGER]+1)/2) if enable else 0.
    elif args.mode=='griptest':
     grip_close=not (2<=sim_t<4);enable=True;forward=.6 if sim_t<6 else 0.;sx=sy=0.
    else:
     enable=True;forward=.7 if sim_t<4 else (-.7 if sim_t>=8 else 0);sx=.65 if 3<=sim_t<6 else (-.65 if 6<=sim_t<9 else 0);sy=.45 if 4<=sim_t<5 else (-.45 if 5<=sim_t<6 else 0.)
    distance=float(np.clip(distance+forward*.032*.02,*STROKE));x_cmd=float(np.clip(x_cmd+sx*.25*.02,-.65,.65));y_cmd=float(np.clip(y_cmd+sy*.25*.02,-.65,.65));desired=start+axis*distance
    track_clamp(m,d,ik_data,q,v,clamp,desired,start_R)
    d.ctrl[jaw_motor]=jaw_target if grip_close else 0.
    d.ctrl[motor_ctrl[0]]=x_cmd;d.ctrl[motor_ctrl[1]]=y_cmd
    for substep in range(10):
     for i in range(4):d.ctrl[steer[2*i]]=d.qpos[motor_q[1]]/4;d.ctrl[steer[2*i+1]]=d.qpos[motor_q[0]]/4
     mujoco.mj_step(m,d)
    mujoco.mj_forward(m,d)
    if not np.all(np.isfinite(d.qpos)) or np.max(np.abs(d.qpos[bends]))>1.0:raise RuntimeError('Unstable scope state')
    nc=0;force=0.;pad_contacts=0;pad_normal=0.;pad_tangential=0.;robot_contacts=0;robot_penetration=0.
    for i in range(d.ncon):
     con=d.contact[i];a=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,con.geom1) or '';b=mujoco.mj_id2name(m,mujoco.mjtObj.mjOBJ_GEOM,con.geom2) or ''
     if m.geom_contype[con.geom1]==8 or m.geom_contype[con.geom2]==8:
      robot_contacts+=1;robot_penetration=max(robot_penetration,max(0.,-float(con.dist)))
     if (a.startswith('shaft_') and b.startswith('jaw_pad_')) or (b.startswith('shaft_') and a.startswith('jaw_pad_')):
      pad_contacts+=1;fc=np.zeros(6);mujoco.mj_contactForce(m,d,i,fc);pad_normal+=max(0.,fc[0]);pad_tangential+=float(np.linalg.norm(fc[1:3]))
     if (a.startswith('shaft_') and b.startswith('wall_')) or (b.startswith('shaft_') and a.startswith('wall_')):
      nc+=1;fv=np.zeros(6);mujoco.mj_contactForce(m,d,i,fv);force+=max(0.,fv[0])
    grip_engaged=grip_close and pad_contacts>0
    eqrows=np.where((d.efc_type==mujoco.mjtConstraint.mjCNSTR_EQUALITY) & (d.efc_id==tube_eq))[0]
    tube_force=float(np.linalg.norm(d.efc_force[eqrows]))
    chain_end=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_SITE,'proximal_chain_end')
    tube_gap=float(np.linalg.norm(d.site_xpos[chain_end]-d.site_xpos[ends[1]]))
    max_bend=float(np.max(np.abs(d.qpos[proximal_q])))
    if renderer and sim_t+1e-9>=nextframe:
     renderer.update_scene(d,camera='scope_camera',scene_option=scope_opt);renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False;im=renderer.render().copy();frame_idx+=1;png(out/'frames'/f'{frame_idx:06d}.png',im);nextframe+=1/30;glfw.make_context_current(window)
    width,height=glfw.get_framebuffer_size(window);split=int(width*.60);left=mujoco.MjrRect(0,0,split,height);right=mujoco.MjrRect(split,0,width-split,height//2);handle_view=mujoco.MjrRect(split,height//2,width-split,height-height//2)
    colon=mujoco.mj_name2id(m,mujoco.mjtObj.mjOBJ_GEOM,'lumen_surface')
    handle_cam.lookat[:]=d.geom_xpos[colon] if detail_view==0 else (d.xpos[clamp] if detail_view==1 else d.site_xpos[ends[0]])
    handle_cam.distance=.30 if detail_view==0 else (.32 if detail_view==1 else .40)
    handle_cam.elevation=-30 if detail_view==0 else -20
    mujoco.mjv_updateScene(m,d,opt,None,cam,mujoco.mjtCatBit.mjCAT_ALL,scene);scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=True;mujoco.mjr_render(left,scene,ctx)
    mujoco.mjv_updateScene(m,d,scope_opt,None,scope,mujoco.mjtCatBit.mjCAT_ALL,scene);scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=False;mujoco.mjr_render(right,scene,ctx)
    mujoco.mjv_updateScene(m,d,opt,None,handle_cam,mujoco.mjtCatBit.mjCAT_ALL,scene);scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW]=True;mujoco.mjr_render(handle_view,scene,ctx)
    label='KEYBOARD' if args.mode=='keyboard' else ('AUTO TEST' if args.mode in ('demo','griptest') else ('CONTROLLER DETECTED' if state else 'CONNECT XBOX CONTROLLER'))
    instructions=('Hold SPACE: enable\nArrows: steer\nW / S: forward / back' if args.mode=='keyboard' else 'Hold LB: enable\nLeft stick: steer\nRT / LT: forward / back')
    mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,left,'ARM 1: steering motors   |   ARM 2: horizontal grip + move\n'+label+'\n'+instructions+'\nMouse left drag: orbit\nRight drag: pan  |  Scroll: zoom\nG: open / close   |   [ / ]: grip target - / +\nC: view   |   ESC: close',f'\n\n\n\n\n\nTime {sim_t:.1f}s\nTravel {(d.xpos[clamp]-start)@axis*1000:+.1f} mm',ctx)
    mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,right,'C3VDv2 sigmoid1\nSIMULATION ONLY',f'\nContacts {nc}',ctx)
    if detail_view==0:
     detail_title='C3VDv2 - SIGMOID1';detail_values=f'Rigid wall / original geometry\nWall force {force:.3f} N\nC: colon / clamp / handle'
    elif detail_view==1:
     detail_title='ARM 2 - CLAMP: '+('CONTACT' if grip_engaged else ('CLOSING / EMPTY' if grip_close else 'OPEN'))
     detail_values=f'Normal {pad_normal:.3f} N   Friction {pad_tangential:.3f} N\nSlide {d.qvel[slide_v]*1000:.1f} mm/s   Target {jaw_target*1000:.2f} mm\nTube force {tube_force:.3f} N   Arm contacts {robot_contacts}'
    else:
     detail_title='ARM 1 - HANDLE + 2 MOTORS';detail_values=f'X: {d.qpos[motor_q[0]]:.2f} rad   Y: {d.qpos[motor_q[1]]:.2f} rad'
    mujoco.mjr_overlay(mujoco.mjtFontScale.mjFONTSCALE_150,mujoco.mjtGridPos.mjGRID_TOPLEFT,handle_view,detail_title,detail_values,ctx)
    if args.screenshot and step_index==0:
     pixels=np.zeros((height,width,3),dtype=np.uint8);mujoco.mjr_readPixels(pixels,None,mujoco.MjrRect(0,0,width,height),ctx);png(Path(args.screenshot),np.ascontiguousarray(pixels[::-1]))
    glfw.swap_buffers(window)
    wr.writerow([f'{sim_t:.3f}',time.monotonic()-t0,int(enable),forward,sx,sy,distance,float((d.xpos[clamp]-start)@axis),x_cmd,y_cmd,*d.site_xpos[tip],nc,force,frame_idx,*d.qpos[q],*d.qpos[sq],*d.qpos[bends],x_cmd,y_cmd,*d.qpos[motor_q],int(grip_close),*d.qpos[jaw_q],int(grip_engaged),d.qpos[slide_q],pad_contacts,pad_normal,pad_tangential,d.qvel[slide_v],jaw_target,tube_force,max_bend,tube_gap,robot_contacts,robot_penetration,float(np.linalg.norm((d.xpos[clamp]-start)-axis*((d.xpos[clamp]-start)@axis))),float(np.arccos(np.clip((np.trace(start_R.T@d.xmat[clamp].reshape(3,3))-1)/2,-1,1)))])
    step_index+=1;sim_t=step_index*.02;delay=t0+sim_t-time.monotonic()
    if delay>0:time.sleep(delay)
  except KeyboardInterrupt:print('Stopped by user',flush=True)
  finally:
   if renderer:renderer.close()
   glfw.make_context_current(window);ctx.free();glfw.destroy_window(window);glfw.terminate()
 print('Saved:',out,flush=True)
if __name__=='__main__':main()
