// H2-native deployment observation contract, ported from SONIC-H2.
// Reference samples are policy targets only; body poses always come from physics.
import {LocalGaitPlanner} from './local-planner.js';
const clamp = (x,a,b) => Math.max(a,Math.min(b,x));
export const conjugate = ([w,x,y,z]) => [w,-x,-y,-z];
export function multiply(a,b) {
  const [w,x,y,z]=a, [v,i,j,k]=b;
  return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v];
}
export function yawOf([w,x,y,z]) {return Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));}
export function yawQuaternion(yaw) {return [Math.cos(yaw/2),0,0,Math.sin(yaw/2)];}
export function orientation6([w,x,y,z]) {
  return [1-2*(y*y+z*z),2*(x*y-w*z),2*(x*y+w*z),1-2*(x*x+z*z),2*(x*z-w*y),2*(y*z+w*x)];
}
export function gravityLocal([w,x,y,z]) {return [2*(w*y-x*z),-2*(y*z+w*x),2*(x*x+y*y)-1];}
function blendQuaternion(a,b,t) {
  const sign = a.reduce((s,v,i)=>s+v*b[i],0)<0 ? -1 : 1;
  const cosine = clamp(a.reduce((s,v,i)=>s+v*b[i]*sign,0),-1,1);
  const angle = Math.acos(cosine), sine = Math.sin(angle);
  const wa = cosine>.9995 ? 1-t : Math.sin((1-t)*angle)/sine;
  const wb = cosine>.9995 ? t : Math.sin(t*angle)/sine;
  const q = a.map((v,i)=>v*wa+sign*b[i]*wb), norm = Math.hypot(...q);
  return q.map(v=>v/norm);
}
function pushHistory(history,value) {
  history.copyWithin(0,value.length); history.set(value,history.length-value.length);
}

export class LocalPolicy {
  constructor(physics,inference,config,motions) {
    config.qposAddresses ??= config.localQposAddresses;
    config.dofAddresses ??= config.localDofAddresses;
    config.defaultAnglesLocal ??= config.qposAddresses.map(address=>config.initialQpos[address]);
    this.physics=physics; this.inference=inference; this.config=config; this.motions=motions;
    this.mode='tracking'; this.motion=Object.keys(motions)[0]; this.time=0; this.paused=false; this.startupStepsRemaining=0;
    this.commandVelocity={vx:0,vy:0,yaw:0}; this.commandAt=performance.now();
    this.history={omega:new Float32Array(30),q:new Float32Array(290),dq:new Float32Array(290),action:new Float32Array(290),gravity:new Float32Array(30)};
    this.lastAction=new Float32Array(29); this.encoderInput=new Float32Array(640); this.decoderInput=new Float32Array(994);
    this.smplInput=new Float32Array(840);
    this.target=new Float64Array(31); this.inferenceMs=0; this.steps=0; this.status='starting';
    this.planner=config.planner.bank ? new LocalGaitPlanner(config.planner.bank,motions) : null;
    this.plannerStyle='slow_walk';
  }

  sample(motion,frame) {
    const ref=this.motions[motion];
    const at=clamp(frame,0,ref.frames.length-1), i=Math.floor(at), next=Math.min(i+1,ref.frames.length-1), t=at-i;
    const a=ref.frames[i], b=ref.frames[next];
    if (ref.referenceSpace==='smpl') {
      const quat=blendQuaternion(a.quat,b.quat,t);
      return {joints:a.joints.map((v,j)=>v+(b.joints[j]-v)*t),wrists:a.wrists.map((v,j)=>v+(b.wrists[j]-v)*t),quat,rootQuat:quat};
    }
    const positions=ref.referenceSpace==='g1'?'policyQ':'q', velocities=ref.referenceSpace==='g1'?'policyDq':'dq';
    return {[positions]:a[positions].map((v,j)=>v+(b[positions][j]-v)*t),[velocities]:a[velocities].map((v,j)=>v+(b[velocities][j]-v)*t),quat:blendQuaternion(a.quat,b.quat,t)};
  }

  get referenceSpace() {return this.mode==='planner'?'g1':this.motions[this.motion].referenceSpace || 'h2';}
  get encoderBranch() {return this.referenceSpace==='g1'?'g1_selector_3':this.referenceSpace==='smpl'?'smpl_selector_2':'h2_native';}
  get motionPreparing() {return this.mode==='tracking' && this.startupStepsRemaining>0;}

  reset(motion=this.motion) {
    this.motion=motion; this.time=0; this.paused=false; this.steps=0;
    this.complete=false;
    this.commandVelocity={vx:0,vy:0,yaw:0}; this.commandAt=performance.now();
    const ref=this.motions[motion];
    this.startupStepsRemaining=this.mode==='tracking'?Math.round((ref.startupHoldSeconds || 0)*50):0;
    this.physics.reset({qpos:this.mode==='planner' || ref.referenceSpace!=='h2'?this.config.initialQpos:ref.initialQpos || this.config.initialQpos});
    this.lastAction.fill(0);
    this.headingOffset=yawOf(this.physics.data.qpos.subarray(3,7))-yawOf(ref.frames[0].quat);
    this.plannerHeading=yawOf(this.physics.data.qpos.subarray(3,7)); this.movementHeading=this.plannerHeading;
    this.plannerPhase=0; this.plannerBlend=0;
    this.planner?.reset(this.plannerHeading);
    const observation=this.observe();
    for(const [name,history] of Object.entries(this.history)) {
      const value=name==='action'?this.lastAction:observation[name];
      for(let i=0;i<10;i++) history.set(value,i*value.length);
    }
    this.status='running';
  }

  observe() {
    const d=this.physics.data, c=this.config;
    return {
      omega:Array.from(d.qvel.subarray(3,6)),
      q:c.policyToLocal.map((local,i)=>d.qpos[c.qposAddresses[local]]-c.defaultAnglesPolicy[i]),
      dq:c.policyToLocal.map(local=>d.qvel[c.dofAddresses[local]]),
      gravity:gravityLocal(d.qpos.subarray(3,7)),
    };
  }

  command(command) {
    if (command.motion && !this.motions[command.motion]) throw new Error('Unknown local motion target.');
    if (command.mode && !['tracking','planner'].includes(command.mode)) throw new Error('Unknown controller mode.');
    if (command.plannerStyle && !this.planner?.styles[command.plannerStyle]) throw new Error('Unknown planner movement style.');
    for(const key of ['vx','vy','yaw']) if(key in command) {
      const value=Number(command[key]);
      if(!Number.isFinite(value) || Math.abs(value)>(key==='yaw'?1:7.5)) throw new Error('Invalid movement command.');
    }
    const changed=command.mode && command.mode!==this.mode;
    if (command.mode) this.mode=command.mode;
    if (changed || command.action==='reset' || (command.motion && command.motion!==this.motion)) this.reset(command.motion || this.motion);
    if(command.plannerStyle) {
      this.planner.setStyle(command.plannerStyle); this.plannerStyle=command.plannerStyle;
      this.commandVelocity={vx:0,vy:0,yaw:0};
    }
    if (command.action==='pause') this.paused=true;
    if (command.action==='play' || command.action==='start') {
      if(this.complete && this.mode==='tracking') this.reset();
      this.paused=false;
    }
    if (command.action==='stop') this.commandVelocity={vx:0,vy:0,yaw:0};
    if (['vx','vy','yaw'].some(k=>k in command)) {
      const vx=Number(command.vx || 0),vy=Number(command.vy || 0);
      const limits=this.planner?.styles[this.plannerStyle].speedLimits;
      const maximum=limits && (vx<0 || Math.abs(vy)>1e-6 ? limits.other : limits.forward);
      if(maximum!==undefined && Math.hypot(vx,vy)>maximum+1e-8) throw new Error(`This direction supports speed commands up to ${maximum} m/s.`);
      for (const key of ['vx','vy','yaw']) {
        const v=Number(command[key] || 0);
        this.commandVelocity[key]=v;
      }
      this.commandAt=performance.now();
    }
    return {accepted:true};
  }

  reference(future) {
    if(this.mode==='tracking') {
      const sample=this.sample(this.motion,this.motionPreparing?0:(this.time+future)*(this.motions[this.motion].fps || 50));
      if(this.motionPreparing || this.complete || this.time+future >= (this.motions[this.motion].frames.length-1)/this.motions[this.motion].fps) (sample.policyDq || sample.dq)?.fill(0);
      sample.quat=multiply(yawQuaternion(this.headingOffset),sample.quat);
      return sample;
    }
    return this.plannerReference(future);
  }

  plannerReference(future) {
    if(!this.planner) throw new Error('Local planner references are missing.');
    return this.planner.reference(future);
  }

  async step() {
    if (this.paused || this.status==='fallen') return;
    const start=performance.now(), c=this.config, d=this.physics.data;
    if (start-this.commandAt>1000) this.commandVelocity={vx:0,vy:0,yaw:0};
    if (this.mode==='planner') {
      if(!this.planner) throw new Error('The local planner bank is not loaded.');
      this.planner.advance(.02,this.commandVelocity);
      this.plannerHeading=this.planner.heading;
      const headingError=this.plannerHeading-yawOf(d.qpos.subarray(3,7));
      this.planner.headingCorrection=clamp(.5*Math.atan2(Math.sin(headingError),Math.cos(headingError)),-.2,.2);
    }
    const observation=this.observe();
    for (const [name,history] of Object.entries(this.history)) pushHistory(history,name==='action'?this.lastAction:observation[name]);
    const baseInverse=conjugate(d.qpos.subarray(3,7));
    const smpl=this.referenceSpace==='smpl';
    for (let f=0;f<10;f++) {
      const ref=this.reference(f*(smpl?.02:.1));
      if (smpl) {
        const inverse=conjugate(ref.rootQuat);
        for(let j=0;j<24;j++) {
          const point=ref.joints.slice(j*3,j*3+3);
          const rotated=multiply(multiply(inverse,[0,...point]),ref.rootQuat).slice(1);
          this.smplInput.set(rotated,f*72+j*3);
        }
        this.smplInput.set(orientation6(multiply(baseInverse,ref.quat)),720+f*6);
        this.smplInput.set(ref.wrists,780+f*6);
        continue;
      }
      for(let i=0;i<29;i++) {
        const local=c.policyToLocal[i];
        this.encoderInput[f*29+i]=ref.policyQ ? ref.policyQ[i] : ref.q[local];
        this.encoderInput[290+f*29+i]=ref.policyDq ? ref.policyDq[i] : ref.dq[local];
      }
      this.encoderInput.set(orientation6(multiply(baseInverse,ref.quat)),580+f*6);
    }
    const tokens=await (smpl?this.inference.encodeSMPL(this.smplInput):this.referenceSpace==='g1'?this.inference.encodePlanner(this.encoderInput):this.inference.encode(this.encoderInput));
    this.decoderInput.set(tokens,0);
    let offset=64;
    for(const history of Object.values(this.history)) {this.decoderInput.set(history,offset);offset+=history.length;}
    const action=await this.inference.decode(this.decoderInput);
    this.inferenceMs=performance.now()-start;
    this.target.set(c.defaultAnglesLocal);
    for(let i=0;i<29;i++) {
      if(!Number.isFinite(action[i])) throw new Error('The policy produced a non-finite action.');
      this.lastAction[i]=clamp(action[i],-6,6);
      this.target[c.policyToLocal[i]]=c.defaultAnglesPolicy[i]+c.actionScalePolicy[i]*this.lastAction[i];
    }
    this.physics.stepPD(this.target,c.kp,c.kd,20);
    this.steps++;
    if (this.mode==='tracking') {
      if (this.motionPreparing) this.startupStepsRemaining--;
      else {
        const ref=this.motions[this.motion], end=(ref.frames.length-1)/(ref.fps || 50);
        this.time=Math.min(this.time+.02,end); this.complete=this.time>=end;
      }
    }
    if(!Array.from(d.qpos).every(Number.isFinite)) throw new Error('The physics state is not finite. Reset the simulation.');
    if(d.qpos[2]<.28) {this.status='fallen';this.commandVelocity={vx:0,vy:0,yaw:0};}
  }
}
