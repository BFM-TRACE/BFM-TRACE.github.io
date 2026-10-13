// Local gait-target scheduling. Every sample is a target for the learned G1
// encoder + H2 decoder; this module never writes a simulated pose or velocity.
const clamp=(value,min,max)=>Math.max(min,Math.min(max,value));
const wrap=value=>((value%1)+1)%1;
// Keep every consumer (Worker and offline physics checks) on the same bank.
export function plannerEntries(bank) {
  return Object.values(bank.styles).flatMap(style=>Object.values(style.clips)
    .flatMap(entry=>[entry,...(entry.variants || [])]));
}
function quaternionBlend(a,b,t) {
  const sign=a.reduce((sum,value,i)=>sum+value*b[i],0)<0?-1:1;
  const q=a.map((value,i)=>value*(1-t)+sign*b[i]*t), norm=Math.hypot(...q);
  return q.map(value=>value/norm);
}
function rotateHeading(q,yaw) {
  const c=Math.cos(yaw/2),s=Math.sin(yaw/2),[w,x,y,z]=q;
  return [c*w-s*z,c*x-s*y,c*y+s*x,c*z+s*w];
}

export class LocalGaitPlanner {
  constructor(bank, references) {
    this.styles=bank.styles;
    this.stopBlendSeconds=bank.stopBlendSeconds ?? .55;
    this.forwardStopBlendSeconds=bank.forwardStopBlendSeconds ?? this.stopBlendSeconds;
    this.walkStopBlendSeconds=bank.walkStopBlendSeconds ?? .25;
    this.clips={};
    this.directions={};
    this.style='slow_walk';
    this.blendSeconds=.25;
    this.rateSeconds=.3;
    for(const [style,settings] of Object.entries(this.styles)) {
      for(const [direction,entry] of Object.entries(settings.clips)) {
        const group=this.directions[`${style}/${direction}`]=[];
        for(const [variantIndex,variant] of [entry,...(entry.variants || [])].entries()) {
        const key=`${style}/${direction}${variantIndex ? `@${variant.targetSpeed}`:''}`;
        const ref=references[variant.id];
        if(!ref?.q29G1) throw new Error(`Missing native G1 planner reference: ${variant.id}`);
        const loop=variant.recommendedLoop || ref.recommendedLoop;
        if(!loop || loop.endFrame<=loop.startFrame) throw new Error(`Invalid gait period: ${variant.id}`);
        const count=loop.endFrame-loop.startFrame;
        // Align left-knee swing peaks before blending different directions or
        // styles, instead of arbitrarily mixing opposite support-leg phases.
        let peak=loop.startFrame;
        for(let i=peak+1;i<loop.endFrame;i++) if(ref.q29G1[i][9]>ref.q29G1[peak][9]) peak=i;
        let sinYaw=0,cosYaw=0;
        for(let i=loop.startFrame;i<loop.endFrame;i++) {
          const [w,x,y,z]=ref.rootQuat[i],yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
          sinYaw+=Math.sin(yaw);cosYaw+=Math.cos(yaw);
        }
        this.clips[key]={ref,start:loop.startFrame,count,
          phaseOffset:style==='idle'?0:(peak-loop.startFrame)/count,
          yawCenter:Math.atan2(sinYaw,cosYaw),
          period:count/(ref.fps*(variant.playbackRate ?? ref.playbackRate ?? 1)), settings, singleStride:!!loop.phaseAnchor};
        group.push({key,speed:variant.targetSpeed ?? ref.targetSpeed ?? settings.speed});
        }
        group.sort((a,b)=>a.speed-b.speed);
      }
    }
    if(!this.clips['idle/idle']) throw new Error('The planner standing reference is missing.');
    this.reset(0);
  }

  reset(heading) {
    this.heading=heading; this.headingCorrection=0; this.yawRate=0; this.phase=0;
    this.phaseRate=1; this.targetRate=1;
    this.gaitPhase=0; this.gaitRate=1; this.targetGaitRate=1;
    this.lastWasForward=false;
    this.weights={'idle/idle':1}; this.targetWeights={'idle/idle':1};
  }

  setStyle(style) {
    if(!this.styles[style]) throw new Error('Unknown planner movement style.');
    this.style=style;
  }

  speedWeights(direction,speed) {
    const group=this.directions[`${this.style}/${direction}`];
    if(!group) throw new Error(`No ${direction} reference for ${this.style}.`);
    if(group.length===1) return [[group[0].key,1]];
    if(speed<=group[0].speed) return [[group[0].key,1]];
    for(let i=1;i<group.length;i++) if(speed<=group[i].speed) {
      const lo=group[i-1],hi=group[i],t=(speed-lo.speed)/(hi.speed-lo.speed);
      return [[lo.key,1-t],[hi.key,t]];
    }
    return [[group.at(-1).key,1]];
  }

  advance(dt, {vx,vy,yaw}) {
    const speed=Math.hypot(vx,vy), active=this.style!=='idle' && speed>.01;
    if(active) this.lastWasForward=vx>0 && Math.abs(vy)<1e-6;
    // Give fast gaits time to shed momentum, including Run -> Idle changes.
    const running=this.style==='run' || Object.entries(this.weights).some(([key,weight])=>key.startsWith('run/') && weight>.01);
    const walking=this.style==='walk' || Object.entries(this.weights).some(([key,weight])=>key.startsWith('walk/') && weight>.01);
    this.blendSeconds=running?(active?.55:(this.lastWasForward?this.forwardStopBlendSeconds:this.stopBlendSeconds)):
      !active && walking?this.walkStopBlendSeconds:.25;
    this.targetWeights={};
    if(active) {
      const sum=Math.abs(vx)+Math.abs(vy);
      for(const [direction,value] of [['forward',Math.max(vx,0)],['back',Math.max(-vx,0)],['left',Math.max(vy,0)],['right',Math.max(-vy,0)]]) {
        if(!value) continue;
        // Equal joint-target weights under-drive the forward component of
        // this slow right-diagonal gait. Calibrate the target mix; simulated
        // translation still comes exclusively from policy and contact forces.
        const gain=this.style==='slow_walk' && direction==='forward' && vy<-.01 ? 1.25:1;
        for(const [key,weight] of this.speedWeights(direction,speed))
          this.targetWeights[key]=value/sum*gain*weight;
      }
      // Speed knots are separately generated planner references. Blend their
      // stride geometry and cadence instead of speeding up a single clip.
      const entries=Object.entries(this.targetWeights).filter(([,w])=>w>0);
      const weight=entries.reduce((sum,[,w])=>sum+w,0);
      this.targetGaitRate=entries.reduce((sum,[key,w])=>sum+w/this.clips[key].period,0)/weight;
      this.targetRate=entries.every(([key])=>this.clips[key].singleStride)?1:clamp(speed/this.styles[this.style].speed,.65,1.5);
    } else this.targetWeights['idle/idle']=1;
    // Keep advancing gait phase while it fades out. Freezing q while leaving
    // a nonzero walking qdot used to create an inconsistent stop command.
    this.phase+=this.targetRate*dt+(this.phaseRate-this.targetRate)*this.rateSeconds*(1-Math.exp(-dt/this.rateSeconds));
    this.phaseRate=this.targetRate+(this.phaseRate-this.targetRate)*Math.exp(-dt/this.rateSeconds);
    this.gaitPhase+=this.targetGaitRate*dt+(this.gaitRate-this.targetGaitRate)*this.rateSeconds*(1-Math.exp(-dt/this.rateSeconds));
    this.gaitRate=this.targetGaitRate+(this.gaitRate-this.targetGaitRate)*Math.exp(-dt/this.rateSeconds);
    for(const key of new Set([...Object.keys(this.weights),...Object.keys(this.targetWeights)])) {
      const target=this.targetWeights[key] || 0;
      const value=target+((this.weights[key] || 0)-target)*Math.exp(-dt/this.blendSeconds);
      if(value<1e-7 && !target) delete this.weights[key]; else this.weights[key]=value;
    }
    // Facing is independent of translation. A/D changes lateral targets;
    // only Q/E (yaw) changes the commanded heading.
    this.yawRate=yaw; this.heading+=yaw*dt;
  }

  sampleClip(clip,phase,gaitPhase) {
    const {ref,start,count}=clip;
    const position=clip.settings.mode===0?0:wrap((clip.singleStride?gaitPhase:phase/clip.period)+clip.phaseOffset)*count;
    const at=Math.floor(position),a=start+at,b=start+(at+1)%count,t=position-at;
    const quat=quaternionBlend(ref.rootQuat[a],ref.rootQuat[b],t);
    const [w,x,y,z]=quat, yaw=Math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));
    // The sampled style must not override the user's independent facing
    // command. Retain small natural torso oscillations, bound large heading
    // excursions in the open-loop source (notably sideways running).
    const relativeYaw=Math.atan2(Math.sin(yaw-clip.yawCenter),Math.cos(yaw-clip.yawCenter));
    return {q:ref.q29G1[a].map((value,i)=>value+(ref.q29G1[b][i]-value)*t),
      quat:rotateHeading(quat,clamp(relativeYaw,-.08,.08)-yaw)};
  }

  predict(future) {
    const phase=this.phase+this.targetRate*future+(this.phaseRate-this.targetRate)*this.rateSeconds*(1-Math.exp(-future/this.rateSeconds));
    const gaitPhase=this.gaitPhase+this.targetGaitRate*future+(this.gaitRate-this.targetGaitRate)*this.rateSeconds*(1-Math.exp(-future/this.rateSeconds));
    const q=new Array(29).fill(0),orientation=[0,0,0,0];
    let anchor, total=0;
    for(const key of new Set([...Object.keys(this.weights),...Object.keys(this.targetWeights)])) {
      const target=this.targetWeights[key] || 0;
      const weight=target+((this.weights[key] || 0)-target)*Math.exp(-future/this.blendSeconds);
      if(weight<1e-8) continue;
      const sample=this.sampleClip(this.clips[key],phase,gaitPhase);
      for(let i=0;i<29;i++) q[i]+=weight*sample.q[i];
      anchor ||= sample.quat;
      const sign=sample.quat.reduce((sum,value,i)=>sum+value*anchor[i],0)<0?-1:1;
      for(let i=0;i<4;i++) orientation[i]+=weight*sign*sample.quat[i];
      total+=weight;
    }
    const norm=Math.hypot(...orientation);
    return {q:q.map(value=>value/total),quat:rotateHeading(orientation.map(value=>value/norm),this.heading+this.headingCorrection+this.yawRate*future)};
  }

  reference(future) {
    const result=this.predict(future);
    const lo=Math.max(0,future-.001),hi=future+.001;
    const before=this.predict(lo),after=this.predict(hi);
    // Derivative of the exact time-scaled, blended target, including blend
    // velocity and future phase. This also makes stop/start transitions agree.
    return {policyQ:result.q,policyDq:after.q.map((value,i)=>(value-before.q[i])/(hi-lo)),quat:result.quat};
  }
}
