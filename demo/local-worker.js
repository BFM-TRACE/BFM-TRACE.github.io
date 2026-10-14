import {createPhysics} from './local-physics.js';
import {createLocalInference} from './local-inference.js';
import {LocalPolicy} from './local-policy.js';
import {plannerEntries} from './local-planner.js';
import {prepareTrackingReference} from './local-reference.js';

let physics, inference, controller, hidden=false, initialized=false, stopped=false;
let realTimeFactor=1, stepMs=0, previousStepAt=null;
const commands=[];
const sessionId=`browser-${Date.now().toString(36)}-${Math.random().toString(36).slice(2,9)}`;
const progress=message=>postMessage({type:'progress',message});
const reply=(id,result,error)=>postMessage({id,result,error});
const sleep=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function json(relative) {
  const response=await fetch(new URL(relative,import.meta.url));
  if(!response.ok) throw new Error(`Missing browser simulation asset: ${relative}`);
  return response.json();
}
function publish() {
  const state=physics.getState();
  postMessage({type:'state',state:{...state,
    source:'browser_mujoco_policy',session_id:sessionId,timestamp:Date.now()/1000,
    status:controller.status==='fallen'?'fallen':hidden||controller.paused?'paused':controller.status,
    mode:controller.mode,planner_style:controller.plannerStyle,motion:controller.motion,command:{...controller.commandVelocity},
    reference_space:controller.referenceSpace,encoder_branch:controller.encoderBranch,
    motion_complete:controller.mode==='tracking' && controller.complete,
    motion_preparing:controller.motionPreparing,motion_prepare_remaining:controller.startupStepsRemaining/50,
    inference_ms:controller.inferenceMs,step_ms:stepMs,real_time_factor:realTimeFactor,
    control_steps:controller.steps,planner:'local_directional_gait_bank',provider:inference.provider,
  }});
}
async function init() {
  progress('Loading local physics and H2 assets…');
  const config=await json('./local-assets/controller.json');
  config.qposAddresses=config.localQposAddresses;
  config.dofAddresses=config.localDofAddresses;
  const manifest=await json('./local-assets/references/manifest.json');
  const visibleMotions=await json('./assets/motions.json');
  const requested=new Set(visibleMotions.motions.map(m=>m.id));
  const motions={};
  for(const entry of manifest.motions) {
    if(!requested.has(entry.id)) continue;
    const ref=await json(`./local-assets/references/${entry.file}`);
    motions[entry.id]=prepareTrackingReference(ref,entry);
  }
  config.planner.bank=await json('./local-assets/planner-bank/manifest.json');
  for(const entry of plannerEntries(config.planner.bank)) motions[entry.id]=await json(`./local-assets/planner-bank/${entry.file}`);
  physics=await createPhysics();
  inference=await createLocalInference({provider:'wasm',onProgress:({phase,model})=>{
    progress(phase==='runtime'?'Loading browser inference runtime…':phase==='loading'?`Loading H2 ${model} model…`:`H2 ${model} ready`);
  }});
  controller=new LocalPolicy(physics,inference,config,motions);
  controller.reset(visibleMotions.motions[0].id);
  await controller.step();
  initialized=true;
  publish();
  loop();
  return {ready:true,sessionId};
}
async function loop() {
  try {
    while(!stopped) {
      const started=performance.now();
      while(commands.length) {
        const {id,payload}=commands.shift();
        try {reply(id,controller.command(payload));} catch(error) {reply(id,null,error.message);}
      }
      if(!hidden && !controller.paused && controller.status!=='fallen') {
        await controller.step();
        stepMs=performance.now()-started;
        const finished=performance.now();
        const elapsed=previousStepAt===null?Math.max(20,stepMs):finished-previousStepAt;
        realTimeFactor=.95*realTimeFactor+.05*Math.min(1,20/Math.max(20,elapsed));
        previousStepAt=finished;
      } else previousStepAt=null;
      publish();
      // Do not accumulate wall-clock debt: slower devices slow simulation time
      // instead of blocking the UI or sending an ever-growing queue of poses.
      await sleep(hidden||controller.paused||controller.status==='fallen'?100:Math.max(0,20-(performance.now()-started)));
    }
  } catch(error) {
    stopped=true;
    postMessage({type:'fatal',error:error.message});
  }
}
self.onmessage=async ({data:{id,type,payload}})=>{
  if(type==='visibility') {
    hidden=payload.hidden;
    if(controller) controller.command({action:'stop'});
    return;
  }
  if(type==='init') {
    try {reply(id,await init());} catch(error) {stopped=true;reply(id,null,error.message);}
  } else if(type==='command') {
    if(!initialized || stopped) reply(id,null,'The browser simulation is not running. Reload to retry.');
    else commands.push({id,payload});
  }
};
