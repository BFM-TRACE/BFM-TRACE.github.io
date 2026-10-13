// Reproducible physical tracking checks; this never edits policy or reference data.
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir,readdir} from 'node:fs/promises';
import {resolve,dirname,join} from 'node:path';
import {fileURLToPath} from 'node:url';
import {createHash} from 'node:crypto';
import {createPhysics} from '../demo/local-physics.js';
import {createLocalInference} from '../demo/local-inference.js';
import {LocalPolicy,yawOf} from '../demo/local-policy.js';
const home=fileURLToPath(new URL('../',import.meta.url));
const args=process.argv.slice(2),opts={};
for(let i=0;i<args.length;i++){if(args[i]==='--help'){console.log('node homepage/scripts/check-dynamic-motions.mjs --candidates DIR [--ids id,id] [--output FILE] [--hold-seconds 2]\nReads DIR/references/*.json (or DIR/*.json), runs unchanged native tracking from the production default reset, and records actual contact, flight, foot motion and terminal stability.');process.exit(0);}assert.ok(['--candidates','--ids','--output','--hold-seconds'].includes(args[i]),'Unknown option '+args[i]);opts[args[i]]=args[++i];}
assert.ok(opts['--candidates'],'--candidates DIR is required');
const directory=resolve(opts['--candidates']),output=resolve(opts['--output']||join(directory,'../dynamic-motion-qa/dynamic-motions.json')),holdSeconds=Number(opts['--hold-seconds']||2);
assert.ok(holdSeconds>=2&&Number.isInteger(holdSeconds*50),'Hold must be at least 2 seconds in .02s intervals');
const limits={minimumRootHeight:.28,maximumNonFootGroundContactSeconds:.04,maximumSustainedTorsoTiltDeg:85,maximumSevereTiltSeconds:.2,terminalMinimumRootHeight:.5,terminalMaximumTorsoTiltDeg:45,terminalMaximumHorizontalSpeedRMS:.25,terminalMaximumVerticalSpeedRMS:.2,minimumKickPoseSeconds:.06,minimumKickFootClearance:.25,minimumKickHorizontalReach:.35,minimumKickLegExcursionRad:.5,minimumFlightMeasurementTimeSeconds:.5,minimumFlightSeconds:.06,minimumFlightFootClearance:.015,minimumFlightForwardDisplacement:.12,minimumLongJumpForwardDisplacement:.35,maximumStandingJumpGroundedFootAdvance:.15,minimumPreFlightDoubleSupportSeconds:.06};
const report={testedAt:new Date().toISOString(),status:'running',scope:'Native H2 encoder (selector0) + H2 decoder + original MuJoCo WASM physics. No state/reference injection during rollouts; no changes to gains, clipping, targets or timing.',host:{runtime:'Node WebAssembly',node:process.version,platform:process.platform,arch:process.arch},protocol:{policyHz:50,physicsHz:1000,holdSeconds,initialization:'Unchanged LocalPolicy.reset with controller.initialQpos/default qvel; no motion-specific initial state',fullReference:true},limits,limitations:['Bounded simulation QA, not a hardware safety guarantee or paper benchmark.','Measurements are at 50Hz policy boundaries. Contact constraints describe the last 1ms physics substep; current kinematics are refreshed without changing solver state.','Flight requires no active floor contacts and both collision-mesh sole minima above the floor. Reference labels or root trajectories alone cannot pass.','Default reset starts slightly above contact; the first .5s cannot count as flight. Standing takeoff is measured from grounded foot advance and double support, not root translation during crouch/push-off.','No browser rendering or scheduling is tested.'],sources:{},cases:[],errors:[]};
const started=performance.now(),sha=b=>createHash('sha256').update(b).digest('hex'),sourcePaths=new Map();
async function source(path,label=path){const b=new Uint8Array(await readFile(path));report.sources[label]={bytes:b.length,sha256:sha(b)};sourcePaths.set(label,path);return b;}
const bytes=p=>source(resolve(home,p),p),json=async p=>JSON.parse(new TextDecoder().decode(await bytes(p)));
const mean=a=>a.length?a.reduce((s,v)=>s+v,0)/a.length:null,rms=a=>a.length?Math.sqrt(mean(a.map(x=>x*x))):null;
const stats=a=>a.length?{min:Math.min(...a),max:Math.max(...a),mean:mean(a),rms:rms(a),peakToPeak:Math.max(...a)-Math.min(...a)}:null;
const tilt=q=>Math.acos(Math.max(-1,Math.min(1,1-2*(q[1]**2+q[2]**2))))*180/Math.PI;
const pitch=q=>Math.asin(Math.max(-1,Math.min(1,2*(q[0]*q[2]-q[3]*q[1]))))*180/Math.PI;
async function save(){await mkdir(dirname(output),{recursive:true});await writeFile(output,JSON.stringify(report,null,2)+'\n');}
let physics,inference,config,policy,footBodies,footGeoms,floorGeom,torsoBody,pelvisBody;
function footClearance(geom){const m=physics.model,d=physics.data,mesh=m.geom_dataid[geom],a=m.mesh_vertadr[mesh],n=m.mesh_vertnum[mesh],z=d.geom_xpos[geom*3+2],mat=d.geom_xmat.subarray(geom*9,geom*9+9);let min=Infinity;for(let i=a;i<a+n;i++){const v=i*3;min=Math.min(min,z+mat[6]*m.mesh_vert[v]+mat[7]*m.mesh_vert[v+1]+mat[8]*m.mesh_vert[v+2]);}return min;}
function snapshot(stage){
 const {mujoco:mj,model:m,data:d}=physics;mj.mj_kinematics(m,d);
 for(const key of ['qpos','qvel','ctrl','xpos','xquat'])assert.ok(d[key].every(Number.isFinite),key+' contains non-finite state');
 assert.ok(policy.lastAction.every(Number.isFinite),'Non-finite policy action');
 const ground=[];for(let i=0;i<d.ncon;i++){const c=d.contact.get(i);try{if(c.exclude===0&&c.efc_address>=0&&(c.geom1===floorGeom||c.geom2===floorGeom)){const geom=c.geom1===floorGeom?c.geom2:c.geom1;ground.push({geom,body:m.geom_bodyid[geom],distance:c.dist});}}finally{c.delete();}}
 const q=d.qpos.subarray(3,7),heading=yawOf(q),position=Array.from(d.qpos.subarray(0,3));
 const feet=Object.fromEntries(['left','right'].map(side=>{const b=footBodies[side],p=Array.from(d.xpos.subarray(b*3,b*3+3)),dx=p[0]-position[0],dy=p[1]-position[1];return[side,{position:p,clearance:footClearance(footGeoms[side]),rootRelativeHorizontalReach:Math.hypot(dx,dy),rootRelativeForward:dx*Math.cos(heading)+dy*Math.sin(heading),contact:ground.some(c=>c.body===b)}];}));
 const actualQ=config.localQposAddresses.map(a=>d.qpos[a]),ref=policy.reference(0),error=actualQ.map((v,i)=>v-ref.q[i]);
 return{stage,time:d.time,referenceTime:policy.time,position,velocity:Array.from(d.qvel.subarray(0,3)),horizontalSpeed:Math.hypot(d.qvel[0],d.qvel[1]),rootTiltDeg:tilt(q),rootPitchDeg:pitch(q),torsoTiltDeg:tilt(d.xquat.subarray(torsoBody*4,torsoBody*4+4)),rootQuaternion:Array.from(q),feet,groundContacts:ground,nonFootGroundContact:ground.some(c=>!Object.values(footBodies).includes(c.body)),q31:actualQ,trackingJointRMS:rms(error),referenceComplete:policy.complete};
}
function segments(samples,predicate){const out=[];let run=[];for(const s of samples){if(predicate(s))run.push(s);else if(run.length){out.push(run);run=[];}}if(run.length)out.push(run);return out;}
function assess(test){
 const a=test.samples,initial=a[0],action=a.filter(s=>s.stage==='motion'),tail=a.filter(s=>s.stage==='hold').slice(-50);
 test.minRootHeight=Math.min(...a.map(s=>s.position[2]));test.maxRootTiltDeg=Math.max(...a.map(s=>s.rootTiltDeg));test.maxTorsoTiltDeg=Math.max(...a.map(s=>s.torsoTiltDeg));test.maxNonFootGroundContactSeconds=Math.max(0,...segments(a,s=>s.nonFootGroundContact).map(x=>x.length/50));
 test.terminal={samples:tail.length,horizontalSpeed:rms(tail.map(s=>s.horizontalSpeed)),verticalSpeed:rms(tail.map(s=>s.velocity[2])),minRootHeight:tail.length?Math.min(...tail.map(s=>s.position[2])):null,maxTorsoTiltDeg:tail.length?Math.max(...tail.map(s=>s.torsoTiltDeg)):null,groundContactFraction:mean(tail.map(s=>s.groundContacts.length>0?1:0))};
 test.jointExcursions=Object.fromEntries(config.localNames.map((name,i)=>[name,stats(action.map(s=>s.q31[i]))?.peakToPeak??0]));test.trackingJointRMS=rms(action.map(s=>s.trackingJointRMS));
 test.feet=Object.fromEntries(['left','right'].map(side=>[side,{maximumClearance:Math.max(...action.map(s=>s.feet[side].clearance)),maximumRootRelativeHorizontalReach:Math.max(...action.map(s=>s.feet[side].rootRelativeHorizontalReach)),maximumRootRelativeForward:Math.max(...action.map(s=>s.feet[side].rootRelativeForward)),heightExcursion:stats(action.map(s=>s.feet[side].position[2]))?.peakToPeak,maximumContiguousKickPoseSeconds:Math.max(0,...segments(action,s=>s.feet[side].clearance>=limits.minimumKickFootClearance&&s.feet[side].rootRelativeHorizontalReach>=limits.minimumKickHorizontalReach).map(a=>a.length/50)),legExcursion:Math.max(test.jointExcursions[side+'_hip_pitch_joint']||0,test.jointExcursions[side+'_knee_joint']||0)}]));
 const yaw=yawOf(initial.rootQuaternion),forward=s=>(s.position[0]-initial.position[0])*Math.cos(yaw)+(s.position[1]-initial.position[1])*Math.sin(yaw);
 test.actualDisplacement=a.at(-1).position.map((v,i)=>v-initial.position[i]);test.actualForwardDisplacement=forward(a.at(-1));
 let grounded=0;for(const s of action){if(s.groundContacts.length)grounded+=1/50;s.flightEligible=grounded>=.1&&s.time>=limits.minimumFlightMeasurementTimeSeconds&&s.groundContacts.length===0&&Math.min(s.feet.left.clearance,s.feet.right.clearance)>limits.minimumFlightFootClearance;}
 test.flights=segments(action,s=>s.flightEligible).map(f=>{
  const prior=action[Math.max(0,action.indexOf(f[0])-1)],landing=a.find(s=>s.time>f.at(-1).time&&s.groundContacts.length>0);
  const before=action.filter(s=>s.time<f[0].time),groundedFootAdvance=[];
  for(const s of before)for(const side of ['left','right'])if(s.feet[side].contact){const p=s.feet[side].position,b=initial.feet[side].position;groundedFootAdvance.push((p[0]-b[0])*Math.cos(yaw)+(p[1]-b[1])*Math.sin(yaw));}
  const preFlightDoubleSupportSeconds=before.filter(s=>s.time>=f[0].time-.3&&s.feet.left.contact&&s.feet.right.contact).length/50;
  return{start:f[0].time,end:f.at(-1).time,duration:f.length/50,forwardDisplacement:forward(f.at(-1))-forward(prior),approachRootForwardDisplacement:forward(prior),maximumGroundedFootForwardAdvanceBeforeFlight:Math.max(0,...groundedFootAdvance),preFlightDoubleSupportSeconds,rootRise:Math.max(...f.map(s=>s.position[2]))-prior.position[2],minimumFootClearance:Math.min(...f.flatMap(s=>[s.feet.left.clearance,s.feet.right.clearance])),landingTime:landing?.time??null};});
 const fail=(condition,message)=>{if(condition)test.failures.push(message);};
 fail(!test.completedReference,'Did not complete full reference');fail(!test.completedHold,'Did not complete terminal hold');fail(test.minRootHeight<limits.minimumRootHeight,'Root below fall threshold');fail(test.maxNonFootGroundContactSeconds>limits.maximumNonFootGroundContactSeconds,'Sustained non-foot contact with floor');
 fail(segments(a,s=>s.torsoTiltDeg>limits.maximumSustainedTorsoTiltDeg).some(x=>x.length/50>limits.maximumSevereTiltSeconds),'Sustained near-horizontal/inverted torso');
 fail(tail.length<49,'Incomplete final one-second stability window');if(tail.length){fail(test.terminal.horizontalSpeed>=limits.terminalMaximumHorizontalSpeedRMS,'Terminal horizontal motion did not settle');fail(test.terminal.verticalSpeed>=limits.terminalMaximumVerticalSpeedRMS,'Terminal vertical motion did not settle');fail(test.terminal.minRootHeight<limits.terminalMinimumRootHeight,'Terminal root posture too low');fail(test.terminal.maxTorsoTiltDeg>limits.terminalMaximumTorsoTiltDeg,'Terminal torso not upright');fail(test.terminal.groundContactFraction<.8,'Terminal state not supported on ground');}
 if(test.kind==='kick'){
  test.qualifyingFeet=Object.entries(test.feet).filter(([side,f])=>(!test.kickSide||side===test.kickSide)&&f.maximumContiguousKickPoseSeconds>=limits.minimumKickPoseSeconds&&f.legExcursion>=limits.minimumKickLegExcursionRad).map(([s])=>s);
  test.kickSupport=Object.fromEntries(test.qualifyingFeet.map(side=>{const other=side==='left'?'right':'left',poses=action.filter(s=>s.feet[side].clearance>=limits.minimumKickFootClearance&&s.feet[side].rootRelativeHorizontalReach>=limits.minimumKickHorizontalReach);return[side,{qualifyingSamples:poses.length,oppositeFootGroundContactFraction:mean(poses.map(s=>s.feet[other].contact?1:0)),maximumTorsoTiltDeg:Math.max(...poses.map(s=>s.torsoTiltDeg))}];}));
  fail(!test.qualifyingFeet.length,'No actual kick with sufficient foot clearance, reach and leg excursion');
 }else if(test.kind==='long_jump'){
  test.qualifyingFlights=test.flights.filter(f=>f.duration>=limits.minimumFlightSeconds&&f.forwardDisplacement>=limits.minimumFlightForwardDisplacement&&f.maximumGroundedFootForwardAdvanceBeforeFlight<=limits.maximumStandingJumpGroundedFootAdvance&&f.preFlightDoubleSupportSeconds>=limits.minimumPreFlightDoubleSupportSeconds&&f.landingTime!==null);
  fail(!test.qualifyingFlights.length,'No supported standing takeoff, airborne forward travel and landing sequence');fail(test.actualForwardDisplacement<limits.minimumLongJumpForwardDisplacement,'Insufficient actual forward jump displacement');
 }else if(test.kind==='jump')fail(!test.flights.some(f=>f.duration>=limits.minimumFlightSeconds&&f.landingTime!==null),'No actual flight and landing');
 else fail(Math.max(...Object.values(test.jointExcursions))<.5,'No substantial actual joint motion');
 test.status=test.failures.length?'fail':'pass';
}
try{
 for(const p of ['scripts/check-dynamic-motions.mjs','demo/local-policy.js','demo/local-planner.js','demo/local-physics.js','demo/local-inference.js','demo/vendor/mujoco/mujoco.js','demo/vendor/mujoco/mujoco.wasm','demo/vendor/ort/ort.wasm.min.mjs','demo/vendor/ort/ort-wasm-simd-threaded.mjs','demo/vendor/ort/ort-wasm-simd-threaded.wasm'])await bytes(p);
 config=await json('demo/local-assets/controller.json');delete config.planner.bank;assert.equal(config.policyHz,50);
 const pm=await json('demo/local-assets/physics/manifest.json');physics=await createPhysics({manifest:pm,config:await json('demo/local-assets/physics/physics-config.json'),files:await Promise.all(pm.files.map(async f=>({path:f.path,data:await bytes('demo/local-assets/physics/'+(f.url||f.path))}))),engineOptions:{wasmBinary:await bytes('demo/vendor/mujoco/mujoco.wasm')}});
 inference=await createLocalInference({models:{encoder:await bytes('demo/local-assets/models/encoder_native.onnx'),decoder:await bytes('demo/local-assets/models/decoder.onnx')}});report.engines={mujoco:physics.mujoco.mj_versionString(),ort:inference.runtimeVersion,provider:inference.provider};
 footBodies=Object.fromEntries(['left','right'].map(s=>[s,physics.bodyNames.indexOf(s+'_ankle_pitch_link')]));torsoBody=physics.bodyNames.indexOf('torso_link');pelvisBody=physics.bodyNames.indexOf('pelvis');
 floorGeom=physics.mujoco.mj_name2id(physics.model,physics.mujoco.mjtObj.mjOBJ_GEOM.value,'floor');assert.ok(floorGeom>=0&&torsoBody>=0&&Object.values(footBodies).every(i=>i>=0));
 footGeoms=Object.fromEntries(Object.entries(footBodies).map(([s,b])=>[s,Array.from({length:physics.model.ngeom},(_,i)=>i).find(i=>physics.model.geom_bodyid[i]===b&&physics.model.geom_type[i]===7&&(physics.model.geom_contype[i]||physics.model.geom_conaffinity[i]))]));assert.ok(Object.values(footGeoms).every(Number.isInteger));report.measurementMapping={footBodies,footGeoms,floorGeom,torsoBody};
 let refDir=join(directory,'references');try{await readdir(refDir);}catch{refDir=directory;}
 const selected=opts['--ids']?new Set(opts['--ids'].split(',')):null;let catalog={};for(const file of [join(directory,'manifest.json'),join(refDir,'manifest.json')]){try{const cm=JSON.parse(new TextDecoder().decode(await source(file)));catalog=Object.fromEntries((cm.motions||[]).map(e=>[e.id,e]));break;}catch{}}
 for(const name of (await readdir(refDir)).filter(f=>f.endsWith('.json')&&f!=='manifest.json').sort()){
  const path=join(refDir,name),data=await source(path),ref=JSON.parse(new TextDecoder().decode(data));if(!ref.rootQuat||!ref.q31||!ref.qd31||selected&&!selected.has(ref.id))continue;
  assert.equal(ref.fps,50,'Reference fps must match production tracking');assert.equal(ref.q31.length,ref.rootQuat.length);assert.equal(ref.qd31.length,ref.rootQuat.length);if(ref.initialQpos)assert.deepEqual(ref.initialQpos,config.initialQpos,'Candidate changes production reset state');
  const info=catalog[ref.id]||{},label=[info.kind,info.category,ref.id].join(' '),kind=/kick/i.test(label)?'kick':/long.?jump|broad.?jump|jump.*forward|standing.*jump|jump.*standing/i.test(label)?'long_jump':/jump/i.test(label)?'jump':'dynamic';
  const motion={...ref,frames:ref.rootQuat.map((quat,i)=>({quat,q:ref.q31[i],dq:ref.qd31[i]}))};policy=new LocalPolicy(physics,inference,config,{[ref.id]:motion});policy.reset(ref.id);
  const test={id:ref.id,kind,kickSide:info.kickSide||null,source:path,sourceSHA256:sha(data),provenance:ref.provenance||ref.source||null,referenceFrames:motion.frames.length,referenceSeconds:(motion.frames.length-1)/50,completedReference:false,completedHold:false,failures:[],samples:[]};report.cases.push(test);test.samples.push(snapshot('initial'));const begin=performance.now();
  try{
   const step=async stage=>{const previous=policy.steps;await policy.step();assert.equal(policy.steps,previous+1,'Policy did not advance');const s=snapshot(stage);test.samples.push(s);assert.notEqual(policy.status,'fallen','Controller detected a fall');assert.ok(s.position[2]>=limits.minimumRootHeight,'Root below fall height');};
   while(!policy.complete){assert.ok(policy.steps<motion.frames.length+3,'Motion never completed');await step('motion');}
   test.completedReference=true;test.completionTime=physics.data.time;assert.equal(policy.time,test.referenceSeconds);
   for(let i=0;i<holdSeconds*50;i++){await step('hold');assert.ok(policy.complete);assert.equal(policy.time,test.referenceSeconds,'Final frame hold restarted or advanced');}test.completedHold=true;test.actualHoldSeconds=physics.data.time-test.completionTime;
  }catch(e){test.failures.push(e.message);}test.wallSeconds=(performance.now()-begin)/1000;assess(test);
  console.log(JSON.stringify({id:test.id,kind:test.kind,status:test.status,seconds:test.samples.at(-1).time,rootMin:test.minRootHeight,forward:test.actualForwardDisplacement,feet:test.feet,flights:test.flights,terminal:test.terminal,failures:test.failures}));await save();
 }
 assert.ok(report.cases.length,'No candidate references matched');report.passed=report.cases.filter(t=>t.status==='pass').map(t=>t.id);report.status=report.cases.every(t=>t.status==='pass')?'pass':'completed_with_failures';
}catch(e){report.status='error';report.errors.push(e.stack||e.message);console.error(e.stack||e.message);}finally{
 try{physics?.dispose();await inference?.release();}catch(e){report.status='error';report.errors.push('Cleanup: '+e.message);}
 report.sourceChanges=[];
 for(const[label,path]of sourcePaths){try{const b=await readFile(path),actualSHA256=sha(b),initial=report.sources[label];if(actualSHA256!==initial.sha256||b.length!==initial.bytes)report.sourceChanges.push({source:label,initialSHA256:initial.sha256,finalSHA256:actualSHA256,finalBytes:b.length});}catch(e){report.sourceChanges.push({source:label,error:e.message});}}
 if(report.sourceChanges.length){report.status='error';report.errors.push('Source files changed or became unavailable during QA; rerun required.');}
 report.wallSeconds=(performance.now()-started)/1000;report.completedAt=new Date().toISOString();await save();
}
console.log('DYNAMIC '+output+': '+report.status);if(report.status!=='pass')process.exitCode=1;
