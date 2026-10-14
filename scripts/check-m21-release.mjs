// Same released ONNX, reference conversion and MuJoCo WASM used by the browser.
import assert from 'node:assert/strict';
import {readFile, writeFile, mkdir} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {createPhysics} from '../demo/local-physics.js';
import {createLocalInference} from '../demo/local-inference.js';
import {LocalPolicy} from '../demo/local-policy.js';
import {prepareTrackingReference} from '../demo/local-reference.js';
import {plannerEntries} from '../demo/local-planner.js';
const root=new URL('../',import.meta.url);
const bytes=async p=>new Uint8Array(await readFile(new URL(p,root)));
const json=async p=>JSON.parse(await readFile(new URL(p,root),'utf8'));
const digest=b=>createHash('sha256').update(b).digest('hex');
const report={testedAt:new Date().toISOString(),scope:'Source integrity and complete 38-motion closed-loop execution using production browser WASM; finite bounded tests, not hardware validation.',motions:[],planner:[]};
const plannerOnly=process.argv.includes('--planner-only');
const trackingOnly=process.argv.includes('--tracking-only');
const selected=process.argv.find(x=>x.startsWith('--motion='))?.slice(9);
const holdOverride=process.argv.find(x=>x.startsWith('--hold='))?.slice(7);
if(plannerOnly) report.motions=(await json('qa/generated/m21-release.json')).motions;
const config=await json('demo/local-assets/controller.json');
const catalog=await json('demo/local-assets/references/manifest.json');
const inventory=await json('demo/local-assets/source-inventory.json');
assert.equal(catalog.motions.length,38);
const models=await json('demo/local-assets/models/manifest.json');
for(const m of Object.values(models.models)) assert.equal(digest(await bytes('demo/local-assets/models/'+m.file)),m.sha256);
report.modelHashes=Object.fromEntries(Object.entries(models.models).map(([k,v])=>[k,v.sha256]));
report.sourceRevision=inventory.sourceRevision;
const motions={};
for(const entry of catalog.motions) {
  const data=await bytes('demo/local-assets/references/'+entry.file);
  assert.equal(digest(data),entry.sha256);
  motions[entry.id]=prepareTrackingReference(JSON.parse(new TextDecoder().decode(data)),entry);
  if(holdOverride!==undefined) motions[entry.id].startupHoldSeconds=Number(holdOverride);
}
config.planner.bank=await json('demo/local-assets/planner-bank/manifest.json');
for(const entry of plannerEntries(config.planner.bank)) motions[entry.id]=await json('demo/local-assets/planner-bank/'+entry.file);
const pm=await json('demo/local-assets/physics/manifest.json');
const physics=await createPhysics({manifest:pm,config:await json('demo/local-assets/physics/physics-config.json'),
  files:await Promise.all(pm.files.map(async f=>({path:f.path,data:await bytes('demo/local-assets/physics/'+(f.url||f.path))}))),
  engineOptions:{wasmBinary:await bytes('demo/vendor/mujoco/mujoco.wasm')}});
const inference=await createLocalInference({models:{encoderFull:await bytes('demo/local-assets/models/encoder_full.onnx'),decoder:await bytes('demo/local-assets/models/decoder.onnx')}});
const policy=new LocalPolicy(physics,inference,config,motions);
async function step() {
  policy.command({vx:0,vy:0,yaw:0});
  await policy.step();
  assert.ok(physics.data.qpos.every(Number.isFinite));
  assert.ok(physics.data.qvel.every(Number.isFinite));
  assert.ok(policy.lastAction.every(Number.isFinite));
}
try {
  for(const entry of catalog.motions.filter(e=>!plannerOnly&&(!selected||e.id===selected))) {
    policy.mode='tracking';policy.reset(entry.id);
    const test={id:entry.id,referenceSpace:policy.referenceSpace,encoderBranch:policy.encoderBranch,frames:entry.frames,minRootHeight:physics.data.qpos[2],status:'pass'};
    const start=performance.now();
    try {
      const budget=entry.frames+Math.round(motions[entry.id].startupHoldSeconds*50)+102;
      while(!policy.complete && policy.steps<=budget) {
        await step();
        test.minRootHeight=Math.min(test.minRootHeight,physics.data.qpos[2]);
        if(policy.status==='fallen') throw Error(`Fall detected at ${policy.time.toFixed(2)} reference seconds`);
      }
      assert.ok(policy.complete,'Motion did not complete');
      test.completedReference=true;
      for(let i=0;i<100;i++) {await step();if(policy.status==='fallen') throw Error('Fall during terminal hold');}
      test.finalRootHeight=physics.data.qpos[2];
    } catch(e) {test.status='fail';test.error=e.message;}
    test.wallSeconds=(performance.now()-start)/1000;test.controlSteps=policy.steps;
    report.motions.push(test);console.log(JSON.stringify(test));
  }
  // The bank's source planner is byte-identical, but the H2 policy changed.
  for(const [style,vx,vy,yaw] of (trackingOnly?[]:[['idle',0,0,0],['slow_walk',.4,0,0],['slow_walk',0,.4,0],['walk',.9,0,0],['run',2.5,0,0],['walk',0,0,.3]])) {
    policy.mode='planner';policy.reset(catalog.motions[0].id);policy.command({plannerStyle:style});
    const test={style,vx,vy,yaw,status:'pass'}; const initial=Array.from(physics.data.qpos.slice(0,3));
    try {
      for(let i=0;i<600;i++) {policy.command({vx:i<400?vx:0,vy:i<400?vy:0,yaw:i<400?yaw:0});await policy.step();assert.ok(physics.data.qpos.every(Number.isFinite));assert.notEqual(policy.status,'fallen');}
      test.displacement=Array.from(physics.data.qpos.slice(0,3),(v,i)=>v-initial[i]);test.finalRootHeight=physics.data.qpos[2];
    } catch(e) {test.status='fail';test.error=e.message;}
    report.planner.push(test);console.log(JSON.stringify(test));
  }
} finally {await inference.release();physics.destroy?.();}
report.failedMotions=report.motions.filter(x=>x.status==='fail').map(x=>x.id);
report.failedPlanner=report.planner.filter(x=>x.status==='fail');
report.status=report.failedMotions.length||report.failedPlanner.length?'fail':'pass';
await mkdir(new URL('qa/generated/',root),{recursive:true});
const output=selected?`qa/generated/m21-motion-hold-${holdOverride||'default'}.json`:'qa/generated/m21-release.json';
await writeFile(new URL(output,root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify({status:report.status,motions:report.motions.length,failedMotions:report.failedMotions,failedPlanner:report.failedPlanner}));
if(report.status!=='pass') process.exitCode=1;
