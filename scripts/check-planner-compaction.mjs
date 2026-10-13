// Verify that compact packaging preserves every sampled control target exactly.
import assert from 'node:assert/strict';
import {readFile,writeFile} from 'node:fs/promises';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import {LocalGaitPlanner,plannerEntries} from '../demo/local-planner.js';
const [sourcePath,compactPath,outputPath]=process.argv.slice(2);
assert.ok(sourcePath && compactPath && outputPath,'Usage: source-bank compact-bank report.json');
async function load(path){
 const raw=await readFile(resolve(path,'manifest.json'));const bank=JSON.parse(raw),refs={};
 for(const entry of plannerEntries(bank))refs[entry.id]=JSON.parse(await readFile(resolve(path,entry.file)));
 return {planner:new LocalGaitPlanner(bank,refs),manifestSHA256:createHash('sha256').update(raw).digest('hex')};
}
const [a,b]=await Promise.all([load(sourcePath),load(compactPath)]);
const commands=[['slow_walk',.1,0,0],['slow_walk',.2,0,0],['slow_walk',.8,0,0],['walk',.9,.4,.1],['walk',-1,0,0],['run',2.5,0,0],['run',4,0,0],['run',1.6,1.6,0],['run',0,-2.5,-.1],['idle',0,0,0]];
let samples=0,maxError=0;
for(const [style,vx,vy,yaw] of commands){
 a.planner.setStyle(style);b.planner.setStyle(style);
 for(let step=0;step<100;step++){
  for(const p of [a.planner,b.planner])p.advance(.02,{vx,vy,yaw});
  for(let f=0;f<10;f++){
   const x=a.planner.reference(f*.1),y=b.planner.reference(f*.1);
   for(const key of ['policyQ','policyDq','quat'])for(let i=0;i<x[key].length;i++){
    maxError=Math.max(maxError,Math.abs(x[key][i]-y[key][i]));
    assert.equal(x[key][i],y[key][i],`${style} ${step} ${f} ${key}[${i}] changed during packaging`);
   }
   samples++;
  }
 }
}
const report={checkedAt:new Date().toISOString(),status:'pass',scope:'Exact JS-number parity of q, qdot, and quaternion inputs for 10 future samples at every 50 Hz step through speeds, directions, transitions and stop. Physics/model code unchanged by packaging.',sourceManifestSHA256:a.manifestSHA256,compactManifestSHA256:b.manifestSHA256,sampledReferences:samples,maxAbsoluteError:maxError};
await writeFile(resolve(outputPath),JSON.stringify(report,null,2)+'\n');console.log(report);
