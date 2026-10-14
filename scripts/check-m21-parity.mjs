// Compare browser observation assembly/inference against source Python replay + CPU ONNX.
import assert from 'node:assert/strict';
import {readFile,writeFile,mkdir} from 'node:fs/promises';
import {createLocalInference} from '../demo/local-inference.js';
import {LocalPolicy} from '../demo/local-policy.js';
import {prepareTrackingReference} from '../demo/local-reference.js';
const root=new URL('../',import.meta.url);
const json=async p=>JSON.parse(await readFile(new URL(p,root),'utf8'));
const bytes=async p=>new Uint8Array(await readFile(new URL(p,root)));
const fixtures=await json('qa/m21-release-fixtures.json');
const catalog=await json('demo/local-assets/references/manifest.json');
const config=await json('demo/local-assets/controller.json');
const motions={};
for(const entry of catalog.motions) motions[entry.id]=prepareTrackingReference(await json('demo/local-assets/references/'+entry.file),entry);
const inference=await createLocalInference({models:{encoderFull:await bytes('demo/local-assets/models/encoder_full.onnx'),decoder:await bytes('demo/local-assets/models/decoder.onnx')}});
const report={status:'pass',cases:0,source:fixtures.source,maxObservationError:0,maxTokenError:0,maxActionError:0,spaces:{}};
const error=(a,b)=>Math.max(...a.map((v,i)=>Math.abs(v-b[i])));
try {
  for(const fixture of fixtures.fixtures) {
    const physics={data:{qpos:Float64Array.from(config.initialQpos),qvel:Float64Array.from(config.initialQvel)},reset({qpos}){this.data.qpos.set(qpos);this.data.qpos.set(fixture.baseQuat,3);},stepPD(){}};
    const check=async input=>{
      const full=new Float32Array(1751);const space=fixture.space,selector={h2:0,smpl:2,g1:3}[space];full[0]=selector;if(selector<3)full[selector+1]=1;
      if(space==='smpl') {full.set(input.subarray(0,720),911);full.set(input.subarray(720,780),1631);full.set(input.subarray(780),1691);}
      else {full.set(input.subarray(0,580),4);full.set(input.subarray(580),590);}
      const observationError=error(full,fixture.fullObservation);report.maxObservationError=Math.max(report.maxObservationError,observationError);
      assert.ok(observationError<1e-5,`${fixture.id}/${fixture.elapsed}: source observation mismatch ${observationError}`);
      const tokens=await (space==='smpl'?inference.encodeSMPL(input):space==='g1'?inference.encodePlanner(input):inference.encode(input));
      const tokenError=error(tokens,fixture.expectedTokens);report.maxTokenError=Math.max(report.maxTokenError,tokenError);
      assert.ok(tokenError<1e-6,`${fixture.id}: source token mismatch ${tokenError}`);
      const zeroHistory=new Float32Array(994);zeroHistory.set(tokens);
      const actions=await inference.decode(zeroHistory);const actionError=error(actions,fixture.expectedZeroHistoryAction);report.maxActionError=Math.max(report.maxActionError,actionError);
      assert.ok(actionError<1e-5,`${fixture.id}: source decoder mismatch ${actionError}`);
      return tokens;
    };
    const policy=new LocalPolicy(physics,{encode:check,encodePlanner:check,encodeSMPL:check,decode:input=>inference.decode(input)},{...config,planner:{}},motions);
    policy.reset(fixture.id);policy.startupStepsRemaining=0;policy.time=fixture.elapsed;policy.complete=fixture.elapsed>=motions[fixture.id].frames.length/50-.02;
    await policy.step();report.cases++;report.spaces[fixture.space]=(report.spaces[fixture.space]||0)+1;
  }
} finally {await inference.release();}
await mkdir(new URL('qa/generated/',root),{recursive:true});
await writeFile(new URL('qa/generated/m21-parity.json',root),JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report));
