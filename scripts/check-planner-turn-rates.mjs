// node homepage/scripts/check-planner-turn-rates.mjs
// Real browser WASM engines and ONNX models, without browser UI or a server.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { resolve, dirname } from 'node:path';
import { fileURLToPath } from 'node:url';
import { createPhysics } from '../demo/local-physics.js';
import { createLocalInference } from '../demo/local-inference.js';
import { LocalPolicy, yawOf } from '../demo/local-policy.js';
import { prepareTrackingReference } from '../demo/local-reference.js';
import { plannerEntries } from '../demo/local-planner.js';

const root = new URL('../', import.meta.url);
const args = process.argv.slice(2), options = {};
for (let i = 0; i < args.length; i++) {
  const key = args[i];
  if (key === '--help') options[key] = true;
  else {
    assert.ok(['--bank', '--output', '--rates', '--cases'].includes(key), `Unknown option: ${key}`);
    assert.ok(args[i + 1] && !args[i + 1].startsWith('--'), `Missing value for ${key}`);
    options[key] = args[++i];
  }
}
if (options['--help']) {
  console.log('Usage: node homepage/scripts/check-planner-turn-rates.mjs [--bank DIR] [--output FILE] [--rates .1,.5,1] [--cases REGEXP]\nDefaults: the active bank; Slow/Walk .1/.5/1 rad/s, Run .1/.2/.3 rad/s; homepage/qa/generated/planner-turn-rates.json.');
  process.exit(0);
}
const bankPath = resolve(options['--bank'] || fileURLToPath(new URL('demo/local-assets/planner-bank/', root)));
const output = resolve(options['--output'] || fileURLToPath(new URL('qa/generated/planner-turn-rates.json', root)));
const caseFilter=options['--cases']?new RegExp(options['--cases']):null;
const customRates=options['--rates']?.split(',').map(Number);
assert.ok(!customRates || customRates.every(rate=>Number.isFinite(rate) && rate>0 && rate<=1),'Turn magnitudes must be in (0,1]');
const zero = { vx: 0, vy: 0, yaw: 0 };
// Keep the original horizontal stop threshold; also measure angular settling.
// A 6-second observation includes the measured 4-5 second walking yaw transient;
// qa/planner-turn-rates-stop4s.json preserves the shorter-window failure.
const limits = {
  minimumHeight: 0.28, minimumMovement: 0.2, maximumYawDrift: 0.3,
  maximumStopTailRMS: 0.25, minimumSignedTurnRadians: 0.15, minimumTurnResponseRatio: .25, maximumStopTailYawRMS: .3,
};
const report = {
  testedAt: new Date().toISOString(), status: 'running', bankPath,
  host: { runtime: 'Node WebAssembly', node: process.version, platform: process.platform, arch: process.arch },
  scope: 'Real local planner targets, G1 encoder, H2 decoder, closed-loop transmissions and MuJoCo contact dynamics.',
  limitations: [
    'This is not a browser rendering, worker scheduling, hardware, or exact target-speed accuracy test.',
    'Finite-state and root-height checks are sampled at the 50 Hz policy boundaries.',
    'Planner references are local targets; the original neural planner does not run online in this test.',
    'Stopping is observed for six seconds; this is not a claim of settling within four seconds. The prior four-second report is retained separately.',
    'Final angular RMS uses free-root body-local angular-z velocity; isolated diagnostics additionally check unwrapped world heading.',
  ],
  protocol: { standSeconds: 1, commandSeconds: 8, stopSeconds: 6, commandRefreshHz: 50, stopTailSeconds: 1 },
  limits, sources: {}, cases: [],
};
let physics, inference, policy, config;
const started = performance.now();
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
const rms = values => values.length ? Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length) : null;
const bytes = async path => new Uint8Array(await readFile(new URL(path, root)));
async function source(path) {
  const data = await bytes(path);
  report.sources[path] = { bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return data;
}
async function json(path) { return JSON.parse(new TextDecoder().decode(await source(path))); }
async function bankJSON(file) {
  const path = resolve(bankPath, file), data = new Uint8Array(await readFile(path));
  report.sources[`bank/${file}`] = { path, bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return JSON.parse(new TextDecoder().decode(data));
}
async function save() {
  await mkdir(dirname(output), { recursive: true });
  await writeFile(output, JSON.stringify(report, null, 2) + '\n');
}
function state() {
  return {
    time: physics.data.time,
    position: Array.from(physics.data.qpos.subarray(0, 3)),
    velocity: Array.from(physics.data.qvel.subarray(0, 3)),
    yaw: yawOf(physics.data.qpos.subarray(3, 7)), status: policy.status,
  };
}

async function segment(test, name, seconds, command) {
  const initial = state();
  const result = {
    name, command, initial, finite: true, fallen: false,
    minZ: initial.position[2], maxAbsYawDrift: 0, yawChange: 0, samples: [initial],
  };
  test.segments.push(result);
  let previous = initial;
  const tail = [], angularTail = [], steps = seconds * config.policyHz;
  try {
    for (let i = 0; i < steps; i++) {
      // Refresh against wall-clock time on every policy step, so a slow host
      // does not turn the requested movement into a command-watchdog stop.
      policy.command(command);
      const stepBefore = policy.steps, timeBefore = physics.data.time;
      await policy.step();
      const current = state();
      result.finite &&= ['qpos', 'qvel', 'ctrl'].every(key => physics.data[key].every(Number.isFinite)) && policy.lastAction.every(Number.isFinite);
      result.minZ = Math.min(result.minZ, current.position[2]);
      result.yawChange += wrap(current.yaw - previous.yaw);
      result.maxAbsYawDrift = Math.max(result.maxAbsYawDrift, Math.abs(result.yawChange));
      if ((i + 1) % config.policyHz === 0) result.samples.push(current);
      if (i >= steps - config.policyHz) {
        tail.push(Math.hypot(current.velocity[0], current.velocity[1]));
        angularTail.push(physics.data.qvel[5]);
      }
      previous = current;
      result.fallen = policy.status === 'fallen' || current.position[2] < limits.minimumHeight;
      assert.ok(result.finite, `${name}: non-finite state at ${current.time}s`);
      assert.ok(!result.fallen, `${name}: fallen at ${current.time}s`);
      assert.equal(policy.steps, stepBefore + 1, 'Policy did not advance');
      assert.ok(Math.abs(current.time - timeBefore - 1 / config.policyHz) < 1e-9, 'Physics did not advance one policy interval');
    }
  } finally {
    result.final = previous;
    result.simSeconds = previous.time - initial.time;
    result.displacement = previous.position.map((value, i) => value - initial.position[i]);
    const [x, y] = result.displacement;
    result.bodyXY = {
      forward: x * Math.cos(initial.yaw) + y * Math.sin(initial.yaw),
      left: -x * Math.sin(initial.yaw) + y * Math.cos(initial.yaw),
    };
    result.finalOneSecondRMSHorizontalSpeed = rms(tail);
    result.finalOneSecondRMSYawRate = rms(angularTail);
  }
  return result;
}

async function checkTurn(style,speed,yaw) {
  if(caseFilter && !caseFilter.test(`${style}/${speed}/${yaw}`)) return;
  const command={vx:speed,vy:0,yaw};
  const test={name:`${style}/${speed}/${yaw}`,style,speed,command,segments:[],failures:[]};
  report.cases.push(test);
  try {
    policy.command({mode:'planner',action:'reset',plannerStyle:style,motion:'squat_001__A359'});
    await segment(test,'stand',1,zero);
    const movement=await segment(test,'command',8,command);
    const stop=await segment(test,'stop',report.protocol.stopSeconds,zero);
    test.signedTurn=Math.sign(yaw)*movement.yawChange;
    test.turnResponseRatio=test.signedTurn/(Math.abs(yaw)*8);
    if(test.signedTurn<limits.minimumSignedTurnRadians || test.turnResponseRatio<limits.minimumTurnResponseRatio)
      test.failures.push(`Insufficient requested-direction turn: ${test.signedTurn} rad, ratio ${test.turnResponseRatio}`);
    if(stop.finalOneSecondRMSHorizontalSpeed>=limits.maximumStopTailRMS)
      test.failures.push(`Stop speed RMS ${stop.finalOneSecondRMSHorizontalSpeed} >= ${limits.maximumStopTailRMS}`);
    if(stop.finalOneSecondRMSYawRate>=limits.maximumStopTailYawRMS)
      test.failures.push(`Stop yaw-rate RMS ${stop.finalOneSecondRMSYawRate} >= ${limits.maximumStopTailYawRMS}`);
  }catch(error){test.failures.push(error.message);}
  test.status=test.failures.length?'fail':'pass';
  console.log(JSON.stringify({name:test.name,status:test.status,turn:test.signedTurn,ratio:test.turnResponseRatio,stop:test.segments.at(-1)?.finalOneSecondRMSHorizontalSpeed,failures:test.failures}));
  await save();
}

try {
  for (const path of ['scripts/check-planner-turn-rates.mjs', 'demo/index.html', 'demo/demo.js', 'demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-physics.js', 'demo/local-inference.js']) await source(path);
  const manifest = await json('demo/local-assets/physics/manifest.json');
  config = await json('demo/local-assets/controller.json');
  assert.equal(config.policyHz, 50);
  physics = await createPhysics({
    manifest, config: await json('demo/local-assets/physics/physics-config.json'),
    files: await Promise.all(manifest.files.map(async file => ({ path: file.path, data: await source(`demo/local-assets/physics/${file.url || file.path}`) }))),
    engineOptions: { wasmBinary: await source('demo/vendor/mujoco/mujoco.wasm') },
  });
  inference = await createLocalInference({ models: {
    encoderFull: await source('demo/local-assets/models/encoder_full.onnx'),
    decoder: await source('demo/local-assets/models/decoder.onnx')
  } });
  const motions = {};
  for (const entry of (await json('demo/local-assets/references/manifest.json')).motions) {
    const reference = await json(`demo/local-assets/references/${entry.file}`);
    motions[entry.id] = prepareTrackingReference(reference, entry);
  }
  config.planner.bank = await bankJSON('manifest.json');
  for (const entry of plannerEntries(config.planner.bank)) motions[entry.id] = await bankJSON(entry.file);
  policy = new LocalPolicy(physics, inference, config, motions);
  for(const [style,speed] of [['slow_walk',0],['slow_walk',.8],['walk',2.5],['run',4]])
    for(const rate of customRates || (style==='run'?[.1,.2,.3]:[.1,.5,1]))
      for(const sign of [1,-1]) await checkTurn(style,speed,rate*sign);
  if(!caseFilter) assert.equal(report.cases.length,(customRates?.length || 3)*8);
  else assert.ok(report.cases.length,'No cases matched');
  for(const [label,entry] of Object.entries(report.sources)) {
    const path=entry.path || fileURLToPath(new URL(label,root));
    assert.equal(createHash('sha256').update(await readFile(path)).digest('hex'),entry.sha256,`${label} changed during verification`);
  }
  report.status = report.cases.every(test => test.status === 'pass') ? 'pass' : 'fail';
} catch (error) {
  report.status = 'fail'; report.error = { name: error.name, message: error.message };
} finally {
  physics?.dispose();
  await inference?.release();
  report.wallSeconds = (performance.now() - started) / 1000;
  await save();
}
console.log(`Saved ${output}: ${report.status}`);
if (report.status !== 'pass') process.exitCode = 1;
