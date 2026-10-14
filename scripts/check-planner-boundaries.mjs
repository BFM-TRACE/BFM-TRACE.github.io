// Real closed-loop boundary checks; --bank selects an isolated candidate bank.
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

const home = fileURLToPath(new URL('../', import.meta.url));
const args = process.argv.slice(2), options = {};
for (let i = 0; i < args.length; i++) {
  const key = args[i];
  if (key === '--help') options[key] = true;
  else {
    assert.ok(['--bank', '--output', '--cases'].includes(key), `Unknown option: ${key}`);
    assert.ok(args[i + 1] && !args[i + 1].startsWith('--'), `Missing value for ${key}`);
    options[key] = args[++i];
  }
}
if (options['--help']) {
  console.log(`Usage: node homepage/scripts/check-planner-boundaries.mjs [--bank DIR] [--output FILE]
Defaults: homepage/demo/local-assets/planner-bank and homepage/qa/generated/planner-boundaries.json.
Optional --cases REGEXP filters case names for focused diagnosis.
20 independently reset cases: Slow .1/.8 in four directions; Walk forward 2.5,
other directions 1.6; Run forward 4, other directions and four diagonals 2.5.
Each case stands 1 s, moves 10 s, and stops 4 s. Uses real ONNX + MuJoCo,
with finite-state and posture samples at each 50 Hz policy boundary.`);
  process.exit(0);
}
const bankPath = resolve(options['--bank'] || resolve(home, 'demo/local-assets/planner-bank'));
const output = resolve(options['--output'] || resolve(home, 'qa/generated/planner-boundaries.json'));
const caseFilter=options['--cases']?new RegExp(options['--cases']):null;
const limits = {
  minimumHeight: 0.28, minimumLowSpeedMovement: 0.15, minimumMovement: 0.2,
  maximumYawDrift: 0.3, maximumStopTailRMS: 0.25,
  maximumAbsRootPitchDeg: 25, maximumAbsTorsoPitchDeg: 40,
  maximumAbsWaistPitchDeg: 30, maximumCycleTorsoPeakToPeakDeg: 20,
};
const report = {
  testedAt: new Date().toISOString(), status: 'running', bankPath, caseFilter:options['--cases'] || null,
  host: { runtime: 'Node WebAssembly', node: process.version, platform: process.platform, arch: process.arch },
  scope: 'Real G1 planner encoder + H2 decoder + MuJoCo contacts at UI speed/direction boundaries. Reference samples are targets; no body-pose or velocity playback.',
  limitations: ['Checks simulated state, not hardware, browser UI, worker scheduling, or exact command-speed accuracy.', 'Finite state, height, and posture are checked at 50 Hz policy boundaries.', 'These engineering gates cover the explicitly listed initial conditions and stop phase.'],
  protocol: { standSeconds: 1, moveSeconds: 10, stopSeconds: 4, sampleHz: 50, stopTailSeconds: 1, steadyAfterMovementSeconds: 2 },
  limits, sources: {}, cases: [], failures: [],
};
const zero = { vx: 0, vy: 0, yaw: 0 }, degrees = 180 / Math.PI;
const pitch = q => Math.asin(Math.max(-1, Math.min(1, 2 * (q[0] * q[2] - q[3] * q[1])))) * degrees;
const wrapAngle = value => Math.atan2(Math.sin(value), Math.cos(value));
const statistics = values => values.length ? {
  min: Math.min(...values), max: Math.max(...values),
  mean: values.reduce((sum, value) => sum + value, 0) / values.length,
  rms: Math.sqrt(values.reduce((sum, value) => sum + value * value, 0) / values.length),
  peakToPeak: Math.max(...values) - Math.min(...values),
} : null;
let physics, inference, policy, config, torso, waistAddress;
const started = performance.now();
async function source(path, label) {
  const data = new Uint8Array(await readFile(path));
  report.sources[label] = { path, bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return data;
}
const localBytes = path => source(resolve(home, path), path);
const localJSON = async path => JSON.parse(new TextDecoder().decode(await localBytes(path)));
const bankJSON = async file => JSON.parse(new TextDecoder().decode(await source(resolve(bankPath, file), `bank/${file}`)));
async function save() {
  await mkdir(dirname(output), { recursive: true });
  await writeFile(output, JSON.stringify(report, null, 2) + '\n');
}
function sample(stage, stageTime) {
  physics.mujoco.mj_kinematics(physics.model, physics.data);
  const d = physics.data, p = policy.planner;
  const dominant = Object.entries(p.weights).filter(([key, weight]) => !key.startsWith('idle/') && weight > 1e-5).sort((a, b) => b[1] - a[1])[0];
  const clip = dominant && p.clips[dominant[0]], phase = clip ? (clip.singleStride ? p.gaitPhase : p.phase / clip.period) + clip.phaseOffset : null;
  return {
    stage, stageTime, time: d.time, position: Array.from(d.qpos.subarray(0, 3)),
    velocity: Array.from(d.qvel.subarray(0, 3)), speed: Math.hypot(d.qvel[0], d.qvel[1]),
    yaw: yawOf(d.qpos.subarray(3, 7)), rootPitchDeg: pitch(d.qpos.subarray(3, 7)),
    torsoPitchDeg: pitch(d.xquat.subarray(torso * 4, torso * 4 + 4)), waistPitchDeg: d.qpos[waistAddress] * degrees,
    gaitPhase: p.gaitPhase, gaitRate: p.gaitRate, dominantClip: dominant?.[0] || 'idle/idle',
    cycleIndex: phase === null ? null : Math.floor(phase), cyclePhase: phase === null ? null : ((phase % 1) + 1) % 1,
  };
}
async function segment(test, stage, seconds, command) {
  const initial = sample(stage, 0), samples = [], result = { stage, seconds, command, initial };
  test.segments.push(result);
  let last = initial, yawChange = 0, maxYawDrift = 0;
  try {
    for (let i = 0; i < seconds * config.policyHz; i++) {
      policy.command(command);
      const previousSteps = policy.steps, previousTime = physics.data.time;
      await policy.step();
      const current = sample(stage, (i + 1) / config.policyHz);
      samples.push(current); test.samples.push(current);
      yawChange += wrapAngle(current.yaw - last.yaw); maxYawDrift = Math.max(maxYawDrift, Math.abs(yawChange)); last = current;
      const finite = ['qpos', 'qvel', 'ctrl', 'xquat'].every(key => physics.data[key].every(Number.isFinite)) && policy.lastAction.every(Number.isFinite);
      test.finite &&= finite;
      test.fallen ||= policy.status === 'fallen' || current.position[2] < limits.minimumHeight;
      assert.ok(finite, `${stage}: non-finite physics or action at ${current.stageTime}s`);
      assert.ok(!test.fallen, `${stage}: fall at ${current.stageTime}s`);
      assert.equal(policy.steps, previousSteps + 1, 'Policy did not advance');
      assert.ok(Math.abs(current.time - previousTime - 1 / config.policyHz) < 1e-9, 'Physics did not advance one policy interval');
    }
  } finally {
    result.final = last; result.simSeconds = last.time - initial.time; result.samples = samples.length;
    result.displacement = last.position.map((value, i) => value - initial.position[i]);
    const [x, y] = result.displacement;
    result.bodyXY = { forward: x * Math.cos(initial.yaw) + y * Math.sin(initial.yaw), left: -x * Math.sin(initial.yaw) + y * Math.cos(initial.yaw) };
    result.maximumYawDrift = maxYawDrift;
    result.minimumHeight = Math.min(initial.position[2], ...samples.map(item => item.position[2]));
    result.finalOneSecondRMS = statistics(samples.slice(-config.policyHz).map(item => item.speed))?.rms ?? null;
    result.posture = Object.fromEntries(['rootPitchDeg', 'torsoPitchDeg', 'waistPitchDeg'].map(key => [key, statistics(samples.map(item => item[key]))]));
  }
  return result;
}
function assess(test) {
  const movement = test.segments.find(item => item.stage === 'move'), stop = test.segments.find(item => item.stage === 'stop');
  if (movement?.samples === 500) {
    const { forward, left } = movement.bodyXY, threshold = test.style === 'slow_walk' && test.speed === 0.1 ? limits.minimumLowSpeedMovement : limits.minimumMovement;
    const [x, y] = test.directionVector;
    test.signedDirectionalProgress = (forward * x + left * y) / Math.hypot(x, y);
    if (test.signedDirectionalProgress <= threshold) test.failures.push(`Directional progress ${test.signedDirectionalProgress} <= ${threshold}m`);
    if (x && y) {
      if (forward * x <= threshold) test.failures.push(`Diagonal forward/backward component ${forward * x} <= ${threshold}m`);
      if (left * y <= threshold) test.failures.push(`Diagonal lateral component ${left * y} <= ${threshold}m`);
    } else {
      const main = x ? forward * x : left * y, cross = x ? left : forward;
      if (main <= Math.abs(cross)) test.failures.push(`Intended direction ${main}m does not dominate cross-direction ${cross}m`);
    }
    if (movement.maximumYawDrift >= limits.maximumYawDrift) test.failures.push(`Yaw drift ${movement.maximumYawDrift} >= ${limits.maximumYawDrift}rad`);
  } else test.failures.push('Movement segment incomplete');
  if (stop?.samples !== 200) test.failures.push('Stop segment incomplete');
  else if (stop.finalOneSecondRMS >= limits.maximumStopTailRMS) test.failures.push(`Stop RMS ${stop.finalOneSecondRMS} >= ${limits.maximumStopTailRMS}m/s`);
  test.minimumHeight = Math.min(...test.samples.map(item => item.position[2]));
  test.posture = Object.fromEntries(['rootPitchDeg', 'torsoPitchDeg', 'waistPitchDeg'].map(key => [key, statistics(test.samples.map(item => item[key]))]));
  for (const [key, threshold] of [['rootPitchDeg', limits.maximumAbsRootPitchDeg], ['torsoPitchDeg', limits.maximumAbsTorsoPitchDeg], ['waistPitchDeg', limits.maximumAbsWaistPitchDeg]]) {
    const peak = test.posture[key];
    if (peak && Math.max(Math.abs(peak.min), Math.abs(peak.max)) > threshold) test.failures.push(`${key} absolute peak exceeds ${threshold}deg`);
  }
  const cycles = new Map();
  for (const item of test.samples.filter(item => item.stage === 'move' && item.stageTime >= 2 && item.cycleIndex !== null)) {
    const key = `${item.dominantClip}:${item.cycleIndex}`;
    if (!cycles.has(key)) cycles.set(key, []); cycles.get(key).push(item);
  }
  test.fullCycles = [...cycles].filter(([, samples]) => Math.min(...samples.map(item => item.cyclePhase)) < 0.08 && Math.max(...samples.map(item => item.cyclePhase)) > 0.92).map(([key, samples]) => ({ key, torsoPitchDeg: statistics(samples.map(item => item.torsoPitchDeg)) }));
  test.maximumCycleTorsoPeakToPeakDeg = Math.max(0, ...test.fullCycles.map(item => item.torsoPitchDeg.peakToPeak));
  if (!test.fullCycles.length) test.failures.push('No full steady gait cycle was sampled');
  if (test.maximumCycleTorsoPeakToPeakDeg >= limits.maximumCycleTorsoPeakToPeakDeg) test.failures.push(`Full-cycle torso excursion ${test.maximumCycleTorsoPeakToPeakDeg} >= ${limits.maximumCycleTorsoPeakToPeakDeg}deg`);
}
async function check(style, speed, direction, x, y) {
  if(caseFilter && !caseFilter.test(`${style}/${direction}/${speed}`)) return;
  const length = Math.hypot(x, y), command = { vx: x / length * speed, vy: y / length * speed, yaw: 0 };
  const test = { name: `${style}/${direction}/${speed}`, style, speed, direction, directionVector: [x, y], command, finite: true, fallen: false, samples: [], segments: [], failures: [] };
  report.cases.push(test);
  try {
    policy.command({ mode: 'planner', action: 'reset', plannerStyle: style, motion: 'squat_001__A359' });
    await segment(test, 'stand', 1, zero); await segment(test, 'move', 10, command); await segment(test, 'stop', 4, zero);
  } catch (error) { test.failures.push(error.message); }
  assess(test); test.status = test.failures.length ? 'fail' : 'pass';
  report.failures.push(...test.failures.map(message => ({ test: test.name, message })));
  console.log(JSON.stringify({ name: test.name, status: test.status, displacement: test.segments.find(item => item.stage === 'move')?.bodyXY, stopRMS: test.segments.find(item => item.stage === 'stop')?.finalOneSecondRMS, failures: test.failures }));
  await save();
}
try {
  for (const path of ['scripts/check-planner-boundaries.mjs', 'demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-physics.js', 'demo/local-inference.js']) await localBytes(path);
  config = await localJSON('demo/local-assets/controller.json'); assert.equal(config.policyHz, 50);
  const manifest = await localJSON('demo/local-assets/physics/manifest.json');
  physics = await createPhysics({ manifest, config: await localJSON('demo/local-assets/physics/physics-config.json'), files: await Promise.all(manifest.files.map(async file => ({ path: file.path, data: await localBytes(`demo/local-assets/physics/${file.url || file.path}`) }))), engineOptions: { wasmBinary: await localBytes('demo/vendor/mujoco/mujoco.wasm') } });
  inference = await createLocalInference({ models: { encoderFull: await localBytes('demo/local-assets/models/encoder_full.onnx'), decoder: await localBytes('demo/local-assets/models/decoder.onnx') } });
  report.engines = { mujoco: physics.mujoco.mj_versionString(), onnxRuntime: inference.runtimeVersion };
  const motions = {};
  for (const entry of (await localJSON('demo/local-assets/references/manifest.json')).motions) {
    const ref = await localJSON(`demo/local-assets/references/${entry.file}`);
    motions[entry.id] = prepareTrackingReference(ref, entry);
  }
  config.planner.bank = await bankJSON('manifest.json');
  for (const entry of plannerEntries(config.planner.bank)) motions[entry.id] = await bankJSON(entry.file);
  torso = physics.bodyNames.indexOf('torso_link'); const waist = config.localNames.indexOf('waist_pitch_joint'); waistAddress = config.localQposAddresses[waist];
  assert.ok(torso >= 0 && waist >= 0, 'Posture body/joint mappings are missing');
  policy = new LocalPolicy(physics, inference, config, motions);
  const cardinal = [['forward', 1, 0], ['back', -1, 0], ['left', 0, 1], ['right', 0, -1]];
  for (const speed of [0.1, 0.8]) for (const [direction, x, y] of cardinal) await check('slow_walk', speed, direction, x, y);
  for (const [style, forward, other] of [['walk', 2.5, 1.6], ['run', 4, 2.5]]) for (const [direction, x, y] of cardinal) await check(style, direction === 'forward' ? forward : other, direction, x, y);
  for (const [direction, x, y] of [['forward_left', 1, 1], ['forward_right', 1, -1], ['back_left', -1, 1], ['back_right', -1, -1]]) await check('run', 2.5, direction, x, y);
  if(!caseFilter) assert.equal(report.cases.length, 20);
  else assert.ok(report.cases.length>0,'No cases matched the filter');
  for (const [label, entry] of Object.entries(report.sources)) assert.equal(createHash('sha256').update(await readFile(entry.path)).digest('hex'), entry.sha256, `${label} changed during the run`);
} catch (error) { report.failures.push({ test: 'setup_or_run', message: error.message }); console.error(error); }
finally {
  try { physics?.dispose(); await inference?.release(); } catch (error) { report.failures.push({ test: 'cleanup', message: error.message }); }
  report.status = report.failures.length ? 'fail' : 'pass'; report.passedCases = report.cases.filter(item => item.status === 'pass').length;
  report.totalSimSeconds = report.cases.reduce((sum, item) => sum + item.segments.reduce((subtotal, segment) => subtotal + segment.simSeconds, 0), 0);
  report.wallSeconds = (performance.now() - started) / 1000; report.completedAt = new Date().toISOString(); await save();
}
console.log(`${report.status.toUpperCase()}: ${report.passedCases}/${report.cases.length} saved ${output}`);
if (report.status !== 'pass') process.exitCode = 1;
