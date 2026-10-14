// Real posture and speed-response diagnosis; run with --help for options.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { resolve, dirname, extname } from 'node:path';
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
  if (['--help', '--only-long', '--skip-long'].includes(key)) options[key] = true;
  else {
    assert.ok(['--bank', '--output', '--csv', '--long-seconds', '--move-seconds', '--stop-seconds', '--speeds'].includes(key), `Unknown option: ${key}`);
    assert.ok(args[i + 1] && !args[i + 1].startsWith('--'), `Missing value for ${key}`);
    options[key] = args[++i];
  }
}
if (options['--help']) {
  console.log(`Usage: node homepage/scripts/check-planner-quality.mjs [options]
  --bank DIR          Planner bank directory containing manifest.json.
                      Defaults to homepage/demo/local-assets/planner-bank.
  --output FILE       JSON report (default: homepage/qa/generated/planner-quality.json).
  --csv FILE          CSV samples (default: same base name as JSON).
  --long-seconds N    Run 2.5 m/s command duration (default 20; stand 1, stop 3).
  --move-seconds N    Per-speed command duration (default 6; stand 1).
  --stop-seconds N    Per-speed braking observation window (default 4).
  --speeds SPEC       e.g. 'slow_walk:.1,.4,.8;walk:.8,.9,2.5;run:2.5,3.5,4'
  --only-long         Only run the long forward-running case.
  --skip-long         Only run the speed sweep.
Reports contain 50 Hz real physics samples, reference and actual posture,
per-loop seam windows, full-cycle torso excursion, speed response and source
hashes. Exit 1 means a physics or quality limit failed. Commanded speed is not
assumed equal to measured speed. No reference pose is assigned to physics.`);
  process.exit(0);
}
assert.ok(!(options['--only-long'] && options['--skip-long']), 'Choose either --only-long or --skip-long');
const positive = (key, fallback) => {
  const value = Number(options[key] ?? fallback);
  assert.ok(Number.isFinite(value) && value > 0 && Math.abs(value * 50 - Math.round(value * 50)) < 1e-8, `${key} must be a positive multiple of 0.02 seconds`);
  return value;
};
const bankDirectory = resolve(options['--bank'] || resolve(home, 'demo/local-assets/planner-bank'));
const output = resolve(options['--output'] || resolve(home, 'qa/generated/planner-quality.json'));
const outputExtension = extname(output);
const csvOutput = resolve(options['--csv'] || (outputExtension ? output.slice(0, -outputExtension.length) : output) + '.csv');
const durations = { stand: 1, long: positive('--long-seconds', 20), move: positive('--move-seconds', 6), stop: positive('--stop-seconds', 4), longStop: 3 };
const speedSpec = options['--speeds'] || 'slow_walk:.1,.2,.4,.8;walk:.8,.9,1.4,2.5;run:2.5,3,3.5,4';
const sweep = speedSpec.split(';').map(part => {
  const [style, list] = part.split(':');
  assert.ok(['slow_walk', 'walk', 'run'].includes(style) && list, `Invalid speed specification: ${part}`);
  const speeds = list.split(',').map(Number);
  assert.ok(speeds.every(speed => Number.isFinite(speed) && speed > 0 && speed <= 7.5), `Invalid speeds: ${part}`);
  return { style, speeds };
});
const limits = {
  minimumRootHeight: 0.28, maximumAbsRootPitchDeg: 25, maximumAbsTorsoPitchDeg: 40,
  maximumAbsWaistPitchDeg: 30, maximumCycleTorsoPeakToPeakDeg: 20,
  maximumStopTailRMS: 0.25, maximumSpeedResponseDecrease: 0.05, minimumSpeedRatio: .25,
};
const report = {
  testedAt: new Date().toISOString(), status: 'running', bankDirectory,
  host: { runtime: 'Node WebAssembly', node: process.version, platform: process.platform, arch: process.arch },
  scope: 'Real G1 encoder + H2 decoder + MuJoCo posture and speed response; no rendered-pose replay.',
  limitations: ['Engineering regression limits, not hardware safety guarantees or exact target-speed accuracy.', 'State checks are at 50 Hz policy boundaries.', 'No browser rendering, worker scheduling or real-time pacing is tested.'],
  protocol: { durations, sweep, sampleHz: 50, seamWindowSeconds: 0.12, steadyAfterMovementSeconds: 2 },
  limits, sources: {}, referenceQuality: [], cases: [], speedResponse: [], failures: [],
};
const zero = { vx: 0, vy: 0, yaw: 0 }, degrees = 180 / Math.PI;
const pitch = q => Math.asin(Math.max(-1, Math.min(1, 2 * (q[0] * q[2] - q[3] * q[1])))) * degrees;
const mean = values => values.length ? values.reduce((a, b) => a + b, 0) / values.length : null;
function statistics(values) {
  if (!values.length) return null;
  const sorted = [...values].sort((a, b) => a - b);
  return { min: sorted[0], mean: mean(values), max: sorted.at(-1), p95: sorted[Math.floor(0.95 * (sorted.length - 1))], rms: Math.sqrt(mean(values.map(value => value * value))), peakToPeak: sorted.at(-1) - sorted[0] };
}
const fields = ['actualHorizontalSpeed', 'actualForwardSpeed', 'rootPitchDeg', 'pelvisPitchDeg', 'torsoPitchDeg', 'waistPitchDeg', 'targetWaistDeg', 'referenceRootPitchDeg', 'referenceWaistDeg', 'referenceWaistRate', 'waistRate', 'phaseRate', 'gaitRate'];
const summarize = samples => Object.fromEntries(fields.map(key => [key, statistics(samples.map(sample => sample[key]))]));
const csvFields = ['stage', 'time', 'commandSpeed', ...fields, 'rootZ', 'dominantClip', 'cycleIndex', 'cyclePhase', 'seamCross', 'seamNear'];
const csv = [['case', ...csvFields].join(',')];
const csvCell = value => typeof value === 'string' ? JSON.stringify(value) : value ?? '';
let physics, inference, policy, config, waistLocal, waistPolicy, waistAddress, waistDof, torso, pelvis;
const started = performance.now();
async function source(path, label = path) {
  const data = new Uint8Array(await readFile(path));
  report.sources[label] = { bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return data;
}
const localBytes = path => source(resolve(home, path), path);
const localJSON = async path => JSON.parse(new TextDecoder().decode(await localBytes(path)));
const bankJSON = async file => JSON.parse(new TextDecoder().decode(await source(resolve(bankDirectory, file), `bank/${file}`)));
async function save() {
  await mkdir(dirname(output), { recursive: true }); await mkdir(dirname(csvOutput), { recursive: true });
  await writeFile(output, JSON.stringify(report, null, 2) + '\n');
  await writeFile(csvOutput, csv.join('\n') + '\n');
}

function sample(stage, command, previousCycles) {
  physics.mujoco.mj_kinematics(physics.model, physics.data);
  const d = physics.data, planner = policy.planner, reference = planner.reference(0), rootQuaternion = d.qpos.subarray(3, 7), heading = yawOf(rootQuaternion);
  const clips = Object.entries(planner.weights).filter(([key, weight]) => weight > 1e-5 && !key.startsWith('idle/')).map(([key, weight]) => {
    const clip = planner.clips[key];
    assert.ok(clip?.period > 0, `No phase metadata for active clip ${key}`);
    const phase = (clip.singleStride ? planner.gaitPhase : planner.phase / clip.period) + clip.phaseOffset;
    const cyclesPerSecond = clip.singleStride ? planner.gaitRate : planner.phaseRate / clip.period;
    const cycleIndex = Math.floor(phase), cyclePhase = ((phase % 1) + 1) % 1;
    const seamCross = stage === 'command' && previousCycles.has(key) && previousCycles.get(key) !== cycleIndex;
    previousCycles.set(key, cycleIndex);
    return { key, id: clip.ref.id, weight, period: clip.period, singleStride: !!clip.singleStride, cyclesPerSecond, cycleIndex, cyclePhase, seamCross, seamNear: Math.min(cyclePhase, 1 - cyclePhase) / cyclesPerSecond <= 0.12 };
  }).sort((a, b) => b.weight - a.weight);
  const dominant = clips[0];
  return {
    stage, time: d.time, commandSpeed: command.vx, actualHorizontalSpeed: Math.hypot(d.qvel[0], d.qvel[1]),
    actualForwardSpeed: d.qvel[0] * Math.cos(heading) + d.qvel[1] * Math.sin(heading), rootZ: d.qpos[2],
    rootPitchDeg: pitch(rootQuaternion), pelvisPitchDeg: pitch(d.xquat.subarray(pelvis * 4, pelvis * 4 + 4)), torsoPitchDeg: pitch(d.xquat.subarray(torso * 4, torso * 4 + 4)),
    waistPitchDeg: d.qpos[waistAddress] * degrees, targetWaistDeg: policy.target[waistLocal] * degrees,
    referenceRootPitchDeg: pitch(reference.quat), referenceWaistDeg: reference.policyQ[waistPolicy] * degrees,
    referenceWaistRate: reference.policyDq[waistPolicy], waistRate: d.qvel[waistDof], phaseRate: planner.phaseRate, gaitRate: planner.gaitRate ?? 1,
    dominantClip: dominant?.key || 'idle/idle', cycleIndex: dominant?.cycleIndex ?? null, cyclePhase: dominant?.cyclePhase ?? null,
    seamCross: dominant?.seamCross || false, seamNear: dominant?.seamNear || false, activeClips: clips,
  };
}

function assess(test) {
  for (const stage of ['stand', 'command', 'stop']) test[stage] = summarize(test.samples.filter(sample => sample.stage === stage));
  const steady = test.samples.filter(sample => sample.stage === 'command' && sample.time >= durations.stand + 2);
  test.steady = summarize(steady);
  test.stopTail = summarize(test.samples.filter(sample => sample.stage === 'stop' && sample.time >= durations.stand + test.moveSeconds + test.stopSeconds - 1));
  test.minRootHeight = Math.min(...test.samples.map(sample => sample.rootZ));
  test.steadySpeedRatio = test.steady.actualForwardSpeed?.mean / test.commandSpeed;
  test.steadyNearSeam = summarize(steady.filter(sample => sample.seamNear));
  test.steadyAwayFromSeam = summarize(steady.filter(sample => !sample.seamNear));
  test.seams = test.samples.filter(sample => sample.seamCross).map(seam => {
    const near = test.samples.filter(sample => sample.stage === 'command' && Math.abs(sample.time - seam.time) <= 0.12);
    return { time: seam.time, clip: seam.dominantClip, maxAbsTorsoPitchDeg: Math.max(...near.map(sample => Math.abs(sample.torsoPitchDeg))), maxAbsWaistRate: Math.max(...near.map(sample => Math.abs(sample.waistRate))), maxAbsReferenceWaistRate: Math.max(...near.map(sample => Math.abs(sample.referenceWaistRate))) };
  });
  const cycles = new Map();
  for (const sample of steady) {
    const key = `${sample.dominantClip}:${sample.cycleIndex}`;
    if (!cycles.has(key)) cycles.set(key, []); cycles.get(key).push(sample);
  }
  test.fullCycles = [...cycles].filter(([, samples]) => Math.min(...samples.map(sample => sample.cyclePhase)) < 0.08 && Math.max(...samples.map(sample => sample.cyclePhase)) > 0.92).map(([key, samples]) => ({ key, start: samples[0].time, end: samples.at(-1).time, torsoPitchDeg: statistics(samples.map(sample => sample.torsoPitchDeg)) }));
  for (const [field, limit] of [['rootPitchDeg', limits.maximumAbsRootPitchDeg], ['torsoPitchDeg', limits.maximumAbsTorsoPitchDeg], ['waistPitchDeg', limits.maximumAbsWaistPitchDeg]]) {
    const peak = Math.max(...test.samples.map(sample => Math.abs(sample[field])));
    if (peak > limit) test.failures.push(`${field} absolute peak ${peak.toFixed(3)}° > ${limit}°`);
  }
  const excursion = Math.max(0, ...test.fullCycles.map(cycle => cycle.torsoPitchDeg.peakToPeak));
  test.maximumFullCycleTorsoPeakToPeakDeg = excursion;
  if (excursion >= limits.maximumCycleTorsoPeakToPeakDeg) test.failures.push(`Full-cycle torso excursion ${excursion.toFixed(3)}° >= ${limits.maximumCycleTorsoPeakToPeakDeg}°`);
  if (!test.fullCycles.length && test.moveSeconds >= 6) test.failures.push('No complete steady gait cycle was sampled');
  // A stationary robot can be upright and monotonic at every slider value.
  // Require actual progress as well, so low-amplitude token dead zones fail.
  if (test.steadySpeedRatio < limits.minimumSpeedRatio) test.failures.push(`Measured/requested forward speed ${test.steadySpeedRatio.toFixed(4)} < ${limits.minimumSpeedRatio}`);
  if (test.stopTail.actualHorizontalSpeed?.rms >= limits.maximumStopTailRMS) test.failures.push(`Stop RMS ${test.stopTail.actualHorizontalSpeed.rms.toFixed(4)} m/s >= ${limits.maximumStopTailRMS}`);
}

async function testCase(name, style, commandSpeed, moveSeconds, stopSeconds, kind) {
  policy.command({ mode: 'planner', action: 'reset', plannerStyle: style, motion: 'squat_001__A359' });
  const test = { name, kind, style, commandSpeed, moveSeconds, stopSeconds, samples: [], finite: true, fallen: false, failures: [] };
  report.cases.push(test); const previousCycles = new Map();
  try {
    for (const [stage, seconds, command] of [['stand', durations.stand, zero], ['command', moveSeconds, { vx: commandSpeed, vy: 0, yaw: 0 }], ['stop', stopSeconds, zero]]) {
      for (let i = 0; i < Math.round(seconds * config.policyHz); i++) {
        policy.command(command); const previousSteps = policy.steps; await policy.step();
        const current = sample(stage, command, previousCycles); test.samples.push(current);
        csv.push([name, ...csvFields.map(field => current[field])].map(csvCell).join(','));
        test.finite &&= ['qpos', 'qvel', 'ctrl'].every(key => physics.data[key].every(Number.isFinite)) && policy.lastAction.every(Number.isFinite);
        test.fallen = policy.status === 'fallen' || current.rootZ < limits.minimumRootHeight;
        assert.ok(test.finite, `${stage}: non-finite state at ${current.time}s`);
        assert.ok(!test.fallen, `${stage}: fallen at ${current.time}s`);
        assert.equal(policy.steps, previousSteps + 1, 'Policy did not advance');
      }
    }
  } catch (error) { test.failures.push(error.message); }
  assess(test);
  test.status = test.failures.length ? 'fail' : 'pass';
  console.log(JSON.stringify({ name, status: test.status, actualForward: test.steady.actualForwardSpeed?.mean, torsoPeak: test.command.torsoPitchDeg?.max, cycleExcursion: test.maximumFullCycleTorsoPeakToPeakDeg, stopRMS: test.stopTail.actualHorizontalSpeed?.rms, failures: test.failures }));
  await save();
}

try {
  for (const path of ['scripts/check-planner-quality.mjs', 'demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-physics.js', 'demo/local-inference.js']) await localBytes(path);
  config = await localJSON('demo/local-assets/controller.json'); assert.equal(config.policyHz, 50);
  const manifest = await localJSON('demo/local-assets/physics/manifest.json');
  physics = await createPhysics({ manifest, config: await localJSON('demo/local-assets/physics/physics-config.json'), files: await Promise.all(manifest.files.map(async file => ({ path: file.path, data: await localBytes(`demo/local-assets/physics/${file.url || file.path}`) }))), engineOptions: { wasmBinary: await localBytes('demo/vendor/mujoco/mujoco.wasm') } });
  inference = await createLocalInference({ models: { encoderFull: await localBytes('demo/local-assets/models/encoder_full.onnx'), decoder: await localBytes('demo/local-assets/models/decoder.onnx') } });
  const motions = {};
  for (const entry of (await localJSON('demo/local-assets/references/manifest.json')).motions) {
    const ref = await localJSON(`demo/local-assets/references/${entry.file}`);
    motions[entry.id] = prepareTrackingReference(ref, entry);
  }
  config.planner.bank = await bankJSON('manifest.json');
  for (const entry of plannerEntries(config.planner.bank)) motions[entry.id] = await bankJSON(entry.file);
  waistLocal = config.localNames.indexOf('waist_pitch_joint'); waistPolicy = config.policyToLocal.indexOf(waistLocal);
  waistAddress = config.localQposAddresses[waistLocal]; waistDof = config.localDofAddresses[waistLocal];
  torso = physics.bodyNames.indexOf('torso_link'); pelvis = physics.bodyNames.indexOf('pelvis');
  assert.ok(waistLocal >= 0 && waistPolicy >= 0 && torso >= 0 && pelvis >= 0, 'H2 posture mappings are missing');
  for (const entry of plannerEntries(config.planner.bank)) {
    const ref = motions[entry.id], loop = entry.recommendedLoop || ref.recommendedLoop;
    report.referenceQuality.push({ id: entry.id, targetSpeed: entry.targetSpeed ?? ref.targetSpeed, loop, rootPitchDeg: statistics(ref.rootQuat.slice(loop.startFrame, loop.endFrame).map(pitch)), waistPitchDeg: statistics(ref.q29G1.slice(loop.startFrame, loop.endFrame).map(q => q[waistPolicy] * degrees)) });
  }
  policy = new LocalPolicy(physics, inference, config, motions);
  if (!options['--skip-long']) await testCase('run_2.5_long', 'run', 2.5, durations.long, durations.longStop, 'long');
  if (!options['--only-long']) for (const { style, speeds } of sweep) for (const speed of speeds) await testCase(`${style}_${speed}_sweep`, style, speed, durations.move, durations.stop, 'sweep');
  for (const { style } of sweep) {
    if (report.speedResponse.some(item => item.style === style)) continue;
    const tests = report.cases.filter(test => test.style === style && test.kind === 'sweep').sort((a, b) => a.commandSpeed - b.commandSpeed);
    const points = tests.map(test => ({ requested: test.commandSpeed, measured: test.steady.actualForwardSpeed?.mean ?? null }));
    const decreases = points.slice(1).flatMap((point, i) => points[i].measured !== null && point.measured !== null && points[i].measured - point.measured > limits.maximumSpeedResponseDecrease ? [{ from: points[i], to: point, decrease: points[i].measured - point.measured }] : []);
    report.speedResponse.push({ style, points, decreases });
    if (decreases.length) report.failures.push(`${style}: speed response decreased by more than ${limits.maximumSpeedResponseDecrease} m/s`);
  }
  for (const test of report.cases) for (const failure of test.failures) report.failures.push(`${test.name}: ${failure}`);
  report.status = report.failures.length ? 'fail' : 'pass';
} catch (error) {
  report.status = 'fail'; report.failures.push(error.message);
} finally {
  physics?.dispose(); await inference?.release();
  report.wallSeconds = (performance.now() - started) / 1000;
  await save();
}
console.log(`QUALITY ${output}: ${report.status}`);
if (report.status !== 'pass') process.exitCode = 1;
