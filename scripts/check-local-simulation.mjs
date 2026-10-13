// Run from any directory: node homepage/scripts/check-local-simulation.mjs
// This loads the same local JS, WASM, ONNX and reference files as the browser.
// It checks the closed loop under Node, not browser rendering or scheduling.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { createPhysics } from '../demo/local-physics.js';
import { createLocalInference } from '../demo/local-inference.js';
import { LocalPolicy, yawOf } from '../demo/local-policy.js';
import { prepareTrackingReference } from '../demo/local-reference.js';
import { plannerEntries } from '../demo/local-planner.js';

const root = new URL('../', import.meta.url);
const output = new URL('qa/generated/local-simulation.json', root);
const bytes = async path => new Uint8Array(await readFile(new URL(path, root)));
const json = async path => JSON.parse(await readFile(new URL(path, root), 'utf8'));
const trackingMotions = [
  'squat_001__A359', 'forward_lunge_R_001__A359_M',
  'dance_in_da_party_001__A464', 'macarena_001__A545',
  'walking_quip_360_R_002__A428',
];
const limits = {
  minimumRootHeight: 0.28,
  minimumDirectionalDisplacement: 0.5,
  maximumStopTailRMSHorizontalSpeed: 0.25,
  minimumSignedTurnRadians: 0.25,
};
const report = {
  testedAt: new Date().toISOString(),
  host: { runtime: 'Node WebAssembly', node: process.version, platform: process.platform, arch: process.arch },
  scope: 'Real native H2 encoder/decoder inference, PD, closed-chain transmissions, gravity and contact. No network service or reference-pose playback.',
  limitations: [
    'Node runs the browser WASM binaries without real-time pacing; this is not a browser rendering or worker scheduling test.',
    'The small local planner chooses gait phase and heading from an offline generated reference. It does not run the original large learned planner online.',
    'Movement, stopping and turning use broad functional smoke thresholds, not target-velocity accuracy claims.',
    'Finite state and the controller root-height fall threshold do not establish tracking accuracy or hardware safety.',
  ],
  sampling: { stateChecksHz: 50, trajectorySamplesHz: 1, minimumHeight: 'Minimum root z at the 50 Hz control boundaries, including each initial state' },
  limits, sources: {}, tracking: [], planner: [], failures: [],
};
let physics, inference, policy, config;
const runStarted = performance.now();
const wrappedAngle = value => Math.atan2(Math.sin(value), Math.cos(value));
const horizontalSpeed = velocity => Math.hypot(velocity[0], velocity[1]);
const describeError = error => ({ name: error.name, message: error.message });

function snapshot() {
  const state = physics.getState();
  for (const field of ['qpos', 'qvel', 'ctrl', 'body_poses']) {
    assert.ok(state[field].every(Number.isFinite), `${field} contains a non-finite value`);
  }
  assert.equal(state.body_poses.length, 315);
  return {
    simTime: state.sim_time,
    position: state.root_position,
    velocity: state.root_velocity,
    yaw: yawOf(state.qpos.subarray(3, 7)),
    status: policy.status,
  };
}

// Every step renews commands on the actual wall clock. A slow host therefore
// does not silently convert a requested movement into a watchdog stop.
async function checkedStep(command = { vx: 0, vy: 0, yaw: 0 }) {
  policy.command(command);
  const previousTime = physics.data.time;
  const previousSteps = policy.steps;
  await policy.step();
  assert.equal(policy.steps, previousSteps + 1, 'Policy failed to advance');
  assert.ok(Math.abs(physics.data.time - previousTime - 1 / config.policyHz) < 1e-9, 'Physics time did not advance one control interval');
  for (const field of ['qpos', 'qvel', 'ctrl']) {
    assert.ok(physics.data[field].every(Number.isFinite), `${field} contains a non-finite value`);
  }
  assert.ok(policy.lastAction.every(Number.isFinite), 'Policy action contains a non-finite value');
  assert.notEqual(policy.status, 'fallen', `Controller detected a fall at ${physics.data.time}s`);
  assert.ok(physics.data.qpos[2] >= limits.minimumRootHeight, 'Root fell below the controller threshold');
}

function newMetrics(name) {
  const initial = snapshot();
  return { name, initial, minRootHeight: initial.position[2], maxHorizontalSpeed: horizontalSpeed(initial.velocity), horizontalPathLength: 0, yawChangeRadians: 0, samples: [initial] };
}

function accumulate(metrics, previous, current) {
  metrics.minRootHeight = Math.min(metrics.minRootHeight, current.position[2]);
  metrics.maxHorizontalSpeed = Math.max(metrics.maxHorizontalSpeed, horizontalSpeed(current.velocity));
  metrics.horizontalPathLength += Math.hypot(current.position[0] - previous.position[0], current.position[1] - previous.position[1]);
  metrics.yawChangeRadians += wrappedAngle(current.yaw - previous.yaw);
}

function finishMetrics(metrics, last, started) {
  metrics.final = last;
  metrics.simSeconds = last.simTime - metrics.initial.simTime;
  metrics.displacement = last.position.map((value, i) => value - metrics.initial.position[i]);
  metrics.horizontalDisplacement = Math.hypot(metrics.displacement[0], metrics.displacement[1]);
  metrics.wallSeconds = (performance.now() - started) / 1000;
  metrics.status = 'pass';
  return metrics;
}

function failMetrics(metrics, last, started, error) {
  // A non-finite snapshot must not hide the original failing assertion.
  try { last = snapshot(); } catch { /* Retain the last finite sample. */ }
  finishMetrics(metrics, last, started);
  metrics.status = 'fail'; metrics.error = describeError(error);
  metrics.failedAtSimTime = Number.isFinite(physics.data.time) ? physics.data.time : null;
  report.failures.push({ test: metrics.name, ...describeError(error) });
}

async function checkTracking(id, motions) {
  policy.command({ mode: 'tracking' });
  policy.reset(id);
  const reference = motions[id], referenceSeconds = (reference.frames.length - 1) / config.policyHz;
  const started = performance.now(), metrics = newMetrics(id);
  Object.assign(metrics, { referenceFrames: reference.frames.length, referenceSeconds });
  report.tracking.push(metrics);
  let last = metrics.initial;
  try {
    while (!policy.complete) {
      assert.ok(policy.steps <= reference.frames.length + 2, 'Reference did not reach its last frame');
      await checkedStep();
      const current = snapshot();
      accumulate(metrics, last, current); last = current;
      if (policy.steps % config.policyHz === 0) metrics.samples.push(current);
    }
    assert.equal(policy.time, referenceSeconds, 'Tracking did not reach the final reference frame');
    metrics.completion = snapshot();
    metrics.completionStep = policy.steps;
    const holdSteps = config.policyHz;
    for (let i = 0; i < holdSteps; i++) {
      await checkedStep();
      const current = snapshot();
      accumulate(metrics, last, current); last = current;
      assert.equal(policy.complete, true);
      assert.equal(policy.time, referenceSeconds, 'Completed reference advanced or restarted');
    }
    metrics.holdSeconds = last.simTime - metrics.completion.simTime;
    metrics.holdControlSteps = holdSteps;
    assert.ok(metrics.holdSeconds >= 1 - 1e-9, 'Terminal-frame hold lasted less than one simulated second');
    metrics.samples.push(last);
    finishMetrics(metrics, last, started);
    console.log(`PASS tracking ${id}: ${metrics.simSeconds.toFixed(2)} sim s, minimum root z ${metrics.minRootHeight.toFixed(3)} m`);
  } catch (error) {
    failMetrics(metrics, last, started, error);
    console.error(`FAIL tracking ${id}: ${error.message}`);
  }
}

async function checkPlannerSegment(segment) {
  const { name, seconds, command, check } = segment;
  const started = performance.now(), metrics = newMetrics(name);
  Object.assign(metrics, { command, requestedSeconds: seconds });
  report.planner.push(metrics);
  let last = metrics.initial;
  const speedTail = [], steps = Math.round(seconds * config.policyHz);
  try {
    for (let i = 0; i < steps; i++) {
      await checkedStep(command);
      const current = snapshot();
      accumulate(metrics, last, current); last = current;
      if (i >= steps - config.policyHz) speedTail.push(horizontalSpeed(current.velocity));
      if ((i + 1) % config.policyHz === 0) metrics.samples.push(current);
    }
    finishMetrics(metrics, last, started);
    metrics.finalOneSecondRMSHorizontalSpeed = Math.sqrt(speedTail.reduce((sum, value) => sum + value * value, 0) / speedTail.length);
    if (check === 'move') {
      const direction = metrics.initial.yaw + Math.atan2(command.vy, command.vx);
      metrics.displacementAlongRequestedInitialDirection = metrics.displacement[0] * Math.cos(direction) + metrics.displacement[1] * Math.sin(direction);
      assert.ok(metrics.displacementAlongRequestedInitialDirection > limits.minimumDirectionalDisplacement, 'Robot did not move far enough along the requested direction');
    } else if (check === 'stop') {
      assert.ok(metrics.finalOneSecondRMSHorizontalSpeed < limits.maximumStopTailRMSHorizontalSpeed, 'Robot did not settle after the stop command');
    } else if (check === 'turn') {
      assert.ok(Math.sign(command.yaw) * metrics.yawChangeRadians > limits.minimumSignedTurnRadians, 'Robot did not turn in the requested direction');
    }
    console.log(`PASS planner ${name}: delta [${metrics.displacement.map(v => v.toFixed(3)).join(', ')}] m, tail RMS ${metrics.finalOneSecondRMSHorizontalSpeed.toFixed(3)} m/s, minimum root z ${metrics.minRootHeight.toFixed(3)} m`);
  } catch (error) {
    failMetrics(metrics, last, started, error);
    throw error; // Later commands would no longer test the intended trajectory.
  }
}

try {
  const sourcePaths = [
    'scripts/check-local-simulation.mjs',
    'demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-physics.js', 'demo/local-inference.js',
    'demo/local-assets/controller.json', 'demo/local-assets/physics/model.xml',
    'demo/local-assets/physics/physics-config.json',
    'demo/local-assets/models/encoder_native.onnx', 'demo/local-assets/models/decoder.onnx', 'demo/local-assets/models/encoder_g1.onnx',
    'demo/vendor/mujoco/mujoco.wasm', 'demo/vendor/ort/ort.wasm.min.mjs',
    'demo/vendor/ort/ort-wasm-simd-threaded.wasm',
  ];
  await Promise.all(sourcePaths.map(async path => {
    const data = await bytes(path);
    report.sources[path] = { bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  }));
  config = await json('demo/local-assets/controller.json');
  assert.equal(config.policyHz, 50);
  const manifest = await json('demo/local-assets/physics/manifest.json');
  physics = await createPhysics({
    manifest, config: await json('demo/local-assets/physics/physics-config.json'),
    files: await Promise.all(manifest.files.map(async f => ({ path: f.path, data: await bytes(`demo/local-assets/physics/${f.url || f.path}`) }))),
    engineOptions: { wasmBinary: await bytes('demo/vendor/mujoco/mujoco.wasm') },
  });
  inference = await createLocalInference({ provider: 'wasm', models: {
    encoder: await bytes('demo/local-assets/models/encoder_native.onnx'),
    decoder: await bytes('demo/local-assets/models/decoder.onnx'),
    plannerEncoder: await bytes('demo/local-assets/models/encoder_g1.onnx'),
  } });
  report.engines = { mujoco: physics.mujoco.mj_versionString(), onnxRuntime: inference.runtimeVersion, provider: inference.provider, onnxThreads: 1 };
  report.modelMetadata = inference.metadata;
  const motions = {}, requested = new Set(trackingMotions);
  for (const entry of (await json('demo/local-assets/references/manifest.json')).motions) {
    if (!requested.has(entry.id)) continue;
    const path = `demo/local-assets/references/${entry.file}`, data = await bytes(path);
    report.sources[path] = { bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
    const reference = JSON.parse(new TextDecoder().decode(data));
    assert.equal(reference.fps, config.policyHz);
    motions[entry.id] = prepareTrackingReference(reference, entry);
  }
  assert.ok([...requested].every(id => motions[id]), 'A required reference is missing');
  config.planner.bank=await json('demo/local-assets/planner-bank/manifest.json');
  for(const entry of plannerEntries(config.planner.bank)) {
    const path=`demo/local-assets/planner-bank/${entry.file}`,data=await bytes(path);
    report.sources[path]={bytes:data.byteLength,sha256:createHash('sha256').update(data).digest('hex')};
    motions[entry.id]=JSON.parse(new TextDecoder().decode(data));
  }
  policy = new LocalPolicy(physics, inference, config, motions);
  for (const id of trackingMotions) await checkTracking(id, motions);

  policy.reset('squat_001__A359');
  policy.command({ mode: 'planner', action: 'start' });
  const stop = { vx: 0, vy: 0, yaw: 0 };
  const segments = [
    { name: 'initial_stand', seconds: 2, command: stop, check: 'stop' },
    { name: 'forward', seconds: 7, command: { vx: 0.4, vy: 0, yaw: 0 }, check: 'move' },
    { name: 'stop_after_forward', seconds: 2, command: stop, check: 'stop' },
    { name: 'left_walk', seconds: 8, command: { vx: 0, vy: 0.4, yaw: 0 }, check: 'move' },
    { name: 'stop_after_left', seconds: 2, command: stop, check: 'stop' },
    { name: 'reverse', seconds: 7, command: { vx: -0.4, vy: 0, yaw: 0 }, check: 'move' },
    { name: 'stop_after_reverse', seconds: 3, command: stop, check: 'stop' },
    { name: 'Q_turn_left', seconds: 3, command: { vx: 0, vy: 0, yaw: 0.5 }, check: 'turn' },
    { name: 'stop_after_Q', seconds: 2, command: stop, check: 'stop' },
    { name: 'E_turn_right', seconds: 3, command: { vx: 0, vy: 0, yaw: -0.5 }, check: 'turn' },
    { name: 'stop_after_E', seconds: 2, command: stop, check: 'stop' },
  ];
  for (const segment of segments) await checkPlannerSegment(segment);
  assert.equal(report.tracking.length, 5);
  assert.equal(report.planner.length, segments.length);
} catch (error) {
  if (!report.failures.some(failure => failure.message === error.message)) report.failures.push({ test: 'setup_or_run', ...describeError(error) });
  console.error(error.stack || error);
} finally {
  try { physics?.dispose(); } catch (error) { report.failures.push({ test: 'physics_cleanup', ...describeError(error) }); }
  try { await inference?.release(); } catch (error) { report.failures.push({ test: 'inference_cleanup', ...describeError(error) }); }
  report.wallSeconds = (performance.now() - runStarted) / 1000;
  report.totalSimSeconds = [...report.tracking, ...report.planner].reduce((sum, test) => sum + (test.simSeconds || 0), 0);
  report.passedTrackingMotions = report.tracking.filter(test => test.status === 'pass').length;
  report.passedPlannerSegments = report.planner.filter(test => test.status === 'pass').length;
  report.status = report.failures.length ? 'fail' : 'pass';
  report.completedAt = new Date().toISOString();
  await mkdir(new URL('qa/generated/', root), { recursive: true });
  await writeFile(output, JSON.stringify(report, null, 2) + '\n');
  console.log(`${report.status.toUpperCase()}: saved ${fileURLToPath(output)}`);
  if (report.failures.length) process.exitCode = 1;
}
