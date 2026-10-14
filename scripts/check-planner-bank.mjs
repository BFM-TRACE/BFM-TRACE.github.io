// node homepage/scripts/check-planner-bank.mjs
// Real browser WASM engines and policy models under Node; no server or replay.
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
    assert.ok(['--bank', '--output'].includes(key), `Unknown option: ${key}`);
    assert.ok(args[i + 1] && !args[i + 1].startsWith('--'), `Missing value for ${key}`);
    options[key] = args[++i];
  }
}
if (options['--help']) {
  console.log('Usage: node homepage/scripts/check-planner-bank.mjs [--bank DIR] [--output FILE]\nDefaults: homepage/demo/local-assets/planner-bank and homepage/qa/generated/planner-bank.json.');
  process.exit(0);
}
const bankPath = resolve(options['--bank'] || fileURLToPath(new URL('demo/local-assets/planner-bank/', root)));
const output = resolve(options['--output'] || fileURLToPath(new URL('qa/generated/planner-bank.json', root)));
const bytes = async path => new Uint8Array(await readFile(new URL(path, root)));
const json = async path => JSON.parse(await readFile(new URL(path, root), 'utf8'));
const zero = { vx: 0, vy: 0, yaw: 0 };
const limits = { minimumHeight: 0.28, minimumMovement: 0.2, maximumYawDrift: 0.3, maximumStopTailRMS: 0.25, maximumIdleDisplacement: 0.25 };
const report = {
  testedAt: new Date().toISOString(), status: 'running', bankPath,
  host: { runtime: 'Node WebAssembly', node: process.version, platform: process.platform, arch: process.arch },
  scope: 'Closed-loop native G1 planner encoder + H2 decoder + MuJoCo WASM. Direction targets are distinct learned-planner references, not rendered-pose translations.',
  limitations: ['This checks actual simulated state, not hardware or exact target-speed accuracy.', 'No browser UI, worker scheduling, or real-time pacing is exercised.', 'Minimum height and finite-state checks are sampled at the 50 Hz policy boundaries.'],
  limits, sources: {}, cases: [], failures: [],
};
let physics, inference, policy, config, bank;
const started = performance.now();
const angleDifference = (a, b) => Math.atan2(Math.sin(a - b), Math.cos(a - b));
const speed = v => Math.hypot(v[0], v[1]);
const errorDetails = error => ({ name: error.name, message: error.message });
async function source(path) {
  const data = await bytes(path);
  report.sources[path] = { bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return data;
}
async function bankJSON(file) {
  const path = resolve(bankPath, file), data = new Uint8Array(await readFile(path));
  report.sources[`bank/${file}`] = { path, bytes: data.byteLength, sha256: createHash('sha256').update(data).digest('hex') };
  return JSON.parse(new TextDecoder().decode(data));
}
function snapshot() {
  const state = physics.getState();
  for (const key of ['qpos', 'qvel', 'ctrl', 'body_poses']) assert.ok(state[key].every(Number.isFinite), `${key} contains a non-finite value`);
  return { simTime: state.sim_time, position: state.root_position, velocity: state.root_velocity, yaw: yawOf(state.qpos.subarray(3, 7)), mode: policy.mode, style: policy.plannerStyle, status: policy.status };
}
function finishSegment(result, last, began, tail) {
  result.final = last;
  result.simSeconds = last.simTime - result.initial.simTime;
  result.wallSeconds = (performance.now() - began) / 1000;
  result.displacement = last.position.map((value, i) => value - result.initial.position[i]);
  const [x, y] = result.displacement, yaw = result.initial.yaw;
  result.bodyFrameDisplacement = { forward: x * Math.cos(yaw) + y * Math.sin(yaw), left: -x * Math.sin(yaw) + y * Math.cos(yaw) };
  result.horizontalDisplacement = Math.hypot(x, y);
  result.finalOneSecondRMSHorizontalSpeed = tail.length ? Math.sqrt(tail.reduce((sum, v) => sum + v * v, 0) / tail.length) : null;
}
async function segment(test, name, seconds, command = zero) {
  const first = snapshot(), began = performance.now();
  const result = { name, command, requestedSeconds: seconds, initial: first, minRootHeight: first.position[2], maxAbsYawDrift: 0, yawChangeRadians: 0, samples: [first] };
  test.segments.push(result);
  let last = first;
  const tail = [], steps = Math.round(seconds * config.policyHz);
  try {
    for (let i = 0; i < steps; i++) {
      // Renew at each real wall-clock step so slow hosts do not time out.
      policy.command(command);
      const stepBefore = policy.steps, timeBefore = physics.data.time;
      await policy.step();
      const current = snapshot();
      result.minRootHeight = Math.min(result.minRootHeight, current.position[2]);
      result.yawChangeRadians += angleDifference(current.yaw, last.yaw);
      result.maxAbsYawDrift = Math.max(result.maxAbsYawDrift, Math.abs(result.yawChangeRadians));
      last = current;
      if ((i + 1) % config.policyHz === 0) result.samples.push(current);
      if (i >= steps - config.policyHz) tail.push(speed(current.velocity));
      assert.equal(policy.steps, stepBefore + 1, 'Policy did not advance');
      assert.ok(Math.abs(current.simTime - timeBefore - 1 / config.policyHz) < 1e-9, 'Physics did not advance one policy interval');
      assert.ok(policy.lastAction.every(Number.isFinite), 'Policy action is not finite');
      assert.notEqual(policy.status, 'fallen', 'Controller detected a fall');
      assert.ok(current.position[2] >= limits.minimumHeight, 'Root height is below the fall threshold');
    }
    finishSegment(result, last, began, tail);
    result.status = 'pass';
    return result;
  } catch (error) {
    finishSegment(result, last, began, tail);
    result.status = 'fail'; result.error = errorDetails(error);
    throw error;
  }
}
function assertStop(result) {
  assert.ok(result.finalOneSecondRMSHorizontalSpeed < limits.maximumStopTailRMS, `Stop RMS ${result.finalOneSecondRMSHorizontalSpeed} >= ${limits.maximumStopTailRMS} m/s`);
}
function assertDirection(result, direction) {
  const { forward, left } = result.bodyFrameDisplacement;
  if (direction === 'left' || direction === 'right') {
    const signed = left * (direction === 'left' ? 1 : -1);
    assert.ok(signed > limits.minimumMovement, `Incorrect/insufficient ${direction} displacement: ${left} m`);
    assert.ok(signed > Math.abs(forward), `Lateral ${signed} m does not dominate forward ${forward} m`);
  } else {
    const signed = forward * (direction === 'forward' ? 1 : -1);
    assert.ok(signed > limits.minimumMovement, `Incorrect/insufficient ${direction} displacement: ${forward} m`);
    assert.ok(signed > Math.abs(left), `${direction} ${signed} m does not dominate lateral ${left} m`);
  }
  assert.ok(result.maxAbsYawDrift < limits.maximumYawDrift, `Heading drift ${result.maxAbsYawDrift} rad: movement must preserve facing`);
}
async function check(name, fn) {
  const test = { name, segments: [] };
  report.cases.push(test);
  try {
    await fn(test);
    test.status = 'pass';
    console.log(`PASS ${name}`);
  } catch (error) {
    test.status = 'fail'; test.error = errorDetails(error);
    report.failures.push({ test: name, ...errorDetails(error) });
    console.error(`FAIL ${name}: ${error.message}`);
  }
}
function resetPlanner(style) {
  policy.command({ mode: 'planner', plannerStyle: style, motion: 'squat_001__A359', action: 'reset' });
  assert.equal(policy.mode, 'planner');
  assert.equal(policy.plannerStyle, style);
}

try {
  for (const path of ['scripts/check-planner-bank.mjs', 'demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-physics.js', 'demo/local-inference.js', 'demo/vendor/mujoco/mujoco.wasm', 'demo/vendor/ort/ort-wasm-simd-threaded.wasm', 'demo/local-assets/controller.json', 'demo/local-assets/physics/model.xml', 'demo/local-assets/physics/physics-config.json']) await source(path);
  config = await json('demo/local-assets/controller.json');
  bank = await bankJSON('manifest.json');
  config.planner.bank = bank;
  assert.equal(config.policyHz, 50);
  const manifest = await json('demo/local-assets/physics/manifest.json');
  physics = await createPhysics({ manifest, config: await json('demo/local-assets/physics/physics-config.json'),
    files: await Promise.all(manifest.files.map(async file => ({ path: file.path, data: await bytes(`demo/local-assets/physics/${file.url || file.path}`) }))),
    engineOptions: { wasmBinary: await bytes('demo/vendor/mujoco/mujoco.wasm') },
  });
  inference = await createLocalInference({ provider: 'wasm', models: {
    encoderFull: await source('demo/local-assets/models/encoder_full.onnx'),
    decoder: await source('demo/local-assets/models/decoder.onnx')
  } });
  report.engines = { mujoco: physics.mujoco.mj_versionString(), onnxRuntime: inference.runtimeVersion, provider: inference.provider };
  const motions = {};
  for (const entry of plannerEntries(bank)) {
      motions[entry.id] = await bankJSON(entry.file);
      assert.ok(motions[entry.id].q29G1 && motions[entry.id].qd29G1);
  }
  const tracking = JSON.parse(new TextDecoder().decode(await source('demo/local-assets/references/squat_001__A359.json')));
  motions[tracking.id] = prepareTrackingReference(tracking);
  policy = new LocalPolicy(physics, inference, config, motions);
  policy.motion = tracking.id;

  await check('idle', async test => {
    resetPlanner('idle');
    await segment(test, 'stand', 1);
    const result = await segment(test, 'idle', 7);
    assert.ok(result.horizontalDisplacement < limits.maximumIdleDisplacement, 'Idle drift exceeded tolerance');
    assertStop(await segment(test, 'stop', 3));
  });
  for (const style of ['slow_walk', 'walk', 'run']) {
    const velocity = bank.styles[style].speed;
    for (const direction of ['forward', 'back', 'left', 'right']) {
      await check(`${style}/${direction}`, async test => {
        resetPlanner(style);
        await segment(test, 'stand', 1);
        const command = { vx: direction === 'forward' ? velocity : direction === 'back' ? -velocity : 0, vy: direction === 'left' ? velocity : direction === 'right' ? -velocity : 0, yaw: 0 };
        const movement = await segment(test, 'move', 7, command);
        // Test stopping even if the movement's direction assertion will fail.
        const stopped = await segment(test, 'stop', 3);
        assertDirection(movement, direction);
        assertStop(stopped);
      });
    }
  }
  await check('continuous_style_switches', async test => {
    resetPlanner('slow_walk');
    await segment(test, 'stand', 1);
    for (const style of ['slow_walk', 'walk', 'run', 'idle']) {
      const before = snapshot(), steps = policy.steps;
      policy.command({ plannerStyle: style });
      assert.equal(physics.data.time, before.simTime, 'Style change reset physics time');
      assert.equal(policy.steps, steps, 'Style change reset policy history/step count');
      assert.equal(policy.plannerStyle, style);
      const result = await segment(test, style, style === 'idle' ? 3 : 5, { vx: bank.styles[style].speed, vy: 0, yaw: 0 });
      if (style === 'idle') assertStop(result);
    }
  });
  await check('tracking_planner_round_trip', async test => {
    policy.command({ mode: 'tracking', motion: tracking.id, action: 'reset' });
    assert.equal(policy.mode, 'tracking');
    await segment(test, 'tracking_before', 3);
    policy.command({ mode: 'planner', plannerStyle: 'slow_walk', action: 'start' });
    assert.equal(policy.mode, 'planner');
    await segment(test, 'planner_middle', 3, { vx: bank.styles.slow_walk.speed, vy: 0, yaw: 0 });
    policy.command({ mode: 'tracking', motion: tracking.id, action: 'start' });
    assert.equal(policy.mode, 'tracking');
    await segment(test, 'tracking_after', 3);
    policy.command({ mode: 'planner', plannerStyle: 'idle', action: 'start' });
    assert.equal(policy.mode, 'planner');
    assertStop(await segment(test, 'planner_after', 3));
  });
  assert.equal(report.cases.length, 15);
  // Detect a concurrent policy edit instead of attributing this run to new code.
  for (const path of ['demo/local-policy.js', 'demo/local-reference.js', 'demo/local-planner.js', 'demo/local-inference.js']) {
    assert.equal(createHash('sha256').update(await bytes(path)).digest('hex'), report.sources[path].sha256, `${path} changed during the run`);
  }
} catch (error) {
  report.failures.push({ test: 'setup_or_run', ...errorDetails(error) });
  console.error(error.stack || error);
} finally {
  try { physics?.dispose(); } catch (error) { report.failures.push({ test: 'physics_cleanup', ...errorDetails(error) }); }
  try { await inference?.release(); } catch (error) { report.failures.push({ test: 'inference_cleanup', ...errorDetails(error) }); }
  report.status = report.failures.length ? 'fail' : 'pass';
  report.passedCases = report.cases.filter(test => test.status === 'pass').length;
  report.totalSimSeconds = report.cases.reduce((sum, test) => sum + test.segments.reduce((subtotal, item) => subtotal + (item.simSeconds || 0), 0), 0);
  report.wallSeconds = (performance.now() - started) / 1000;
  report.completedAt = new Date().toISOString();
  await mkdir(dirname(output), { recursive: true });
  await writeFile(output, JSON.stringify(report, null, 2) + '\n');
  console.log(`${report.status.toUpperCase()}: saved ${output}`);
  if (report.failures.length) process.exitCode = 1;
}
