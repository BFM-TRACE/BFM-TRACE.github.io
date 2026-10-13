import assert from 'node:assert/strict';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { createPhysics, leastSquares } from '../demo/local-physics.js';

// Uses exactly the browser's official WASM binary, under Node's WebAssembly
// host. Browser/controller integration is checked separately in the real UI.
const root = new URL('../', import.meta.url);
const assetBase = new URL('demo/local-assets/physics/', root);
const wasmBinary = await readFile(new URL('demo/vendor/mujoco/mujoco.wasm', root));
const report = { testedAt: new Date().toISOString(), host: 'Node WebAssembly', checks: [] };
const check = (name, details) => report.checks.push({ name, status: 'pass', ...details });
const ball = await createPhysics({
  engineOptions: { wasmBinary },
  xml: '<mujoco><option timestep="0.001"/><worldbody><geom type="plane" size="10 10 .1"/><body name="ball" pos="0 0 1"><freejoint/><geom type="sphere" size=".1" mass="1"/></body></worldbody></mujoco>',
});
ball.step(100);
assert.ok(Math.abs(ball.data.qvel[2] + 0.981) < 1e-12);
assert.ok(Math.abs(ball.data.qpos[2] - 0.9504595) < 1e-12);
check('Real gravity integration', { simTime: ball.data.time, height: ball.data.qpos[2], velocity: ball.data.qvel[2] });
ball.step(2000);
assert.ok(Math.abs(ball.data.qpos[2] - 0.1) < 0.001);
assert.ok(Math.abs(ball.data.qvel[2]) < 1e-8);
check('Real ground contact', { simTime: ball.data.time, height: ball.data.qpos[2] });
ball.reset();
assert.equal(ball.data.time, 0);
assert.equal(ball.data.qpos[2], 1);
check('Reset restores physical state');
ball.dispose();

const underdetermined = leastSquares([1, 1], [1], 1, 2, 1);
assert.ok(underdetermined.solution.every(value => Math.abs(value - 0.5) < 1e-12));
const rankDeficient = leastSquares([1, 2, 2, 4], [3, 6], 2, 2, 1);
assert.equal(rankDeficient.rank, 1);
assert.ok(Math.abs(rankDeficient.solution[0] - 0.6) < 1e-12);
assert.ok(Math.abs(rankDeficient.solution[1] - 1.2) < 1e-12);
check('Minimum-norm SVD, underdetermined and rank-deficient constraints');

const manifest = JSON.parse(await readFile(new URL('manifest.json', assetBase)));
const config = JSON.parse(await readFile(new URL('physics-config.json', assetBase)));
const files = await Promise.all(manifest.files.map(async f => ({ ...f, data: new Uint8Array(await readFile(new URL(f.path, assetBase))) })));
const physics = await createPhysics({ manifest, config, files, engineOptions: { wasmBinary } });
report.engine = physics.mujoco.mj_versionString();
assert.equal(report.engine, config.mujocoVersion);
assert.equal(physics.model.nq, config.nq);
assert.equal(physics.model.nv, config.nv);
assert.equal(physics.model.nu, config.nu);
assert.deepEqual(physics.bodyNames, config.bodyNames);
check('Full H2 model and renderer body order', { nq: physics.model.nq, nv: physics.model.nv, nu: physics.model.nu, bodies: physics.model.nbody });

const fixtures = JSON.parse(await readFile(new URL('transmission-regression.json', assetBase)));
let maximumJacobianError = 0, maximumTorqueError = 0;
for (const fixture of fixtures) {
  physics.reset({ qpos: fixture.qpos });
  for (const group of config.closed_loop_transmissions) {
    const actual = physics.transmissionJacobian(group);
    const expected = fixture.jacobians.find(value => value.name === group.name).J.flat();
    for (let i = 0; i < actual.length; i++) maximumJacobianError = Math.max(maximumJacobianError, Math.abs(actual[i] - expected[i]));
  }
  const torques = physics.mapBodyTorques(fixture.bodyTorques);
  for (let i = 0; i < torques.length; i++) maximumTorqueError = Math.max(maximumTorqueError, Math.abs(torques[i] - fixture.actuatorTorques[i]));
}
// Python fixtures use the exact saved XML also loaded by WASM.
assert.ok(maximumJacobianError < 1e-10, `J error: ${maximumJacobianError}`);
assert.ok(maximumTorqueError < 1e-10, `torque error: ${maximumTorqueError}`);
check('Five closed-loop transmissions match Python fixtures', { fixtures: fixtures.length, maximumJacobianError, maximumTorqueError, tolerance: 1e-10 });

physics.reset();
const sparseTorques = physics.mapBodyTorques(fixtures[0].bodyTorques);
physics.model.opt.jacobian = 0; // mjJAC_DENSE
physics.mujoco.mj_forward(physics.model, physics.data);
const denseTorques = physics.mapBodyTorques(fixtures[0].bodyTorques);
const denseSparseError = Math.max(...denseTorques.map((x, i) => Math.abs(x - sparseTorques[i])));
assert.ok(denseSparseError < 1e-10);
check('Dense and sparse constraint storage agree', { maximumTorqueError: denseSparseError });
physics.model.opt.jacobian = 2; // mjJAC_AUTO, as in the exported model

const target = config.localQposAddresses.map(index => config.initialQpos[index]);
const timings = [];
for (let i = 0; i < 50; i++) {
  physics.reset();
  const start = performance.now();
  physics.stepPD(target, undefined, undefined, 20);
  timings.push(performance.now() - start);
  assert.ok(physics.data.qpos.every(Number.isFinite));
  assert.ok(Math.abs(physics.data.time - 0.02) < 1e-12);
  assert.ok(physics.data.ctrl.every((value, j) => Math.abs(value) <= config.torque_limit[j]));
}
timings.sort((a, b) => a - b);
assert.equal(physics.getState().body_poses.length, 315);
check('20 real PD steps produce finite, torque-limited state', { trials: timings.length, p50Milliseconds: timings[25], p95Milliseconds: timings[47], simulatedMilliseconds: 20 });
physics.dispose();
report.status = 'pass';
await mkdir(new URL('qa/generated/', root), { recursive: true });
const output = new URL('qa/generated/local-physics.json', root);
await writeFile(output, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report, null, 2));
console.log(`Saved ${fileURLToPath(output)}`);
