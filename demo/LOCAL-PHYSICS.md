# Browser physics runtime

`local-physics.js` loads the vendored, official MuJoCo 3.10.0 WebAssembly engine
and the exported H2 closed-loop model. It is an ES module that can run directly
inside the inference worker; it does not need a server process or CDN.

```js
import { createPhysics } from './local-physics.js';
const physics = await createPhysics();
physics.stepPD(targetAnglesLocal31, kpLocal31, kdLocal31, 20);
const snapshot = physics.getState();
physics.reset();
physics.dispose();
```

`stepPD` applies the deployment PD equation at every 1 ms physics step, then maps
the logical output torques to physical motors through all five closed-loop
transmissions (ankles, knees, waist). The mapping uses the current MuJoCo equality
constraint Jacobian, a minimum-norm SVD solve, and the deployment torque limits.
Both dense and sparse MuJoCo constraint layouts are supported. No root position
or joint position is assigned during stepping; MuJoCo advances dynamics and
contacts from applied motor torques. The exported profile has no optional motor
response or ankle response filter enabled.

`physics.data`, `physics.model`, and `physics.mujoco` expose the original WASM API
for controller observations. Reacquire typed-array views after stepping because
WASM memory can grow. `getState()` copies the 68 positions, 61 velocities, 31
motor commands, and 45 body poses. Body poses are ordered as the exported model
and use `[x,y,z,qw,qx,qy,qz]`, with world Z up. The optional `step(n, ctrl)` API
applies direct motor torque control instead of PD.

The deployment exporter saves its initialized, calibrated qpos and resolved
joint/actuator maps in `local-assets/physics/physics-config.json`. `model.xml`
includes the active solver and passive-parameter settings. Reset uses that
calibrated state and calls MuJoCo forward dynamics.

Run `node homepage/scripts/check-local-physics.mjs` from the repository root.
It executes the same WASM binary under Node and records real gravity/contact,
reset, body-order, dense/sparse, Python transmission parity, torque-limit, and
timing checks in `homepage/qa/local-physics.json`. Browser and ONNX integration
must be validated separately. A static pose PD test alone is not a balance-policy
test.

Upstream API and threading details are documented in the
[official MuJoCo bindings guide](https://github.com/google-deepmind/mujoco/blob/3.10.0/wasm/README.md).
The local vendor directory contains the Apache 2.0 license and package provenance.
