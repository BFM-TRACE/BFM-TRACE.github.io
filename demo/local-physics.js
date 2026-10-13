import loadMujoco from './vendor/mujoco/mujoco.js';

// One-sided Jacobi SVD. This computes the minimum-norm solution without forming
// AᵀA, preserving the small singular values of the H2 linkage constraints.
export function leastSquares(A, B, rows, cols, rhs, rcond = 1e-15) {
  const W = Float64Array.from(A);
  const V = new Float64Array(cols * cols);
  for (let j = 0; j < cols; j++) V[j * cols + j] = 1;
  for (let sweep = 0; sweep < 80; sweep++) {
    let changed = false;
    for (let p = 0; p < cols; p++) for (let q = p + 1; q < cols; q++) {
      let a = 0, b = 0, cross = 0;
      for (let i = 0; i < rows; i++) {
        const x = W[i * cols + p], y = W[i * cols + q];
        a += x * x; b += y * y; cross += x * y;
      }
      if (!a || !b || Math.abs(cross) <= 2e-15 * Math.sqrt(a * b)) continue;
      const z = (b - a) / (2 * cross);
      const t = (z >= 0 ? 1 : -1) / (Math.abs(z) + Math.hypot(1, z));
      const c = 1 / Math.hypot(1, t), s = c * t;
      for (let i = 0; i < rows; i++) {
        const x = W[i * cols + p], y = W[i * cols + q];
        W[i * cols + p] = c * x - s * y;
        W[i * cols + q] = s * x + c * y;
      }
      for (let i = 0; i < cols; i++) {
        const x = V[i * cols + p], y = V[i * cols + q];
        V[i * cols + p] = c * x - s * y;
        V[i * cols + q] = s * x + c * y;
      }
      changed = true;
    }
    if (!changed) break;
  }
  const norms = new Float64Array(cols);
  let maximum = 0;
  for (let j = 0; j < cols; j++) {
    for (let i = 0; i < rows; i++) norms[j] += W[i * cols + j] ** 2;
    maximum = Math.max(maximum, norms[j]);
  }
  const X = new Float64Array(cols * rhs);
  let rank = 0;
  for (let j = 0; j < cols; j++) {
    if (norms[j] <= maximum * rcond * rcond || norms[j] === 0) continue;
    rank++;
    for (let k = 0; k < rhs; k++) {
      let projection = 0;
      for (let i = 0; i < rows; i++) projection += W[i * cols + j] * B[i * rhs + k];
      projection /= norms[j];
      for (let i = 0; i < cols; i++) X[i * rhs + k] += V[i * cols + j] * projection;
    }
  }
  let residualSquared = 0;
  for (let i = 0; i < rows; i++) for (let k = 0; k < rhs; k++) {
    let value = -B[i * rhs + k];
    for (let j = 0; j < cols; j++) value += A[i * cols + j] * X[j * rhs + k];
    residualSquared += value * value;
  }
  return { solution: X, rank, residual: Math.sqrt(residualSquared) };
}

let modelSerial = 0;
function mkdirTree(FS, path) {
  let prefix = '';
  for (const component of path.split('/').filter(Boolean)) {
    prefix += '/' + component;
    if (!FS.analyzePath(prefix).exists) FS.mkdir(prefix);
  }
}
function assertVector(values, length, label) {
  if (!values || values.length !== length || !Array.from(values).every(Number.isFinite)) {
    throw new Error(`${label} must contain ${length} finite numbers`);
  }
}
async function fetchChecked(url, binary = false) {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`Could not load ${url}: HTTP ${response.status}`);
  return binary ? new Uint8Array(await response.arrayBuffer()) : response.json();
}

/**
 * Load the actual MuJoCo model into its local WASM filesystem. Import this module
 * from the controller worker so physics and inference share live state without
 * serializing each 1 ms physics step. No animation or reference data is used.
 *
 * Tests can supply {xml, files:[{path,data}], config, engineOptions:{wasmBinary}}.
 */
export async function createPhysics(options = {}) {
  const manifestUrl = new URL(options.manifestUrl || './local-assets/physics/manifest.json', import.meta.url);
  const manifest = options.manifest || (options.xml ? { entry: 'model.xml', files: [] } : await fetchChecked(manifestUrl));
  const config = options.config || (manifest.config ? await fetchChecked(new URL(manifest.config, manifestUrl)) : {});
  const mj = options.mujoco || await loadMujoco({
    locateFile: file => new URL(`./vendor/mujoco/${file}`, import.meta.url).href,
    ...options.engineOptions,
  });
  const directory = `/trace-model-${++modelSerial}`;
  mkdirTree(mj.FS, directory);
  const files = options.files || manifest.files || [];
  await Promise.all(files.map(async entry => {
    const file = typeof entry === 'string' ? { path: entry, url: entry } : entry;
    if (file.path.startsWith('/') || file.path.split('/').includes('..')) throw new Error(`Invalid model asset path: ${file.path}`);
    const content = file.data ?? await fetchChecked(new URL(file.url || file.path, manifestUrl), true);
    const path = `${directory}/${file.path}`;
    mkdirTree(mj.FS, path.slice(0, path.lastIndexOf('/')));
    mj.FS.writeFile(path, content);
  }));
  if (options.xml) mj.FS.writeFile(`${directory}/${manifest.entry || 'model.xml'}`, options.xml);
  const model = mj.MjModel.from_xml_path(`${directory}/${manifest.entry || 'model.xml'}`);
  if (!model) throw new Error('MuJoCo could not compile the H2 model');
  const data = new mj.MjData(model);
  return new LocalPhysics(mj, model, data, config, directory);
}

export class LocalPhysics {
  constructor(mujoco, model, data, config, directory) {
    this.mujoco = mujoco;
    this.model = model;
    this.data = data;
    this.config = {
      ...config,
      default_qpos: config.default_qpos || config.initialQpos,
      default_qvel: config.default_qvel || config.initialQvel,
      kp: config.kp || config.robotConfig?.MOTOR_KP,
      kd: config.kd || config.robotConfig?.MOTOR_KD,
    };
    this.directory = directory;
    this.disposed = false;
    this.lastPD = null;
    for (const [name, value] of Object.entries(config.options || {})) {
      if (Array.isArray(value)) model.opt[name].set(value);
      else model.opt[name] = value;
    }
    for (const [name, value] of Object.entries(config.model_arrays || {})) {
      if (!model[name]?.set || model[name].length !== value.length) throw new Error(`Invalid model array override: ${name}`);
      model[name].set(value);
    }
    if (config.motor_response_enabled || config.ankle_response_enabled) {
      throw new Error('This physics export requires an unsupported motor response model');
    }
    this.bodyNames = Array.from({ length: model.nbody }, (_, i) => mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY.value, i) || `body_${i}`);
    this.reset();
  }

  reset({ qpos = this.config.default_qpos, qvel = this.config.default_qvel } = {}) {
    this.ensureAlive();
    this.mujoco.mj_resetData(this.model, this.data);
    if (qpos) { assertVector(qpos, this.model.nq, 'qpos'); this.data.qpos.set(qpos); }
    if (qvel) { assertVector(qvel, this.model.nv, 'qvel'); this.data.qvel.set(qvel); }
    this.mujoco.mj_forward(this.model, this.data);
    this.lastPD = null;
    return this.getState();
  }

  ensureAlive() { if (this.disposed) throw new Error('MuJoCo physics has been disposed'); }
  setControl(control) {
    this.ensureAlive();
    assertVector(control, this.model.nu, 'control');
    this.data.ctrl.set(control);
  }
  step(steps = 1, control) {
    this.ensureAlive();
    if (!Number.isInteger(steps) || steps < 0 || steps > 100000) throw new Error('Invalid physics step count');
    if (control) this.setControl(control);
    for (let i = 0; i < steps; i++) this.mujoco.mj_step(this.model, this.data);
    return this.data.time;
  }

  constraintBlock(rows, columns) {
    const d = this.data, result = new Float64Array(rows.length * columns.length);
    const jacobian = d.efc_J;
    if (this.mujoco.mj_isSparse(this.model)) {
      const rowAddress = d.efc_J_rowadr, rowCount = d.efc_J_rownnz, columnIndex = d.efc_J_colind;
      const destinations = new Map(columns.map((column, i) => [column, i]));
      for (let i = 0; i < rows.length; i++) {
        const row = rows[i], end = rowAddress[row] + rowCount[row];
        for (let k = rowAddress[row]; k < end; k++) {
          const dest = destinations.get(columnIndex[k]);
          if (dest !== undefined) result[i * columns.length + dest] = jacobian[k];
        }
      }
    } else {
      for (let i = 0; i < rows.length; i++) for (let j = 0; j < columns.length; j++) {
        result[i * columns.length + j] = jacobian[rows[i] * this.model.nv + columns[j]];
      }
    }
    return result;
  }

  transmissionJacobian(group) {
    const equalities = new Set(group.equality_ids), rows = [];
    const types = this.data.efc_type, ids = this.data.efc_id;
    for (let i = 0; i < this.data.nefc; i++) if (types[i] === 0 && equalities.has(ids[i])) rows.push(i);
    if (!rows.length) throw new Error(`${group.name}: no active linkage equality constraints`);
    const dependentDofs = [...group.dependent_output_dofs, ...group.auxiliary_dofs];
    const A = this.constraintBlock(rows, dependentDofs);
    const B = this.constraintBlock(rows, group.motor_dofs);
    for (let i = 0; i < B.length; i++) B[i] = -B[i];
    const count = group.motor_dofs.length;
    const result = leastSquares(A, B, rows.length, dependentDofs.length, count);
    const J = new Float64Array(group.output_dofs.length * count);
    for (let i = 0; i < group.output_dofs.length; i++) {
      const dof = group.output_dofs[i], motor = group.motor_dofs.indexOf(dof);
      if (motor >= 0) J[i * count + motor] = 1;
      else {
        const dependent = group.dependent_output_dofs.indexOf(dof);
        if (dependent < 0) throw new Error(`${group.name}: missing dependent output DOF`);
        J.set(result.solution.subarray(dependent * count, (dependent + 1) * count), i * count);
      }
    }
    const rank = leastSquares(J, new Float64Array(J.length), group.output_dofs.length, count, count, Math.max(count, group.output_dofs.length) * Number.EPSILON).rank;
    if (!Number.isFinite(result.residual) || result.residual > 1e-7 || rank < group.output_dofs.length) {
      throw new Error(`${group.name}: singular closed-loop transmission (residual ${result.residual})`);
    }
    return J;
  }

  mapBodyTorques(bodyTorques) {
    const motorTorques = Float64Array.from(bodyTorques);
    for (const group of this.config.closed_loop_transmissions || []) {
      const J = this.transmissionJacobian(group), n = group.motor_positions.length;
      for (let j = 0; j < n; j++) {
        let torque = 0;
        for (let i = 0; i < group.output_positions.length; i++) torque += J[i * n + j] * bodyTorques[group.output_positions[i]];
        motorTorques[group.motor_positions[j]] = torque;
      }
    }
    return motorTorques;
  }

  /** Targets and gains are in H2 local/training joint order, as on the server. */
  stepPD(target, kp = this.config.kp, kd = this.config.kd, steps = 20) {
    this.ensureAlive();
    const c = this.config, count = c.body_qpos_index?.length;
    if (!count) throw new Error('This model export has no PD actuator mapping');
    assertVector(target, count, 'PD target');
    assertVector(kp, count, 'PD kp');
    assertVector(kd, count, 'PD kd');
    if (!Number.isInteger(steps) || steps < 0 || steps > 100000) throw new Error('Invalid physics step count');
    const body = new Float64Array(count);
    for (let step = 0; step < steps; step++) {
      // Reacquire heap views: MuJoCo may grow WASM memory during a step.
      const qpos = this.data.qpos, qvel = this.data.qvel;
      for (let local = 0; local < count; local++) {
        const sim = c.local_to_sim ? c.local_to_sim[local] : local;
        body[sim] = kp[local] * (target[local] - qpos[c.body_qpos_index[sim]]) - kd[local] * qvel[c.body_dof_index[sim]];
      }
      const motors = this.mapBodyTorques(body), ctrl = this.data.ctrl;
      ctrl.fill(0);
      for (let sim = 0; sim < count; sim++) {
        const actuator = c.body_actuator_index[sim], limit = c.torque_limit?.[actuator] ?? Infinity;
        ctrl[actuator] = Math.max(-limit, Math.min(limit, motors[sim]));
      }
      this.mujoco.mj_step(this.model, this.data);
      this.lastPD = { body, motors };
    }
    return this.data.time;
  }

  getState() {
    this.ensureAlive();
    // mj_step reports qpos at the new time, but xpos is from its last dynamics
    // evaluation. Update kinematics only; do not disturb solver warm starts.
    this.mujoco.mj_kinematics(this.model, this.data);
    const xpos = this.data.xpos, xquat = this.data.xquat;
    const poses = new Float64Array(this.model.nbody * 7);
    for (let i = 0; i < this.model.nbody; i++) {
      poses.set(xpos.subarray(i * 3, i * 3 + 3), i * 7);
      poses.set(xquat.subarray(i * 4, i * 4 + 4), i * 7 + 3);
    }
    return {
      source: 'local_mujoco_policy', engine: `MuJoCo ${this.mujoco.mj_versionString()} WASM`,
      sim_time: this.data.time, timestep: this.model.opt.timestep,
      qpos: this.data.qpos.slice(), qvel: this.data.qvel.slice(), ctrl: this.data.ctrl.slice(),
      body_poses: poses, body_names: this.bodyNames,
      root_position: Array.from(this.data.qpos.subarray(0, 3)),
      root_velocity: Array.from(this.data.qvel.subarray(0, 3)),
    };
  }

  dispose() {
    if (this.disposed) return;
    this.data.delete(); this.model.delete(); this.disposed = true;
    const remove = path => {
      for (const name of this.mujoco.FS.readdir(path)) {
        if (name === '.' || name === '..') continue;
        const child = `${path}/${name}`;
        if (this.mujoco.FS.isDir(this.mujoco.FS.stat(child).mode)) remove(child);
        else this.mujoco.FS.unlink(child);
      }
      this.mujoco.FS.rmdir(path);
    };
    remove(this.directory);
  }
}
