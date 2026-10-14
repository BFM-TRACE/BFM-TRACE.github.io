import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';
import { LocalSimulation } from './local-client.js';

const $ = (id) => document.getElementById(id);
const ui = { stage: $('stage'), loading: $('loading'), motion: $('motion'), timeline: $('timeline') };
const state = { mode: 'tracking', playing: false, referenceOnly: false, connectionAttempt: 0,
  speed: 1, time: 0, live: false, reachable: false, follow: true, motion: null, frames: null,
  vx: 0, vy: 0, yaw: 0, frame: null, lastLive: 0, lastProbe: -Infinity, busy: false,
  liveStatus: null, liveConfirmed: false, snapshotId: null, actualMode: null, failed: false,
  pendingMode: null, plannerStyle: 'slow_walk', pendingPlannerStyle: null, motionPreparing: false };
const plannerStyles = {
  idle: {min:0,max:0,speed:0,directionalMax:0,turnMax:1},
  slow_walk: {min:.1,max:.8,speed:.4,directionalMax:.8,turnMax:1},
  walk: {min:.8,max:2.5,speed:.9,directionalMax:1.6,turnMax:1},
  run: {min:2.5,max:4,speed:2.5,directionalMax:2.5,turnMax:.3},
};
const heldKeys = new Set(), heldPointers = new Map();
const keyDirections = {KeyW:'forward',KeyA:'left',KeyS:'back',KeyD:'right',KeyQ:'turn-left',KeyE:'turn-right',ArrowUp:'forward',ArrowLeft:'left',ArrowDown:'back',ArrowRight:'right'};
let sendTimer, commandGeneration = 0;
let renderer, scene, camera, orbit, ground, keyLight, bodies = [], manifest, model;
const frameCache = new Map();
const rootPrevious = new THREE.Vector3();
const q = new THREE.Quaternion();
const qNext = new THREE.Quaternion();
let loadSequence = 0;
const localSimulation = new LocalSimulation(message => {
  if (!state.busy) return;
  $('runtime-note').textContent = message;
  $('hint-detail').textContent = message;
});

function resize() {
  if (!renderer) return;
  const width = ui.stage.clientWidth, height = ui.stage.clientHeight;
  renderer.setSize(width, height, false);
  camera.aspect = width / height;
  camera.updateProjectionMatrix();
}
function resetCamera() {
  const target = bodies[1]?.position || new THREE.Vector3();
  orbit.target.set(target.x, target.y, .85);
  camera.position.set(target.x + 3.1, target.y - 4.5, 2.35);
  orbit.update();
}
function createGround() {
  // One texture period is 1 m, with 0.5 m tiles. Generate it locally so the
  // portable homepage does not need another asset or a network dependency.
  const tile = document.createElement('canvas'); tile.width = tile.height = 64;
  const context = tile.getContext('2d');
  context.fillStyle = '#9aaab5'; context.fillRect(0,0,64,64);
  context.fillStyle = '#b5c1c9'; context.fillRect(0,0,32,32); context.fillRect(32,32,32,32);
  const texture = new THREE.CanvasTexture(tile);
  texture.colorSpace = THREE.SRGBColorSpace;
  texture.wrapS = texture.wrapT = THREE.RepeatWrapping;
  texture.repeat.set(120,120);
  texture.anisotropy = Math.min(8,renderer.capabilities.getMaxAnisotropy());
  ground = new THREE.Group();
  const floor = new THREE.Mesh(new THREE.PlaneGeometry(120,120),new THREE.MeshStandardMaterial({map:texture,roughness:1}));
  floor.receiveShadow = true; floor.position.z = -.007; ground.add(floor);
  for (const [divisions,color,opacity,z] of [[120,0x718594,.35,0],[30,0x647c8d,.55,.001]]) {
    const grid = new THREE.GridHelper(60,divisions,color,color);
    grid.rotation.x = Math.PI/2; grid.position.z = z;
    grid.material.transparent = true; grid.material.opacity = opacity;
    grid.material.depthWrite = false; grid.material.toneMapped = false;
    ground.add(grid);
  }
  scene.add(ground);
}
function updateGround() {
  // Move only by full major-grid periods: nearby world lines and checker
  // tiles remain stationary even during camera follow or long planner runs.
  ground.position.set(Math.round(orbit.target.x/2)*2,Math.round(orbit.target.y/2)*2,0);
  const root = bodies[1].position;
  keyLight.position.set(root.x+3,root.y-5,7);
  keyLight.target.position.set(root.x,root.y,0);
}
function setConnection(label, online) {
  $('connection').classList.toggle('online', online);
  $('connection').lastElementChild.textContent = label;
}
function renderPose(a, b = a, mix = 0) {
  if (!a || a.length !== bodies.length * 7) return;
  for (let i = 0; i < bodies.length; i++) {
    const p = i * 7;
    bodies[i].position.set(a[p] + (b[p] - a[p]) * mix, a[p+1] + (b[p+1] - a[p+1]) * mix, a[p+2] + (b[p+2] - a[p+2]) * mix);
    q.set(a[p+4], a[p+5], a[p+6], a[p+3]);
    qNext.set(b[p+4], b[p+5], b[p+6], b[p+3]);
    bodies[i].quaternion.copy(q.slerp(qNext, mix));
  }
  const root = bodies[1].position;
  if (state.follow) {
    const dx = root.x - rootPrevious.x, dy = root.y - rootPrevious.y;
    camera.position.x += dx; camera.position.y += dy;
    orbit.target.x += dx; orbit.target.y += dy;
  }
  rootPrevious.copy(root);
}

async function loadMotion(id) {
  const sequence = ++loadSequence;
  const motion = manifest.motions.find((entry) => entry.id === id);
  if (!motion) return;
  if (state.referenceOnly && !hasReferencePreview(motion)) {ui.motion.value=state.motion.id; return;}
  const preview = hasReferencePreview(motion);
  if (preview && !frameCache.has(id)) {
    const response = await fetch(`assets/${motion.file}`);
    if (!response.ok) throw new Error('The motion data could not be loaded.');
    frameCache.set(id, new Float32Array(await response.arrayBuffer()));
  }
  if (sequence !== loadSequence) return;
  state.motion = motion; state.frames = preview ? frameCache.get(id) : null; state.time = 0;
  ui.motion.value = id;
  $('motion-duration').textContent = `${motion.duration.toFixed(1)} s`;
  $('motion-source-rate').textContent = `${motion.sourceFPS || 50} Hz source`;
  if (!state.live && state.frames) { renderPose(state.frames.subarray(0, bodies.length*7)); resetCamera(); }
  syncControls();
  if (controlReady() && state.mode === 'tracking' && await command({motion:id, action:'reset'})) {
    state.playing = true; $('play').textContent = 'Pause';
  }
}

function hasReferencePreview(motion = state.motion) {
  return !!motion?.file && motion.previewAvailable !== false;
}

async function request(path, options = {}) {
  return localSimulation.request(path,options.body ? JSON.parse(options.body) : undefined);
}
async function command(payload) {
  try {
    const result = await request('command', {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(payload)});
    if (result.error) throw new Error(result.error);
    return true;
  } catch (error) {
    state.error = error.message;
    $('runtime-note').textContent = error.message;
    return false;
  }
}
function controlReady() {
  return state.live && state.liveConfirmed && !state.pendingMode && !state.pendingPlannerStyle && ['running','paused'].includes(state.liveStatus);
}
function syncControls() {
  const ready = controlReady();
  document.querySelectorAll('.mode').forEach(button => {button.disabled = !ready;});
  $('planner-style').disabled = !ready;
  ui.motion.disabled = !(ready || state.referenceOnly);
  $('play').disabled = !(ready || state.referenceOnly);
  $('reset').disabled = !(state.liveConfirmed || state.referenceOnly);
  $('velocity').disabled = !ready || state.plannerStyle === 'idle';
  $('turn-speed').disabled = !ready || state.mode !== 'planner' || state.referenceOnly || state.plannerStyle === 'idle';
  $('planner-reset').disabled = !state.liveConfirmed;
  document.querySelectorAll('[data-direction]').forEach(button => {button.disabled = !ready || state.plannerStyle === 'idle';});
  $('reference-timeline').hidden = !state.referenceOnly;
  $('reference-speed').hidden = !state.referenceOnly;
  $('policy-caption').hidden = state.referenceOnly || !state.liveConfirmed;
  $('robot-label').textContent = state.referenceOnly ? 'UNITREE H2 · REFERENCE ONLY' : state.liveConfirmed ? 'UNITREE H2 · CLOSED LOOP' : 'UNITREE H2';
  $('motion-heading').textContent = state.referenceOnly ? 'REFERENCE MOTION · VIEW ONLY' : 'POLICY MOTION TARGET';
  $('reference-only').hidden = state.referenceOnly;
  $('reference-only').disabled = !hasReferencePreview();
  $('reference-only').title = hasReferencePreview() ? '' : 'This motion has no reference-only preview. Use browser policy and physics.';
  for (const option of ui.motion.options || []) {
    const motion = manifest?.motions.find(entry => entry.id === option.value);
    option.disabled = state.referenceOnly && !hasReferencePreview(motion);
  }
  $('motion-reference-note').textContent = state.motion?.referenceSpace === 'g1'
    ? 'G1 reference targets for the H2 controller. No reference-only preview.'
    : state.motion?.referenceSpace === 'smpl' ? 'SMPL reference targets for the H2 controller. No reference-only preview.'
    : state.referenceOnly ? 'H2 reference motion · view only.' : 'H2 reference targets for the H2 controller.';
  $('live-telemetry').hidden = !state.liveConfirmed;
  if (!state.live && !state.referenceOnly) $('play').textContent = 'Pause';
}
function showHint(title, detail) {
  $('hint-title').textContent = title; $('hint-detail').textContent = detail;
  $('planner-hint').hidden = false;
}
async function connect() {
  if (state.busy) return;
  if (localSimulation.error) localSimulation.restart();
  localSimulation.suspend(document.hidden);
  const attempt = ++state.connectionAttempt;
  state.busy = true; state.referenceOnly = false; state.playing = false;
  state.live = false; state.liveConfirmed = false; state.failed = false;
  $('connect').disabled = true; $('connect').textContent = 'Loading browser simulation…';
  setConnection('Loading local controller',false);
  $('runtime-note').textContent = 'Loading the H2 policy and physics into your browser. Each visitor has an independent simulation.';
  $('source-label').textContent = 'Loading browser policy and physics · static model';
  $('footer-state').textContent = 'Loading · independent browser simulation';
  showHint('Starting in your browser','Loading local models and physics. No SSH or simulation server is needed.');
  syncControls(); updateSceneLabel();
  // A new worker starts in tracking mode. Later mode changes go only through
  // the explicit mode switch, never through a movement/keepalive command.
  const payload = {motion:state.motion.id, action:'start'};
  const accepted = await command(payload);
  if (attempt !== state.connectionAttempt) return;
  if (accepted) {
    state.live = true; state.snapshotId = null;
    state.frame = null; state.actualMode = null;
    state.lastLive = performance.now(); state.playing = true; state.liveStatus = 'starting';
    $('play').textContent = 'Pause';
    $('runtime-note').textContent = 'Local controller ready. Waiting for the first physics state before enabling motion commands.';
    $('source-label').textContent = 'Initializing simulation · waiting for live pose';
    $('footer-state').textContent = 'Local controller initializing';
    setConnection('Controller initializing', false);
  } else {
    endLive('Browser simulation unavailable',state.error || 'The browser simulation could not start. Retry or explicitly choose reference-only viewing.');
  }
  state.busy = false; $('connect').disabled = false;
  $('connect').innerHTML = state.live ? 'Restart local rollout <span>↗</span>' : 'Retry browser simulation <span>↗</span>';
  syncControls(); updateSceneLabel();
}
function updateSceneLabel() {
  const initializing = state.live && (!state.liveConfirmed || ['starting','warming_up'].includes(state.liveStatus));
  $('scene-label').textContent = state.referenceOnly ? 'H2 · REFERENCE ONLY · NO POLICY INFERENCE' : initializing ? 'H2 · LOADING LOCAL CONTROLLER' : state.liveConfirmed ? `H2 · LOCAL ${(state.actualMode || state.mode) === 'tracking' ? 'MOTION TRACKING' : 'PLANNER CONTROL'}` : state.busy ? 'H2 · LOADING LOCAL CONTROLLER' : 'H2 · LOCAL SIMULATION UNAVAILABLE';
}
function renderMode(mode) {
  state.mode = mode;
  if (mode === 'planner' && state.referenceOnly) state.time = 0;
  document.querySelectorAll('.mode').forEach((button) => {
    button.classList.toggle('active',button.dataset.mode === mode);
    button.setAttribute('aria-pressed',String(button.dataset.mode === mode));
  });
  $('tracking-controls').hidden = mode !== 'tracking';
  $('planner-controls').hidden = mode !== 'planner';
  $('mode-title').textContent = mode === 'tracking' ? 'A motion. A whole body.' : 'Choose a direction.';
  $('mode-description').textContent = state.referenceOnly ? 'Reference-only viewing. No learned policy or physics feedback is running in this viewer.' : mode === 'tracking' ? 'The H2 policy tracks motion targets in a physics simulation running entirely in your browser.' : 'Choose a locomotion style and move independently of facing direction. Local motion targets guide the real H2 policy and physics.';
  if (state.referenceOnly && mode === 'planner') showHint('Planner requires local simulation','Return to browser simulation to use the planner.');
  else if (state.referenceOnly) $('planner-hint').hidden = true;
  syncControls(); updateSceneLabel();
}
async function setMode(mode) {
  if (!controlReady() || mode === state.mode || !['tracking','planner'].includes(mode)) return;
  stopMovement(false);
  state.pendingMode = mode;
  syncControls();
  const payload = {mode, action:'reset'};
  if (mode === 'tracking') payload.motion = state.motion.id;
  else payload.plannerStyle = state.plannerStyle;
  if (!await command(payload)) {state.pendingMode = null; syncControls();}
  // The next worker state acknowledges the switch and changes the panel.
}
function applyPlannerStyle(style) {
  if (!(style in plannerStyles)) return;
  const changed = style !== state.plannerStyle;
  state.plannerStyle = style;
  $('planner-style').value = style;
  const range = plannerStyles[style], velocity = $('velocity');
  velocity.min = String(range.min); velocity.max = String(range.max); velocity.step = '.1';
  velocity.value = String(changed ? range.speed : Math.max(range.min,Math.min(range.max,Number(velocity.value))));
  $('velocity-label').textContent = style === 'idle' ? 'Standing' : `${Number(velocity.value).toFixed(1)} m/s`;
  const turnSpeed = $('turn-speed');
  turnSpeed.max = String(range.turnMax);
  turnSpeed.value = String(Math.max(Number(turnSpeed.min),Math.min(range.turnMax,Number(turnSpeed.value))));
  $('turn-speed-label').textContent = `${Number(turnSpeed.value).toFixed(1)} rad/s`;
  $('direction-limit').textContent = style === 'idle' ? 'Standing; no translation command.' : `Forward up to ${range.max.toFixed(1)} m/s; strafe/backward up to ${range.directionalMax.toFixed(1)} m/s.`;
}
async function setPlannerStyle(style) {
  if (!controlReady() || state.mode !== 'planner' || !(style in plannerStyles)) return;
  stopMovement(false);
  state.pendingPlannerStyle = style;
  syncControls();
  if (!await command({plannerStyle:style,vx:0,vy:0,yaw:0})) {
    state.pendingPlannerStyle = null; applyPlannerStyle(state.plannerStyle); syncControls();
  }
}
function enterReferenceOnly() {
  if (!hasReferencePreview()) return;
  stopMovement();
  localSimulation.suspend(true);
  if (state.liveConfirmed) request('command',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({action:'pause'})}).catch(()=>{});
  ++state.connectionAttempt; state.busy = false; state.live = false; state.liveConfirmed = false;
  state.referenceOnly = true; state.frame = null; state.playing = true; state.time = 0;
  $('connect').disabled = false; $('connect').innerHTML = 'Return to browser simulation <span>↗</span>';
  $('play').textContent = 'Pause';
  setConnection('Reference only · no inference',false);
  $('runtime-note').textContent = 'You selected reference-only viewing. This is kinematic playback, with no policy inference or physics feedback.';
  $('source-label').textContent = 'REFERENCE ONLY · kinematic playback · no controller';
  $('footer-state').textContent = 'Reference-only viewing explicitly selected';
  state.pendingMode = null; state.pendingPlannerStyle = null;
  renderMode('tracking'); syncControls(); updateSceneLabel();
}
function stopMovement(send = true) {
  clearTimeout(sendTimer); commandGeneration++;
  heldKeys.clear(); heldPointers.clear();
  state.vx = state.vy = state.yaw = 0;
  document.querySelectorAll('[data-direction]').forEach((button) => button.classList.remove('pressed'));
  updateCommand(send);
}
function updateCommand(send = true) {
  clearTimeout(sendTimer);
  $('vx').textContent = state.vx.toFixed(1); $('vy').textContent = state.vy.toFixed(1); $('yaw').textContent = state.yaw.toFixed(1);
  if (send && controlReady() && state.mode === 'planner') {
    const generation = commandGeneration;
    sendTimer = setTimeout(() => {
      if (generation !== commandGeneration || !controlReady() || state.mode !== 'planner') return;
      command({vx:state.vx,vy:state.vy,yaw:state.yaw});
    },20);
  }
}
function updateHeldInput() {
  if (!controlReady() || state.mode !== 'planner' || state.plannerStyle === 'idle') return;
  const directions = new Set([...heldKeys].map(code => keyDirections[code]).concat([...heldPointers.values()]));
  const range = plannerStyles[state.plannerStyle];
  const requestedSpeed = Math.max(range.min,Math.min(range.max,Number($('velocity').value)));
  const x = Number(directions.has('forward')) - Number(directions.has('back'));
  const y = Number(directions.has('left')) - Number(directions.has('right'));
  const length = Math.hypot(x,y) || 1;
  const speed = x < 0 || y !== 0 ? Math.min(requestedSpeed,range.directionalMax) : requestedSpeed;
  state.vx = x / length * speed; state.vy = y / length * speed;
  const turnSpeed = $('turn-speed');
  const turnRate = Math.max(Number(turnSpeed.min),Math.min(Number(turnSpeed.max),Number(turnSpeed.value)));
  state.yaw = turnRate * (Number(directions.has('turn-left')) - Number(directions.has('turn-right')));
  document.querySelectorAll('[data-direction]').forEach(button => button.classList.toggle('pressed',directions.has(button.dataset.direction)));
  updateCommand();
}
function bindControls() {
  document.querySelectorAll('.mode').forEach((button) => button.addEventListener('click',() => setMode(button.dataset.mode)));
  $('planner-style').addEventListener('change',() => setPlannerStyle($('planner-style').value));
  ui.motion.addEventListener('change',() => loadMotion(ui.motion.value).catch(showError));
  $('play').addEventListener('click',async () => {
    const next = state.motionComplete || !state.playing;
    if (state.live && !await command({action:next ? 'play' : 'pause'})) return;
    state.playing = next; $('play').textContent = next ? 'Pause' : 'Resume';
    if (state.live && !next) $('runtime-note').textContent = 'Local physics is paused. Resume to continue this rollout.';
  });
  $('reset').addEventListener('click',() => {state.time = 0; if(state.live) command({action:'reset'}); resetCamera();});
  $('planner-reset').addEventListener('click',() => {stopMovement(false); state.time = 0; if(state.live) command({action:'reset'}); resetCamera();});
  $('speed').addEventListener('change',() => {state.speed = Number($('speed').value);});
  ui.timeline.addEventListener('input',() => {state.time = Number(ui.timeline.value)*state.motion.duration;});
  $('camera-reset').addEventListener('click',resetCamera);
  $('follow').addEventListener('click',() => {state.follow = !state.follow; $('follow').setAttribute('aria-pressed',String(state.follow));});
  $('connect').addEventListener('click',() => {
    if (state.liveConfirmed) {stopMovement(false); command({action:'reset'});}
    else connect();
  });
  $('reference-only').addEventListener('click',enterReferenceOnly);
  $('velocity').addEventListener('input',() => {$('velocity-label').textContent = `${Number($('velocity').value).toFixed(1)} m/s`; updateHeldInput();});
  $('turn-speed').addEventListener('input',() => {
    if ($('turn-speed').disabled) return;
    $('turn-speed-label').textContent = `${Number($('turn-speed').value).toFixed(1)} rad/s`;
    updateHeldInput();
  });
  document.querySelectorAll('[data-direction]').forEach((button) => {
    button.addEventListener('pointerdown',(event) => {
      if (!controlReady() || state.mode !== 'planner' || state.plannerStyle === 'idle') return;
      event.preventDefault();
      if (button.dataset.direction === 'stop') {stopMovement(); return;}
      button.setPointerCapture(event.pointerId); heldPointers.set(event.pointerId,button.dataset.direction); updateHeldInput();
    });
    const release = event => {if (heldPointers.delete(event.pointerId)) updateHeldInput();};
    button.addEventListener('pointerup',release); button.addEventListener('pointercancel',release); button.addEventListener('lostpointercapture',release);
  });
  window.addEventListener('keydown',(event) => {
    const target = event.target;
    const editing = target.isContentEditable || /SELECT|TEXTAREA/.test(target.tagName) || (target.tagName === 'INPUT' && (target.type !== 'range' || event.code.startsWith('Arrow')));
    if(!controlReady() || state.mode !== 'planner' || state.plannerStyle === 'idle' || !keyDirections[event.code] || editing) return;
    event.preventDefault();
    if (!heldKeys.has(event.code)) {heldKeys.add(event.code); updateHeldInput();}
  });
  window.addEventListener('keyup',(event) => {if(heldKeys.delete(event.code)) {event.preventDefault(); updateHeldInput();}});
  window.addEventListener('blur',() => stopMovement());
  document.addEventListener('visibilitychange',() => {if(document.hidden) stopMovement(); localSimulation.suspend(document.hidden || state.referenceOnly);});
  window.addEventListener('pagehide',()=>localSimulation.dispose());
  // The simulator has a command watchdog. Renew a held command while this
  // visible, focused demo owns the input; releasing or leaving sends zero.
  setInterval(() => {
    if (controlReady() && state.mode === 'planner' && !document.hidden && (state.vx || state.vy || state.yaw))
      command({vx:state.vx,vy:state.vy,yaw:state.yaw});
  },250);
}
function showError(error) {
  ui.loading.replaceChildren(document.createTextNode(error.message || 'The 3D viewer could not be loaded.'));
  ui.loading.classList.remove('hidden');
}
function endLive(label, message) {
  state.live = false; state.liveConfirmed = false; state.failed = true;
  state.referenceOnly = false; state.playing = false; state.vx = state.vy = state.yaw = 0;
  state.pendingMode = null; state.pendingPlannerStyle = null;
  clearTimeout(sendTimer);
  stopMovement();
  setConnection(label,false); $('runtime-note').textContent = message;
  $('play').textContent = 'Pause';
  $('source-label').textContent = 'LOCAL SIMULATION UNAVAILABLE · model held still';
  $('footer-state').textContent = 'Disconnected · no prerecorded motion is playing';
  $('connect').innerHTML = 'Retry browser simulation <span>↗</span>';
  showHint('Browser simulation unavailable',message);
  syncControls(); updateSceneLabel();
}
async function poll() {
  if (state.live) {
    try {
      const frame = await request('state');
      if (state.live && (['error','stopped','offline'].includes(frame.status) || frame.error)) {
        endLive('Simulation unavailable', frame.error || frame.message || 'The simulation stopped. Reconnect to start another rollout.');
      }
      const pose = frame.body_poses || frame.bodyPoses;
      const wasConfirmed = state.liveConfirmed;
      const snapshotId = `${frame.timestamp ?? ''}:${frame.sim_time ?? ''}`;
      const hasClock = Number.isFinite(frame.timestamp) || Number.isFinite(frame.sim_time);
      if (state.live && pose && hasClock && snapshotId !== state.snapshotId) {
        const flat = Array.isArray(pose[0]) ? pose.flat() : pose;
        if (flat.length === bodies.length * 7 && flat.every(Number.isFinite)) {
          const now=performance.now();
          state.previousFrame = state.frame || flat; state.frame = flat;
          state.frameInterval=Math.max(16,Math.min(100,now-state.lastLive)); state.lastLive=now; state.snapshotId = snapshotId;
          state.liveConfirmed = true; state.actualMode = frame.mode;
          if (['tracking','planner'].includes(frame.mode) && (!state.pendingMode || frame.mode === state.pendingMode)) {
            state.pendingMode = null;
            if (state.mode !== frame.mode) renderMode(frame.mode);
          }
          if (frame.planner_style in plannerStyles && (!state.pendingPlannerStyle || frame.planner_style === state.pendingPlannerStyle)) {
            state.pendingPlannerStyle = null; applyPlannerStyle(frame.planner_style);
          }
          $('source-label').textContent = frame.mode === 'tracking' && frame.reference_space === 'g1'
            ? 'In your browser · G1 reference → H2 policy + physics'
            : 'In your browser · MuJoCo + H2 policy';
          $('footer-state').textContent = 'LOCAL · independent simulation · no SSH';
        }
      }
      if (state.liveConfirmed && Number.isFinite(frame.sim_time)) $('sim-clock').textContent = `${frame.sim_time.toFixed(1)} s`;
      if (state.liveConfirmed && Array.isArray(frame.root_velocity)) $('measured-speed').textContent = `${Math.hypot(frame.root_velocity[0],frame.root_velocity[1]).toFixed(2)} m/s`;
      if (state.liveConfirmed && Number.isFinite(frame.real_time_factor)) $('runtime-rate').textContent = `${frame.real_time_factor.toFixed(2)}× real time`;
      if (state.liveConfirmed) {
        state.motionComplete=!!frame.motion_complete; state.playing=frame.status==='running';
        $('play').textContent=state.motionComplete?'Restart motion':state.playing?'Pause':'Resume';
      }
      const preparing = frame.mode === 'tracking' && !!frame.motion_preparing;
      const preparingChanged = preparing !== state.motionPreparing;
      state.motionPreparing = preparing;
      if (state.live && (frame.status !== state.liveStatus || !state.liveConfirmed || !wasConfirmed || preparingChanged || preparing)) {
        state.liveStatus = frame.status;
        if (frame.status === 'starting' || frame.status === 'warming_up' || !state.liveConfirmed) {
          setConnection('Simulation starting',false);
          $('runtime-note').textContent = 'The browser is initializing the H2 policy and physics.';
        } else if (frame.status === 'running' && preparing) {
          setConnection('Preparing motion',true); $('planner-hint').hidden = true;
          const remaining = Number.isFinite(frame.motion_prepare_remaining) ? ` (${Math.max(0,frame.motion_prepare_remaining).toFixed(1)} s remaining)` : '';
          $('runtime-note').textContent = `The policy is holding the first target in physics before starting the full motion${remaining}.`;
        } else if (frame.status === 'running') {
          setConnection('Browser policy + physics',true); $('planner-hint').hidden = true;
          $('runtime-note').textContent = 'Your controls, policy inference and physics run on this device. Other visitors have their own simulation.';
        } else if (frame.status === 'paused') {
          setConnection('Local simulation · paused',true); $('planner-hint').hidden = true;
          $('runtime-note').textContent = preparing ? 'Motion preparation and physics are paused. Resume to continue the remaining startup hold.' : 'Physics is paused on this device. Resume to continue.';
        } else if (frame.status === 'fallen') {
          setConnection('Reset needed',false);
          $('runtime-note').textContent = 'The robot lost balance in simulation. Use Reset to start a new rollout.';
        }
      }
      if (state.live) { syncControls(); updateSceneLabel(); }
    } catch { /* The connection indicator below exposes stalled live output. */ }
    if (state.live && performance.now() - state.lastLive > 5000) {
      endLive('Connection lost','The simulation stopped producing fresh states. Reconnect to resume live control.');
    }
  } else if (!state.referenceOnly && !state.busy && performance.now() - state.lastProbe > 10000) {
    state.lastProbe = performance.now();
    try {
      const health = await request('health');
      state.reachable = !health.error && !['error','offline','stopped'].includes(health.status);
      if (!state.live && !state.busy && !state.referenceOnly && state.reachable && ['running','paused'].includes(health.status)) await connect();
    } catch { state.reachable = false; }
  }
  setTimeout(poll,state.live ? 16 : 1000);
}
async function init() {
  renderer = new THREE.WebGLRenderer({antialias:true,alpha:false});
  renderer.setPixelRatio(Math.min(devicePixelRatio,2)); renderer.shadowMap.enabled = true;
  renderer.shadowMap.type = THREE.PCFSoftShadowMap;
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping; renderer.toneMappingExposure = 1.28;
  ui.stage.appendChild(renderer.domElement);
  scene = new THREE.Scene(); scene.background = new THREE.Color('#f0f3f5'); scene.fog = new THREE.Fog('#f0f3f5',10,27);
  camera = new THREE.PerspectiveCamera(34,1,.02,60); camera.up.set(0,0,1);
  orbit = new OrbitControls(camera,renderer.domElement); orbit.enableDamping = true; orbit.dampingFactor = .08;
  orbit.minDistance = 2; orbit.maxDistance = 12; orbit.maxPolarAngle = Math.PI*.49;
  const hemi = new THREE.HemisphereLight(0xffffff,0x8492a0,2.8); hemi.position.set(0,0,6); scene.add(hemi);
  keyLight = new THREE.DirectionalLight(0xffffff,3.3); keyLight.castShadow = true;
  keyLight.shadow.mapSize.set(2048,2048); keyLight.shadow.camera.left=-5; keyLight.shadow.camera.right=5; keyLight.shadow.camera.top=5; keyLight.shadow.camera.bottom=-5;
  keyLight.shadow.camera.near=.1; keyLight.shadow.camera.far=20;
  keyLight.shadow.normalBias=.015; keyLight.shadow.bias=-.0001; scene.add(keyLight,keyLight.target);
  const fill = new THREE.DirectionalLight(0xdce9f5,1.5); fill.position.set(-4,2,4); scene.add(fill);
  createGround();
  const results = await Promise.all([fetch('assets/h2-model.json').then(r=>r.json()),fetch('assets/h2-meshes.bin').then(r=>r.arrayBuffer()),fetch('assets/motions.json').then(r=>r.json())]);
  model=results[0]; const meshBuffer=results[1]; manifest=results[2];
  bodies = model.bodies.map(() => {const body = new THREE.Group();scene.add(body);return body;});
  const geometries = model.meshes.map((entry) => {
    const geometry=new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(new Float32Array(meshBuffer,entry.vertexOffset,entry.vertexCount*3),3));
    geometry.setIndex(new THREE.BufferAttribute(new Uint32Array(meshBuffer,entry.indexOffset,entry.indexCount),1));
    geometry.computeVertexNormals(); return geometry;
  });
  const silver = new THREE.MeshStandardMaterial({color:0xb6c0c7,metalness:.48,roughness:.4});
  const dark = new THREE.MeshStandardMaterial({color:0x303943,metalness:.25,roughness:.55});
  for(const geom of model.geoms) {
    const mesh=new THREE.Mesh(geometries[geom.mesh],geom.dark?dark:silver);
    mesh.position.fromArray(geom.position); mesh.quaternion.set(geom.quaternion[1],geom.quaternion[2],geom.quaternion[3],geom.quaternion[0]);
    mesh.castShadow=true; mesh.receiveShadow=true; bodies[geom.body].add(mesh);
  }
  ui.motion.replaceChildren(...manifest.motions.map(entry=>{const option=document.createElement('option');option.value=entry.id;option.textContent=entry.label;return option;}));
  await loadMotion(manifest.motions[0].id);
  bindControls(); resize(); new ResizeObserver(resize).observe(ui.stage);
  syncControls(); ui.loading.classList.add('hidden');
  let previous=performance.now();
  function animate(now) {
    const dt=Math.min((now-previous)/1000,.1); previous=now;
    if (!document.hidden) {
      if(state.live && state.frame) renderPose(state.previousFrame || state.frame,state.frame,Math.min(1,(now-state.lastLive)/(state.frameInterval || 20)));
      else if(state.referenceOnly && state.frames && state.motion) {
        if(state.playing && state.mode === 'tracking') state.time=(state.time+dt*state.speed)%state.motion.duration;
        const f=state.time*state.motion.fps, index=Math.min(Math.floor(f),state.motion.frames-1), next=Math.min(index+1,state.motion.frames-1), stride=bodies.length*7;
        renderPose(state.frames.subarray(index*stride,(index+1)*stride),state.frames.subarray(next*stride,(next+1)*stride),f-Math.floor(f));
        ui.timeline.value=state.time/state.motion.duration; $('time-label').textContent=`${state.time.toFixed(1)} s`;
      }
      orbit.update(); updateGround(); renderer.render(scene,camera);
    }
    requestAnimationFrame(animate);
  }
  requestAnimationFrame(animate); poll(); connect();
}
init().catch(showError);
