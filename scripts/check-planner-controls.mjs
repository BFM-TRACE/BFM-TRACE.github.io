import assert from 'node:assert/strict';
import vm from 'node:vm';
import { createHash } from 'node:crypto';
import { readFile, writeFile, mkdir } from 'node:fs/promises';
import { fileURLToPath } from 'node:url';
import { prepareTrackingReference } from '../demo/local-reference.js';
import { LocalPolicy } from '../demo/local-policy.js';

// Execute the production UI functions and event handlers in a small DOM host.
// No browser automation, rendering, inference, or simulated robot is involved.
const demoURL = new URL('../demo/demo.js', import.meta.url);
const htmlURL = new URL('../demo/index.html', import.meta.url);
const outputURL = new URL('../qa/generated/planner-controls.json', import.meta.url);
const [source, html] = await Promise.all([readFile(demoURL, 'utf8'), readFile(htmlURL, 'utf8')]);
const ids = new Set([...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]));
const report = {
  testedAt: new Date().toISOString(),
  host: `Node ${process.version} DOM event harness`,
  scope: 'Production planner UI input/state synchronization and tracking reference routing with stub inference/physics; no browser or physical-motion validation.',
  sourceSha256: createHash('sha256').update(source).digest('hex'),
  htmlSha256: createHash('sha256').update(html).digest('hex'),
  checks: [],
};
report.referenceSourceSha256 = createHash('sha256').update(await readFile(new URL('../demo/local-reference.js',import.meta.url))).digest('hex');
report.policySourceSha256 = createHash('sha256').update(await readFile(new URL('../demo/local-policy.js',import.meta.url))).digest('hex');
const nativeMotion = {id:'squat_001__A359',file:'squat_001__A359.bin',duration:8.5,sourceFPS:50};
const g1Motion = {id:'g1_explore_test',file:null,previewAvailable:false,referenceSpace:'g1',duration:1.2,sourceFPS:50};

function harness(mode = 'planner') {
  const timers = new Map(), intervals = [], elements = new Map(), windowEvents = new Map(), documentEvents = new Map(), fetched = [], suspensions = [];
  let serial = 0, clock = 1000, snapshot = 0;
  function makeElement(id = '') {
    const events = new Map(), attributes = new Map(), classes = new Set();
    const markup = id ? html.match(new RegExp(`<[^>]*\\bid="${id}"[^>]*>`))?.[0] || '' : '';
    const attribute = name => markup.match(new RegExp(`\\b${name}="([^"]*)"`))?.[1];
    return {
      id, events, attributes, value: attribute('value') ?? (id === 'planner-style' ? 'slow_walk' : ''),
      min: attribute('min') ?? '', max: attribute('max') ?? '', step: attribute('step') ?? '',
      textContent: '', innerHTML: '', hidden: false, disabled: false, dataset: {}, tagName: 'BUTTON', options: [],
      lastElementChild: { textContent: '' },
      classList: {
        add(...values) { values.forEach(value => classes.add(value)); },
        remove(...values) { values.forEach(value => classes.delete(value)); },
        toggle(value, force) { const selected = force ?? !classes.has(value); selected ? classes.add(value) : classes.delete(value); return selected; },
        contains(value) { return classes.has(value); },
      },
      setAttribute(name, value) { attributes.set(name, value); },
      addEventListener(name, callback) { if (!events.has(name)) events.set(name, []); events.get(name).push(callback); },
      setPointerCapture() {},
    };
  }
  const element = id => {
    assert.ok(ids.has(id), `Production HTML must contain #${id}`);
    if (!elements.has(id)) elements.set(id, makeElement(id));
    return elements.get(id);
  };
  const modes = ['tracking', 'planner'].map(value => Object.assign(makeElement(), { dataset: { mode: value } }));
  const directions = ['forward', 'back', 'left', 'right', 'turn-left', 'turn-right', 'stop'].map(value => Object.assign(makeElement(), { dataset: { direction: value } }));
  const addEvent = (map, name, callback) => { if (!map.has(name)) map.set(name, []); map.get(name).push(callback); };
  const context = vm.createContext({
    console, Math, Float32Array, Set, Map, Number, Array,
    performance: { now: () => clock },
    document: {
      hidden: false, getElementById: element,
      querySelectorAll: selector => selector === '.mode' ? modes : selector === '[data-direction]' ? directions : [],
      addEventListener: (name, callback) => addEvent(documentEvents, name, callback),
    },
    window: { addEventListener: (name, callback) => addEvent(windowEvents, name, callback) },
    THREE: { Vector3: class {}, Quaternion: class {} },
    LocalSimulation: class { suspend(hidden) {suspensions.push(hidden);} dispose() {} },
    fetch: async url => {fetched.push(url); return {ok:true,arrayBuffer:async()=>new ArrayBuffer(315*4)};},
    setTimeout: (callback, delay) => { const id = ++serial; timers.set(id, { callback, delay }); return id; },
    clearTimeout: id => timers.delete(id),
    setInterval: callback => intervals.push(callback),
  });
  const executable = source.replace(/^import .*;\n/gm, '').replace(/init\(\)\.catch\(showError\);\s*$/, '');
  assert.notEqual(executable, source, 'The harness must remove module imports and automatic page initialization');
  vm.runInContext(executable + `
    globalThis.sent = [];
    request = async (path, options = {}) => {
      if (path === 'command') { sent.push(JSON.parse(options.body)); return {accepted: true}; }
      if (path === 'state') return globalThis.workerFrame;
      throw new Error('Unexpected request: ' + path);
    };
    bodies = Array.from({length: 45}, () => ({}));
    globalThis.api = {state,heldKeys,heldPointers,bindControls,syncControls,poll,setMode,
      setPlannerStyle,applyPlannerStyle,renderMode,stopMovement,updateHeldInput,
      loadMotion,enterReferenceOnly,connect,hasReferencePreview,
      setManifest: value => {manifest=value;}};
  `, context, { filename: fileURLToPath(demoURL) });
  const api = context.api;
  Object.assign(api.state, {
    live: true, liveConfirmed: true, liveStatus: 'running', mode, actualMode: mode,
    motion: nativeMotion, lastLive: clock,
  });
  api.setManifest({motions:[nativeMotion,g1Motion]});
  element('motion').options = [nativeMotion,g1Motion].map(motion=>({value:motion.id,disabled:false}));
  api.bindControls(); api.renderMode(mode); api.syncControls();
  const dispatch = async (map, name, event = {}) => {
    const input = { preventDefault() {}, target: { tagName: 'BODY' }, ...event };
    for (const callback of map.get(name) || []) await callback(input);
  };
  return {
    api, context, timers, intervals, element, modes, directions, fetched, suspensions,
    sent: () => JSON.parse(JSON.stringify(context.sent)),
    key: (name, code, target) => dispatch(windowEvents, name, { code, ...(target ? { target } : {}) }),
    blur: () => dispatch(windowEvents, 'blur'),
    event: (id, name, event = {}) => dispatch(element(id).events, name, event),
    pointer: (direction, name, pointerId) => dispatch(directions.find(button => button.dataset.direction === direction).events, name, { pointerId }),
    flushMovement: async () => {
      for (const [id, entry] of [...timers]) if (entry.delay === 20) { timers.delete(id); await entry.callback(); }
      await Promise.resolve();
    },
    frame: async (values = {}) => {
      clock += 20; snapshot++;
      context.workerFrame = {
        status: 'running', mode: api.state.actualMode, planner_style: api.state.plannerStyle,
        sim_time: snapshot * 0.02, timestamp: snapshot, root_velocity: [0, 0, 0],
        body_poses: Array.from({ length: 315 }, (_, index) => index % 7 === 3 ? 1 : 0),
        ...values,
      };
      await api.poll();
      // poll scheduling is not part of these tests; each fresh state is explicit.
      for (const [id, entry] of timers) if (entry.delay !== 20) timers.delete(id);
    },
  };
}

async function check(name, callback) {
  try {
    const details = await callback();
    report.checks.push({ name, status: 'pass', ...details });
  } catch (error) {
    report.checks.push({ name, status: 'fail', error: error.message });
  }
}

await check('W+A combines translation and normalizes diagonal target speed', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyA');
  assert.ok(Math.abs(h.api.state.vx - 0.4 / Math.sqrt(2)) < 1e-12);
  assert.ok(Math.abs(h.api.state.vy - 0.4 / Math.sqrt(2)) < 1e-12);
  assert.ok(Math.abs(Math.hypot(h.api.state.vx, h.api.state.vy) - 0.4) < 1e-12);
});
await check('Q/E yaw is independent of translation; opposite turns cancel', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyQ');
  assert.equal(h.api.state.vx, 0.4); assert.equal(h.api.state.yaw, 0.5);
  await h.key('keydown', 'KeyE'); assert.equal(h.api.state.yaw, 0); assert.equal(h.api.state.vx, 0.4);
});
await check('Turn-rate slider declares rad/s range and preserves the default 0.5 command', async () => {
  const h = harness(), slider = h.element('turn-speed');
  assert.equal(Number(slider.min), 0.1); assert.equal(Number(slider.max), 1);
  assert.equal(Number(slider.step), 0.1); assert.equal(Number(slider.value), 0.5);
  assert.equal(slider.disabled, false);
  assert.match(html, /for="turn-speed">TURN RATE COMMAND/);
  assert.match(html, /id="turn-speed-label">0\.5 rad\/s/);
  await h.key('keydown', 'KeyQ'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0, vy: 0, yaw: 0.5 });
});
await check('Dragging turn rate updates held Q immediately and releasing Q stops yaw only', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyQ');
  for (const rate of [0.1, 0.8, 1]) {
    h.element('turn-speed').value = String(rate); await h.event('turn-speed', 'input'); await h.flushMovement();
    assert.deepEqual(h.sent().at(-1), { vx: 0.4, vy: 0, yaw: rate });
    assert.equal(h.element('turn-speed-label').textContent, `${rate.toFixed(1)} rad/s`);
    assert.equal(h.element('yaw').textContent, rate.toFixed(1));
  }
  await h.key('keyup', 'KeyQ'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0.4, vy: 0, yaw: 0 });
  assert.equal(h.element('turn-speed').value, '1');
});
await check('Turn and translation sliders independently update combined E and diagonal input', async () => {
  const h = harness();
  for (const key of ['KeyW', 'KeyA', 'KeyE']) await h.key('keydown', key, { tagName: 'INPUT', type: 'range' });
  h.element('turn-speed').value = '0.7'; await h.event('turn-speed', 'input'); await h.flushMovement();
  const first = h.sent().at(-1);
  assert.ok(Math.abs(Math.hypot(first.vx, first.vy) - 0.4) < 1e-12); assert.equal(first.yaw, -0.7);
  h.element('velocity').value = '0.6'; await h.event('velocity', 'input'); await h.flushMovement();
  const second = h.sent().at(-1);
  assert.ok(Math.abs(Math.hypot(second.vx, second.vy) - 0.6) < 1e-12); assert.equal(second.yaw, -0.7);
  await h.key('keyup', 'KeyW'); await h.key('keyup', 'KeyA'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0, vy: 0, yaw: -0.7 });
});
await check('Opposite held turns stay cancelled while dragging, then resume the remaining sign', async () => {
  const h = harness(); await h.key('keydown', 'KeyQ'); await h.key('keydown', 'KeyE');
  h.element('turn-speed').value = '0.9'; await h.event('turn-speed', 'input'); await h.flushMovement();
  assert.equal(h.sent().at(-1).yaw, 0);
  await h.key('keyup', 'KeyQ'); await h.flushMovement(); assert.equal(h.sent().at(-1).yaw, -0.9);
  await h.key('keyup', 'KeyE'); await h.flushMovement(); assert.equal(h.sent().at(-1).yaw, 0);
});
await check('Pointer turn buttons use the slider amplitude and release or lost capture sends zero', async () => {
  const h = harness(); await h.pointer('turn-right', 'pointerdown', 1);
  h.element('turn-speed').value = '0.8'; await h.event('turn-speed', 'input'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0, vy: 0, yaw: -0.8 });
  await h.pointer('turn-right', 'lostpointercapture', 1); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0, vy: 0, yaw: 0 });
  await h.pointer('turn-left', 'pointerdown', 2); await h.flushMovement(); assert.equal(h.sent().at(-1).yaw, 0.8);
  await h.pointer('turn-left', 'pointerup', 2); await h.flushMovement(); assert.equal(h.sent().at(-1).yaw, 0);
});
await check('Moving the turn-rate slider without a held turn never starts rotation', async () => {
  const h = harness(); h.element('turn-speed').value = '1'; await h.event('turn-speed', 'input'); await h.flushMovement();
  assert.equal(h.api.state.yaw, 0); assert.ok(h.sent().every(command => command.yaw === 0));
});
await check('Turn-rate input is disabled outside ready planner control and ignores disabled input events', async () => {
  const tested = [];
  for (const [name, patch] of [
    ['initializing', { liveConfirmed: false }], ['disconnected', { live: false }],
    ['reference-only', { referenceOnly: true }], ['idle', { plannerStyle: 'idle' }],
    ['tracking', { mode: 'tracking' }], ['pending-mode', { pendingMode: 'tracking' }],
    ['pending-style', { pendingPlannerStyle: 'walk' }],
  ]) {
    const h = harness(); Object.assign(h.api.state, patch); h.api.syncControls();
    assert.equal(h.element('turn-speed').disabled, true, name);
    h.element('turn-speed').value = '0.8'; await h.event('turn-speed', 'input'); await h.flushMovement();
    assert.equal(h.api.state.yaw, 0, name); assert.equal(h.sent().length, 0, name); tested.push(name);
  }
  const h = harness(); await h.api.setPlannerStyle('idle'); await h.frame({ planner_style: 'idle' });
  assert.equal(h.element('turn-speed').disabled, true);
  await h.api.setPlannerStyle('walk'); await h.frame({ planner_style: 'walk' });
  assert.equal(h.element('turn-speed').disabled, false);
  return { tested };
});
await check('Turn-rate limits wait for style acknowledgement and clamp only when entering a narrower range', async () => {
  const h = harness(); h.element('turn-speed').value = '0.9'; await h.event('turn-speed', 'input');
  await h.api.setPlannerStyle('run');
  assert.equal(Number(h.element('turn-speed').max), 1); assert.equal(h.element('turn-speed').value, '0.9');
  assert.equal(h.element('turn-speed').disabled, true);
  await h.frame({ planner_style: 'slow_walk' });
  assert.equal(Number(h.element('turn-speed').max), 1); assert.equal(h.element('turn-speed').value, '0.9');
  await h.frame({ planner_style: 'run' });
  assert.equal(Number(h.element('turn-speed').max), 0.3); assert.equal(h.element('turn-speed').value, '0.3');
  assert.equal(h.element('turn-speed-label').textContent, '0.3 rad/s'); assert.equal(h.element('turn-speed').disabled, false);
  await h.api.setPlannerStyle('walk'); await h.frame({ planner_style: 'walk' });
  assert.equal(Number(h.element('turn-speed').max), 1); assert.equal(h.element('turn-speed').value, '0.3');
});
await check('A legal user-selected turn rate survives Run, Idle and repeated worker states', async () => {
  const h = harness(); h.element('turn-speed').value = '0.2'; await h.event('turn-speed', 'input');
  for (const style of ['run', 'idle', 'walk', 'slow_walk']) {
    await h.api.setPlannerStyle(style); await h.frame({ planner_style: style }); await h.frame({ planner_style: style });
    assert.equal(h.element('turn-speed').value, '0.2', style);
    assert.equal(h.element('turn-speed').disabled, style === 'idle', style);
  }
});
await check('Run turn-rate clamping and dragging preserve the independent forward speed command', async () => {
  const h = harness(); await h.api.setPlannerStyle('run'); await h.frame({ planner_style: 'run' });
  assert.equal(h.element('turn-speed').value, '0.3');
  h.element('velocity').value = '4'; await h.event('velocity', 'input');
  await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyQ'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 4, vy: 0, yaw: 0.3 });
  h.element('turn-speed').value = '0.1'; await h.event('turn-speed', 'input'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 4, vy: 0, yaw: 0.1 });
  h.element('velocity').value = '2.5'; await h.event('velocity', 'input'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 2.5, vy: 0, yaw: 0.1 });
});
await check('Single-key release preserves other held directions and yaw', async () => {
  const h = harness();
  for (const code of ['KeyW', 'KeyA', 'KeyQ']) await h.key('keydown', code);
  await h.key('keyup', 'KeyA');
  assert.equal(h.api.state.vx, 0.4); assert.equal(h.api.state.vy, 0); assert.equal(h.api.state.yaw, 0.5);
  await h.key('keyup', 'KeyQ'); assert.equal(h.api.state.vx, 0.4); assert.equal(h.api.state.yaw, 0);
});
await check('Arrow keys use the same composable translation input', async () => {
  const h = harness(); await h.key('keydown', 'ArrowUp'); await h.key('keydown', 'ArrowRight');
  assert.ok(h.api.state.vx > 0 && h.api.state.vy < 0);
  await h.key('keyup', 'ArrowRight'); assert.equal(h.api.state.vx, 0.4); assert.equal(h.api.state.vy, 0);
});
await check('Held movement reacts immediately to speed changes', async () => {
  const h = harness(); await h.key('keydown', 'KeyW');
  h.element('velocity').value = '0.6'; await h.event('velocity', 'input');
  assert.equal(h.api.state.vx, 0.6); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 0.6, vy: 0, yaw: 0 });
});
await check('WASD works after range input focus; arrows retain native range editing', async () => {
  const h = harness(), range = { tagName: 'INPUT', type: 'range' };
  await h.key('keydown', 'KeyW', range); assert.equal(h.api.state.vx, 0.4);
  await h.key('keydown', 'ArrowLeft', range); assert.equal(h.api.state.vy, 0);
  await h.key('keydown', 'KeyA', { tagName: 'SELECT' }); assert.equal(h.api.state.vy, 0);
});
await check('Pointer translation and turn combine; releasing one preserves the other', async () => {
  const h = harness(); await h.pointer('right', 'pointerdown', 1); await h.pointer('turn-left', 'pointerdown', 2);
  assert.equal(h.api.state.vy, -0.4); assert.equal(h.api.state.yaw, 0.5);
  await h.pointer('right', 'pointerup', 1); assert.equal(h.api.state.vy, 0); assert.equal(h.api.state.yaw, 0.5);
  await h.pointer('turn-left', 'lostpointercapture', 2); assert.equal(h.api.state.yaw, 0);
});
await check('Blur clears held keyboard and pointer state and sends a stop', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.pointer('turn-right', 'pointerdown', 1);
  await h.blur(); assert.equal(h.api.heldKeys.size, 0); assert.equal(h.api.heldPointers.size, 0);
  await h.flushMovement(); assert.deepEqual(h.sent().at(-1), { vx: 0, vy: 0, yaw: 0 });
});
await check('Explicit mode reset cancels timers; even a stale callback cannot revert the mode', async () => {
  const h = harness(); await h.key('keydown', 'KeyA');
  const stale = [...h.timers.values()].find(entry => entry.delay === 20).callback;
  await h.api.setMode('tracking');
  assert.equal(h.timers.size, 0); assert.equal(h.api.heldKeys.size, 0);
  assert.deepEqual(h.sent().at(-1), { mode: 'tracking', action: 'reset', motion: 'squat_001__A359' });
  const count = h.sent().length; stale(); await Promise.resolve(); assert.equal(h.sent().length, count);
});
await check('Mode panels wait for a matching fresh worker acknowledgement', async () => {
  const h = harness(); await h.api.setMode('tracking');
  assert.equal(h.api.state.mode, 'planner'); assert.equal(h.api.state.pendingMode, 'tracking');
  assert.ok(h.modes.every(button => button.disabled));
  await h.frame({ mode: 'planner' }); assert.equal(h.api.state.pendingMode, 'tracking'); assert.equal(h.api.state.mode, 'planner');
  await h.frame({ mode: 'tracking' }); assert.equal(h.api.state.pendingMode, null); assert.equal(h.api.state.mode, 'tracking');
  assert.equal(h.element('tracking-controls').hidden, false); assert.equal(h.element('planner-controls').hidden, true);
  assert.equal(h.modes.find(button => button.dataset.mode === 'tracking').attributes.get('aria-pressed'), 'true');
});
await check('Initialization and fallen state disable mode and style changes', async () => {
  const h = harness(); h.api.state.liveConfirmed = false; h.api.syncControls();
  assert.ok(h.modes.every(button => button.disabled)); assert.equal(h.element('planner-style').disabled, true);
  await h.api.setMode('tracking'); assert.equal(h.sent().length, 0);
  h.api.state.liveConfirmed = true; h.api.state.liveStatus = 'fallen'; h.api.syncControls();
  assert.ok(h.modes.every(button => button.disabled)); assert.equal(h.element('planner-reset').disabled, false);
});
await check('Planner style waits for matching state and applies its speed range', async () => {
  const h = harness(); await h.api.setPlannerStyle('walk');
  assert.deepEqual(h.sent().at(-1), { plannerStyle: 'walk', vx: 0, vy: 0, yaw: 0 });
  assert.equal(h.api.state.plannerStyle, 'slow_walk'); assert.equal(h.element('planner-style').disabled, true);
  await h.frame({ planner_style: 'slow_walk' }); assert.equal(h.api.state.pendingPlannerStyle, 'walk');
  await h.frame({ planner_style: 'walk' });
  assert.equal(h.api.state.pendingPlannerStyle, null); assert.equal(h.element('planner-style').value, 'walk');
  assert.equal(h.element('velocity').min, '0.8'); assert.equal(h.element('velocity').max, '2.5'); assert.equal(h.element('velocity').value, '0.9');
});
await check('Run and slow-walk use the requested default and permitted speed ranges', async () => {
  const h = harness(); await h.api.setPlannerStyle('run'); await h.frame({ planner_style: 'run' });
  assert.equal(h.element('velocity').min, '2.5'); assert.equal(h.element('velocity').max, '4'); assert.equal(h.element('velocity').value, '2.5');
  await h.api.setPlannerStyle('slow_walk'); await h.frame({ planner_style: 'slow_walk' });
  assert.equal(h.element('velocity').min, '0.1'); assert.equal(h.element('velocity').max, '0.8'); assert.equal(h.element('velocity').value, '0.4');
});
await check('Validated style speed boundaries reach pure-forward held commands, including Run 4.0 m/s', async () => {
  const tested = [];
  for (const [style, bounds] of [['slow_walk', [0.1, 0.8]], ['walk', [0.8, 2.5]], ['run', [2.5, 4]]]) {
    const h = harness(); await h.api.setPlannerStyle(style); await h.frame({ planner_style: style });
    assert.equal(Number(h.element('velocity').min), bounds[0]); assert.equal(Number(h.element('velocity').max), bounds[1]);
    await h.key('keydown', 'KeyW');
    for (const speed of bounds) {
      h.element('velocity').value = String(speed); await h.event('velocity', 'input'); await h.flushMovement();
      assert.deepEqual(h.sent().at(-1), { vx: speed, vy: 0, yaw: 0 });
      tested.push({ style, requestedSpeed: speed });
    }
  }
  return { tested };
});
for (const [style, forwardMax, directionalMax] of [['slow_walk', 0.8, 0.8], ['walk', 2.5, 1.6], ['run', 4, 2.5]]) {
  await check(`${style}: high-speed cardinal and diagonal commands preserve direction and obey vector limits`, async () => {
    const tested = [];
    for (const [keys, x, y] of [
      [['KeyW'], 1, 0], [['KeyS'], -1, 0], [['KeyA'], 0, 1], [['KeyD'], 0, -1],
      [['KeyW', 'KeyA'], 1, 1], [['KeyW', 'KeyD'], 1, -1],
      [['KeyS', 'KeyA'], -1, 1], [['KeyS', 'KeyD'], -1, -1],
    ]) {
      const h = harness(); await h.api.setPlannerStyle(style); await h.frame({ planner_style: style });
      h.element('velocity').value = String(forwardMax); await h.event('velocity', 'input');
      for (const key of keys) await h.key('keydown', key);
      await h.flushMovement();
      const sent = h.sent().at(-1), expectedSpeed = x < 0 || y !== 0 ? directionalMax : forwardMax;
      assert.ok(Math.abs(sent.vx - x / Math.hypot(x, y) * expectedSpeed) < 1e-12);
      assert.ok(Math.abs(sent.vy - y / Math.hypot(x, y) * expectedSpeed) < 1e-12);
      assert.ok(Math.abs(Math.hypot(sent.vx, sent.vy) - expectedSpeed) < 1e-12);
      assert.equal(sent.yaw, 0);
      assert.equal(h.element('vx').textContent, sent.vx.toFixed(1));
      assert.equal(h.element('vy').textContent, sent.vy.toFixed(1));
      assert.equal(h.element('velocity').value, String(forwardMax));
      tested.push({ keys, slider: forwardMax, sent, norm: Math.hypot(sent.vx, sent.vy) });
    }
    return { tested };
  });
}
await check('Releasing a strafe component restores full forward speed; Q/E does not reduce translation', async () => {
  const h = harness(); await h.api.setPlannerStyle('run'); await h.frame({ planner_style: 'run' });
  h.element('velocity').value = '4'; await h.event('velocity', 'input');
  await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyA'); await h.key('keydown', 'KeyQ');
  assert.ok(Math.abs(Math.hypot(h.api.state.vx, h.api.state.vy) - 2.5) < 1e-12);
  await h.key('keyup', 'KeyA'); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: 4, vy: 0, yaw: 0.3 });
});
await check('Dragging the slider while holding a diagonal remains capped and preserves its ratio', async () => {
  const h = harness(); await h.api.setPlannerStyle('walk'); await h.frame({ planner_style: 'walk' });
  await h.key('keydown', 'KeyW'); await h.key('keydown', 'KeyD');
  for (const requested of [0.8, 1.6, 2.5]) {
    h.element('velocity').value = String(requested); await h.event('velocity', 'input'); await h.flushMovement();
    const sent = h.sent().at(-1);
    assert.ok(Math.abs(Math.hypot(sent.vx, sent.vy) - Math.min(requested, 1.6)) < 1e-12);
    assert.ok(Math.abs(sent.vx + sent.vy) < 1e-12);
  }
});
await check('Pointer and mixed keyboard/pointer inputs share the same directional speed cap', async () => {
  const h = harness(); await h.api.setPlannerStyle('run'); await h.frame({ planner_style: 'run' });
  h.element('velocity').value = '4'; await h.event('velocity', 'input');
  await h.pointer('back', 'pointerdown', 1); await h.flushMovement();
  assert.deepEqual(h.sent().at(-1), { vx: -2.5, vy: 0, yaw: 0 });
  await h.pointer('back', 'pointerup', 1); await h.key('keydown', 'KeyW'); await h.pointer('right', 'pointerdown', 2); await h.flushMovement();
  const sent = h.sent().at(-1);
  assert.ok(Math.abs(sent.vx - 2.5 / Math.sqrt(2)) < 1e-12);
  assert.ok(Math.abs(sent.vy + 2.5 / Math.sqrt(2)) < 1e-12);
});
await check('Direction-limit hint follows acknowledged style and identifies command versus measured speed', async () => {
  const h = harness();
  for (const [style, text] of [
    ['slow_walk', 'Forward up to 0.8 m/s; strafe/backward up to 0.8 m/s.'],
    ['walk', 'Forward up to 2.5 m/s; strafe/backward up to 1.6 m/s.'],
    ['run', 'Forward up to 4.0 m/s; strafe/backward up to 2.5 m/s.'],
    ['idle', 'Standing; no translation command.'],
  ]) {
    await h.api.setPlannerStyle(style); await h.frame({ planner_style: style });
    assert.equal(h.element('direction-limit').textContent, text);
  }
  assert.ok(html.includes('Requested planner speed; measured speed may differ.'));
});
await check('Idle disables movement, then a Walk acknowledgement restores it', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.api.setPlannerStyle('idle'); await h.frame({ planner_style: 'idle' });
  assert.equal(h.element('velocity').disabled, true); assert.ok(h.directions.every(button => button.disabled));
  assert.equal(h.api.heldKeys.size, 0); assert.equal(h.api.state.vx, 0);
  await h.key('keydown', 'KeyW'); assert.equal(h.api.state.vx, 0);
  await h.api.setPlannerStyle('walk'); await h.frame({ planner_style: 'walk' });
  assert.equal(h.element('velocity').disabled, false); assert.ok(h.directions.every(button => !button.disabled));
  await h.key('keydown', 'KeyW'); assert.equal(h.api.state.vx, 0.9);
});
await check('Movement and keepalive commands cannot implicitly switch controller mode', async () => {
  const h = harness(); await h.key('keydown', 'KeyW'); await h.flushMovement();
  for (const callback of h.intervals) await callback();
  assert.ok(h.sent().length >= 2); assert.ok(h.sent().every(payload => !('mode' in payload)));
});

await check('A G1 target without preview sends a real tracking reset without fetching or rendering a preview', async () => {
  const h=harness('tracking');
  h.api.state.frames=new Float32Array(315);
  await h.api.loadMotion(g1Motion.id);
  assert.equal(h.api.state.motion.id,g1Motion.id); assert.equal(h.api.state.frames,null);
  assert.deepEqual(h.fetched,[]);
  assert.deepEqual(h.sent(),[{motion:g1Motion.id,action:'reset'}]);
  assert.equal(h.element('reference-only').disabled,true);
  assert.match(h.element('reference-only').title,/no reference-only preview/);
  assert.match(h.element('motion-reference-note').textContent,/G1 reference targets for the H2 controller/);
});
await check('No-preview G1 cannot enter reference-only even through a direct event call', async () => {
  const h=harness('tracking'); await h.api.loadMotion(g1Motion.id);
  const count=h.sent().length; await h.event('reference-only','click');
  assert.equal(h.api.state.referenceOnly,false); assert.equal(h.api.state.live,true);
  assert.equal(h.sent().length,count); assert.deepEqual(h.suspensions,[]);
});
await check('Reference-only disables only missing previews and returning to simulation restores their options', async () => {
  const h=harness('tracking'); h.api.enterReferenceOnly();
  const options=h.element('motion').options;
  assert.equal(h.api.state.referenceOnly,true); assert.equal(options[0].disabled,false); assert.equal(options[1].disabled,true);
  const count=h.sent().length; await h.api.loadMotion(g1Motion.id);
  assert.equal(h.api.state.motion.id,nativeMotion.id); assert.equal(h.sent().length,count); assert.deepEqual(h.fetched,[]);
  await h.api.connect();
  assert.equal(h.api.state.referenceOnly,false); assert.ok(options.every(option=>!option.disabled));
});
await check('Returning from a G1 target to H2 preserves the existing H2 preview path', async () => {
  const h=harness('tracking'); await h.api.loadMotion(g1Motion.id); await h.api.loadMotion(nativeMotion.id);
  assert.deepEqual(h.fetched,[`assets/${nativeMotion.file}`]);
  assert.equal(h.api.state.frames.length,315); assert.equal(h.element('reference-only').disabled,false);
  assert.deepEqual(h.sent().at(-1),{motion:nativeMotion.id,action:'reset'});
});
await check('Live source wording follows acknowledged G1 tracking rather than changing the H2 renderer', async () => {
  const h=harness('tracking'); await h.frame({mode:'tracking',reference_space:'g1',encoder_branch:'g1_selector_3'});
  assert.match(h.element('source-label').textContent,/G1 reference → H2 policy \+ physics/);
  assert.equal(h.element('robot-label').textContent,'UNITREE H2 · CLOSED LOOP');
  await h.frame({mode:'planner',reference_space:'g1',encoder_branch:'g1_selector_3'});
  assert.equal(h.element('source-label').textContent,'In your browser · MuJoCo + H2 policy');
});

function fixtureReference(space='g1',fps=50) {
  const width=space==='g1'?29:31;
  return {id:`${space}-fixture`,referenceSpace:space,fps,frames:12,
    rootQuat:Array.from({length:12},()=>[1,0,0,0]),
    [space==='g1'?'q29G1':'q31']:Array.from({length:12},(_,f)=>Array.from({length:width},(_,i)=>(f*width+i)/1000)),
    [space==='g1'?'qd29G1':'qd31']:Array.from({length:12},()=>Array.from({length:width},(_,i)=>(i+1)/100)),
  };
}
await check('Typed reference preparation preserves raw G1 semantic values and rejects mixed or malformed spaces', async () => {
  const raw=fixtureReference(), prepared=prepareTrackingReference(raw);
  assert.equal(prepared.frames[0].policyQ,raw.q29G1[0]); assert.equal(prepared.frames[0].policyDq,raw.qd29G1[0]);
  assert.equal(prepared.encoderMode,3); assert.equal(prepared.frames[0].q,undefined);
  const badCases=[{...raw,referenceType:'h2'},{...raw,encoderMode:0},{...raw,initialQpos:[0]},
    {...raw,q31:[]},{...raw,frames:13},{...raw,fps:0},{...raw,rootQuat:[[0,0,0,0]]},
    {...raw,q29G1:raw.q29G1.map(row=>row.slice(1))},
    {...raw,qd29G1:raw.qd29G1.map((row,i)=>i===0?row.map(()=>NaN):row)}];
  for(const bad of badCases) assert.throws(()=>prepareTrackingReference(bad));
  const native=fixtureReference('h2'); delete native.referenceSpace;
  const legacy=prepareTrackingReference(native);
  assert.equal(legacy.referenceSpace,'h2'); assert.equal(legacy.frames[0].q,native.q31[0]);
});
await check('Tracking routes native G1 through its encoder and H2 actions while H2 references retain the native branch', async () => {
  const config=JSON.parse(await readFile(new URL('../demo/local-assets/controller.json',import.meta.url),'utf8'));
  config.planner={};
  const calls={native:0,g1:0,decoder:0,physics:0};
  let captured;
  const physics={data:{qpos:new Float64Array(config.initialQpos),qvel:new Float64Array(67)},
    reset({qpos}) {this.data.qpos.set(qpos);this.data.qvel.fill(0);},
    stepPD(target) {assert.equal(target.length,31);calls.physics++;}};
  const inference={encode:async input=>{calls.native++;captured=input.slice();return new Float32Array(64);},
    encodePlanner:async input=>{calls.g1++;captured=input.slice();return new Float32Array(64);},
    decode:async input=>{calls.decoder++;assert.equal(input.length,994);return new Float32Array(29);}};
  const g1raw=fixtureReference(),h2raw=fixtureReference('h2');
  const motions={[g1raw.id]:prepareTrackingReference(g1raw),[h2raw.id]:prepareTrackingReference(h2raw)};
  const policy=new LocalPolicy(physics,inference,config,motions); policy.reset(g1raw.id); await policy.step();
  assert.equal(policy.referenceSpace,'g1'); assert.equal(policy.encoderBranch,'g1_selector_3');
  assert.deepEqual(calls,{native:0,g1:1,decoder:1,physics:1});
  for(let i=0;i<29;i++) {assert.equal(captured[i],Math.fround(g1raw.q29G1[0][i]));assert.equal(captured[290+i],Math.fround(g1raw.qd29G1[0][i]));}
  assert.deepEqual(Array.from(physics.data.qpos),config.initialQpos);
  policy.command({motion:h2raw.id,action:'reset'}); await policy.step();
  assert.equal(policy.encoderBranch,'h2_native'); assert.deepEqual(calls,{native:1,g1:1,decoder:2,physics:2});
  for(let i=0;i<29;i++) assert.equal(captured[i],Math.fround(h2raw.q31[0][config.policyToLocal[i]]));
  policy.command({motion:g1raw.id,action:'reset'}); policy.time=1; policy.complete=true;
  const held=policy.reference(0); assert.ok(held.policyDq.every(value=>value===0));
  assert.ok(g1raw.qd29G1[0].some(value=>value!==0));
});
await check('Startup hold metadata defaults to zero, accepts matching manifest configuration and rejects conflicts', async () => {
  const raw=fixtureReference();
  assert.equal(prepareTrackingReference(raw).startupHoldSeconds,0);
  assert.equal(prepareTrackingReference(raw,{startupHoldSeconds:.5}).startupHoldSeconds,.5);
  assert.equal(prepareTrackingReference({...raw,startupHoldSeconds:.5},{startupHoldSeconds:.5}).startupHoldSeconds,.5);
  assert.throws(()=>prepareTrackingReference({...raw,startupHoldSeconds:.5},{startupHoldSeconds:1}));
  for(const startupHoldSeconds of [-.1,Infinity,NaN,'0.5']) assert.throws(()=>prepareTrackingReference(raw,{startupHoldSeconds}));
});
await check('G1 startup runs exactly 25 true controller steps with frame-zero futures, then advances without resetting physics or history', async () => {
  const config=JSON.parse(await readFile(new URL('../demo/local-assets/controller.json',import.meta.url),'utf8')); config.planner={};
  const raw=fixtureReference(), ref=prepareTrackingReference(raw,{startupHoldSeconds:.5});
  const calls={resets:0,g1:0,decoder:0,physics:0}; const inputs=[];
  const trackedAddress=config.localQposAddresses[config.policyToLocal[0]];
  const physics={data:{qpos:new Float64Array(config.initialQpos),qvel:new Float64Array(67)},
    reset({qpos}) {calls.resets++;this.data.qpos.set(qpos);this.data.qvel.fill(0);},
    stepPD() {calls.physics++;this.data.qpos[trackedAddress]+=.001;}};
  const inference={encode:async()=>{throw new Error('G1 must not enter the native encoder');},
    encodePlanner:async input=>{calls.g1++;inputs.push(input.slice());return new Float32Array(64);},
    decode:async()=>{calls.decoder++;return new Float32Array(29).fill(.1);}};
  const policy=new LocalPolicy(physics,inference,config,{[raw.id]:ref}); policy.reset(raw.id);
  const heading=policy.headingOffset;
  assert.equal(policy.startupStepsRemaining,25); assert.equal(policy.motionPreparing,true);
  for(let step=0;step<25;step++) {
    await policy.step(); assert.equal(policy.time,0); assert.equal(policy.complete,false);
    assert.equal(policy.startupStepsRemaining,24-step);
    const input=inputs.at(-1);
    for(let f=0;f<10;f++) for(let i=0;i<29;i++) {
      assert.equal(input[f*29+i],Math.fround(raw.q29G1[0][i])); assert.equal(input[290+f*29+i],0);
    }
  }
  assert.equal(policy.motionPreparing,false); assert.equal(policy.steps,25); assert.equal(calls.resets,1);
  assert.ok(Math.abs(policy.history.q[261]-.024)<1e-5,'The H2 observation history must accumulate during startup');
  assert.equal(policy.lastAction[0],Math.fround(.1));
  await policy.step();
  assert.equal(policy.time,.02); assert.equal(policy.steps,26); assert.equal(policy.headingOffset,heading);
  assert.equal(calls.resets,1); assert.equal(calls.g1,26); assert.equal(calls.decoder,26); assert.equal(calls.physics,26);
  assert.equal(inputs.at(-1)[29],Math.fround(raw.q29G1[5][0]));
  assert.equal(inputs.at(-1)[290],Math.fround(raw.qd29G1[0][0]));
});
await check('Pausing startup freezes its counter; resume continues while reset and completed-motion replay start a fresh hold', async () => {
  const config=JSON.parse(await readFile(new URL('../demo/local-assets/controller.json',import.meta.url),'utf8')); config.planner={};
  const raw=fixtureReference(),native=fixtureReference('h2'); let resets=0,steps=0;
  const physics={data:{qpos:new Float64Array(config.initialQpos),qvel:new Float64Array(67)},reset({qpos}){resets++;this.data.qpos.set(qpos);},stepPD(){steps++;}};
  const inference={encode:async()=>new Float32Array(64),encodePlanner:async()=>new Float32Array(64),decode:async()=>new Float32Array(29)};
  const policy=new LocalPolicy(physics,inference,config,{[raw.id]:prepareTrackingReference(raw,{startupHoldSeconds:.5}),[native.id]:prepareTrackingReference(native)});
  policy.reset(raw.id); for(let i=0;i<7;i++) await policy.step();
  policy.command({action:'pause'}); await policy.step(); await policy.step();
  assert.equal(policy.startupStepsRemaining,18); assert.equal(steps,7); assert.equal(policy.time,0);
  policy.command({action:'play'}); await policy.step();
  assert.equal(policy.startupStepsRemaining,17); assert.equal(resets,1); assert.equal(steps,8);
  policy.command({action:'reset'}); assert.equal(policy.startupStepsRemaining,25); assert.equal(resets,2);
  policy.startupStepsRemaining=0; policy.complete=true; policy.command({action:'play'});
  assert.equal(policy.startupStepsRemaining,25); assert.equal(policy.time,0); assert.equal(resets,3);
  policy.command({mode:'planner',action:'reset'}); assert.equal(policy.startupStepsRemaining,0); assert.equal(policy.motionPreparing,false);
  policy.command({mode:'tracking',motion:native.id,action:'reset'}); assert.equal(policy.startupStepsRemaining,0);
  await policy.step(); assert.equal(policy.time,.02);
});
await check('Preparing motion status keeps Pause available, reports paused preparation, and clears after startup', async () => {
  const h=harness('tracking');
  await h.frame({motion_preparing:true,motion_prepare_remaining:.4,reference_space:'g1'});
  assert.equal(h.element('connection').lastElementChild.textContent,'Preparing motion');
  assert.match(h.element('runtime-note').textContent,/0.4 s remaining/);
  assert.equal(h.element('play').disabled,false); assert.equal(h.element('play').textContent,'Pause');
  await h.event('play','click'); assert.deepEqual(h.sent().at(-1),{action:'pause'});
  await h.frame({status:'paused',motion_preparing:true,motion_prepare_remaining:.4});
  assert.equal(h.element('play').textContent,'Resume'); assert.match(h.element('runtime-note').textContent,/preparation and physics are paused/);
  await h.event('play','click'); assert.deepEqual(h.sent().at(-1),{action:'play'});
  await h.frame({motion_preparing:false,motion_prepare_remaining:0});
  assert.equal(h.element('connection').lastElementChild.textContent,'Browser policy + physics');
  assert.doesNotMatch(h.element('runtime-note').textContent,/remaining|holding the first target/);
});

report.status = report.checks.every(item => item.status === 'pass') ? 'pass' : 'fail';
await mkdir(new URL('./', outputURL), { recursive: true });
await writeFile(outputURL, JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report, null, 2));
console.log(`Saved ${fileURLToPath(outputURL)}`);
if (report.status !== 'pass') process.exitCode = 1;
