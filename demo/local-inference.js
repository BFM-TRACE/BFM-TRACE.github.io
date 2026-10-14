/**
 * Real, local ONNX inference for the browser simulation worker.
 *
 * const inference = await createLocalInference({ provider: 'wasm', onProgress });
 * const tokens = await inference.encode(encoderObservation); // Float32Array
 * const gaitTokens = await inference.encodePlanner(g1Observation); // Float32Array
 * const actions = await inference.decode(decoderObservation); // Float32Array
 * // Multiple-input models use named feeds: { name: { data, dims, type } }.
 * const outputs = await inference.run('planner', feeds); // ORT tensor dictionary
 * await inference.release();
 *
 * This module performs no reference playback, physics, observation assembly,
 * action rescaling, or planner substitution. The caller owns those operations.
 */

const VENDOR = new URL('./vendor/ort/', import.meta.url);
const DEFAULT_MODELS = {
  encoderFull: new URL('./local-assets/models/encoder_full.onnx?v=c0938bb3da2f', import.meta.url).href,
  decoder: new URL('./local-assets/models/decoder.onnx?v=6d44817fc666', import.meta.url).href,
};
let runtimePromise;

export async function localInferenceCapabilities() {
  return {
    wasm: typeof WebAssembly !== 'undefined', wasmThreads: 1, webgpu: false,
    webgpuReason: 'This compact demo bundles the CPU WASM runtime only.',
  };
}

async function runtime() {
  runtimePromise ??= import('./vendor/ort/ort.wasm.min.mjs').then(ort => {
    // This module runs inside the caller's worker, not ORT's proxy worker.
    // One WASM thread avoids SharedArrayBuffer and COOP/COEP requirements.
    ort.env.wasm.numThreads = 1;
    ort.env.wasm.proxy = false;
    ort.env.wasm.wasmPaths = VENDOR.href;
    ort.env.logLevel = 'warning';
    return ort;
  });
  return runtimePromise;
}

function modelURL(value) {
  if (value instanceof Uint8Array || value instanceof ArrayBuffer) return value;
  return new URL(value, import.meta.url).href;
}

function dimensions(metadata, count, override) {
  if (override) return override;
  const shape = [...metadata.shape];
  const unknown = shape.map((value, index) => typeof value === 'number' && value > 0 ? -1 : index).filter(index => index >= 0);
  if (unknown.length > 1) throw new Error(`Input ${metadata.name} has dynamic dimensions; provide {data, dims}.`);
  if (unknown.length === 1) {
    const product = shape.reduce((total, value) => total * (typeof value === 'number' && value > 0 ? value : 1), 1);
    if (count % product) throw new Error(`Input ${metadata.name} cannot fit ${count} values into the model shape.`);
    shape[unknown[0]] = count / product;
  }
  return shape;
}

function tensorFor(ort, metadata, value) {
  if (value instanceof ort.Tensor) return value;
  const spec = value?.data && !ArrayBuffer.isView(value) ? value : { data: value };
  const type = spec.type || metadata.type;
  let data = spec.data;
  if (type === 'float32' && !(data instanceof Float32Array)) data = Float32Array.from(data);
  if (type === 'int64' && !(data instanceof BigInt64Array)) data = BigInt64Array.from(data, BigInt);
  if (type === 'int32' && !(data instanceof Int32Array)) data = Int32Array.from(data);
  if (!ArrayBuffer.isView(data) && !Array.isArray(data)) throw new TypeError(`Input ${metadata.name} requires a typed array or {data, dims}.`);
  const dims = dimensions(metadata, data.length, spec.dims);
  if (dims.reduce((total, dim) => total * dim, 1) !== data.length) throw new Error(`Input ${metadata.name} has ${data.length} values but shape [${dims}] requires a different count.`);
  if ((type === 'float32' || type === 'float64') && !data.every(Number.isFinite)) throw new Error(`Input ${metadata.name} contains a non-finite value.`);
  return new ort.Tensor(type, data, dims);
}

/**
 * Load the exact released universal encoder and H2 decoder from SONIC_unitree_h2.
 * H2/G1 compact observation buffers are placed in selectors 0/3 of the 1751D
 * source layout. SMPL selector 2 carries 720 joints, 60 orientations and 60 wrists.
 * No model graph extraction, weight conversion or CDN dependency is used.
 *
 * models entries can be a URL/Uint8Array or {url, externalData, options}.
 * External data entries follow ORT: {path: modelEmbeddedPath, data: localURL}.
 * encodePlanner encodes already planned G1 targets with selector 3. The
 * approximately 739 MiB neural planner is not bundled in this browser demo.
 * onProgress receives {phase, model, provider, elapsedMs?}; it is not a
 * download-percentage estimate. Sessions are initialized sequentially to
 * avoid peak memory from several model compilations at once.
 *
 * This compact bundle supports CPU WASM only. Unsupported providers fail
 * explicitly; no additional runtime or model is fetched from a CDN.
 */
export async function createLocalInference({
  provider = 'wasm', models = DEFAULT_MODELS, onProgress = () => {}, signal,
} = {}) {
  if (provider !== 'wasm') throw new Error('This compact demo supports the CPU WASM provider only.');
  signal?.throwIfAborted();
  onProgress({ phase: 'runtime', provider });
  const ort = await runtime();
  const sessions = {};
  const metadata = {};
  let released = false;
  try {
    for (const [name, value] of Object.entries(models)) {
      if (!value) continue;
      signal?.throwIfAborted();
      const spec = typeof value === 'string' || value instanceof URL || value instanceof Uint8Array || value instanceof ArrayBuffer ? { url: value } : value;
      const started = performance.now();
      onProgress({ phase: 'loading', model: name, provider });
      const options = {
        graphOptimizationLevel: 'all', executionMode: 'sequential',
        preferredOutputLocation: 'cpu', ...spec.options,
        // Do not let model-specific options override the requested device.
        executionProviders: [provider],
      };
      if (spec.externalData) options.externalData = spec.externalData.map(entry => ({
        path: entry.path, data: typeof entry.data === 'string' || entry.data instanceof URL ? modelURL(entry.data) : entry.data,
      }));
      sessions[name] = await ort.InferenceSession.create(modelURL(spec.url), options);
      signal?.throwIfAborted();
      metadata[name] = {
        inputs: sessions[name].inputMetadata,
        outputs: sessions[name].outputMetadata,
        loadMs: performance.now() - started,
      };
      onProgress({ phase: 'ready', model: name, provider, elapsedMs: metadata[name].loadMs });
    }
  } catch (error) {
    await Promise.allSettled(Object.values(sessions).map(session => session.release()));
    throw error;
  }

  async function run(name, input) {
    if (released) throw new Error('The local inference sessions were released.');
    const session = sessions[name];
    if (!session) throw new Error(`Model ${name} is not loaded. No substitute inference is available.`);
    const single = ArrayBuffer.isView(input) || Array.isArray(input) || input instanceof ort.Tensor || (input?.data && input?.dims);
    if (single && session.inputNames.length !== 1) throw new Error(`Model ${name} needs named inputs: ${session.inputNames.join(', ')}.`);
    const supplied = single ? { [session.inputNames[0]]: input } : input;
    const feeds = {};
    for (let i = 0; i < session.inputNames.length; i++) {
      const key = session.inputNames[i];
      if (supplied?.[key] === undefined) throw new Error(`Missing ${name} input ${key}.`);
      feeds[key] = tensorFor(ort, session.inputMetadata[i], supplied[key]);
    }
    const outputs = await session.run(feeds);
    for (const [key, tensor] of Object.entries(outputs)) {
      if ((tensor.type === 'float32' || tensor.type === 'float64') && !tensor.data.every(Number.isFinite)) {
        for (const output of Object.values(outputs)) output.dispose();
        throw new Error(`${name} output ${key} contains a non-finite value. Physics must not consume it.`);
      }
    }
    return outputs;
  }
  async function firstOutput(name, input) {
    const outputs = await run(name, input);
    const data = outputs[sessions[name].outputNames[0]].data.slice();
    for (const output of Object.values(outputs)) output.dispose();
    return data;
  }
  function encodeReference(input, selector) {
    const full = new Float32Array(1751);
    full[0] = selector;
    if (selector < 3) full[1 + selector] = 1;
    if (selector === 2) {
      if (input.length !== 840) throw new Error('SMPL observations require 720 joints, 60 orientations and 60 wrist values.');
      full.set(input.subarray(0, 720), 911);
      full.set(input.subarray(720, 780), 1631);
      full.set(input.subarray(780, 840), 1691);
    } else {
      if (input.length !== 640) throw new Error('Robot reference observations require 640 values.');
      full.set(input.subarray(0, 580), 4);
      full.set(input.subarray(580, 640), 590);
    }
    return firstOutput('encoderFull', full);
  }
  return {
    provider, runtimeVersion: ort.env.versions.web || '1.30.0', metadata, sessions,
    run,
    encode: input => encodeReference(input, 0),
    encodePlanner: input => encodeReference(input, 3),
    encodeSMPL: input => encodeReference(input, 2),
    decode: input => firstOutput('decoder', input),
    plan: input => run('planner', input),
    tensor: (type, data, dims) => new ort.Tensor(type, data, dims),
    async release() {
      if (released) return;
      released = true;
      await Promise.all(Object.values(sessions).map(session => session.release()));
    },
  };
}
