// Verify the shipped G1 branch with the same local WASM runtime as the demo.
// Run: node homepage/demo/check_g1_encoder.mjs
// Requires no npm dependencies and does not start physics or a server.
import {readFile, writeFile} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {createLocalInference} from './local-inference.js';

const asset = name => new URL(`./local-assets/models/${name}`, import.meta.url);
const fixtures = JSON.parse(await readFile(asset('encoder-g1-fixtures.json'), 'utf8'));
const validation = JSON.parse(await readFile(asset('encoder-g1-validation.json'), 'utf8'));
const model = new Uint8Array(await readFile(asset('encoder_g1.onnx')));
const modelHash = createHash('sha256').update(model).digest('hex');
if (modelHash !== validation.sha256) throw new Error('G1 encoder hash differs from the validated artifact.');
const inference = await createLocalInference({models: {
  encoder: new Uint8Array(await readFile(asset('encoder_native.onnx'))),
  decoder: new Uint8Array(await readFile(asset('decoder.onnx'))),
  plannerEncoder: model,
}});
const report = {
  provider: inference.provider, runtimeVersion: inference.runtimeVersion,
  runtimeVariant: 'CPU-only WASM SIMD, one thread in Node', modelHash,
  sourceFullSha256: fixtures.sourceFullSha256, sourceSelector: fixtures.selector,
  metadata: inference.metadata, cases: [], maxAbsError: 0,
  boundary: 'Real planner observations compared with source full encoder selector 3. Numerical and API verification only; no physics or browser performance claim.',
};
try {
  for (const fixture of fixtures.fixtures) {
    const started = performance.now();
    const tokens = await inference.encodePlanner(Float32Array.from(fixture.input));
    if (tokens.length !== 64) throw new Error('G1 encoder did not return 64 tokens.');
    const error = Math.max(...tokens.map((value, i) => Math.abs(value - fixture.expectedTokens[i])));
    if (error > 1e-6) throw new Error(`${fixture.label}: G1 WASM differs from full selector 3 by ${error}.`);
    report.cases.push({label: fixture.label, maxAbsError: error, inferenceMs: performance.now() - started});
    report.maxAbsError = Math.max(report.maxAbsError, error);
    const decoderInput = new Float32Array(994);
    decoderInput.set(tokens);
    const action = await inference.decode(decoderInput);
    if (action.length !== 29 || !action.every(Number.isFinite)) throw new Error('G1 tokens did not produce finite H2 decoder actions.');
  }
  const trackingTokens = await inference.encode(new Float32Array(640));
  if (trackingTokens.length !== 64 || !trackingTokens.every(Number.isFinite)) throw new Error('The original H2 tracking encoder failed.');
  report.originalTrackingEncoderSmoke = 'passed';
  report.status = 'passed';
  await writeFile(new URL('./local-inference-g1-wasm-qa.json', import.meta.url), `${JSON.stringify(report, null, 2)}\n`);
  console.log(JSON.stringify({status: report.status, cases: report.cases.length, maxAbsError: report.maxAbsError, modelHash}));
} finally {
  await inference.release();
}
