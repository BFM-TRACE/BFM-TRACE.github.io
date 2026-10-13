import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {lstat, mkdir, mkdtemp, readFile, realpath, rename, rm, writeFile} from 'node:fs/promises';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {plannerEntries} from '../demo/local-planner.js';
import {prepareTrackingReference} from '../demo/local-reference.js';

export const homepageRoot = fileURLToPath(new URL('../', import.meta.url));
export const sha256 = bytes => createHash('sha256').update(bytes).digest('hex');
export const treeHash = files => sha256(files.map(file => `${file.path}\0${file.bytes}\0${file.sha256}\n`).join(''));

// URLs loaded indirectly by the vendored engines are explicit, so an unused
// runtime, source map, declaration, candidate bank or QA file is never copied.
const engineFiles = [
  'demo/vendor/three.module.min.js', 'demo/vendor/three.core.min.js',
  'demo/vendor/OrbitControls.js', 'demo/vendor/THREE-LICENSE',
  'demo/vendor/mujoco/mujoco.js', 'demo/vendor/mujoco/mujoco.wasm', 'demo/vendor/mujoco/LICENSE', 'demo/vendor/mujoco/README.md',
  'demo/vendor/ort/ort.wasm.min.mjs', 'demo/vendor/ort/ort-wasm-simd-threaded.mjs',
  'demo/vendor/ort/ort-wasm-simd-threaded.wasm', 'demo/vendor/ort/LICENSE', 'demo/vendor/ort/ThirdPartyNotices.txt', 'demo/vendor/ort/provenance.json',
];
const catalogPaths = {
  media: 'content/media.json', visible: 'demo/assets/motions.json',
  targets: 'demo/local-assets/references/manifest.json',
  planner: 'demo/local-assets/planner-bank/manifest.json',
  physics: 'demo/local-assets/physics/manifest.json',
};

export async function collectSiteFiles(root = homepageRoot, {modelCatalog} = {}) {
  const paths = new Set(), expectedHashes = new Map(), imports = new Map();
  const json = async relative => JSON.parse(await readFile(path.join(root, relative), 'utf8'));
  function add(relative, base = '', expectedHash) {
    assert.equal(typeof relative, 'string', 'An asset URL must be a string');
    assert.ok(relative && !/^(?:[a-z][a-z\d+.-]*:|\/)/i.test(relative) && !relative.includes('\\'), `Expected a relative asset URL: ${relative}`);
    const target = path.posix.normalize(path.posix.join(base, relative));
    assert.ok(target !== '..' && !target.startsWith('../') && !/[?#]/.test(target), `Unsafe asset URL: ${relative}`);
    assert.ok(!/(^|\/)(?:qa|backend|candidates|scripts|dist)(\/|$)/.test(target), `A non-runtime directory was referenced: ${target}`);
    paths.add(target);
    if (expectedHash) {
      if (expectedHashes.has(target)) assert.equal(expectedHashes.get(target), expectedHash, `Conflicting hash for ${target}`);
      expectedHashes.set(target, expectedHash);
    }
    return target;
  }
  function url(value, base) {
    if (!value || /^(?:#|[a-z][a-z\d+.-]*:|\/\/)/i.test(value)) return;
    // A directory URL supplies the base for the engine's runtime filenames.
    if (value.endsWith('/')) return;
    add(decodeURIComponent(value.split(/[?#]/)[0]), base);
  }
  add('index.html'); add('demo/index.html'); add('THIRD_PARTY_NOTICES.md'); add('demo/local-assets/controller.json');
  Object.values(catalogPaths).forEach(relative => add(relative));
  engineFiles.forEach(relative => add(relative));
  const [media, visible, targets, bank, physics] = await Promise.all(Object.values(catalogPaths).map(json));
  for (const item of [media.hero, media.overview, ...media.items]) {
    for (const key of ['src', 'poster', 'captions']) if (item[key]) add(item[key]);
  }
  assert.equal(new Set(targets.motions.map(entry => entry.id)).size, targets.motions.length, 'Duplicate target id');
  // Keep all runtime-manifest references, including currently hidden native
  // targets, so the copied manifest remains byte-for-byte complete.
  for (const entry of targets.motions) {
    const relative = add(entry.file, 'demo/local-assets/references', entry.sha256);
    const reference = prepareTrackingReference(await json(relative), entry);
    assert.equal(reference.id, entry.id);
  }
  assert.equal(new Set(visible.motions.map(entry => entry.id)).size, visible.motions.length, 'Duplicate menu id');
  for (const motion of visible.motions) {
    const target = targets.motions.find(entry => entry.id === motion.id);
    assert.ok(target, `Missing policy target: ${motion.id}`);
    if (motion.referenceSHA256) add(target.file, 'demo/local-assets/references', motion.referenceSHA256);
    if (motion.previewAvailable === false) assert.equal(motion.file, null, 'Unavailable previews must be explicit');
    else {
      const relative = add(motion.file, 'demo/assets', motion.previewSHA256);
      assert.equal((await lstat(path.join(root, relative))).size, motion.frames * visible.bodyCount * 7 * 4, `Invalid preview shape: ${motion.id}`);
    }
  }
  const gaitEntries = plannerEntries(bank);
  for (const entry of gaitEntries) add(entry.file, 'demo/local-assets/planner-bank', entry.sha256);
  const virtualFiles = new Set(physics.files.map(file => file.path));
  assert.ok(virtualFiles.has(physics.entry), 'Physics entry is missing from the virtual filesystem');
  for (const file of physics.files) add(file.url || file.path, 'demo/local-assets/physics');
  if (physics.config) add(physics.config, 'demo/local-assets/physics');
  const xmlFile = physics.files.find(file => file.path === physics.entry);
  const xml = await readFile(path.join(root, 'demo/local-assets/physics', xmlFile.url || xmlFile.path), 'utf8');
  for (const [, filename] of xml.matchAll(/\bfile="([^"]+)"/g)) assert.ok(virtualFiles.has(filename), `Missing XML asset in physics manifest: ${filename}`);
  if (modelCatalog) {
    for (const model of Object.values(modelCatalog.models).filter(model => model.bundled)) {
      assert.equal(model.external_data, false, 'External ONNX tensor files require an explicit build dependency');
      add(model.file, 'demo/local-assets/models', model.sha256);
    }
  }

  // Follow static HTML/CSS and application module URLs. Manifest-dependent
  // template strings are handled above. Vendor-only dynamic filenames are in
  // engineFiles, kept beside their original license and notice documents.
  const scanned = new Set();
  while ([...paths].some(relative => !scanned.has(relative))) {
    for (const relative of [...paths]) {
      if (scanned.has(relative)) continue;
      scanned.add(relative);
      const extension = path.extname(relative), base = path.posix.dirname(relative);
      if (!['.html', '.css', '.js', '.mjs'].includes(extension) || relative.startsWith('demo/vendor/')) continue;
      const text = await readFile(path.join(root, relative), 'utf8');
      if (extension === '.html') {
        for (const [, value] of text.matchAll(/(?:src|href|data-src|poster)="([^"]+)"/g)) url(value, base);
        for (const [, content] of text.matchAll(/<script\b[^>]*type="importmap"[^>]*>([\s\S]*?)<\/script>/g)) {
          for (const [name, value] of Object.entries(JSON.parse(content).imports || {})) {
            imports.set(name, add(value, base));
          }
        }
      } else if (extension === '.css') {
        for (const [, value] of text.matchAll(/url\(\s*["']?([^\s"')]+)["']?\s*\)/g)) url(value, base);
      } else {
        const statements = /(?:\b(?:import|export)\s+(?:[^'";]*?\s+from\s*)?|\bimport\s*\(\s*)(['"])([^'"\n]+)\1/g;
        for (const [, , value] of text.matchAll(statements)) {
          if (imports.has(value)) add(imports.get(value));
          else { assert.ok(value.startsWith('.') || /^[a-z][a-z\d+.-]*:/i.test(value), `Unresolved module import: ${value}`); url(value, base); }
        }
        for (const [, , value] of text.matchAll(/\b(?:new\s+URL|fetch|json)\s*\(\s*(['"])([^'"\n]+)\1/g)) url(value, base);
      }
    }
  }
  const realRoot = await realpath(root);
  for (const relative of paths) {
    const absolute = path.join(root, relative);
    assert.ok((await lstat(absolute)).isFile(), `Missing regular asset: ${relative}`);
    assert.ok((await realpath(absolute)).startsWith(realRoot + path.sep), `Asset escapes site root: ${relative}`);
  }
  return {
    paths: [...paths].sort(), expectedHashes,
    counts: {galleryVideos: media.items.length, selectableMotions: visible.motions.length, trackingReferences: targets.motions.length, plannerReferences: gaitEntries.length, physicsFiles: physics.files.length},
  };
}

export async function buildSite() {
  const modelCatalog = JSON.parse(await readFile(path.join(homepageRoot, 'demo/local-assets/models/manifest.json'), 'utf8'));
  const plan = await collectSiteFiles(homepageRoot, {modelCatalog});
  const temporary = await mkdtemp(path.join(homepageRoot, '.dist-build-'));
  try {
    const files = [];
    for (const relative of plan.paths) {
      const bytes = await readFile(path.join(homepageRoot, relative));
      const hash = sha256(bytes), expected = plan.expectedHashes.get(relative);
      if (expected) assert.equal(hash, expected, `Source hash mismatch: ${relative}`);
      const destination = path.join(temporary, relative);
      await mkdir(path.dirname(destination), {recursive:true});
      await writeFile(destination, bytes);
      files.push({path:relative, bytes:bytes.length, sha256:hash});
    }
    await writeFile(path.join(temporary, '.nojekyll'), '');
    files.push({path:'.nojekyll', bytes:0, sha256:sha256('')}); files.sort((a,b)=>a.path < b.path ? -1 : a.path > b.path ? 1 : 0);
    const manifest = {schema:1, payloadFileCount:files.length, totalFileCount:files.length+1,
      payloadBytes:files.reduce((sum,file)=>sum+file.bytes,0), treeSha256:treeHash(files),
      digestScope:'Sorted payload paths, byte lengths and SHA-256 values; build-manifest.json is excluded from its own digest.',
      runtime:plan.counts, files};
    const manifestBytes = Buffer.from(JSON.stringify(manifest, null, 2)+'\n');
    await writeFile(path.join(temporary, 'build-manifest.json'), manifestBytes);
    const destination = path.join(homepageRoot, 'dist');
    await rm(destination, {recursive:true, force:true});
    await rename(temporary, destination);
    const summary = {output:'homepage/dist', files:manifest.totalFileCount, bytes:manifest.payloadBytes+manifestBytes.length,
      treeSha256:manifest.treeSha256, manifestSha256:sha256(manifestBytes), runtime:plan.counts};
    console.log(JSON.stringify(summary, null, 2));
    return summary;
  } finally { await rm(temporary, {recursive:true, force:true}); }
}

if (process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url)) {
  if (process.argv.includes('--help')) console.log('node scripts/build-site.mjs\nBuild a self-contained, deterministic static site in homepage/dist. No external packages required.');
  else {assert.equal(process.argv.length,2,'Unknown build argument'); await buildSite();}
}
