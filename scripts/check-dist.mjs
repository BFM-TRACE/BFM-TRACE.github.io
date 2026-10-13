import assert from 'node:assert/strict';
import {readFile, readdir, mkdir, writeFile} from 'node:fs/promises';
import http from 'node:http';
import {once} from 'node:events';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {collectSiteFiles, homepageRoot, sha256, treeHash} from './build-site.mjs';

const options = {dir:path.join(homepageRoot,'dist'), output:path.join(homepageRoot,'qa/generated/static-build.json')};
for (let i=2;i<process.argv.length;i++) {
  const argument=process.argv[i];
  if (argument==='--help') {
    console.log('node scripts/check-dist.mjs [--dir DIST] [--output REPORT]\nVerify an already built static directory, including root/subpath HTTP delivery. No simulation server or browser automation is used.');
    process.exit(0);
  }
  assert.ok(['--dir','--output'].includes(argument) && process.argv[i+1], `Unknown or incomplete argument: ${argument}`);
  options[argument.slice(2)]=path.resolve(process.argv[++i]);
}
const bytes=await readFile(path.join(options.dir,'build-manifest.json'));
const manifest=JSON.parse(bytes), checks=[];
assert.equal(manifest.schema,1);
const expected=new Map(manifest.files.map(file=>[file.path,file]));
assert.equal(expected.size,manifest.files.length,'Duplicate build path');
assert.equal(manifest.payloadFileCount,expected.size);
assert.equal(manifest.totalFileCount,expected.size+1);
assert.equal(manifest.payloadBytes,manifest.files.reduce((sum,file)=>sum+file.bytes,0));
assert.equal(manifest.treeSha256,treeHash(manifest.files));

async function listFiles(directory, prefix='') {
  const files=[];
  for (const entry of await readdir(directory,{withFileTypes:true})) {
    const relative=prefix+entry.name;
    assert.ok(!entry.isSymbolicLink(),`Published symlink: ${relative}`);
    if(entry.isDirectory()) files.push(...await listFiles(path.join(directory,entry.name),relative+'/'));
    else {assert.ok(entry.isFile(),`Unexpected publication entry: ${relative}`); files.push(relative);}
  }
  return files;
}
const actual=(await listFiles(options.dir)).sort();
assert.deepEqual(actual,[...expected.keys(),'build-manifest.json'].sort(),'Extra or missing files in publication');
for(const file of manifest.files) {
  assert.ok(!file.path.startsWith('/') && !file.path.split('/').includes('..') && !file.path.includes('\\'),`Unsafe publication path: ${file.path}`);
  assert.ok(!/(^|\/)(?:qa|backend|candidates|scripts)(\/|$)/.test(file.path),`Development content was published: ${file.path}`);
  assert.ok(!/\.(?:py|npy|d\.ts|map)$/i.test(file.path),`Non-runtime source was published: ${file.path}`);
  const content=await readFile(path.join(options.dir,file.path));
  assert.equal(content.length,file.bytes,`Byte count: ${file.path}`);
  assert.equal(sha256(content),file.sha256,`Digest: ${file.path}`);
}
checks.push({name:'Complete publication inventory and SHA-256 integrity',status:'pass',files:manifest.totalFileCount});
const graph=await collectSiteFiles(options.dir);
assert.deepEqual([...expected.keys()].filter(name=>name!=='.nojekyll').sort(),graph.paths,'Published directory differs from its runtime dependency graph');
for(const [relative,hash] of graph.expectedHashes) assert.equal(expected.get(relative)?.sha256,hash,`Runtime catalog digest: ${relative}`);
for(const relative of ['demo/vendor/THREE-LICENSE','demo/vendor/mujoco/LICENSE','demo/vendor/ort/LICENSE','demo/vendor/ort/ThirdPartyNotices.txt']) assert.ok(expected.has(relative),`Missing license: ${relative}`);
const notices=await readFile(path.join(options.dir,'THIRD_PARTY_NOTICES.md'),'utf8');
for(const [, target] of notices.matchAll(/\[[^\]]+\]\(([^)]+)\)/g)) {
  if(/^(?:[a-z][a-z\d+.-]*:|#)/i.test(target)) continue;
  assert.ok(expected.has(target.split('#')[0]),`Notice link is missing from the publication: ${target}`);
}
checks.push({name:'Runtime manifests, module paths, typed targets, preview shapes and licenses',status:'pass',runtime:graph.counts});

const types={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.mjs':'text/javascript; charset=utf-8',
  '.css':'text/css; charset=utf-8','.json':'application/json','.wasm':'application/wasm','.mp4':'video/mp4',
  '.jpg':'image/jpeg','.png':'image/png','.webp':'image/webp','.svg':'image/svg+xml','.pdf':'application/pdf',
  '.vtt':'text/vtt; charset=utf-8','.xml':'application/xml'};
const mount='/project-site/';
const server=http.createServer(async(req,res)=>{
  try {
    let requested=decodeURIComponent(new URL(req.url,'http://localhost').pathname);
    if(requested.startsWith(mount)) requested=requested.slice(mount.length);
    else requested=requested.slice(1);
    if(!requested || requested.endsWith('/')) requested+='index.html';
    if(!expected.has(requested) && requested!=='build-manifest.json') {res.writeHead(404);res.end();return;}
    const content=await readFile(path.join(options.dir,requested));
    const headers={'content-type':types[path.extname(requested).toLowerCase()] || 'application/octet-stream','accept-ranges':'bytes'};
    if(req.headers.range) {
      const range=/^bytes=(\d+)-(\d*)$/.exec(req.headers.range);
      const start=range?Number(range[1]):NaN,end=range&&range[2]?Math.min(Number(range[2]),content.length-1):content.length-1;
      if(!Number.isSafeInteger(start) || start>end || start>=content.length) {res.writeHead(416,{'content-range':`bytes */${content.length}`});res.end();return;}
      const slice=content.subarray(start,end+1);
      res.writeHead(206,{...headers,'content-length':slice.length,'content-range':`bytes ${start}-${end}/${content.length}`});res.end(slice);return;
    }
    res.writeHead(200,{...headers,'content-length':content.length});res.end(req.method==='HEAD'?undefined:content);
  } catch(error) {res.writeHead(500);res.end(error.message);}
});
server.listen(0,'127.0.0.1'); await once(server,'listening');
try {
  const origin=`http://127.0.0.1:${server.address().port}`, root=origin+mount;
  for(const route of ['/',mount,mount+'demo/']) {
    const response=await fetch(origin+route); assert.equal(response.status,200,route); assert.match(response.headers.get('content-type'),/^text\/html/);
  }
  // Fetch the complete payload under a project subdirectory. The temporary
  // server exposes only this build, with no /api proxy or source checkout.
  let cursor=0;
  await Promise.all(Array.from({length:4},async()=>{
    while(cursor<manifest.files.length) {
      const file=manifest.files[cursor++];
      const response=await fetch(root+file.path,{signal:AbortSignal.timeout(30000)});
      assert.equal(response.status,200,`HTTP: ${file.path}`);
      assert.equal(Number(response.headers.get('content-length')),file.bytes);
      assert.equal(sha256(new Uint8Array(await response.arrayBuffer())),file.sha256,`HTTP bytes: ${file.path}`);
      const extension=path.extname(file.path).toLowerCase();
      if(types[extension]) assert.equal(response.headers.get('content-type'),types[extension],`MIME: ${file.path}`);
    }
  }));
  checks.push({name:'Root and project-subpath static serving; every payload fetched with correct bytes and MIME',status:'pass',fetchedFiles:manifest.files.length,projectMount:mount});
  const hero=expected.get('assets/hero-loop.mp4');
  const range=await fetch(root+'assets/hero-loop.mp4',{headers:{range:'bytes=0-255'}});
  assert.equal(range.status,206);assert.equal((await range.arrayBuffer()).byteLength,256);
  assert.equal(range.headers.get('content-type'),'video/mp4');
  assert.equal(range.headers.get('content-range'),`bytes 0-255/${hero.bytes}`);
  assert.equal((await fetch(root+'assets/hero-loop.mp4',{headers:{range:`bytes=${hero.bytes}-`}})).status,416);
  for(const route of ['api/state','backend/live_server.py','scripts/build-site.mjs','qa/planner-controls.json','demo/local-assets/planner-bank/candidates/manifest.json','missing.html']) assert.equal((await fetch(root+route)).status,404,route);
  checks.push({name:'Video byte ranges and absence of backend, QA, scripts and candidate routes',status:'pass'});
} finally {server.close();await once(server,'close');}

const report={status:'pass',validatedAt:new Date().toISOString(),scope:'Build integrity and static HTTP delivery, not a browser rendering or physical-policy regression.',
  files:manifest.totalFileCount,bytes:manifest.payloadBytes+bytes.length,treeSha256:manifest.treeSha256,
  manifestSha256:sha256(bytes),runtime:graph.counts,checks};
await mkdir(path.dirname(options.output),{recursive:true}); await writeFile(options.output,JSON.stringify(report,null,2)+'\n');
console.log(JSON.stringify(report,null,2));console.log(`Saved ${options.output}`);
