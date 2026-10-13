import assert from "node:assert/strict";
import { createHash } from "node:crypto";
import { readFile, stat } from "node:fs/promises";
import http from "node:http";
import { spawn } from "node:child_process";
import { once } from "node:events";
import { fileURLToPath } from "node:url";
import path from "node:path";
import { prepareTrackingReference } from "../demo/local-reference.js";

const root = fileURLToPath(new URL("../", import.meta.url));
const html = await readFile(path.join(root, "index.html"), "utf8");
const media = JSON.parse(
  await readFile(path.join(root, "content/media.json"), "utf8"),
);
const resources = new Set(
  [...html.matchAll(/(?:src|href|data-src|poster)="([^"]+)"/g)]
    .map((m) => m[1])
    .filter((x) => !x.startsWith("#") && !/^https?:/.test(x))
    .map((x) => x.split(/[?#]/)[0]),
);
for (const item of media.items) {
  resources.add(item.src);
  resources.add(item.poster);
}
// Every selectable motion needs a typed local target; H2 reference previews
// additionally require a complete binary. G1-only entries explicitly omit it.
const motionCatalog = JSON.parse(await readFile(path.join(root, "demo/assets/motions.json"), "utf8"));
const targetCatalog = JSON.parse(await readFile(path.join(root, "demo/local-assets/references/manifest.json"), "utf8"));
assert.equal(new Set(motionCatalog.motions.map(m => m.id)).size, motionCatalog.motions.length);
assert.equal(targetCatalog.localNames.length, 31);
for (const motion of motionCatalog.motions) {
  const target = targetCatalog.motions.find(m => m.id === motion.id);
  assert.ok(target, `Missing physics target for ${motion.id}`);
  const targetPath = `demo/local-assets/references/${target.file}`;
  assert.ok(!targetPath.includes("..") && !path.isAbsolute(target.file));
  resources.add(targetPath);
  const targetBytes = await readFile(path.join(root, targetPath));
  const targetHash = motion.referenceSHA256 || target.sha256;
  if (targetHash) assert.equal(createHash("sha256").update(targetBytes).digest("hex"), targetHash, `${motion.id}: target hash`);
  const reference = JSON.parse(targetBytes);
  const prepared = prepareTrackingReference(reference, target);
  assert.equal(reference.fps, 50);
  assert.equal(reference.id, motion.id);
  assert.equal(prepared.referenceSpace, motion.referenceSpace || "h2");
  if (motion.previewAvailable === false) {
    assert.equal(motion.file, null, `${motion.id}: preview must be explicitly absent`);
    assert.equal(prepared.referenceSpace, "g1");
  } else {
    assert.ok(typeof motion.file === "string" && !path.isAbsolute(motion.file) && !motion.file.includes(".."));
    const previewPath = `demo/assets/${motion.file}`;
    const previewBytes = await readFile(path.join(root, previewPath));
    resources.add(previewPath);
    assert.equal(previewBytes.byteLength, motion.frames * motionCatalog.bodyCount * 7 * 4, `${motion.id}: preview shape`);
    if (motion.previewSHA256) assert.equal(createHash("sha256").update(previewBytes).digest("hex"), motion.previewSHA256, `${motion.id}: preview hash`);
  }
}
for (const resource of resources)
  assert(
    (await stat(path.join(root, resource))).isFile(),
    `Missing ${resource}`,
  );
for (const [, anchor] of html.matchAll(/href="#([^"]+)"/g))
  assert(html.includes(`id="${anchor}"`), `Missing anchor ${anchor}`);
assert.equal(media.items.filter((x) => x.category === "motion").length, 8);
assert.equal(
  media.items.filter((x) => x.category === "teleoperation").length,
  5,
);
assert.equal(media.items.filter((x) => x.category === "vla").length, 4);

// Use an isolated fake upstream: this check never sends commands to a robot
// or the user's live simulation and does not require the SSH tunnel.
const upstream = http.createServer(async (req, res) => {
  const chunks = [];
  for await (const chunk of req) chunks.push(chunk);
  const body = Buffer.concat(chunks).toString();
  res.writeHead(req.method === "POST" ? 202 : 200, {
    "content-type": "application/json",
  });
  res.end(
    JSON.stringify({
      method: req.method,
      path: req.url,
      length: req.headers["content-length"],
      body,
    }),
  );
});
upstream.listen(0, "127.0.0.1");
await once(upstream, "listening");
const child = spawn(process.execPath, ["server.mjs"], {
  cwd: root,
  env: {
    ...process.env,
    PORT: "0",
    HOST: "127.0.0.1",
    TRACE_BACKEND: `http://127.0.0.1:${upstream.address().port}`,
  },
  stdio: ["ignore", "pipe", "pipe"],
});
try {
  const url = await new Promise((resolve, reject) => {
    const timer = setTimeout(
      () => reject(new Error("Preview server did not start")),
      10000,
    );
    child.stdout.on("data", (chunk) => {
      const m = String(chunk).match(/TRACE homepage: (http:\/\/\S+)/);
      if (m) {
        clearTimeout(timer);
        resolve(m[1]);
      }
    });
    child.on("error", reject);
  });
  assert.equal((await fetch(url)).status, 200, "Root must serve index.html");
  assert.equal((await fetch(`${url}/demo/index.html`)).status, 200);
  const video = await fetch(`${url}/assets/hero-loop.mp4`, {
    headers: { Range: "bytes=0-255" },
  });
  assert.equal(video.status, 206);
  assert.equal((await video.arrayBuffer()).byteLength, 256);
  assert.equal(video.headers.get("content-type"), "video/mp4");
  assert.equal(
    (
      await fetch(`${url}/assets/hero-loop.mp4`, {
        headers: { Range: "bytes=999999999-" },
      })
    ).status,
    416,
  );
  assert.equal((await fetch(`${url}/backend/live_server.py`)).status, 404);
  assert.equal((await fetch(`${url}/.env`)).status, 404);
  assert.equal((await fetch(`${url}/missing.html`)).status, 404);
  const payload = JSON.stringify({ action: "stop" });
  const response = await fetch(`${url}/api/command`, {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: payload,
  });
  assert.equal(response.status, 202);
  const forwarded = await response.json();
  assert.equal(forwarded.body, payload);
  assert.equal(Number(forwarded.length), Buffer.byteLength(payload));
  assert.equal(forwarded.path, "/api/command");
  assert.equal(
    (
      await fetch(`${url}/api/command`, {
        method: "POST",
        body: "x".repeat(8193),
      })
    ).status,
    413,
  );
  console.log(
    `PASS: ${resources.size} assets, ${motionCatalog.motions.length} typed local motion targets and their declared previews, navigation anchors, 17 gallery clips, root/demo routes, video ranges, private paths, and simulation proxy.`,
  );
} finally {
  child.kill();
  upstream.close();
}
