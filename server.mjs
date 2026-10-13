import http from "node:http";
import https from "node:https";
import { createReadStream } from "node:fs";
import { stat, realpath } from "node:fs/promises";
import { fileURLToPath } from "node:url";
import path from "node:path";

const root = path.resolve(fileURLToPath(new URL(".", import.meta.url)));
const isWithinRoot = (file) =>
  file === root || file.startsWith(root + path.sep);
const port = Number(process.env.PORT || 4173);
const host = process.env.HOST || "127.0.0.1";
const backend = new URL(process.env.TRACE_BACKEND || "http://127.0.0.1:8766");
const types = {
  ".html": "text/html; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".json": "application/json",
  ".mp4": "video/mp4",
  ".webm": "video/webm",
  ".vtt": "text/vtt; charset=utf-8",
  ".jpg": "image/jpeg",
  ".jpeg": "image/jpeg",
  ".png": "image/png",
  ".webp": "image/webp",
  ".svg": "image/svg+xml",
  ".pdf": "application/pdf",
  ".wasm": "application/wasm",
  ".glb": "model/gltf-binary",
  ".gltf": "model/gltf+json",
  ".bin": "application/octet-stream",
  ".stl": "application/octet-stream",
  ".woff2": "font/woff2",
};

const server = http.createServer(async (req, res) => {
  let requestURL, pathname;
  try {
    requestURL = new URL(req.url, "http://localhost");
    pathname = decodeURIComponent(requestURL.pathname);
  } catch {
    res.writeHead(400).end("Invalid URL");
    return;
  }
  if (pathname.startsWith("/api/")) {
    // Buffer the small JSON command body so chunked clients also reach the
    // simulation service with its required, accurate Content-Length header.
    const chunks = [];
    let bodyLength = 0;
    try {
      for await (const chunk of req) {
        bodyLength += chunk.length;
        if (bodyLength > 8192) {
          res
            .writeHead(413, { "content-type": "application/json" })
            .end(JSON.stringify({ error: "Request body exceeds 8192 bytes." }));
          return;
        }
        chunks.push(chunk);
      }
    } catch {
      if (!res.destroyed) res.writeHead(400).end("Incomplete request body");
      return;
    }
    const body = Buffer.concat(chunks);
    const requestHeaders = {
      "content-type": req.headers["content-type"] || "application/json",
      accept: req.headers.accept || "application/json",
    };
    if (bodyLength || req.headers["content-length"] !== undefined)
      requestHeaders["content-length"] = String(bodyLength);
    // The destination is configured by the operator, never supplied by a page visitor.
    const target = new URL(backend);
    target.pathname = requestURL.pathname;
    target.search = requestURL.search;
    const upstream = (target.protocol === "https:" ? https : http).request(
      target,
      {
        method: req.method,
        headers: requestHeaders,
        timeout: 15000,
      },
      (response) => {
        res.writeHead(response.statusCode, {
          ...response.headers,
          "cache-control": "no-store",
        });
        response.pipe(res);
      },
    );
    upstream.on("timeout", () => upstream.destroy());
    upstream.on("error", () => {
      if (!res.headersSent)
        res.writeHead(503, { "content-type": "application/json" });
      res.end(
        JSON.stringify({
          available: false,
          error: "Live simulation is not connected.",
        }),
      );
    });
    req.on("aborted", () => upstream.destroy());
    upstream.end(body);
    return;
  }
  if (!["GET", "HEAD"].includes(req.method)) {
    res.writeHead(405).end();
    return;
  }
  try {
    let file = path.resolve(root, "." + pathname);
    if (!isWithinRoot(file)) throw new Error("Outside root");
    let info = await stat(file);
    if (info.isDirectory()) {
      file = path.join(file, "index.html");
      info = await stat(file);
    }
    const resolved = await realpath(file);
    if (!isWithinRoot(resolved)) throw new Error("Outside root");
    if (
      path
        .relative(root, resolved)
        .split(path.sep)
        .some((p) => p.startsWith(".")) ||
      /\.(?:py|toml|log|env)$/.test(resolved)
    )
      throw new Error("Private file");
    const headers = {
      "content-type": types[path.extname(file)] || "application/octet-stream",
      "accept-ranges": "bytes",
      "cache-control": /\.(?:html|js|mjs|css|json)$/.test(file)
        ? "no-cache"
        : "public, max-age=3600",
      "x-content-type-options": "nosniff",
    };
    const range = req.headers.range;
    if (range) {
      const match = /^bytes=(\d*)-(\d*)$/.exec(range);
      let start, end;
      if (match && (match[1] || match[2])) {
        start = match[1]
          ? Number(match[1])
          : Math.max(0, info.size - Number(match[2]));
        end = match[1]
          ? match[2]
            ? Math.min(Number(match[2]), info.size - 1)
            : info.size - 1
          : info.size - 1;
      }
      if (start === undefined || start > end || start >= info.size) {
        res.writeHead(416, { "content-range": `bytes */${info.size}` }).end();
        return;
      }
      res.writeHead(206, {
        ...headers,
        "content-length": end - start + 1,
        "content-range": `bytes ${start}-${end}/${info.size}`,
      });
      if (req.method === "HEAD") res.end();
      else createReadStream(file, { start, end }).pipe(res);
    } else {
      res.writeHead(200, { ...headers, "content-length": info.size });
      if (req.method === "HEAD") res.end();
      else createReadStream(file).pipe(res);
    }
  } catch {
    res.writeHead(404, { "content-type": "text/plain" }).end("Not found");
  }
});
server.listen(port, host, () =>
  console.log(
    `TRACE homepage: http://${host}:${server.address().port}\nSimulation proxy: ${backend.origin}`,
  ),
);
