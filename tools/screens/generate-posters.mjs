#!/usr/bin/env node
// Capture light/dark WebGL posters for the HUB hero.
// Requires: playwright (npm i -D playwright) and a reachable CDN for three.js.
//
//   node tools/screens/generate-posters.mjs
//
import { createServer } from "node:http";
import { writeFile } from "node:fs/promises";
import { createReadStream, existsSync, statSync } from "node:fs";
import { extname, join, dirname } from "node:path";
import { fileURLToPath } from "node:url";
import { chromium } from "playwright";
import { spawn } from "node:child_process";

const repo = join(dirname(fileURLToPath(import.meta.url)), "../..");
const root = join(repo, "docs");
const MIME = {
  ".html": "text/html; charset=utf-8",
  ".js": "text/javascript; charset=utf-8",
  ".mjs": "text/javascript; charset=utf-8",
  ".css": "text/css; charset=utf-8",
  ".glb": "model/gltf-binary",
  ".webp": "image/webp",
  ".png": "image/png",
  ".svg": "image/svg+xml",
  ".woff2": "font/woff2",
  ".webmanifest": "application/manifest+json",
  ".json": "application/json"
};

function serve(port) {
  const server = createServer((req, res) => {
    let path = decodeURIComponent(new URL(req.url, "http://x").pathname);
    if (path === "/") path = "/index.html";
    const file = join(root, path);
    if (!file.startsWith(root) || !existsSync(file) || statSync(file).isDirectory()) {
      res.writeHead(404); res.end("missing " + path); return;
    }
    res.writeHead(200, {
      "Content-Type": MIME[extname(file)] || "application/octet-stream",
      "Cache-Control": "no-store"
    });
    createReadStream(file).pipe(res);
  });
  return new Promise((resolve) => server.listen(port, "127.0.0.1", () => resolve(server)));
}

async function capture(page, base, scheme, outName) {
  await page.emulateMedia({ colorScheme: scheme });
  await page.goto(base + "/", { waitUntil: "networkidle", timeout: 60000 });
  await page.waitForFunction(
    () => document.querySelector(".device-stage")?.classList.contains("is-ready"),
    null,
    { timeout: 30000 }
  );
  await page.addStyleTag({
    content: ".screen,.device-poster{opacity:0!important;pointer-events:none!important}"
  });
  await page.waitForTimeout(700);
  const png = await page.locator(".device-stage").screenshot({ type: "png" });
  const out = join(root, outName);
  // Prefer pillow via python for webp; fall back to canvas toBlob webp
  const tmp = join(root, ".poster-tmp-" + scheme + ".png");
  await writeFile(tmp, png);
  await new Promise((resolve, reject) => {
    const py = spawn("python3", ["-c",
      "from PIL import Image; import sys;\n" +
      "im=Image.open(sys.argv[1]).convert('RGB');\n" +
      "im.save(sys.argv[2],'WEBP',quality=92,method=6)\n",
      tmp, out
    ], { stdio: "inherit" });
    py.on("exit", (code) => code === 0 ? resolve() : reject(new Error("pillow exit " + code)));
  });
  await writeFile(tmp, ""); // truncate
  const { unlink } = await import("node:fs/promises");
  await unlink(tmp).catch(() => {});
  console.log("wrote", outName);
}

const port = 8765;
const server = await serve(port);
const browser = await chromium.launch();
const context = await browser.newContext({
  viewport: { width: 430, height: 900 },
  deviceScaleFactor: 2
});
const page = await context.newPage();
page.on("pageerror", (err) => console.error("pageerror:", err.message));

try {
  const base = `http://127.0.0.1:${port}`;
  await capture(page, base, "light", "object-poster-light.webp");
  await capture(page, base, "dark", "object-poster-dark.webp");
} finally {
  await browser.close();
  server.close();
}
