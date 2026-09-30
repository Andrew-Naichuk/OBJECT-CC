// Node regressions for transfer abort (F5) and ride sort (G2).
// Run: node test/web_transfer/abort_and_sort.test.mjs

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";

const root = join(dirname(fileURLToPath(import.meta.url)), "../..");
const html = readFileSync(join(root, "tools/index.html"), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

assert.match(script, /transferGen/);
assert.match(script, /META_END_TIME/);
assert.match(script, /phase = "idle"/);
assert.match(script, /resetTransfer\(\);\s*\n\s*if \(message\)/s);

// Mirror rideCompare from tools/index.html.
function rideCompare(a, b) {
  if (a.name === "CURRENT.GPX") return -1;
  if (b.name === "CURRENT.GPX") return 1;
  const ta = a.endUtc || 0;
  const tb = b.endUtc || 0;
  if (ta !== 0 || tb !== 0) {
    if (ta === 0) return 1;
    if (tb === 0) return -1;
    if (ta !== tb) return tb - ta;
  }
  if (a.name < b.name) return 1;
  if (a.name > b.name) return -1;
  return 0;
}

function testSortByEndUtc() {
  const rides = [
    { name: "26093024.GPX", size: 1, endUtc: 1000 },
    { name: "26093011.GPX", size: 1, endUtc: 2000 },
    { name: "RIDE0001.GPX", size: 1, endUtc: 0 },
    { name: "CURRENT.GPX", size: 1, endUtc: 0 }
  ];
  const sorted = rides.slice().sort(rideCompare);
  assert.equal(sorted[0].name, "CURRENT.GPX");
  assert.equal(sorted[1].name, "26093011.GPX");
  assert.equal(sorted[2].name, "26093024.GPX");
  assert.equal(sorted[3].name, "RIDE0001.GPX");
  console.log("ok: G2 sort uses endUtc, not collision suffix as hour");
}

// Simulate the abortTransfer ordering fixed in tools/index.html.
async function testAbortClearsBeforeAwait() {
  let phase = "downloading";
  let transferGen = 1;
  let fileBuf = new Uint8Array(4);
  let received = 4;
  let doneCrc = null;
  let downloads = 0;
  let writeStarted = false;
  let writeFinished = false;

  function resetTransfer() {
    fileBuf = null;
    received = 0;
    doneCrc = null;
  }

  function maybeFinish(gen) {
    if (gen !== transferGen || phase !== "downloading") return;
    if (doneCrc === null || !fileBuf || received !== fileBuf.length) return;
    downloads++;
  }

  async function abortTransfer() {
    const wasBusy = phase === "listing" || phase === "downloading";
    const gen = ++transferGen;
    phase = "idle";
    resetTransfer();
    if (wasBusy) {
      writeStarted = true;
      await new Promise((r) => setTimeout(r, 40));
      writeFinished = true;
    }
    void gen;
  }

  const abortPromise = abortTransfer();
  assert.equal(phase, "idle");
  assert.equal(fileBuf, null);
  assert.equal(writeStarted, true);
  assert.equal(writeFinished, false);

  // Late completion attempt while abort write is unresolved.
  doneCrc = 1;
  maybeFinish(1); // stale generation
  assert.equal(downloads, 0);

  await abortPromise;
  assert.equal(writeFinished, true);
  assert.equal(downloads, 0);
  console.log("ok: F5 late data cannot complete cancelled file");
}

testSortByEndUtc();
await testAbortClearsBeforeAwait();

// Confirm page abort clears before awaiting writeCmd.
const abortIdx = script.indexOf("async function abortTransfer");
const abortBlock = script.slice(abortIdx, abortIdx + 700);
const idleBeforeAwait =
  abortBlock.indexOf('phase = "idle"') < abortBlock.indexOf("await Promise.race") &&
  abortBlock.indexOf("resetTransfer()") < abortBlock.indexOf("await Promise.race");
assert.equal(idleBeforeAwait, true);
console.log("ok: F5 page abort clears local state before BLE await");
console.log("all web transfer regressions passed");
