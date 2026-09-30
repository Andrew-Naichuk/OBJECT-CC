// Node regressions for transfer identity (R4), abort, sort, and CRC.
// Run: node test/web_transfer/abort_and_sort.test.mjs

import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { fileURLToPath } from "node:url";
import { dirname, join } from "node:path";
import vm from "node:vm";

const root = join(dirname(fileURLToPath(import.meta.url)), "../..");
const html = readFileSync(join(root, "tools/index.html"), "utf8");
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];

function el(id) {
  return {
    id,
    textContent: "",
    className: "",
    disabled: false,
    style: { setProperty() {} },
    value: null,
    dataset: {},
    children: [],
    append(...nodes) { this.children.push(...nodes); },
    replaceChildren(...nodes) { this.children = [...nodes]; },
    addEventListener() {},
    setAttribute() {},
    classList: { add() {}, remove() {} },
    click() {}
  };
}

const elements = {};
const document = {
  getElementById(id) {
    return elements[id] || (elements[id] = el(id));
  },
  createElement() {
    return el("node");
  },
  body: { dataset: {} }
};

const context = {
  document,
  navigator: {},
  URL: {
    createObjectURL() { return "blob:ride"; },
    revokeObjectURL() {}
  },
  Blob,
  setTimeout,
  clearTimeout,
  console,
  Uint8Array,
  DataView,
  ArrayBuffer
};
vm.createContext(context);
vm.runInContext(script + `
this.__api = {
  onMeta, onData, abortTransfer, rideCompare, crc32, renderRides,
  listRides, protocolIsV2, allocReqId, encodeCmd, setStatus,
  get phase() { return phase; },
  set phase(v) { phase = v; },
  get expectedId() { return expectedId; },
  set expectedId(v) { expectedId = v; },
  get abortId() { return abortId; },
  get connGen() { return connGen; },
  set connGen(v) { connGen = v; },
  get rides() { return rides; },
  get fileBuf() { return fileBuf; },
  get fileName() { return fileName; },
  set fileName(v) { fileName = v; },
  get nextReqId() { return nextReqId; },
  set nextReqId(v) { nextReqId = v; },
  set cmdChar(v) { cmdChar = v; },
  get status() { return statusEl.textContent; },
  get firmwareMsg() { return FIRMWARE_MSG; }
};
`, context);
const api = context.__api;

function packet(bytes) {
  const u8 = Uint8Array.from(bytes);
  return { target: { value: { buffer: u8.buffer, byteOffset: u8.byteOffset, byteLength: u8.length } } };
}

function u16(n) {
  return [n & 0xff, (n >> 8) & 0xff];
}
function u32(n) {
  return [n & 0xff, (n >> 8) & 0xff, (n >> 16) & 0xff, (n >> 24) & 0xff];
}
function name12(name) {
  const out = new Array(12).fill(0);
  for (let i = 0; i < name.length && i < 12; i++) out[i] = name.charCodeAt(i);
  return out;
}

function walkText(node, out) {
  if (node == null) return;
  if (typeof node === "string") {
    out.push(node);
    return;
  }
  if (node.textContent) out.push(node.textContent);
  for (const child of node.children || []) walkText(child, out);
}

const rides = [
  { name: "26093024.GPX", size: 1, endUtc: 1000 },
  { name: "26093011.GPX", size: 1, endUtc: 2000 },
  { name: "RIDE0001.GPX", size: 1, endUtc: 0 },
  { name: "CURRENT.GPX", size: 1, endUtc: 0 },
  { name: "C000002A.GPX", size: 1, endUtc: 0, current: true }
];
const sorted = rides.slice().sort(api.rideCompare);
assert.equal(sorted[0].name, "C000002A.GPX");
assert.equal(sorted[1].name, "CURRENT.GPX");
assert.equal(sorted[2].name, "26093011.GPX");
assert.equal(api.crc32(Uint8Array.from("123456789", (c) => c.charCodeAt(0))), 0xcbf43926);
console.log("ok: sort, CRC, and current-ride ordering");

api.phase = "listing";
api.expectedId = 7;
api.connGen = 1;
api.onMeta(packet([0x01, ...u16(7), ...name12("C000002A.GPX"), ...u32(20), 1]), 1);
api.onMeta(packet([0x01, ...u16(7), ...name12("RIDE0001.GPX"), ...u32(8), 0]), 1);
api.renderRides();
const texts = [];
walkText(elements.files, texts);
assert.ok(texts.includes("Current ride"));
assert.ok(texts.includes("C000002A"));
console.log("ok: continuation filename can carry the current-ride badge");

// Download B is id 2. Late packets from A (id 1) do not change it.
api.phase = "downloading";
api.expectedId = 2;
api.connGen = 4;
api.fileName = "B.GPX";
const before = api.fileBuf;
api.onMeta(packet([0x7f, ...u16(1), 4]), 4);
api.onMeta(packet([0x03, ...u16(1), ...name12("A.GPX"), ...u32(4), 0]), 4);
api.onMeta(packet([0x04, ...u16(1), ...u32(1)]), 4);
api.onMeta(packet([0x02, ...u16(1)]), 4);
api.onData(packet([...u16(1), ...u32(0), 9, 9, 9, 9]), 4);
assert.equal(api.phase, "downloading");
assert.equal(api.expectedId, 2);
assert.equal(api.fileBuf, before);
assert.equal(api.fileName, "B.GPX");
console.log("ok: late abort error, START, DONE, LIST_END, and data from A do not change B");

// Previous connection, same numeric id.
api.onMeta(packet([0x03, ...u16(2), ...name12("OLD.GPX"), ...u32(4), 0]), 3);
api.onData(packet([...u16(2), ...u32(0), 1, 2, 3, 4]), 3);
assert.equal(api.fileBuf, null);
console.log("ok: a notification from the previous connection does not apply");

// In-connection wrap refuses the new request and does not accept an old packet.
api.phase = "idle";
api.expectedId = 0;
api.nextReqId = 0;
api.connGen = 4;
api.cmdChar = { writeValue() { return Promise.resolve(); } };
await api.listRides();
assert.equal(api.phase, "idle");
assert.equal(api.expectedId, 0);
api.onMeta(packet([0x03, ...u16(1), ...name12("OLD.GPX"), ...u32(4), 0]), 4);
assert.equal(api.fileBuf, null);
console.log("ok: an in-connection id wrap does not accept an old packet");

// Error decided for an earlier id does not clear the newer request.
api.phase = "listing";
api.expectedId = 9;
api.connGen = 4;
api.onMeta(packet([0x7f, ...u16(8), 2]), 4);
assert.equal(api.phase, "listing");
assert.equal(api.expectedId, 9);
console.log("ok: an earlier error does not clear a newer request");

// Old-layout meta is not a v2 start. Missing version is the firmware message.
const oldStart = new Array(17).fill(0);
oldStart[0] = 0x03;
oldStart[1] = 9;
api.phase = "downloading";
api.expectedId = 9;
api.onMeta(packet(oldStart), 4);
assert.equal(api.fileBuf, null);
assert.equal(api.protocolIsV2(null), false);
assert.equal(api.protocolIsV2(new DataView(Uint8Array.from([1, 0]).buffer)), false);
assert.equal(api.protocolIsV2(new DataView(Uint8Array.from([2, 0]).buffer)), true);
assert.equal(api.firmwareMsg, "Update the firmware on the computer.");
const cmd = api.encodeCmd(0x02, 3, "C0000002.GPX");
assert.equal(cmd[0], 0xa5);
assert.equal(cmd[1], 2);
assert.notEqual(cmd[0], 0x02);
console.log("ok: old layout does not start a transfer; missing version asks for a firmware update");

// Abort clears local state before the write resolves.
api.phase = "downloading";
api.expectedId = 9;
api.connGen = 4;
api.fileName = "B.GPX";
api.onMeta(packet([0x03, ...u16(9), ...name12("B.GPX"), ...u32(4), 0]), 4);
assert.equal(api.fileBuf.length, 4);
let writeResolved = false;
api.cmdChar = {
  writeValue() {
    return new Promise((resolve) => {
      setTimeout(() => {
        writeResolved = true;
        resolve();
      }, 50);
    });
  }
};
const pending = api.abortTransfer("cancelled", true);
assert.equal(api.phase, "idle");
assert.equal(api.fileBuf, null);
assert.equal(api.expectedId, 0);
assert.equal(api.abortId, 9);
assert.equal(writeResolved, false);
api.onMeta(packet([0x04, ...u16(9), ...u32(1)]), 4);
api.onData(packet([...u16(9), ...u32(0), 1, 2, 3, 4]), 4);
assert.equal(api.fileBuf, null);
await pending;
assert.equal(writeResolved, true);
assert.equal(api.fileBuf, null);
console.log("ok: local clear happens before the abort write resolves");

const abortIdx = script.indexOf("async function abortTransfer");
const abortBlock = script.slice(abortIdx, abortIdx + 800);
assert.ok(abortBlock.indexOf('phase = "idle"') < abortBlock.indexOf("await Promise.race"));
assert.ok(abortBlock.indexOf("resetTransfer()") < abortBlock.indexOf("await Promise.race"));
assert.ok(abortBlock.indexOf("expectedId = 0") < abortBlock.indexOf("await Promise.race"));
console.log("ok: page abort clears local state before the BLE await");
console.log("all web transfer regressions passed");
