// Fake OBJECT computer for docs/index.html. Loaded by the page when the URL has ?mock.
//   ?mock          card with rides, connects on load
//   ?mock=empty    card with no rides
//   ?mock=nocard   no SD card in the computer
//   ?mock=manual   do not connect on load; press Connect
//   ?mock=deny     never allowed; drops the link after the 10 s window
//   ?mock=latest   already runs the published firmware, so no update is offered
//   ?mock=old      firmware from before the version command
(() => {
  const mode = new URLSearchParams(location.search).get("mock") || "";
  const CHUNK = 240;
  const CHUNK_MS = 8;

  const settings = {
    card: mode !== "nocard",
    wheel: 2105,
    tz: -new Date().getTimezoneOffset(),
    backlight: 0,
    units: 0,
    dim: 128,
    maxKmh: 80,
    stoppedMs: 3000,
    name: "OBJECT-001",
    animations: 1
  };

  // Points every 2 s, ending at the hour in a YYMMDDHH name (or now).
  function gpx(name, seed, size) {
    const head = '<?xml version="1.0" encoding="UTF-8"?>\n<gpx version="1.1" creator="OBJECT"><trk><trkseg>\n';
    const m = /^(\d\d)(\d\d)(\d\d)(\d\d)\./.exec(name);
    const end = m ? new Date(2000 + +m[1], m[2] - 1, +m[3], Math.min(+m[4], 23), 40).getTime() : Date.now();
    const points = Math.floor((size - head.length) / 125);
    let s = head;
    for (let i = 0; i < points; i++) {
      const lat = (50.45 + Math.sin((i + seed) / 50) * 0.01).toFixed(6);
      const lon = (30.52 + Math.cos((i + seed) / 50) * 0.01).toFixed(6);
      const ele = (170 + 25 * Math.sin((i + seed) / 90) + (i % 3) * 0.4).toFixed(1);
      const t = new Date(end - (points - i) * 2000).toISOString().replace(/\.\d+Z$/, "Z");
      s += '<trkpt lat="' + lat + '" lon="' + lon + '"><ele>' + ele + "</ele><time>" + t + "</time></trkpt>\n";
    }
    return new TextEncoder().encode(s + "</trkseg></trk></gpx>\n");
  }

  function crc32(bytes) {
    let c = 0xffffffff;
    for (let i = 0; i < bytes.length; i++) {
      c ^= bytes[i];
      for (let b = 0; b < 8; b++) c = (c >>> 1) ^ (0xedb88320 & -(c & 1));
    }
    return (~c) >>> 0;
  }

  // Stands in for docs/firmware/, which the page would fetch from GitHub
  // Pages. A short image keeps the fake transfer quick.
  const published = "2026.10.05-a6e3bc5";
  const fwVersion = mode === "latest" ? published : "2026.09.20-1f2a3b4";
  const fwImage = new Uint8Array(24 * 1024);
  for (let i = 0; i < fwImage.length; i++) fwImage[i] = (i * 31 + (i >> 8)) & 0xff;
  window.objectMockManifest = {
    version: published,
    notes: "distance milestone fix",
    bin: URL.createObjectURL(new Blob([fwImage])),
    dat: "firmware.dat",
    size: fwImage.length
  };

  const files = new Map();
  if (settings.card && mode !== "empty") {
    [["CURRENT.GPX", 18000], ["26100401.GPX", 142000], ["26100202.GPX", 96000], ["26100201.GPX", 51000],
      ["26092801.GPX", 7400], ["RIDE0003.GPX", 23000]]
      .forEach(([name, size], i) => files.set(name, gpx(name, i * 97, size)));
  }

  function putU32(b, at, v) {
    b[at] = v & 0xff;
    b[at + 1] = (v >>> 8) & 0xff;
    b[at + 2] = (v >>> 16) & 0xff;
    b[at + 3] = (v >>> 24) & 0xff;
  }

  function named(type, name, size) {
    const b = new Uint8Array(17);
    b[0] = type;
    for (let i = 0; i < name.length && i < 12; i++) b[1 + i] = name.charCodeAt(i);
    putU32(b, 13, size);
    return b;
  }

  function packCfg() {
    const b = new Uint8Array(35);
    b[0] = 0x08;
    b[1] = 2;
    b[2] = settings.card ? 1 : 0;
    b[3] = settings.wheel & 0xff;
    b[4] = settings.wheel >> 8;
    b[5] = settings.tz & 0xff;
    b[6] = (settings.tz >> 8) & 0xff;
    b[7] = settings.backlight;
    b[8] = settings.units;
    b[9] = settings.dim;
    b[10] = settings.maxKmh;
    b[11] = settings.stoppedMs & 0xff;
    b[12] = settings.stoppedMs >> 8;
    b[13] = settings.name.length;
    for (let i = 0; i < settings.name.length; i++) b[14 + i] = settings.name.charCodeAt(i);
    b[34] = settings.animations;
    return b;
  }

  function unpackCfg(b) {
    settings.wheel = b[2] | (b[3] << 8);
    settings.tz = ((b[4] | (b[5] << 8)) << 16) >> 16;
    settings.backlight = b[6];
    settings.units = b[7];
    settings.dim = b[8];
    settings.maxKmh = b[9];
    settings.stoppedMs = b[10] | (b[11] << 8);
    settings.name = String.fromCharCode(...b.subarray(13, 13 + Math.min(b[12], 20)));
    settings.animations = b[33];
  }

  // One queue for every write, so writes arrive in the order they were made
  // even when two characteristics answer at different speeds.
  let writeQueue = Promise.resolve();

  class Char extends EventTarget {
    constructor(onWrite, delay) {
      super();
      this.value = null;
      this.onWrite = onWrite;
      this.delay = delay === undefined ? 20 : delay;
    }
    get gatt() { return device.gatt; }
    async startNotifications() { return this; }
    async writeValue(buf) {
      if (!this.gatt.connected) throw new Error("GATT Server is disconnected.");
      const b = new Uint8Array(buf instanceof Uint8Array ? buf : new Uint8Array(buf.buffer || buf));
      writeQueue = writeQueue.then(() => new Promise((done) => {
        setTimeout(() => {
          this.onWrite(b);
          done();
        }, this.delay);
      }));
      await writeQueue;
    }
    async writeValueWithoutResponse(buf) { return this.writeValue(buf); }
    async writeValueWithResponse(buf) { return this.writeValue(buf); }
    emit(bytes) {
      if (!this.gatt.connected) return;
      this.value = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength);
      this.dispatchEvent(new Event("characteristicvaluechanged"));
    }
  }

  let timers = [];
  const later = (fn, ms) => timers.push(setTimeout(fn, ms));
  const stopAll = () => { timers.forEach(clearTimeout); timers = []; };

  const meta = new Char(() => {});
  const data = new Char(() => {});
  const err = (code) => meta.emit(new Uint8Array([0x7f, code]));

  let sending = false;
  let ota = { active: false, size: 0, crc: 0, got: 0, buf: null };

  function send(name) {
    const body = files.get(name);
    if (!settings.card) return err(1);
    if (!body) return err(3);
    meta.emit(named(0x03, name, body.length));
    sending = true;
    let off = 0;
    const step = () => {
      if (off >= body.length) {
        sending = false;
        const done = new Uint8Array(5);
        done[0] = 0x04;
        putU32(done, 1, crc32(body));
        meta.emit(done);
        return;
      }
      const n = Math.min(CHUNK, body.length - off);
      const pkt = new Uint8Array(4 + n);
      putU32(pkt, 0, off);
      pkt.set(body.subarray(off, off + n), 4);
      data.emit(pkt);
      off += n;
      later(step, CHUNK_MS);
    };
    later(step, CHUNK_MS);
  }

  const cmd = new Char((b) => {
    const op = b[0];
    const name = String.fromCharCode(...b.subarray(1, 13)).replace(/\0+$/, "");
    if (op === 0x01) {
      if (!settings.card) return err(1);
      let delay = 0;
      for (const [n, body] of files) later(() => meta.emit(named(0x01, n, body.length)), (delay += 40));
      const end = new Uint8Array(5);
      end[0] = 0x02;
      putU32(end, 1, 31154688);
      later(() => meta.emit(end), delay + 40);
    } else if (op === 0x02) {
      send(name);
    } else if (op === 0x03) {
      stopAll();
      if (sending) {
        sending = false;
        err(5);
      }
      if (ota.active) ota = { active: false, size: 0, crc: 0, got: 0, buf: null };
    } else if (op === 0x04) {
      if (!settings.card) return err(1);
      if (!files.delete(name)) return err(3);
      later(() => meta.emit(new Uint8Array([0x07])), 300);
    } else if (op === 0x05) {
      later(() => meta.emit(packCfg()), 150);
    } else if (op === 0x06) {
      if (!settings.card) return err(1);
      unpackCfg(b.subarray(1));
      later(() => {
        meta.emit(new Uint8Array([0x09]));
        meta.emit(packCfg());
      }, 400);
    } else if (op === 0x07) {
      if (mode === "old") return err(2);
      const info = new Uint8Array(1 + fwVersion.length);
      info[0] = 0x0a;
      for (let i = 0; i < fwVersion.length; i++) info[1 + i] = fwVersion.charCodeAt(i);
      later(() => meta.emit(info), 120);
    } else if (op === 0x08) {
      if (b.length < 9) return err(8);
      const size = b[1] | (b[2] << 8) | (b[3] << 16) | (b[4] << 24);
      const crc = b[5] | (b[6] << 8) | (b[7] << 16) | (b[8] << 24);
      if (size < 256 || size > 0x50000) return err(8);
      ota = { active: true, size, crc, got: 0, buf: new Uint8Array(size) };
      later(() => meta.emit(new Uint8Array([0x0b])), 80);
    } else if (op === 0x09) {
      if (!ota.active) return err(8);
      const chunk = b.subarray(1);
      if (ota.got + chunk.length > ota.size) return err(8);
      ota.buf.set(chunk, ota.got);
      ota.got += chunk.length;
      const note = new Uint8Array(5);
      note[0] = 0x0d;
      putU32(note, 1, ota.got);
      meta.emit(note);
    } else if (op === 0x0a) {
      if (!ota.active || ota.got !== ota.size || crc32(ota.buf) !== ota.crc) return err(8);
      console.info("[mock-ble] fake firmware install ok (" + ota.size + " bytes)");
      later(() => {
        meta.emit(new Uint8Array([0x0c]));
        later(() => device.gatt.disconnect(), 200);
      }, 150);
    }
  }, 4);

  const service = {
    async getCharacteristic(uuid) {
      return { "7a1e0002": cmd, "7a1e0003": meta, "7a1e0004": data }[uuid.slice(0, 8)];
    }
  };

  const device = new EventTarget();
  device.name = settings.name;
  device.gatt = {
    connected: false,
    async connect() {
      await new Promise((r) => setTimeout(r, 400));
      this.connected = true;
      later(() => meta.emit(new Uint8Array([0x05])), 300);
      if (mode === "deny") later(() => device.gatt.disconnect(), 10000);
      else later(() => meta.emit(new Uint8Array([0x06])), 1300);
      return { getPrimaryService: async () => service };
    },
    disconnect() {
      if (!this.connected) return;
      this.connected = false;
      sending = false;
      ota = { active: false, size: 0, crc: 0, got: 0, buf: null };
      stopAll();
      setTimeout(() => device.dispatchEvent(new Event("gattserverdisconnected")), 0);
    }
  };

  Object.defineProperty(navigator, "bluetooth", {
    configurable: true,
    value: {
      requestDevice: async () => device
    }
  });

  console.info("[mock-ble] fake OBJECT computer active (" + (mode || "default") + ")");
  if (mode !== "manual") document.getElementById("connect").click();
})();
