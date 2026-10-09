// Studio 3D hero for OBJECT HUB. Progressive: posters first, WebGL after.
// OLED UI is painted onto the display mesh (canvas texture) so it stays
// locked to the bezel; the HTML .screen stays in the DOM for a11y / fallback.

import * as THREE from "three";
import { GLTFLoader } from "three/addons/loaders/GLTFLoader.js";
import { MeshoptDecoder } from "three/addons/libs/meshopt_decoder.module.js";
import { RoomEnvironment } from "three/addons/environments/RoomEnvironment.js";

const MODEL_URL = "./object-cc.glb";
const MAX_DPR = 1.75;
const TILT_MAX = 0.14; // rad ≈ 8°
const BREATHE_AMP = 0.07; // rad ≈ 4°
const OLED_W = 240;
const OLED_H = 378; // portrait ≈ 486∶764
const REDUCED = matchMedia("(prefers-reduced-motion: reduce)").matches;
// Demo ride numbers match docs/screens/units-metric.png (imperial converts like firmware).
const DEMO = {
  speedKmh: 27.4,
  avgKmh: 25.9,
  distKm: 2.25,
  maxKmh: 30.1,
  altM: 250,
  elapsedMs: (5 * 60 + 21) * 1000,
  movingMs: (5 * 60 + 12) * 1000,
  battPct: 82,
  sats: 13
};
const MPH = 0.621371;
const FT = 3.28084;
const FW_H = 320; // firmware framebuffer; texture is taller — scale Y to fill
const CFG_ATTRS = [
  "data-view", "data-cfg", "data-cfg-units", "data-cfg-backlight", "data-cfg-dim",
  "data-cfg-animations", "data-cfg-name", "data-cfg-tz", "data-cfg-card"
];

// 5×7 matrix glyphs from xiao_oled (bit cols-1 = left).
const DOT_FONT = {
  " ": [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
  "0": [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
  "1": [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
  "2": [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
  "3": [0x0E, 0x11, 0x01, 0x06, 0x01, 0x11, 0x0E],
  "4": [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
  "5": [0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E],
  "6": [0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E],
  "7": [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
  "8": [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
  "9": [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C],
  ".": [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01]
};
const DOT_COLS = { " ": 5, ".": 1 };
const COL = {
  fg: "#ffffff",
  dim: "#8f8f8f",
  rule: "#242424",
  off: "#1c1c1c",
  lo: "#3c3c3c",
  bg: "#000000"
};
// Satellite icon 17×9 from firmware SAT_ICON (MSB left).
const SAT_ICON = [
  0x00, 0x80, 0x00,
  0x00, 0x80, 0x00,
  0xF8, 0x8F, 0x80,
  0xA9, 0xCA, 0x80,
  0xA9, 0xCA, 0x80,
  0xFF, 0xFF, 0x80,
  0xA9, 0xCA, 0x80,
  0xA9, 0xCA, 0x80,
  0xF8, 0x0F, 0x80
];

// Distances are multipliers of the auto-fit distance (full unit visible with margin).
const POSE = {
  off:        { yaw: 0.22, pitch: 0.09, dist: 1.00, frontal: 0 },
  pairing:    { yaw: 0.17, pitch: 0.08, dist: 0.97, frontal: 0.25 },
  awaiting:   { yaw: 0.08, pitch: 0.06, dist: 0.94, frontal: 0.7 },
  connected:  { yaw: 0.12, pitch: 0.07, dist: 0.96, frontal: 0.55 },
  busy:       { yaw: 0.06, pitch: 0.05, dist: 0.93, frontal: 0.85 }
};

/**
 * @param {HTMLElement} root  section.device
 */
export function initDeviceHero(root) {
  const stage = root.querySelector(".device-stage");
  const canvas = root.querySelector(".device-canvas");
  const screen = root.querySelector(".screen");
  const posters = root.querySelectorAll(".device-poster");
  if (!stage || !canvas || !screen) return { destroy() {} };

  let renderer, scene, camera, model, displayMesh, buttonGroup;
  let keyLight, rimLight, fillLight;
  let oledCanvas, oledCtx, oledTex, displayMat;
  let raf = 0;
  let running = false;
  let dirty = true;
  let loaded = false;
  let destroyed = false;
  let pointerActive = false;
  let pointerId = null;
  let tiltX = 0, tiltY = 0;       // current
  let targetTiltX = 0, targetTiltY = 0;
  let breathe = 0;
  let buttonPulse = 0;
  let fitDist = 0.24; // set after model load from bounding box + FOV
  let pose = { ...POSE.off, dist: fitDist };
  let poseTarget = { ...POSE.off, dist: fitDist };
  let startX = 0, startY = 0;
  let baseTiltX = 0, baseTiltY = 0;
  const clock = { t: performance.now() };

  function needFrame() {
    dirty = true;
    if (!running && loaded && !destroyed) startLoop();
  }

  function currentPoseKey() {
    const link = document.body.dataset.link;
    const phase = document.body.dataset.phase;
    if (phase === "pairing") return "pairing";
    if (phase === "awaiting") return "awaiting";
    if (phase === "downloading" || phase === "updating" || phase === "listing" ||
        phase === "deleting" || phase === "reading" || phase === "saving") return "busy";
    if (link === "on") return "connected";
    return "off";
  }

  function poseFor(key) {
    const p = POSE[key] || POSE.off;
    return { yaw: p.yaw, pitch: p.pitch, frontal: p.frontal, dist: fitDist * p.dist };
  }

  function syncPoseFromBody() {
    poseTarget = poseFor(currentPoseKey());
    paintOled();
    needFrame();
  }

  function settingsPreviewActive() {
    const d = document.body.dataset;
    return d.view === "settings" && d.link === "on" && d.phase === "idle" && d.cfg === "1";
  }

  function settingsAnimActive() {
    // Firmware pauses animations while the backlight is off.
    return settingsPreviewActive()
      && document.body.dataset.cfgAnimations === "1"
      && Number(document.body.dataset.cfgBacklight) !== 2
      && !REDUCED;
  }

  function cfgBacklightAlpha() {
    const d = document.body.dataset;
    const bl = Number(d.cfgBacklight);
    if (bl === 2) return 0.08;
    if (bl === 1) {
      const t = Math.max(1, Math.min(254, Number(d.cfgDim) || 1)) / 254;
      return 0.25 + t * 0.6;
    }
    return 1;
  }

  /** Dim the painted OLED toward black (PWM backlight), not material alpha. */
  function applyBacklightVeil(ctx) {
    const a = cfgBacklightAlpha();
    if (a >= 0.999) return;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.fillStyle = "rgba(0,0,0," + (1 - a) + ")";
    ctx.fillRect(0, 0, OLED_W, OLED_H);
  }

  function setDisplayOpacity(opacity, transparent) {
    if (!displayMat) return;
    displayMat.opacity = opacity;
    if (displayMat.transparent !== transparent) {
      displayMat.transparent = transparent;
      // Chrome keeps the opaque program unless the material is recompiled.
      displayMat.needsUpdate = true;
    }
  }

  function cfgImperial() {
    return document.body.dataset.cfgUnits === "1";
  }

  function cfgSpeedShown(kmh) {
    return cfgImperial() ? kmh * MPH : kmh;
  }

  function cfgDistShown(km) {
    return cfgImperial() ? km * MPH : km;
  }

  function cfgAltShown(m) {
    return cfgImperial() ? m * FT : m;
  }

  function cfgSpeedUnit() {
    return cfgImperial() ? "mph" : "km/h";
  }

  function cfgDistUnit() {
    return cfgImperial() ? "mi" : "km";
  }

  function cfgAltUnit() {
    return cfgImperial() ? "ft" : "m";
  }

  function cfgClock() {
    const tz = Number(document.body.dataset.cfgTz) || 0;
    const d = new Date(Date.now() + tz * 60000);
    const pad = (n) => String(n).padStart(2, "0");
    return pad(d.getUTCHours()) + ":" + pad(d.getUTCMinutes());
  }

  function formatHms(ms) {
    const total = Math.floor(ms / 1000);
    const h = Math.floor(total / 3600);
    const m = Math.floor(total / 60) % 60;
    const s = total % 60;
    return h + ":" + String(m).padStart(2, "0") + ":" + String(s).padStart(2, "0");
  }

  function cfgDemoSpeed() {
    return { value: cfgSpeedShown(DEMO.speedKmh).toFixed(1), unit: cfgSpeedUnit() };
  }

  function computeFitDistance() {
    if (!model || !camera) return;
    // Fit the front body only — the bike mount sits behind and would push
    // the camera too far for a readable OLED.
    const mount = model.getObjectByName("bike_mount");
    if (mount) mount.visible = false;
    const box = new THREE.Box3().setFromObject(model);
    if (mount) mount.visible = true;
    const size = box.getSize(new THREE.Vector3());
    const halfH = size.y * 0.5;
    const vFov = THREE.MathUtils.degToRad(camera.fov);
    // Margin covers yaw/pitch/breathe so bezels never meet the crop.
    // 1.46 ≈ 10% tighter than the previous 1.62 fit.
    const margin = 1.2;
    fitDist = (halfH * margin) / Math.tan(vFov * 0.5);
    pose.dist = fitDist * (POSE[currentPoseKey()]?.dist ?? 1);
    poseTarget = poseFor(currentPoseKey());
  }

  function isDark() {
    const attr = document.documentElement.getAttribute("data-theme");
    if (attr === "dark" || attr === "light") return attr === "dark";
    return matchMedia("(prefers-color-scheme: dark)").matches;
  }

  function applyTheme() {
    if (!scene || !renderer) return;
    const dark = isDark();
    // Match page --paper so the stage reads as continuous with the hub, not a grey plate.
    scene.background = new THREE.Color(dark ? 0x000000 : 0xffffff);
    if (keyLight) keyLight.intensity = dark ? 0.7 : 0.85;
    if (rimLight) {
      rimLight.intensity = dark ? 0.85 : 0.3;
      rimLight.position.set(dark ? -0.55 : -0.5, dark ? 0.35 : 0.2, dark ? -0.35 : -0.4);
    }
    if (fillLight) fillLight.intensity = dark ? 0.4 : 0.12;
    renderer.toneMappingExposure = dark ? 1.05 : 1.05;
    needFrame();
  }

  async function boot() {
    try {
      if (!canvas.getContext("webgl2") && !canvas.getContext("webgl")) {
        stage.classList.add("is-fallback");
        return;
      }

      renderer = new THREE.WebGLRenderer({
        canvas,
        antialias: true,
        alpha: false,
        powerPreference: "low-power"
      });
      renderer.setPixelRatio(Math.min(devicePixelRatio || 1, MAX_DPR));
      renderer.outputColorSpace = THREE.SRGBColorSpace;
      renderer.toneMapping = THREE.NeutralToneMapping;
      renderer.toneMappingExposure = 1.05;

      scene = new THREE.Scene();
      camera = new THREE.PerspectiveCamera(30, 1, 0.01, 4);
      camera.position.set(0, 0.01, 0.28);

      const pmrem = new THREE.PMREMGenerator(renderer);
      scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
      pmrem.dispose();

      keyLight = new THREE.DirectionalLight(0xffffff, 0.85);
      keyLight.position.set(0.4, 0.7, 0.55);
      scene.add(keyLight);
      rimLight = new THREE.DirectionalLight(0xc8d4ff, 0.3);
      rimLight.position.set(-0.5, 0.2, -0.4);
      scene.add(rimLight);
      fillLight = new THREE.DirectionalLight(0xffffff, 0.12);
      fillLight.position.set(0.1, -0.6, 0.4);
      scene.add(fillLight);
      scene.add(new THREE.AmbientLight(0xffffff, 0.18));

      applyTheme();
      resize();

      const loader = new GLTFLoader();
      loader.setMeshoptDecoder(MeshoptDecoder);
      const gltf = await loader.loadAsync(MODEL_URL);
      if (destroyed) return;

      model = gltf.scene;
      // Parent stray side screws under the housing if they are scene roots
      const rootObj = model.getObjectByName("OBJECT_cycling_computer") || model;
      const housing = model.getObjectByName("housing") || rootObj;
      for (const name of ["side_screw_head", "side_screw_slot_a", "side_screw_slot_b"]) {
        const screw = model.getObjectByName(name);
        if (screw && screw.parent !== housing && housing) {
          housing.attach(screw);
        }
      }

      // Tone down plastic env response; keep steel/brass lively
      model.traverse((obj) => {
        if (!obj.isMesh || !obj.material) return;
        const mats = Array.isArray(obj.material) ? obj.material : [obj.material];
        for (const m of mats) {
          if (!m) continue;
          const n = (m.name || "").toLowerCase();
          if (n.includes("matte") || n.includes("gloss") || n.includes("cavity") || n.includes("display") || n.includes("glass")) {
            m.envMapIntensity = 0.35;
          } else if (n.includes("steel") || n.includes("brass")) {
            m.envMapIntensity = 0.9;
          }
          m.needsUpdate = true;
        }
      });

      displayMesh = model.getObjectByName("display");
      const glass = model.getObjectByName("screen_glass");
      if (displayMesh) {
        // Scale the display quad up slightly so the OLED fills the bezel
        // opening (glass is larger than the inner display mesh).
        displayMesh.scale.multiplyScalar(1.12);
        setupDisplayTexture(displayMesh);
      }
      if (glass) glass.visible = false;

      buttonGroup = model.getObjectByName("button");

      // Centre on the front body (exclude bike mount — it sits behind and
      // would pull the silhouette downward in frame).
      const mount = model.getObjectByName("bike_mount");
      if (mount) mount.visible = false;
      const box = new THREE.Box3().setFromObject(model);
      if (mount) mount.visible = true;
      const center = box.getCenter(new THREE.Vector3());
      model.position.sub(center);

      scene.add(model);
      resize();
      computeFitDistance();
      loaded = true;
      canvas.hidden = false;
      stage.classList.add("is-ready");
      screen.classList.add("is-mesh-screen");
      for (const img of posters) img.setAttribute("aria-hidden", "true");
      await document.fonts.ready.catch(() => {});
      paintOled();
      syncPoseFromBody();
      placeCamera();
      needFrame();
    } catch (err) {
      console.warn("OBJECT hero: WebGL unavailable", err);
      stage.classList.add("is-fallback");
    }
  }

  function setupDisplayTexture(mesh) {
    const geo = mesh.geometry;
    const pos = geo.attributes.position;
    let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
    for (let i = 0; i < pos.count; i++) {
      const x = pos.getX(i), y = pos.getY(i);
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
    }
    const uvs = new Float32Array(pos.count * 2);
    const spanX = maxX - minX || 1;
    const spanY = maxY - minY || 1;
    for (let i = 0; i < pos.count; i++) {
      uvs[i * 2] = (pos.getX(i) - minX) / spanX;
      uvs[i * 2 + 1] = (pos.getY(i) - minY) / spanY;
    }
    geo.setAttribute("uv", new THREE.BufferAttribute(uvs, 2));

    oledCanvas = document.createElement("canvas");
    oledCanvas.width = OLED_W;
    oledCanvas.height = OLED_H;
    oledCtx = oledCanvas.getContext("2d");
    oledTex = new THREE.CanvasTexture(oledCanvas);
    oledTex.colorSpace = THREE.SRGBColorSpace;
    oledTex.anisotropy = 4;
    oledTex.generateMipmaps = true;
    oledTex.minFilter = THREE.LinearMipmapLinearFilter;
    oledTex.magFilter = THREE.LinearFilter;

    // transparent starts false; pairing toggles it via setDisplayOpacity (needsUpdate).
    displayMat = new THREE.MeshBasicMaterial({
      map: oledTex,
      toneMapped: false,
      transparent: false,
      opacity: 1
    });
    mesh.material = displayMat;
  }

  function paintOled() {
    if (!oledCtx || !oledTex) return;
    if (settingsPreviewActive()) paintSettingsPreview();
    else paintHubOled();
  }

  function drawOledChrome(ctx, W, H, alpha, caption, right, value, unit, figureLabel, p) {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, W, H);

    const padX = W * 0.08;
    const padTop = H * 0.07;

    ctx.globalAlpha = alpha;
    ctx.fillStyle = "rgba(255,255,255,0.6)";
    ctx.textBaseline = "top";
    ctx.textAlign = "center";
    const capSize = Math.round(W * 0.058);
    const lineH = capSize * 1.25;
    ctx.font = `600 ${capSize}px Archivo, system-ui, sans-serif`;

    const capMaxW = W - padX * 2;
    const capLines = wrapLines(ctx, caption, capMaxW, 2);
    for (let i = 0; i < capLines.length; i++) {
      ctx.fillText(capLines[i], W / 2, padTop + i * lineH);
    }
    if (right) {
      ctx.fillText(right, W / 2, padTop + Math.max(1, capLines.length) * lineH);
    }

    ctx.fillStyle = "#fff";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    const valSize = Math.round(W * 0.38);
    const labelSize = Math.round(W * 0.055);
    const midY = figureLabel ? H * 0.52 : H * 0.56;
    ctx.font = `900 ${valSize}px Doto, Archivo, system-ui, sans-serif`;
    const valW = ctx.measureText(value).width;
    if (unit) {
      const unitSize = Math.round(W * 0.18);
      ctx.font = `900 ${unitSize}px Doto, Archivo, system-ui, sans-serif`;
      const unitW = ctx.measureText(unit).width;
      const gap = W * 0.03;
      const total = valW + gap + unitW;
      const baseX = (W - total) / 2;
      ctx.font = `900 ${valSize}px Doto, Archivo, system-ui, sans-serif`;
      ctx.textAlign = "left";
      ctx.fillText(value, baseX, midY);
      ctx.font = `900 ${unitSize}px Doto, Archivo, system-ui, sans-serif`;
      ctx.textBaseline = "alphabetic";
      ctx.fillText(unit, baseX + valW + gap, midY - valSize * 0.18);
      ctx.textBaseline = "middle";
      ctx.textAlign = "center";
    } else {
      ctx.fillText(value, W / 2, midY);
    }
    if (figureLabel) {
      ctx.fillStyle = "#fff";
      ctx.font = `600 ${labelSize}px Archivo, system-ui, sans-serif`;
      ctx.letterSpacing = "0.12em";
      ctx.fillText(figureLabel, W / 2, midY + valSize * 0.52);
      ctx.letterSpacing = "0";
    }

    const dots = 24;
    const gaugeY = H * 0.935;
    const gaugeLeft = padX;
    const gaugeRight = W - padX;
    const step = (gaugeRight - gaugeLeft) / (dots - 1);
    const r = Math.max(1.6, W * 0.012);
    const filled = Math.round(Math.max(0, Math.min(1, p)) * dots);
    for (let i = 0; i < dots; i++) {
      const x = gaugeLeft + i * step;
      ctx.beginPath();
      ctx.arc(x, gaugeY, r, 0, Math.PI * 2);
      ctx.fillStyle = i < filled ? "#fff" : "#242424";
      ctx.fill();
    }

    oledTex.needsUpdate = true;
    needFrame();
  }

  function paintSettingsPreview() {
    if (!oledCtx || !oledTex) return;
    const ctx = oledCtx;
    const d = document.body.dataset;
    const noCard = d.cfgCard === "0";
    const sy = OLED_H / FW_H;

    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.fillStyle = COL.bg;
    ctx.fillRect(0, 0, OLED_W, OLED_H);
    // Firmware lays out in 240×320; stretch Y so the ride UI fills the taller texture.
    ctx.setTransform(1, 0, 0, sy, 0, 0);

    const PAD = 12;
    const CAPTION_Y = 34;
    const HERO_Y = 60;
    const HERO_PITCH = 10;
    const HERO_DOT = 8;
    const MAT_COLS = 19;
    const MAT_ROWS = 7;
    const GAUGE_Y = 142;
    const GAUGE_DOTS = 24;
    const GAUGE_PITCH = 9;
    const GAUGE_R = 2;
    const ROWS_Y = 156;
    const ROW_H = 34;
    const ROW_COUNT = 4;
    const FOOTER_Y = 296;
    const fontSm = "600 13px Archivo, system-ui, sans-serif";
    const fontVal = "700 16px Archivo, system-ui, sans-serif";

    // —— Status bar: battery | clock | sats ——
    const battPct = DEMO.battPct;
    const bars = Math.min(5, Math.floor((battPct + 10) / 20));
    const bx = PAD, by = 8;
    ctx.strokeStyle = COL.fg;
    ctx.lineWidth = 1;
    ctx.strokeRect(bx + 0.5, by + 0.5, 21, 11);
    ctx.fillStyle = COL.fg;
    ctx.fillRect(bx + 22, by + 3, 2, 6);
    for (let i = 0; i < 5; i++) {
      ctx.fillStyle = i < bars ? COL.fg : COL.off;
      ctx.fillRect(bx + 2 + i * 4, by + 2, 3, 8);
    }
    ctx.fillStyle = COL.fg;
    ctx.font = fontSm;
    ctx.textBaseline = "alphabetic";
    ctx.textAlign = "left";
    ctx.fillText(battPct + "%", PAD + 30, 19);

    const clock = cfgClock();
    ctx.textAlign = "center";
    ctx.fillText(clock, OLED_W / 2, 19);

    const satLabel = String(DEMO.sats);
    ctx.textAlign = "right";
    ctx.fillText(satLabel, OLED_W - PAD, 19);
    const satTw = ctx.measureText(satLabel).width;
    drawSatIcon(ctx, OLED_W - PAD - satTw - 6 - 17, 19 - 6 - 4);

    // —— Caption: unit | Avg ——
    const speedU = cfgSpeedUnit();
    const avg = cfgSpeedShown(DEMO.avgKmh).toFixed(1);
    ctx.fillStyle = COL.dim;
    ctx.font = fontSm;
    ctx.textAlign = "left";
    ctx.fillText(speedU, PAD, CAPTION_Y + 15);
    ctx.textAlign = "right";
    ctx.fillText("Avg " + avg, OLED_W - PAD, CAPTION_Y + 15);

    // —— Dot-matrix speed ——
    let shown = cfgSpeedShown(DEMO.speedKmh);
    if (shown < 0) shown = 0;
    if (shown > 99.9) shown = 99.9;
    const digits = shown.toFixed(1).padStart(4, " ");
    const mat = buildMatrix(digits);
    if (settingsAnimActive()) {
      // Soft heartbeat on the decimal gap column (firmware fxHeartbeat).
      const beat = (performance.now() % 2000) < 180;
      if (beat) {
        for (let r = 0; r < 6; r++) {
          if (mat[r][12] === "off") mat[r][12] = "lo";
        }
      }
    }
    const matW = MAT_COLS * HERO_PITCH - (HERO_PITCH - HERO_DOT);
    const x0 = (OLED_W - matW) / 2;
    for (let r = 0; r < MAT_ROWS; r++) {
      for (let c = 0; c < MAT_COLS; c++) {
        const cell = mat[r][c];
        if (cell === "bg") continue;
        ctx.fillStyle = cell === "fg" ? COL.fg : cell === "lo" ? COL.lo : COL.off;
        ctx.fillRect(x0 + c * HERO_PITCH, HERO_Y + r * HERO_PITCH, HERO_DOT, HERO_DOT);
      }
    }

    // —— 24-dot gauge (always km/h scale, 2 km/h per dot) ——
    let lit = Math.round(DEMO.speedKmh / 2);
    if (lit > GAUGE_DOTS) lit = GAUGE_DOTS;
    const gx0 = (OLED_W - (GAUGE_DOTS - 1) * GAUGE_PITCH) / 2;
    for (let i = 0; i < GAUGE_DOTS; i++) {
      ctx.beginPath();
      ctx.arc(gx0 + i * GAUGE_PITCH, GAUGE_Y, GAUGE_R, 0, Math.PI * 2);
      ctx.fillStyle = i < lit ? COL.fg : COL.off;
      ctx.fill();
    }

    // —— Stat rows ——
    const dist = cfgDistShown(DEMO.distKm);
    const distStr = dist < 100 ? dist.toFixed(2) : dist.toFixed(1);
    const rows = [
      { label: "Distance " + cfgDistUnit(), value: distStr },
      { label: "Time", value: formatHms(DEMO.elapsedMs) },
      { label: "Moving", value: formatHms(DEMO.movingMs) },
      { label: "Max " + speedU, value: cfgSpeedShown(DEMO.maxKmh).toFixed(1) }
    ];
    for (let i = 0; i <= ROW_COUNT; i++) {
      ctx.fillStyle = COL.rule;
      ctx.fillRect(PAD, ROWS_Y + i * ROW_H, OLED_W - 2 * PAD, 1);
    }
    for (let i = 0; i < ROW_COUNT; i++) {
      const base = ROWS_Y + i * ROW_H + 25;
      ctx.fillStyle = COL.dim;
      ctx.font = fontSm;
      ctx.textAlign = "left";
      ctx.fillText(rows[i].label, PAD, base);
      ctx.fillStyle = COL.fg;
      ctx.font = fontVal;
      ctx.textAlign = "right";
      ctx.fillText(rows[i].value, OLED_W - PAD, base);
    }

    // —— Footer: Alt | Phone / No card ——
    const altBase = FOOTER_Y + 15;
    ctx.fillStyle = COL.dim;
    ctx.font = fontSm;
    ctx.textAlign = "left";
    ctx.fillText("Alt " + Math.round(cfgAltShown(DEMO.altM)) + " " + cfgAltUnit(), PAD, altBase);

    ctx.textAlign = "right";
    if (noCard) {
      const label = "No card";
      ctx.font = "700 13px Archivo, system-ui, sans-serif";
      const tw = ctx.measureText(label).width;
      const cx = OLED_W - PAD - tw - 14;
      ctx.fillStyle = COL.fg;
      ctx.beginPath();
      ctx.arc(cx, altBase - 5, 7, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillStyle = COL.bg;
      ctx.font = "700 11px Archivo, system-ui, sans-serif";
      ctx.textAlign = "center";
      ctx.fillText("!", cx, altBase - 1);
      ctx.fillStyle = COL.fg;
      ctx.font = "700 13px Archivo, system-ui, sans-serif";
      ctx.textAlign = "right";
      ctx.fillText(label, OLED_W - PAD, altBase);
    } else {
      // Connected hub session → Phone mark (matches docs/screens/footer-phone.png)
      const label = "Phone";
      ctx.font = fontSm;
      const tw = ctx.measureText(label).width;
      const cx = OLED_W - PAD - tw - 12;
      ctx.fillStyle = COL.fg;
      ctx.beginPath();
      ctx.arc(cx, altBase - 5, 4, 0, Math.PI * 2);
      ctx.fill();
      ctx.fillText(label, OLED_W - PAD, altBase);
    }

    // Backlight PWM: veil toward black on the texture (material opacity is a no-op
    // on Chrome's opaque MeshBasic program, and would also punch through the panel).
    applyBacklightVeil(ctx);
    oledTex.needsUpdate = true;
    needFrame();
  }

  function drawSatIcon(ctx, x, y) {
    const w = 17, h = 9;
    ctx.fillStyle = COL.fg;
    for (let row = 0; row < h; row++) {
      const o = row * 3;
      const bits = (SAT_ICON[o] << 16) | (SAT_ICON[o + 1] << 8) | SAT_ICON[o + 2];
      for (let col = 0; col < w; col++) {
        if ((bits >> (23 - col)) & 1) ctx.fillRect(x + col, y + row, 1, 1);
      }
    }
  }

  function buildMatrix(digits) {
    const MAT_COLS = 19, MAT_ROWS = 7;
    const mat = Array.from({ length: MAT_ROWS }, () => Array(MAT_COLS).fill("bg"));
    const digitCol = (c) => c !== 5 && c !== 11 && c !== 13;
    for (let r = 0; r < MAT_ROWS; r++) {
      for (let c = 0; c < MAT_COLS; c++) {
        if (digitCol(c)) mat[r][c] = "off";
      }
    }
    let x = 0;
    for (const ch of digits) {
      const rows = DOT_FONT[ch] || DOT_FONT[" "];
      const cols = DOT_COLS[ch] ?? 5;
      for (let r = 0; r < 7; r++) {
        for (let c = 0; c < cols; c++) {
          const on = (rows[r] >> (cols - 1 - c)) & 1;
          if (x + c < MAT_COLS) mat[r][x + c] = on ? "fg" : "off";
        }
      }
      x += cols + 1;
    }
    return mat;
  }

  function paintHubOled() {
    const linked = document.body.dataset.link === "on";
    const dim = linked ? 1 : 0.34;
    const captionEl = document.getElementById("caption");
    const isError = captionEl?.classList.contains("error");
    const alpha = (!linked && isError) ? 1 : dim;

    const caption = (isError ? "! " : "") + (captionEl?.textContent || "");
    const right = document.getElementById("progress-label")?.textContent || "";
    const value = document.getElementById("value")?.textContent || "--";
    const unit = document.getElementById("unit")?.textContent || "";
    const figureLabel = document.getElementById("figure-label")?.textContent || "";
    const progressEl = document.getElementById("progress");
    const phase = document.body.dataset.phase;
    let p = Math.max(0, Math.min(1, parseFloat(progressEl?.style.getPropertyValue("--p") || "0") || 0));
    // Busy phases sweep the gauge in firmware; mirror that on the mesh.
    if (phase === "listing" || phase === "deleting" || phase === "reading" || phase === "saving") {
      const t = (performance.now() % 1500) / 1500;
      p = t < 0.5 ? t * 2 : (1 - t) * 2;
    }

    drawOledChrome(oledCtx, OLED_W, OLED_H, alpha, caption, right, value, unit, figureLabel, p);
  }

  function wrapLines(ctx, text, maxW, maxLines) {
    const words = text.split(/\s+/).filter(Boolean);
    const out = [];
    let line = "";
    for (let i = 0; i < words.length; i++) {
      const test = line ? line + " " + words[i] : words[i];
      if (ctx.measureText(test).width > maxW && line) {
        out.push(line);
        if (out.length >= maxLines) return out;
        line = words[i];
      } else {
        line = test;
      }
    }
    if (line && out.length < maxLines) out.push(line);
    return out;
  }

  function resize() {
    if (!renderer || !camera) return;
    const w = stage.clientWidth;
    const h = stage.clientHeight;
    if (w < 1 || h < 1) return;
    renderer.setSize(w, h, false);
    camera.aspect = w / h;
    camera.updateProjectionMatrix();
    if (loaded) computeFitDistance();
    needFrame();
  }

  function lerp(a, b, t) { return a + (b - a) * t; }

  function updatePose(dt) {
    const k = 1 - Math.exp(-dt * 4.5);
    pose.yaw = lerp(pose.yaw, poseTarget.yaw, k);
    pose.pitch = lerp(pose.pitch, poseTarget.pitch, k);
    pose.dist = lerp(pose.dist, poseTarget.dist, k);
    pose.frontal = lerp(pose.frontal, poseTarget.frontal, k);

    const tiltK = pointerActive ? 1 - Math.exp(-dt * 14) : 1 - Math.exp(-dt * 6);
    tiltX = lerp(tiltX, targetTiltX, tiltK);
    tiltY = lerp(tiltY, targetTiltY, tiltK);

    if (!REDUCED && !pointerActive && document.body.dataset.phase !== "downloading" &&
        document.body.dataset.phase !== "updating") {
      breathe += dt * 0.55;
    }

    const awaiting = document.body.dataset.phase === "awaiting";
    if (awaiting && !REDUCED) buttonPulse += dt * 3.2;
    else buttonPulse = 0;

    if (buttonGroup) {
      const s = awaiting && !REDUCED ? 1 + Math.sin(buttonPulse) * 0.04 : 1;
      buttonGroup.scale.setScalar(s);
    }
    if (displayMat) {
      // Settings backlight is painted into the OLED texture; only pairing uses material alpha.
      if (document.body.dataset.phase === "pairing" && !REDUCED) {
        setDisplayOpacity(0.34 + Math.sin(breathe * 2.2) * 0.18, true);
      } else {
        setDisplayOpacity(1, false);
      }
    }
    const busy = document.body.dataset.phase;
    if (busy === "listing" || busy === "deleting" || busy === "reading" || busy === "saving" ||
        settingsAnimActive()) {
      paintOled();
    }
  }

  function placeCamera() {
    if (!model) return;
    const breatheYaw = REDUCED || pointerActive ? 0 : Math.sin(breathe) * BREATHE_AMP * (1 - pose.frontal);
    const yaw = pose.yaw + tiltY + breatheYaw;
    const pitch = pose.pitch + tiltX;
    const dist = pose.dist;

    const x = Math.sin(yaw) * dist;
    const z = Math.cos(yaw) * dist;
    const y = Math.sin(pitch) * dist * 0.45;
    camera.position.set(x, y, z);
    camera.lookAt(0, 0, 0);
  }

  function frame(now) {
    raf = 0;
    if (destroyed || !loaded) { running = false; return; }
    const dt = Math.min(0.05, (now - clock.t) / 1000);
    clock.t = now;

    const wasPointer = pointerActive;
    updatePose(dt);
    placeCamera();

    const stillSettling =
      Math.abs(pose.yaw - poseTarget.yaw) > 0.001 ||
      Math.abs(pose.pitch - poseTarget.pitch) > 0.001 ||
      Math.abs(tiltX - targetTiltX) > 0.0005 ||
      Math.abs(tiltY - targetTiltY) > 0.0005 ||
      (!REDUCED && !pointerActive && document.body.dataset.phase !== "downloading") ||
      document.body.dataset.phase === "awaiting" ||
      settingsAnimActive();

    renderer.render(scene, camera);
    dirty = false;

    if (stillSettling || wasPointer || pointerActive || dirty) {
      raf = requestAnimationFrame(frame);
    } else {
      running = false;
    }
  }

  function startLoop() {
    if (running || destroyed || !loaded) return;
    if (document.visibilityState === "hidden") return;
    running = true;
    clock.t = performance.now();
    raf = requestAnimationFrame(frame);
  }

  // Pointer tilt (no pinch-zoom)
  function onPointerDown(e) {
    if (!loaded || REDUCED) return;
    if (e.pointerType === "mouse" && e.button !== 0) return;
    pointerActive = true;
    pointerId = e.pointerId;
    startX = e.clientX;
    startY = e.clientY;
    baseTiltX = targetTiltX;
    baseTiltY = targetTiltY;
    stage.setPointerCapture(e.pointerId);
    needFrame();
  }
  function onPointerMove(e) {
    if (!pointerActive || e.pointerId !== pointerId) return;
    const dx = e.clientX - startX;
    const dy = e.clientY - startY;
    const w = stage.clientWidth || 1;
    targetTiltY = THREE.MathUtils.clamp(baseTiltY + (dx / w) * TILT_MAX * 2, -TILT_MAX, TILT_MAX);
    targetTiltX = THREE.MathUtils.clamp(baseTiltX + (dy / w) * TILT_MAX * 1.2, -TILT_MAX, TILT_MAX);
    needFrame();
  }
  function onPointerUp(e) {
    if (e.pointerId !== pointerId) return;
    pointerActive = false;
    pointerId = null;
    targetTiltX = 0;
    targetTiltY = 0;
    needFrame();
  }

  const ro = new ResizeObserver(() => resize());
  ro.observe(stage);

  const mo = new MutationObserver(() => syncPoseFromBody());
  mo.observe(document.body, {
    attributes: true,
    attributeFilter: ["data-link", "data-phase", "data-auth", ...CFG_ATTRS]
  });
  const screenMo = new MutationObserver(() => paintOled());
  screenMo.observe(screen, { subtree: true, childList: true, characterData: true, attributes: true, attributeFilter: ["style", "class"] });

  const themeMq = matchMedia("(prefers-color-scheme: dark)");
  const onTheme = () => applyTheme();
  themeMq.addEventListener?.("change", onTheme);
  const themeAttrMo = new MutationObserver(onTheme);
  themeAttrMo.observe(document.documentElement, { attributes: true, attributeFilter: ["data-theme"] });

  const onVis = () => {
    if (document.visibilityState === "hidden") {
      if (raf) cancelAnimationFrame(raf);
      raf = 0;
      running = false;
    } else {
      needFrame();
    }
  };
  document.addEventListener("visibilitychange", onVis);

  stage.addEventListener("pointerdown", onPointerDown);
  stage.addEventListener("pointermove", onPointerMove);
  stage.addEventListener("pointerup", onPointerUp);
  stage.addEventListener("pointercancel", onPointerUp);

  // Kick off after paint
  const start = () => { if (!destroyed) boot(); };
  if ("requestIdleCallback" in window) requestIdleCallback(start, { timeout: 1200 });
  else setTimeout(start, 100);

  return {
    destroy() {
      destroyed = true;
      if (raf) cancelAnimationFrame(raf);
      ro.disconnect();
      mo.disconnect();
      screenMo.disconnect();
      themeMq.removeEventListener?.("change", onTheme);
      themeAttrMo.disconnect();
      oledTex?.dispose();
      displayMat?.dispose();
      document.removeEventListener("visibilitychange", onVis);
      stage.removeEventListener("pointerdown", onPointerDown);
      stage.removeEventListener("pointermove", onPointerMove);
      stage.removeEventListener("pointerup", onPointerUp);
      stage.removeEventListener("pointercancel", onPointerUp);
      renderer?.dispose();
    },
    /** Capture the current WebGL frame as a blob (for poster generation). */
    async capturePoster() {
      if (!renderer || !loaded) return null;
      needFrame();
      await new Promise((r) => requestAnimationFrame(() => requestAnimationFrame(r)));
      return new Promise((resolve) => canvas.toBlob(resolve, "image/webp", 0.92));
    }
  };
}
