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

    displayMat = new THREE.MeshBasicMaterial({
      map: oledTex,
      toneMapped: false
    });
    mesh.material = displayMat;
  }

  function paintOled() {
    if (!oledCtx || !oledTex) return;
    const ctx = oledCtx;
    const W = OLED_W, H = OLED_H;
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    ctx.globalAlpha = 1;
    ctx.fillStyle = "#000";
    ctx.fillRect(0, 0, W, H);

    const linked = document.body.dataset.link === "on";
    const dim = linked ? 1 : 0.34;
    const captionEl = document.getElementById("caption");
    const isError = captionEl?.classList.contains("error");
    const alpha = (!linked && isError) ? 1 : dim;

    const padX = W * 0.08;
    const padTop = H * 0.07;
    const caption = captionEl?.textContent || "";
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

    ctx.globalAlpha = alpha;
    ctx.fillStyle = "rgba(255,255,255,0.6)";
    ctx.textBaseline = "top";
    ctx.textAlign = "center";
    const capSize = Math.round(W * 0.058);
    const lineH = capSize * 1.25;
    ctx.font = `600 ${capSize}px Archivo, system-ui, sans-serif`;

    // Status + space available stacked in a centered column
    const capMaxW = W - padX * 2;
    const capLines = wrapLines(ctx, (isError ? "! " : "") + caption, capMaxW, 2);
    for (let i = 0; i < capLines.length; i++) {
      ctx.fillText(capLines[i], W / 2, padTop + i * lineH);
    }
    if (right) {
      ctx.fillText(right, W / 2, padTop + Math.max(1, capLines.length) * lineH);
    }
    ctx.textAlign = "left";

    // Hero figure (+ optional label under the count)
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

    // Dot gauge
    const dots = 24;
    const gaugeY = H * 0.935;
    const gaugeLeft = padX;
    const gaugeRight = W - padX;
    const step = (gaugeRight - gaugeLeft) / (dots - 1);
    const r = Math.max(1.6, W * 0.012);
    const filled = Math.round(p * dots);
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
      if (document.body.dataset.phase === "pairing" && !REDUCED) {
        displayMat.opacity = 0.34 + Math.sin(breathe * 2.2) * 0.18;
        displayMat.transparent = true;
      } else {
        displayMat.opacity = 1;
        displayMat.transparent = false;
      }
    }
    const busy = document.body.dataset.phase;
    if (busy === "listing" || busy === "deleting" || busy === "reading" || busy === "saving") {
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
      document.body.dataset.phase === "awaiting";

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
  mo.observe(document.body, { attributes: true, attributeFilter: ["data-link", "data-phase", "data-auth"] });
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
