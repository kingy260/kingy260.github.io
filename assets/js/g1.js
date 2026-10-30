/*!
 * Interactive Unitree G1 for the homepage hero.
 *
 * Robot model: Unitree G1 (29-DoF) from MuJoCo Menagerie, BSD-3-Clause,
 * Copyright (c) 2016-2023 HangZhou YuShu TECHNOLOGY CO., LTD. ("Unitree Robotics").
 * Meshes were decimated and quantized for the web; see licenses/unitree_g1_LICENSE.txt.
 *
 * How it moves: the centre of mass follows a linear inverted pendulum. A capture-point
 * (divergent component of motion) controller places the ZMP inside the support polygon;
 * when the capture point leaves the polygon the robot takes a step to catch it.
 * Legs are solved each frame with damped least-squares IK on the real G1 joint chain.
 * Everything runs on the main thread in well under a millisecond per frame.
 */
(function () {
  'use strict';

  // ------------------------------------------------------------------ math
  const I3 = () => [1, 0, 0, 0, 1, 0, 0, 0, 1];
  function mul3(A, B) {
    return [
      A[0] * B[0] + A[1] * B[3] + A[2] * B[6], A[0] * B[1] + A[1] * B[4] + A[2] * B[7], A[0] * B[2] + A[1] * B[5] + A[2] * B[8],
      A[3] * B[0] + A[4] * B[3] + A[5] * B[6], A[3] * B[1] + A[4] * B[4] + A[5] * B[7], A[3] * B[2] + A[4] * B[5] + A[5] * B[8],
      A[6] * B[0] + A[7] * B[3] + A[8] * B[6], A[6] * B[1] + A[7] * B[4] + A[8] * B[7], A[6] * B[2] + A[7] * B[5] + A[8] * B[8]];
  }
  const mv3 = (A, v) => [A[0] * v[0] + A[1] * v[1] + A[2] * v[2], A[3] * v[0] + A[4] * v[1] + A[5] * v[2], A[6] * v[0] + A[7] * v[1] + A[8] * v[2]];
  const tr3 = (A) => [A[0], A[3], A[6], A[1], A[4], A[7], A[2], A[5], A[8]];
  const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
  function quatMat(q) { // MuJoCo order [w, x, y, z]
    const w = q[0], x = q[1], y = q[2], z = q[3];
    return [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)];
  }
  function axisAng(a, t) {
    const c = Math.cos(t), s = Math.sin(t), C = 1 - c, x = a[0], y = a[1], z = a[2];
    return [c + x * x * C, x * y * C - z * s, x * z * C + y * s,
      y * x * C + z * s, c + y * y * C, y * z * C - x * s,
      z * x * C - y * s, z * y * C + x * s, c + z * z * C];
  }
  const rotX = (t) => axisAng([1, 0, 0], t);
  const rotY = (t) => axisAng([0, 1, 0], t);
  function logMap(R) {
    const c = Math.max(-1, Math.min(1, (R[0] + R[4] + R[8] - 1) / 2));
    const th = Math.acos(c);
    const v = [R[7] - R[5], R[2] - R[6], R[3] - R[1]];
    const k = th < 1e-6 ? 0.5 : th / (2 * Math.sin(th));
    return [v[0] * k, v[1] * k, v[2] * k];
  }
  function solve6(A, b) {
    const n = 6, M = A.slice(), x = b.slice();
    for (let i = 0; i < n; i++) {
      let p = i, mx = Math.abs(M[i * n + i]);
      for (let r = i + 1; r < n; r++) { const v = Math.abs(M[r * n + i]); if (v > mx) { mx = v; p = r; } }
      if (mx < 1e-12) return null;
      if (p !== i) {
        for (let k = 0; k < n; k++) { const t = M[i * n + k]; M[i * n + k] = M[p * n + k]; M[p * n + k] = t; }
        const t = x[i]; x[i] = x[p]; x[p] = t;
      }
      const d = M[i * n + i];
      for (let r = i + 1; r < n; r++) {
        const f = M[r * n + i] / d;
        if (!f) continue;
        for (let k = i; k < n; k++) M[r * n + k] -= f * M[i * n + k];
        x[r] -= f * x[i];
      }
    }
    for (let i = n - 1; i >= 0; i--) {
      let s = x[i];
      for (let k = i + 1; k < n; k++) s -= M[i * n + k] * x[k];
      x[i] = s / M[i * n + i];
    }
    return x;
  }
  const clamp = (v, a, b) => (v < a ? a : v > b ? b : v);
  const lerp = (a, b, t) => a + (b - a) * t;
  const smooth = (t) => { t = clamp(t, 0, 1); return t * t * (3 - 2 * t); };

  // ------------------------------------------------------- 2-D support polygon
  function hull2(pts) {
    const P = pts.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
    if (P.length < 3) return P;
    const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
    const lo = [], up = [];
    for (const p of P) { while (lo.length >= 2 && cr(lo[lo.length - 2], lo[lo.length - 1], p) <= 0) lo.pop(); lo.push(p); }
    for (let i = P.length - 1; i >= 0; i--) { const p = P[i]; while (up.length >= 2 && cr(up[up.length - 2], up[up.length - 1], p) <= 0) up.pop(); up.push(p); }
    up.pop(); lo.pop();
    return lo.concat(up);
  }
  function closestIn(H, p) { // -> [x, y, distanceOutside]
    let inside = true;
    for (let i = 0; i < H.length; i++) {
      const a = H[i], b = H[(i + 1) % H.length];
      if ((b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) < 0) { inside = false; break; }
    }
    if (inside) return [p[0], p[1], 0];
    let bx = 0, by = 0, bd = Infinity;
    for (let i = 0; i < H.length; i++) {
      const a = H[i], b = H[(i + 1) % H.length];
      const ex = b[0] - a[0], ey = b[1] - a[1], L = ex * ex + ey * ey;
      const t = L > 0 ? clamp(((p[0] - a[0]) * ex + (p[1] - a[1]) * ey) / L, 0, 1) : 0;
      const x = a[0] + t * ex, y = a[1] + t * ey, d = Math.hypot(p[0] - x, p[1] - y);
      if (d < bd) { bd = d; bx = x; by = y; }
    }
    return [bx, by, bd];
  }
  function polyCentroid(H) {
    let A = 0, cx = 0, cy = 0;
    for (let i = 0; i < H.length; i++) {
      const a = H[i], b = H[(i + 1) % H.length], c = a[0] * b[1] - b[0] * a[1];
      A += c; cx += (a[0] + b[0]) * c; cy += (a[1] + b[1]) * c;
    }
    if (Math.abs(A) < 1e-9) {
      let x = 0, y = 0; for (const p of H) { x += p[0]; y += p[1]; }
      return [x / H.length, y / H.length];
    }
    return [cx / (3 * A), cy / (3 * A)];
  }

  // -------------------------------------------------------------- constants
  const G = 9.81, CH = 0.75, W = Math.sqrt(G / CH), W2 = W * W;   // LIPM height and natural frequency
  const REACT_T = 0.26;                        // reactive step duration
  const ANKLE_Z = 0.035;                       // ankle_roll origin above the sole
  const FOOT = { back: -0.055, front: 0.125, half: 0.035, cx: 0.035 };
  const HALF_W = 0.118;                        // nominal lateral ankle offset
  const PELVIS_Z = 0.74;
  const KXI = 2.6, DAMP = 0.35;
  const MINW = 0.17, MAXW = 0.68, MAXF = 0.42, MAXB = 0.32;
  const HOME = [0, 0];
  const LEG_REACH = 0.605;
  const HIP = [0, HALF_W - 0.0015, -0.133];    // hip-roll origin in the pelvis frame (left)
  const TH_MAX = 1.40;                         // lying angle after a fall
  const LEGS = ['hip_pitch', 'hip_roll', 'hip_yaw', 'knee', 'ankle_pitch', 'ankle_roll'];
  const ARMS = ['shoulder_pitch', 'shoulder_roll', 'shoulder_yaw', 'elbow', 'wrist_roll', 'wrist_pitch', 'wrist_yaw'];
  const POSE_HANG = { leg: [-0.05, 0, 0, 0.3, -0.18, 0], arm: [0.12, 0.14, 0, 0.95, 0, 0, 0], waist: [0, 0, 0] };
  const POSE_SLUMP = { leg: [-0.95, 0, 0, 1.55, -0.4, 0], arm: [-0.55, 0.55, 0, 0.75, 0, 0, 0], waist: [0, 0, 0.3] };

  // --------------------------------------------------------------- loading
  async function loadModel(url) {
    const res = await fetch(url);
    if (!res.ok) throw new Error('model ' + res.status);
    let buf;
    if (/\.json(\?|$)/.test(url)) {
      // the packed model is shipped base64-encoded inside JSON so any static host serves (and gzips) it
      const bin = atob((await res.json()).bin);
      const bytes = new Uint8Array(bin.length);
      for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
      buf = bytes.buffer;
    } else buf = await res.arrayBuffer();
    const dv = new DataView(buf);
    if (dv.getUint32(0, true) !== 0x31314731) throw new Error('bad model file');
    const jl = dv.getUint32(4, true);
    const meta = JSON.parse(new TextDecoder().decode(new Uint8Array(buf, 8, jl)));
    const THREE = window.THREE;
    const geoms = meta.meshes.map((m) => {
      const qp = new Uint16Array(buf, m.pos, m.nv * 3);
      const pos = new Float32Array(m.nv * 3);
      for (let i = 0; i < m.nv; i++) for (let k = 0; k < 3; k++) pos[i * 3 + k] = m.min[k] + (qp[i * 3 + k] / 65535) * m.ext[k];
      const g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      g.setAttribute('normal', new THREE.BufferAttribute(new Int8Array(buf.slice(m.nrm, m.nrm + m.nv * 3)), 3, true));
      g.setIndex(new THREE.BufferAttribute(new Uint16Array(buf.slice(m.idx, m.idx + m.ni * 2)), 1));
      g.computeBoundingSphere();
      g.computeBoundingBox();
      return g;
    });
    return { meta, geoms };
  }

  // ----------------------------------------------------------------- mount
  function mount(container, opts) {
    opts = opts || {};
    const THREE = window.THREE;
    if (!THREE) throw new Error('three.js missing');
    if (THREE.ColorManagement && 'legacyMode' in THREE.ColorManagement) THREE.ColorManagement.legacyMode = false;
    const reduceMotion = !!(window.matchMedia && window.matchMedia('(prefers-reduced-motion: reduce)').matches);
    const onStats = opts.onStats || function () {};

    // renderer
    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, powerPreference: 'high-performance', preserveDrawingBuffer: !!opts.preserveDrawingBuffer });
    } catch (e) { return Promise.reject(e); }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputEncoding = THREE.sRGBEncoding;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 0.95;
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;
    renderer.setClearColor(0x000000, 0);
    const canvas = renderer.domElement;
    canvas.className = 'g1-canvas';
    canvas.setAttribute('aria-hidden', 'true');
    container.appendChild(canvas);

    const scene = new THREE.Scene();
    const camera = new THREE.PerspectiveCamera(28, 1, 0.05, 40);

    // environment for the aluminium shells
    const pmrem = new THREE.PMREMGenerator(renderer);
    const envScene = new THREE.Scene();
    {
      const sg = new THREE.SphereGeometry(6, 32, 16);
      const col = [], p = sg.attributes.position;
      const top = new THREE.Color(1.0, 1.0, 1.02), mid = new THREE.Color(0.55, 0.57, 0.6), bot = new THREE.Color(0.16, 0.16, 0.17);
      for (let i = 0; i < p.count; i++) {
        const y = p.getY(i) / 6, c = y > 0 ? mid.clone().lerp(top, smooth(y)) : mid.clone().lerp(bot, smooth(-y * 1.6));
        col.push(c.r, c.g, c.b);
      }
      sg.setAttribute('color', new THREE.Float32BufferAttribute(col, 3));
      envScene.add(new THREE.Mesh(sg, new THREE.MeshBasicMaterial({ vertexColors: true, side: THREE.BackSide })));
      const box = new THREE.MeshBasicMaterial({ color: 0xffffff }); box.color.setScalar(3.2);
      const a = new THREE.Mesh(new THREE.PlaneGeometry(4, 2.4), box); a.position.set(3.2, 3.2, 3.6); a.lookAt(0, 0, 0); envScene.add(a);
      const b = new THREE.Mesh(new THREE.PlaneGeometry(2.4, 3.2), box.clone()); b.material.color.setScalar(1.6); b.position.set(-4, 1.8, -2.4); b.lookAt(0, 0, 0); envScene.add(b);
    }
    scene.environment = pmrem.fromScene(envScene, 0.035).texture;

    const hemi = new THREE.HemisphereLight(0xffffff, 0x8d9096, 0.42);
    scene.add(hemi);
    const key = new THREE.DirectionalLight(0xffffff, 1.35);
    key.position.set(1.6, 5.6, 1.9);
    key.castShadow = true;
    key.shadow.mapSize.set(1024, 1024);
    key.shadow.camera.left = -1.1; key.shadow.camera.right = 1.1; key.shadow.camera.top = 1.1; key.shadow.camera.bottom = -1.1;
    key.shadow.camera.near = 1; key.shadow.camera.far = 9;
    key.shadow.bias = -0.0004; key.shadow.normalBias = 0.015;
    scene.add(key); scene.add(key.target);
    const rim = new THREE.DirectionalLight(0xdbe4ff, 0.75);
    rim.position.set(-3, 2.6, -2.6);
    scene.add(rim);

    // MuJoCo world (z up) inside a y-up three.js scene
    const world = new THREE.Group();
    world.rotation.x = -Math.PI / 2;
    scene.add(world);
    const fallPivot = new THREE.Group(), fallOffset = new THREE.Group();
    world.add(fallPivot); fallPivot.add(fallOffset);

    // ground: faded grid + shadow catcher + balance overlay
    const gridCanvas = document.createElement('canvas'); gridCanvas.width = gridCanvas.height = 1024;
    const gridTex = new THREE.CanvasTexture(gridCanvas);
    gridTex.encoding = THREE.sRGBEncoding;
    gridTex.anisotropy = Math.min(8, renderer.capabilities.getMaxAnisotropy());
    const GRID_SIZE = 7;
    const grid = new THREE.Mesh(new THREE.PlaneGeometry(GRID_SIZE, GRID_SIZE), new THREE.MeshBasicMaterial({ map: gridTex, transparent: true, depthWrite: false }));
    grid.renderOrder = -2;
    world.add(grid);
    const shadowMat = new THREE.ShadowMaterial({ opacity: 0.2 });
    const shadowPlane = new THREE.Mesh(new THREE.PlaneGeometry(GRID_SIZE, GRID_SIZE), shadowMat);
    shadowPlane.receiveShadow = true; shadowPlane.position.z = 0.0005; shadowPlane.renderOrder = -1;
    world.add(shadowPlane);

    const overlay = new THREE.Group(); overlay.position.z = 0.0025; world.add(overlay);
    const hullGeo = new THREE.BufferGeometry(); hullGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 12), 3));
    const hullMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.55, depthWrite: false });
    const hullLine = new THREE.LineLoop(hullGeo, hullMat); overlay.add(hullLine);
    const hullFillGeo = new THREE.BufferGeometry(); hullFillGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(3 * 3 * 12), 3));
    const hullFillMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.07, depthWrite: false, side: THREE.DoubleSide });
    const hullFill = new THREE.Mesh(hullFillGeo, hullFillMat); overlay.add(hullFill);
    const comMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.85, depthWrite: false });
    const comDot = new THREE.Mesh(new THREE.CircleGeometry(0.013, 20), comMat); overlay.add(comDot);
    const xiMat = new THREE.MeshBasicMaterial({ transparent: true, opacity: 0.95, depthWrite: false, side: THREE.DoubleSide });
    const xiRing = new THREE.Mesh(new THREE.RingGeometry(0.022, 0.032, 32), xiMat); overlay.add(xiRing);
    const linkGeo = new THREE.BufferGeometry(); linkGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const linkMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.8, depthWrite: false });
    const comLink = new THREE.Line(linkGeo, linkMat); overlay.add(comLink);

    // drag handle (MuJoCo-style perturbation line)
    const dragGeo = new THREE.BufferGeometry(); dragGeo.setAttribute('position', new THREE.BufferAttribute(new Float32Array(6), 3));
    const dragMat = new THREE.LineBasicMaterial({ transparent: true, opacity: 0.95, depthTest: false });
    const dragLine = new THREE.Line(dragGeo, dragMat); dragLine.renderOrder = 10; dragLine.visible = false; world.add(dragLine);
    const dragTip = new THREE.Mesh(new THREE.SphereGeometry(0.018, 16, 12), new THREE.MeshBasicMaterial({ depthTest: false, transparent: true }));
    dragTip.renderOrder = 10; dragTip.visible = false; world.add(dragTip);

    // gantry cable
    const cableMat = new THREE.MeshStandardMaterial({ color: 0x3a3d42, roughness: 0.6, metalness: 0.2 });
    const cable = new THREE.Mesh(new THREE.CylinderGeometry(0.006, 0.006, 1, 8), cableMat);
    cable.visible = false; world.add(cable);
    const hook = new THREE.Mesh(new THREE.TorusGeometry(0.022, 0.006, 8, 20), cableMat);
    hook.visible = false; world.add(hook);

    // theme
    let theme = {};
    function readTheme() {
      const cs = getComputedStyle(container);
      const v = (n, d) => (cs.getPropertyValue(n) || '').trim() || d;
      theme = {
        grid: v('--g1-grid', 'rgba(20,24,28,0.10)'), gridStrong: v('--g1-grid-strong', 'rgba(20,24,28,0.22)'),
        ink: v('--g1-ink', '#1b1f24'), accent: v('--g1-accent', '#c41230'),
        shadow: parseFloat(v('--g1-shadow', '0.2')) || 0.2,
        rim: parseFloat(v('--g1-rim', '0.75')) || 0.75,
      };
      rim.intensity = theme.rim;
      const ctx = gridCanvas.getContext('2d');
      const N = gridCanvas.width, px = N / GRID_SIZE;
      ctx.clearRect(0, 0, N, N);
      const drawLines = (step, color, width) => {
        ctx.strokeStyle = color; ctx.lineWidth = width; ctx.beginPath();
        for (let m = -GRID_SIZE / 2; m <= GRID_SIZE / 2 + 1e-6; m += step) {
          const p = Math.round((m + GRID_SIZE / 2) * px) + 0.5;
          ctx.moveTo(p, 0); ctx.lineTo(p, N); ctx.moveTo(0, p); ctx.lineTo(N, p);
        }
        ctx.stroke();
      };
      drawLines(0.25, theme.grid, 1.25);
      drawLines(1.0, theme.gridStrong, 1.6);
      ctx.globalCompositeOperation = 'destination-in';
      const g = ctx.createRadialGradient(N / 2, N / 2, N * 0.06, N / 2, N / 2, N * 0.5);
      g.addColorStop(0, 'rgba(0,0,0,1)'); g.addColorStop(0.55, 'rgba(0,0,0,0.75)'); g.addColorStop(1, 'rgba(0,0,0,0)');
      ctx.fillStyle = g; ctx.fillRect(0, 0, N, N);
      ctx.globalCompositeOperation = 'source-over';
      gridTex.needsUpdate = true;
      shadowMat.opacity = theme.shadow;
      hullMat.color.set(theme.ink); hullFillMat.color.set(theme.ink); comMat.color.set(theme.ink); linkMat.color.set(theme.ink);
      xiMat.color.set(theme.accent); dragMat.color.set(theme.accent); dragTip.material.color.set(theme.accent);
    }

    // --------------------------------------------------------------- state
    const S = {
      mode: 'boot', time: 0, modeT: 0,
      c: [HOME[0], HOME[1]], v: [0, 0], acc: [0, 0], xi: [0, 0], zmp: [0, 0], sat: 0,
      feet: [
        { x: HOME[0] - FOOT.cx, y: HOME[1] + HALF_W, lift: 0, planted: true },
        { x: HOME[0] - FOOT.cx, y: HOME[1] - HALF_W, lift: 0, planted: true }],
      swing: null, vstep: null, ds: 1, settle: 0, lastSwing: 1,
      aext: [0, 0], grab: null, steps: 0, falls: 0, reactSteps: 0,
      pz: PELVIS_Z, yaw: 0, yawT: 0, react: 0, pointer: null, idleLook: 0,
      fall: null, hoist: null, cableOut: 0, log: [],
    };

    let R = null; // robot (built after load)
    const api = {
      state: S,
      push(vx, vy) { if (S.mode === 'stand') { S.v[0] += vx; S.v[1] += vy; S.settle = 0; } },
      shove() {
        const a = Math.random() * Math.PI * 2, m = 0.95 + Math.random() * 0.5;
        api.push(Math.cos(a) * m, Math.sin(a) * m);
      },
      reset() { resetStand(); },
      destroy() { running = false; cancelAnimationFrame(raf); ro && ro.disconnect(); io && io.disconnect(); renderer.dispose(); canvas.remove(); },
    };

    function footRect(f) {
      return [[f.x + FOOT.back, f.y - FOOT.half], [f.x + FOOT.front, f.y - FOOT.half], [f.x + FOOT.front, f.y + FOOT.half], [f.x + FOOT.back, f.y + FOOT.half]];
    }
    function supportHull() {
      const pts = [];
      for (const f of S.feet) if (f.planted) pts.push(...footRect(f));
      return pts.length ? hull2(pts) : footRect(S.feet[0]);
    }
    function resetStand() {
      S.mode = 'stand'; S.modeT = 0;
      S.c = [HOME[0], HOME[1]]; S.v = [0, 0]; S.acc = [0, 0];
      S.feet[0] = { x: HOME[0] - FOOT.cx, y: HOME[1] + HALF_W, lift: 0, planted: true };
      S.feet[1] = { x: HOME[0] - FOOT.cx, y: HOME[1] - HALF_W, lift: 0, planted: true };
      S.swing = null; S.vstep = null; S.ds = 1; S.settle = 0; S.reactSteps = 0; S.aext = [0, 0];
      S.fall = null; S.hoist = null; S.cableOut = 0; S.pz = PELVIS_Z;
      fallPivot.position.set(0, 0, 0); fallPivot.quaternion.identity(); fallOffset.position.set(0, 0, 0);
      if (R) R.legs.forEach((L) => { L.q = [-0.32, 0, 0, 0.66, -0.34, 0]; });
      onStats(S);
    }

    // ------------------------------------------------------- controller
    function planStep(foot, Trem) {
      const st = S.feet[1 - foot];
      const off = [S.aext[0] / W2, S.aext[1] / W2];
      const xp = [S.xi[0] + off[0], S.xi[1] + off[1]];
      const z = closestIn(footRect(st), xp);
      const E = Math.exp(W * Math.max(0, Trem));
      const xT = [z[0] + (xp[0] - z[0]) * E, z[1] + (xp[1] - z[1]) * E];
      const sc = [st.x + FOOT.cx, st.y];
      const dx = xT[0] - sc[0], dy = xT[1] - sc[1], dl = Math.hypot(dx, dy) || 1;
      const ax = xT[0] + (0.045 * dx) / dl - FOOT.cx, ay = xT[1] + (0.045 * dy) / dl;
      const side = foot === 0 ? 1 : -1;
      const ry = side * clamp(side * (ay - st.y), MINW, MAXW);
      const rx = clamp(ax - st.x, -MAXB, MAXF);
      const tx = st.x + rx, ty = st.y + ry;
      // how far the predicted capture point would still be outside the new support polygon
      const res = closestIn(hull2(footRect(st).concat(footRect({ x: tx, y: ty }))), xT)[2];
      return [tx, ty, res];
    }
    function startReactiveStep(xp) {
      const hc = polyCentroid(supportHull());
      const d = [xp[0] - hc[0], xp[1] - hc[1]];
      // search swing foot x step time; prefer slower steps and, for sideways pushes, the outer foot
      const lateral = Math.abs(d[1]) > 0.6 * Math.abs(d[0]);
      const s0 = (S.feet[0].x + FOOT.cx - hc[0]) * d[0] + (S.feet[0].y - hc[1]) * d[1];
      const s1 = (S.feet[1].x + FOOT.cx - hc[0]) * d[0] + (S.feet[1].y - hc[1]) * d[1];
      const trailing = Math.abs(s0 - s1) < 0.003 ? 1 - S.lastSwing : s0 < s1 ? 0 : 1;
      let best = null;
      for (const foot of [0, 1]) {
        for (const T of [0.3, 0.25, 0.2]) {
          const p = planStep(foot, T);
          const pref = lateral ? ((d[1] > 0) === (foot === 0) ? 0 : 0.02) : (foot === trailing ? 0 : 0.012);
          const score = p[2] + (0.3 - T) * 0.15 + pref;
          if (!best || score < best.score) best = { foot, T, p, score };
        }
      }
      const foot = best.foot, T = best.T, tgt = best.p;
      if (opts.debug) S.log.push(['react', foot, +S.feet[foot].x.toFixed(2), +S.feet[foot].y.toFixed(2), '->', +tgt[0].toFixed(2), +tgt[1].toFixed(2), 'xi', +xp[0].toFixed(2), +xp[1].toFixed(2), 't', +S.time.toFixed(2)]);
      const f = S.feet[foot];
      const len = Math.hypot(tgt[0] - f.x, tgt[1] - f.y);
      S.swing = { foot, sx: f.x, sy: f.y, tx: tgt[0], ty: tgt[1], t: 0, T, H: 0.045 + 0.07 * Math.min(1, len / 0.4), kind: 'react' };
      f.planted = false; S.lastSwing = foot; S.vstep = null;
    }
    function shiftTarget(vs) {
      const st = S.feet[1 - vs.foot], sw = S.feet[vs.foot];
      return [st.x + FOOT.cx, st.y + 0.022 * Math.sign(sw.y - st.y)];
    }
    function startVoluntary(foot, tx, ty, kind) { S.vstep = { foot, tx, ty, kind, phase: 'shift', t: 0 }; S.settle = 0; if (opts.debug) S.log.push(['vol', kind, foot, +S.feet[foot].x.toFixed(2), +S.feet[foot].y.toFixed(2), '->', +tx.toFixed(2), +ty.toFixed(2), 'c', +S.c[1].toFixed(2), 't', +S.time.toFixed(2)]); }
    function updateVstep(h) {
      const vs = S.vstep; vs.t += h;
      if (vs.phase === 'shift') {
        const ref = shiftTarget(vs);
        const close = Math.hypot(S.xi[0] - ref[0], S.xi[1] - ref[1]) < 0.04 && Math.hypot(S.c[0] - ref[0], S.c[1] - ref[1]) < 0.065;
        if ((close && Math.hypot(S.v[0], S.v[1]) < 0.2) || vs.t > 0.7) {
          const f = S.feet[vs.foot], len = Math.hypot(vs.tx - f.x, vs.ty - f.y);
          S.swing = { foot: vs.foot, sx: f.x, sy: f.y, tx: vs.tx, ty: vs.ty, t: 0, T: 0.3 + 0.12 * Math.min(1, len / 0.3), H: 0.04 + 0.05 * Math.min(1, len / 0.3), kind: 'vol' };
          f.planted = false; vs.phase = 'swing'; S.lastSwing = vs.foot;
        }
      } else if (vs.phase === 'settle' && vs.t > 0.12) S.vstep = null;
    }
    function maybeTidyOrHome(h) {
      const speed = Math.hypot(S.v[0], S.v[1]);
      const toH = [HOME[0] - S.c[0], HOME[1] - S.c[1]], dH = Math.hypot(toH[0], toH[1]);
      if (speed < (dH > 0.16 ? 0.14 : 0.06) && S.sat < 0.004 && !S.grab) S.settle += h; else S.settle = 0;
      if (S.settle > 0.2) S.reactSteps = 0;
      if (S.settle < (dH > 0.16 ? 0.1 : 0.5)) return;
      const L = S.feet[0], Rf = S.feet[1];
      if (dH > 0.16) {
        // walk home: move whichever foot is furthest from a nominal stance centred one step closer
        const step = Math.min(0.3, dH), ux = toH[0] / dH, uy = toH[1] / dH;
        const nc = [S.c[0] + step * ux, S.c[1] + step * uy];
        const tg = [[nc[0] - FOOT.cx, nc[1] + HALF_W], [nc[0] - FOOT.cx, nc[1] - HALF_W]];
        const e0 = Math.hypot(L.x - tg[0][0], L.y - tg[0][1]), e1 = Math.hypot(Rf.x - tg[1][0], Rf.y - tg[1][1]);
        const lead = Math.abs(uy) > 0.5 * Math.abs(ux) ? (uy > 0 ? 0 : 1) : ((L.x - Rf.x) * ux > 0 ? 1 : 0);
        let foot = Math.abs(e0 - e1) < 0.02 ? lead : e0 > e1 ? 0 : 1;
        if (foot === S.lastSwing && Math.min(e0, e1) > 0.05) foot = 1 - foot;
        const place = (fi) => {
          const st = S.feet[1 - fi], side = fi === 0 ? 1 : -1;
          const ry = side * clamp(side * (tg[fi][1] - st.y), MINW, 0.42), rx = clamp(tg[fi][0] - st.x, -0.25, 0.3);
          return [st.x + rx, st.y + ry, Math.hypot(st.x + rx - S.feet[fi].x, st.y + ry - S.feet[fi].y)];
        };
        let p = place(foot);
        if (p[2] < 0.03) { foot = 1 - foot; p = place(foot); }
        if (p[2] >= 0.03) { startVoluntary(foot, p[0], p[1], 'home'); return; }
      }
      const wErr = Math.abs(L.y - Rf.y - 2 * HALF_W), xErr = Math.abs(L.x - Rf.x);
      if (wErr > 0.045 || xErr > 0.06) {
        const nL = [S.c[0] - FOOT.cx, S.c[1] + HALF_W], nR = [S.c[0] - FOOT.cx, S.c[1] - HALF_W];
        const eL = Math.hypot(L.x - nL[0], L.y - nL[1]), eR = Math.hypot(Rf.x - nR[0], Rf.y - nR[1]);
        const foot = eL > eR ? 0 : 1, st = S.feet[1 - foot], side = foot === 0 ? 1 : -1;
        startVoluntary(foot, st.x, st.y + side * 2 * HALF_W, 'tidy');
      }
    }
    function startFall() {
      const hull = supportHull();
      let P = closestIn(hull, S.c), u;
      if (P[2] > 1e-3) u = [(S.c[0] - P[0]) / P[2], (S.c[1] - P[1]) / P[2]];
      else {
        const sp = Math.hypot(S.v[0], S.v[1]) || 1; u = [S.v[0] / sp, S.v[1] / sp];
        let best = -Infinity;
        for (const p of hull) { const d = p[0] * u[0] + p[1] * u[1]; if (d > best) { best = d; P = [p[0], p[1], 0]; } }
      }
      const d0 = Math.max(0, (S.c[0] - P[0]) * u[0] + (S.c[1] - P[1]) * u[1]);
      const th0 = Math.atan2(d0, CH);
      S.fall = { P: [P[0], P[1]], u, th0, th: th0, w: Math.max(0.7, (S.v[0] * u[0] + S.v[1] * u[1]) / CH), q0: snapshotJoints(), lift: 0 };
      S.mode = 'fall'; S.modeT = 0; S.falls++; S.swing = null; S.vstep = null; S.aext = [0, 0];
      endGrab();
      fallPivot.position.set(P[0], P[1], 0); fallOffset.position.set(-P[0], -P[1], 0);
      onStats(S);
    }
    function physics(h) {
      S.time += h; S.modeT += h;
      if (S.mode === 'fall') {
        const F = S.fall;
        F.w += (G / CH) * Math.sin(F.th) * h; F.th += F.w * h;
        if (F.th >= TH_MAX) { F.th = TH_MAX; F.w = -0.2 * F.w; if (Math.abs(F.w) < 0.3) { S.mode = 'fallen'; S.modeT = 0; } }
        return;
      }
      if (S.mode === 'fallen') { if (S.modeT > (reduceMotion ? 0.6 : 1.0)) startHoist(); return; }
      if (S.mode === 'hoist') { S.hoist.t += h; if (S.hoist.t >= S.hoist.end) finishHoist(); return; }
      if (S.mode !== 'stand') return;

      let ax = S.aext[0], ay = S.aext[1];
      const rh = Math.hypot(S.c[0] - HOME[0], S.c[1] - HOME[1]);
      if (rh > 1.2) { const k = (12 * (rh - 1.2)) / rh; ax -= k * (S.c[0] - HOME[0]); ay -= k * (S.c[1] - HOME[1]); }
      const hull = supportHull();
      const xi = [S.c[0] + S.v[0] / W, S.c[1] + S.v[1] / W];
      const off = [ax / W2, ay / W2];
      const shifting = S.vstep && S.vstep.phase !== 'settle';
      const ref = shifting ? shiftTarget(S.vstep) : polyCentroid(hull);
      const kx = shifting ? 3.4 : KXI;
      const zd = [xi[0] + (kx / W) * (xi[0] - ref[0]) + off[0], xi[1] + (kx / W) * (xi[1] - ref[1]) + off[1]];
      const z = closestIn(hull, zd);
      S.sat = Math.hypot(zd[0] - z[0], zd[1] - z[1]);
      const a0 = W2 * (S.c[0] - z[0]) + ax - DAMP * S.v[0], a1 = W2 * (S.c[1] - z[1]) + ay - DAMP * S.v[1];
      S.v[0] += a0 * h; S.v[1] += a1 * h; S.c[0] += S.v[0] * h; S.c[1] += S.v[1] * h;
      S.acc[0] += (a0 - S.acc[0]) * Math.min(1, h * 20); S.acc[1] += (a1 - S.acc[1]) * Math.min(1, h * 20);
      S.zmp = [z[0], z[1]];
      S.xi = [S.c[0] + S.v[0] / W, S.c[1] + S.v[1] / W];

      if (S.swing) {
        const sw = S.swing; sw.t += h;
        const s = Math.min(1, sw.t / sw.T);
        if (sw.kind === 'react' && s < 0.65) {
          const tgt = planStep(sw.foot, (1 - s) * sw.T);
          sw.tx += (tgt[0] - sw.tx) * 0.15; sw.ty += (tgt[1] - sw.ty) * 0.15;
        }
        const e = smooth(s), f = S.feet[sw.foot];
        f.x = lerp(sw.sx, sw.tx, e); f.y = lerp(sw.sy, sw.ty, e);
        f.lift = sw.H * Math.sin(Math.PI * Math.min(1, s * 1.04));
        if (s >= 1) {
          f.x = sw.tx; f.y = sw.ty; f.lift = 0; f.planted = true; S.swing = null; S.ds = 0; S.steps++;
          if (sw.kind === 'react') S.reactSteps++;
          if (S.vstep) { S.vstep.phase = 'settle'; S.vstep.t = 0; }
          onStats(S);
        }
        const st = S.feet[1 - sw.foot];
        if (closestIn(footRect(st), S.c)[2] > 0.5) startFall();
        return;
      }
      S.ds += h;
      const xp = [S.xi[0] + off[0], S.xi[1] + off[1]];
      const out = closestIn(hull, xp)[2];
      const cOut = closestIn(hull, S.c)[2];
      if (cOut > 0.27 || out > 1.15 || S.reactSteps > 10) { startFall(); return; }
      if (S.ds > (out > 0.12 ? 0.015 : 0.04) && (out > 0.02 || (!shifting && S.sat > 0.05))) { startReactiveStep(xp); return; }
      if (S.vstep) updateVstep(h); else maybeTidyOrHome(h);
    }

    // ------------------------------------------------------ hoist (gantry)
    const _m = new THREE.Matrix4(), _q = new THREE.Quaternion(), _p = new THREE.Vector3(), _s = new THREE.Vector3();
    function startHoist(fromBoot) {
      let pos0, quat0, q0;
      if (fromBoot) {
        pos0 = new THREE.Vector3(HOME[0], HOME[1], 0.95); quat0 = new THREE.Quaternion(); q0 = poseJoints(POSE_HANG);
      } else {
        R.root.updateMatrix(); fallPivot.updateMatrix(); fallOffset.updateMatrix();
        _m.copy(fallPivot.matrix).multiply(fallOffset.matrix).multiply(R.root.matrix);
        _m.decompose(_p, _q, _s);
        pos0 = _p.clone(); quat0 = _q.clone(); q0 = snapshotJoints();
      }
      fallPivot.position.set(0, 0, 0); fallPivot.quaternion.identity(); fallOffset.position.set(0, 0, 0);
      R.root.position.copy(pos0); R.root.quaternion.copy(quat0);
      const tA = fromBoot ? 0 : 0.55, tB = fromBoot ? 0.2 : tA + 1.5, tC = tB + 0.75;
      S.hoist = { t: 0, tA, tB, tC, end: tC, pos0, quat0, q0, boot: !!fromBoot };
      S.mode = 'hoist'; S.modeT = 0; S.fall = null;
      S.c = [HOME[0], HOME[1]]; S.v = [0, 0]; S.acc = [0, 0];
      S.feet[0] = { x: HOME[0] - FOOT.cx, y: HOME[1] + HALF_W, lift: 0, planted: true };
      S.feet[1] = { x: HOME[0] - FOOT.cx, y: HOME[1] - HALF_W, lift: 0, planted: true };
      S.swing = null; S.vstep = null; S.aext = [0, 0];
    }
    function finishHoist() {
      S.mode = 'stand'; S.modeT = 0; S.hoist = null; S.ds = 1; S.settle = 0; S.reactSteps = 0;
      S.cableOut = 1; // retract animation
      onStats(S);
    }

    // ------------------------------------------------------------ robot
    function build(meta, geoms) {
      const env = scene.environment;
      const mats = {
        metal: new THREE.MeshStandardMaterial({ color: 0xa9aeb5, metalness: 0.62, roughness: 0.3, envMap: env, envMapIntensity: 1.0 }),
        black: new THREE.MeshStandardMaterial({ color: 0x2a2c30, metalness: 0.18, roughness: 0.5, envMap: env, envMapIntensity: 0.9 }),
      };
      const bodies = meta.bodies.map((b) => Object.assign({}, b));
      const meshes = [];
      bodies.forEach((b, i) => {
        const g = new THREE.Group();
        if (i > 0) { g.position.fromArray(b.pos); g.quaternion.set(b.quat[1], b.quat[2], b.quat[3], b.quat[0]); }
        const jg = new THREE.Group(); g.add(jg);
        b.group = g; b.jg = jg; b.q = 0;
        b.axisV = b.joint ? new THREE.Vector3().fromArray(b.joint.axis) : null;
        b.geoms.forEach((gm) => {
          const mesh = new THREE.Mesh(geoms[gm.mesh], mats[gm.mat] || mats.metal);
          mesh.position.fromArray(gm.pos); mesh.quaternion.set(gm.quat[1], gm.quat[2], gm.quat[3], gm.quat[0]);
          mesh.castShadow = true;
          const src = geoms[gm.mesh].attributes.position.array, step = 3 * 3, sample = [];
          for (let k = 0; k < src.length; k += step) sample.push(src[k], src[k + 1], src[k + 2]);
          mesh.userData.sample = sample;
          jg.add(mesh); meshes.push(mesh);
        });
        if (b.parent >= 0) bodies[b.parent].jg.add(g);
      });
      const byName = {};
      bodies.forEach((b) => { byName[b.name] = b; });
      const legs = ['left', 'right'].map((s) => {
        const chain = LEGS.map((n) => {
          const b = byName[`${s}_${n}_link`];
          const lo = n === 'knee' ? 0.03 : b.joint.range[0];
          return { b, p: b.pos, R: quatMat(b.quat), a: b.joint.axis, lo, hi: b.joint.range[1] };
        });
        return { side: s, chain, q: [-0.32, 0, 0, 0.66, -0.34, 0] };
      });
      const arms = ['left', 'right'].map((s) => ARMS.map((n) => byName[`${s}_${n}_link`]));
      const waist = [byName.waist_yaw_link, byName.waist_roll_link, byName.torso_link];
      const root = bodies[0].group;
      fallOffset.add(root);
      // hook point: top of the head, in the torso frame
      let attach = new THREE.Vector3(0, 0, 0.45);
      const torsoJG = byName.torso_link.jg;
      torsoJG.children.forEach((m) => {
        if (m.isMesh && m.geometry.boundingBox && m.material === mats.black) {
          const bb = m.geometry.boundingBox.clone().applyMatrix4(new THREE.Matrix4().compose(m.position, m.quaternion, new THREE.Vector3(1, 1, 1)));
          if (bb.max.z > attach.z - 0.2) attach = new THREE.Vector3((bb.min.x + bb.max.x) / 2, 0, bb.max.z - 0.004);
        }
      });
      return { bodies, byName, legs, arms, waist, root, meshes, attach, mats };
    }
    function setJ(b, q) { b.q = q; b.jg.quaternion.setFromAxisAngle(b.axisV, q); }
    function snapshotJoints() {
      return { leg: R.legs.map((L) => L.q.slice()), arm: R.arms.map((A) => A.map((b) => b.q)), waist: R.waist.map((b) => b.q) };
    }
    function poseJoints(p) {
      return {
        leg: [p.leg.slice(), p.leg.slice()],
        arm: [p.arm.slice(), p.arm.map((v, i) => (i === 1 || i === 2 || i === 4 || i === 6 ? -v : v))],
        waist: p.waist.slice(),
      };
    }
    const _inv = new THREE.Matrix4(), _mm = new THREE.Matrix4();
    function lowestPoint() { // exact-enough lowest vertex of the robot, in the MuJoCo world frame
      _inv.copy(world.matrixWorld).invert();
      let minZ = Infinity;
      for (const m of R.meshes) {
        _mm.multiplyMatrices(_inv, m.matrixWorld);
        const e = _mm.elements, s = m.userData.sample;
        for (let i = 0; i < s.length; i += 3) {
          const z = e[2] * s[i] + e[6] * s[i + 1] + e[10] * s[i + 2] + e[14];
          if (z < minZ) minZ = z;
        }
      }
      return minZ;
    }
    function applyJoints(J) {
      R.legs.forEach((L, i) => { L.chain.forEach((c, k) => setJ(c.b, J.leg[i][k])); L.q = J.leg[i].slice(); });
      R.arms.forEach((A, i) => A.forEach((b, k) => setJ(b, J.arm[i][k])));
      R.waist.forEach((b, k) => setJ(b, J.waist[k]));
    }
    function blendJoints(A, B, t) {
      const mix = (a, b) => a.map((v, i) => lerp(v, b[i], t));
      return { leg: A.leg.map((a, i) => mix(a, B.leg[i])), arm: A.arm.map((a, i) => mix(a, B.arm[i])), waist: mix(A.waist, B.waist) };
    }
    function ikLeg(L, tp, tR, iters) {
      const q = L.q, ch = L.chain, n = 6, WR = 0.6, LAM2 = 4e-4;
      const jp = [], w = [];
      for (let it = 0; it < iters; it++) {
        let Rm = I3(), p = [0, 0, 0];
        for (let i = 0; i < n; i++) {
          const c = ch[i], d = mv3(Rm, c.p);
          p = [p[0] + d[0], p[1] + d[1], p[2] + d[2]];
          Rm = mul3(Rm, c.R);
          jp[i] = p; w[i] = mv3(Rm, c.a);
          Rm = mul3(Rm, axisAng(c.a, q[i]));
        }
        const ep = [tp[0] - p[0], tp[1] - p[1], tp[2] - p[2]];
        const er = logMap(mul3(tR, tr3(Rm)));
        if (Math.hypot(ep[0], ep[1], ep[2]) + 0.15 * Math.hypot(er[0], er[1], er[2]) < 1.5e-4) break;
        const e = [ep[0], ep[1], ep[2], er[0] * WR, er[1] * WR, er[2] * WR];
        const J = new Array(36);
        for (let i = 0; i < n; i++) {
          const r = [p[0] - jp[i][0], p[1] - jp[i][1], p[2] - jp[i][2]], lv = cross(w[i], r);
          J[i] = lv[0]; J[n + i] = lv[1]; J[2 * n + i] = lv[2];
          J[3 * n + i] = w[i][0] * WR; J[4 * n + i] = w[i][1] * WR; J[5 * n + i] = w[i][2] * WR;
        }
        const A = new Array(36).fill(0), g = [0, 0, 0, 0, 0, 0];
        for (let i = 0; i < n; i++) {
          for (let j = i; j < n; j++) {
            let s = 0;
            for (let r = 0; r < 6; r++) s += J[r * n + i] * J[r * n + j];
            A[i * n + j] = s; A[j * n + i] = s;
          }
          A[i * n + i] += LAM2;
          let s = 0;
          for (let r = 0; r < 6; r++) s += J[r * n + i] * e[r];
          g[i] = s;
        }
        const dq = solve6(A, g);
        if (!dq) break;
        for (let i = 0; i < n; i++) q[i] = clamp(q[i] + clamp(dq[i], -0.35, 0.35), ch[i].lo, ch[i].hi);
      }
    }
    const _mat4 = new THREE.Matrix4();
    function setRootRot(Rp) {
      _mat4.set(Rp[0], Rp[1], Rp[2], 0, Rp[3], Rp[4], Rp[5], 0, Rp[6], Rp[7], Rp[8], 0, 0, 0, 0, 1);
      R.root.quaternion.setFromRotationMatrix(_mat4);
    }
    function solveLegs(rootP, Rp, iters) {
      const RpT = tr3(Rp);
      R.legs.forEach((L, i) => {
        const f = S.feet[i];
        const tw = [f.x - rootP[0], f.y - rootP[1], ANKLE_Z + f.lift - rootP[2]];
        const pitch = f.planted ? 0 : -0.18 * Math.sin(Math.PI * clamp(S.swing ? S.swing.t / S.swing.T : 0, 0, 1));
        ikLeg(L, mv3(RpT, tw), mul3(RpT, rotY(pitch)), iters);
        L.chain.forEach((c, k) => setJ(c.b, L.q[k]));
      });
    }
    function pelvisLimit() {
      let zmax = PELVIS_Z;
      for (let i = 0; i < 2; i++) {
        const f = S.feet[i], side = i === 0 ? 1 : -1;
        const dh = Math.hypot(f.x - (S.c[0] + HIP[0]), f.y - (S.c[1] + side * HIP[1]));
        const vz = Math.sqrt(Math.max(0.01, LEG_REACH * LEG_REACH - dh * dh));
        zmax = Math.min(zmax, ANKLE_Z + f.lift + vz - HIP[2]);
      }
      return Math.max(0.5, zmax);
    }
    function armsAndWaist(dt, extraReact) {
      const t = S.time, L = S.feet[0], Rf = S.feet[1];
      const hull = supportHull(), ref = polyCentroid(hull);
      const tgtReact = clamp(Math.hypot(S.xi[0] - ref[0], S.xi[1] - ref[1]) / 0.24 - 0.15, 0, 1) + (extraReact || 0);
      S.react += (clamp(tgtReact, 0, 1) - S.react) * Math.min(1, dt * (tgtReact > S.react ? 10 : 2.5));
      const r = S.react;
      const swingTerm = clamp((L.x - Rf.x) * 1.7, -0.5, 0.5);
      const br = reduceMotion ? 0 : 0.03 * Math.sin(t * 1.15);
      const sideways = clamp((S.xi[1] - ref[1]) / 0.25, -1, 1);
      for (let i = 0; i < 2; i++) {
        const s = i === 0 ? 1 : -1;
        const out = r * (0.75 + 0.35 * Math.max(0, s * sideways));
        const q = [
          0.2 + s * swingTerm + br - 0.55 * r,
          s * (0.18 + out),
          s * 0.0,
          1.1 - 0.5 * r,
          0, 0.05 * Math.sin(t * 0.9 + i), 0];
        R.arms[i].forEach((b, k) => setJ(b, q[k]));
      }
      // waist: turn towards the pointer, lean against acceleration
      if (S.pointer) S.yawT = clamp(Math.atan2(S.pointer[1] - S.c[1], S.pointer[0] - S.c[0]), -0.55, 0.55);
      else if (!reduceMotion) { S.idleLook += dt; S.yawT = 0.22 * Math.sin(S.idleLook * 0.23) * Math.sin(S.idleLook * 0.11 + 1); }
      else S.yawT = 0;
      S.yaw += (S.yawT - S.yaw) * Math.min(1, dt * 3.2);
      setJ(R.waist[0], S.yaw);
      setJ(R.waist[1], clamp(0.022 * S.acc[1], -0.22, 0.22));
      setJ(R.waist[2], clamp(-0.022 * S.acc[0], -0.2, 0.2) + 0.03);
    }

    function pose(dt) {
      if (S.mode === 'stand') {
        const zt = pelvisLimit();
        S.pz += (zt - S.pz) * Math.min(1, dt * 14);
        const rootP = [S.c[0], S.c[1], S.pz];
        const Rp = mul3(rotY(clamp(0.05 * S.v[0] - 0.008 * S.acc[0], -0.12, 0.12)), rotX(clamp(-0.05 * S.v[1] + 0.008 * S.acc[1], -0.12, 0.12)));
        R.root.position.set(rootP[0], rootP[1], rootP[2]);
        setRootRot(Rp);
        solveLegs(rootP, Rp, 6);
        armsAndWaist(dt, S.grab ? 0.25 : 0);
        if (S.cableOut > 0) {
          S.cableOut = Math.max(0, S.cableOut - dt / 0.55);
          drawCable(1 - smooth(1 - S.cableOut));
        } else { cable.visible = false; hook.visible = false; }
      } else if (S.mode === 'fall' || S.mode === 'fallen') {
        const F = S.fall;
        const k = smooth((F.th - F.th0) / (TH_MAX - F.th0 + 1e-6));
        applyJoints(blendJoints(F.q0, poseJoints(POSE_SLUMP), k * 0.85));
        const axis = new THREE.Vector3(-F.u[1], F.u[0], 0);
        fallPivot.quaternion.setFromAxisAngle(axis, F.th - F.th0);
        // keep the body on (not through) the floor
        fallPivot.position.z = 0;
        fallPivot.updateMatrixWorld(true);
        const minZ = lowestPoint();
        if (minZ < 0 || S.mode === 'fallen') fallPivot.position.z = -minZ;
        cable.visible = false; hook.visible = false;
      } else if (S.mode === 'hoist') {
        const Hs = S.hoist, t = Hs.t;
        if (t < Hs.tA) {
          R.root.position.copy(Hs.pos0); R.root.quaternion.copy(Hs.quat0); applyJoints(Hs.q0);
          drawCable(smooth(t / Hs.tA));
        } else if (t < Hs.tB) {
          const e = smooth((t - Hs.tA) / (Hs.tB - Hs.tA));
          R.root.position.set(lerp(Hs.pos0.x, HOME[0], e), lerp(Hs.pos0.y, HOME[1], e), lerp(Hs.pos0.z, 0.95, smooth(e * 1.4)));
          R.root.quaternion.copy(Hs.quat0).slerp(new THREE.Quaternion(), e);
          applyJoints(blendJoints(Hs.q0, poseJoints(POSE_HANG), e));
          drawCable(1);
        } else {
          const e = smooth((t - Hs.tB) / (Hs.tC - Hs.tB));
          const z = lerp(0.95, PELVIS_Z, e);
          R.root.position.set(HOME[0], HOME[1], z); R.root.quaternion.identity();
          S.c = [HOME[0], HOME[1]]; S.pz = z;
          const hang = poseJoints(POSE_HANG);
          solveLegs([HOME[0], HOME[1], z], I3(), 8);
          const ik = snapshotJoints();
          const J = blendJoints(hang, ik, smooth(e * 1.25));
          J.arm = hang.arm; J.waist = hang.waist;
          applyJoints(J);
          R.legs.forEach((L, i) => { L.q = ik.leg[i]; });
          drawCable(1);
        }
      }
    }
    function drawCable(amount) {
      // amount 0 = fully retracted above the view, 1 = attached to the robot
      R.root.updateMatrixWorld(true);
      const torso = R.byName.torso_link.jg;
      const p = R.attach.clone();
      torso.localToWorld(p); world.worldToLocal(p);
      const top = 3.2, bottom = lerp(top, p.z, amount);
      cable.visible = amount > 0.01; hook.visible = amount > 0.98;
      cable.position.set(p.x, p.y, (top + bottom) / 2);
      cable.scale.set(1, Math.max(0.001, top - bottom), 1);
      cable.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
      hook.position.set(p.x, p.y, p.z + 0.01);
      hook.quaternion.setFromAxisAngle(new THREE.Vector3(1, 0, 0), Math.PI / 2);
    }

    function drawOverlay() {
      const show = S.mode === 'stand' && !opts.bare;
      overlay.visible = show;
      if (!show) return;
      const H = supportHull();
      const a = hullGeo.attributes.position;
      for (let i = 0; i < 12; i++) { const p = H[Math.min(i, H.length - 1)]; a.setXYZ(i, p[0], p[1], 0); }
      hullGeo.setDrawRange(0, H.length); a.needsUpdate = true;
      const fa = hullFillGeo.attributes.position, cen = polyCentroid(H);
      let k = 0;
      for (let i = 0; i < H.length; i++) {
        const p = H[i], q = H[(i + 1) % H.length];
        fa.setXYZ(k++, cen[0], cen[1], 0); fa.setXYZ(k++, p[0], p[1], 0); fa.setXYZ(k++, q[0], q[1], 0);
      }
      hullFillGeo.setDrawRange(0, k); fa.needsUpdate = true;
      comDot.position.set(S.c[0], S.c[1], 0.0005);
      xiRing.position.set(S.xi[0], S.xi[1], 0.001);
      const outside = closestIn(H, S.xi)[2] > 0.001;
      const sc = outside ? 1.25 : 1; xiRing.scale.set(sc, sc, 1);
      const la = linkGeo.attributes.position;
      la.setXYZ(0, S.c[0], S.c[1], 0); la.setXYZ(1, S.xi[0], S.xi[1], 0); la.needsUpdate = true;
    }

    // ------------------------------------------------------- camera & size
    const camTarget = new THREE.Vector3(0, 0.6, 0);
    const camDir = new THREE.Vector3(2.75, 0.62, -0.95).normalize(); // three.js coords (x, y up, z = -mujoco y)
    let camDist = 3.2, follow = new THREE.Vector2(0, 0); const _fp = new THREE.Vector3();
    function resize() {
      const w = Math.max(1, container.clientWidth), h = Math.max(1, container.clientHeight);
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      const tanH = Math.tan(THREE.MathUtils.degToRad(camera.fov / 2));
      camDist = Math.max(1.72 / (2 * tanH), 1.6 / (2 * tanH * camera.aspect));
      camera.updateProjectionMatrix();
    }
    function placeCamera(dt) {
      let fx = S.c[0], fy = S.c[1];
      if (S.mode !== 'stand' && R) { R.root.getWorldPosition(_fp); fx = _fp.x; fy = -_fp.z; }
      follow.x += (fx * 0.75 - follow.x) * Math.min(1, dt * 1.6);
      follow.y += (fy * 0.75 - follow.y) * Math.min(1, dt * 1.6);
      const tx = follow.x, tz = -follow.y;
      camTarget.set(tx, 0.62, tz);
      camera.position.copy(camDir).multiplyScalar(camDist).add(camTarget);
      camera.lookAt(camTarget);
      key.position.set(tx + 1.6, 5.6, tz + 1.9); key.target.position.set(tx, 0, tz); key.target.updateMatrixWorld();
    }

    // ------------------------------------------------------------ input
    const ray = new THREE.Raycaster(), ndc = new THREE.Vector2();
    const toMJ = (v) => new THREE.Vector3(v.x, -v.z, v.y);
    function setNDC(e) {
      const r = canvas.getBoundingClientRect();
      ndc.set(((e.clientX - r.left) / r.width) * 2 - 1, -((e.clientY - r.top) / r.height) * 2 + 1);
      ray.setFromCamera(ndc, camera);
    }
    function hitRobot() {
      if (!R || S.mode !== 'stand') return null;
      const hits = ray.intersectObjects(R.meshes, false);
      return hits.length ? hits[0] : null;
    }
    function rayToHeight(zMJ) {
      const p = new THREE.Vector3();
      return ray.ray.intersectPlane(new THREE.Plane(new THREE.Vector3(0, 1, 0), -zMJ), p) ? toMJ(p) : null;
    }
    function endGrab() {
      S.grab = null; S.aext = [0, 0]; dragLine.visible = false; dragTip.visible = false;
      container.classList.remove('is-grabbing');
    }
    canvas.addEventListener('pointerdown', (e) => {
      if (e.button !== undefined && e.button !== 0) return;
      setNDC(e);
      const hit = hitRobot();
      if (!hit) return;
      const jg = hit.object.parent, local = jg.worldToLocal(hit.point.clone());
      const pMJ = toMJ(hit.point);
      S.grab = { jg, local, z: pMJ.z, x0: e.clientX, y0: e.clientY, t0: performance.now(), target: pMJ.clone(), hist: [], moved: false, id: e.pointerId };
      try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
      container.classList.add('is-grabbing');
      e.preventDefault();
    });
    canvas.addEventListener('touchstart', (e) => { if (S.grab) e.preventDefault(); }, { passive: false });
    canvas.addEventListener('pointermove', (e) => {
      setNDC(e);
      const g = S.grab;
      if (g && g.id === e.pointerId) {
        if (Math.hypot(e.clientX - g.x0, e.clientY - g.y0) > 6) g.moved = true;
        const p = rayToHeight(g.z);
        if (p) {
          g.target = p;
          const now = performance.now();
          g.hist.push([now, p.x, p.y]);
          while (g.hist.length > 2 && now - g.hist[0][0] > 90) g.hist.shift();
        }
        return;
      }
      const head = rayToHeight(1.05);
      S.pointer = head ? [head.x, head.y] : null;
      container.classList.toggle('is-hover', !!hitRobot());
    });
    function release(e) {
      const g = S.grab;
      if (!g || (e && g.id !== e.pointerId)) return;
      const dt = performance.now() - g.t0;
      if (!g.moved && dt < 300) {
        // tap: push away from the viewer, harder the higher it lands
        const d = toMJ(ray.ray.direction.clone()); const n = Math.hypot(d.x, d.y) || 1;
        const m = 0.5 + 0.8 * clamp((g.z - 0.3) / 0.85, 0, 1);
        api.push((d.x / n) * m, (d.y / n) * m);
      } else if (g.hist.length >= 2) {
        const a = g.hist[0], b = g.hist[g.hist.length - 1], el = (b[0] - a[0]) / 1000;
        if (el > 0.01) {
          let vx = (b[1] - a[1]) / el, vy = (b[2] - a[2]) / el; const sp = Math.hypot(vx, vy), cap = 2.3;
          if (sp > cap) { vx *= cap / sp; vy *= cap / sp; }
          api.push(vx * 0.5, vy * 0.5);
        }
      }
      endGrab();
    }
    canvas.addEventListener('pointerup', release);
    canvas.addEventListener('pointercancel', () => endGrab());
    canvas.addEventListener('pointerleave', (e) => { if (!S.grab) { S.pointer = null; container.classList.remove('is-hover'); } });
    function updateGrab() {
      const g = S.grab;
      if (!g) return;
      const p = g.jg.localToWorld(g.local.clone()); const pm = toMJ(p);
      const dx = g.target.x - pm.x, dy = g.target.y - pm.y;
      let ax = 16 * dx, ay = 16 * dy; const am = Math.hypot(ax, ay);
      if (am > 12) { ax *= 12 / am; ay *= 12 / am; }
      S.aext = [ax, ay];
      const la = dragGeo.attributes.position;
      la.setXYZ(0, pm.x, pm.y, pm.z); la.setXYZ(1, g.target.x, g.target.y, g.target.z); la.needsUpdate = true;
      dragGeo.computeBoundingSphere();
      dragTip.position.set(g.target.x, g.target.y, g.target.z);
      dragLine.visible = true; dragTip.visible = true;
    }

    // --------------------------------------------------------------- loop
    let raf = 0, last = 0, running = false, visible = true, onScreen = true, ro = null, io = null;
    let paused = false;
    function tick(dt, draw) {
      updateGrab();
      let acc = dt;
      while (acc > 1e-6) { const h = Math.min(1 / 240, acc); physics(h); acc -= h; }
      pose(dt);
      drawOverlay();
      placeCamera(dt);
      if (draw) renderer.render(scene, camera);
    }
    function loop(now) {
      raf = requestAnimationFrame(loop);
      const dt = Math.min(0.05, Math.max(0, (now - last) / 1000)); last = now;
      if (!R || !visible || !onScreen || paused) return;
      tick(dt, true);
    }
    function start() { if (running) return; running = true; last = performance.now(); raf = requestAnimationFrame(loop); }
    // deterministic stepping, used for testing and for rendering a still poster
    api.pause = () => { paused = true; };
    api.resume = () => { paused = false; last = performance.now(); };
    api.advance = (sec) => { const n = Math.round(sec * 60); for (let i = 0; i < n; i++) tick(1 / 60, i === n - 1); };

    readTheme();
    if (opts.bare) { grid.visible = false; overlay.visible = false; }
    resize();
    if (window.ResizeObserver) { ro = new ResizeObserver(resize); ro.observe(container); } else window.addEventListener('resize', resize);
    if (window.IntersectionObserver) { io = new IntersectionObserver((en) => { onScreen = en[0].isIntersecting; }, { rootMargin: '80px' }); io.observe(container); }
    document.addEventListener('visibilitychange', () => { visible = document.visibilityState !== 'hidden'; last = performance.now(); });
    const mq = window.matchMedia ? window.matchMedia('(prefers-color-scheme: dark)') : null;
    if (mq && mq.addEventListener) mq.addEventListener('change', readTheme);
    new MutationObserver(readTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme', 'class'] });

    return loadModel(opts.modelUrl || 'assets/models/g1.json').then(({ meta, geoms }) => {
      R = build(meta, geoms);
      resetStand();
      if (opts.intro === false || reduceMotion) { solveLegs([S.c[0], S.c[1], S.pz], I3(), 30); }
      else { applyJoints(poseJoints(POSE_HANG)); startHoist(true); }
      placeCamera(1);
      pose(0.016);
      renderer.render(scene, camera);
      start();
      api.ready = true; api.dbg = { overlay, xiRing, comDot, hullLine, scene, camera, world, cable, hook, R };
      return api;
    });
  }

  window.G1Hero = { mount };
})();
