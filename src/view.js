// three.js rendering of a 4D World.
//
// We live in a 3D slice of the 4D world. The view matrix V rotates world
// coordinates into view coordinates (x′, y′, z′, w′); the solid geometry is the
// cross-section w′ = s0, drawn in (x′, y′, z′). Gravity is along y, so the view
// only tilts in the xw and zw planes and the floor always stays flat.
//
// Each body draws:
//   • its slice: a hypersphere cuts to a sphere, a hyperbox to a convex
//     polyhedron, one polygon per cubic facet it crosses, coloured by facet;
//   • optionally its shadow: the whole body projected along w′, tinted warm
//     where it lies at +w′ of the slice and cool where it lies at −w′.

import * as THREE from '../vendor/three/three.module.js';
import { OrbitControls } from '../vendor/three/addons/controls/OrbitControls.js';
import { mPlane, mMul, mApply, mApplyT, sub, addScaled, dot, len } from './math4.js';

// facet tints by body axis (x, y, z, w)
const AXIS_TINT = [0xe0684f, 0x67b26a, 0x5b8fd9, 0xd9a33f];
export const WARM = new THREE.Color(0xf0a040), COOL = new THREE.Color(0x40b8e8), MID = new THREE.Color(0xc8ccd6);

export function wColor(d, out = new THREE.Color(), range = 1) {
  const t = Math.tanh(d / range);
  return t >= 0 ? out.copy(MID).lerp(WARM, t) : out.copy(MID).lerp(COOL, -t);
}

export class View {
  constructor(canvas) {
    this.canvas = canvas;
    const r = this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: false });
    r.setPixelRatio(Math.min(2, devicePixelRatio));
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(45, 1, 0.05, 300);
    this.camera.position.set(5, 3.5, 7);
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0.6, 0);

    this.scene.add(new THREE.HemisphereLight(0xe6ecff, 0x3a3630, 1.5));
    const sun = new THREE.DirectionalLight(0xffffff, 1.9);
    sun.position.set(4, 9, 6);
    this.scene.add(sun);
    const fill = new THREE.DirectionalLight(0xbfd4ff, 0.5);
    fill.position.set(-6, 3, -4);
    this.scene.add(fill);

    this.s0 = 0;
    this.axw = 0; this.azw = 0;
    this.V = mMul(mPlane(2, 3, 0), mPlane(0, 3, 0));
    this.showGhosts = true;
    this.showTrails = true;
    this.selected = null;
    this.items = [];
    this.root = new THREE.Group();
    this.scene.add(this.root);

    this.floor = this.makeFloor();
    this.scene.add(this.floor);

    this.sphereGeo = new THREE.IcosahedronGeometry(1, 4);
    this.tmpColor = new THREE.Color();

    this.raycaster = new THREE.Raycaster();
    this.onPick = null;
    let down = null;
    canvas.addEventListener('pointerdown', e => { down = { x: e.clientX, y: e.clientY }; });
    canvas.addEventListener('pointerup', e => {
      if (!down || Math.hypot(e.clientX - down.x, e.clientY - down.y) > 5) return;
      down = null;
      const rc = canvas.getBoundingClientRect();
      const ndc = new THREE.Vector2(((e.clientX - rc.left) / rc.width) * 2 - 1, -((e.clientY - rc.top) / rc.height) * 2 + 1);
      this.raycaster.setFromCamera(ndc, this.camera);
      const meshes = this.items.filter(i => i.slice.visible && !i.body.static).map(i => i.slice);
      const hit = this.raycaster.intersectObjects(meshes, false)[0];
      const it = hit && this.items.find(i => i.slice === hit.object);
      this.onPick?.(it ? it.body : null);
    });
  }

  setTheme(dark) {
    this.scene.background = new THREE.Color(dark ? 0x101318 : 0xe9ecf1);
    this.scene.fog = new THREE.Fog(this.scene.background, 18, 60);
    const u = this.floor.material.uniforms;
    u.colA.value.set(dark ? 0x232933 : 0xc4ccd7);
    u.colB.value.set(dark ? 0x353d4a : 0xdfe4ea);
    u.fogCol.value.copy(this.scene.background);
  }

  setTilt(axw, azw) {
    this.axw = axw; this.azw = azw;
    this.V = mMul(mPlane(2, 3, azw), mPlane(0, 3, axw));
  }
  // world point → [x′, y′, z′, w′]
  toView(p) { return mApply(this.V, p); }
  fromView(q) { return mApplyT(this.V, q); }

  makeFloor() {
    const mat = new THREE.ShaderMaterial({
      uniforms: {
        Vt: { value: new THREE.Matrix4() }, s0: { value: 0 },
        colA: { value: new THREE.Color(0x1d222b) }, colB: { value: new THREE.Color(0x262c37) },
        fogCol: { value: new THREE.Color(0x101318) },
      },
      vertexShader: `varying vec3 vPos; void main(){ vPos = position.xzy * vec3(1.0, 1.0, -1.0);
        gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0); }`,
      // the floor is the 3D hyperplane y = 0; our slice of it is a 2D plane.
      // The checker is a 4D one (1 m cells in x, z and w), so sliding the slice
      // through w shifts the pattern.
      fragmentShader: `uniform mat4 Vt; uniform float s0; uniform vec3 colA, colB, fogCol; varying vec3 vPos;
        void main(){
          vec4 p = Vt * vec4(vPos.x, 0.0, vPos.z, s0);
          float c = mod(floor(p.x) + floor(p.z) + floor(p.w), 2.0);
          vec3 col = mix(colA, colB, c);
          float f = smoothstep(14.0, 48.0, length(vPos.xz - cameraPosition.xz));
          gl_FragColor = vec4(mix(col, fogCol, f), 1.0);
        }`,
    });
    const m = new THREE.Mesh(new THREE.PlaneGeometry(200, 200), mat);
    m.rotation.x = -Math.PI / 2;
    return m;
  }

  setWorld(world) {
    this.world = world;
    for (const it of this.items) this.disposeItem(it);
    this.items = [];
    this.root.clear();
    for (const b of world.bodies) {
      if (b.shape.type === 'plane') continue;
      this.items.push(this.makeItem(b));
    }
    this.floor.visible = !!world.ground;
    // joints and anchors
    this.jointLines = new THREE.LineSegments(new THREE.BufferGeometry(),
      new THREE.LineBasicMaterial({ color: 0x9aa3b5, transparent: true, opacity: 0.8 }));
    this.jointLines.frustumCulled = false;
    this.root.add(this.jointLines);
    this.anchorGeo = new THREE.SphereGeometry(0.045, 12, 8);
    this.anchors = [];
  }

  makeItem(b) {
    const transparent = !!b.user.glass;
    const mat = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.55, metalness: 0.05, side: THREE.DoubleSide,
      transparent, opacity: transparent ? 0.28 : 1, depthWrite: !transparent, flatShading: !!b.hull });
    let slice;
    if (b.shape.type === 'sphere') {
      const g = this.sphereGeo.clone();
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(g.attributes.position.count * 3), 3));
      slice = new THREE.Mesh(g, mat);
    } else {
      const g = new THREE.BufferGeometry();
      // each cell's slice is a polygon with at most one point per cell edge → fan triangles
      const max = b.hull.facetEdges.reduce((s, e) => s + Math.max(0, e.length - 2), 0) * 3;
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
      g.setAttribute('normal', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(max * 3), 3));
      slice = new THREE.Mesh(g, mat);
      slice.frustumCulled = false;
    }
    this.root.add(slice);

    // shadow (projection of the whole body)
    let ghost;
    if (b.shape.type === 'sphere') {
      ghost = new THREE.Mesh(this.sphereGeo, new THREE.MeshBasicMaterial({ color: 0xffffff, transparent: true, opacity: 0.1, depthWrite: false, wireframe: true }));
    } else {
      const g = new THREE.BufferGeometry();
      const ne = b.hull.edges.length;
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(ne * 2 * 3), 3));
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(ne * 2 * 3), 3));
      ghost = new THREE.LineSegments(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: b.static ? 0.12 : 0.55, depthWrite: false }));
      ghost.frustumCulled = false;
    }
    this.root.add(ghost);

    // trail
    let trail = null;
    if (b.trail || b.user.marker) {
      const n = b.user.trailLength ?? 900, g = new THREE.BufferGeometry();
      g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
      g.setAttribute('color', new THREE.BufferAttribute(new Float32Array(n * 3), 3));
      g.setDrawRange(0, 0);
      trail = new THREE.Line(g, new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity: 0.85 }));
      trail.frustumCulled = false;
      this.root.add(trail);
      b.user.history = b.user.history ?? [];
    }
    let marker = null;
    if (b.user.marker) {
      marker = new THREE.Mesh(new THREE.SphereGeometry(0.06, 12, 8), new THREE.MeshBasicMaterial({ color: 0xffffff }));
      this.root.add(marker);
    }
    return { body: b, slice, ghost, trail, marker, base: new THREE.Color(b.color) };
  }

  disposeItem(it) {
    it.slice.geometry.dispose(); it.slice.material.dispose();
    if (it.ghost.geometry !== this.sphereGeo) it.ghost.geometry.dispose();
    it.ghost.material.dispose();
    it.trail?.geometry.dispose();
  }

  // record trail points (call once per physics step)
  recordTrails() {
    for (const it of this.items) {
      if (!it.trail) continue;
      const b = it.body, h = b.user.history;
      h.push(b.user.marker ? b.toWorld(b.user.marker) : b.x.slice());
      const n = b.user.trailLength ?? 900;
      if (h.length > n) h.splice(0, h.length - n);
    }
  }

  render() {
    const Vt = this.floor.material.uniforms.Vt.value, V = this.V;
    // Vt = Vᵀ as a three.js matrix (set() takes row-major)
    Vt.set(V[0], V[4], V[8], V[12], V[1], V[5], V[9], V[13], V[2], V[6], V[10], V[14], V[3], V[7], V[11], V[15]);
    this.floor.material.uniforms.s0.value = this.s0;
    for (const it of this.items) {
      if (it.body.shape.type === 'sphere') this.drawSphere(it); else this.drawBox(it);
      if (it.trail) this.drawTrail(it);
    }
    this.drawJoints();
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  highlight(it, c) {
    if (this.selected && it.body === this.selected) c.lerp(this.tmpColor.set(0xffffff), 0.35);
    return c;
  }

  drawSphere(it) {
    const b = it.body, r = b.shape.r, q = this.toView(b.x), d = q[3] - this.s0;
    const m = it.slice;
    m.visible = Math.abs(d) < r;
    if (m.visible) {
      const rho = Math.sqrt(r * r - d * d);
      m.position.set(q[0], q[1], q[2]);
      m.scale.setScalar(rho);
      // colour by the body-frame orthant (16 of them, two tones): this makes
      // the 4D rotation of a round body visible on its slice
      const pos = m.geometry.attributes.position, col = m.geometry.attributes.color;
      const base = it.base, dark = this.tmpColor.copy(base).multiplyScalar(0.42), c = new THREE.Color();
      for (let i = 0; i < pos.count; i++) {
        const u = [pos.getX(i), pos.getY(i), pos.getZ(i)];
        const p = this.fromView([q[0] + rho * u[0], q[1] + rho * u[1], q[2] + rho * u[2], this.s0]);
        const s = b.toLocal(p);
        const t = Math.max(-1, Math.min(1, 6 * s[0] * s[1] * s[2] * s[3] / (r * r * r * r) * 8));
        c.copy(dark).lerp(base, 0.5 + 0.5 * Math.sign(t) * Math.sqrt(Math.abs(t)));
        this.highlight(it, c);
        col.setXYZ(i, c.r, c.g, c.b);
      }
      col.needsUpdate = true;
    }
    const g = it.ghost;
    g.visible = this.showGhosts && Math.abs(d) > 0.02;
    if (g.visible) {
      g.position.set(q[0], q[1], q[2]);
      g.scale.setScalar(r);
      wColor(d, g.material.color);
      g.material.opacity = 0.16;
    }
  }

  drawBox(it) {
    const b = it.body, vs = b.verts.map(p => this.toView(p)), s0 = this.s0;
    const sd = vs.map(q => q[3] - s0);
    const g = it.slice.geometry, P = g.attributes.position, N = g.attributes.normal, C = g.attributes.color;
    let nv = 0;
    // crossing point on edge (i, j) if the slice passes between them
    const cross = (i, j) => {
      const a = sd[i], c = sd[j];
      if ((a < 0) === (c < 0)) return null;
      const t = a / (a - c), p = vs[i], q = vs[j];
      return [p[0] + (q[0] - p[0]) * t, p[1] + (q[1] - p[1]) * t, p[2] + (q[2] - p[2]) * t];
    };
    const col = new THREE.Color(), H = b.hull, tint = this.cellTints(it);
    for (let f = 0; f < H.normals.length; f++) {
      // the cell's slice: one point where the slice crosses each of its edges
      const pts = [];
      for (const k of H.facetEdges[f]) { const p = cross(H.edges[k][0], H.edges[k][1]); if (p) pts.push(p); }
      if (pts.length < 3) continue;
      const nw = mApply(this.V, b.fn[f]);
      let n = [nw[0], nw[1], nw[2]];
      const nl = Math.hypot(...n);
      if (nl < 1e-6) continue;
      n = n.map(x => x / nl);
      const c = [0, 1, 2].map(a => pts.reduce((s, p) => s + p[a], 0) / pts.length);
      let e1 = sub3(pts[0], c);
      e1 = sub3(e1, sc3(n, dot3(e1, n)));
      const l1 = Math.hypot(...e1) || 1; e1 = sc3(e1, 1 / l1);
      const e2 = cr3(n, e1);
      pts.sort((p, q) => Math.atan2(dot3(sub3(p, c), e2), dot3(sub3(p, c), e1)) - Math.atan2(dot3(sub3(q, c), e2), dot3(sub3(q, c), e1)));
      col.copy(tint[f]);
      this.highlight(it, col);
      for (let i = 1; i + 1 < pts.length; i++) for (const p of [pts[0], pts[i], pts[i + 1]]) {
        if (nv >= P.count) break;
        P.setXYZ(nv, p[0], p[1], p[2]); N.setXYZ(nv, n[0], n[1], n[2]); C.setXYZ(nv, col.r, col.g, col.b); nv++;
      }
    }
    g.setDrawRange(0, nv);
    P.needsUpdate = N.needsUpdate = C.needsUpdate = true;
    g.computeBoundingSphere();
    it.slice.visible = nv > 0;

    const gh = it.ghost;
    gh.visible = this.showGhosts;
    if (gh.visible) {
      const GP = gh.geometry.attributes.position, GC = gh.geometry.attributes.color;
      let e = 0;
      for (const ij of H.edges) {
        for (const v of ij) {
          GP.setXYZ(e, vs[v][0], vs[v][1], vs[v][2]);
          wColor(sd[v], col, 0.8);
          GC.setXYZ(e, col.r, col.g, col.b);
          e++;
        }
      }
      GP.needsUpdate = GC.needsUpdate = true;
    }
  }

  // each cell's colour, from its normal in the body frame: x red, y green,
  // z blue, w amber, mixed by the squared components; darker for the negative side
  cellTints(it) {
    if (it.tints) return it.tints;
    const b = it.body, axis = AXIS_TINT.map(c => new THREE.Color(c)), k = b.static ? 0.12 : 0.38;
    return it.tints = b.hull.normals.map(n => {
      const mix = new THREE.Color(0, 0, 0);
      let a = 0;
      for (let i = 0; i < 4; i++) { mix.add(axis[i].clone().multiplyScalar(n[i] * n[i])); if (Math.abs(n[i]) > Math.abs(n[a])) a = i; }
      const c = it.base.clone().lerp(mix, k);
      return n[a] < 0 ? c.multiplyScalar(0.72) : c;
    });
  }

  drawTrail(it) {
    const h = it.body.user.history, g = it.trail.geometry, P = g.attributes.position, C = g.attributes.color, col = new THREE.Color();
    const n = Math.min(h.length, P.count);
    for (let i = 0; i < n; i++) {
      const q = this.toView(h[h.length - n + i]);
      P.setXYZ(i, q[0], q[1], q[2]);
      wColor(q[3] - this.s0, col, 0.8);
      const fade = 0.25 + 0.75 * i / n;
      C.setXYZ(i, col.r * fade, col.g * fade, col.b * fade);
    }
    g.setDrawRange(0, n);
    P.needsUpdate = C.needsUpdate = true;
    it.trail.visible = this.showTrails;
    if (it.marker) {
      const b = it.body, q = this.toView(b.toWorld(b.user.marker));
      it.marker.position.set(q[0], q[1], q[2]);
      wColor(q[3] - this.s0, it.marker.material.color, 0.8);
      it.marker.visible = this.showTrails;
    }
  }

  drawJoints() {
    const w = this.world, pts = [];
    const used = [];
    for (const j of w.joints) {
      const [pa] = j.anchors(), qa = this.toView(pa);
      for (const B of [j.a, j.b]) {
        if (B === w.anchor) continue;
        const qb = this.toView(B.x);
        pts.push(qa[0], qa[1], qa[2], qb[0], qb[1], qb[2]);
      }
      used.push(qa);
    }
    const g = this.jointLines.geometry;
    g.setAttribute('position', new THREE.BufferAttribute(new Float32Array(pts), 3));
    while (this.anchors.length < used.length) {
      const m = new THREE.Mesh(this.anchorGeo, new THREE.MeshBasicMaterial({ color: 0xffffff }));
      this.root.add(m); this.anchors.push(m);
    }
    this.anchors.forEach((m, i) => {
      m.visible = i < used.length;
      if (m.visible) { const q = used[i]; m.position.set(q[0], q[1], q[2]); wColor(q[3] - this.s0, m.material.color, 0.5); }
    });
  }

  resize(w, h) {
    this.renderer.setSize(w, h, false);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
  }
}

const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const sc3 = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cr3 = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
