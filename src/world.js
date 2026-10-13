// A small 4D rigid-body engine.
//
// Bodies carry position x (4-vector), orientation R (4×4 rotation, body → world),
// velocity v and angular velocity w. In 4D, w is a bivector: six components,
// one per rotation plane (xy xz xw yz yw zw). The body-frame inertia is
// diagonal on that basis: J_ab = M_a + M_b where M_a = ∫ρ x_a² is the second
// moment along axis a. Angular momentum L = I w is a bivector too, and it is
// what free motion conserves, so the integrator carries L through each rotation
// step and recomputes w from it (that is what produces precession, the
// "tennis racket" flips and the like).
//
// Each frame is split into substeps. A substep: gravity and force fields →
// collision detection → sequential impulses on velocities (contacts with
// Coulomb friction in the 3D tangent space, joints, motors) → a separate
// pseudo-velocity pass that pushes penetrations and joint drift apart without
// adding kinetic energy → position and orientation update.

import {
  add, sub, scale, addScaled, dot, len, normalize, wedge, bapply, bzero, badd, bsub, bscale, bdot,
  mIdentity, mApply, mApplyT, mCol, cayleyStep, orthonormalize, compound, m6Apply, solveN, solve4,
} from './math4.js';
import { collidePair } from './collide.js';

let nextId = 1;

export class Body {
  constructor(o = {}) {
    this.id = nextId++;
    this.name = o.name ?? '';
    this.shape = o.shape ?? { type: 'sphere', r: 0.5 };
    this.x = (o.pos ?? [0, 0, 0, 0]).slice();
    this.R = o.rot ? Float64Array.from(o.rot) : mIdentity();
    this.v = (o.vel ?? [0, 0, 0, 0]).slice();
    this.w = (o.omega ?? bzero()).slice();
    this.static = !!o.static || this.shape.type === 'plane';
    this.friction = o.friction ?? 0.5;
    this.restitution = o.restitution ?? 0.2;
    this.gravityScale = o.gravityScale ?? 1;
    this.collide = o.collide ?? true;
    this.color = o.color ?? 0xd0d4dc;
    this.trail = o.trail ?? false;
    this.user = o.user ?? {};
    this.vp = [0, 0, 0, 0]; this.wp = bzero(); // pseudo velocities (position correction)

    const s = this.shape;
    if (this.static) {
      this.mass = Infinity; this.invMass = 0;
      this.J = [0, 0, 0, 0, 0, 0]; this.Jinv = [0, 0, 0, 0, 0, 0];
    } else {
      const vol = s.type === 'sphere' ? Math.PI ** 2 * s.r ** 4 / 2 : 16 * s.h[0] * s.h[1] * s.h[2] * s.h[3];
      this.mass = o.mass ?? (o.density ?? 1000) * vol;
      this.invMass = 1 / this.mass;
      // second moments M_a = ∫ρ x_a² (per axis)
      const M = s.type === 'sphere'
        ? Array(4).fill(this.mass * s.r * s.r / (s.hollow ? 4 : 6))
        : s.h.map(h => this.mass * h * h / 3);
      this.M = M;
      this.J = PAIRS6.map(([a, b]) => M[a] + M[b]);
      this.Jinv = this.J.map(j => 1 / j);
    }
    this.radius = s.type === 'sphere' ? s.r : s.type === 'box' ? len(s.h) : Infinity;
    this.update();
  }

  // derived world quantities: axes, inverse inertia, box corners
  update() {
    this.axes = [0, 1, 2, 3].map(j => mCol(this.R, j));
    if (!this.static) {
      const C = this.C = compound(this.R), I = new Float64Array(36);
      for (let i = 0; i < 6; i++) for (let j = 0; j < 6; j++) {
        let s = 0; for (let k = 0; k < 6; k++) s += C[i * 6 + k] * this.Jinv[k] * C[j * 6 + k];
        I[i * 6 + j] = s;
      }
      this.invI = I;
    } else this.invI = ZERO36;
    if (this.shape.type === 'box') {
      const h = this.shape.h;
      this.verts = [];
      for (let b = 0; b < 16; b++) {
        let p = this.x.slice();
        for (let k = 0; k < 4; k++) p = addScaled(p, this.axes[k], (b >> k & 1 ? 1 : -1) * h[k]);
        this.verts.push(p);
      }
    }
  }

  // angular momentum about the centre of mass: L = C J Cᵀ w
  angMom(w = this.w) {
    if (this.static) return bzero();
    const Cb = m6ApplyT(this.C, w);
    return m6Apply(this.C, Cb.map((x, k) => x * this.J[k]));
  }
  // angular velocity in body coordinates (Cᵀ w)
  bodyOmega() { return this.static ? bzero() : m6ApplyT(this.C, this.w); }
  velAt(r) { return add(this.v, bapply(this.w, r)); }
  pvelAt(r) { return add(this.vp, bapply(this.wp, r)); }
  impulse(P, r) {
    if (this.static) return;
    this.v = addScaled(this.v, P, this.invMass);
    this.w = badd(this.w, m6Apply(this.invI, wedge(r, P)));
  }
  pimpulse(P, r) {
    if (this.static) return;
    this.vp = addScaled(this.vp, P, this.invMass);
    this.wp = badd(this.wp, m6Apply(this.invI, wedge(r, P)));
  }
  angImpulse(Lb) { if (!this.static) this.w = badd(this.w, m6Apply(this.invI, Lb)); }
  pangImpulse(Lb) { if (!this.static) this.wp = badd(this.wp, m6Apply(this.invI, Lb)); }
  toWorld(s) { return add(this.x, mApply(this.R, s)); }
  toLocal(p) { return mApplyT(this.R, sub(p, this.x)); }
  kinetic() {
    if (this.static) return { lin: 0, rot: 0 };
    return { lin: 0.5 * this.mass * dot(this.v, this.v), rot: 0.5 * bdot(this.w, this.angMom()) };
  }
}

const PAIRS6 = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];
const ZERO36 = new Float64Array(36);
function m6ApplyT(M, x) {
  const o = new Array(6);
  for (let i = 0; i < 6; i++) { let s = 0; for (let j = 0; j < 6; j++) s += M[j * 6 + i] * x[j]; o[i] = s; }
  return o;
}

// effective inverse mass of body B for an impulse along unit n at offset r
function kOf(B, r, n) {
  if (B.static) return 0;
  const rn = wedge(r, n);
  return B.invMass + bdot(rn, m6Apply(B.invI, rn));
}

// ---------------------------------------------------------------- joints

// Ball joint: an anchor point shared by two bodies (all rotations free).
export class BallJoint {
  constructor(a, b, anchorWorld) {
    this.a = a; this.b = b;
    this.sa = a.toLocal(anchorWorld); this.sb = b.toLocal(anchorWorld);
    this.collide = false;
  }
  anchors() { return [this.a.toWorld(this.sa), this.b.toWorld(this.sb)]; }
  prepare(h) {
    const [pa, pb] = this.anchors();
    this.ra = sub(pa, this.a.x); this.rb = sub(pb, this.b.x); this.err = sub(pa, pb);
    this.K = pointMass(this.a, this.ra, this.b, this.rb);
    this.h = h;
  }
  solveVel() { pointSolve(this, 'vel', scale(this.err, -1 / this.h)); }
  solvePos() {}
}

// 4×4 effective-mass matrix of a point constraint
function pointMass(A, ra, B, rb) {
  const K = new Float64Array(16);
  for (let j = 0; j < 4; j++) {
    const e = [0, 0, 0, 0]; e[j] = 1;
    let col = scale(e, A.invMass + B.invMass);
    if (!A.static) col = add(col, bapply(m6Apply(A.invI, wedge(ra, e)), ra));
    if (!B.static) col = add(col, bapply(m6Apply(B.invI, wedge(rb, e)), rb));
    for (let i = 0; i < 4; i++) K[i * 4 + j] = col[i];
  }
  return K;
}
function pointSolve(j, mode, bias) {
  const A = j.a, B = j.b, pos = mode === 'pos';
  const rel = pos ? sub(A.pvelAt(j.ra), B.pvelAt(j.rb)) : sub(A.velAt(j.ra), B.velAt(j.rb));
  const rhs = bias ? sub(bias, rel) : scale(rel, -1);
  const X = solve4(j.K, Float64Array.from([rhs[0], 0, 0, 0, rhs[1], 0, 0, 0, rhs[2], 0, 0, 0, rhs[3], 0, 0, 0]));
  const P = [X[0], X[4], X[8], X[12]];
  if (pos) { A.pimpulse(P, j.ra); B.pimpulse(scale(P, -1), j.rb); }
  else { A.impulse(P, j.ra); B.impulse(scale(P, -1), j.rb); }
}

// Hinge joint. In 4D a hinge doesn't turn about a line: it turns in one plane,
// and the whole orthogonal plane through the anchor stays fixed. `plane` is a
// pair of world vectors spanning the turning plane at creation time.
// Optional motor (target speed, max torque) and angle limits.
export class HingeJoint {
  constructor(a, b, anchorWorld, plane, o = {}) {
    this.a = a; this.b = b;
    this.sa = a.toLocal(anchorWorld); this.sb = b.toLocal(anchorWorld);
    const u1 = normalize(plane[0]);
    const u2 = normalize(addScaled(plane[1], u1, -dot(plane[1], u1)));
    const basis = [u1, u2];
    for (let i = 0; i < 4 && basis.length < 4; i++) {
      let e = [0, 0, 0, 0]; e[i] = 1;
      for (const q of basis) e = addScaled(e, q, -dot(e, q));
      if (len(e) > 0.1) basis.push(normalize(e));
    }
    // the hinge frame in each body's coordinates
    this.fa = basis.map(v => mApplyT(a.R, v));
    this.fb = basis.map(v => mApplyT(b.R, v));
    this.motorSpeed = o.motorSpeed ?? 0;
    this.maxTorque = o.maxTorque ?? 0;
    this.lo = o.lo ?? -Infinity; this.hi = o.hi ?? Infinity;
    this.collide = false;
  }
  anchors() { return [this.a.toWorld(this.sa), this.b.toWorld(this.sb)]; }
  // angle of b relative to a in the turning plane
  angle() {
    const A = this.fa.map(v => mApply(this.a.R, v)), Bv = mApply(this.b.R, this.fb[0]);
    return Math.atan2(dot(Bv, A[1]), dot(Bv, A[0]));
  }
  prepare(h) {
    const [pa, pb] = this.anchors();
    this.ra = sub(pa, this.a.x); this.rb = sub(pb, this.b.x); this.err = sub(pa, pb);
    this.K = pointMass(this.a, this.ra, this.b, this.rb);
    const F = this.fa.map(v => mApply(this.a.R, v)), G = this.fb.map(v => mApply(this.b.R, v));
    this.U = wedge(F[0], F[1]); // turning plane, world
    // the five constrained planes, all orthogonal to U
    this.rows = [wedge(F[0], F[2]), wedge(F[0], F[3]), wedge(F[1], F[2]), wedge(F[1], F[3]), wedge(F[2], F[3])];
    const Iab = i => badd(m6Apply(this.a.invI, i), m6Apply(this.b.invI, i));
    this.Kang = this.rows.map(r => this.rows.map(s => bdot(r, Iab(s))));
    this.kU = bdot(this.U, Iab(this.U));
    // orientation error: rotate b's fixed-plane vectors onto a's
    this.angErr = badd(wedge(G[2], F[2]), wedge(G[3], F[3]));
    this.theta = this.angle();
    this.motorAcc = 0; this.limitAcc = 0; this.h = h;
  }
  angRel(pos) { return pos ? bsub(this.a.wp, this.b.wp) : bsub(this.a.w, this.b.w); }
  applyAng(L, pos) {
    if (pos) { this.a.pangImpulse(L); this.b.pangImpulse(bscale(L, -1)); }
    else { this.a.angImpulse(L); this.b.angImpulse(bscale(L, -1)); }
  }
  solveVel() {
    pointSolve(this, 'vel', scale(this.err, -1 / this.h));
    let wr = this.angRel(false);
    const lam = solveN(this.Kang, this.rows.map(r => -bdot(r, this.angErr) / this.h - bdot(r, wr)));
    let L = bzero();
    this.rows.forEach((r, i) => { L = badd(L, bscale(r, lam[i])); });
    this.applyAng(L, false);
    if (this.kU < 1e-12) return;
    // motor: relative speed in the turning plane is -U·(wa - wb)
    if (this.maxTorque > 0) {
      wr = this.angRel(false);
      const speed = -bdot(this.U, wr), max = this.maxTorque * this.h;
      let d = (speed - this.motorSpeed) / this.kU; // impulse along U on a
      const acc = Math.max(-max, Math.min(max, this.motorAcc + d));
      d = acc - this.motorAcc; this.motorAcc = acc;
      this.applyAng(bscale(this.U, d), false);
    }
    if (this.theta <= this.lo || this.theta >= this.hi) {
      wr = this.angRel(false);
      const speed = -bdot(this.U, wr), sg = this.theta <= this.lo ? 1 : -1;
      // keep sg·speed ≥ 0: impulse d along U changes speed by -d·kU
      let d = speed / this.kU;
      const acc = sg > 0 ? Math.min(0, this.limitAcc + d) : Math.max(0, this.limitAcc + d);
      d = acc - this.limitAcc; this.limitAcc = acc;
      this.applyAng(bscale(this.U, d), false);
    }
  }
  solvePos() {}
}

// ---------------------------------------------------------------- world

export class World {
  constructor(o = {}) {
    this.gravity = o.gravity ?? [0, -9.81, 0, 0];
    this.substeps = o.substeps ?? 8;
    this.iterations = o.iterations ?? 10;
    this.posIterations = o.posIterations ?? 4;
    this.margin = 0.002;
    this.slop = 0.001;
    this.beta = 0.25;
    this.restThreshold = o.restThreshold ?? 0.25;
    this.bodies = [];
    this.joints = [];
    this.forces = [];   // functions (world, h) applying extra velocity kicks
    this.time = 0;
    this.contacts = [];
    this.anchor = new Body({ static: true, collide: false, shape: { type: 'point' }, name: 'world' });
    if (o.ground !== false) {
      this.ground = this.add(new Body({ shape: { type: 'plane', n: [0, 1, 0, 0], d: 0 },
        friction: o.groundFriction ?? 0.6, restitution: o.groundRestitution ?? 0.2, name: 'ground' }));
    }
  }
  add(b) { this.bodies.push(b); return b; }
  addJoint(j) { this.joints.push(j); return j; }
  remove(b) {
    this.bodies = this.bodies.filter(x => x !== b);
    this.joints = this.joints.filter(j => j.a !== b && j.b !== b);
  }

  step(dt = 1 / 60) {
    const h = dt / this.substeps;
    for (let s = 0; s < this.substeps; s++) this.substep(h);
  }

  substep(h) {
    const dyn = this.bodies.filter(b => !b.static);
    for (const b of dyn) b.v = addScaled(b.v, this.gravity, b.gravityScale * h);
    for (const f of this.forces) f(this, h);

    const contacts = this.contacts = this.detect();
    for (const c of contacts) this.prepareContact(c);
    this.warmStart(contacts);
    for (const j of this.joints) j.prepare(h);

    for (let it = 0; it < this.iterations; it++) {
      for (const j of this.joints) j.solveVel();
      for (const c of contacts) this.solveContact(c);
    }

    for (const b of dyn) { b.vp = [0, 0, 0, 0]; b.wp = bzero(); }
    for (let it = 0; it < this.posIterations; it++) {
      for (const j of this.joints) j.solvePos(h, this.beta);
      for (const c of contacts) this.solveContactPos(c, h);
    }

    for (const b of dyn) this.integrate(b, h);
    this.time += h;
  }

  detect() {
    const out = [], bs = this.bodies.filter(b => b.collide), skip = new Set();
    for (const j of this.joints) if (!j.collide) skip.add(pairKey(j.a, j.b));
    for (let i = 0; i < bs.length; i++) for (let k = i + 1; k < bs.length; k++) {
      const A = bs[i], B = bs[k];
      if (A.static && B.static) continue;
      if (A.user.group && A.user.group === B.user.group && A.user.selfCollide === false) continue;
      if (skip.has(pairKey(A, B))) continue;
      if (A.radius + B.radius !== Infinity && len(sub(A.x, B.x)) > A.radius + B.radius + this.margin) continue;
      collidePair(A, B, out, this.margin);
    }
    return out;
  }

  prepareContact(c) {
    const A = c.a, B = c.b, n = c.n;
    c.ra = sub(c.p, A.x); c.rb = sub(c.p, B.x);
    c.kn = kOf(A, c.ra, n) + kOf(B, c.rb, n);
    c.mu = Math.sqrt(A.friction * B.friction);
    const e = Math.max(A.restitution, B.restitution);
    const vn = dot(sub(A.velAt(c.ra), B.velAt(c.rb)), n);
    c.target = vn < -this.restThreshold ? -e * vn : 0;
    c.ln = 0; c.lt = [0, 0, 0, 0]; c.lp = 0;
  }

  // Reuse last substep's impulses for contacts that persist (matched by the
  // contact point in a's body frame), so stacks converge with few iterations.
  warmStart(contacts) {
    const prev = this.prevContacts ?? new Map(), next = new Map();
    for (const c of contacts) {
      const key = pairKey(c.a, c.b), local = c.a.toLocal(c.p);
      const old = prev.get(key);
      if (old) {
        let best = null, bd = 0.02;
        for (const o of old) {
          if (o.a !== c.a) continue;
          const d = len(sub(o.local, local));
          if (d < bd) { bd = d; best = o; }
        }
        if (best && c.depth >= 0) {
          c.ln = best.ln;
          c.lt = addScaled(best.lt, c.n, -dot(best.lt, c.n));
          const P = addScaled(c.lt, c.n, c.ln);
          c.a.impulse(P, c.ra); c.b.impulse(scale(P, -1), c.rb);
        }
      }
      c.local = local;
      if (!next.has(key)) next.set(key, []);
      next.get(key).push(c);
    }
    this.prevContacts = next;
  }

  solveContact(c) {
    if (c.depth < 0) return; // inside the margin but not touching
    const A = c.a, B = c.b, n = c.n;
    // friction first (uses the normal impulse from the previous iteration)
    let vr = sub(A.velAt(c.ra), B.velAt(c.rb));
    const vt = addScaled(vr, n, -dot(vr, n)), s = len(vt);
    if (s > 1e-9) {
      const t = scale(vt, 1 / s), kt = kOf(A, c.ra, t) + kOf(B, c.rb, t);
      let lt = addScaled(c.lt, t, -s / kt);
      const max = c.mu * c.ln, m = len(lt);
      if (m > max) lt = scale(lt, max / m);
      const d = sub(lt, c.lt); c.lt = lt;
      A.impulse(d, c.ra); B.impulse(scale(d, -1), c.rb);
    }
    vr = sub(A.velAt(c.ra), B.velAt(c.rb));
    const vn = dot(vr, n);
    const ln = Math.max(0, c.ln + (c.target - vn) / c.kn), d = ln - c.ln;
    c.ln = ln;
    A.impulse(scale(n, d), c.ra); B.impulse(scale(n, -d), c.rb);
  }

  solveContactPos(c, h) {
    const err = c.depth - this.slop;
    if (err <= 0) return;
    const A = c.a, B = c.b, n = c.n;
    const vn = dot(sub(A.pvelAt(c.ra), B.pvelAt(c.rb)), n);
    const lp = Math.max(0, c.lp + (this.beta * err / h - vn) / c.kn), d = lp - c.lp;
    c.lp = lp;
    A.pimpulse(scale(n, d), c.ra); B.pimpulse(scale(n, -d), c.rb);
  }

  // Positions: x += v h. Orientation: a midpoint step that conserves the world
  // angular momentum L exactly while the body turns (torque-free precession).
  integrate(b, h) {
    b.x = addScaled(b.x, add(b.v, b.vp), h);
    const L = b.angMom();
    const R0 = b.R;
    b.R = cayleyStep(R0, b.w, h / 2); b.update();
    const wHalf = m6Apply(b.invI, L);
    b.R = orthonormalize(cayleyStep(R0, badd(wHalf, b.wp), h));
    b.update();
    b.w = m6Apply(b.invI, L);
  }

  // ---- diagnostics
  energy() {
    let lin = 0, rot = 0, pot = 0;
    const g = this.gravity;
    for (const b of this.bodies) {
      if (b.static) continue;
      const k = b.kinetic(); lin += k.lin; rot += k.rot;
      pot -= b.mass * b.gravityScale * dot(g, b.x);
    }
    for (const f of this.forces) if (f.potential) pot += f.potential(this);
    return { lin, rot, pot, total: lin + rot + pot };
  }
  momentum() {
    let P = [0, 0, 0, 0], L = bzero();
    for (const b of this.bodies) {
      if (b.static) continue;
      const p = scale(b.v, b.mass);
      P = add(P, p);
      L = badd(L, badd(b.angMom(), wedge(b.x, p)));
    }
    return { P, L };
  }
}

const pairKey = (a, b) => a.id < b.id ? a.id + ':' + b.id : b.id + ':' + a.id;

// central force field toward a point (or a body): |F| = k m / r^p.
// p = 3 is what Gauss's law gives in four space dimensions; p = 2 is our 3D law.
export function centralForce(o) {
  const f = (world, h) => {
    const c = o.center.x ?? o.center;
    for (const b of world.bodies) {
      if (b.static || b === o.center || (o.only && !o.only.includes(b))) continue;
      const d = sub(c, b.x), r = len(d);
      if (r < (o.rmin ?? 1e-3)) continue;
      b.v = addScaled(b.v, d, h * o.k / Math.pow(r, f.p + 1));
    }
  };
  f.p = o.p ?? 3;
  f.potential = world => {
    const c = o.center.x ?? o.center;
    let U = 0;
    for (const b of world.bodies) {
      if (b.static || b === o.center || (o.only && !o.only.includes(b))) continue;
      const r = len(sub(c, b.x));
      U += f.p === 1 ? b.mass * o.k * Math.log(r) : -b.mass * o.k / ((f.p - 1) * Math.pow(r, f.p - 1));
    }
    return U;
  };
  return f;
}
