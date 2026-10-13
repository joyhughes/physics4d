// Headless checks of the 4D engine: conservation laws, contacts, rolling, joints.
import { World, Body, BallJoint, HingeJoint, centralForce } from '../src/world.js';
import { polyHull } from '../src/polytopes.js';
import { gjk } from '../src/collide.js';
import { bdot, mPlane, mMul, mIdentity, randomRotation, rng, biv, blen, bsub, len, sub, dot, mCol } from '../src/math4.js';

let fails = 0;
const check = (name, ok, info) => { console.log((ok ? 'ok   ' : 'FAIL ') + name + (info ? '  ' + info : '')); if (!ok) fails++; };
const f = x => x.toFixed(4);

{ // 1. free tumbling hyperbox: L and E conserved
  const w = new World({ gravity: [0, 0, 0, 0], ground: false });
  const b = w.add(new Body({ shape: { type: 'box', h: [0.5, 0.35, 0.25, 0.15] }, omega: biv(1.3, -0.4, 2.1, 0.7, -1.6, 0.9), rot: randomRotation(rng(3)) }));
  const E0 = w.energy().total, L0 = w.momentum().L;
  let maxDE = 0;
  for (let i = 0; i < 600; i++) { w.step(); maxDE = Math.max(maxDE, Math.abs(w.energy().total - E0)); }
  const dL = blen(bsub(w.momentum().L, L0)) / blen(L0);
  check('free hyperbox conserves L', dL < 1e-9, 'rel dL ' + dL.toExponential(2));
  check('free hyperbox conserves E', maxDE / E0 < 2e-3, 'rel dE ' + (maxDE / E0).toExponential(2));
}

{ // 2. hypersphere drops and settles
  const w = new World();
  const b = w.add(new Body({ shape: { type: 'sphere', r: 0.4 }, pos: [0, 2, 0, 0.3], restitution: 0.5 }));
  const E0 = w.energy().total; let emax = -Infinity, rise = 0, emin = E0;
  for (let i = 0; i < 300; i++) { w.step(); const E = w.energy().total; rise = Math.max(rise, E - emin); emin = Math.min(emin, E); }
  check('sphere comes to rest on floor', Math.abs(b.x[1] - 0.4) < 0.003 && len(b.v) < 0.01, 'y=' + f(b.x[1]));
  check('sphere bounce never gains energy', rise / b.mass < 0.05, 'max rise ' + f(rise / b.mass) + ' J/kg');
}

{ // 3. tumbling tesseract lands on a facet
  const w = new World();
  const b = w.add(new Body({ shape: { type: 'box', h: [0.3, 0.3, 0.3, 0.3] }, pos: [0, 1.5, 0, 0], rot: randomRotation(rng(7)), omega: biv(2, 1, -3, 0.5, 1, 2) }));
  let emin = Infinity, rise = 0;
  for (let i = 0; i < 480; i++) { w.step(); const E = w.energy().total; rise = Math.max(rise, E - emin); emin = Math.min(emin, E); }
  const up = Math.max(...[0, 1, 2, 3].map(j => Math.abs(mCol(b.R, j)[1])));
  check('tesseract settles on a cubic facet', up > 0.999 && Math.abs(b.x[1] - 0.3) < 0.005 && len(b.v) < 0.02, 'y=' + f(b.x[1]) + ' up=' + f(up) + ' |v|=' + f(len(b.v)));
  check('tesseract tumble never gains energy', rise / b.mass < 0.2, 'max rise ' + f(rise / b.mass) + ' J/kg');
}

{ // 4. rolling down a ramp: a = 3/4 g sinθ for a solid 4-ball, 2/3 for a shell
  for (const hollow of [false, true]) {
    const th = 0.25, w = new World({ ground: false });
    const ramp = w.add(new Body({ static: true, shape: { type: 'box', h: [20, 0.5, 2, 2] }, rot: mPlane(0, 1, -th), friction: 1 }));
    const n = mCol(ramp.R, 1), down = scale4(mCol(ramp.R, 0), 1);
    const r = 0.3, start = [n[0] * (0.5 + r), n[1] * (0.5 + r), 0, 0];
    const b = w.add(new Body({ shape: { type: 'sphere', r, hollow }, pos: start, friction: 1, restitution: 0 }));
    for (let i = 0; i < 30; i++) w.step();
    const s0 = -dot(sub(b.x, start), down), t0 = w.time;
    for (let i = 0; i < 60; i++) w.step();
    const s1 = -dot(sub(b.x, start), down), v1 = len(b.v), v0est = 0;
    // fit a from s = s0 + v0 t + a t²/2 using velocities: a = Δv/Δt
    const a = v1 / w.time; // started from rest
    const want = 9.81 * Math.sin(th) * (hollow ? 2 / 3 : 3 / 4);
    check((hollow ? 'hollow' : 'solid') + ' hypersphere rolls at ' + (hollow ? '2/3' : '3/4') + ' g sinθ', Math.abs(a / want - 1) < 0.02, 'a=' + f(a) + ' want ' + f(want));
  }
}
function scale4(v, s) { return v.map(x => x * s); }

{ // 5. stack of five tesseracts stays up
  const w = new World();
  const bs = [];
  for (let i = 0; i < 5; i++) bs.push(w.add(new Body({ shape: { type: 'box', h: [0.25, 0.25, 0.25, 0.25] }, pos: [0.01 * i, 0.25 + 0.5 * i + 0.001, 0, 0.01 * i] })));
  for (let i = 0; i < 600; i++) w.step();
  const top = bs[4];
  check('stack of 5 tesseracts stands', Math.abs(top.x[1] - 2.25) < 0.02 && Math.abs(top.x[0] - 0.04) < 0.02, 'top y=' + f(top.x[1]) + ' x=' + f(top.x[0]));
}

{ // 6. ball-joint pendulum conserves energy and swings into w
  const w = new World({ ground: false });
  const b = w.add(new Body({ shape: { type: 'sphere', r: 0.15 }, pos: [1, 2, 0, 0], vel: [0, 0, 0, 1.5] }));
  w.addJoint(new BallJoint(b, w.anchor, [0, 2, 0, 0]));
  const E0 = w.energy().total; let maxDE = 0, maxW = 0, maxErr = 0;
  for (let i = 0; i < 600; i++) { w.step(); maxDE = Math.max(maxDE, Math.abs(w.energy().total - E0)); maxW = Math.max(maxW, Math.abs(b.x[3])); maxErr = Math.max(maxErr, Math.abs(len(sub(b.x, [0, 2, 0, 0])) - 1)); }
  check('ball pendulum conserves energy', maxDE / b.mass < 0.05, 'max dE ' + f(maxDE / b.mass) + ' J/kg');
  check('ball pendulum holds length', maxErr < 0.005, 'max err ' + f(maxErr));
  check('ball pendulum moves in w', maxW > 0.3, 'max |w| ' + f(maxW));
}

{ // 7. hinge pendulum: turning plane xy only, w push is resisted
  const w = new World({ ground: false });
  const b = w.add(new Body({ shape: { type: 'box', h: [0.5, 0.05, 0.05, 0.05] }, pos: [0.5, 2, 0, 0], vel: [0, 0, 0, 1.5] }));
  const hj = w.addJoint(new HingeJoint(b, w.anchor, [0, 2, 0, 0], [[1, 0, 0, 0], [0, 1, 0, 0]]));
  let maxOff = 0;
  for (let i = 0; i < 300; i++) { w.step(); maxOff = Math.max(maxOff, Math.abs(b.x[2]), Math.abs(b.x[3])); }
  check('hinge keeps the bar in its turning plane', maxOff < 0.01, 'max off-plane ' + f(maxOff));
  check('hinge pendulum swings', Math.abs(hj.angle()) > 0.1 || b.x[1] < 1.9, 'angle ' + f(hj.angle()));
}

{ // 8. elastic hypersphere collision: P and E conserved
  const w = new World({ gravity: [0, 0, 0, 0], ground: false, restThreshold: 0 });
  const a = w.add(new Body({ shape: { type: 'sphere', r: 0.3 }, pos: [-2, 0, 0, 0.2], vel: [3, 0, 0, 0], restitution: 1, friction: 0 }));
  const c = w.add(new Body({ shape: { type: 'sphere', r: 0.3 }, pos: [0, 0, 0, 0], restitution: 1, friction: 0 }));
  const E0 = w.energy().total, P0 = w.momentum().P;
  for (let i = 0; i < 120; i++) w.step();
  const E1 = w.energy().total, P1 = w.momentum().P;
  check('elastic collision conserves E', Math.abs(E1 / E0 - 1) < 0.01, 'E1/E0 ' + f(E1 / E0));
  check('elastic collision conserves P', len(sub(P1, P0)) / len(P0) < 1e-6);
  check('glancing hit sends target into w', c.v[3] < -0.5, 'target v_w ' + f(c.v[3]));
}

{ // 9. inverse-cube circular orbit: only one speed works
  const out = [];
  for (const fac of [0.98, 1, 1.02]) {
    const w = new World({ gravity: [0, 0, 0, 0], ground: false, substeps: 16 });
    const k = 4, r = 2, vc = Math.sqrt(k) / r; // k/r³ = v²/r
    const p = w.add(new Body({ shape: { type: 'sphere', r: 0.05 }, pos: [r, 0, 0, 0], vel: [0, 0, 0, vc * fac], collide: false }));
    w.forces.push(centralForce({ center: [0, 0, 0, 0], k, p: 3, rmin: 0.05 }));
    for (let i = 0; i < 600; i++) w.step();
    out.push(len(p.x));
  }
  check('1/r³ orbits: slow falls in, exact stays, fast escapes', out[0] < 1 && Math.abs(out[1] - 2) < 0.05 && out[2] > 2.5, out.map(f).join(' '));
}

{ // 10. chain of four ball-jointed bars: stays connected, no energy gain
  const w = new World({ ground: false });
  let prev = w.anchor, top = [0, 3, 0, 0];
  const links = [];
  for (let i = 0; i < 4; i++) {
    const b = w.add(new Body({ shape: { type: 'box', h: [0.2, 0.05, 0.05, 0.05] }, pos: [0.2 + 0.4 * i, 3, 0, 0], vel: [0, 0, 0.3 * i, 0.5 * i] }));
    w.addJoint(new BallJoint(b, prev, [0.4 * i, 3, 0, 0]));
    links.push(b); prev = b;
  }
  const E0 = w.energy().total; let rise = 0, gap = 0;
  for (let i = 0; i < 600; i++) {
    w.step();
    rise = Math.max(rise, w.energy().total - E0);
    for (const j of w.joints) { const [p, q] = j.anchors(); gap = Math.max(gap, len(sub(p, q))); }
  }
  const m = links.reduce((s, b) => s + b.mass, 0);
  check('chain stays connected', gap < 0.005, 'max gap ' + f(gap));
  check('chain energy bounded', rise / m < 0.1, 'max rise ' + f(rise / m) + ' J/kg');
}

{ // 11. hinge motor and limits
  const w = new World({ ground: false, gravity: [0, 0, 0, 0] });
  const b = w.add(new Body({ shape: { type: 'box', h: [0.5, 0.1, 0.1, 0.1] }, pos: [0, 0, 0, 0] }));
  const hj = w.addJoint(new HingeJoint(b, w.anchor, [0, 0, 0, 0], [[0, 0, 1, 0], [0, 0, 0, 1]], { motorSpeed: 2, maxTorque: 50 }));
  for (let i = 0; i < 120; i++) w.step();
  const sp = -bdot(hj.U, b.w);
  check('hinge motor reaches its speed', Math.abs(sp - 2) < 0.02, 'speed ' + f(sp));
  const w2 = new World({ ground: false });
  const c = w2.add(new Body({ shape: { type: 'box', h: [0.5, 0.05, 0.05, 0.05] }, pos: [0.5, 2, 0, 0] }));
  const lj = w2.addJoint(new HingeJoint(c, w2.anchor, [0, 2, 0, 0], [[1, 0, 0, 0], [0, 1, 0, 0]], { lo: -0.5, hi: 0.5 }));
  let mx = 0;
  for (let i = 0; i < 120; i++) { w2.step(); mx = Math.max(mx, Math.abs(lj.angle())); }
  check('hinge limit stops the swing', mx > 0.45 && mx < 0.56, 'max |angle| ' + f(mx));
}

{ // 12. polychora: exact volumes, isotropic inertia, rest on a cell at the inradius
  const exact = { '5-cell': Math.sqrt(5) / 96, '16-cell': 1 / 6, '24-cell': 2, '600-cell': 25 / 4 * (2 + Math.sqrt(5)), '120-cell': 15 / 4 * (105 + 47 * Math.sqrt(5)) };
  for (const kind of Object.keys(exact)) {
    const H = polyHull(kind, 1), rel = Math.abs(H.vol / H.edgeLength ** 4 / exact[kind] - 1);
    const iso = Math.max(...H.second) - Math.min(...H.second);
    check(kind + ' volume and isotropic inertia', rel < 1e-9 && iso < 1e-12, 'vol/a⁴ rel err ' + rel.toExponential(1));
  }
  for (const kind of ['5-cell', '24-cell', '120-cell']) {
    const w = new World();
    const b = w.add(new Body({ shape: { type: 'poly', kind, r: 0.5 }, pos: [0, 1.2, 0, 0], rot: randomRotation(rng(5)), omega: biv(1, -2, 0.5, 1.5, -1, 0.7) }));
    let emin = w.energy().total, rise = 0;
    for (let i = 0; i < 900; i++) { w.step(); const E = w.energy().total; rise = Math.max(rise, E - emin); emin = Math.min(emin, E); }
    check(kind + ' settles on a cell', Math.abs(b.x[1] - Math.min(...b.hull.d)) < 0.003 && b.fn.some(n => n[1] < -0.999) && rise / b.mass < 0.01, 'y=' + f(b.x[1]) + ' inradius ' + f(Math.min(...b.hull.d)));
  }
}

{ // 13. GJK distance between two polychora matches the best separating direction
  const A = new Body({ shape: { type: 'poly', kind: '24-cell', r: 0.5 }, rot: randomRotation(rng(1)) });
  const B = new Body({ shape: { type: 'poly', kind: '16-cell', r: 0.5 }, pos: [0.7, 0.5, 0.3, -0.2], rot: randomRotation(rng(2)) });
  const sup = X => d => X.verts.reduce((m, v) => dot(v, d) > dot(m, d) ? v : m);
  const g = gjk(sup(A), sup(B));
  const n = sub(g.pb, g.pa).map(x => x / g.dist);
  const gap = Math.min(...B.verts.map(v => dot(v, n))) - Math.max(...A.verts.map(v => dot(v, n)));
  check('GJK distance is a true separation', Math.abs(gap - g.dist) < 1e-9, 'dist ' + f(g.dist) + ' gap along its normal ' + f(gap));
}

{ // 14. stack of 24-cells and a tesseract holds still
  const w = new World();
  const s = [
    w.add(new Body({ shape: { type: 'poly', kind: '24-cell', r: 0.45 }, pos: [0, 0.5, 0, 0] })),
    w.add(new Body({ shape: { type: 'poly', kind: '24-cell', r: 0.45 }, pos: [0.02, 1.4, 0, 0.01] })),
    w.add(new Body({ shape: { type: 'box', h: [0.25, 0.25, 0.25, 0.25] }, pos: [0, 2.2, 0.03, 0] })),
  ];
  for (let i = 0; i < 900; i++) w.step();
  const d = 0.45 / Math.SQRT2, want = [d, 3 * d, 4 * d + 0.25];
  const err = Math.max(...s.map((b, i) => Math.abs(b.x[1] - want[i])));
  check('polychoron stack rests at cell-to-cell heights', err < 0.004 && s.every(b => len(b.v) < 0.01), 'max height error ' + f(err));
}

console.log(fails ? fails + ' failed' : 'all passed');
process.exit(fails ? 1 : 0);
