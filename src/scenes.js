// Teaching scenes. Each scene builds a World and may return:
//   readouts(world) → [[label, value], …]   live numbers for the panel
//   chart(world)    → { label, series: [{ name, color, value }] }
//   update(world)   → called after each physics step
//   controls        → [{ label, run } | { label, options, value, set }]
// `p` holds the scene's parameters, kept across resets.

import { World, Body, BallJoint, HingeJoint, centralForce } from './world.js';
import { POLYTOPES } from './polytopes.js';
import {
  mPlane, mMul, mApply, mCol, mIdentity, randomRotation, rng, biv, bsplit, len, sub, dot, scale, add, normalize, bdot, blen,
  PLANE_NAMES, AXIS_NAMES,
} from './math4.js';

const C = { amber: 0xe8b04a, blue: 0x4f9fd9, red: 0xd9634f, green: 0x6cbf6a, violet: 0xa98ae0, teal: 0x4fc1b8, rose: 0xd97aa6, stone: 0x8a93a3, sun: 0xffd970 };
const PLANE_COLORS = ['#e8b04a', '#d9634f', '#d97aa6', '#6cbf6a', '#4fc1b8', '#4f9fd9'];

const ball = o => new Body({ ...o, shape: { type: 'sphere', r: o.r ?? 0.4, hollow: !!o.hollow } });
const hbox = (h, o = {}) => new Body({ ...o, shape: { type: 'box', h: Array.isArray(h) ? h : [h, h, h, h] } });
const poly = (kind, o = {}) => new Body({ ...o, shape: { type: 'poly', kind, r: o.r ?? 0.5 } });
const KINDS = ['5-cell', '16-cell', '24-cell', '600-cell', '120-cell'];
const KIND_COLOR = { '5-cell': C.red, '16-cell': C.blue, '24-cell': C.green, '600-cell': C.amber, '120-cell': C.violet };
const deg = x => (x * 180 / Math.PI).toFixed(1) + '°';

// a rotation taking orthonormal a1, a2 to orthonormal b1, b2
function frameTo(a1, a2, b1, b2) {
  const complete = (u, v) => {
    const F = [u, v];
    for (let k = 0; k < 4 && F.length < 4; k++) {
      let e = [0, 0, 0, 0]; e[k] = 1;
      for (const f of F) e = sub(e, scale(f, dot(e, f)));
      if (len(e) > 1e-3) F.push(normalize(e));
    }
    return F;
  };
  const F = complete(a1, a2), T = complete(b1, b2), R = new Float64Array(16);
  const build = () => { for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) R[i * 4 + j] = T.reduce((s, t, k) => s + t[i] * F[k][j], 0); };
  build();
  if (det4(R) < 0) { T[3] = scale(T[3], -1); build(); }
  return R;
}
function det4(m) {
  const a = Array.from(m), n = 4;
  let d = 1;
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r * 4 + c]) > Math.abs(a[p * 4 + c])) p = r;
    if (p !== c) { for (let j = 0; j < 4; j++) [a[c * 4 + j], a[p * 4 + j]] = [a[p * 4 + j], a[c * 4 + j]]; d = -d; }
    d *= a[c * 4 + c];
    for (let r = c + 1; r < n; r++) { const f = a[r * 4 + c] / a[c * 4 + c]; for (let j = c; j < 4; j++) a[r * 4 + j] -= f * a[c * 4 + j]; }
  }
  return d;
}
// how many cells the current slice cuts through (= faces of the slice polyhedron)
function sliceFaces(b, view) {
  const sd = b.verts.map(p => (view ? view.toView(p)[3] - view.s0 : p[3]));
  let n = 0;
  for (const fv of b.hull.facetVerts) {
    let lo = false, hi = false;
    for (const i of fv) { if (sd[i] < 0) lo = true; else hi = true; }
    if (lo && hi) n++;
  }
  return n;
}
// the cell a body rests flat on, if any
function restingCell(b) {
  for (const n of b.fn) if (n[1] < -0.995) return b.hull.cell;
  return null;
}
const add6 = (a, b) => a.map((x, i) => x + b[i]);
const fx = (x, d = 2) => (Math.abs(x) < 0.5 * 10 ** -d ? 0 : x).toFixed(d);
const vec = (v, d = 2) => '(' + v.map(x => fx(x, d)).join(', ') + ')';

// describe a rotation bivector as one or two plane rates
export function describeRotation(w) {
  const { s1, s2, P1, P2 } = bsplit(w);
  const name = P => {
    if (!P) return null;
    let k = 0;
    for (let i = 1; i < 6; i++) if (Math.abs(P[i]) > Math.abs(P[k])) k = i;
    return Math.abs(P[k]) > 0.98 ? PLANE_NAMES[k] : 'a tilted plane';
  };
  if (s1 < 1e-3) return 'not turning';
  if (s2 < 1e-3 * Math.max(1, s1)) {
    const P = w.map(x => x / s1);
    return `simple: ${fx(s1)} rad/s in ${name(P)}`;
  }
  if (Math.abs(s1 - s2) < 1e-3 * s1) return `isoclinic: ${fx(s1)} rad/s in every plane of a family`;
  return `double: ${fx(s1)} rad/s in ${name(P1)} + ${fx(s2)} in ${name(P2)}`;
}

export const SCENES = [
  {
    id: 'slices', kicker: 'Seeing', title: 'Slices of the fourth dimension',
    blurb: 'You see a 3D slice of a 4D world, the hyperplane w = 0. Objects exist off to either side of it in w, the direction you can\'t look along.',
    notice: [
      'The amber hypersphere slides along w. Its slice grows, shrinks, vanishes and returns as it bounces between two walls.',
      'Those walls sit at w = ±1.45. They never cross your slice, so you can\'t see them. Turn on Shadows, or find them on the w-map.',
      'The blue tesseract turns in the xw plane on a motor. Its slice starts as a cube, stretches, and becomes a box with bevelled faces.',
      'Drag the slice through the green tesseract. Its slice stays the same cube for the whole range |w| < 0.35, then vanishes at once, because one of its cubic facets lies flat in the slice.',
    ],
    view: { cam: [4.5, 3.2, 6.5], target: [0, 0.6, 0], map: 2.5 },
    build(w) {
      const b = w.add(ball({ name: 'sliding hypersphere', r: 0.5, pos: [-1.6, 0.5, 0.9, 0], vel: [0, 0, 0, 0.9], friction: 0, restitution: 1, color: C.amber }));
      for (const s of [-1, 1]) w.add(hbox([3.2, 0.8, 3, 0.05], { static: true, pos: [0, 0.8, 0, s * 1.45], restitution: 1, friction: 0, color: C.stone, name: 'wall at w = ' + s * 1.45 }));
      const t = w.add(hbox(0.4, { name: 'turning tesseract', pos: [1.5, 1.1, -0.2, 0], color: C.blue }));
      w.addJoint(new HingeJoint(t, w.anchor, t.x, [[1, 0, 0, 0], [0, 0, 0, 1]], { motorSpeed: 0.5, maxTorque: 1e4 }));
      w.add(hbox(0.35, { name: 'resting tesseract', pos: [-0.2, 0.35, -1.8, 0], color: C.green }));
      return {
        readouts: () => {
          const d = b.x[3], r = b.shape.r;
          return [['hypersphere at w', fx(d)], ['its slice radius', Math.abs(d) < r ? fx(Math.sqrt(r * r - d * d)) + ' m' : 'off-slice']];
        },
      };
    },
  },

  {
    id: 'drop', kicker: 'Gravity', title: 'Falling and landing',
    blurb: 'Gravity pulls along y, as at home. The floor is a 3D hyperplane, y = 0, so there are three horizontal directions to scatter into: x, z and w.',
    notice: [
      'A falling tesseract can strike a corner, an edge, a square face, or a whole cubic facet. Watch it chatter down through each.',
      'Tesseracts end up resting on a cubic facet. They can still be turned within the floor (planes xz, xw, zw), so their resting slice is often a stretched box or a hexagonal prism, not a cube.',
      'Bodies that drift off in w shrink and vanish from view without leaving. The w-map shows where they went.',
    ],
    view: { cam: [5, 4, 6.5], target: [0, 0.5, 0], map: 3 },
    params: { seed: 4 },
    build(w, p) {
      const rand = rng(p.seed * 7919);
      const colors = [C.amber, C.blue, C.red, C.green, C.violet, C.teal, C.rose];
      for (let i = 0; i < 7; i++) {
        const pos = [(rand() - 0.5) * 3, 1.2 + i * 0.7, (rand() - 0.5) * 3, (rand() - 0.5) * 1.2];
        const o = { pos, rot: randomRotation(rand), omega: biv(...[0, 0, 0, 0, 0, 0].map(() => (rand() - 0.5) * 4)), color: colors[i], restitution: 0.3 };
        w.add(i % 3 === 2 ? ball({ ...o, r: 0.3 + rand() * 0.15, name: 'hypersphere' }) : hbox([0.22 + rand() * 0.2, 0.22 + rand() * 0.2, 0.22 + rand() * 0.2, 0.22 + rand() * 0.2], { ...o, name: 'hyperbox' }));
      }
      return {
        controls: [{ label: 'New drop', run: ctx => { p.seed++; ctx.reset(); } }],
        readouts: world => {
          const dyn = world.bodies.filter(b => !b.static);
          const still = dyn.filter(b => len(b.v) < 0.02).length;
          return [['at rest', still + ' of ' + dyn.length], ['contact points', world.contacts.length]];
        },
      };
    },
  },

  {
    id: 'roll', kicker: 'Inertia', title: 'The rolling race',
    blurb: 'Three racers start together on a 15° ramp. A frictionless block slides at g sin θ. Rolling bodies are slower, because some of their energy goes into spin.',
    notice: [
      'A solid 4D ball has second moment m r²/6 along each axis, so its rotational inertia in any plane is m r²/3. It rolls at ¾ g sin θ. A solid 3D ball rolls at 5/7.',
      'A hollow 4D ball (a thin 3-sphere shell) has inertia m r²/2 and rolls at ⅔ g sin θ. A hollow 3D ball rolls at 3/5.',
      'In n dimensions a solid ball rolls at (n + 2)/(n + 4) g sin θ. Its second moment along each axis is m r²/(n + 2): with more axes to share the spread of mass, each one gets less, so the inertia in each plane drops.',
      'Rolling here is a turn in the xy plane. Nothing stops a ball also spinning in xz, xw or zw: the floor\'s own three planes.',
    ],
    view: { cam: [1, 3.5, 8.5], target: [0.5, 1.0, 0], map: 5 },
    build(w) {
      const th = 15 * Math.PI / 180;
      const R = mPlane(0, 1, -th), c = [0, 1.25, 0, 0];
      const ramp = w.add(hbox([4.2, 0.1, 1.7, 0.6], { static: true, pos: c, rot: R, friction: 1, color: C.stone, name: 'ramp' }));
      const at = (s, hgt, z) => add(c, mApply(R, [s, 0.1 + hgt, z, 0]));
      const racers = [
        { b: w.add(ball({ name: 'solid hypersphere', r: 0.3, pos: at(-3.6, 0.3, -1.05), friction: 1, restitution: 0, color: C.amber })), k: 3 / 4, k3: 5 / 7 },
        { b: w.add(ball({ name: 'hollow hypersphere', r: 0.3, hollow: true, pos: at(-3.6, 0.3, 0), friction: 1, restitution: 0, color: C.blue })), k: 2 / 3, k3: 3 / 5 },
        { b: w.add(hbox(0.25, { name: 'frictionless tesseract', pos: at(-3.6, 0.25, 1.05), rot: R, friction: 0, restitution: 0, color: C.green })), k: 1, k3: 1 },
      ];
      const down = mCol(R, 0);
      for (const r of racers) { r.x0 = r.b.x.slice(); r.best = null; }
      const gs = 9.81 * Math.sin(th);
      return {
        update(world) {
          for (const r of racers) {
            const s = dot(sub(r.b.x, r.x0), down), t = world.time;
            if (t > 0.3 && s < 6.4) r.best = 2 * s / (t * t) / gs;
          }
        },
        readouts: () => racers.map(r => [r.b.name, (r.best == null ? '…' : fx(r.best, 3)) + ' g sin θ  (theory ' + fx(r.k, 3) + ', 3D ' + fx(r.k3, 3) + ')']),
      };
    },
  },

  {
    id: 'orbits', kicker: 'Gravitation', title: 'Orbits under inverse-cube gravity',
    blurb: 'Gravity spreads out over the surface around a mass. In 4D that surface is a 3-sphere with area 2π²r³, so gravity falls off as 1/r³ instead of 1/r².',
    notice: [
      'Three planets start at the same distance, at 95%, 100% and 105% of circular speed. Under 1/r³ the slow one spirals into the star and the fast one spirals away. Only the exact speed circles, and even it drifts.',
      'With 1/r³ the centrifugal term L²/2mr² and the gravity term −k/2r² have the same shape. The effective potential has no dip, so there is nothing to hold an orbit near a radius. No stable planets, no ellipses.',
      'Switch the law to 1/r² (our 3D law) and the same three planets settle into closed ellipses.',
      'The violet planet orbits in the xw plane. You see it only as it crosses your slice. The w-map shows its whole path.',
    ],
    view: { cam: [0.01, 11, 3.5], target: [0, 0, 0], map: 5, tilt: [0, 0] },
    params: { law: 3 },
    world: { gravity: [0, 0, 0, 0], ground: false, substeps: 16 },
    build(w, p) {
      const v = 1.1, r0 = 2;
      const k = p.law === 3 ? v * v * r0 * r0 : v * v * r0;
      const star = w.add(ball({ name: 'star', r: 0.25, static: true, collide: false, color: C.sun }));
      const planets = [];
      const mk = (name, ang, fac, color, plane = [0, 2]) => {
        const e1 = [0, 0, 0, 0], e2 = [0, 0, 0, 0];
        e1[plane[0]] = 1; e2[plane[1]] = 1;
        const pos = add(scale(e1, r0 * Math.cos(ang)), scale(e2, r0 * Math.sin(ang)));
        const vel = scale(add(scale(e1, -Math.sin(ang)), scale(e2, Math.cos(ang))), v * fac);
        const b = w.add(ball({ name, r: 0.09, pos, vel, collide: false, color, trail: true, user: { trailLength: 1600, fac } }));
        planets.push(b);
      };
      mk('slow planet (95%)', 0, 0.95, C.red);
      mk('circular planet', 2.1, 1, C.green);
      mk('fast planet (105%)', 4.2, 1.05, C.blue);
      mk('xw planet', 1, 1, C.violet, [0, 3]);
      const only = planets.slice();
      w.forces.push(centralForce({ center: [0, 0, 0, 0], k, p: p.law, only, rmin: 0.05 }));
      return {
        controls: [
          { label: 'Force law', options: [['3', '1/r³ (4D)'], ['2', '1/r² (3D)']], value: String(p.law), set: (ctx, x) => { p.law = +x; ctx.reset(); } },
          { label: 'Nudge planets +2%', run: () => { for (const b of only) b.v = scale(b.v, 1.02); } },
        ],
        update() {
          for (const b of planets) {
            if (b.user.status) continue;
            const r = len(b.x);
            if (r < 0.25) { b.user.status = 'fell into the star'; b.v = [0, 0, 0, 0]; only.splice(only.indexOf(b), 1); }
            else if (r > 14) { b.user.status = 'escaped'; }
          }
        },
        readouts: () => planets.map(b => [b.name, b.user.status ?? 'r = ' + fx(len(b.x)) + ' m']),
      };
    },
  },

  {
    id: 'double', kicker: 'Rotation', title: 'Simple, double and isoclinic rotation',
    blurb: 'In 3D every rotation has an axis that stays put. In 4D you turn in planes, and a body can turn in two perpendicular planes at once.',
    notice: [
      'Left, a simple rotation in the xy plane. A whole plane through the centre stays still, here the zw plane. The marked corner traces a plain circle.',
      'Middle, a double rotation: xy at 1.2 rad/s and zw at 0.5 rad/s together. Only the centre stays still. The corner winds around a torus and its path closes only if the two rates have a rational ratio.',
      'Right, isoclinic: equal rates in xy and zw. Every point moves at the same speed on a great circle, and every plane of a whole family turns at that rate.',
      'Angular momentum is a bivector with six parts, one per plane. Its "wedge square" L∧L is zero exactly when the motion is a simple rotation.',
    ],
    view: { cam: [0.5, 2.6, 7.5], target: [0, 0, 0], map: 3.5 },
    world: { gravity: [0, 0, 0, 0], ground: false },
    build(w) {
      const h = 0.45, mk = (name, x, om, color) => w.add(hbox(h, { name, pos: [x, 0, 0, 0], omega: om, color, user: { marker: [h, h, h, h], trailLength: 1400 } }));
      const bs = [
        mk('simple', -2.3, biv(1.2, 0, 0, 0, 0, 0), C.amber),
        mk('double', 0, biv(1.2, 0, 0, 0, 0, 0.5), C.blue),
        mk('isoclinic', 2.3, biv(1, 0, 0, 0, 0, 1), C.green),
      ];
      return { readouts: () => bs.map(b => [b.name, describeRotation(b.w)]) };
    },
  },

  {
    id: 'spin', kicker: 'Stability', title: 'Which spins are stable?',
    blurb: 'A hyperbox measuring 1.2 × 0.9 × 0.6 × 0.3 m spins in one of its six planes, with a tiny wobble added. Some spins hold steady. Others flip and tumble.',
    notice: [
      'The chart shows the six parts of the box\'s angular velocity, in the box\'s own frame. A stable spin stays a flat line.',
      'Rule: a spin in plane ab is stable unless the box is longer along some third axis c than along one of a, b and shorter than along the other. So the planes of neighbouring sizes (xy, yz, zw) are stable. xz, yw and xw tumble.',
      'In 3D that rule is the tennis-racket theorem: spinning about the middle axis is unstable. 4D has three unstable planes instead of one.',
      'Angular momentum (the 6-part bivector in the readouts) never changes while the box tumbles. Energy is shared between planes, but its total stays fixed.',
    ],
    view: { cam: [3.5, 2.5, 4.5], target: [0, 0, 0], map: 2 },
    world: { gravity: [0, 0, 0, 0], ground: false },
    params: { plane: 2 },
    build(w, p) {
      const om = [0.01, -0.012, 0.011, 0.009, -0.01, 0.013];
      om[p.plane] = 2.5;
      const b = w.add(hbox([0.6, 0.45, 0.3, 0.15], { name: 'hyperbox', omega: om, color: C.amber }));
      const h = b.shape.h, [a, c] = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]][p.plane];
      const stable = [0, 1, 2, 3].filter(k => k !== a && k !== c).every(k => (h[a] - h[k]) * (h[c] - h[k]) > 0);
      return {
        controls: [{ label: 'Spin plane', options: PLANE_NAMES.map((n, i) => [String(i), n]), value: String(p.plane), set: (ctx, x) => { p.plane = +x; ctx.reset(); } }],
        readouts: () => [
          ['spin plane', PLANE_NAMES[p.plane] + (stable ? ' (predicted stable)' : ' (predicted to tumble)')],
          ['inertia in that plane', fx(b.J[p.plane], 1) + ' kg m²'],
        ],
        chart: () => {
          const wb = b.bodyOmega();
          return { label: 'ω in body frame (rad/s)', series: PLANE_NAMES.map((n, i) => ({ name: n, color: PLANE_COLORS[i], value: wb[i] })) };
        },
      };
    },
  },

  {
    id: 'billiards', kicker: 'Momentum', title: 'Hyperspheres colliding',
    blurb: 'A cue ball strikes a rack of five touching hyperspheres arranged at the corners of a 5-cell, the 4D tetrahedron. Every collision is perfectly elastic.',
    notice: [
      'Momentum is a 4-vector and is conserved exactly. Watch the total in the readouts: (P, 0, 0, 0) before and after.',
      'The rack sends balls out in all four directions. Those heading into ±w leave your slice: they shrink and wink out while still in the room.',
      'Aim with an offset in w and the cue ball hits the apex ball off-centre in a direction you cannot see. Both balls slide away out of view.',
      'Kinetic energy is conserved too, up to small losses when several balls touch at once. The sequential solver can\'t keep a perfect Newton\'s cradle.',
    ],
    view: { cam: [0.5, 3, 7], target: [0.5, 0, 0], map: 4 },
    world: { gravity: [0, 0, 0, 0], ground: false, restThreshold: 0, iterations: 20 },
    params: { aim: 0 },
    build(w, p) {
      const r = 0.3, e = 0.6 + 0.002, o = { r, restitution: 1, friction: 0 };
      const cue = w.add(ball({ ...o, name: 'cue ball', pos: [-4, 0, 0, p.aim], vel: [5, 0, 0, 0], color: 0xeeeeee }));
      w.add(ball({ ...o, name: 'apex', pos: [0, 0, 0, 0], color: C.red }));
      // a regular tetrahedron in (y, z, w), edge e, centred on the x axis
      const R = e * Math.sqrt(6) / 4, dx = Math.sqrt(e * e - R * R);
      const tet = [[1, 1, 1], [1, -1, -1], [-1, 1, -1], [-1, -1, 1]].map(t => t.map(c => c * R / Math.sqrt(3)));
      const cols = [C.amber, C.blue, C.green, C.violet];
      tet.forEach((t, i) => w.add(ball({ ...o, name: 'ball ' + (i + 1), pos: [dx, t[0], t[1], t[2]], color: cols[i] })));
      const P0 = w.momentum().P, E0 = w.energy().total;
      return {
        controls: [{ label: 'Cue aim', options: [['0', 'dead centre'], ['0.12', 'offset 0.12 m in w'], ['0.25', 'offset 0.25 m in w']], value: String(p.aim), set: (ctx, x) => { p.aim = +x; ctx.reset(); } }],
        readouts: world => [
          ['momentum at start', vec(P0)],
          ['momentum now', vec(world.momentum().P)],
          ['kinetic energy', fx(world.energy().total / E0 * 100, 1) + '% of start'],
          ['cue ball velocity', vec(cue.v)],
        ],
      };
    },
  },

  {
    id: 'room', kicker: 'Enclosure', title: 'The locked room',
    blurb: 'A hypersphere bounces inside a cube of six glass walls. In 3D it would be trapped. In 4D the room is open along w.',
    notice: [
      'The walls reach only from w = −0.5 to 0.5. Press "Nudge into w". Once the ball has drifted past that range it passes straight "through" the walls. They aren\'t there at its w.',
      'A 4D room needs eight walls, one per facet of a tesseract. "Add the w walls" places the missing two at w = ±0.6. You can\'t see them in the slice. The w-map shows them as bars above and below.',
      'With all eight walls, a nudge in w just adds a bounce off walls you never see.',
      'The same reason makes 3D knots fall open in 4D, and lets a 4D surgeon reach inside you without cutting.',
    ],
    view: { cam: [3.2, 2.4, 4.2], target: [0, 0, 0], map: 2.5 },
    world: { gravity: [0, 0, 0, 0], ground: false, restThreshold: 0 },
    params: { closed: false },
    build(w, p) {
      const o = { static: true, restitution: 1, friction: 0, color: C.stone, user: { glass: true } }, L = 1.2, t = 0.1, W = 0.5;
      for (const s of [-1, 1]) {
        w.add(hbox([t, L, L, W], { ...o, pos: [s * 1.1, 0, 0, 0], name: 'x wall' }));
        w.add(hbox([L, t, L, W], { ...o, pos: [0, s * 1.1, 0, 0], name: 'y wall' }));
        w.add(hbox([L, L, t, W], { ...o, pos: [0, 0, s * 1.1, 0], name: 'z wall' }));
        if (p.closed) w.add(hbox([L, L, L, t], { ...o, pos: [0, 0, 0, s * 0.6], name: 'w wall' }));
      }
      const b = w.add(ball({ name: 'hypersphere', r: 0.25, vel: [1.7, 1.1, 0.6, 0], restitution: 1, friction: 0, color: C.amber }));
      return {
        controls: [
          { label: 'Nudge into w', run: () => { b.v[3] += 0.35; } },
          { label: p.closed ? 'Remove the w walls' : 'Add the w walls', run: ctx => { p.closed = !p.closed; ctx.reset(); } },
        ],
        readouts: () => {
          const inside = b.x.slice(0, 3).every(c => Math.abs(c) < 1) && Math.abs(b.x[3]) < 0.5;
          return [['ball position', vec(b.x)], ['status', inside ? 'inside the room' : (Math.abs(b.x[3]) > 0.5 ? 'outside, past the walls in w' : 'outside')]];
        },
      };
    },
  },

  {
    id: 'pendulums', kicker: 'Joints', title: 'Pendulums and the 4D hinge',
    blurb: 'Three pendulums get the same push, along x and along w. A ball joint lets a pendulum swing in any direction. A 4D hinge allows only one plane of motion.',
    notice: [
      'Left, a ball joint. The bob swings round a 3-sphere of directions (x, z and w), so the push in w gives a conical swing you see only partly.',
      'Middle, a hinge. In 3D a hinge turns about a line. In 4D it turns in one plane (here xy) and holds a whole plane still, the zw plane through the pivot. The push in w is simply absorbed.',
      'Right, a double pendulum on ball joints. Chaos in four dimensions has more room to wander.',
      'Total energy stays flat (see the chart): the joints do no work.',
    ],
    view: { cam: [0.5, 2.2, 7], target: [0, 1.6, 0], map: 3 },
    build(w) {
      w.add(hbox([2.2, 0.05, 0.05, 0.05], { static: true, collide: false, pos: [0, 2.6, 0, 0], color: C.stone, name: 'beam' }));
      const push = [1.4, 0, 0, 1.1];
      const a = w.add(ball({ name: 'ball-joint bob', r: 0.18, pos: [-1.5, 1.3, 0, 0], vel: push, color: C.amber }));
      w.addJoint(new BallJoint(a, w.anchor, [-1.5, 2.55, 0, 0]));
      const rod = w.add(hbox([0.05, 0.6, 0.05, 0.05], { name: 'hinged rod', pos: [0, 1.95, 0, 0], vel: scale(push, 0.5), color: C.blue }));
      w.addJoint(new HingeJoint(rod, w.anchor, [0, 2.55, 0, 0], [[1, 0, 0, 0], [0, 1, 0, 0]]));
      const c1 = w.add(ball({ name: 'double pendulum, upper', r: 0.13, pos: [1.5, 1.9, 0, 0], color: C.green }));
      const c2 = w.add(ball({ name: 'double pendulum, lower', r: 0.13, pos: [1.5, 1.25, 0, 0], vel: push, color: C.green }));
      w.addJoint(new BallJoint(c1, w.anchor, [1.5, 2.55, 0, 0]));
      w.addJoint(new BallJoint(c2, c1, [1.5, 1.9, 0, 0]));
      return {
        controls: [{ label: 'Push all in w', run: () => { for (const b of [a, rod, c2]) b.v[3] += 1; } }],
        readouts: () => [
          ['ball bob sideways (z, w)', fx(a.x[2]) + ', ' + fx(a.x[3])],
          ['hinged rod sideways (z, w)', fx(rod.x[2], 4) + ', ' + fx(rod.x[3], 4)],
        ],
      };
    },
  },

  {
    id: 'tower', kicker: 'Contact', title: 'Toppling a tower',
    blurb: 'A hypersphere is thrown at a tower of tesseracts. Friction, stacking and impacts, the same work the block creatures\' engine does in 3D.',
    notice: [
      'A head-on throw knocks blocks down along x, inside your slice.',
      'Throw with an offset in w and the tower falls partly into w. Blocks tip over a cubic facet edge you can\'t see, and their slices shrink to nothing as they go.',
      'Friction acts in the 3D tangent space of each contact. A block sliding on the floor can drift in x, z and w at once.',
    ],
    view: { cam: [-3.5, 2.8, 6.5], target: [0, 1, 0], map: 3 },
    params: { off: 0.18 },
    build(w, p) {
      const cols = [C.blue, C.teal, C.green, C.amber, C.red, C.violet, C.rose];
      for (let i = 0; i < 7; i++) w.add(hbox([0.25, 0.2, 0.25, 0.25], { name: 'block ' + (i + 1), pos: [0, 0.2 + 0.4 * i + 0.0005 * i, 0, 0], color: cols[i], friction: 0.6 }));
      const b = w.add(ball({ name: 'thrown hypersphere', r: 0.32, pos: [-4.5, 1.4, 0, p.off], vel: [6.5, 2.6, 0, 0], color: 0xeeeeee, density: 2500 }));
      return {
        controls: [{ label: 'Throw', options: [['0', 'head-on'], ['0.18', 'offset 0.18 m in w'], ['0.35', 'offset 0.35 m in w']], value: String(p.off), set: (ctx, x) => { p.off = +x; ctx.reset(); } }],
        readouts: world => {
          const blocks = world.bodies.filter(x => x.name.startsWith('block'));
          const offW = blocks.filter(x => Math.abs(x.x[3]) > 0.5).length;
          return [['blocks still standing', blocks.filter(x => x.x[1] > 0.4 * blocks.indexOf(x) + 0.1 && len(sub([x.x[0], 0, x.x[2], x.x[3]], [0, 0, 0, 0])) < 0.15).length + ' of 7'], ['blocks gone off into w', offW]];
        },
      };
    },
  },
  {
    id: 'polychora', kicker: 'Polychora', title: 'The regular polychora',
    blurb: 'The five convex regular polychora, the 4D Platonic solids, float free. Each turns steadily, so you see a changing 3D slice of each one.',
    notice: [
      'Each slice is a polyhedron, and each of its faces is the slice of one cell. The face count in the readouts is the number of cells the slice cuts through.',
      'Left to right: the 5-cell (5 tetrahedra), 16-cell (16 tetrahedra), 24-cell (24 octahedra, a shape with no 3D counterpart), 600-cell (600 tetrahedra) and 120-cell (120 dodecahedra). The tesseract (8 cubes) is the sixth; it is in the other scenes.',
      'No motors drive them. A regular polychoron has the same inertia in all six planes, so any spin it is given just carries on: simple, double or isoclinic.',
      'Drag the slice in w. The 600-cell and 120-cell are so nearly round that their slices barely change shape, only size, like a hypersphere\'s.',
    ],
    view: { cam: [0.01, 1.8, 7.5], target: [0, 0, 0], map: 3.5 },
    world: { gravity: [0, 0, 0, 0], ground: false },
    params: { spin: 'xw' },
    build(w, p) {
      const om = { xw: biv(0, 0, 0.45, 0, 0, 0), double: biv(0, 0, 0.45, 0.2, 0, 0), iso: biv(0.3, 0, 0, 0, 0, 0.3), none: biv() }[p.spin];
      const bodies = KINDS.map((k, i) => w.add(poly(k, { name: k, r: 0.55, pos: [(i - 2) * 1.35, 0, 0, 0], omega: om, color: KIND_COLOR[k] })));
      return {
        controls: [{ label: 'Spin', options: [['xw', 'simple, in xw'], ['double', 'double, xw + yz'], ['iso', 'isoclinic, xy = zw'], ['none', 'still']], value: p.spin, set: (ctx, x) => { p.spin = x; ctx.reset(); } }],
        readouts: world => bodies.map(b => [b.name + ' ' + b.hull.schlafli, `${b.hull.cells} ${b.hull.cellAdj} cells; slice has ${sliceFaces(b, world.view)} faces`]),
      };
    },
  },

  {
    id: 'tipping', kicker: 'Polychora', title: 'Which polytopes roll?',
    blurb: 'Six bodies of the same outer radius start at rest, each sitting on a cell, on a ramp whose slope you choose. A hypersphere always rolls. A polytope tips over only if the slope is steep enough.',
    notice: [
      'A polytope sitting on a cell tips when the slope carries its centre of mass past the edge of that cell, a 2-face in 4D. The readouts list each one\'s tipping angle.',
      'More cells make a rounder body and a smaller tipping angle. What matters is the number of cells, not vertices: the 600-cell (600 cells, 120 vertices) tips at 7.8°, the 120-cell (120 cells, 600 vertices) only at 18°.',
      'The 16-cell and 24-cell both tip at exactly 30°, the tesseract at 45° and the 5-cell at 52°. In 3D: a tetrahedron tips at 54.7°, a cube at 45°, an icosahedron at 20.9°.',
      'Once tipped, a polytope rocks from cell to cell and loses energy at every landing. The next cell may sit less favourably on the slope: at 33° the 16-cell tips once and stops, while the 24-cell keeps going. The hypersphere rolls smoothly at ¾ g sin θ.',
      'Watch the w-map: rocking polytopes drift off in w as well as z, because each new cell faces partly into w. They leave the slice while still on the ramp.',
    ],
    view: { cam: [-0.6, 4.6, 12.5], target: [0.6, 1.3, 0], map: 5 },
    params: { slope: 20 },
    build(w, p) {
      const th = p.slope * Math.PI / 180, L = 3.6, R = mPlane(0, 1, -th), r = 0.32;
      const c = [0, L * Math.sin(th) + 0.1, 0, 0];
      w.add(hbox([L, 0.1, 3.3, 1.6], { static: true, pos: c, rot: R, friction: 1.6, color: C.stone, name: 'ramp' }));
      w.add(hbox([0.1, 0.35, 3.3, 3], { static: true, pos: [L * Math.cos(th) + 1.6, 0.35, 0, 0], friction: 0.5, color: C.stone, name: 'stop' }));
      const at = (s, hgt, z) => add(c, mApply(R, [s, 0.1 + hgt, z, 0]));
      const kinds = ['600-cell', '120-cell', '24-cell', '16-cell', '5-cell'];
      const racers = [w.add(ball({ name: 'hypersphere', r, pos: at(-L + 0.6, r, -2.75), friction: 1.6, restitution: 0, color: 0xdddddd }))];
      racers[0].user.tip = 0;
      kinds.forEach((k, i) => {
        const b0 = poly(k, { r });
        // cell 0 flat on the ramp, with its nearest edge facing downhill
        const rot = frameTo(b0.hull.normals[0], b0.hull.tipDir, [0, -1, 0, 0], [1, 0, 0, 0]);
        const b = w.add(poly(k, { name: k, r, pos: at(-L + 0.6, b0.hull.d[0] + 0.001, -2.75 + 1.1 * (i + 1)), rot: mMul(R, rot), friction: 1.6, restitution: 0, color: KIND_COLOR[k] }));
        b.user.tip = b.hull.tipAngle;
        racers.push(b);
      });
      for (const b of racers) b.user.x0 = b.x.slice();
      const along = mCol(R, 0);
      return {
        controls: [{ label: 'Slope', options: [['5', '5°'], ['10', '10°'], ['20', '20°'], ['33', '33°'], ['55', '55°']], value: String(p.slope), set: (ctx, x) => { p.slope = +x; ctx.reset(); } }],
        readouts: () => racers.map(b => {
          const s = dot(sub(b.x, b.user.x0), along);
          const still = len(b.v) < 0.05;
          const state = s > 2 * L - 1.2 ? 'reached the bottom' : !still ? 'rolling' : s < 0.02 ? 'resting' : 'tipped, then stopped';
          return [b.name, `${state}, ${fx(Math.min(2 * L, Math.max(0, s)), 1)} m${b.user.tip ? '; tips above ' + deg(b.user.tip) : '; always rolls'}`];
        }),
      };
    },
  },

  {
    id: 'steady', kicker: 'Polychora', title: 'Spins that never wobble',
    blurb: 'A long hyperbox, a 24-cell and a 120-cell get the same spin, with the same tiny wobble. The hyperbox may tumble. The polychora never do, whatever the spin.',
    notice: [
      'The chart shows how much of each body\'s angular velocity still lies in its starting plane. The two polychora hold a flat line at 100%. The hyperbox, set spinning in xz or yw, flips away as in "Which spins are stable?".',
      'The polychora have the same inertia in all six planes (see the readouts), like a hypersphere. Angular velocity is then angular momentum divided by a single number, and angular momentum is conserved, so the spin stays fixed in any plane or mix of planes.',
      'Every regular polychoron is like this, and so is a tesseract with equal sides: their symmetry leaves no plane special. In 3D a cube, a regular tetrahedron and a ball all spin like this.',
      'Choose a random spin. The hyperbox wanders chaotically, while the two polychora carry the same double rotation forever.',
    ],
    view: { cam: [0.01, 2.2, 6.5], target: [0, 0, 0], map: 3 },
    world: { gravity: [0, 0, 0, 0], ground: false },
    params: { spin: 'xz' },
    build(w, p) {
      const spins = { xz: biv(0, 2.2, 0, 0, 0, 0), yw: biv(0, 0, 0, 0, 2.2, 0), xy: biv(2.2, 0, 0, 0, 0, 0), random: biv(1.2, -0.8, 1.5, 0.6, -1.1, 0.9) };
      const om = add6(spins[p.spin], [0.01, -0.012, 0.011, 0.009, -0.01, 0.013]);
      const bodies = [
        w.add(hbox([0.6, 0.45, 0.3, 0.15], { name: 'hyperbox', pos: [-1.9, 0, 0, 0], omega: om, color: C.stone })),
        w.add(poly('24-cell', { name: '24-cell', r: 0.6, pos: [0, 0, 0, 0], omega: om, color: KIND_COLOR['24-cell'] })),
        w.add(poly('120-cell', { name: '120-cell', r: 0.6, pos: [1.9, 0, 0, 0], omega: om, color: KIND_COLOR['120-cell'] })),
      ];
      const o0 = om.map(x => x / blen(om));
      const kept = b => Math.max(0, bdot(b.w, o0) / blen(b.w));
      return {
        controls: [{ label: 'Spin', options: [['xz', 'in xz (box tumbles)'], ['yw', 'in yw (box tumbles)'], ['xy', 'in xy (box stable)'], ['random', 'random double rotation']], value: p.spin, set: (ctx, x) => { p.spin = x; ctx.reset(); } }],
        readouts: () => bodies.map(b => {
          const J = b.J, same = Math.max(...J) - Math.min(...J) < 1e-6 * Math.max(...J);
          return [b.name + ' inertia', same ? `${fx(J[0], 2)} kg m² in all six planes` : J.map((j, i) => PLANE_NAMES[i] + ' ' + fx(j, 1)).join('  ')];
        }),
        chart: () => ({ label: 'share of spin still in the starting plane', series: bodies.map((b, i) => ({ name: b.name, color: ['#8a93a3', '#6cbf6a', '#a98ae0'][i], value: kept(b) })) }),
      };
    },
  },

  {
    id: 'bin', kicker: 'Polychora', title: 'A bin of polychora',
    blurb: 'A dozen polychora fall into a 4D bin and pile up. Each lands, rocks and comes to rest on one of its cells.',
    notice: [
      'A 4D bin has eight walls: the floor, and walls facing x, z and w (plus an open top). The two w walls are glass you never see in the slice; the w-map shows them.',
      'A polychoron on the floor touches it across a whole cell, a solid 3D region: a tetrahedron for the 5-, 16- and 600-cells, an octahedron for the 24-cell, a dodecahedron for the 120-cell. The readouts count them.',
      'Bodies at other w shrink and vanish from the slice, yet still fill the bin. Drag the slice to look through the pile layer by layer.',
      '16-cells and 24-cells (and tesseracts) can fill 4D space with no gaps, the way cubes fill 3D. 5-cells, 120-cells and 600-cells cannot.',
    ],
    view: { cam: [3.4, 4.3, 5.2], target: [0, 1.0, 0], map: 2.2 },
    params: { seed: 2 },
    build(w, p) {
      const o = { static: true, restitution: 0.2, friction: 0.4, color: C.stone, user: { glass: true } }, Hh = 1.6, t = 0.08, B = 0.85;
      for (const s of [-1, 1]) {
        w.add(hbox([t, Hh, B, B], { ...o, pos: [s * (B + t), Hh, 0, 0], name: 'x wall' }));
        w.add(hbox([B, Hh, t, B], { ...o, pos: [0, Hh, s * (B + t), 0], name: 'z wall' }));
        w.add(hbox([B, Hh, B, t], { ...o, pos: [0, Hh, 0, s * (B + t)], name: 'w wall' }));
      }
      const rand = rng(p.seed * 104729);
      const mix = ['24-cell', '16-cell', '5-cell', '600-cell', '24-cell', '16-cell', '120-cell', '5-cell', '24-cell', '600-cell', '16-cell', '5-cell'];
      const bodies = mix.map((k, i) => w.add(poly(k, {
        name: k, r: k === '5-cell' ? 0.45 : 0.36,
        pos: [(rand() - 0.5) * 0.9, 0.7 + i * 0.75, (rand() - 0.5) * 0.9, (rand() - 0.5) * 0.9],
        rot: randomRotation(rand), omega: biv(...[0, 0, 0, 0, 0, 0].map(() => (rand() - 0.5) * 2)),
        restitution: 0.15, friction: 0.5, color: KIND_COLOR[k],
      })));
      return {
        controls: [
          { label: 'New drop', run: ctx => { p.seed++; ctx.reset(); } },
          { label: 'Shake the bin', run: () => { const r = rng(Date.now() & 0xffff); for (const b of bodies) { b.v = add(b.v, [(r() - 0.5) * 2, 2.5 + r() * 2, (r() - 0.5) * 2, (r() - 0.5) * 2]); } } },
        ],
        readouts: world => {
          const still = bodies.filter(b => len(b.v) < 0.03).length;
          const tally = {};
          let piled = 0;
          for (const b of bodies) {
            const c = b.x[1] < b.hull.d[0] + 0.01 && restingCell(b);
            if (c) tally[c] = (tally[c] ?? 0) + 1;
            else if (len(b.v) < 0.03) piled++;
          }
          return [
            ['at rest', still + ' of ' + bodies.length],
            ['flat on the floor', Object.entries(tally).map(([c, n]) => `${n} on a${/^[aeiou]/.test(c) ? 'n' : ''} ${c}`).join(', ') || 'none yet'],
            ['resting on other polychora', piled],
            ['contact points', world.contacts.length],
          ];
        },
      };
    },
  },
];

export { PLANE_COLORS };
