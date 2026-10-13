// Teaching scenes. Each scene builds a World and may return:
//   readouts(world) → [[label, value], …]   live numbers for the panel
//   chart(world)    → { label, series: [{ name, color, value }] }
//   update(world)   → called after each physics step
//   controls        → [{ label, run } | { label, options, value, set }]
// `p` holds the scene's parameters, kept across resets.

import { World, Body, BallJoint, HingeJoint, centralForce } from './world.js';
import {
  mPlane, mMul, mApply, mCol, mIdentity, randomRotation, rng, biv, bsplit, len, sub, dot, scale, add,
  PLANE_NAMES, AXIS_NAMES,
} from './math4.js';

const C = { amber: 0xe8b04a, blue: 0x4f9fd9, red: 0xd9634f, green: 0x6cbf6a, violet: 0xa98ae0, teal: 0x4fc1b8, rose: 0xd97aa6, stone: 0x8a93a3, sun: 0xffd970 };
const PLANE_COLORS = ['#e8b04a', '#d9634f', '#d97aa6', '#6cbf6a', '#4fc1b8', '#4f9fd9'];

const ball = o => new Body({ ...o, shape: { type: 'sphere', r: o.r ?? 0.4, hollow: !!o.hollow } });
const hbox = (h, o = {}) => new Body({ ...o, shape: { type: 'box', h: Array.isArray(h) ? h : [h, h, h, h] } });
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
];

export { PLANE_COLORS };
