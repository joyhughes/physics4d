// Narrowphase collision in 4D: planes (static half-spaces), hyperspheres and
// hyperboxes (tesseracts with any side lengths).
//
// Every contact is { a, b, n, p, depth }: n is the unit normal that pushes a away
// from b, p the world contact point, depth the penetration (negative = gap).
//
// Box–box uses the separating axis test. In 4D the candidate axes are the 4 + 4
// facet normals, plus the normals of every edge of one box against every 2-face
// of the other (a vector orthogonal to three directions), 48 more. A facet axis
// gives a contact patch (edges of the incident cell clipped to the reference
// cell); an edge–face axis gives one point.

import { add, sub, scale, addScaled, dot, len, cross3, lerp } from './math4.js';

export function collidePair(A, B, out, margin) {
  const ta = A.shape.type, tb = B.shape.type;
  if (ta === 'plane') { if (tb !== 'plane') collidePair(B, A, out, margin); return; }
  if (tb === 'plane') return ta === 'sphere' ? spherePlane(A, B, out, margin) : boxPlane(A, B, out, margin);
  if (ta === 'sphere' && tb === 'sphere') return sphereSphere(A, B, out, margin);
  if (ta === 'sphere') return sphereBox(A, B, out, margin);
  if (tb === 'sphere') return sphereBox(B, A, out, margin);
  return boxBox(A, B, out, margin);
}

function spherePlane(S, P, out, margin) {
  const n = P.shape.n, dist = dot(n, S.x) - P.shape.d - S.shape.r;
  if (dist < margin) out.push({ a: S, b: P, n, p: addScaled(S.x, n, -S.shape.r - dist / 2), depth: -dist });
}

function boxPlane(Bx, P, out, margin) {
  const n = P.shape.n;
  for (const v of Bx.verts) {
    const dist = dot(n, v) - P.shape.d;
    if (dist < margin) out.push({ a: Bx, b: P, n, p: addScaled(v, n, -dist / 2), depth: -dist });
  }
}

function sphereSphere(A, B, out, margin) {
  const d = sub(A.x, B.x), l = len(d), depth = A.shape.r + B.shape.r - l;
  if (depth < -margin || l < 1e-12) return;
  const n = scale(d, 1 / l);
  out.push({ a: A, b: B, n, p: addScaled(B.x, n, B.shape.r - depth / 2), depth });
}

function sphereBox(S, Bx, out, margin) {
  const h = Bx.shape.h, r = S.shape.r, ax = Bx.axes;
  const rel = sub(S.x, Bx.x), s = ax.map(a => dot(a, rel));
  const inside = s.every((c, i) => Math.abs(c) <= h[i]);
  if (inside) {
    let k = 0, m = Infinity;
    for (let i = 0; i < 4; i++) { const g = h[i] - Math.abs(s[i]); if (g < m) { m = g; k = i; } }
    const n = scale(ax[k], s[k] >= 0 ? 1 : -1);
    out.push({ a: S, b: Bx, n, p: addScaled(S.x, n, -r), depth: m + r });
    return;
  }
  const q = s.map((c, i) => Math.max(-h[i], Math.min(h[i], c)));
  let qw = Bx.x.slice();
  for (let i = 0; i < 4; i++) qw = addScaled(qw, ax[i], q[i]);
  const d = sub(S.x, qw), l = len(d), depth = r - l;
  if (depth < -margin) return;
  const n = scale(d, 1 / l);
  out.push({ a: S, b: Bx, n, p: addScaled(qw, n, -depth / 2), depth });
}

// ---- box–box ----

const FACE_PAIRS = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];

function boxBox(A, B, out, margin) {
  const d = sub(A.x, B.x); // from B toward A
  const aa = A.axes, ba = B.axes, ha = A.shape.h, hb = B.shape.h;

  // overlap along unit axis n, with n flipped to point from B toward A
  const overlap = n => {
    let rA = 0, rB = 0;
    for (let k = 0; k < 4; k++) { rA += ha[k] * Math.abs(dot(aa[k], n)); rB += hb[k] * Math.abs(dot(ba[k], n)); }
    let dn = dot(d, n);
    if (dn < 0) { n = scale(n, -1); dn = -dn; }
    return { ov: rA + rB - dn, n };
  };

  let best = null;
  for (let i = 0; i < 4; i++) {
    for (const [body, axes] of [[A, aa], [B, ba]]) {
      const { ov, n } = overlap(axes[i]);
      if (ov < -margin) return;
      if (!best || ov < best.ov) best = { ov, n, kind: body === A ? 'faceA' : 'faceB' };
    }
  }
  const faceBest = best.ov;
  // edge of A against 2-face of B, and 2-face of A against edge of B
  for (let i = 0; i < 4; i++) for (const [j, k] of FACE_PAIRS) {
    for (const kind of ['edgeA', 'edgeB']) {
      const c = kind === 'edgeA' ? cross3(aa[i], ba[j], ba[k]) : cross3(ba[i], aa[j], aa[k]);
      const l = len(c);
      if (l < 1e-5) continue;
      const { ov, n } = overlap(scale(c, 1 / l));
      if (ov < -margin) return;
      if (ov < best.ov && ov < 0.95 * faceBest - 0.002) best = { ov, n, kind, i, j, k };
    }
  }

  const n = best.n;
  if (best.kind === 'faceA' || best.kind === 'faceB') {
    const ref = best.kind === 'faceB' ? B : A, inc = ref === B ? A : B;
    const nOut = ref === B ? n : scale(n, -1);
    for (const c of faceContacts(ref, inc, nOut, margin)) out.push({ a: A, b: B, n, p: c.p, depth: c.depth });
    return;
  }
  // edge–face: the edge's box sits on the side its support points toward
  const E = best.kind === 'edgeA' ? A : B, F = E === A ? B : A;
  const toF = E === A ? scale(n, -1) : n; // direction from E toward F
  const ea = E.axes, fa = F.axes, he = E.shape.h, hf = F.shape.h;
  let e0 = E.x.slice();
  for (let l = 0; l < 4; l++) if (l !== best.i) e0 = addScaled(e0, ea[l], he[l] * Math.sign(dot(ea[l], toF) || 1));
  let f0 = F.x.slice();
  for (let l = 0; l < 4; l++) if (l !== best.j && l !== best.k) f0 = addScaled(f0, fa[l], -hf[l] * Math.sign(dot(fa[l], toF) || 1));
  const [pe, pf] = closestSegSquare(e0, ea[best.i], he[best.i], f0, fa[best.j], fa[best.k], hf[best.j], hf[best.k]);
  out.push({ a: A, b: B, n, p: lerp(pe, pf, 0.5), depth: best.ov });
}

// closest points of the segment e0 + t·ed (|t| ≤ eh) and the square f0 + u·f1 + v·f2
function closestSegSquare(e0, ed, eh, f0, f1, f2, h1, h2) {
  let t = 0, u = 0, v = 0;
  const cl = (x, h) => Math.max(-h, Math.min(h, x));
  for (let it = 0; it < 16; it++) {
    const pf = addScaled(addScaled(f0, f1, u), f2, v);
    t = cl(dot(sub(pf, e0), ed), eh);
    const pe = addScaled(e0, ed, t), r = sub(pe, f0);
    u = cl(dot(r, f1), h1);
    v = cl(dot(r, f2), h2);
  }
  return [addScaled(e0, ed, t), addScaled(addScaled(f0, f1, u), f2, v)];
}

// Contact patch between the reference box's facet facing nOut and the incident
// box's facet facing back. Facets of a 4D box are cubes; their intersection is
// a polyhedron whose corners we sample with clipped incident edges plus the
// reference corners that lie under the incident cube.
function faceContacts(ref, inc, nOut, margin) {
  const ra = ref.axes, ia = inc.axes, hr = ref.shape.h, hi = inc.shape.h;
  let k = 0, km = -1;
  for (let i = 0; i < 4; i++) { const a = Math.abs(dot(ra[i], nOut)); if (a > km) { km = a; k = i; } }
  const c = addScaled(ref.x, ra[k], hr[k] * Math.sign(dot(ra[k], nOut)));
  let m = 0, mm = -1;
  for (let i = 0; i < 4; i++) { const a = Math.abs(dot(ia[i], nOut)); if (a > mm) { mm = a; m = i; } }
  const ci = addScaled(inc.x, ia[m], -hi[m] * Math.sign(dot(ia[m], nOut)));

  const pts = [];
  // incident cube corners and edges
  const io = [0, 1, 2, 3].filter(i => i !== m);
  const corner = bits => {
    let p = ci.slice();
    for (let q = 0; q < 3; q++) p = addScaled(p, ia[io[q]], (bits >> q & 1 ? 1 : -1) * hi[io[q]]);
    return p;
  };
  const icorners = [0, 1, 2, 3, 4, 5, 6, 7].map(corner);
  const ro = [0, 1, 2, 3].filter(i => i !== k);
  const cubeEdges = cs => {
    const e = [];
    for (let b = 0; b < 8; b++) for (let q = 0; q < 3; q++) if (!(b >> q & 1)) e.push([cs[b], cs[b | 1 << q]]);
    return e;
  };
  // incident edges clipped to the reference cell's side slabs
  for (const [p0, p1] of cubeEdges(icorners)) {
    const s = clipSeg(p0, p1, ref.x, ra, hr, ro);
    if (s) pts.push(...s);
  }
  // reference edges, projected along nOut onto the incident cell's hyperplane,
  // clipped to the incident cell's side slabs (catches corners where an
  // incident 2-face crosses a reference edge)
  const den = dot(nOut, ia[m]);
  if (Math.abs(den) > 1e-6) {
    const rcorners = [0, 1, 2, 3, 4, 5, 6, 7].map(b => {
      let v = c.slice();
      for (let q = 0; q < 3; q++) v = addScaled(v, ra[ro[q]], (b >> q & 1 ? 1 : -1) * hr[ro[q]]);
      return addScaled(v, nOut, dot(sub(ci, v), ia[m]) / den);
    });
    for (const [p0, p1] of cubeEdges(rcorners)) {
      const s = clipSeg(p0, p1, inc.x, ia, hi, io);
      if (s) pts.push(...s);
    }
  }

  const res = [];
  for (const q of pts) {
    const sep = dot(sub(q, c), nOut);
    if (sep >= margin) continue;
    const p = addScaled(q, nOut, -sep / 2);
    if (res.some(r => len(sub(r.p, p)) < 1e-4)) continue;
    res.push({ p, depth: -sep });
  }
  return reduce(res, 8);
}

// keep the deepest point, then repeatedly the point farthest from those kept
function reduce(pts, max) {
  if (pts.length <= max) return pts;
  let first = 0;
  for (let i = 1; i < pts.length; i++) if (pts[i].depth > pts[first].depth) first = i;
  const keep = [pts[first]], dmin = pts.map(q => len(sub(q.p, pts[first].p)));
  while (keep.length < max) {
    let j = 0;
    for (let i = 1; i < pts.length; i++) if (dmin[i] > dmin[j]) j = i;
    keep.push(pts[j]);
    for (let i = 0; i < pts.length; i++) dmin[i] = Math.min(dmin[i], len(sub(pts[i].p, pts[j].p)));
  }
  return keep;
}

// Liang–Barsky: the part of segment p0p1 inside the slabs |(p - c)·axes[j]| ≤ h[j], j ∈ js
function clipSeg(p0, p1, c, axes, h, js) {
  let t0 = 0, t1 = 1;
  for (const j of js) {
    const a0 = dot(sub(p0, c), axes[j]), da = dot(sub(p1, c), axes[j]) - a0;
    for (const sg of [1, -1]) {
      // keep sg·a(t) ≤ h
      const num = h[j] - sg * a0, den = sg * da;
      if (Math.abs(den) < 1e-12) { if (num < -1e-9) return null; continue; }
      const tt = num / den;
      if (den > 0) t1 = Math.min(t1, tt); else t0 = Math.max(t0, tt);
    }
  }
  return t0 <= t1 ? [lerp(p0, p1, t0), lerp(p0, p1, t1)] : null;
}
