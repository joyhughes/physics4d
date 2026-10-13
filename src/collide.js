// Narrowphase collision in 4D: planes (static half-spaces), hyperspheres,
// hyperboxes (tesseracts with any side lengths) and convex polychora.
//
// Every contact is { a, b, n, p, depth }: n is the unit normal that pushes a away
// from b, p the world contact point, depth the penetration (negative = gap).
//
// Box–box uses the separating axis test. In 4D the candidate axes are the 4 + 4
// facet normals, plus the normals of every edge of one box against every 2-face
// of the other (a vector orthogonal to three directions), 48 more. A facet axis
// gives a contact patch (edges of the incident cell clipped to the reference
// cell); an edge–face axis gives one point.
//
// Other convex pairs (polychoron with polychoron or box) test the cell normals
// of both shapes as separating axes, confirm with GJK (exact distance between
// convex hulls), and clip the best cell against the other shape's most opposed
// cell, as for boxes. Where neither shape's cell is the true contact normal
// (edge against 2-face), GJK's closest points give one contact point.

import { add, sub, scale, addScaled, dot, len, cross3, lerp } from './math4.js';

export function collidePair(A, B, out, margin) {
  const ta = A.shape.type, tb = B.shape.type;
  if (ta === 'plane') { if (tb !== 'plane') collidePair(B, A, out, margin); return; }
  if (tb === 'plane') return ta === 'sphere' ? spherePlane(A, B, out, margin) : hullPlane(A, B, out, margin);
  if (ta === 'sphere' && tb === 'sphere') return sphereSphere(A, B, out, margin);
  if (ta === 'sphere') return tb === 'box' ? sphereBox(A, B, out, margin) : sphereHull(A, B, out, margin);
  if (tb === 'sphere') return ta === 'box' ? sphereBox(B, A, out, margin) : sphereHull(B, A, out, margin);
  if (ta === 'box' && tb === 'box') return boxBox(A, B, out, margin);
  return hullHull(A, B, out, margin);
}

function spherePlane(S, P, out, margin) {
  const n = P.shape.n, dist = dot(n, S.x) - P.shape.d - S.shape.r;
  if (dist < margin) out.push({ a: S, b: P, n, p: addScaled(S.x, n, -S.shape.r - dist / 2), depth: -dist });
}

function hullPlane(Bx, P, out, margin) {
  const n = P.shape.n, res = [];
  if (dot(n, Bx.x) - P.shape.d > Bx.radius + margin) return;
  for (const v of Bx.verts) {
    const dist = dot(n, v) - P.shape.d;
    if (dist < margin) res.push({ p: addScaled(v, n, -dist / 2), depth: -dist });
  }
  for (const c of reduce(res, 8)) out.push({ a: Bx, b: P, n, p: c.p, depth: c.depth });
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

// ---- general convex hulls ----

function sphereHull(S, H, out, margin) {
  const c = S.x, r = S.shape.r;
  let fm = 0, sm = -Infinity;
  for (let f = 0; f < H.fn.length; f++) { const s = dot(H.fn[f], c) - H.fd[f]; if (s > sm) { sm = s; fm = f; } }
  if (sm > r + margin) return;
  if (sm <= 0) { // centre inside: push out through the nearest cell
    const n = H.fn[fm];
    out.push({ a: S, b: H, n, p: addScaled(c, n, -r), depth: r - sm });
    return;
  }
  const g = gjk(hullSupport(H), d => c);
  const dd = sub(c, g.pa), l = len(dd), depth = r - l;
  if (depth < -margin) return;
  const n = l > 1e-9 ? scale(dd, 1 / l) : H.fn[fm];
  out.push({ a: S, b: H, n, p: addScaled(g.pa, n, -depth / 2), depth });
}

const hullSupport = B => d => {
  const V = B.verts;
  let k = 0, m = -Infinity;
  for (let i = 0; i < V.length; i++) { const s = dot(V[i], d); if (s > m) { m = s; k = i; } }
  return V[k];
};

function hullHull(A, B, out, margin) {
  // separating axes: the cell normals of both shapes. The incident centre c
  // bounds each overlap from below (min over vertices of n·v ≤ n·c), so the
  // cell with the lowest bound is tried first, then only cells that could beat it.
  let best = null;
  for (const [ref, inc] of [[A, B], [B, A]]) {
    const V = inc.verts, c = inc.x, N = ref.fn, D = ref.fd, nf = N.length, lb = new Float64Array(nf);
    let f0 = 0;
    for (let f = 0; f < nf; f++) {
      const n = N[f];
      lb[f] = D[f] - (n[0] * c[0] + n[1] * c[1] + n[2] * c[2] + n[3] * c[3]);
      if (lb[f] < lb[f0]) f0 = f;
    }
    for (let k = -1; k < nf; k++) {
      const f = k < 0 ? f0 : k;
      if (k === f0 || (best && lb[f] >= best.ov)) continue;
      const n = N[f], n0 = n[0], n1 = n[1], n2 = n[2], n3 = n[3];
      let m = Infinity;
      for (let i = 0; i < V.length; i++) { const v = V[i], s = v[0] * n0 + v[1] * n1 + v[2] * n2 + v[3] * n3; if (s < m) m = s; }
      const ov = D[f] - m;
      if (ov < -margin) return;
      if (!best || ov < best.ov) best = { ov, ref, inc, f };
    }
  }
  const g = gjk(hullSupport(A), hullSupport(B));
  if (g.dist > margin) return;
  const { ref, inc, f } = best, nOut = ref.fn[f]; // out of ref, toward inc
  const sgn = ref === B ? 1 : -1; // contact normal pushes A away from B
  if (g.dist > 1e-7) {
    const gn = scale(sub(g.pa, g.pb), 1 / g.dist); // from B toward A
    if (dot(gn, nOut) * sgn < 0.995) {
      out.push({ a: A, b: B, n: gn, p: lerp(g.pa, g.pb, 0.5), depth: -g.dist });
      return;
    }
  }
  const n = scale(nOut, sgn);
  const pts = cellContacts(ref, f, inc, nOut, margin);
  if (pts.length) { for (const c of pts) out.push({ a: A, b: B, n, p: c.p, depth: c.depth }); return; }
  // no clipped points: use the incident shape's deepest vertex
  const v = hullSupport(inc)(scale(nOut, -1)), depth = ref.fd[f] - dot(nOut, v);
  out.push({ a: A, b: B, n, p: addScaled(v, nOut, depth / 2), depth });
}

// contact patch between cell f of ref and the most opposed cell of inc
function cellContacts(ref, f, inc, nOut, margin) {
  const RH = ref.hull, IH = inc.hull;
  let g = 0, gm = Infinity;
  for (let k = 0; k < inc.fn.length; k++) { const s = dot(inc.fn[k], nOut); if (s < gm) { gm = s; g = k; } }
  // the prism over the reference cell: its neighbours' hyperplanes, made parallel to nOut
  const refSides = RH.adj[f].map(({ g: h, v }) => {
    const m0 = ref.fn[h], m = sub(m0, scale(nOut, dot(m0, nOut))), l = len(m);
    return l < 1e-9 ? null : { m: scale(m, 1 / l), o: dot(scale(m, 1 / l), ref.verts[v]) };
  }).filter(Boolean);
  const incSides = IH.adj[g].map(({ g: h }) => ({ m: inc.fn[h], o: inc.fd[h] }));
  const pts = [];
  for (const k of IH.facetEdges[g]) {
    const [i, j] = IH.edges[k], seg = clipHalf(inc.verts[i], inc.verts[j], refSides);
    if (seg) pts.push(...seg);
  }
  // reference edges, carried along nOut onto the incident cell's hyperplane
  const ng = inc.fn[g], den = dot(ng, nOut);
  if (Math.abs(den) > 1e-6) {
    const proj = v => addScaled(v, nOut, (inc.fd[g] - dot(ng, v)) / den);
    for (const k of RH.facetEdges[f]) {
      const [i, j] = RH.edges[k], seg = clipHalf(proj(ref.verts[i]), proj(ref.verts[j]), incSides);
      if (seg) pts.push(...seg);
    }
  }
  const res = [];
  for (const q of pts) {
    const sep = dot(q, nOut) - ref.fd[f];
    if (sep >= margin) continue;
    const p = addScaled(q, nOut, -sep / 2);
    if (res.some(r => len(sub(r.p, p)) < 1e-4)) continue;
    res.push({ p, depth: -sep });
  }
  return reduce(res, 8);
}

// the part of segment p0p1 with m·x ≤ o for every side
function clipHalf(p0, p1, sides) {
  let t0 = 0, t1 = 1;
  for (const { m, o } of sides) {
    const a0 = dot(m, p0) - o, da = dot(m, p1) - o - a0; // want a0 + t·da ≤ 0
    if (Math.abs(da) < 1e-12) { if (a0 > 1e-9) return null; continue; }
    const t = -a0 / da;
    if (da > 0) t1 = Math.min(t1, t); else t0 = Math.max(t0, t);
    if (t0 > t1) return null;
  }
  return [lerp(p0, p1, t0), lerp(p0, p1, t1)];
}

// GJK distance between two convex sets given by support functions.
// Returns the distance and closest points (dist 0 when they overlap).
export function gjk(supA, supB) {
  const sup = d => { const a = supA(d), b = supB(scale(d, -1)); return { a, b, p: sub(a, b) }; };
  let simplex = [sup([1, 0, 0, 0])], v = simplex[0].p, lam = [1];
  for (let it = 0; it < 64; it++) {
    const vv = dot(v, v);
    if (vv < 1e-18) return { dist: 0, pa: simplex[0].a, pb: simplex[0].b };
    const w = sup(scale(v, -1));
    if (vv - dot(v, w.p) <= 1e-12 * Math.max(1, vv)) break;
    if (simplex.some(s => len(sub(s.p, w.p)) < 1e-12)) break;
    simplex.push(w);
    const c = closestOnSimplex(simplex.map(s => s.p), simplex.length - 1);
    simplex = c.idx.map(i => simplex[i]); lam = c.lam; v = c.v;
    if (simplex.length === 5) return { dist: 0, pa: v, pb: v };
  }
  let pa = [0, 0, 0, 0], pb = [0, 0, 0, 0];
  simplex.forEach((s, i) => { pa = addScaled(pa, s.a, lam[i]); pb = addScaled(pb, s.b, lam[i]); });
  return { dist: len(v), pa, pb };
}

// closest point to the origin on the hull of up to 5 points: try every subset
// that holds the newest point (the old ones alone were already searched)
const GD = new Float64Array(16), GG = new Float64Array(20), GMU = new Float64Array(4);
function closestOnSimplex(P, must) {
  const n = P.length;
  let best = null;
  const idx = [0, 0, 0, 0, 0];
  for (let mask = 1; mask < 1 << n; mask++) {
    if (!(mask >> must & 1)) continue;
    let m = 0;
    for (let i = 0; i < n; i++) if (mask >> i & 1) idx[m++] = i;
    const p0 = P[idx[0]], k = m - 1;
    for (let i = 0; i < k; i++) { const q = P[idx[i + 1]]; for (let c = 0; c < 4; c++) GD[i * 4 + c] = q[c] - p0[c]; }
    // Gram system G μ = −Dᵀ p0, as an augmented k × (k+1) matrix
    for (let i = 0; i < k; i++) {
      for (let j = 0; j < k; j++) { let t = 0; for (let c = 0; c < 4; c++) t += GD[i * 4 + c] * GD[j * 4 + c]; GG[i * 5 + j] = t; }
      let t = 0; for (let c = 0; c < 4; c++) t -= GD[i * 4 + c] * p0[c]; GG[i * 5 + k] = t;
    }
    if (k && !gaussInPlace(k)) continue;
    let l0 = 1, ok = true;
    for (let i = 0; i < k; i++) { l0 -= GMU[i]; if (GMU[i] < -1e-12) ok = false; }
    if (!ok || l0 < -1e-12) continue;
    const v = p0.slice();
    for (let i = 0; i < k; i++) for (let c = 0; c < 4; c++) v[c] += GD[i * 4 + c] * GMU[i];
    const vv = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3];
    if (!best || vv < best.vv - 1e-15) {
      const lam = [l0]; for (let i = 0; i < k; i++) lam.push(GMU[i]);
      best = { vv, v, idx: idx.slice(0, m), lam };
    }
  }
  return best;
}

// solve the k × k system in GG (row stride 5, right-hand side in column k) into GMU
function gaussInPlace(k) {
  let sc = 1e-300;
  for (let i = 0; i < k; i++) sc = Math.max(sc, Math.abs(GG[i * 5 + i]));
  for (let c = 0; c < k; c++) {
    let p = c;
    for (let r = c + 1; r < k; r++) if (Math.abs(GG[r * 5 + c]) > Math.abs(GG[p * 5 + c])) p = r;
    if (Math.abs(GG[p * 5 + c]) < 1e-10 * sc) return false;
    if (p !== c) for (let j = 0; j <= k; j++) { const t = GG[c * 5 + j]; GG[c * 5 + j] = GG[p * 5 + j]; GG[p * 5 + j] = t; }
    for (let r = c + 1; r < k; r++) { const f = GG[r * 5 + c] / GG[c * 5 + c]; for (let j = c; j <= k; j++) GG[r * 5 + j] -= f * GG[c * 5 + j]; }
  }
  for (let r = k - 1; r >= 0; r--) { let t = GG[r * 5 + k]; for (let j = r + 1; j < k; j++) t -= GG[r * 5 + j] * GMU[j]; GMU[r] = t / GG[r * 5 + r]; }
  return true;
}
