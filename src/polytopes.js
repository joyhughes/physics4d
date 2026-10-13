// Convex polytopes as rigid-body shapes: the regular polychora (5-cell,
// 16-cell, 24-cell, 120-cell, 600-cell) and the hyperbox, all described the
// same way for collision and drawing.
//
// Every hull has local vertices, facet (cell) hyperplanes n·x = d, the vertex
// and edge lists of each facet, and each facet's neighbours across a 2-face.
// Volume and second moments come from an exact split into 4-simplices, one
// per flag (centre, cell centre, face centre, edge): ∫ x xᵀ over a simplex
// with one vertex at the origin is vol/30 (Σ vᵢvᵢᵀ + s sᵀ), s = Σ vᵢ.

import { dot, sub, len, normalize } from './math4.js';

const PHI = (1 + Math.sqrt(5)) / 2;

export const POLYTOPES = {
  '5-cell': { cells: 5, cell: 'tetrahedron', cellAdj: 'tetrahedral', verts: 5, schlafli: '{3,3,3}' },
  '16-cell': { cells: 16, cell: 'tetrahedron', cellAdj: 'tetrahedral', verts: 8, schlafli: '{3,3,4}' },
  '24-cell': { cells: 24, cell: 'octahedron', cellAdj: 'octahedral', verts: 24, schlafli: '{3,4,3}' },
  '120-cell': { cells: 120, cell: 'dodecahedron', cellAdj: 'dodecahedral', verts: 600, schlafli: '{5,3,3}' },
  '600-cell': { cells: 600, cell: 'tetrahedron', cellAdj: 'tetrahedral', verts: 120, schlafli: '{3,3,5}' },
};

function signs(base) {
  let out = [base.slice()];
  for (let i = 0; i < 4; i++) {
    if (base[i] === 0) continue;
    out = out.flatMap(v => { const w = v.slice(); w[i] = -w[i]; return [v, w]; });
  }
  return out;
}
function perms(base) {
  const out = [], seen = new Set();
  const go = (rest, acc) => {
    if (!rest.length) { const k = acc.join(); if (!seen.has(k)) { seen.add(k); out.push(acc); } return; }
    rest.forEach((x, i) => go(rest.filter((_, j) => j !== i), [...acc, x]));
  };
  go(base, []);
  return out;
}
function evenPerms(base) {
  const out = [];
  const idx = perms([0, 1, 2, 3]).filter(p => {
    let inv = 0;
    for (let i = 0; i < 4; i++) for (let j = i + 1; j < 4; j++) if (p[i] > p[j]) inv++;
    return inv % 2 === 0;
  });
  for (const p of idx) { const v = [0, 0, 0, 0]; for (let i = 0; i < 4; i++) v[p[i]] = base[i]; out.push(v); }
  return out;
}
const dedupe = pts => { const out = []; for (const p of pts) if (!out.some(q => dot(p, q) > 1 - 1e-9)) out.push(p); return out; };

function verts600() {
  const pts = [];
  for (let i = 0; i < 4; i++) for (const s of [1, -1]) { const v = [0, 0, 0, 0]; v[i] = s; pts.push(v); }
  pts.push(...signs([0.5, 0.5, 0.5, 0.5]));
  for (const v of evenPerms([PHI / 2, 0.5, 1 / (2 * PHI), 0])) pts.push(...signs(v));
  return dedupe(pts);
}
function verts24() { return dedupe(perms([1, 1, 0, 0]).flatMap(signs).map(normalize)); }
function verts16() { const o = []; for (let i = 0; i < 4; i++) for (const s of [1, -1]) { const v = [0, 0, 0, 0]; v[i] = s; o.push(v); } return o; }
function verts5() {
  // the 5-cell: (1,1,1,1)/√5-type construction, centred and on the unit sphere
  const s5 = Math.sqrt(5);
  const raw = [[1, 1, 1, -1 / s5], [1, -1, -1, -1 / s5], [-1, 1, -1, -1 / s5], [-1, -1, 1, -1 / s5], [0, 0, 0, 4 / s5]];
  return raw.map(normalize);
}

// vertices at unit circumradius, and the directions of their cell centres
function regular(kind) {
  switch (kind) {
    case '5-cell': { const V = verts5(); return { V, N: V.map(v => v.map(x => -x)) }; }
    case '16-cell': return { V: verts16(), N: signs([0.5, 0.5, 0.5, 0.5]) };
    case '24-cell': return { V: verts24(), N: [...verts16(), ...signs([0.5, 0.5, 0.5, 0.5])] };
    case '600-cell': { const V = verts600(); return { V, N: tetCentres(V) }; }
    case '120-cell': { const N = verts600(); return { V: tetCentres(N), N }; }
  }
  throw new Error('unknown polytope ' + kind);
}
// centres of the 600 tetrahedra of the 600-cell = vertex directions of the 120-cell
function tetCentres(V) {
  const n = V.length, e = PHI / 2, adj = V.map(() => new Set());
  for (let i = 0; i < n; i++) for (let j = i + 1; j < n; j++) if (Math.abs(dot(V[i], V[j]) - e) < 1e-6) { adj[i].add(j); adj[j].add(i); }
  const out = [];
  for (let a = 0; a < n; a++) for (const b of adj[a]) {
    if (b <= a) continue;
    for (const c of adj[b]) {
      if (c <= b || !adj[a].has(c)) continue;
      for (const d of adj[c]) if (d > c && adj[a].has(d) && adj[b].has(d)) out.push(normalize([0, 1, 2, 3].map(k => V[a][k] + V[b][k] + V[c][k] + V[d][k])));
    }
  }
  return out;
}

// Build hull data from vertices, unit facet normals and an edge list.
export function makeHull(V, N, edges, info = {}) {
  const d = N.map(n => Math.max(...V.map(v => dot(v, n))));
  const tol = 1e-7 * Math.max(...d);
  const facetVerts = N.map((n, f) => V.map((_, i) => i).filter(i => Math.abs(dot(V[i], n) - d[f]) < tol));
  const vf = V.map(() => []);
  facetVerts.forEach((fv, f) => fv.forEach(i => vf[i].push(f)));
  const inFacet = facetVerts.map(fv => new Set(fv));
  const ve = V.map(() => []);
  edges.forEach(([a, b], k) => { ve[a].push(k); ve[b].push(k); });
  const facetEdges = facetVerts.map((fv, f) => [...new Set(fv.flatMap(i => ve[i]))].filter(k => inFacet[f].has(edges[k][0]) && inFacet[f].has(edges[k][1])));
  // neighbours across a 2-face (≥ 3 shared vertices), with the face's vertices
  const adj = N.map(() => []), faces = [];
  for (let f = 0; f < N.length; f++) {
    const cand = new Map();
    for (const i of facetVerts[f]) for (const g of vf[i]) if (g > f) cand.set(g, (cand.get(g) ?? 0) + 1);
    for (const [g, c] of cand) {
      if (c < 3) continue;
      const shared = facetVerts[f].filter(i => inFacet[g].has(i));
      adj[f].push({ g, v: shared[0] }); adj[g].push({ g: f, v: shared[0] });
      faces.push({ cells: [f, g], verts: shared });
    }
  }
  // volume and second moments from flags
  const S = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
  let vol = 0;
  const mean = ids => [0, 1, 2, 3].map(k => ids.reduce((s, i) => s + V[i][k], 0) / ids.length);
  for (const F of faces) {
    const fc = mean(F.verts), fs = new Set(F.verts);
    const fedges = [...new Set(F.verts.flatMap(i => ve[i]))].map(k => edges[k]).filter(([a, b]) => fs.has(a) && fs.has(b));
    for (const c of F.cells) {
      const cc = mean(facetVerts[c]);
      for (const [a, b] of fedges) {
        const P = [cc, fc, V[a], V[b]], v = Math.abs(det4(P)) / 24;
        vol += v;
        const s = [0, 1, 2, 3].map(k => P.reduce((t, p) => t + p[k], 0));
        for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
          let t = s[i] * s[j];
          for (const p of P) t += p[i] * p[j];
          S[i * 4 + j] += v / 30 * t;
        }
      }
    }
  }
  // steepest slope it can rest on without tipping: resting on cell 0, the
  // centre must stay above the cell; tan θ = (cell centre to nearest ridge) / (centre to cell)
  let tip = Infinity, tipDir = null;
  const c0 = N[0].map(x => x * d[0]);
  for (const { g, v } of adj[0]) {
    const m0 = N[g], m = m0.map((x, k) => x - dot(m0, N[0]) * N[0][k]), l = len(m);
    const t = (dot(m, V[v]) - dot(m, c0)) / l;
    if (t < tip) { tip = t; tipDir = m.map(x => x / l); }
  }
  return {
    ...info, verts: V, normals: N, d, facetVerts, tipAngle: Math.atan(tip / d[0]), tipDir, facetEdges, edges, adj, nFaces: faces.length,
    vol, second: [S[0], S[5], S[10], S[15]].map(x => x / vol), // per unit mass, along each body axis
    radius: Math.max(...V.map(len)),
  };
}

function det4(P) {
  const m = P.map(r => r.slice());
  let det = 1;
  for (let c = 0; c < 4; c++) {
    let p = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(m[r][c]) > Math.abs(m[p][c])) p = r;
    if (Math.abs(m[p][c]) < 1e-300) return 0;
    if (p !== c) { [m[p], m[c]] = [m[c], m[p]]; det = -det; }
    det *= m[c][c];
    for (let r = c + 1; r < 4; r++) { const f = m[r][c] / m[c][c]; for (let k = c; k < 4; k++) m[r][k] -= f * m[c][k]; }
  }
  return det;
}

function minEdges(V) {
  let m = Infinity;
  for (let i = 0; i < V.length; i++) for (let j = i + 1; j < V.length; j++) m = Math.min(m, len(sub(V[i], V[j])));
  const out = [];
  for (let i = 0; i < V.length; i++) for (let j = i + 1; j < V.length; j++) if (len(sub(V[i], V[j])) < m * (1 + 1e-6)) out.push([i, j]);
  return out;
}

const unitCache = new Map();
function unitHull(kind) {
  if (!unitCache.has(kind)) {
    const { V, N } = regular(kind);
    unitCache.set(kind, makeHull(V, N, minEdges(V), { kind, ...POLYTOPES[kind] }));
  }
  return unitCache.get(kind);
}

// a regular polychoron of circumradius r
export function polyHull(kind, r) {
  const u = unitHull(kind);
  return {
    ...u, verts: u.verts.map(v => v.map(x => x * r)), d: u.d.map(x => x * r),
    vol: u.vol * r ** 4, second: u.second.map(x => x * r * r), radius: r,
    edgeLength: len(sub(u.verts[u.edges[0][0]], u.verts[u.edges[0][1]])) * r,
  };
}

// a hyperbox with half-sides h; vertex i has coordinate k = ±h[k] by bit k of i
export function boxHull(h) {
  const V = [];
  for (let b = 0; b < 16; b++) V.push([0, 1, 2, 3].map(k => (b >> k & 1 ? 1 : -1) * h[k]));
  const N = [];
  for (let k = 0; k < 4; k++) for (const s of [-1, 1]) { const n = [0, 0, 0, 0]; n[k] = s; N.push(n); }
  const E = [];
  for (let b = 0; b < 16; b++) for (let k = 0; k < 4; k++) if (!(b >> k & 1)) E.push([b, b | 1 << k]);
  return makeHull(V, N, E, { kind: 'box' });
}
