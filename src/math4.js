// 4D vector, bivector and rotation helpers.
//
// Vectors are plain arrays [x, y, z, w]. Rotation in 4D happens in planes, not
// about axes, so angular velocity, angular momentum and torque are bivectors:
// six components, one per coordinate plane, in the order of PAIRS. Rotations
// are 4×4 orthogonal matrices stored row-major in Float64Array(16), mapping
// body coordinates to world coordinates.
//
// Sign convention: a bivector ω moves the point r with velocity v = r·ω, that is
// v_b = Σ_a r_a ω_ab with ω_ba = -ω_ab. Positive ω_xy turns +x toward +y.

export const PAIRS = [[0, 1], [0, 2], [0, 3], [1, 2], [1, 3], [2, 3]];
export const PLANE_NAMES = ['xy', 'xz', 'xw', 'yz', 'yw', 'zw'];
export const AXIS_NAMES = ['x', 'y', 'z', 'w'];
// index of plane (a, b) in PAIRS
export const PLANE = { xy: 0, xz: 1, xw: 2, yz: 3, yw: 4, zw: 5 };

export const v4 = (x = 0, y = 0, z = 0, w = 0) => [x, y, z, w];
export const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2], a[3] + b[3]];
export const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2], a[3] - b[3]];
export const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s, a[3] * s];
export const addScaled = (a, b, s) => [a[0] + b[0] * s, a[1] + b[1] * s, a[2] + b[2] * s, a[3] + b[3] * s];
export const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
export const len = a => Math.sqrt(dot(a, a));
export const normalize = a => { const l = len(a); return l > 1e-300 ? scale(a, 1 / l) : [0, 0, 0, 0]; };
export const lerp = (a, b, t) => addScaled(a, sub(b, a), t);

// ---- bivectors (Array(6)) ----
export const biv = (xy = 0, xz = 0, xw = 0, yz = 0, yw = 0, zw = 0) => [xy, xz, xw, yz, yw, zw];
export const bzero = () => [0, 0, 0, 0, 0, 0];
export const badd = (a, b) => a.map((x, i) => x + b[i]);
export const bsub = (a, b) => a.map((x, i) => x - b[i]);
export const bscale = (a, s) => a.map(x => x * s);
export const bdot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3] + a[4] * b[4] + a[5] * b[5];
export const blen = a => Math.sqrt(bdot(a, a));

// a ∧ b
export function wedge(a, b) {
  const o = new Array(6);
  for (let k = 0; k < 6; k++) { const [i, j] = PAIRS[k]; o[k] = a[i] * b[j] - a[j] * b[i]; }
  return o;
}

// velocity r·ω of the point r under angular velocity ω
export function bapply(w, r) {
  const o = [0, 0, 0, 0];
  for (let k = 0; k < 6; k++) {
    const [a, b] = PAIRS[k];
    o[b] += r[a] * w[k];
    o[a] -= r[b] * w[k];
  }
  return o;
}

// B ∧ B / 2 as a number: zero exactly when B is a simple rotation (one plane).
// Nonzero means a double rotation: two orthogonal planes turning at once.
export const bwedgeSelf = B => B[0] * B[5] - B[1] * B[4] + B[2] * B[3];

// Split a bivector into its two orthogonal simple parts: B = s1·P1 + s2·P2,
// with P1, P2 unit simple bivectors in completely orthogonal planes, s1 ≥ s2 ≥ 0.
// Uses the self-dual / anti-self-dual decomposition of so(4).
export function bsplit(B) {
  const dual = b => [b[5], -b[4], b[3], b[2], -b[1], b[0]]; // Hodge star
  const D = dual(B);
  const P = bscale(badd(B, D), 0.5), M = bscale(bsub(B, D), 0.5);
  const p = blen(P) * Math.SQRT2, m = blen(M) * Math.SQRT2; // rates of the two isoclinic parts
  const s1 = (p + m) / 2, s2 = Math.abs(p - m) / 2;
  let P1 = null, P2 = null;
  if (p > 1e-12 && m > 1e-12) {
    const u = bscale(P, 1 / p), v = bscale(M, 1 / m); // each has |·|² = 1/2
    P1 = badd(u, v);                                     // unit simple
    P2 = p >= m ? bsub(u, v) : bsub(v, u);
  }
  return { s1, s2, P1, P2 };
}

// ---- 4×4 matrices (row-major Float64Array(16)) ----
export function mIdentity() { const m = new Float64Array(16); m[0] = m[5] = m[10] = m[15] = 1; return m; }
export function mMul(a, b) {
  const o = new Float64Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) {
    let s = 0; for (let k = 0; k < 4; k++) s += a[i * 4 + k] * b[k * 4 + j];
    o[i * 4 + j] = s;
  }
  return o;
}
export function mT(a) {
  const o = new Float64Array(16);
  for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) o[j * 4 + i] = a[i * 4 + j];
  return o;
}
export function mApply(m, v) {
  return [
    m[0] * v[0] + m[1] * v[1] + m[2] * v[2] + m[3] * v[3],
    m[4] * v[0] + m[5] * v[1] + m[6] * v[2] + m[7] * v[3],
    m[8] * v[0] + m[9] * v[1] + m[10] * v[2] + m[11] * v[3],
    m[12] * v[0] + m[13] * v[1] + m[14] * v[2] + m[15] * v[3],
  ];
}
export function mApplyT(m, v) {
  return [
    m[0] * v[0] + m[4] * v[1] + m[8] * v[2] + m[12] * v[3],
    m[1] * v[0] + m[5] * v[1] + m[9] * v[2] + m[13] * v[3],
    m[2] * v[0] + m[6] * v[1] + m[10] * v[2] + m[14] * v[3],
    m[3] * v[0] + m[7] * v[1] + m[11] * v[2] + m[15] * v[3],
  ];
}
export const mCol = (m, j) => [m[j], m[4 + j], m[8 + j], m[12 + j]];

// rotation by angle t in the plane (a, b): turns +a toward +b
export function mPlane(a, b, t) {
  const m = mIdentity(), c = Math.cos(t), s = Math.sin(t);
  m[a * 4 + a] = c; m[b * 4 + b] = c;
  m[b * 4 + a] = s; m[a * 4 + b] = -s;
  return m;
}

// rotation exp(ω) for a bivector ω: scaling and squaring of the Taylor series
// (works for simple, double and isoclinic rotations alike)
export function mExp(w) {
  const W = skew(w);
  let n = 0, l = blen(w);
  while (l > 0.25) { l /= 2; n++; }
  const f = 1 / (1 << n);
  for (let i = 0; i < 16; i++) W[i] *= f;
  let term = mIdentity(), m = mIdentity();
  for (let k = 1; k <= 10; k++) {
    term = mMul(term, W);
    for (let i = 0; i < 16; i++) { term[i] /= k; m[i] += term[i]; }
  }
  for (let i = 0; i < n; i++) m = mMul(m, m);
  return orthonormalize(m);
}

// matrix W with W r = r·ω
export function skew(w) {
  const m = new Float64Array(16);
  for (let k = 0; k < 6; k++) {
    const [a, b] = PAIRS[k];
    m[b * 4 + a] += w[k];
    m[a * 4 + b] -= w[k];
  }
  return m;
}

// R ← cay(W h) R = (I - W h/2)⁻¹ (I + W h/2) R; exactly orthogonal.
export function cayleyStep(R, w, h) {
  const W = skew(w), A = new Float64Array(16), B = new Float64Array(16);
  for (let i = 0; i < 16; i++) { A[i] = -W[i] * h / 2; B[i] = W[i] * h / 2; }
  for (let i = 0; i < 4; i++) { A[i * 5] += 1; B[i * 5] += 1; }
  return solve4(A, mMul(B, R));
}

// solve A X = B for 4×4 A and B (Gaussian elimination with partial pivoting)
export function solve4(A, B) {
  const a = Float64Array.from(A), x = Float64Array.from(B);
  for (let c = 0; c < 4; c++) {
    let p = c;
    for (let r = c + 1; r < 4; r++) if (Math.abs(a[r * 4 + c]) > Math.abs(a[p * 4 + c])) p = r;
    if (p !== c) for (let j = 0; j < 4; j++) {
      [a[c * 4 + j], a[p * 4 + j]] = [a[p * 4 + j], a[c * 4 + j]];
      [x[c * 4 + j], x[p * 4 + j]] = [x[p * 4 + j], x[c * 4 + j]];
    }
    const d = a[c * 4 + c];
    for (let r = 0; r < 4; r++) {
      if (r === c) continue;
      const f = a[r * 4 + c] / d;
      if (!f) continue;
      for (let j = 0; j < 4; j++) { a[r * 4 + j] -= f * a[c * 4 + j]; x[r * 4 + j] -= f * x[c * 4 + j]; }
    }
  }
  for (let r = 0; r < 4; r++) { const d = a[r * 5]; for (let j = 0; j < 4; j++) x[r * 4 + j] /= d; }
  return x;
}

// Gram–Schmidt on the columns, to keep R orthogonal against round-off
export function orthonormalize(R) {
  const cols = [0, 1, 2, 3].map(j => mCol(R, j));
  for (let j = 0; j < 4; j++) {
    for (let k = 0; k < j; k++) cols[j] = addScaled(cols[j], cols[k], -dot(cols[j], cols[k]));
    cols[j] = normalize(cols[j]);
  }
  const o = new Float64Array(16);
  for (let j = 0; j < 4; j++) for (let i = 0; i < 4; i++) o[i * 4 + j] = cols[j][i];
  return o;
}

// The 6×6 matrix C (row-major Float64Array(36)) that rotates bivectors:
// (R a) ∧ (R b) = C (a ∧ b). Entries are 2×2 minors of R.
export function compound(R) {
  const C = new Float64Array(36);
  for (let k = 0; k < 6; k++) {
    const [a, b] = PAIRS[k];
    for (let l = 0; l < 6; l++) {
      const [c, d] = PAIRS[l];
      C[k * 6 + l] = R[a * 4 + c] * R[b * 4 + d] - R[a * 4 + d] * R[b * 4 + c];
    }
  }
  return C;
}
export function m6Apply(M, x) {
  const o = new Array(6);
  for (let i = 0; i < 6; i++) { let s = 0; for (let j = 0; j < 6; j++) s += M[i * 6 + j] * x[j]; o[i] = s; }
  return o;
}

// generalized cross product: a vector orthogonal to a, b and c
export function cross3(a, b, c) {
  const d3 = (i, j, k) =>
    a[i] * (b[j] * c[k] - b[k] * c[j]) - a[j] * (b[i] * c[k] - b[k] * c[i]) + a[k] * (b[i] * c[j] - b[j] * c[i]);
  return [d3(1, 2, 3), -d3(0, 2, 3), d3(0, 1, 3), -d3(0, 1, 2)];
}

// solve the n×n system A x = b in place (A as array of rows); returns x
export function solveN(A, b) {
  const n = b.length, a = A.map(r => r.slice()), x = b.slice();
  for (let c = 0; c < n; c++) {
    let p = c;
    for (let r = c + 1; r < n; r++) if (Math.abs(a[r][c]) > Math.abs(a[p][c])) p = r;
    [a[c], a[p]] = [a[p], a[c]]; [x[c], x[p]] = [x[p], x[c]];
    const d = a[c][c];
    if (Math.abs(d) < 1e-14) continue;
    for (let r = 0; r < n; r++) {
      if (r === c) continue;
      const f = a[r][c] / d;
      if (!f) continue;
      for (let j = c; j < n; j++) a[r][j] -= f * a[c][j];
      x[r] -= f * x[c];
    }
  }
  for (let r = 0; r < n; r++) x[r] = Math.abs(a[r][r]) < 1e-14 ? 0 : x[r] / a[r][r];
  return x;
}

// small seeded PRNG so scenes are reproducible
export function rng(seed = 1) {
  let s = seed >>> 0 || 1;
  return () => { s ^= s << 13; s >>>= 0; s ^= s >>> 17; s ^= s << 5; s >>>= 0; return s / 4294967296; };
}
export function randomRotation(rand) {
  // compose random rotations in all six planes, then re-orthonormalize
  let R = mIdentity();
  for (let k = 0; k < 6; k++) R = mMul(mPlane(...PAIRS[k], (rand() * 2 - 1) * Math.PI), R);
  return orthonormalize(R);
}
