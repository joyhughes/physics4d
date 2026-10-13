// Which of a hyperbox's six principal rotation planes are stable spins?
// Spin in one plane plus a tiny wobble; report how far the body-frame
// angular velocity strays from that plane (1 = left it completely).
import { World, Body } from '../src/world.js';
import { PLANE_NAMES, m6Apply, blen } from '../src/math4.js';
const h = [0.5, 0.4, 0.3, 0.2];
for (let k = 0; k < 6; k++) {
  const w = new World({ gravity: [0, 0, 0, 0], ground: false });
  const om = [0.01, -0.012, 0.011, 0.009, -0.01, 0.013]; om[k] = 3;
  const b = w.add(new Body({ shape: { type: 'box', h }, omega: om }));
  let worst = 0;
  for (let i = 0; i < 1800; i++) {
    w.step();
    // body-frame angular velocity = Cᵀ w
    const wb = []; for (let r = 0; r < 6; r++) { let s = 0; for (let c = 0; c < 6; c++) s += b.C[c * 6 + r] * b.w[c]; wb.push(s); }
    worst = Math.max(worst, 1 - Math.abs(wb[k]) / blen(wb));
  }
  console.log(PLANE_NAMES[k], 'J=' + b.J[k].toFixed(1), 'max stray', worst.toFixed(4), worst > 0.05 ? 'UNSTABLE' : 'stable');
}
