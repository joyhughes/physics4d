// Run every scene headless: timing, NaN check, energy rise, final readouts.
import { World } from '../src/world.js';
import { SCENES } from '../src/scenes.js';
const only = process.argv[2];
for (const s of SCENES) {
  if (only && s.id !== only) continue;
  const p = structuredClone(s.params ?? {});
  const w = new World(s.world ?? {});
  const hooks = s.build(w, p) ?? {};
  const E0 = w.energy().total;
  let emin = E0, rise = 0, bad = false;
  const t0 = performance.now(), steps = +(process.argv[3] ?? 600);
  for (let i = 0; i < steps; i++) {
    w.step(1 / 60); hooks.update?.(w);
    const E = w.energy().total;
    if (!isFinite(E)) { bad = true; break; }
    rise = Math.max(rise, E - emin); emin = Math.min(emin, E);
  }
  const ms = (performance.now() - t0) / steps;
  const m = w.bodies.filter(b => !b.static).reduce((a, b) => a + b.mass, 0);
  console.log(`\n${s.id}: ${ms.toFixed(2)} ms/frame, ${bad ? 'NaN!' : 'ok'}, energy ${E0.toFixed(1)} → ${w.energy().total.toFixed(1)} J, max rise ${(rise / m).toFixed(4)} J/kg`);
  for (const [k, v] of hooks.readouts?.(w) ?? []) console.log('   ', k, '=', v);
}
