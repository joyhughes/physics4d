// App wiring: scene list, time controls, slice and tilt, readouts, w-map, chart.
import { World } from './world.js';
import { View } from './view.js';
import { SCENES, describeRotation } from './scenes.js';
import { PLANE_NAMES, len, bwedgeSelf, bzero, mApply } from './math4.js';

const $ = id => document.getElementById(id);
let view;
try { view = new View($('view')); }
catch (e) { throw new Error('WebGL is unavailable in this browser (' + e.message + '). Check that hardware acceleration is on.'); }
const fx = (x, d = 2) => (Math.abs(x) < 0.5 * 10 ** -d ? 0 : x).toFixed(d);

let scene, world, hooks = {}, running = true, speed = 1, acc = 0, history = [];
const params = {};

const dark = () => getComputedStyle(document.documentElement).colorScheme.includes('dark');
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const applyTheme = () => view.setTheme(dark());
matchMedia('(prefers-color-scheme: dark)').addEventListener('change', applyTheme);
new MutationObserver(applyTheme).observe(document.documentElement, { attributes: true, attributeFilter: ['data-theme'] });
applyTheme();

// ---- scene list
for (const s of SCENES) {
  const b = document.createElement('button');
  b.innerHTML = `<small>${s.kicker}</small>${s.title.replace(/^(.{0,34}\S*).*$/, '$1')}`;
  b.dataset.id = s.id;
  b.onclick = () => load(s.id);
  $('scenes').append(b);
}

function load(id, keepView = false) {
  scene = SCENES.find(s => s.id === id) ?? SCENES[0];
  params[scene.id] ??= structuredClone(scene.params ?? {});
  world = new World(scene.world ?? {});
  hooks = scene.build(world, params[scene.id]) ?? {};
  view.setWorld(world);
  view.selected = null;
  history = [];
  acc = 0;
  for (const b of document.querySelectorAll('#scenes button')) b.classList.toggle('on', b.dataset.id === scene.id);
  $('kicker').textContent = scene.kicker;
  $('stitle').textContent = scene.title;
  $('blurb').textContent = scene.blurb;
  $('notice').replaceChildren(...scene.notice.map(t => Object.assign(document.createElement('li'), { textContent: t })));
  buildControls();
  if (!keepView) {
    const v = scene.view ?? {};
    view.camera.position.set(...(v.cam ?? [5, 3.5, 7]));
    view.controls.target.set(...(v.target ?? [0, 0.6, 0]));
    setSlice(v.slice ?? 0);
    setTilt(...(v.tilt ?? [0, 0]));
  }
  pick(null);
  try { location.hash = scene.id; } catch {}
  updatePanel(true);
}

const ctx = { reset: () => load(scene.id, true) };
function buildControls() {
  const box = $('controls');
  box.replaceChildren();
  for (const c of hooks.controls ?? []) {
    if (c.options) {
      const l = document.createElement('label'), s = document.createElement('select');
      s.id = 'ctl-' + scene.id + '-' + c.label.replace(/\W+/g, '');
      for (const [v, t] of c.options) s.append(new Option(t, v));
      s.value = c.value;
      s.onchange = () => c.set(ctx, s.value);
      l.append(c.label, s);
      box.append(l);
    } else {
      const b = document.createElement('button');
      b.textContent = c.label;
      b.onclick = () => c.run(ctx);
      box.append(b);
    }
  }
}

// ---- time
$('play').onclick = () => setRunning(!running);
function setRunning(r) { running = r; $('play').textContent = r ? 'Pause' : 'Play'; }
$('step').onclick = () => { setRunning(false); stepOnce(); };
$('reset').onclick = () => ctx.reset();
for (const b of $('speed').children) b.onclick = () => {
  speed = +b.dataset.s;
  for (const x of $('speed').children) x.classList.toggle('on', x === b);
};
function stepOnce() {
  world.step(1 / 60);
  hooks.update?.(world);
  view.recordTrails();
  sample();
}

// ---- slice and tilt
function setSlice(s) {
  view.s0 = Math.max(-3, Math.min(3, s));
  $('slice').value = view.s0;
  $('sliceOut').textContent = fx(view.s0) + ' m';
}
function setTilt(a, b) {
  $('tiltxw').value = a; $('tiltzw').value = b;
  $('tiltxwOut').textContent = a + '°'; $('tiltzwOut').textContent = b + '°';
  view.setTilt(a * Math.PI / 180, b * Math.PI / 180);
  $('mapaxes').textContent = a || b ? 'x′ → · w′ ↑' : 'x → · w ↑';
}
$('slice').oninput = e => setSlice(+e.target.value);
$('tiltxw').oninput = $('tiltzw').oninput = () => setTilt(+$('tiltxw').value, +$('tiltzw').value);
$('ghosts').onchange = e => { view.showGhosts = e.target.checked; };
$('trails').onchange = e => { view.showTrails = e.target.checked; };
addEventListener('keydown', e => {
  if (e.target.closest?.('select, input')) return;
  if (e.key === ' ') { e.preventDefault(); setRunning(!running); }
  else if (e.key === '[') setSlice(view.s0 - 0.05);
  else if (e.key === ']') setSlice(view.s0 + 0.05);
  else if (e.key === 'r' || e.key === 'R') ctx.reset();
});

// ---- w-map: the x′–w′ plane seen from above; the slice is a horizontal line
const map = $('map'), mg = map.getContext('2d');
let dragging = false;
const mapRange = () => scene.view?.map ?? 3;
const toMap = (x, w) => { const R = mapRange(), S = map.width; return [(x / R + 1) * S / 2, (1 - w / R) * S / 2]; };
const fromMapY = y => (1 - 2 * y / map.getBoundingClientRect().height) * mapRange();
map.addEventListener('pointerdown', e => { dragging = true; map.setPointerCapture(e.pointerId); setSlice(fromMapY(e.offsetY)); });
map.addEventListener('pointermove', e => { if (dragging) setSlice(fromMapY(e.offsetY)); });
map.addEventListener('pointerup', () => { dragging = false; });

function hex(c) { return '#' + c.toString(16).padStart(6, '0'); }
function drawMap() {
  const S = map.width, R = mapRange();
  mg.clearRect(0, 0, S, S);
  mg.fillStyle = css('--raise'); mg.fillRect(0, 0, S, S);
  // metre grid
  mg.strokeStyle = css('--line'); mg.lineWidth = 1;
  for (let m = -Math.floor(R); m <= R; m++) {
    const [a] = toMap(m, 0), [, b] = toMap(0, m);
    mg.beginPath(); mg.moveTo(a, 0); mg.lineTo(a, S); mg.moveTo(0, b); mg.lineTo(S, b); mg.stroke();
  }
  // trails
  for (const it of view.items) {
    const h = it.body.user.history;
    if (!h || !view.showTrails || h.length < 2) continue;
    mg.strokeStyle = hex(it.body.color); mg.globalAlpha = 0.6; mg.lineWidth = 2;
    mg.beginPath();
    h.forEach((p, i) => { const q = view.toView(p), [x, y] = toMap(q[0], q[3]); i ? mg.lineTo(x, y) : mg.moveTo(x, y); });
    mg.stroke(); mg.globalAlpha = 1;
  }
  // bodies
  for (const it of view.items) {
    const b = it.body;
    mg.fillStyle = hex(b.color);
    mg.globalAlpha = b.static ? 0.35 : 0.85;
    if (b.shape.type === 'sphere') {
      const q = view.toView(b.x), [x, y] = toMap(q[0], q[3]);
      mg.beginPath(); mg.arc(x, y, Math.max(2, b.shape.r / R * S / 2), 0, 7); mg.fill();
    } else {
      const pts = hull(b.verts.map(p => { const q = view.toView(p); return toMap(q[0], q[3]); }));
      mg.beginPath(); pts.forEach(([x, y], i) => i ? mg.lineTo(x, y) : mg.moveTo(x, y)); mg.closePath(); mg.fill();
    }
    if (b === view.selected) { mg.globalAlpha = 1; mg.strokeStyle = css('--fg'); mg.lineWidth = 2; mg.stroke(); }
    mg.globalAlpha = 1;
  }
  // slice line
  const [, ys] = toMap(0, view.s0);
  mg.strokeStyle = css('--accent'); mg.lineWidth = 3;
  mg.beginPath(); mg.moveTo(0, ys); mg.lineTo(S, ys); mg.stroke();
  mg.fillStyle = css('--accent'); mg.font = '600 22px "JetBrains Mono", monospace';
  mg.fillText('slice', 8, ys - 8);
}
function hull(p) {
  p = p.slice().sort((a, b) => a[0] - b[0] || a[1] - b[1]);
  const cr = (o, a, b) => (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0]);
  const lo = [], up = [];
  for (const q of p) { while (lo.length > 1 && cr(lo.at(-2), lo.at(-1), q) <= 0) lo.pop(); lo.push(q); }
  for (const q of p.reverse()) { while (up.length > 1 && cr(up.at(-2), up.at(-1), q) <= 0) up.pop(); up.push(q); }
  return lo.slice(0, -1).concat(up.slice(0, -1));
}

// ---- chart: scene series, or the energy budget by default
function chartData() {
  if (hooks.chart) return hooks.chart(world);
  const e = world.energy();
  return { label: 'energy (J)', series: [
    { name: 'total', color: css('--fg'), value: e.total },
    { name: 'kinetic, moving', color: '#f0a040', value: e.lin },
    { name: 'kinetic, turning', color: '#40b8e8', value: e.rot },
    { name: 'potential', color: '#6cbf6a', value: e.pot },
  ] };
}
function sample() {
  history.push(chartData().series.map(s => s.value));
  if (history.length > 900) history.shift();
}
function drawChart() {
  const c = $('chart'), g = c.getContext('2d'), W = c.width, H = c.height, d = chartData();
  g.clearRect(0, 0, W, H);
  if (!history.length) return;
  let lo = Infinity, hi = -Infinity;
  for (const row of history) for (const v of row) { lo = Math.min(lo, v); hi = Math.max(hi, v); }
  if (hi - lo < 1e-6) { hi += 1; lo -= 1; }
  const pad = (hi - lo) * 0.08; lo -= pad; hi += pad;
  const X = i => 8 + (W - 16) * i / 899, Y = v => H - 24 - (H - 40) * (v - lo) / (hi - lo);
  g.strokeStyle = css('--line'); g.lineWidth = 1;
  if (lo < 0 && hi > 0) { g.beginPath(); g.moveTo(0, Y(0)); g.lineTo(W, Y(0)); g.stroke(); }
  d.series.forEach((s, k) => {
    g.strokeStyle = s.color; g.lineWidth = k === 0 && !hooks.chart ? 3 : 2;
    g.beginPath();
    history.forEach((row, i) => { const x = X(i), y = Y(row[k]); i ? g.lineTo(x, y) : g.moveTo(x, y); });
    g.stroke();
  });
  g.fillStyle = css('--muted'); g.font = '500 20px "JetBrains Mono", monospace';
  g.fillText(d.label + '   ' + fx(hi, 1) + ' … ' + fx(lo, 1), 10, H - 4);
}

// ---- panel numbers
let lastPanel = 0, keyFor = '';
function dl(el, rows) { el.innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join(''); }
function updatePanel(force) {
  const now = performance.now();
  if (!force && now - lastPanel < 120) return;
  lastPanel = now;
  $('clock').textContent = 't = ' + fx(world.time) + ' s';
  dl($('readouts'), hooks.readouts?.(world) ?? []);
  const e = world.energy(), m = world.momentum();
  dl($('conserve'), [
    ['energy, total', fx(e.total, 3) + ' J'],
    ['kinetic (moving / turning)', fx(e.lin, 2) + ' / ' + fx(e.rot, 2) + ' J'],
    ['linear momentum P', '(' + m.P.map(x => fx(x)).join(', ') + ')'],
    ['angular momentum L∧L', fx(bwedgeSelf(m.L), 3)],
  ]);
  $('bivec').innerHTML = PLANE_NAMES.map((n, i) => `<div><b>${fx(m.L[i])}</b>L<sub>${n}</sub></div>`).join('');
  const d = chartData(), key = d.series.map(s => s.name).join();
  if (key !== keyFor) {
    keyFor = key;
    $('chartkey').innerHTML = d.series.map(s => `<span><i style="background:${s.color}"></i>${s.name}</span>`).join('');
  }
  showPick();
}

function pick(b) { view.selected = b; showPick(); }
view.onPick = pick;
function showPick() {
  const b = view.selected;
  if (!b || !world.bodies.includes(b)) { $('pick').textContent = 'Click a body in the view to inspect it.'; return; }
  const el = $('pick');
  el.innerHTML = '';
  const t = document.createElement('dl');
  dl(t, [
    ['body', b.name + ' (' + fx(b.mass, 1) + ' kg)'],
    ['position', '(' + b.x.map(x => fx(x)).join(', ') + ')'],
    ['velocity', '(' + b.v.map(x => fx(x)).join(', ') + ')'],
    ['rotation', describeRotation(b.w)],
    ['ω by plane', b.w.map((x, i) => PLANE_NAMES[i] + ' ' + fx(x)).join('  ')],
  ]);
  el.append(t);
}

// ---- loop
function resize() {
  const r = $('stage').getBoundingClientRect();
  view.resize(r.width, r.height);
}
addEventListener('resize', resize);
let last = performance.now();
function frame(t) {
  const dt = Math.min(0.1, (t - last) / 1000); last = t;
  if (running) {
    acc += dt * speed;
    let n = 0;
    while (acc >= 1 / 60 && n < 4) { stepOnce(); acc -= 1 / 60; n++; }
    if (n === 4) acc = 0;
  }
  view.render();
  drawMap();
  drawChart();
  updatePanel(false);
  requestAnimationFrame(frame);
}

load((location.hash || '').slice(1) || SCENES[0].id);
resize();
requestAnimationFrame(frame);
window.__app = { get world() { return world; }, view, load, setSlice, setTilt, advance: n => { for (let i = 0; i < n; i++) stepOnce(); } };
