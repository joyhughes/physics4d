// Headless Chrome check: load each scene, collect console errors, screenshot.
// Usage: node tools/shot.mjs [outdir] [scene ...]
import { spawn } from 'node:child_process';
import { writeFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { join } from 'node:path';
const out = process.argv[2] ?? '.';
const scenes = process.argv.slice(3);
const chrome = spawn('/Applications/Google Chrome.app/Contents/MacOS/Google Chrome', [
  '--headless=new', '--remote-debugging-port=9333', '--use-angle=swiftshader', '--enable-unsafe-swiftshader', '--mute-audio',
  '--user-data-dir=' + mkdtempSync(join(tmpdir(), 'p4d-')), '--window-size=1400,900', 'about:blank'], { stdio: 'ignore' });
await new Promise(r => setTimeout(r, 1500));
const tabs = await (await fetch('http://127.0.0.1:9333/json')).json();
const ws = new WebSocket(tabs.find(t => t.type === 'page').webSocketDebuggerUrl);
await new Promise(r => ws.onopen = r);
let id = 0; const pend = new Map(), logs = [];
ws.onmessage = e => {
  const m = JSON.parse(e.data);
  if (m.id && pend.has(m.id)) { pend.get(m.id)(m); pend.delete(m.id); }
  if (m.method === 'Runtime.consoleAPICalled') logs.push(m.params.type + ': ' + m.params.args.map(a => a.value ?? a.description).join(' '));
  if (m.method === 'Runtime.exceptionThrown') logs.push('EXCEPTION: ' + (m.params.exceptionDetails.exception?.description ?? m.params.exceptionDetails.text));
};
const send = (method, params = {}) => new Promise(r => { const i = ++id; pend.set(i, r); ws.send(JSON.stringify({ id: i, method, params })); });
await send('Runtime.enable'); await send('Page.enable'); await send('Network.enable'); await send('Network.clearBrowserCache');
const [W, H] = (process.env.SIZE ?? '1400x900').split('x').map(Number);
await send('Emulation.setDeviceMetricsOverride', { width: W, height: H, deviceScaleFactor: 1, mobile: W < 600 });
if (process.env.SCHEME) await send('Emulation.setEmulatedMedia', { features: [{ name: 'prefers-color-scheme', value: process.env.SCHEME }] });
await send('Page.navigate', { url: (process.env.BASE ?? 'http://localhost:8940/') + '#' + (scenes[0] ?? 'slices') });
await new Promise(r => setTimeout(r, 4000));
for (const s of scenes.length ? scenes : ['slices']) {
  await send('Runtime.evaluate', { expression: `__app.load('${s}')` });
  // EVAL: script run before advancing (e.g. change a scene control, select a body)
  if (process.env.EVAL) await send('Runtime.evaluate', { expression: process.env.EVAL });
  await send('Runtime.evaluate', { expression: `__app.advance(${process.env.ADV ?? 150})` });
  if (process.env.EVAL2) await send('Runtime.evaluate', { expression: process.env.EVAL2 });
  await new Promise(r => setTimeout(r, 2500));
  const shot = await send('Page.captureScreenshot', { format: 'png' });
  writeFileSync(join(out, s + '.png'), Buffer.from(shot.result.data, 'base64'));
  const t = await send('Runtime.evaluate', { expression: '__app.world.time.toFixed(2)', returnByValue: true });
  console.log(s, 'sim time', t.result.result.value);
}
console.log(logs.join('\n') || 'no console output');
ws.close(); chrome.kill();
