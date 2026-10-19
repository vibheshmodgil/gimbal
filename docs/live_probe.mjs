import { writeFile } from 'node:fs/promises';
import { setTimeout as sleep } from 'node:timers/promises';

const host = 'anvyxo.local';
const base = `http://${host}`;
const phase = process.argv[2] || 'baseline';
const samples = [];
const replies = [];

async function command(cmd) {
  const started = performance.now();
  const response = await fetch(`${base}/cmd?c=${encodeURIComponent(cmd)}`, {
    signal: AbortSignal.timeout(4000),
  });
  const body = await response.text();
  return { cmd, ms: Math.round(performance.now() - started), status: response.status, body };
}

async function connectStatus() {
  const ws = new WebSocket(`ws://${host}:81/`);
  await new Promise((resolve, reject) => {
    ws.onopen = resolve;
    ws.onerror = reject;
  });
  ws.onmessage = ({ data }) => {
    try {
      const value = JSON.parse(data);
      if (value.t === 's') samples.push({ at: Date.now(), ...value });
    } catch { replies.push({ at: Date.now(), text: String(data) }); }
  };
  return ws;
}

async function saveFrame(path) {
  const controller = new AbortController();
  const timer = setTimeout(() => controller.abort(), 5000);
  try {
    const response = await fetch(`http://${host}:82/stream`, { signal: controller.signal });
    if (!response.ok) return { status: response.status, error: await response.text() };
    let bytes = Buffer.alloc(0);
    for await (const chunk of response.body) {
      bytes = Buffer.concat([bytes, Buffer.from(chunk)]);
      const start = bytes.indexOf(Buffer.from([0xff, 0xd8]));
      const end = start < 0 ? -1 : bytes.indexOf(Buffer.from([0xff, 0xd9]), start + 2);
      if (end >= 0) {
        const jpeg = bytes.subarray(start, end + 2);
        await writeFile(path, jpeg);
        return { status: response.status, bytes: jpeg.length, path };
      }
      if (bytes.length > 2_000_000) throw new Error('Frame exceeded 2 MB');
    }
    throw new Error('Stream ended before a JPEG frame');
  } finally {
    clearTimeout(timer);
    controller.abort();
  }
}

const ws = await connectStatus();
try {
  await sleep(450);
  console.log('START', JSON.stringify(samples.at(-1) || null));
  console.log('DIAG', JSON.stringify(await command('DIAG')));
  const camera = await command('CAMGET');
  console.log('CAM', JSON.stringify(camera));
  if (phase === 'status') {
    await sleep(1200);
  } else if (phase === 'baseline') {
    await sleep(2500);
    console.log('FRAME', JSON.stringify(await saveFrame('docs/live_probe_frame.jpg')));
  } else if (phase === 'camera') {
    const original = JSON.parse(camera.body);
    const tests = ['FAST', 'BAL', 'HQ', 'MAX'];
    try {
      for (const preset of tests) {
        const result = await command(`CAMP:${preset}`);
        console.log('PRESET', preset, JSON.stringify(result));
        const since = Date.now();
        await sleep(4300);
        const measure = samples.filter(s => s.at >= since + 1000);
        const fps = measure.map(s => s.fps);
        console.log('PERF', preset, JSON.stringify({
          count: measure.length,
          fpsMean: fps.reduce((a, b) => a + b, 0) / fps.length,
          fpsMin: Math.min(...fps), fpsMax: Math.max(...fps),
          kb: measure.at(-1)?.kb, viewers: measure.at(-1)?.viewers,
        }));
      }
    } finally {
      for (const [key, value] of [['res', original.res], ['q', original.q], ['fps', original.fps]]) {
        console.log('RESTORE', JSON.stringify(await command(`CAM:${key},${value}`)));
      }
    }
  } else if (phase === 'motion') {
    const start = samples.at(-1);
    if (!start || start.calOn || !start.panEn || !start.tiltEn || start.laser) {
      throw new Error('Motion precheck failed: missing status, calibration active, servo off, or laser on');
    }
    const moves = [
      ['center', 'PT:0,0', 1800],
      ['pan_right', 'PT:20,0', 1700],
      ['pan_left', 'PT:-20,0', 1700],
      ['center', 'PT:0,0', 1400],
      ['tilt_up', 'PT:0,12', 1400],
      ['tilt_down', 'PT:0,-12', 1400],
      ['center', 'PT:0,0', 1400],
      ['diagonal', 'PT:20,12', 1700],
      ['center', 'PT:0,0', 1700],
    ];
    for (const [label, cmd, duration] of moves) {
      const sentAt = Date.now();
      console.log('MOVE', label, JSON.stringify(await command(cmd)));
      await sleep(duration);
      console.log('STATE', label, JSON.stringify(samples.at(-1) || null));
      const segment = samples.filter(s => s.at >= sentAt);
      const target = cmd.slice(3).split(',').map(Number);
      const reached = segment.find(s => Math.abs(s.pan - target[0]) <= 0.5 && Math.abs(s.tilt - target[1]) <= 0.5);
      const fps = segment.map(s => s.fps);
      console.log('TIMING', label, JSON.stringify({
        reportedReachMs: reached ? reached.at - sentAt : null,
        fpsMean: fps.reduce((a, b) => a + b, 0) / fps.length,
        fpsMin: Math.min(...fps), fpsMax: Math.max(...fps),
      }));
      if (['pan_right', 'pan_left', 'tilt_up', 'tilt_down'].includes(label)) {
        console.log('FRAME', label, JSON.stringify(await saveFrame(`docs/live_probe_${label}.jpg`)));
      }
    }
  } else if (phase === 'ws_motion') {
    const start = samples.at(-1);
    if (!start || !start.panEn || !start.tiltEn || start.laser ||
        Math.abs(start.pan) > 1 || Math.abs(start.tilt) > 1) {
      throw new Error('Motion precheck failed: expected centered, both servos on, laser off');
    }
    for (let i = 0; i < 12; i++) {
      const sentAt = Date.now();
      ws.send('PING');
      await sleep(250);
      const reply = replies.find(r => r.at >= sentAt && r.text === 'PONG');
      console.log('PING', JSON.stringify({ i, rttMs: reply ? reply.at - sentAt : null }));
    }
    const moves = [
      [10, 0], [-10, 0], [0, 0], [30, 0], [-30, 0], [0, 0],
      [0, 15], [0, -15], [0, 0], [20, 12], [-20, -12], [0, 0],
    ];
    for (const [pan, tilt] of moves) {
      const sentAt = Date.now();
      ws.send(`PT:${pan},${tilt}`);
      await sleep(1200);
      const segment = samples.filter(s => s.at >= sentAt);
      const accepted = segment.find(s => s.tp === pan && s.tt === tilt);
      const reached = segment.find(s => Math.abs(s.pan - pan) <= 0.5 && Math.abs(s.tilt - tilt) <= 0.5);
      const fps = segment.map(s => s.fps);
      console.log('WS_MOVE', JSON.stringify({ pan, tilt,
        acceptMs: accepted ? accepted.at - sentAt : null,
        reportedReachMs: reached ? reached.at - sentAt : null,
        fpsMean: fps.reduce((a, b) => a + b, 0) / fps.length,
        last: segment.at(-1) ? { pan: segment.at(-1).pan, tilt: segment.at(-1).tilt } : null,
      }));
      if (!accepted || !reached) throw new Error('Movement status did not settle within 1.2 s');
    }
    console.log('END_DIAG', JSON.stringify(await command('DIAG')));
  } else throw new Error(`Unknown phase ${phase}`);
  const fps = samples.map(s => s.fps).filter(Number.isFinite);
  console.log('SUMMARY', JSON.stringify({ count: samples.length,
    fpsMin: Math.min(...fps), fpsMax: Math.max(...fps),
    fpsMean: fps.reduce((a, b) => a + b, 0) / fps.length,
    last: samples.at(-1) || null }));
  await writeFile(`docs/live_probe_${phase}.json`, JSON.stringify(samples, null, 2));
} finally {
  if (phase === 'motion' || phase === 'ws_motion') {
    try { console.log('HOME', JSON.stringify(await command('PT:0,0'))); } catch (error) { console.error('HOME FAILED', error); }
  }
  try { await writeFile(`docs/live_probe_${phase}.json`, JSON.stringify(samples, null, 2)); } catch (error) { console.error('SAVE FAILED', error); }
  ws.close();
}
