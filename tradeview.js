// tradeview.js — GUI viewer for tradebot.py's SQLite database.
//
// Renders candles, model predictions, bot actions, trade brackets and the
// equity curve from tradebot.db — live (WAL polling) or as bar-by-bar replay.
//
// Layout, top to bottom:
//   CandlePane  — candles, regime shading, trade brackets, entry/exit markers
//   SignalPane  — p_long / p_short / meta_p with entry thresholds
//   EquityPane  — equity curve, drawdown fill, HALT columns
//   Minimap     — whole-history overview, draggable viewport, replay cursor
//   StatusBar   — mode, price, model version, equity, last action + reason
//
// Keys:
//   space       play / pause replay
//   left/right  step one bar (enters replay mode)
//   up/down     replay speed
//   home/end    jump to start / end
//   l           back to live mode (follow right edge)
//   f           fit last 200 bars
//   pgup/pgdn   zoom in / out   (mouse wheel too)
//   q / esc     quit
// Mouse: drag chart = pan, drag minimap = move viewport, wheel = zoom.
//
// Run: qjsm tradeview.js [tradebot.db]

import { CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, KEY_DOWN, KEY_END, KEY_ESCAPE, KEY_F, KEY_HOME, KEY_L, KEY_LEFT, KEY_PAGE_DOWN, KEY_PAGE_UP, KEY_Q, KEY_RIGHT, KEY_SPACE, KEY_UP, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, OPENGL_PROFILE, RESIZABLE, SAMPLES, Window, context, poll } from 'glfw';
import { SQLite3 } from 'sqlite';
import { ALIGN_LEFT, ALIGN_MIDDLE, ALIGN_RIGHT, ANTIALIAS, CreateGL3, DeleteGL3, RGB, RGBA, STENCIL_STROKES } from 'nanovg';

const MB_LEFT = 0;
const PRESS = 1;

const POLL_MS = 2000; // live DB poll interval
const ENTRY_DIR = 0.45; // gate thresholds, mirror tradebot.Config
const ENTRY_META = 0.58;
const SPEEDS = [1, 2, 4, 8, 16, 32, 64, 128, 256]; // replay bars/second

const COL = {
  bg: RGB(21, 24, 31),
  panel: RGB(26, 30, 38),
  grid: RGBA(255, 255, 255, 14),
  axis: RGB(150, 158, 170),
  text: RGB(190, 198, 210),
  dim: RGB(110, 118, 130),
  up: RGB(38, 166, 126),
  dn: RGB(226, 82, 80),
  upFill: RGBA(38, 166, 126, 220),
  dnFill: RGBA(226, 82, 80, 220),
  regime: RGBA(128, 132, 140, 22),
  entry: RGB(230, 234, 240),
  stop: RGBA(226, 82, 80, 180),
  tp: RGBA(38, 166, 126, 180),
  pLong: RGB(80, 200, 140),
  pShort: RGB(235, 110, 100),
  meta: RGB(240, 180, 70),
  thresh: RGBA(240, 180, 70, 90),
  equity: RGB(120, 180, 250),
  ddFill: RGBA(226, 82, 80, 40),
  halt: RGBA(240, 180, 70, 45),
  cursor: RGB(240, 200, 90),
  viewRect: RGBA(120, 180, 250, 40),
  viewEdge: RGBA(120, 180, 250, 160),
  cross: RGBA(220, 224, 232, 90),
  boxBg: RGBA(15, 17, 22, 235),
};

function fmtTs(ts) {
  const d = new Date(ts * 1000);
  const p = n => (n < 10 ? '0' + n : '' + n);
  return `${d.getUTCFullYear()}-${p(d.getUTCMonth() + 1)}-${p(d.getUTCDate())} ${p(d.getUTCHours())}:${p(d.getUTCMinutes())}`;
}

function fmtNum(x, digits = 2) {
  return x === null || x === undefined ? '-' : x.toFixed(digits);
}

function clamp(x, lo, hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

// nanovg has no dash pattern; emulate with segments.
function dashedLine(vg, x1, y1, x2, y2, dash = 4, gap = 4) {
  const dx = x2 - x1;
  const dy = y2 - y1;
  const len = Math.sqrt(dx * dx + dy * dy);
  if(len < 1) return;
  const ux = dx / len;
  const uy = dy / len;
  vg.BeginPath();
  for(let d = 0; d < len; d += dash + gap) {
    const e = Math.min(d + dash, len);
    vg.MoveTo(x1 + ux * d, y1 + uy * d);
    vg.LineTo(x1 + ux * e, y1 + uy * e);
  }
  vg.Stroke();
}

// ────────────────────────────────────────────────────────────────────────
// Db — thin read-only wrapper over the sqlite binding
// ────────────────────────────────────────────────────────────────────────
class Db {
  constructor(path) {
    this.db = new SQLite3(path, SQLite3.OPEN_READONLY);
  }

  rows(sql) {
    const res = this.db.query(sql);
    const out = [];
    let row;
    while((row = res.fetchAssoc())) out.push(row);
    return out;
  }

  one(sql) {
    const r = this.rows(sql);
    return r.length ? r[0] : null;
  }
}

// ────────────────────────────────────────────────────────────────────────
// Store — all bot data, loaded once, then incrementally polled
// ────────────────────────────────────────────────────────────────────────
class Store {
  constructor(db) {
    this.db = db;
    this.candles = []; // ascending [{ts,open,high,low,close,volume}]
    this.idxByTs = new Map(); // ts -> candle index
    this.predByTs = new Map(); // ts -> prediction row
    this.actsByTs = new Map(); // ts -> [action rows]
    this.haltTs = new Set();
    this.trades = [];
    this.equity = []; // ascending [{ts,equity,drawdown,day_pnl}]
    this.eqByTs = new Map();
    this.state = {};
    this.tf = 3600;
    this.lastPoll = 0;

    const p = db.one(`SELECT product FROM candles LIMIT 1`);
    if(!p) throw new Error('database has no candles — run the bot first');
    this.product = p.product;

    this.refresh(0);
    if(this.candles.length > 2) this.tf = this.candles[1].ts - this.candles[0].ts;
  }

  get lastTs() {
    return this.candles.length ? this.candles[this.candles.length - 1].ts : 0;
  }

  refresh(afterTs) {
    for(const c of this.db.rows(`SELECT ts,open,high,low,close,volume FROM candles WHERE product='${this.product}' AND ts>${afterTs} ORDER BY ts`)) {
      this.idxByTs.set(c.ts, this.candles.length);
      this.candles.push(c);
    }

    for(const r of this.db.rows(`SELECT * FROM predictions WHERE product='${this.product}' AND ts>${afterTs} ORDER BY ts`)) this.predByTs.set(r.ts, r);

    for(const a of this.db.rows(`SELECT * FROM actions WHERE product='${this.product}' AND ts>${afterTs} ORDER BY ts`)) {
      if(!this.actsByTs.has(a.ts)) this.actsByTs.set(a.ts, []);
      this.actsByTs.get(a.ts).push(a);
      if(a.action === 'HALT') this.haltTs.add(a.ts);
    }

    for(const e of this.db.rows(`SELECT * FROM equity WHERE ts>${afterTs} ORDER BY ts`)) {
      this.eqByTs.set(e.ts, e);
      this.equity.push(e);
    }

    // trades get UPDATEd in place on close — refetch wholesale, it's small
    this.trades = this.db.rows(`SELECT * FROM trades WHERE product='${this.product}' ORDER BY ts_open`);

    this.state = {};
    for(const s of this.db.rows(`SELECT key,value FROM state`)) this.state[s.key] = JSON.parse(s.value);
  }

  poll(now) {
    if(now - this.lastPoll < POLL_MS) return false;
    this.lastPoll = now;
    const before = this.candles.length;
    // re-read the last stored candle too: the forming one gets overwritten
    const last = this.lastTs;
    if(last && this.idxByTs.has(last)) {
      const row = this.db.one(`SELECT ts,open,high,low,close,volume FROM candles WHERE product='${this.product}' AND ts=${last}`);
      if(row) this.candles[this.idxByTs.get(last)] = row;
    }
    this.refresh(last);
    return this.candles.length !== before;
  }

  indexOfTs(ts) {
    // binary search: greatest index with candle.ts <= ts
    let lo = 0;
    let hi = this.candles.length - 1;
    let ans = 0;
    while(lo <= hi) {
      const mid = (lo + hi) >> 1;
      if(this.candles[mid].ts <= ts) {
        ans = mid;
        lo = mid + 1;
      } else hi = mid - 1;
    }
    return ans;
  }
}

// ────────────────────────────────────────────────────────────────────────
// Viewport — index<->pixel and price<->pixel mapping, zoom/pan state
// ────────────────────────────────────────────────────────────────────────
class Viewport {
  constructor(store) {
    this.store = store;
    this.i0 = 0;
    this.i1 = 1;
    this.pMin = 0;
    this.pMax = 1;
    this.axisW = 74; // right-side price axis
    this.fit(200);
  }

  get span() {
    return this.i1 - this.i0;
  }

  fit(nBars, endIdx = this.store.candles.length - 1) {
    this.i1 = endIdx + 2;
    this.i0 = Math.max(0, this.i1 - nBars);
  }

  follow() {
    const span = this.span;
    this.i1 = this.store.candles.length + 1;
    this.i0 = this.i1 - span;
  }

  zoom(factor, centerIdx) {
    const n = this.store.candles.length;
    let a = centerIdx - (centerIdx - this.i0) * factor;
    let b = centerIdx + (this.i1 - centerIdx) * factor;
    if(b - a < 10) return;
    if(b - a > n * 1.2 + 10) return;
    this.i0 = a;
    this.i1 = b;
  }

  pan(dIdx) {
    this.i0 += dIdx;
    this.i1 += dIdx;
  }

  barW(rect) {
    return (rect.w - this.axisW) / this.span;
  }

  xOfIndex(i, rect) {
    return rect.x + (i - this.i0) * this.barW(rect);
  }

  indexOfX(x, rect) {
    return this.i0 + (x - rect.x) / this.barW(rect);
  }

  // auto-scale price range to the candles visible up to maxIdx
  updatePriceRange(maxIdx) {
    const cs = this.store.candles;
    let lo = Infinity;
    let hi = -Infinity;
    const a = Math.max(0, Math.floor(this.i0));
    const b = Math.min(maxIdx, Math.ceil(this.i1));
    for(let i = a; i <= b; i++) {
      if(cs[i].low < lo) lo = cs[i].low;
      if(cs[i].high > hi) hi = cs[i].high;
    }
    if(!isFinite(lo)) {
      lo = 0;
      hi = 1;
    }
    const m = (hi - lo) * 0.08 || 1;
    this.pMin = lo - m;
    this.pMax = hi + m;
  }

  yOfPrice(p, rect) {
    return rect.y + rect.h * (1 - (p - this.pMin) / (this.pMax - this.pMin));
  }

  priceOfY(y, rect) {
    return this.pMin + (1 - (y - rect.y) / rect.h) * (this.pMax - this.pMin);
  }
}

// ────────────────────────────────────────────────────────────────────────
// Replay — live / replay mode, cursor, playback
// ────────────────────────────────────────────────────────────────────────
class Replay {
  constructor(store) {
    this.store = store;
    this.mode = 'LIVE';
    this.cursor = Math.max(0, store.candles.length - 1);
    this.playing = false;
    this.speedIdx = 3; // 8 bars/s
    this.acc = 0;
  }

  get speed() {
    return SPEEDS[this.speedIdx];
  }

  // index of the last bar the panes may show
  get maxIdx() {
    return this.mode === 'LIVE' ? this.store.candles.length - 1 : this.cursor;
  }

  get tsLimit() {
    const c = this.store.candles;
    return c.length ? c[Math.min(this.maxIdx, c.length - 1)].ts : 0;
  }

  enterReplay() {
    if(this.mode === 'LIVE') {
      this.mode = 'REPLAY';
      this.cursor = this.store.candles.length - 1;
      this.playing = false;
    }
  }

  enterLive() {
    this.mode = 'LIVE';
    this.playing = false;
  }

  step(d) {
    this.enterReplay();
    this.cursor = clamp(this.cursor + d, 0, this.store.candles.length - 1);
  }

  advance(dt) {
    if(this.mode !== 'REPLAY' || !this.playing) return;
    this.acc += dt * this.speed;
    const n = Math.floor(this.acc);
    if(n > 0) {
      this.acc -= n;
      this.cursor = clamp(this.cursor + n, 0, this.store.candles.length - 1);
      if(this.cursor >= this.store.candles.length - 1) this.playing = false;
    }
  }
}

// ────────────────────────────────────────────────────────────────────────
// Panes
// ────────────────────────────────────────────────────────────────────────
class Pane {
  constructor() {
    this.rect = { x: 0, y: 0, w: 1, h: 1 };
  }

  layout(x, y, w, h) {
    this.rect = { x, y, w, h };
  }

  contains(mx, my) {
    const r = this.rect;
    return mx >= r.x && mx < r.x + r.w && my >= r.y && my < r.y + r.h;
  }
}

class CandlePane extends Pane {
  draw(app) {
    const { vg, store, view, replay } = app;
    const r = this.rect;
    const maxIdx = replay.maxIdx;
    const cs = store.candles;
    view.updatePriceRange(maxIdx);

    vg.Save();
    vg.IntersectScissor(r.x, r.y, r.w, r.h);

    const bw = view.barW(r);
    const a = Math.max(0, Math.floor(view.i0));
    const b = Math.min(maxIdx, Math.ceil(view.i1));

    // regime shading: grey columns where the filter said "don't trade"
    vg.FillColor(COL.regime);
    for(let i = a; i <= b; i++) {
      const p = store.predByTs.get(cs[i].ts);
      if(p && !p.regime_ok) {
        vg.BeginPath();
        vg.Rect(view.xOfIndex(i, r), r.y, bw, r.h);
        vg.Fill();
      }
    }

    // price gridlines + axis labels
    const step = this.priceStep(view.pMax - view.pMin);
    app.font(11, COL.axis, ALIGN_LEFT | ALIGN_MIDDLE);
    for(let p = Math.ceil(view.pMin / step) * step; p < view.pMax; p += step) {
      const y = view.yOfPrice(p, r);
      vg.BeginPath();
      vg.MoveTo(r.x, y);
      vg.LineTo(r.x + r.w - view.axisW, y);
      vg.StrokeColor(COL.grid);
      vg.StrokeWidth(1);
      vg.Stroke();
      app.text(r.x + r.w - view.axisW + 6, y, fmtNum(p, step < 1 ? 4 : 2));
    }

    // candles
    const now = Date.now() / 1000;
    for(let i = a; i <= b; i++) {
      const c = cs[i];
      const x = view.xOfIndex(i, r);
      const cx = x + bw / 2;
      const up = c.close >= c.open;
      const col = up ? COL.upFill : COL.dnFill;
      const yB0 = view.yOfPrice(Math.max(c.open, c.close), r);
      const yB1 = view.yOfPrice(Math.min(c.open, c.close), r);
      const forming = replay.mode === 'LIVE' && i === cs.length - 1 && now - c.ts < store.tf;

      vg.BeginPath();
      vg.MoveTo(cx, view.yOfPrice(c.high, r));
      vg.LineTo(cx, yB0);
      vg.MoveTo(cx, yB1);
      vg.LineTo(cx, view.yOfPrice(c.low, r));
      vg.StrokeColor(up ? COL.up : COL.dn);
      vg.StrokeWidth(Math.max(1, bw * 0.08));
      vg.Stroke();

      vg.BeginPath();
      vg.Rect(x + bw * 0.15, yB0, bw * 0.7, Math.max(1, yB1 - yB0));
      if(forming) {
        vg.StrokeColor(up ? COL.up : COL.dn); // outline only: still mutating
        vg.StrokeWidth(1);
        vg.Stroke();
      } else {
        vg.FillColor(col);
        vg.Fill();
      }
    }

    this.drawTrades(app, a, b);
    this.drawCrosshair(app);
    vg.Restore();

    // current price tag
    if(maxIdx >= 0 && maxIdx < cs.length) {
      const last = cs[maxIdx].close;
      const y = clamp(view.yOfPrice(last, r), r.y + 8, r.y + r.h - 8);
      vg.BeginPath();
      vg.Rect(r.x + r.w - view.axisW, y - 9, view.axisW, 18);
      vg.FillColor(COL.boxBg);
      vg.Fill();
      app.font(11, COL.cursor, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(r.x + r.w - view.axisW + 6, y, fmtNum(last));
    }
  }

  priceStep(range) {
    const raw = range / 6;
    const mag = Math.pow(10, Math.floor(Math.log10(raw)));
    for(const m of [1, 2, 5, 10]) if(raw <= m * mag) return m * mag;
    return 10 * mag;
  }

  drawTrades(app, a, b) {
    const { vg, store, view, replay } = app;
    const r = this.rect;
    const tsLimit = replay.tsLimit;
    const bw = view.barW(r);

    for(const t of store.trades) {
      if(t.ts_open > tsLimit) continue;
      const iO = store.indexOfTs(t.ts_open);
      const closed = t.status === 'CLOSED' && t.ts_close <= tsLimit;
      const iC = closed ? store.indexOfTs(t.ts_close) : replay.maxIdx;
      if(iC < a || iO > b) continue;

      const x0 = view.xOfIndex(iO, r) + bw / 2;
      const x1 = view.xOfIndex(iC, r) + bw / 2;

      vg.StrokeWidth(1);
      vg.StrokeColor(COL.entry);
      dashedLine(vg, x0, view.yOfPrice(t.entry, r), x1, view.yOfPrice(t.entry, r), 2, 3);
      vg.StrokeColor(COL.stop);
      dashedLine(vg, x0, view.yOfPrice(t.stop, r), x1, view.yOfPrice(t.stop, r), 5, 4);
      vg.StrokeColor(COL.tp);
      dashedLine(vg, x0, view.yOfPrice(t.tp, r), x1, view.yOfPrice(t.tp, r), 5, 4);

      // entry marker: triangle under (LONG) / over (SHORT) the entry bar
      const long = t.side === 'LONG';
      const c = store.candles[iO];
      const ey = long ? view.yOfPrice(c.low, r) + 14 : view.yOfPrice(c.high, r) - 14;
      vg.BeginPath();
      if(long) {
        vg.MoveTo(x0, ey - 6);
        vg.LineTo(x0 - 5, ey + 4);
        vg.LineTo(x0 + 5, ey + 4);
      } else {
        vg.MoveTo(x0, ey + 6);
        vg.LineTo(x0 - 5, ey - 4);
        vg.LineTo(x0 + 5, ey - 4);
      }
      vg.ClosePath();
      vg.FillColor(long ? COL.up : COL.dn);
      vg.Fill();

      // exit marker: circle at exit price, colored by outcome
      if(closed) {
        vg.BeginPath();
        vg.Circle(x1, view.yOfPrice(t.exit, r), 4);
        vg.FillColor(t.pnl >= 0 ? COL.up : COL.dn);
        vg.Fill();
        vg.StrokeColor(COL.entry);
        vg.StrokeWidth(1);
        vg.Stroke();
      }
    }
  }

  drawCrosshair(app) {
    const { vg, store, view, replay, mouse } = app;
    const r = this.rect;
    if(!this.contains(mouse.x, mouse.y)) return;

    const i = clamp(Math.floor(view.indexOfX(mouse.x, r)), 0, replay.maxIdx);
    if(i < 0 || i >= store.candles.length) return;
    const c = store.candles[i];
    const bw = view.barW(r);
    const cx = view.xOfIndex(i, r) + bw / 2;

    vg.BeginPath();
    vg.MoveTo(cx, r.y);
    vg.LineTo(cx, r.y + r.h);
    vg.MoveTo(r.x, mouse.y);
    vg.LineTo(r.x + r.w - view.axisW, mouse.y);
    vg.StrokeColor(COL.cross);
    vg.StrokeWidth(1);
    vg.Stroke();

    const lines = [`${fmtTs(c.ts)}  O ${fmtNum(c.open)}  H ${fmtNum(c.high)}  L ${fmtNum(c.low)}  C ${fmtNum(c.close)}  V ${fmtNum(c.volume, 3)}`];
    const p = store.predByTs.get(c.ts);
    if(p) lines.push(`long ${fmtNum(p.p_long, 3)}  flat ${fmtNum(p.p_flat, 3)}  short ${fmtNum(p.p_short, 3)}  meta ${fmtNum(p.meta_p, 3)}  regime ${p.regime_ok ? 'ok' : 'OFF'}  ${p.model_ver}`);
    for(const act of store.actsByTs.get(c.ts) ?? []) lines.push(`${act.action}  ${act.reason}`);

    const bh = 8 + lines.length * 16;
    const bwx = 8 + this.maxLineWidth(lines) * 6.4;
    let bx = mouse.x + 14;
    let by = mouse.y + 14;
    if(bx + bwx > r.x + r.w) bx = mouse.x - 14 - bwx;
    if(by + bh > r.y + r.h) by = mouse.y - 14 - bh;

    vg.BeginPath();
    vg.RoundedRect(bx, by, bwx, bh, 4);
    vg.FillColor(COL.boxBg);
    vg.Fill();
    app.font(11, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
    for(let k = 0; k < lines.length; k++) app.text(bx + 6, by + 12 + k * 16, lines[k]);
  }

  maxLineWidth(lines) {
    let m = 0;
    for(const l of lines) if(l.length > m) m = l.length;
    return m;
  }
}

class SignalPane extends Pane {
  draw(app) {
    const { vg, view } = app;
    const r = this.rect;
    vg.Save();
    vg.IntersectScissor(r.x, r.y, r.w, r.h);

    const yOf = v => r.y + r.h * (1 - v);

    // threshold guides
    vg.StrokeColor(COL.thresh);
    vg.StrokeWidth(1);
    dashedLine(vg, r.x, yOf(ENTRY_META), r.x + r.w - view.axisW, yOf(ENTRY_META), 3, 5);
    dashedLine(vg, r.x, yOf(ENTRY_DIR), r.x + r.w - view.axisW, yOf(ENTRY_DIR), 3, 5);

    this.line(app, p => p.p_long, COL.pLong, yOf);
    this.line(app, p => p.p_short, COL.pShort, yOf);
    this.line(app, p => p.meta_p, COL.meta, yOf);

    app.font(11, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(r.x + 6, r.y + 10, 'signals: long / short / meta');
    vg.Restore();
  }

  line(app, get, color, yOf) {
    const { vg, store, view, replay } = app;
    const r = this.rect;
    const bw = view.barW(r);
    const a = Math.max(0, Math.floor(view.i0));
    const b = Math.min(replay.maxIdx, Math.ceil(view.i1));
    vg.BeginPath();
    let started = false;
    for(let i = a; i <= b; i++) {
      const p = store.predByTs.get(store.candles[i].ts);
      if(!p) continue;
      const x = view.xOfIndex(i, r) + bw / 2;
      const y = yOf(get(p));
      if(!started) {
        vg.MoveTo(x, y);
        started = true;
      } else vg.LineTo(x, y);
    }
    vg.StrokeColor(color);
    vg.StrokeWidth(1.2);
    vg.Stroke();
  }
}

class EquityPane extends Pane {
  draw(app) {
    const { vg, store, view, replay } = app;
    const r = this.rect;
    vg.Save();
    vg.IntersectScissor(r.x, r.y, r.w, r.h);

    // visible equity rows
    const rows = [];
    let lo = Infinity;
    let hi = -Infinity;
    const a = Math.max(0, Math.floor(view.i0));
    const b = Math.min(replay.maxIdx, Math.ceil(view.i1));
    for(let i = a; i <= b; i++) {
      const e = store.eqByTs.get(store.candles[i].ts);
      if(!e) continue;
      rows.push({ i, e });
      if(e.equity < lo) lo = e.equity;
      if(e.equity > hi) hi = e.equity;
    }
    if(rows.length < 2) {
      app.font(11, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(r.x + 6, r.y + 10, 'equity (no data yet)');
      vg.Restore();
      return;
    }
    const m = (hi - lo) * 0.15 || 1;
    lo -= m;
    hi += m;
    const yOf = v => r.y + r.h * (1 - (v - lo) / (hi - lo));
    const bw = view.barW(r);

    // HALT columns
    vg.FillColor(COL.halt);
    for(let i = a; i <= b; i++) {
      if(store.haltTs.has(store.candles[i].ts)) {
        vg.BeginPath();
        vg.Rect(view.xOfIndex(i, r), r.y, bw, r.h);
        vg.Fill();
      }
    }

    // drawdown fill: area between running peak and equity
    let peak = -Infinity;
    vg.FillColor(COL.ddFill);
    for(const { i, e } of rows) {
      if(e.equity > peak) peak = e.equity;
      if(peak - e.equity <= 0) continue;
      const x = view.xOfIndex(i, r) + bw / 2;
      vg.BeginPath();
      vg.Rect(x - bw / 2, yOf(peak), bw, yOf(e.equity) - yOf(peak));
      vg.Fill();
    }

    // equity line
    vg.BeginPath();
    rows.forEach(({ i, e }, k) => {
      const x = view.xOfIndex(i, r) + bw / 2;
      const y = yOf(e.equity);
      k === 0 ? vg.MoveTo(x, y) : vg.LineTo(x, y);
    });
    vg.StrokeColor(COL.equity);
    vg.StrokeWidth(1.4);
    vg.Stroke();

    const lastE = rows[rows.length - 1].e;
    app.font(11, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(r.x + 6, r.y + 10, `equity ${fmtNum(lastE.equity)}  dd ${fmtNum(lastE.drawdown * 100, 2)}%  day ${fmtNum(lastE.day_pnl)}`);
    vg.Restore();
  }
}

class Minimap extends Pane {
  draw(app) {
    const { vg, store, view, replay } = app;
    const r = this.rect;
    const cs = store.candles;
    const n = cs.length;
    if(n < 2) return;

    vg.BeginPath();
    vg.Rect(r.x, r.y, r.w, r.h);
    vg.FillColor(COL.panel);
    vg.Fill();

    let lo = Infinity;
    let hi = -Infinity;
    for(const c of cs) {
      if(c.close < lo) lo = c.close;
      if(c.close > hi) hi = c.close;
    }
    const yOf = p => r.y + 4 + (r.h - 8) * (1 - (p - lo) / (hi - lo || 1));
    const xOf = i => r.x + (i / (n - 1)) * r.w;

    vg.BeginPath();
    const step = Math.max(1, Math.floor(n / r.w));
    for(let i = 0; i < n; i += step) {
      const x = xOf(i);
      const y = yOf(cs[i].close);
      i === 0 ? vg.MoveTo(x, y) : vg.LineTo(x, y);
    }
    vg.StrokeColor(COL.dim);
    vg.StrokeWidth(1);
    vg.Stroke();

    // viewport rectangle
    const vx0 = xOf(clamp(view.i0, 0, n - 1));
    const vx1 = xOf(clamp(view.i1, 0, n - 1));
    vg.BeginPath();
    vg.Rect(vx0, r.y, Math.max(3, vx1 - vx0), r.h);
    vg.FillColor(COL.viewRect);
    vg.Fill();
    vg.StrokeColor(COL.viewEdge);
    vg.StrokeWidth(1);
    vg.Stroke();

    // replay cursor
    if(replay.mode === 'REPLAY') {
      const cx = xOf(replay.cursor);
      vg.BeginPath();
      vg.MoveTo(cx, r.y);
      vg.LineTo(cx, r.y + r.h);
      vg.StrokeColor(COL.cursor);
      vg.StrokeWidth(2);
      vg.Stroke();
    }
  }

  // drag → recenter the viewport on that fraction of history
  dragTo(app, mx) {
    const { store, view } = app;
    const n = store.candles.length;
    const frac = clamp((mx - this.rect.x) / this.rect.w, 0, 1);
    const center = frac * (n - 1);
    const span = view.span;
    view.i0 = center - span / 2;
    view.i1 = center + span / 2;
    app.follow = false;
  }
}

class StatusBar extends Pane {
  draw(app) {
    const { vg, store, replay } = app;
    const r = this.rect;
    vg.BeginPath();
    vg.Rect(r.x, r.y, r.w, r.h);
    vg.FillColor(COL.panel);
    vg.Fill();

    const cs = store.candles;
    const maxIdx = Math.min(replay.maxIdx, cs.length - 1);
    const c = cs[maxIdx];
    const pred = store.predByTs.get(c.ts);
    const eq = store.eqByTs.get(c.ts);
    const halted = store.state.halted === true;

    let lastAct = null;
    for(let i = maxIdx; i >= 0 && !lastAct; i--) {
      const acts = store.actsByTs.get(cs[i].ts);
      if(acts && acts.length) lastAct = acts[acts.length - 1];
    }

    const mode = replay.mode === 'LIVE' ? 'LIVE' : `REPLAY ${replay.playing ? '▶' : '⏸'} ${replay.speed}x`;
    let s = `${mode}  |  ${store.product}  ${fmtTs(c.ts)}  ${fmtNum(c.close)}`;
    if(pred) s += `  |  model ${pred.model_ver}`;
    if(eq) s += `  |  eq ${fmtNum(eq.equity)} (dd ${fmtNum(eq.drawdown * 100, 1)}%)`;
    if(lastAct) s += `  |  ${lastAct.action}: ${lastAct.reason}`;

    app.font(12, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(r.x + 8, r.y + r.h / 2, s);

    if(halted) {
      app.font(12, COL.dn, ALIGN_RIGHT | ALIGN_MIDDLE);
      app.text(r.x + r.w - 8, r.y + r.h / 2, 'HALTED — kill switch');
    }
  }
}

// ────────────────────────────────────────────────────────────────────────
// App — window, input, frame loop
// ────────────────────────────────────────────────────────────────────────
class App {
  constructor(dbPath) {
    this.store = new Store(new Db(dbPath));
    this.view = new Viewport(this.store);
    this.replay = new Replay(this.store);
    this.follow = true;

    this.candlePane = new CandlePane();
    this.signalPane = new SignalPane();
    this.equityPane = new EquityPane();
    this.minimap = new Minimap();
    this.statusBar = new StatusBar();

    this.mouse = { x: 0, y: 0 };
    this.dragPane = null; // 'chart' | 'minimap'
    this.dragX = 0;
    this.running = true;
    this.hasFont = false;
  }

  initWindow() {
    Window.hint(CONTEXT_VERSION_MAJOR, 3);
    Window.hint(CONTEXT_VERSION_MINOR, 2);
    Window.hint(OPENGL_PROFILE, OPENGL_CORE_PROFILE);
    Window.hint(OPENGL_FORWARD_COMPAT, true);
    Window.hint(RESIZABLE, false);
    Window.hint(SAMPLES, 4);

    this.window = context.current = new Window(1280, 800, `tradeview — ${this.store.product}`);
    const { width, height } = this.window.size;
    this.width = width;
    this.height = height;

    this.vg = CreateGL3(STENCIL_STROKES | ANTIALIAS);
    for(const path of ['/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf', '/usr/share/fonts/TTF/DejaVuSans.ttf', '/usr/share/fonts/dejavu/DejaVuSans.ttf']) {
      if(this.vg.CreateFont('sans', path) >= 0) {
        this.hasFont = true;
        break;
      }
    }

    this.layout();
    this.bindInput();
  }

  layout() {
    const w = this.width;
    const statusH = 26;
    const miniH = 56;
    const rest = this.height - statusH - miniH;
    const candleH = Math.floor(rest * 0.62);
    const sigH = Math.floor(rest * 0.19);
    const eqH = rest - candleH - sigH;

    let y = 0;
    this.candlePane.layout(0, y, w, candleH);
    y += candleH;
    this.signalPane.layout(0, y, w, sigH);
    y += sigH;
    this.equityPane.layout(0, y, w, eqH);
    y += eqH;
    this.minimap.layout(0, y, w, miniH);
    y += miniH;
    this.statusBar.layout(0, y, w, statusH);
  }

  bindInput() {
    const app = this;
    Object.assign(this.window, {
      handleCursorPos(x, y) {
        if(app.dragPane === 'chart') {
          const dIdx = -(x - app.dragX) / app.view.barW(app.candlePane.rect);
          app.view.pan(dIdx);
          app.dragX = x;
          app.follow = false;
        } else if(app.dragPane === 'minimap') {
          app.minimap.dragTo(app, x);
        }
        app.mouse = { x, y };
      },
      handleMouseButton(button, action) {
        if(button !== MB_LEFT) return;
        if(action !== PRESS) {
          app.dragPane = null;
          return;
        }
        if(app.minimap.contains(app.mouse.x, app.mouse.y)) {
          app.dragPane = 'minimap';
          app.minimap.dragTo(app, app.mouse.x);
        } else if(app.candlePane.contains(app.mouse.x, app.mouse.y) || app.signalPane.contains(app.mouse.x, app.mouse.y) || app.equityPane.contains(app.mouse.x, app.mouse.y)) {
          app.dragPane = 'chart';
          app.dragX = app.mouse.x;
        }
      },
      handleScroll(dx, dy) {
        // wheel zoom around the bar under the cursor
        const center = app.view.indexOfX(app.mouse.x, app.candlePane.rect);
        app.view.zoom(dy > 0 ? 0.85 : 1.18, center);
        app.follow = false;
      },
      handleKey(keyCode, scancode, action) {
        if(!action) return;
        const { replay, view, store } = app;
        switch(keyCode) {
          case KEY_ESCAPE:
          case KEY_Q:
            app.running = false;
            break;
          case KEY_SPACE:
            replay.enterReplay();
            replay.playing = !replay.playing;
            break;
          case KEY_LEFT:
            replay.step(-1);
            break;
          case KEY_RIGHT:
            replay.step(1);
            break;
          case KEY_UP:
            replay.speedIdx = clamp(replay.speedIdx + 1, 0, SPEEDS.length - 1);
            break;
          case KEY_DOWN:
            replay.speedIdx = clamp(replay.speedIdx - 1, 0, SPEEDS.length - 1);
            break;
          case KEY_HOME:
            replay.enterReplay();
            replay.cursor = 0;
            view.fit(200, 0);
            break;
          case KEY_END:
            replay.enterReplay();
            replay.cursor = store.candles.length - 1;
            view.fit(200);
            break;
          case KEY_L:
            replay.enterLive();
            app.follow = true;
            view.fit(200);
            break;
          case KEY_F:
            view.fit(200, replay.maxIdx);
            break;
          case KEY_PAGE_UP:
            view.zoom(0.7, (view.i0 + view.i1) / 2);
            break;
          case KEY_PAGE_DOWN:
            view.zoom(1.4, (view.i0 + view.i1) / 2);
            break;
        }
      },
    });
  }

  font(size, color, align) {
    if(!this.hasFont) return;
    this.vg.FontFace('sans');
    this.vg.FontSize(size);
    this.vg.FillColor(color);
    this.vg.TextAlign(align);
  }

  text(x, y, str) {
    if(this.hasFont) this.vg.Text(x, y, str);
  }

  frame(dt) {
    const { vg, store, replay } = this;

    // live: poll DB, keep following the right edge
    if(replay.mode === 'LIVE') {
      const grew = store.poll(Date.now());
      if(grew && this.follow) this.view.follow();
    }
    replay.advance(dt);
    // keep replay cursor in view while playing
    if(replay.mode === 'REPLAY' && replay.playing) {
      if(replay.cursor > this.view.i1 - 2 || replay.cursor < this.view.i0) {
        const span = this.view.span;
        this.view.i1 = replay.cursor + 2;
        this.view.i0 = this.view.i1 - span;
      }
    }

    vg.BeginPath();
    vg.Rect(0, 0, this.width, this.height);
    vg.FillColor(COL.bg);
    vg.Fill();

    this.candlePane.draw(this);
    this.signalPane.draw(this);
    this.equityPane.draw(this);
    this.minimap.draw(this);
    this.statusBar.draw(this);
  }

  run() {
    let last = Date.now();
    while((this.running &&= !this.window.shouldClose)) {
      const now = Date.now();
      const dt = (now - last) / 1000;
      last = now;

      this.vg.BeginFrame(this.width, this.height, 1);
      this.frame(dt);
      this.vg.EndFrame();

      this.window.swapBuffers();
      poll();
    }
    DeleteGL3(this.vg);
    this.window.destroy();
  }
}

function main(...args) {
  const dbPath = args[0] ?? 'tradebot.db';
  console.log(`tradeview: opening ${dbPath} (read-only)`);

  const app = new App(dbPath);
  console.log(`tradeview: ${app.store.product}, ${app.store.candles.length} candles, tf=${app.store.tf}s, ${app.store.trades.length} trades`);

  app.initWindow();
  app.run();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message ?? error}`);
  if(error && error.stack) console.log(error.stack);
  if(error && error.constructor) console.log(`type: ${error.constructor.name}`);
}
