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

import { CONTEXT_VERSION_MAJOR, CONTEXT_VERSION_MINOR, KEY_DOWN, KEY_END, KEY_ESCAPE, KEY_F, KEY_HOME, KEY_L, KEY_LEFT, KEY_PAGE_DOWN, KEY_PAGE_UP, KEY_Q, KEY_RIGHT, KEY_SPACE, KEY_UP, OPENGL_CORE_PROFILE, OPENGL_FORWARD_COMPAT, OPENGL_PROFILE, RESIZABLE, SAMPLES, Window, context, poll, } from 'glfw';
import { SQLite3 } from 'sqlite';
import { ALIGN_LEFT, ALIGN_MIDDLE, ALIGN_RIGHT, ANTIALIAS, CreateGL3, DeleteGL3, RGB, RGBA, STENCIL_STROKES } from 'nanovg';

const MB_LEFT = 0;
const PRESS = 1;

const POLL_MS = 2000; // live DB poll interval
const SPEEDS = [1, 2, 4, 8, 16, 32, 64, 128, 256]; // replay bars/second

// ── tradebot.Config mirror ────────────────────────────────────────────────
// Everything below is a bit-for-bit reproduction of the deterministic
// (non-ML) side of tradebot.py so the viewer can compute the same features,
// evaluate the same regime filter, and show *why* the bot would or would
// not have entered on any given bar — no torch/xgboost needed.
const FEE_RATE = 0.006;
const ENTRY_DIR = 0.45; // primary direction prob gate
const ENTRY_META = 0.58; // meta-model gate
const REGIME_ADX_MIN = 20; // RegimeFilter.ok: adx > 20
const REGIME_ATR_MIN = 2.5 * FEE_RATE; // and atr_rel > 2.5 × fee_rate
const SEQ_LEN = 64; // LSTM lookback
const MIN_BARS_FOR_PRED = SEQ_LEN + 60; // Bot.on_bar guard
const STOP_ATR = 2.0;
const TP_ATR = 3.0;
const TARGET_VOL = 0.01;
const MAX_POS_FRAC = 0.25;
const DAILY_LOSS_LIMIT = 0.02;
const MAX_DRAWDOWN = 0.1;
const START_EQUITY = 10_000.0;

const COL = {
  bg: RGB(248, 241, 215),
  panel: RGB(236, 226, 192),
  grid: RGBA(80, 60, 40, 32),
  axis: RGB(70, 55, 40),
  text: RGB(40, 32, 22),
  dim: RGB(130, 115, 90),
  up: RGB(38, 166, 126),
  dn: RGB(226, 82, 80),
  upFill: RGBA(38, 166, 126, 220),
  dnFill: RGBA(226, 82, 80, 220),
  regime: RGBA(90, 70, 45, 28),
  entry: RGB(40, 32, 22),
  stop: RGBA(170, 50, 50, 200),
  tp: RGBA(40, 120, 75, 200),
  pLong: RGB(35, 85, 130),
  pShort: RGB(160, 60, 50),
  meta: RGB(150, 100, 30),
  thresh: RGBA(150, 100, 30, 100),
  equity: RGB(35, 60, 110),
  ddFill: RGBA(170, 50, 50, 40),
  halt: RGBA(180, 100, 30, 55),
  cursor: RGB(140, 80, 30),
  viewRect: RGBA(70, 90, 130, 45),
  viewEdge: RGBA(40, 70, 120, 170),
  cross: RGBA(70, 55, 40, 130),
  boxBg: RGBA(230, 218, 180, 240),
  cardBg: RGBA(244, 234, 200, 245),
  cardEdge: RGBA(80, 60, 40, 140),
  sma: RGB(40, 70, 110),
  bbBand: RGBA(110, 90, 60, 110),
  vwap: RGB(160, 120, 40),
  gateOk: RGB(20, 100, 60),
  gateFail: RGB(160, 45, 45),
  gateSkip: RGB(120, 105, 80),
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
// Math helpers — recursive EWM (pandas ewm(adjust=False)) and rolling
// window primitives, kept simple; O(n) each and plenty fast for a viewer.
// ────────────────────────────────────────────────────────────────────────
function ewm(v, alpha) {
  const n = v.length;
  const out = new Float64Array(n);
  if(n === 0) return out;
  out[0] = v[0];
  for(let i = 1; i < n; i++) out[i] = alpha * v[i] + (1 - alpha) * out[i - 1];
  return out;
}

function rollingMean(v, w) {
  const n = v.length;
  const out = new Float64Array(n).fill(NaN);
  let s = 0;
  for(let i = 0; i < n; i++) {
    s += v[i];
    if(i >= w) s -= v[i - w];
    if(i >= w - 1) out[i] = s / w;
  }
  return out;
}

function rollingSum(v, w) {
  const n = v.length;
  const out = new Float64Array(n).fill(NaN);
  let s = 0;
  for(let i = 0; i < n; i++) {
    s += v[i];
    if(i >= w) s -= v[i - w];
    if(i >= w - 1) out[i] = s;
  }
  return out;
}

function rollingStd(v, w) {
  // pandas default ddof=1 (sample std) — matches Bollinger convention
  const n = v.length;
  const out = new Float64Array(n).fill(NaN);
  let s = 0;
  let s2 = 0;
  for(let i = 0; i < n; i++) {
    s += v[i];
    s2 += v[i] * v[i];
    if(i >= w) {
      s -= v[i - w];
      s2 -= v[i - w] * v[i - w];
    }
    if(i >= w - 1) {
      const m = s / w;
      const varr = (s2 - w * m * m) / (w - 1);
      out[i] = Math.sqrt(Math.max(0, varr));
    }
  }
  return out;
}

// ────────────────────────────────────────────────────────────────────────
// Indicators — deterministic replica of tradebot.py's FeatureEngine.
//   sma20 / bb_upper / bb_lower  — 20-bar Bollinger context
//   atr_abs (Wilder 14)          — used for stops, sizing, regime
//   rsi14, macd_hist             — momentum
//   vwap24, vol_z                — flow / volume
//   adx14                        — trend strength for RegimeFilter
//   regimeOk = adx > 20 AND atr_rel > 2.5 × fee_rate  (per-bar boolean)
// Recomputed when the candle count changes; cheap enough to redo whole.
// ────────────────────────────────────────────────────────────────────────
class Indicators {
  constructor(store) {
    this.store = store;
    this.n = -1;
  }

  refresh() {
    const cs = this.store.candles;
    if(this.n === cs.length) return;
    const n = cs.length;
    this.n = n;
    if(n < 2) return;

    const close = new Float64Array(n);
    const high = new Float64Array(n);
    const low = new Float64Array(n);
    const vol = new Float64Array(n);
    for(let i = 0; i < n; i++) {
      close[i] = cs[i].close;
      high[i] = cs[i].high;
      low[i] = cs[i].low;
      vol[i] = cs[i].volume;
    }
    this.close = close;
    this.high = high;
    this.low = low;
    this.vol = vol;

    // ── SMA(20) + Bollinger (20, 2σ) ──
    this.sma20 = rollingMean(close, 20);
    const std20 = rollingStd(close, 20);
    this.bbUpper = new Float64Array(n);
    this.bbLower = new Float64Array(n);
    for(let i = 0; i < n; i++) {
      this.bbUpper[i] = this.sma20[i] + 2 * std20[i];
      this.bbLower[i] = this.sma20[i] - 2 * std20[i];
    }

    // ── True range → ATR (Wilder EWM 14) ──
    const tr = new Float64Array(n);
    tr[0] = high[0] - low[0];
    for(let i = 1; i < n; i++) {
      tr[i] = Math.max(high[i] - low[i], Math.abs(high[i] - close[i - 1]), Math.abs(low[i] - close[i - 1]));
    }
    this.atrAbs = ewm(tr, 1 / 14);
    this.atrRel = new Float64Array(n);
    for(let i = 0; i < n; i++) this.atrRel[i] = this.atrAbs[i] / close[i];

    // ── RSI(14, Wilder) ──
    const up = new Float64Array(n);
    const dn = new Float64Array(n);
    for(let i = 1; i < n; i++) {
      const d = close[i] - close[i - 1];
      up[i] = d > 0 ? d : 0;
      dn[i] = d < 0 ? -d : 0;
    }
    const avgUp = ewm(up, 1 / 14);
    const avgDn = ewm(dn, 1 / 14);
    this.rsi = new Float64Array(n);
    for(let i = 0; i < n; i++) {
      const rs = avgUp[i] / (avgDn[i] + 1e-12);
      this.rsi[i] = 100 - 100 / (1 + rs);
    }

    // ── MACD histogram (12, 26, 9); α = 2/(span+1), recursive ──
    const ema12 = ewm(close, 2 / 13);
    const ema26 = ewm(close, 2 / 27);
    const macd = new Float64Array(n);
    for(let i = 0; i < n; i++) macd[i] = ema12[i] - ema26[i];
    const macdSig = ewm(macd, 2 / 10);
    this.macdHist = new Float64Array(n);
    for(let i = 0; i < n; i++) this.macdHist[i] = macd[i] - macdSig[i];

    // ── VWAP over rolling 24 bars ──
    const cv = new Float64Array(n);
    for(let i = 0; i < n; i++) cv[i] = close[i] * vol[i];
    const cvSum = rollingSum(cv, 24);
    const vSum = rollingSum(vol, 24);
    this.vwap24 = new Float64Array(n);
    for(let i = 0; i < n; i++) this.vwap24[i] = cvSum[i] / (vSum[i] + 1e-12);

    // ── Volume z-score (48) ──
    const volMean = rollingMean(vol, 48);
    const volStd = rollingStd(vol, 48);
    this.volZ = new Float64Array(n);
    for(let i = 0; i < n; i++) this.volZ[i] = (vol[i] - volMean[i]) / (volStd[i] + 1e-12);

    // ── ADX(14, Wilder) — trend strength; regime uses this ──
    const pdm = new Float64Array(n);
    const ndm = new Float64Array(n);
    for(let i = 1; i < n; i++) {
      const upM = high[i] - high[i - 1];
      const dnM = low[i - 1] - low[i];
      pdm[i] = upM > dnM && upM > 0 ? upM : 0;
      ndm[i] = dnM > upM && dnM > 0 ? dnM : 0;
    }
    const pdmS = ewm(pdm, 1 / 14);
    const ndmS = ewm(ndm, 1 / 14);
    const dx = new Float64Array(n);
    for(let i = 0; i < n; i++) {
      const pdi = (100 * pdmS[i]) / (this.atrAbs[i] + 1e-12);
      const ndi = (100 * ndmS[i]) / (this.atrAbs[i] + 1e-12);
      dx[i] = (100 * Math.abs(pdi - ndi)) / (pdi + ndi + 1e-12);
    }
    this.adx = ewm(dx, 1 / 14);

    // ── RegimeFilter.ok per bar ──
    this.regimeOk = new Uint8Array(n);
    for(let i = 0; i < n; i++) {
      this.regimeOk[i] = this.adx[i] > REGIME_ADX_MIN && this.atrRel[i] > REGIME_ATR_MIN ? 1 : 0;
    }
  }

  // RiskManager.position_size mirror — returns {size, sizeEur}
  positionSize(equity, i) {
    if(i < 0 || i >= this.n) return { size: 0, sizeEur: 0 };
    const price = this.close[i];
    const atrRel = this.atrRel[i];
    if(!isFinite(atrRel) || atrRel <= 0) return { size: 0, sizeEur: 0 };
    const riskEur = TARGET_VOL * equity;
    const sizeEur = Math.min(riskEur / (STOP_ATR * atrRel), MAX_POS_FRAC * equity);
    return { size: sizeEur / price, sizeEur };
  }
}

// Evaluate every gate the bot checks in on_bar(), so we can render *why*
// it did (or would have) HOLD/ENTER/HALT on this bar. Returns null if the
// bar index is out of range.
function evalGate(store, indicators, idx) {
  if(idx < 0 || idx >= store.candles.length || !indicators.adx) return null;
  const c = store.candles[idx];
  const pred = store.predByTs.get(c.ts);
  const st = store.state;

  const adx = indicators.adx[idx];
  const atrRel = indicators.atrRel[idx];
  const adxPass = adx > REGIME_ADX_MIN;
  const atrPass = atrRel > REGIME_ATR_MIN;
  const regimePass = adxPass && atrPass;

  const dirP = pred ? Math.max(pred.p_long, pred.p_short) : NaN;
  const dirPass = pred ? dirP >= ENTRY_DIR : null;
  const metaPass = pred ? pred.meta_p >= ENTRY_META : null;

  const halted = st.halted === true;
  const dayStartEq = st.day_start_eq ?? st.equity ?? START_EQUITY;
  const equity = st.equity ?? START_EQUITY;
  const dayPnl = equity - dayStartEq;
  const dayLimit = dayPnl <= -DAILY_LOSS_LIMIT * dayStartEq;
  const riskPass = !halted && !dayLimit;

  const totalPreds = store.predByTs.size;
  const modelVer = pred?.model_ver ?? st.model_version ?? 'untrained';
  const barsReady = store.candles.length >= MIN_BARS_FOR_PRED;

  let decision;
  let why;
  if(halted) {
    decision = 'HALT';
    why = 'kill switch (max drawdown hit) — manual reset-halt needed';
  } else if(dayLimit) {
    decision = 'HALT';
    why = `daily loss limit hit (${dayPnl.toFixed(2)})`;
  } else if(!barsReady) {
    decision = 'HOLD';
    why = `only ${store.candles.length} bars — need ≥ ${MIN_BARS_FOR_PRED}`;
  } else if(totalPreds === 0) {
    decision = 'HOLD';
    why = 'model never ran — no predictions logged in DB';
  } else if(!pred) {
    decision = 'HOLD';
    why = 'no prediction for this bar (model missed / gap)';
  } else if(!regimePass) {
    decision = 'HOLD';
    why = 'regime off — chop / low volatility\nfees would eat edge';
  } else if(!dirPass) {
    decision = 'HOLD';
    why = `direction prob ${dirP.toFixed(3)} < ${ENTRY_DIR}`;
  } else if(!metaPass) {
    decision = 'HOLD';
    why = `meta prob ${pred.meta_p.toFixed(3)} < ${ENTRY_META}`;
  } else {
    decision = 'ENTER';
    why = (pred.p_long > pred.p_short ? 'LONG' : 'SHORT') + ' cleared all gates';
  }

  return {
    pred,
    adx,
    atrRel,
    adxPass,
    atrPass,
    regimePass,
    dirP,
    dirPass,
    metaPass,
    riskPass,
    halted,
    dayLimit,
    dayPnl,
    equity,
    totalPreds,
    modelVer,
    barsReady,
    decision,
    why,
  };
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
    const { vg, store, view, replay, indicators } = app;
    const r = this.rect;
    const maxIdx = replay.maxIdx;
    const cs = store.candles;
    view.updatePriceRange(maxIdx);

    vg.Save();
    vg.IntersectScissor(r.x, r.y, r.w, r.h);

    const bw = view.barW(r);
    const a = Math.max(0, Math.floor(view.i0));
    const b = Math.min(maxIdx, Math.ceil(view.i1));

    // regime shading — driven by the *replicated* RegimeFilter so every
    // bar is annotated, not just the ones the bot happened to log.
    vg.FillColor(COL.regime);
    if(indicators.regimeOk) {
      for(let i = a; i <= b; i++) {
        if(!indicators.regimeOk[i]) {
          vg.BeginPath();
          vg.Rect(view.xOfIndex(i, r), r.y, bw, r.h);
          vg.Fill();
        }
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

    this.drawOverlays(app, a, b);
    this.drawTrades(app, a, b);
    this.drawHypotheticalBrackets(app);
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

    // legend + gate status card (drawn unclipped, on top of everything)
    this.drawLegend(app);
    this.drawGateStatus(app);
  }

  // draw a price-space line from an indicator array, skipping NaN
  drawSeries(app, arr, a, b, color, width) {
    const { vg, view } = app;
    const r = this.rect;
    const bw = view.barW(r);
    vg.BeginPath();
    let started = false;
    for(let i = a; i <= b; i++) {
      const v = arr[i];
      if(!isFinite(v)) continue;
      const x = view.xOfIndex(i, r) + bw / 2;
      const y = view.yOfPrice(v, r);
      if(!started) {
        vg.MoveTo(x, y);
        started = true;
      } else vg.LineTo(x, y);
    }
    vg.StrokeColor(color);
    vg.StrokeWidth(width);
    vg.Stroke();
  }

  drawOverlays(app, a, b) {
    const { indicators } = app;
    if(!indicators.sma20) return;
    this.drawSeries(app, indicators.bbUpper, a, b, COL.bbBand, 1);
    this.drawSeries(app, indicators.bbLower, a, b, COL.bbBand, 1);
    this.drawSeries(app, indicators.vwap24, a, b, COL.vwap, 1.2);
    this.drawSeries(app, indicators.sma20, a, b, COL.sma, 1.4);
  }

  // dashed stop/TP levels at the right edge, showing what a hypothetical
  // entry at the current bar's close would target — reads exactly what
  // RiskManager+bot would place (stop_atr / tp_atr × ATR).
  drawHypotheticalBrackets(app) {
    const { vg, view, indicators, replay } = app;
    const r = this.rect;
    const idx = replay.maxIdx;
    if(idx < 0 || !indicators.atrAbs) return;
    const atr = indicators.atrAbs[idx];
    const price = indicators.close[idx];
    if(!isFinite(atr) || !isFinite(price)) return;

    const x0 = view.xOfIndex(idx, r) + view.barW(r) / 2;
    const x1 = r.x + r.w - view.axisW;
    if(x0 > x1 - 6) return;

    const tp = price + TP_ATR * atr;
    const sl = price - STOP_ATR * atr;
    vg.StrokeWidth(1);
    vg.StrokeColor(COL.tp);
    dashedLine(vg, x0, view.yOfPrice(tp, r), x1, view.yOfPrice(tp, r), 2, 6);
    vg.StrokeColor(COL.stop);
    dashedLine(vg, x0, view.yOfPrice(sl, r), x1, view.yOfPrice(sl, r), 2, 6);

    app.font(9, COL.tp, ALIGN_RIGHT | ALIGN_MIDDLE);
    app.text(x1 - 4, view.yOfPrice(tp, r) - 7, `hyp TP +${TP_ATR}×ATR`);
    app.font(9, COL.stop, ALIGN_RIGHT | ALIGN_MIDDLE);
    app.text(x1 - 4, view.yOfPrice(sl, r) + 7, `hyp SL -${STOP_ATR}×ATR`);
  }

  drawLegend(app) {
    const { vg } = app;
    const r = this.rect;
    const y = r.y + r.h - 14;
    let x = r.x + 12;
    const chip = (color, label, w) => {
      vg.BeginPath();
      vg.MoveTo(x, y);
      vg.LineTo(x + 18, y);
      vg.StrokeColor(color);
      vg.StrokeWidth(w);
      vg.Stroke();
      app.font(10, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(x + 22, y, label);
      x += 22 + label.length * 6 + 12;
    };
    chip(COL.sma, 'SMA(20)', 1.4);
    chip(COL.bbBand, 'BB(20,2σ)', 1);
    chip(COL.vwap, 'VWAP(24)', 1.2);
    chip(COL.regime, 'regime-off shade', 6);
  }

  drawGateStatus(app) {
    const { vg, store, indicators, replay } = app;
    const r = this.rect;
    const idx = replay.maxIdx;
    const g = evalGate(store, indicators, idx);
    if(!g) return;
    const c = store.candles[idx];

    const cardW = 306;
    const cardH = 224;
    const cardX = r.x + 12;
    const cardY = r.y + 12;

    vg.BeginPath();
    vg.RoundedRect(cardX, cardY, cardW, cardH, 6);
    vg.FillColor(COL.cardBg);
    vg.Fill();
    vg.StrokeColor(COL.cardEdge);
    vg.StrokeWidth(1);
    vg.Stroke();

    app.font(12, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(cardX + 12, cardY + 14, `GATE STATUS — ${fmtTs(c.ts)}`);

    // subheader: model + bars context
    app.font(10, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(cardX + 12, cardY + 30, `model: ${g.modelVer}   (${g.totalPreds} preds logged)`);
    app.text(cardX + 12, cardY + 44, `bars: ${store.candles.length} (min ${MIN_BARS_FOR_PRED})   equity: ${fmtNum(g.equity)}`);

    // divider
    const hr = y => {
      vg.BeginPath();
      vg.MoveTo(cardX + 8, y);
      vg.LineTo(cardX + cardW - 8, y);
      vg.StrokeColor(COL.grid);
      vg.StrokeWidth(1);
      vg.Stroke();
    };
    hr(cardY + 54);

    const row = (y, label, val, pass) => {
      const dotCol = pass === null || pass === undefined ? COL.gateSkip : pass ? COL.gateOk : COL.gateFail;
      vg.BeginPath();
      vg.Circle(cardX + 14, y, 4);
      vg.FillColor(dotCol);
      vg.Fill();
      app.font(11, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(cardX + 26, y, label);
      if(val !== null && val !== undefined) {
        const valCol = pass === false ? COL.gateFail : COL.text;
        app.font(11, valCol, ALIGN_RIGHT | ALIGN_MIDDLE);
        app.text(cardX + cardW - 12, y, val);
      }
    };

    let y = cardY + 68;
    const rowH = 17;
    row(y, 'Regime filter', g.regimePass ? 'PASS' : 'FAIL', g.regimePass);
    y += rowH;
    row(y, `  ADX > ${REGIME_ADX_MIN}`, `${g.adx.toFixed(1)}`, g.adxPass);
    y += rowH;
    row(y, `  ATR% > ${(REGIME_ATR_MIN * 100).toFixed(2)}%`, `${(g.atrRel * 100).toFixed(2)}%`, g.atrPass);
    y += rowH + 3;

    if(g.pred) {
      row(y, `Direction ≥ ${ENTRY_DIR}`, `${g.dirP.toFixed(3)}`, g.dirPass);
      y += rowH;
      row(y, `Meta ≥ ${ENTRY_META}`, `${g.pred.meta_p.toFixed(3)}`, g.metaPass);
    } else {
      row(y, `Direction ≥ ${ENTRY_DIR}`, g.totalPreds === 0 ? 'no model' : 'n/a', null);
      y += rowH;
      row(y, `Meta ≥ ${ENTRY_META}`, g.totalPreds === 0 ? 'no model' : 'n/a', null);
    }
    y += rowH + 3;

    const riskLabel = g.halted ? 'HALTED' : g.dayLimit ? 'DAY LIMIT' : 'ok';
    row(y, 'Risk / kill switch', riskLabel, g.riskPass);
    y += rowH + 5;

    hr(y);
    y += 12;
    const decCol = g.decision === 'ENTER' ? COL.gateOk : g.decision === 'HALT' ? COL.gateFail : COL.text;
    app.font(12, decCol, ALIGN_LEFT | ALIGN_MIDDLE);

    let lines = g.why.split('\n');

    for(let i = 0; i < lines.length; i++) {
      app.text(cardX + 12, y + i * 12, (i == 0 ? `→ ${g.decision}: ` : `              `) + lines[i]);
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
    const { vg, store, view, replay, mouse, indicators } = app;
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
    if(indicators.rsi && isFinite(indicators.rsi[i])) {
      const vwapD = ((c.close / indicators.vwap24[i] - 1) * 100).toFixed(2);
      lines.push(
        `RSI ${indicators.rsi[i].toFixed(1)}  MACDh ${(indicators.macdHist[i] * 100).toFixed(3)}%  ADX ${indicators.adx[i].toFixed(1)}  ATR% ${(indicators.atrRel[i] * 100).toFixed(2)}%  vwap-d ${vwapD}%  volZ ${indicators.volZ[i].toFixed(2)}`,
      );
      const okAdx = indicators.adx[i] > REGIME_ADX_MIN;
      const okAtr = indicators.atrRel[i] > REGIME_ATR_MIN;
      lines.push(`regime: adx>${REGIME_ADX_MIN}? ${okAdx ? 'yes' : 'NO'}   atr>${(REGIME_ATR_MIN * 100).toFixed(2)}%? ${okAtr ? 'yes' : 'NO'}   → ${indicators.regimeOk[i] ? 'TRADEABLE' : 'off'}`);
    }
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

// RegimePane — shows ADX and ATR% as *ratios* to their trade-enable
// thresholds. Everything above y = 1.0 is a passing gate; the shaded
// columns are the exact bars where CandlePane also greys out.
class RegimePane extends Pane {
  draw(app) {
    const { vg, view, indicators, store, replay } = app;
    const r = this.rect;
    vg.Save();
    vg.IntersectScissor(r.x, r.y, r.w, r.h);

    if(!indicators.adx) {
      app.font(11, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(r.x + 6, r.y + 10, 'regime (indicators warming up…)');
      vg.Restore();
      return;
    }

    const yMax = 3;
    const yOf = v => r.y + r.h * (1 - Math.min(v, yMax) / yMax);

    const bw = view.barW(r);
    const a = Math.max(0, Math.floor(view.i0));
    const b = Math.min(replay.maxIdx, Math.ceil(view.i1));

    // regime-off shading (matches CandlePane)
    vg.FillColor(COL.regime);
    for(let i = a; i <= b; i++) {
      if(!indicators.regimeOk[i]) {
        vg.BeginPath();
        vg.Rect(view.xOfIndex(i, r), r.y, bw, r.h);
        vg.Fill();
      }
    }

    // gridline at ratio = 2 (comfortably above threshold)
    vg.StrokeColor(COL.grid);
    vg.StrokeWidth(1);
    vg.BeginPath();
    vg.MoveTo(r.x, yOf(2));
    vg.LineTo(r.x + r.w - view.axisW, yOf(2));
    vg.Stroke();

    // pass/fail threshold at ratio = 1
    vg.StrokeColor(COL.thresh);
    dashedLine(vg, r.x, yOf(1), r.x + r.w - view.axisW, yOf(1), 3, 5);

    const drawRatio = (getRaw, thr, color) => {
      vg.BeginPath();
      let started = false;
      for(let i = a; i <= b; i++) {
        const raw = getRaw(i);
        if(!isFinite(raw)) continue;
        const v = raw / thr;
        const x = view.xOfIndex(i, r) + bw / 2;
        const y = yOf(v);
        if(!started) {
          vg.MoveTo(x, y);
          started = true;
        } else vg.LineTo(x, y);
      }
      vg.StrokeColor(color);
      vg.StrokeWidth(1.3);
      vg.Stroke();
    };
    drawRatio(i => indicators.adx[i], REGIME_ADX_MIN, COL.sma);
    drawRatio(i => indicators.atrRel[i], REGIME_ATR_MIN, COL.vwap);

    // labels
    app.font(11, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(r.x + 6, r.y + 10, `regime: ADX/${REGIME_ADX_MIN} (navy)  &  ATR%/${(REGIME_ATR_MIN * 100).toFixed(1)}% (ochre)  — both must exceed 1`);
    app.font(10, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(r.x + r.w - view.axisW + 6, yOf(1), 'thresh');
    app.text(r.x + r.w - view.axisW + 6, yOf(2), '2×');

    // current values readout on the right
    const idx = replay.maxIdx;
    if(idx >= 0 && idx < store.candles.length && isFinite(indicators.adx[idx])) {
      const adx = indicators.adx[idx];
      const atrPct = indicators.atrRel[idx] * 100;
      const ok = indicators.regimeOk[idx];
      app.font(11, ok ? COL.gateOk : COL.gateFail, ALIGN_RIGHT | ALIGN_MIDDLE);
      app.text(r.x + r.w - view.axisW - 8, r.y + 10, `ADX ${adx.toFixed(1)}   ATR ${atrPct.toFixed(2)}%   ${ok ? 'TRADEABLE' : 'OFF'}`);
    }

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
    this.indicators = new Indicators(this.store);
    this.indicators.refresh();
    this.view = new Viewport(this.store);
    this.replay = new Replay(this.store);
    this.follow = true;

    this.candlePane = new CandlePane();
    this.signalPane = new SignalPane();
    this.regimePane = new RegimePane();
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
    const candleH = Math.floor(rest * 0.52);
    const sigH = Math.floor(rest * 0.16);
    const regH = Math.floor(rest * 0.14);
    const eqH = rest - candleH - sigH - regH;

    let y = 0;
    this.candlePane.layout(0, y, w, candleH);
    y += candleH;
    this.signalPane.layout(0, y, w, sigH);
    y += sigH;
    this.regimePane.layout(0, y, w, regH);
    y += regH;
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
        switch (keyCode) {
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
    this.indicators.refresh();
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
    this.regimePane.draw(this);
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
