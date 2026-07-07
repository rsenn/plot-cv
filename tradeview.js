// tradeview.js — GUI viewer for tradebot.py's SQLite database.
//
// Renders candles, model predictions, bot actions, trade brackets and the
// equity curve from tradebot.db — live (WAL polling) or as bar-by-bar replay.
//
// ─── Primer for JS devs who've never traded ─────────────────────────────
// The full walkthrough lives in "tradeview - Complete Documentation.md".
// Quick glossary so the source reads sensibly:
//   candle       one time slice as (open, high, low, close, volume). A "1h
//                candle" = the first/last/max/min trade price over that
//                hour plus the total traded amount.
//   long/short   long = bought, wins if price rises. short = "sold-what-
//                you-don't-own", wins if price falls. Spot exchanges don't
//                let you short, so the bot only shorts in paper mode.
//   SMA(N)      simple moving average — mean of the last N closes. Slow
//                trend anchor.
//   Bollinger   SMA ± 2·rolling_std. An envelope of "typical" prices.
//   VWAP        volume-weighted mean of close — "fair" intraday price.
//   ATR         average true range = "average candle size". Volatility
//                yardstick. Stops are placed a few ATRs away so calm and
//                wild markets get proportional room.
//   ADX         trend-strength index in [0..100]. >20 ≈ trending, <20 ≈
//                sideways chop.
//   regime OK   the deterministic pre-filter tradebot uses: ADX > 20 AND
//                ATR% > 2.5·fee_rate. Off = market too flat, fees would
//                eat the edge.
//   gate        the ordered pipeline of yes/no checks the bot runs before
//                opening a trade; the Gate Status card enumerates them.
//   drawdown    percent fall from all-time equity peak. 10% triggers a
//                kill switch.
//   equity      simulated (paper mode) or real account value in EUR.
// The full doc explains each in more depth.
//
// Layout, top to bottom:
//   CandlePane  — candles, MA/BB/VWAP overlays, regime shading, trade
//                 brackets, entry/exit markers, Gate Status card
//   SignalPane  — p_long / p_short / meta_p with entry thresholds
//   RegimePane  — ADX and ATR% plotted as ratios to their pass thresholds
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
//
// If a value looks arbitrary: it usually isn't. Comments spell out the
// trading reasoning so a JS-only reader can follow.
const FEE_RATE = 0.006; // Coinbase taker fee, 0.6% per side. Any strategy
//                              must clear this per round-trip just to be flat.
const ENTRY_DIR = 0.45; // "will price move" prob must reach this to enter
const ENTRY_META = 0.58; // second-opinion prob "trust the direction?" gate
const REGIME_ADX_MIN = 20; // ADX (trend strength 0..100) — under 20 = chop
const REGIME_ATR_MIN = 2.5 * FEE_RATE; // avg candle must span ≥2.5× fees, or
//                              even a correct call can't pay for the trade
const SEQ_LEN = 64; // LSTM lookback (bars). Purely informational here.
const MIN_BARS_FOR_PRED = SEQ_LEN + 60; // bot needs this much history to
//                              even *attempt* a prediction (see Bot.on_bar)
const STOP_ATR = 2.0; // stop-loss placed 2× ATR from entry
const TP_ATR = 3.0; // take-profit placed 3× ATR from entry (3:2 R:R)
const TARGET_VOL = 0.01; // risk 1% of equity per trade
const MAX_POS_FRAC = 0.25; // hard cap: no single trade ≥ 25% of equity
const DAILY_LOSS_LIMIT = 0.02; // -2% day → pause until next UTC day
const MAX_DRAWDOWN = 0.1; // -10% from peak → HALT (manual reset required)
const START_EQUITY = 10_000.0; // paper starting capital, in EUR

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
  rowHover: RGBA(80, 60, 40, 32),
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

// Greedy word-wrap by *approximate* pixel width, using the same 6.4 px/char
// factor the crosshair tooltip uses. Handles embedded '\n' as forced breaks.
function wrapText(text, maxWidth, charW = 6.4, pad = 20) {
  const maxLen = Math.max(4, Math.floor((maxWidth - pad) / charW));
  const out = [];
  for(const para of String(text).split('\n')) {
    const words = para.split(' ');
    let cur = '';
    for(const w of words) {
      const test = cur ? cur + ' ' + w : w;
      if(test.length > maxLen && cur.length > 0) {
        out.push(cur);
        cur = w;
      } else cur = test;
    }
    if(cur.length) out.push(cur);
  }
  return out;
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

// ewm — one-pole IIR low-pass:  y[i] = α · x[i] + (1-α) · y[i-1]
// Same recurrence you'd use for a signal-processing exponential smoother,
// and it's what pandas .ewm(adjust=False) does. Small α = long memory
// (smooth), large α = reactive. "Wilder 14" (α = 1/14) is the classic
// technical-analysis smoothing. The first output = first input, so the
// filter warms up over roughly 1/α samples.
function ewm(v, alpha) {
  const n = v.length;
  const out = new Float64Array(n);
  if(n === 0) return out;
  out[0] = v[0];
  for(let i = 1; i < n; i++) out[i] = alpha * v[i] + (1 - alpha) * out[i - 1];
  return out;
}

// rollingMean — mean of the last `w` samples, one output per input. The
// first (w-1) outputs are NaN because the window isn't full yet. Runs
// with an O(1) update per step (add newcomer, drop the one falling off).
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

// rollingSum — like rollingMean but without the divide. Used for VWAP
// where we need Σ(price·vol) and Σ(vol) separately.
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
    // SMA = plain arithmetic mean over the last 20 closes → slow trend line.
    // Bollinger = SMA ± 2·(sample std of the same window). ~95% of closes
    // sit inside the band; touches of the outer bands are "unusually far
    // from average". Traders read a squeeze (narrow band) as pent-up
    // volatility about to release.
    this.sma20 = rollingMean(close, 20);
    const std20 = rollingStd(close, 20);
    this.bbUpper = new Float64Array(n);
    this.bbLower = new Float64Array(n);
    for(let i = 0; i < n; i++) {
      this.bbUpper[i] = this.sma20[i] + 2 * std20[i];
      this.bbLower[i] = this.sma20[i] - 2 * std20[i];
    }

    // ── True range → ATR (Wilder EWM 14) ──
    // True range = candle height in "gap-aware" terms:
    //   TR = max(high−low, |high−prev_close|, |low−prev_close|)
    // The last two terms handle gaps between yesterday's close and today's
    // open. ATR = Wilder-smoothed TR → "typical candle size, in price
    // units, right now". atrRel = ATR / price → same thing as a fraction
    // of price (e.g. 0.008 = "candles are typically ±0.8% tall").
    const tr = new Float64Array(n);
    tr[0] = high[0] - low[0];
    for(let i = 1; i < n; i++) {
      tr[i] = Math.max(high[i] - low[i], Math.abs(high[i] - close[i - 1]), Math.abs(low[i] - close[i - 1]));
    }
    this.atrAbs = ewm(tr, 1 / 14);
    this.atrRel = new Float64Array(n);
    for(let i = 0; i < n; i++) this.atrRel[i] = this.atrAbs[i] / close[i];

    // ── RSI(14, Wilder) ──
    // Momentum oscillator in [0..100]. Split close-to-close moves into
    // "up part" (positive delta) and "down part" (magnitude of negative
    // delta), Wilder-smooth each, then rs = avgUp / avgDn and
    //   rsi = 100 − 100 / (1 + rs).
    // Convention: RSI < 30 = "oversold" (recent drops are extreme),
    // > 70 = "overbought". Not a signal on its own — a tension gauge.
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

    // ── MACD histogram (12, 26, 9) ──
    // MACD = fast EMA − slow EMA of close (12 vs 26 bars). Signal =
    // 9-bar EMA of MACD. Histogram = MACD − signal. It's a differentiator:
    // hist crossing zero upward means the fast average just overtook the
    // slow one, historically a trend-flip cue. We store the hist / close
    // ratio for scale independence. α = 2/(span+1) is the standard
    // conversion from pandas "span" to the recursive smoother's decay.
    const ema12 = ewm(close, 2 / 13);
    const ema26 = ewm(close, 2 / 27);
    const macd = new Float64Array(n);
    for(let i = 0; i < n; i++) macd[i] = ema12[i] - ema26[i];
    const macdSig = ewm(macd, 2 / 10);
    this.macdHist = new Float64Array(n);
    for(let i = 0; i < n; i++) this.macdHist[i] = macd[i] - macdSig[i];

    // ── VWAP over rolling 24 bars ──
    // VWAP = Σ(price·volume) / Σ(volume) over a window. It's the mean
    // price weighted by how much actually traded. Institutional traders
    // measure fills against VWAP ("did I get a good price today?"), and
    // (close/vwap − 1) tells you whether current price sits above or
    // below the volume-weighted centre of the last day.
    const cv = new Float64Array(n);
    for(let i = 0; i < n; i++) cv[i] = close[i] * vol[i];
    const cvSum = rollingSum(cv, 24);
    const vSum = rollingSum(vol, 24);
    this.vwap24 = new Float64Array(n);
    for(let i = 0; i < n; i++) this.vwap24[i] = cvSum[i] / (vSum[i] + 1e-12);

    // ── Volume z-score (48) ──
    // Standard z-score of volume vs its 48-bar mean and std. |z| > 2 means
    // "unusually heavy volume for this hour of the week" — a candle worth
    // paying attention to.
    const volMean = rollingMean(vol, 48);
    const volStd = rollingStd(vol, 48);
    this.volZ = new Float64Array(n);
    for(let i = 0; i < n; i++) this.volZ[i] = (vol[i] - volMean[i]) / (volStd[i] + 1e-12);

    // ── ADX(14, Wilder) — trend strength; regime uses this ──
    // Wilder's Directional Movement:
    //   +DM = "how much higher" today's high is vs yesterday's, but only
    //         if that up-move exceeds the down-move (else 0). −DM mirrors.
    // Smooth both with the Wilder EWM, divide by ATR to get +DI and −DI
    // (directional indexes). Their normalized gap
    //   DX = 100 · |+DI − −DI| / (+DI + −DI)
    // measures how one-sided moves have been. ADX = smoothed DX ∈ [0..100]:
    // > 20 ≈ trending, < 20 ≈ chopping. tradebot uses > 20 as the pass gate.
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

  // RiskManager.position_size mirror.
  //
  // Volatility targeting: instead of "always buy X coins", risk a fixed
  // *fraction of equity* per trade (TARGET_VOL, 1%). The stop is at
  // STOP_ATR × ATR away — so if the stop hits, we lose ~(stop distance ×
  // position size), and we want that to equal riskEur. Solve for size:
  //   sizeEur = riskEur / (STOP_ATR × atrRel)
  // then hard-cap at MAX_POS_FRAC (25%) of equity so a single trade can't
  // blow the account even if the stop slips. Finally divide by price to
  // convert EUR notional into base-coin units (e.g. BTC).
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
// it did (or would have) HOLD/ENTER/HALT on this bar.
//
// The pipeline order matters — it's a short-circuit chain, and the first
// failing gate is the one we surface to the user:
//   1. halted?                       → HALT (kill switch)
//   2. daily loss limit hit today?   → HALT (paused till next UTC day)
//   3. enough bars to predict?       → HOLD (warmup)
//   4. any predictions logged ever?  → HOLD (model never ran)
//   5. prediction for THIS bar?      → HOLD (gap in log)
//   6. regime OK (adx & atr)?        → HOLD (market too flat / choppy)
//   7. direction prob ≥ 0.45?        → HOLD (model unsure)
//   8. meta prob ≥ 0.58?             → HOLD (meta model distrusts primary)
//   9. → ENTER (LONG or SHORT)
// Returns null if the bar index is out of range.
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
    why = 'regime off — chop / low volatility, fees would eat edge';
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
    //
    // Anatomy of a candlestick:
    //   ─┬─       ← high (top of upper wick)
    //    │
    //   ┌┴┐  ← body top    = max(open, close)
    //   │ │  ← body        = filled green if close ≥ open (bull), red else
    //   └┬┘  ← body bottom = min(open, close)
    //    │
    //   ─┴─       ← low  (bottom of lower wick)
    //
    // So we draw two things per bar: the thin wick line (high→bodyTop and
    // bodyBottom→low as two separate segments so it doesn't cross the
    // filled body) and the body rectangle. `forming` = the last live bar
    // isn't closed yet, so we draw its body as an outline to signal
    // "still mutating".
    const now = Date.now() / 1000;
    for(let i = a; i <= b; i++) {
      const c = cs[i];
      const x = view.xOfIndex(i, r);
      const cx = x + bw / 2; // centre of the bar column, where the wick sits
      const up = c.close >= c.open;
      const col = up ? COL.upFill : COL.dnFill;
      const yB0 = view.yOfPrice(Math.max(c.open, c.close), r); // body top
      const yB1 = view.yOfPrice(Math.min(c.open, c.close), r); // body bottom
      const forming = replay.mode === 'LIVE' && i === cs.length - 1 && now - c.ts < store.tf;

      // wicks — two segments so the line doesn't cross the body fill
      vg.BeginPath();
      vg.MoveTo(cx, view.yOfPrice(c.high, r));
      vg.LineTo(cx, yB0);
      vg.MoveTo(cx, yB1);
      vg.LineTo(cx, view.yOfPrice(c.low, r));
      vg.StrokeColor(up ? COL.up : COL.dn);
      vg.StrokeWidth(Math.max(1, bw * 0.08));
      vg.Stroke();

      // body: centred rectangle taking 70% of column width
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

  // Rect for the Gate Status card. Base position is top-left of the candle
  // pane, offset by app.gateCardOffset (user can drag the card). Height is
  // dynamic — grown to fit the wrapped decision text — and cached from the
  // previous frame in this.gateCardH (one-frame lag is invisible).
  _gateCardRect(app) {
    const r = this.rect;
    const off = app.gateCardOffset;
    return { x: r.x + 12 + off.dx, y: r.y + 12 + off.dy, w: 306, h: this.gateCardH || 224 };
  }

  drawGateStatus(app) {
    const { vg, store, indicators, replay, mouse } = app;
    const gc = this._gateCardRect(app);
    const idx = replay.maxIdx;
    const g = evalGate(store, indicators, idx);
    if(!g) return;
    const c = store.candles[idx];

    const cardX = gc.x;
    const cardY = gc.y;
    const cardW = gc.w;
    const rowH = 17;

    // ── Build hotspots (rect + tooltip text) as data. Everything painted
    //    below reads from these arrays, so hit-testing and rendering can't
    //    disagree about where a row is.
    const hotspots = [];
    const hs = (y, h, tt) => hotspots.push({ y, h, tt });

    // header rows
    hs(cardY + 14, 14, "The bar this card describes. In replay use ←/→ to step — every gate below re-evaluates for that bar's state.");
    hs(cardY + 30, 14, `Model version = timestamp of the last successful train. "(N preds logged)" = bars for which the ML stack ran and wrote a row into the predictions table. If 0, the bot has never trained; run 'python tradebot.py train'.`);
    hs(cardY + 44, 14, `Bars available in the DB. LSTM needs ≥ SEQ_LEN(64) + feature warmup(60) = ${MIN_BARS_FOR_PRED} bars to attempt any prediction. Equity = paper account value, simulated from €${START_EQUITY.toFixed(0)}.`);

    // gate rows
    const gateRows = [];
    let y = cardY + 68;

    gateRows.push({
      y, label: 'Regime filter', val: g.regimePass ? 'PASS' : 'FAIL', pass: g.regimePass,
      tt: 'Compound gate: both ADX and ATR% must clear. This one filter probably matters more than the entire ML stack — trading in choppy, low-volatility markets is how accounts bleed to fees.',
    });
    y += rowH;
    gateRows.push({
      y, label: `  ADX > ${REGIME_ADX_MIN}`, val: g.adx.toFixed(1), pass: g.adxPass,
      tt: 'Average Directional Index ∈ [0..100]. Measures trend STRENGTH (not direction). >20 ≈ a real trend exists; <20 ≈ sideways chop. Wilder\'s original 1978 threshold.',
    });
    y += rowH;
    gateRows.push({
      y, label: `  ATR% > ${(REGIME_ATR_MIN * 100).toFixed(2)}%`, val: `${(g.atrRel * 100).toFixed(2)}%`, pass: g.atrPass,
      tt: `ATR ÷ price = "how tall are candles?". Threshold = 2.5 × the taker fee (0.6%). Below this, even a perfectly correct call can't cover the round-trip fee + slippage — the market is too calm to pay for the trade.`,
    });
    y += rowH + 3;

    if(g.pred) {
      gateRows.push({
        y, label: `Direction ≥ ${ENTRY_DIR}`, val: g.dirP.toFixed(3), pass: g.dirPass,
        tt: 'max(p_long, p_short) from the LSTM + XGBoost primary ensemble. Read as "model\'s confidence that price will move 1.5×ATR within 12 bars, in either direction". Below 0.45 the model is essentially guessing between up/flat/down.',
      });
      y += rowH;
      gateRows.push({
        y, label: `Meta ≥ ${ENTRY_META}`, val: g.pred.meta_p.toFixed(3), pass: g.metaPass,
        tt: 'Second-opinion XGBoost, trained on the question "was the primary signal actually right?". A meta model that grades whether to trust the primary. Filters over-confident false positives from the LSTM+XGB ensemble.',
      });
    } else {
      const na = g.totalPreds === 0 ? 'no model' : 'n/a';
      gateRows.push({
        y, label: `Direction ≥ ${ENTRY_DIR}`, val: na, pass: null,
        tt: 'max(p_long, p_short) from the LSTM + XGBoost ensemble. Blank because no prediction was logged for this bar (see the "model" line above the divider).',
      });
      y += rowH;
      gateRows.push({
        y, label: `Meta ≥ ${ENTRY_META}`, val: na, pass: null,
        tt: 'Second-opinion XGBoost that grades the primary signal. Blank because no prediction was logged for this bar.',
      });
    }
    y += rowH + 3;

    const riskLabel = g.halted ? 'HALTED' : g.dayLimit ? 'DAY LIMIT' : 'ok';
    gateRows.push({
      y, label: 'Risk / kill switch', val: riskLabel, pass: g.riskPass,
      tt: 'Two circuit breakers. Soft: -2% intraday loss pauses trading until 00:00 UTC. Hard: -10% drawdown from equity peak triggers HALT that only clears via CLI `python tradebot.py reset-halt`.',
    });
    y += rowH + 5;

    // decision area — wrap the "→ HOLD/HALT/ENTER: <reason>" text to the
    // card width so long reasons flow onto multiple lines instead of
    // running off the right edge.
    const dividerY = y;
    const decY = y + 12;
    const decLineH = 14;
    const decLines = wrapText(`→ ${g.decision}: ${g.why}`, cardW - 24);
    // grow the card if the decision took more than one line
    const cardH = Math.max(224, decY + decLines.length * decLineH + 10 - cardY);
    this.gateCardH = cardH;

    // one combined hotspot for the whole decision block
    hs(decY + (decLines.length - 1) * decLineH / 2, decLines.length * decLineH,
      'The first failing gate above, spelled out. To reach ENTER, every gate must be green. HALT = risk breaker fired; HOLD = a trading gate blocked.');

    for(const gr of gateRows) hs(gr.y, rowH, gr.tt);

    // find hovered hotspot (mouse must be inside the card box)
    let hovered = null;
    if(mouse.x >= cardX + 4 && mouse.x < cardX + cardW - 4) {
      for(const h of hotspots) {
        if(mouse.y >= h.y - h.h / 2 && mouse.y < h.y + h.h / 2) {
          hovered = h;
          break;
        }
      }
    }

    // ── Now draw: card background, hover highlight, header, divider, rows,
    //    decision — in that z-order — then the floating tooltip on top.
    vg.BeginPath();
    vg.RoundedRect(cardX, cardY, cardW, cardH, 6);
    vg.FillColor(COL.cardBg);
    vg.Fill();
    vg.StrokeColor(COL.cardEdge);
    vg.StrokeWidth(1);
    vg.Stroke();

    if(hovered) {
      vg.BeginPath();
      vg.Rect(cardX + 4, hovered.y - hovered.h / 2, cardW - 8, hovered.h);
      vg.FillColor(COL.rowHover);
      vg.Fill();
    }

    app.font(12, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(cardX + 12, cardY + 14, `GATE STATUS — ${fmtTs(c.ts)}`);

    app.font(10, COL.dim, ALIGN_LEFT | ALIGN_MIDDLE);
    app.text(cardX + 12, cardY + 30, `model: ${g.modelVer}   (${g.totalPreds} preds logged)`);
    app.text(cardX + 12, cardY + 44, `bars: ${store.candles.length} (min ${MIN_BARS_FOR_PRED})   equity: ${fmtNum(g.equity)}`);

    const hr = hy => {
      vg.BeginPath();
      vg.MoveTo(cardX + 8, hy);
      vg.LineTo(cardX + cardW - 8, hy);
      vg.StrokeColor(COL.grid);
      vg.StrokeWidth(1);
      vg.Stroke();
    };
    hr(cardY + 54);

    for(const gr of gateRows) {
      const dotCol = gr.pass === null || gr.pass === undefined ? COL.gateSkip : gr.pass ? COL.gateOk : COL.gateFail;
      vg.BeginPath();
      vg.Circle(cardX + 14, gr.y, 4);
      vg.FillColor(dotCol);
      vg.Fill();
      app.font(11, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
      app.text(cardX + 26, gr.y, gr.label);
      if(gr.val !== null && gr.val !== undefined) {
        const valCol = gr.pass === false ? COL.gateFail : COL.text;
        app.font(11, valCol, ALIGN_RIGHT | ALIGN_MIDDLE);
        app.text(cardX + cardW - 12, gr.y, gr.val);
      }
    }

    hr(dividerY);
    const decCol = g.decision === 'ENTER' ? COL.gateOk : g.decision === 'HALT' ? COL.gateFail : COL.text;
    app.font(12, decCol, ALIGN_LEFT | ALIGN_MIDDLE);
    for(let i = 0; i < decLines.length; i++) {
      app.text(cardX + 12, decY + i * decLineH, decLines[i]);
    }

    if(hovered) this.drawGateTooltip(app, hovered.tt, cardX, cardY, cardW, cardH);
  }

  drawGateTooltip(app, text, cardX, cardY, cardW, cardH) {
    const { vg, mouse } = app;
    const r = this.rect;
    const ttW = 300;
    const lines = wrapText(text, ttW);
    const lineH = 15;
    const ttH = 14 + lines.length * lineH;

    // prefer right of the card; fall back to below if we'd run off-pane
    let ttX = cardX + cardW + 8;
    let ttY = clamp(mouse.y - ttH / 2, r.y + 8, r.y + r.h - ttH - 8);
    if(ttX + ttW > r.x + r.w - 8) {
      ttX = cardX;
      ttY = cardY + cardH + 8;
      if(ttY + ttH > r.y + r.h - 8) ttY = Math.max(r.y + 8, cardY - ttH - 8);
    }

    vg.BeginPath();
    vg.RoundedRect(ttX, ttY, ttW, ttH, 5);
    vg.FillColor(COL.boxBg);
    vg.Fill();
    vg.StrokeColor(COL.cardEdge);
    vg.StrokeWidth(1);
    vg.Stroke();

    app.font(11, COL.text, ALIGN_LEFT | ALIGN_MIDDLE);
    for(let k = 0; k < lines.length; k++) app.text(ttX + 10, ttY + 12 + k * lineH, lines[k]);
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

    // Gate-card hover takes precedence — its own tooltip is drawn later, and
    // the crosshair line/box across the card would just be visual noise.
    const gc = this._gateCardRect(app);
    if(mouse.x >= gc.x && mouse.x < gc.x + gc.w && mouse.y >= gc.y && mouse.y < gc.y + gc.h) return;

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
//
// Why plot ratios instead of raw values? ADX lives in [0..100] and ATR%
// lives in roughly [0..0.05]. Sharing one y-axis would squash one of them
// into a flat line at the bottom. Dividing each by its own pass threshold
// puts them on the same "1.0 = just passing" scale, so both are readable
// on one chart and the horizontal line at y = 1 is a literal answer to
// "does this bar clear the regime filter?".
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

    // y-axis maps 0..3 → bottom..top; anything larger is clamped so a
    // huge ADX spike doesn't rescale the whole pane.
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

    // resize handles: thicker vertical bars at each edge, with a small
    // notch in the middle, so the drag zones are visually obvious.
    vg.StrokeColor(COL.viewEdge);
    vg.StrokeWidth(3);
    const midY = r.y + r.h / 2;
    for(const hx of [vx0, vx1]) {
      vg.BeginPath();
      vg.MoveTo(hx, r.y + 4);
      vg.LineTo(hx, r.y + r.h - 4);
      vg.Stroke();
      vg.BeginPath();
      vg.MoveTo(hx - 3, midY);
      vg.LineTo(hx + 3, midY);
      vg.StrokeWidth(1);
      vg.Stroke();
      vg.StrokeWidth(3);
    }

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

  // Convert a minimap x-pixel to a candle index in [0..n-1].
  _idxOfX(app, mx) {
    const n = app.store.candles.length;
    const frac = clamp((mx - this.rect.x) / this.rect.w, 0, 1);
    return frac * Math.max(1, n - 1);
  }

  // hitTest — classify a click on the minimap into one of four zones:
  //   'leftEdge'  — within EDGE px of the viewport rect's left edge
  //   'rightEdge' — within EDGE px of the viewport rect's right edge
  //   'inside'    — inside the viewport rect (not on an edge)
  //   'outside'   — anywhere else on the minimap
  // 'leftEdge'/'rightEdge' let the user resize the range (change zoom).
  // 'inside'/'outside' pan the range (recenter).
  hitTest(app, mx) {
    const { store, view } = app;
    const n = store.candles.length;
    if(n < 2) return 'outside';
    const xOf = i => this.rect.x + (i / (n - 1)) * this.rect.w;
    const vx0 = xOf(clamp(view.i0, 0, n - 1));
    const vx1 = xOf(clamp(view.i1, 0, n - 1));
    const EDGE = 8;
    if(Math.abs(mx - vx0) < EDGE) return 'leftEdge';
    if(Math.abs(mx - vx1) < EDGE) return 'rightEdge';
    if(mx > vx0 && mx < vx1) return 'inside';
    return 'outside';
  }

  // applyDrag — the drag body. Dispatches on the mode captured at press.
  //   left/right edge → move only that edge (change span → zoom)
  //   inside/outside  → recenter viewport on the mouse (pan, span kept)
  applyDrag(app, mx, mode) {
    const { view } = app;
    const idx = this._idxOfX(app, mx);
    if(mode === 'leftEdge') {
      view.i0 = Math.min(idx, view.i1 - 10);
    } else if(mode === 'rightEdge') {
      view.i1 = Math.max(idx, view.i0 + 10);
    } else {
      const span = view.span;
      view.i0 = idx - span / 2;
      view.i1 = idx + span / 2;
    }
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
    this.dragPane = null; // 'chart' | 'minimap' | 'gateCard'
    this.dragX = 0;
    this.minimapMode = 'inside'; // 'leftEdge' | 'rightEdge' | 'inside' | 'outside'
    this.gateCardOffset = { dx: 0, dy: 0 };
    this.dragStart = null; // snapshot at press-time for delta-based dragging
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
          app.minimap.applyDrag(app, x, app.minimapMode);
        } else if(app.dragPane === 'gateCard') {
          // Move the card by the mouse delta since press. Clamp so the
          // card's header (title bar) always stays inside the candle pane —
          // otherwise you could throw it off-screen and never grab it back.
          const ds = app.dragStart;
          const cpr = app.candlePane.rect;
          const w = 306;
          const headerH = 54;
          const dxRaw = ds.dx + (x - ds.x);
          const dyRaw = ds.dy + (y - ds.y);
          app.gateCardOffset.dx = clamp(dxRaw, -12, cpr.w - w - 12);
          app.gateCardOffset.dy = clamp(dyRaw, -12, cpr.h - headerH - 12);
        }
        app.mouse = { x, y };
      },
      handleMouseButton(button, action) {
        if(button !== MB_LEFT) return;
        if(action !== PRESS) {
          app.dragPane = null;
          app.dragStart = null;
          return;
        }
        // Gate card wins over chart/pane hit-tests: header area = drag
        // handle, body below is left alone so hover tooltips work.
        if(app.candlePane.contains(app.mouse.x, app.mouse.y)) {
          const gc = app.candlePane._gateCardRect(app);
          const inCard = app.mouse.x >= gc.x && app.mouse.x < gc.x + gc.w &&
                         app.mouse.y >= gc.y && app.mouse.y < gc.y + gc.h;
          if(inCard && app.mouse.y < gc.y + 54) {
            app.dragPane = 'gateCard';
            app.dragStart = { x: app.mouse.x, y: app.mouse.y,
                              dx: app.gateCardOffset.dx, dy: app.gateCardOffset.dy };
            return;
          }
          if(inCard) return; // click on card body → do nothing; keeps hover live
        }
        if(app.minimap.contains(app.mouse.x, app.mouse.y)) {
          app.minimapMode = app.minimap.hitTest(app, app.mouse.x);
          app.dragPane = 'minimap';
          app.minimap.applyDrag(app, app.mouse.x, app.minimapMode);
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
          case KEY_HOME: {
            // Preserve the current zoom; anchor the left edge at bar 0.
            // (Older code called view.fit(200, 0) which collapsed the
            // viewport to a 2-bar span — replay then dragged bars in one
            // at a time because auto-scroll preserves span across frames.)
            const span = Math.max(Math.round(view.span), 10);
            replay.enterReplay();
            replay.cursor = 0;
            view.i0 = 0;
            view.i1 = span;
            break;
          }
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
