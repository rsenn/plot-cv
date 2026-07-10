# tradeview.js — Complete Documentation

Written for a JS expert who has never traded. Trading concepts are explained
from zero; the rendering + I/O parts are described in terms of things a JS
dev already knows (Canvas2D, DOM events, `Float64Array`, IIR filters if
you've written a synth, etc). Read top to bottom once, then use it as a
reference while hacking.

The companion piece is `tradebot - Complete Documentation`; this doc
assumes you have that available but doesn't require you to have read it.

---

## Table of contents

1. What this viewer actually does (30-second version)
2. Trading concepts you'll see on screen (from zero)
3. Design philosophy
4. The DB schema — your read-only viewer contract
5. JS runtime + rendering stack (`qjsm`, glfw, nanovg)
6. Class-by-class walkthrough
7. What each pane shows and how to read it
8. The Gate Status card — why isn't it trading?
9. Indicator math, verbatim (so you can port it)
10. Tuning knobs & how to modify things by hand
11. Keys, mouse, and other operations
12. Roadmap for your own improvements

---

## 1. What this viewer actually does

`tradeview.js` is a read-only GUI for the SQLite database `tradebot.py`
writes. Every hour (a "bar"), the bot logs a candle, its model's opinion,
whatever it decided to do, and where the equity curve stands. This viewer
opens that database and paints all of it — plus a lot of derived stuff the
bot doesn't log — into a single window.

Concretely, one frame:

1. Polls `tradebot.db` (WAL mode, so we read while the bot writes).
2. Recomputes indicators (SMA, Bollinger, ATR, RSI, MACD, VWAP, ADX) from
   the candle array if new bars arrived.
3. Draws candles, moving averages, Bollinger bands, VWAP, trade brackets,
   entry/exit markers, and a card in the top-left that spells out
   **exactly which gate the bot is stuck on right now**.
4. Draws a signal pane (`p_long`, `p_short`, `meta_p`), a regime pane
   (ADX and ATR% relative to their pass thresholds), an equity curve with
   drawdown shading, a minimap, and a status bar.
5. Handles keyboard/mouse input for pan, zoom, replay stepping, live-vs-
   replay mode switching.

You can drive it in **live** mode (it follows the right edge as new bars
arrive) or **replay** mode (step bar-by-bar or auto-play at 1× to 256×).

Two things it does that the Python bot cannot:

- Compute every deterministic (non-ML) part of `tradebot.py`
  **on every historical bar**, not just the ones the bot happened to
  process while running. If you delete `tradebot.db` and re-`backfill`,
  the viewer will show you the ADX/ATR/regime state for the entire year
  even though the bot never made a decision on those bars.
- Show *why* the bot didn't trade. If it's stuck on "not enough bars",
  "model never ran", "regime off", "meta prob too low", etc., that
  answer is one glance at the Gate Status card — no grepping SQLite.

---

## 2. Trading concepts you'll see on screen

Enough to make the source readable. Full versions live in
`tradebot - Complete Documentation.md`.

**Candle (OHLCV).** One time slice of the market as a struct:

```js
{ ts, open, high, low, close, volume }
```

For a 1-hour candle: `open` = price when the hour started, `close` =
price when it ended, `high`/`low` = extremes during the hour, `volume` =
how much of the coin actually traded. Every pane's x-axis is indexed by
bar, not by wall-clock time.

**Candlestick chart.** The graphical representation. Anatomy:

```
─┬─  ← high
 │
┌┴┐  ← body top    = max(open, close)
│ │  ← body        = filled green if close ≥ open (a "bull" candle), red else
└┬┘  ← body bottom = min(open, close)
 │
─┴─  ← low
```

A tall body means the price moved a lot; long wicks mean the price got
volatile inside the hour but came back.

**Long / Short.** *Long* = you bought and win if price goes up. *Short* =
you sold something you didn't own (borrowed it) and win if price falls.
On a spot exchange like Coinbase you can't really short, so the bot only
shorts in paper mode for research.

**Stop-loss / Take-profit.** Exit prices decided **at entry time**.
Stop-loss = "if it falls to X, sell immediately, accept the small loss".
Take-profit = "if it rises to Y, sell, lock the win". Both get placed on
every trade. Positions without stops are how accounts die.

**ATR (Average True Range).** The average size of a recent candle — a
volatility yardstick. "Stop at 2× ATR" = "give the trade room proportional
to how wild the market currently is". A fixed stop like "-1%" would be
suicide in a wild market and too loose in a calm one. `atrRel = ATR/price`
puts it on a scale-invariant footing (e.g. 0.008 = "candles are typically
±0.8% tall").

**Fees & slippage.** Coinbase takes ~0.6% of every trade. Slippage = the
gap between the price you saw and the price you actually got. Together
they're the house edge. If your average per-trade win is smaller than
fees + slippage, you lose even with a great model.

**Regime.** Markets alternate between *trending* (moves directionally,
strategies can work) and *chopping* (random noise around a level, fees
kill every strategy). The bot's `RegimeFilter` refuses to trade in chop.
The viewer replicates this filter and shades every chop bar so you can
see where the bot was on strike.

**ADX (0..100).** A standard trend-strength indicator. Above 20 usually
means "there's a trend"; below 20 means "sideways chop". Doesn't tell you
direction — just how one-sided recent moves have been.

**SMA / EMA (moving averages).** Slow trend lines. `SMA(20)` = mean of
the last 20 closes. `EMA(N)` = the exponential version (recursive IIR
smoother). Price above its SMA is a common "uptrend" heuristic.

**Bollinger Bands.** `SMA(20) ± 2 · rolling_std(close, 20)`. An envelope
containing ~95% of closes. When the band is narrow, volatility is
compressed and often about to release. Touches of the outer band mean
"price is unusually far from average".

**VWAP.** Volume-weighted average price. Instead of averaging price,
weight each candle's close by how much traded that hour. Institutions
grade their fills against VWAP ("did I get a good price?"). Close above
VWAP = current price is above the volume-weighted centre of the last day.

**RSI (0..100).** Momentum oscillator. <30 = "recent drops look
overdone" (oversold), >70 = "recent rallies look overdone" (overbought).
A tension gauge, not a signal.

**MACD histogram.** `EMA(12) − EMA(26)`, minus its own 9-bar EMA. Signed
number that flips at trend changes. Hist crossing zero from below = fast
average just overtook slow → often (not always) a trend turn.

**Drawdown.** How far your account has fallen from its all-time peak,
as a percentage. If you had €10 000 and now have €9 200, drawdown = 8%.
The bot has a kill switch at 10%.

**Equity.** Your account value. In paper mode it's simulated starting
from €10 000. The equity pane plots it over time.

**Regime OK.** `adx > 20 AND atr_rel > 2.5 × fee_rate`. That is: the
market must trend enough (ADX) *and* candles must be big enough for a
correct call to actually cover fees (ATR%). If either fails, the bot
just sits.

**Gate.** The ordered pipeline of yes/no checks the bot runs before
opening a trade. The Gate Status card in the top-left of the candle pane
enumerates them; the first `FAIL` is the one blocking the trade.

**Prediction (`p_long`, `p_short`, `p_flat`, `meta_p`).** The bot's ML
stack produces three probabilities that sum to 1: chance of a decent
up-move, no-move, or down-move over the next 12 bars. `meta_p` is a
second-opinion model that grades whether the primary signal itself
should be trusted. Both are logged into the `predictions` table.

---

## 3. Design philosophy

**Read-only, always.** The viewer opens the SQLite file with
`SQLite3.OPEN_READONLY`. It cannot corrupt the bot's state. If you crash
mid-frame, nothing changes on disk.

**One source of truth: `tradebot.db`.** Nothing is inferred from the
filesystem, no state files, no config. Everything the viewer knows comes
from the six tables the bot writes (see §4). This lets you run the viewer
against any database — the bot's live one, a backup, a totally offline
DB you shipped in for debugging.

**Deterministic replicas, not re-derivations.** The `Indicators` class
computes the same numbers `tradebot.FeatureEngine` computes. Same
formulas, same window sizes, same smoothing constants. That gives you a
free correctness cross-check: if the viewer's `regimeOk[i]` doesn't
agree with the DB's `predictions.regime_ok` for that timestamp, one of
the two implementations has drifted.

**Two clocks.** LIVE mode polls the DB every 2 s and follows the right
edge. REPLAY mode has a cursor and a bars-per-second knob; time is
frozen at that cursor and the panes only see bars `[0..cursor]`.

**Immediate mode rendering.** Nothing is retained. Every frame, every
pane iterates the visible slice and issues nanovg calls. No damage
tracking, no dirty rects. At 60 FPS with ~500 visible candles, this is
comfortably below the frame budget on a laptop.

**No hidden state.** Every pane derives its output from `store` +
`indicators` + `replay.cursor`. If you screenshot the app and later want
to know why a specific gate failed, you can — the answer is in the bar
timestamp and the DB.

**Fail loud on bad data.** Anything that isn't ready yet is shown as
"warming up..." with `dim` text. NaN indicators are simply skipped in the
line paths (the code checks `isFinite`). Never render a wrong number
silently.

**Match the bot's constants exactly.** Every knob at the top of
`tradeview.js` mirrors a field in `tradebot.Config` with the same name
and semantics. If you edit one place, edit both (or accept that the
regime shading in the viewer will diverge from the bot's actual gate).

---

## 4. The DB schema — your read-only viewer contract

Owned by `tradebot.py`. All `ts` are unix seconds, UTC. This is exactly
the schema the bot writes; the viewer only reads.

```sql
candles     (ts, product, open, high, low, close, volume)      PK(product,ts)
predictions (ts, product, p_long, p_flat, p_short,
             meta_p, regime_ok, model_ver)                     PK(product,ts)
actions     (ts, product, action, reason, price, size, confidence)
                              -- action ∈ BUY/SELL/HOLD/HALT/STOP/TP/SKIP
trades      (id, product, side, ts_open, entry, size, stop, tp,
             ts_close, exit, pnl, fees, status)   -- status ∈ OPEN/CLOSED
equity      (ts, equity, drawdown, day_pnl)
events      (ts, level, msg)                      -- INFO/WARN/ERROR
state       (key, value)                          -- JSON KV blob
```

Which pane/thing reads what:

- **`candles`** → `CandlePane` (candles + wicks + body), `Indicators`
  (everything derived), `Minimap` (whole-history preview line).
- **`predictions`** → `SignalPane` (three lines), Gate Status card
  (dir/meta gate), crosshair tooltip.
- **`actions`** → crosshair tooltip (`ACTION reason` lines), status bar
  (last action + reason). Also drives `haltTs` for equity pane shading.
- **`trades`** → `CandlePane.drawTrades` (entry marker + stop/entry/TP
  dashed lines + circle at exit coloured by outcome).
- **`equity`** → `EquityPane` (curve, drawdown fill, HALT columns).
- **`state`** → Gate Status card (halted, equity, day\_start\_eq, model
  version, bars\_since\_train). Anything the bot has to persist across
  restarts lives here as JSON.

`state` is a KV blob: `Store` reads it once per frame and parses the
values as JSON. Keys the viewer actually consults:
`halted`, `equity`, `equity_peak`, `day`, `day_start_eq`,
`bars_since_train`, `model_version`.

---

## 5. JS runtime + rendering stack

You run this file with `qjsm` — QuickJS with a handful of native modules
compiled in. Nothing you'll ever see on npm.

**`qjsm`** = QuickJS + module loader + the native bindings we use:

- **`sqlite`** — a thin JS wrapper around SQLite3. We only use
  `new SQLite3(path, OPEN_READONLY)`, `.query(sql)`, and the returned
  result's `.fetchAssoc()` (row → object).
- **`glfw`** — windowing/input. Familiar Linux/GL loop:
  `Window.hint(...)` for pixel format hints, `new Window(w, h, title)`,
  then a main loop that calls `poll()` and `window.swapBuffers()`. Input
  handlers are set by assigning methods (`handleCursorPos`,
  `handleMouseButton`, `handleScroll`, `handleKey`) directly onto the
  window object.
- **`nanovg`** — vector-graphics API on top of OpenGL. Think Canvas2D
  with the same immediate-mode feel: `BeginPath` → `MoveTo`/`LineTo`/
  `Rect`/`Circle`/`RoundedRect` → `Stroke`/`Fill`. Colors via `RGB(r,g,b)`
  or `RGBA(r,g,b,a)` (all 0..255). Text via `FontFace`, `FontSize`,
  `TextAlign`, `FillColor`, `Text(x, y, str)`. `Save`/`Restore` push
  and pop the graphics state including clip rects.
  `IntersectScissor(x, y, w, h)` clips subsequent draws to a rectangle.

**Coordinate system.** nanovg's origin is top-left, y goes down. `App`
tracks a `width` and `height`; each pane is given its own `rect =
{x, y, w, h}` in that space by `App.layout()`.

**Frame loop** (`App.run`):

```js
while (running) {
  vg.BeginFrame(width, height, devicePixelRatio);
  frame(dt);            // poll DB, refresh indicators, draw all panes
  vg.EndFrame();
  window.swapBuffers();
  poll();               // process pending glfw events → handlers fire
}
```

Every draw call between `BeginFrame`/`EndFrame` gets batched and issued
to the GPU on `EndFrame`. Frame budget on a laptop with ~500 candles
visible is well under 2 ms — you can drop way more expensive things in.

---

## 6. Class-by-class walkthrough

The file is deliberately class-per-concern. Nothing is very deep.

**`Db`** — sqlite wrapper. Two methods: `rows(sql)` returns an array of
associative-array rows; `one(sql)` returns the first row or null. Never
uses parameters (bot IDs and product names are inlined) because the file
only reads and the input is our own product string. Opened read-only.

**`Store`** — owns everything loaded from the DB:

- `candles` — sorted ascending array of candle objects
- `idxByTs` — `Map<ts, index>` for binary-lookup by timestamp
- `predByTs` — `Map<ts, predictionRow>`
- `actsByTs` — `Map<ts, actionRow[]>` (multiple actions per bar allowed)
- `haltTs` — `Set<ts>` of bars where an action was `HALT`
- `trades` — refetched wholesale each poll because trades can UPDATE
  (`OPEN` → `CLOSED`); the table is small enough to reload cheaply.
- `equity` / `eqByTs` — both an array and a per-timestamp map
- `state` — parsed KV blob (see §4)

`refresh(afterTs)` walks each table with `ts > afterTs` and appends. On
first construction it loads everything. `poll(now)` throttles to
`POLL_MS = 2000` ms and re-reads the last stored candle too (the live
forming bar mutates until it closes).

**`Viewport`** — the 2D transform. Two independent axes:

- **X (bar index).** `i0`, `i1` = visible bar range (floats — can be
  fractional between bars during zoom animations). `barW = (paneW − 74) /
  (i1 − i0)`; the 74 px is the right-side price axis. `xOfIndex(i, r)`
  and `indexOfX(x, r)` convert. `zoom(factor, centerIdx)` scales `i0`
  and `i1` symmetrically around the bar under the cursor. `pan(dIdx)`
  shifts both.
- **Y (price).** `pMin`, `pMax` = visible price range, **auto-fitted
  every frame** to the visible candles' highs/lows plus 8% padding. This
  is why zoom and pan feel effortless: you never have to think about the
  price axis. `yOfPrice(p, r)` and `priceOfY(y, r)` convert.

**`Replay`** — LIVE / REPLAY mode + cursor + play/pause + speed. `SPEEDS
= [1, 2, 4, 8, 16, 32, 64, 128, 256]` bars/second; up/down arrows step
through it. `advance(dt)` accumulates time × speed into `.acc` and moves
the cursor by whole bars.

**`Indicators`** — the deterministic math. §9 has the formulas verbatim
and cross-references to `tradebot.FeatureEngine`. Recomputes if
`candles.length` changes; otherwise the cached arrays stand.

**`Pane`** — base class with just `layout(x, y, w, h)` (sets `this.rect`)
and `contains(mx, my)`. Every visual thing on screen is a `Pane`
subclass.

**`CandlePane`** — the big one. Draws, in order:

1. `IntersectScissor` to clip to the pane rect
2. Regime-off column shading (using the **replicated** `regimeOk`)
3. Horizontal price gridlines + right-axis labels
4. Candles (wick + body per bar; body outlined if the last bar is still
   forming)
5. Overlay indicator lines (SMA, BB upper/lower, VWAP) via
   `drawOverlays`
6. Trade brackets + entry/exit markers via `drawTrades`
7. Hypothetical stop/TP dashed lines at the right edge via
   `drawHypotheticalBrackets`
8. Crosshair + tooltip if mouse is over the pane (`drawCrosshair`)
9. `Restore`, then unclipped: current price tag, bottom legend, and
   the Gate Status card

**`SignalPane`** — three probability lines (`p_long`, `p_short`,
`meta_p`) in [0..1], plus dashed threshold lines at `ENTRY_DIR = 0.45`
and `ENTRY_META = 0.58`.

**`RegimePane`** — see §9 for the ratio trick. Plots ADX/20 and
ATR%/1.5% on a shared y-axis where 1.0 is the pass line.

**`EquityPane`** — equity curve, drawdown fill (area between running
peak and current equity), HALT columns (bars where the bot logged a
HALT action).

**`Minimap`** — coarse close-price line over the entire history with a
draggable rectangle showing what the CandlePane is currently zoomed to,
plus a vertical replay cursor when in replay mode.

**`StatusBar`** — mode indicator, product, current price, model version,
equity + drawdown, last action + reason. Everything you want to glance
at while working with something else.

**`App`** — window init, layout, input binding, main loop. `bindInput`
attaches `handleCursorPos`, `handleMouseButton`, `handleScroll`,
`handleKey` directly onto the glfw `Window`. Keyboard focus is implicit
(you clicked on the window, you type at it).

---

## 7. What each pane shows and how to read it

**CandlePane.** Look for:

- Candles going in the direction of the SMA line = you're inside a
  trend.
- Bollinger squeeze (bands narrow) = volatility compression, often a
  regime change soon after.
- Close breaking above/below VWAP = intraday buyers/sellers just took
  control.
- Grey regime-off shading = the bot literally cannot enter here even if
  its models are begging to. If you see a nice-looking move happen
  during grey shading, the bot missed it *by design* — its risk model
  says it wasn't a paying opportunity, only an obvious one in hindsight.
- Hypothetical `hyp TP / hyp SL` dashed lines at the right edge show
  exactly where a trade opened right now would exit. Compare to your
  eyeball estimate of what "reasonable" is.

**SignalPane.** Three lines:

- **`p_long`** (top green-ish) — model's probability of a decent up-move
  in the next 12 bars.
- **`p_short`** (bottom red-ish) — same but for down-moves.
- **`meta_p`** (middle ochre) — the second-opinion model's probability
  that the primary signal is right.

Only when the direction line pokes above 0.45 *and* meta pokes above
0.58 does the bot even *consider* entering. Very often meta stays flat
and low — that's the meta model saying "yeah the primary is guessing".

**RegimePane.** Two lines, both plotted as ratio-to-threshold so 1.0 =
pass:

- **Navy** = ADX/20. Above 1.0 → trending enough. Peaks of ~2 (i.e. ADX
  ≈ 40) are strong trends.
- **Ochre** = ATR%/1.5%. Above 1.0 → candles big enough to pay fees.

Both must be above 1.0 for the bot to trade. Grey shading on this pane
mirrors CandlePane's shading — one glance shows how much of history was
"tradeable".

**EquityPane.** In paper mode you're staring at a simulated €10 000
account. The line is what you'd have. The pink drawdown fill is the gap
between your running peak and current — the bot halts if it ever
touches 10%. Yellow HALT columns mark bars where a hard stop or day
limit fired.

**Minimap.** The whole story on one strip. The lavender rectangle is
"what the CandlePane is showing right now" — drag it to jump. The gold
vertical line only appears in replay mode; it's the cursor.

**StatusBar.** Left: `LIVE | BTC-EUR | 2026-01-04 15:00 | 62 421.50 |
model 20260101-0400 | eq 10 042.31 (dd 0.1%) | HOLD: gate: dir=0.421
meta=0.512 regime=true`. Right: if halted, a bold "HALTED — kill switch"
label.

---

## 8. The Gate Status card

This is the killer feature for figuring out "why isn't it trading?".

The card lives in the top-left of the candle pane. It's regenerated
every frame from `evalGate(store, indicators, replay.maxIdx)`. Structure:

```
GATE STATUS — 2026-01-04 15:00
model: 20260101-0400   (247 preds logged)
bars: 8741 (min 124)   equity: 10042.31
────────────────────────────────────────
● Regime filter                       PASS
●   ADX > 20                          22.4
●   ATR% > 1.50%                     1.83%

● Direction ≥ 0.45                   0.612
● Meta ≥ 0.58                        0.634

● Risk / kill switch                    ok
────────────────────────────────────────
→ ENTER: LONG cleared all gates
```

The three colored dots:

- **Green** — this gate passed.
- **Red** — this gate failed. This is what's blocking the trade.
- **Sepia (skip)** — inapplicable because a prior gate blocked (e.g.
  meta/dir are skipped when there's no prediction).

The final arrow line is the actual decision the bot would take and the
first reason found in the pipeline. See §9 of the source (`evalGate`) for
the exact short-circuit order.

**Failure modes you'll actually see:**

- *"model never ran — no predictions logged in DB"* — you ran
  `tradebot backfill` but not `tradebot train`. Fix: `python tradebot.py
  train`.
- *"only N bars — need ≥ 124"* — the LSTM needs 64 bars for its
  lookback + 60 more for feature warmup. Do more backfill.
- *"regime off — chop / low volatility, fees would eat edge"* — the
  most common gate to fail. Not a bug. Look at the RegimePane; you'll
  probably see both ratios pinned below 1.
- *"direction prob 0.421 < 0.45"* — the model is guessing. Waiting for
  a bar with a stronger opinion.
- *"meta prob 0.512 < 0.58"* — the primary looks confident but the
  second-opinion model doesn't trust it. Usually a good save.
- *"HALTED — kill switch"* — you hit -10% drawdown. `python tradebot.py
  reset-halt` clears it.
- *"daily loss limit hit"* — you're already -2% today; the bot pauses
  until 00:00 UTC.

In replay mode, stepping (left/right) recomputes the card for the bar
under the cursor. That's the most productive way to build intuition:
step through a stretch of history and watch which gate blinks red for
each bar.

---

## 9. Indicator math, verbatim

All formulas here mirror `tradebot.FeatureEngine` exactly. If you port
this to another language, port from these definitions and the ported
result will match both the Python bot and this viewer bit-for-bit.

**One-pole IIR (`ewm`).** The recurrence:

```
y[0] = x[0]
y[i] = α · x[i] + (1 − α) · y[i-1]
```

This is `pandas.Series.ewm(alpha, adjust=False).mean()`. It's also
exactly the recursive form of an RC low-pass filter. Two conventions:

- **Wilder smoothing:** `α = 1/N`. That's what ATR, RSI, ADX use.
- **EMA(span=N):** `α = 2 / (N + 1)`. That's what MACD uses (span 12,
  26, 9).

**SMA(N) + Bollinger.**

```
sma[i]    = mean of close[i-N+1 .. i]           (rollingMean, N=20)
std[i]    = sample std of same window           (rollingStd, ddof=1)
bbUp[i]   = sma[i] + 2 · std[i]
bbLow[i]  = sma[i] − 2 · std[i]
```

Sample std (ddof=1, i.e. divide by `N−1`) matches pandas' default and
the standard Bollinger definition.

**True range → ATR (Wilder 14).**

```
tr[i]  = max(high[i] − low[i],
             |high[i] − close[i-1]|,
             |low[i]  − close[i-1]|)
atr[i] = ewm(tr, α = 1/14)
atrRel = atr / close
```

The `|·− close[i-1]|` terms exist so gaps between yesterday's close and
today's open are counted correctly.

**RSI (Wilder 14).**

```
Δ[i] = close[i] − close[i-1]
up[i] = max(Δ[i], 0)
dn[i] = max(−Δ[i], 0)
avgUp = ewm(up, 1/14)
avgDn = ewm(dn, 1/14)
rs   = avgUp / avgDn
rsi  = 100 − 100 / (1 + rs)              ∈ [0..100]
```

**MACD histogram (12, 26, 9).**

```
ema12  = ewm(close, 2/13)
ema26  = ewm(close, 2/27)
macd   = ema12 − ema26
sig    = ewm(macd, 2/10)
hist   = macd − sig
```

The stored value is `hist / close` (scale-invariant, better feature).

**VWAP over 24 bars.**

```
vwap[i] = Σ(close·vol) over last 24 bars
       / Σ(vol)        over last 24 bars
```

`(close / vwap) − 1` = distance above (positive) or below (negative)
the 24-bar volume-weighted average.

**Volume z-score (48).**

```
volMean = rollingMean(vol, 48)
volStd  = rollingStd (vol, 48)
volZ    = (vol − volMean) / volStd
```

**ADX (Wilder 14).**

```
+DM[i] = high[i] − high[i-1]                 (if > down-move AND > 0, else 0)
−DM[i] = low[i-1] − low[i]                   (mirror)
+DI    = 100 · ewm(+DM, 1/14) / atr
−DI    = 100 · ewm(−DM, 1/14) / atr
dx     = 100 · |+DI − −DI| / (+DI + −DI)
adx    = ewm(dx, 1/14)                       ∈ [0..100]
```

**RegimeFilter.ok (per bar).**

```
regimeOk[i] = (adx[i] > 20) AND (atrRel[i] > 2.5 · fee_rate)
            = (adx[i] > 20) AND (atrRel[i] > 0.015)
```

**RiskManager position size.** ATR-scaled, capped at 25% of equity:

```
riskEur = target_vol · equity                (= 1% · equity)
sizeEur = min(riskEur / (stop_atr · atrRel),
              max_position_frac · equity)
size    = sizeEur / price                    (base coin units, e.g. BTC)
```

**The RegimePane ratio trick.** ADX lives in [0..100] and ATR% lives in
[0..0.05]. Sharing one y-axis would squash one into a flat line.
Dividing each by its own pass threshold puts them both on a
"1.0 = just passing" scale, and the horizontal `y=1` line becomes a
literal answer to "does this bar clear the regime filter?".

---

## 10. Tuning knobs & how to modify

Every knob is a `const` at the top of the file, grouped under `//
── tradebot.Config mirror ──`. **Keep these in sync with tradebot.Config**
or the viewer's replicated regime shading will not match what the bot
actually did.

- `FEE_RATE` — if you move to a different venue with a different taker
  fee, change it here *and* in the Python `Config`. It cascades into
  `REGIME_ATR_MIN`.
- `ENTRY_DIR`, `ENTRY_META` — lowering these makes the bot trade more
  aggressively (in the viewer's Gate Status card too). Watch what happens
  to your equity curve when you back-test.
- `REGIME_ADX_MIN` — lowering to 15 opens far more bars for trading but
  usually with worse win-rate. Try it in replay: step through history
  with 15 vs 25 and count how many false-positive bars you get.
- `REGIME_ATR_MIN` — this defaults to `2.5 × fee_rate`. Set to
  `2 × fee_rate` to unlock more bars but expect fees to eat more edge.
- `STOP_ATR`, `TP_ATR` — the reward:risk ratio (3:2) and the ATR
  multiples. Wider stops = fewer stop-outs but bigger losses when they
  happen.
- `TARGET_VOL`, `MAX_POS_FRAC` — position sizing. `TARGET_VOL = 0.01`
  means "risk 1% of equity per trade". `MAX_POS_FRAC = 0.25` means "no
  single trade can be more than 25% of the account".
- `DAILY_LOSS_LIMIT`, `MAX_DRAWDOWN` — soft/hard circuit breakers.

**Visual knobs** live in `const COL = {...}`. Every colour on the page.
Editing there recolours the whole app instantly. Candle bodies are
`up`, `dn`, `upFill`, `dnFill` — keep those distinguishable if you
change the palette.

**Layout percentages** live in `App.layout()`. Currently
`0.52 / 0.16 / 0.14 / rest` for candle / signal / regime / equity of
the "middle" space. Change these to give the panes more room.

**Adding a new pane.** Subclass `Pane`, add a `draw(app)` method,
instantiate in `App` constructor, add to `App.layout()` (allocate
vertical space + call `.layout(x, y, w, h)`), and call its `.draw(this)`
in `App.frame()` in the desired order. That's ~15 lines total.

**Adding a new indicator.** Add the computation to `Indicators.refresh()`
(the same block-per-indicator pattern) and expose it as an instance
field. Reference it from your pane's `draw()`. `Float64Array` and NaN
skipping are the only conventions.

---

## 11. Keys, mouse, and other operations

**Keys:**

- `space` — play / pause replay
- `←`/`→` — step one bar (enters replay if in live)
- `↑`/`↓` — replay speed up/down
- `home`/`end` — jump to start/end
- `l` — back to live mode (follow right edge)
- `f` — fit last 200 bars
- `pgup`/`pgdn` — zoom in / out
- `q`/`esc` — quit

**Mouse:**

- Drag the chart — pan
- Drag the minimap rectangle — recenter the viewport
- Wheel — zoom around the bar under the cursor

**Running:** `qjsm tradeview.js [tradebot.db]`. Defaults to
`tradebot.db` in the current directory. Read-only — never touches the
file the bot is writing to.

**Live + replay simultaneously:** the bot can be running against the DB
while you REPLAY. The Store just keeps polling; the Replay cursor
doesn't move on its own, so live bars accumulate off the right edge of
your view until you press `l`.

**Multiple viewers on the same DB:** fine. SQLite WAL mode lets many
readers coexist with one writer.

---

## 12. Roadmap for your own improvements

Small, focused ideas — most are afternoon-scale, and each one teaches a
piece of trading + rendering:

- **Hover the Gate Status card to expand a line.** Show the last N bars
  where that gate flipped state (e.g. "last time regime was ON was 47
  bars ago"). Educational for building intuition about how sparse
  regime-on windows really are.
- **"Jump to next PASS bar" hotkey.** From current cursor, scan forward
  and land on the first bar where all gates would pass. Great for
  finding the bot's would-be entries in replay.
- **Backtest overlay.** Since the deterministic side is already ported,
  the last missing piece is: given the DB's stored predictions, replay
  the *entire* pipeline and draw all the ENTER decisions as ghost trades
  on the CandlePane. Compare to the actual `trades` table to see what
  would have been different if a single knob changed.
- **Export computed indicators as CSV.** One button, one file, useful
  for cross-checking against Python-side computations.
- **Trade explorer pane.** List of closed trades with pnl/duration; click
  one to jump the CandlePane to it.
- **RSI/MACD sub-panes.** Same treatment as the RegimePane but for the
  momentum indicators, with the standard 30/70 and 0 threshold lines.
- **Alerts.** In live mode, flash the window title / play a bell when
  the gate flips to ENTER. Useful when the bot is on and you want to
  supervise it.
- **Snapshot mode.** Freeze all data and let the user annotate; export
  a PNG. Nice for sharing "look what the bot did here".
- **Second product overlay.** Load a second candle stream and overlay
  its normalised close on the CandlePane. Watch BTC and ETH drift
  apart.
- **UI for editing gate thresholds live.** Slider for `ENTRY_DIR` /
  `ENTRY_META`; the Gate Status card updates instantly. **Do not** write
  these back to the DB — the bot must own its own config. It's a
  what-if visualiser.

---

## Appendix: source-file map

Line numbers as a rough compass — they'll drift as you edit. Search by
symbol name to find the current location.

| Section | Symbol |
| --- | --- |
| File header + primer | (top, ~50 lines of `//` comments) |
| Config mirror constants | `FEE_RATE`, `ENTRY_DIR`, ..., `START_EQUITY` |
| Colour palette | `const COL = { ... }` |
| Small helpers | `fmtTs`, `fmtNum`, `clamp`, `dashedLine` |
| Math primitives | `ewm`, `rollingMean`, `rollingSum`, `rollingStd` |
| Indicators class | `class Indicators` |
| Gate pipeline | `function evalGate(...)` |
| DB wrapper | `class Db` |
| DB-backed store | `class Store` |
| Coordinate transform | `class Viewport` |
| Replay state machine | `class Replay` |
| Pane base | `class Pane` |
| Candles + overlays + gate card | `class CandlePane` |
| Model probabilities | `class SignalPane` |
| Equity curve + drawdown | `class EquityPane` |
| ADX/ATR ratio pane | `class RegimePane` |
| Minimap | `class Minimap` |
| Status line | `class StatusBar` |
| Window + input + loop | `class App` |
| Entry point | `function main(...)` |
