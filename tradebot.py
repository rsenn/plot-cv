#!/usr/bin/env python3
"""
tradebot.py — restartable ensemble trading bot (Coinbase Advanced Trade)
=========================================================================
Single file, class-per-concern. Everything the bot sees, thinks and does
is persisted to SQLite so an external viewer (qjs-glfw/nanovg) can render
it live or as replay.

    pip install torch numpy pandas coinbase-advanced-py xgboost

Usage:
    python tradebot.py run            # main loop (paper or live per Config)
    python tradebot.py backfill       # fetch candle history only
    python tradebot.py train          # force (re)train models
    python tradebot.py report         # print performance metrics
    python tradebot.py reset-halt     # clear kill-switch after manual review

Run under systemd with Restart=on-failure. The process may die at any
time; state is reconstructed from the DB on start (crash-only design).

DB SCHEMA (viewer contract — all ts are unix seconds, UTC)
----------------------------------------------------------
candles     (ts, product, open, high, low, close, volume)      PK(product,ts)
predictions (ts, product, p_long, p_flat, p_short,
             meta_p, regime_ok, model_ver)                     PK(product,ts)
actions     (ts, product, action, reason, price, size,
             confidence)          -- action: BUY/SELL/HOLD/HALT/STOP/TP/SKIP
trades      (id, product, side, ts_open, entry, size, stop, tp,
             ts_close, exit, pnl, fees, status)  -- status: OPEN/CLOSED
equity      (ts, equity, drawdown, day_pnl)
events      (ts, level, msg)                     -- INFO/WARN/ERROR
state       (key, value)                         -- json values
"""

import json, math, os, sys, time, uuid, sqlite3, signal
from dataclasses import dataclass, field
from datetime import datetime, timezone

import numpy as np
import pandas as pd
import torch
import torch.nn as nn

try:
    import xgboost as xgb
    HAVE_XGB = True
except ImportError:
    HAVE_XGB = False

try:
    from coinbase.rest import RESTClient
    HAVE_CB = True
except ImportError:
    HAVE_CB = False


# ────────────────────────────────────────────────────────────────────────
# Config
# ────────────────────────────────────────────────────────────────────────
@dataclass
class Config:
    # exchange
    api_key: str = os.environ.get("COINBASE_API_KEY", "")
    api_secret: str = os.environ.get("COINBASE_API_SECRET", "")
    product: str = "BTC-EUR"
    paper: bool = True                    # dry trading — flip only after 2 months
    # timeframe
    granularity: str = "ONE_HOUR"         # FIFTEEN_MINUTE / ONE_HOUR / SIX_HOUR / ONE_DAY
    tf_sec: int = 3600
    history_days: int = 365               # backfill depth for training
    # model
    seq_len: int = 64                     # LSTM lookback (bars)
    horizon: int = 12                     # triple-barrier horizon (bars)
    barrier_atr: float = 1.5              # TP/SL distance in ATR for labeling
    retrain_every_bars: int = 24          # nightly on 1h bars
    min_train_bars: int = 2000
    # trading
    fee_rate: float = 0.006               # taker, conservative
    slippage: float = 0.0005
    start_equity: float = 10_000.0        # paper starting capital (EUR)
    target_vol: float = 0.01              # per-trade risk fraction of equity
    max_position_frac: float = 0.25       # hard cap: fraction of equity per trade
    stop_atr: float = 2.0
    tp_atr: float = 3.0
    entry_meta_min: float = 0.58          # meta-model gate
    entry_dir_min: float = 0.45           # primary direction prob gate
    # risk / kill switch
    daily_loss_limit: float = 0.02        # -2% day → pause until next UTC day
    max_drawdown: float = 0.10            # -10% from peak → HALT (manual reset)
    # infra
    db_path: str = "tradebot.db"
    model_dir: str = "models"
    heartbeat_path: str = "tradebot.heartbeat"


# ────────────────────────────────────────────────────────────────────────
# Database — single source of truth, viewer-friendly
# ────────────────────────────────────────────────────────────────────────
class Database:
    SCHEMA = """
    CREATE TABLE IF NOT EXISTS candles(
      ts INTEGER, product TEXT, open REAL, high REAL, low REAL, close REAL,
      volume REAL, PRIMARY KEY(product, ts));
    CREATE TABLE IF NOT EXISTS predictions(
      ts INTEGER, product TEXT, p_long REAL, p_flat REAL, p_short REAL,
      meta_p REAL, regime_ok INTEGER, model_ver TEXT,
      PRIMARY KEY(product, ts));
    CREATE TABLE IF NOT EXISTS actions(
      ts INTEGER, product TEXT, action TEXT, reason TEXT,
      price REAL, size REAL, confidence REAL);
    CREATE TABLE IF NOT EXISTS trades(
      id TEXT PRIMARY KEY, product TEXT, side TEXT, ts_open INTEGER,
      entry REAL, size REAL, stop REAL, tp REAL,
      ts_close INTEGER, exit REAL, pnl REAL, fees REAL, status TEXT);
    CREATE TABLE IF NOT EXISTS equity(
      ts INTEGER PRIMARY KEY, equity REAL, drawdown REAL, day_pnl REAL);
    CREATE TABLE IF NOT EXISTS events(
      ts INTEGER, level TEXT, msg TEXT);
    CREATE TABLE IF NOT EXISTS state(
      key TEXT PRIMARY KEY, value TEXT);
    CREATE INDEX IF NOT EXISTS idx_actions_ts ON actions(ts);
    CREATE INDEX IF NOT EXISTS idx_events_ts  ON events(ts);
    """

    def __init__(self, path):
        self.con = sqlite3.connect(path)
        self.con.execute("PRAGMA journal_mode=WAL")   # viewer can read while we write
        self.con.execute("PRAGMA synchronous=NORMAL")
        self.con.executescript(self.SCHEMA)

    # -- state kv --------------------------------------------------------
    def get(self, key, default=None):
        row = self.con.execute("SELECT value FROM state WHERE key=?", (key,)).fetchone()
        return json.loads(row[0]) if row else default

    def set(self, key, value):
        self.con.execute("INSERT OR REPLACE INTO state VALUES(?,?)",
                         (key, json.dumps(value)))
        self.con.commit()

    # -- writers ---------------------------------------------------------
    def upsert_candles(self, product, df):
        rows = [(int(r.ts), product, r.open, r.high, r.low, r.close, r.volume)
                for r in df.itertuples()]
        self.con.executemany(
            "INSERT OR REPLACE INTO candles VALUES(?,?,?,?,?,?,?)", rows)
        self.con.commit()

    def log_prediction(self, ts, product, p, meta_p, regime_ok, ver):
        self.con.execute("INSERT OR REPLACE INTO predictions VALUES(?,?,?,?,?,?,?,?)",
                         (ts, product, p[0], p[1], p[2], meta_p, int(regime_ok), ver))
        self.con.commit()

    def log_action(self, ts, product, action, reason, price=0.0, size=0.0, conf=0.0):
        self.con.execute("INSERT INTO actions VALUES(?,?,?,?,?,?,?)",
                         (ts, product, action, reason, price, size, conf))
        self.con.commit()

    def log_event(self, level, msg):
        self.con.execute("INSERT INTO events VALUES(?,?,?)",
                         (int(time.time()), level, msg))
        self.con.commit()
        print(f"[{level}] {msg}", flush=True)

    def log_equity(self, ts, equity, drawdown, day_pnl):
        self.con.execute("INSERT OR REPLACE INTO equity VALUES(?,?,?,?)",
                         (ts, equity, drawdown, day_pnl))
        self.con.commit()

    def open_trade(self, t):
        self.con.execute("INSERT INTO trades VALUES(?,?,?,?,?,?,?,?,NULL,NULL,NULL,?, 'OPEN')",
                         (t["id"], t["product"], t["side"], t["ts_open"], t["entry"],
                          t["size"], t["stop"], t["tp"], t["fees"]))
        self.con.commit()

    def close_trade(self, tid, ts, exit_px, pnl, fees):
        self.con.execute("""UPDATE trades SET ts_close=?, exit=?, pnl=?,
                            fees=fees+?, status='CLOSED' WHERE id=?""",
                         (ts, exit_px, pnl, fees, tid))
        self.con.commit()

    # -- readers ---------------------------------------------------------
    def load_candles(self, product, limit=None):
        q = "SELECT ts,open,high,low,close,volume FROM candles WHERE product=? ORDER BY ts"
        df = pd.read_sql_query(q, self.con, params=(product,))
        return df.tail(limit) if limit else df

    def open_position(self, product):
        row = self.con.execute(
            "SELECT id,side,ts_open,entry,size,stop,tp,fees FROM trades "
            "WHERE product=? AND status='OPEN'", (product,)).fetchone()
        if not row:
            return None
        keys = ["id", "side", "ts_open", "entry", "size", "stop", "tp", "fees"]
        return dict(zip(keys, row))

    def closed_trades(self):
        return pd.read_sql_query(
            "SELECT * FROM trades WHERE status='CLOSED' ORDER BY ts_close", self.con)

    def last_candle_ts(self, product):
        row = self.con.execute(
            "SELECT MAX(ts) FROM candles WHERE product=?", (product,)).fetchone()
        return row[0] or 0


# ────────────────────────────────────────────────────────────────────────
# Exchange adapters
# ────────────────────────────────────────────────────────────────────────
class ExchangeAdapter:
    def fetch_candles(self, product, granularity, start, end): raise NotImplementedError
    def market_buy(self, product, quote_size, price): raise NotImplementedError
    def market_sell(self, product, base_size, price): raise NotImplementedError


class CoinbaseAdapter(ExchangeAdapter):
    GRAN_SEC = {"FIFTEEN_MINUTE": 900, "ONE_HOUR": 3600,
                "SIX_HOUR": 21600, "ONE_DAY": 86400}

    def __init__(self, cfg: Config):
        if not HAVE_CB:
            raise RuntimeError("pip install coinbase-advanced-py")
        self.client = RESTClient(api_key=cfg.api_key, api_secret=cfg.api_secret)
        self.cfg = cfg

    def fetch_candles(self, product, granularity, start, end):
        """Chunked fetch (API max 350 candles per call). Returns ascending DataFrame."""
        sec, out = self.GRAN_SEC[granularity], []
        cur = start
        while cur < end:
            chunk_end = min(cur + 349 * sec, end)
            resp = self.client.get_candles(product_id=product, start=str(cur),
                                           end=str(chunk_end), granularity=granularity)
            for c in resp["candles"]:
                out.append((int(c["start"]), float(c["open"]), float(c["high"]),
                            float(c["low"]), float(c["close"]), float(c["volume"])))
            cur = chunk_end + sec
            time.sleep(0.25)  # rate-limit courtesy
        df = pd.DataFrame(out, columns=["ts", "open", "high", "low", "close", "volume"])
        return df.drop_duplicates("ts").sort_values("ts").reset_index(drop=True)

    def market_buy(self, product, quote_size, price):
        o = self.client.market_order_buy(client_order_id=str(uuid.uuid4()),
                                         product_id=product, quote_size=str(round(quote_size, 2)))
        return {"order_id": o["order_id"], "price": price, "fee": quote_size * self.cfg.fee_rate}

    def market_sell(self, product, base_size, price):
        o = self.client.market_order_sell(client_order_id=str(uuid.uuid4()),
                                          product_id=product, base_size=str(round(base_size, 8)))
        return {"order_id": o["order_id"], "price": price,
                "fee": base_size * price * self.cfg.fee_rate}


class PaperAdapter(ExchangeAdapter):
    """Simulated fills at live price with slippage + fees.
    Candle data still comes from the real exchange (delegate)."""

    def __init__(self, cfg: Config, delegate: ExchangeAdapter):
        self.cfg, self.delegate = cfg, delegate

    def fetch_candles(self, *a, **kw):
        return self.delegate.fetch_candles(*a, **kw)

    def market_buy(self, product, quote_size, price):
        px = price * (1 + self.cfg.slippage)
        return {"order_id": "paper-" + uuid.uuid4().hex[:12], "price": px,
                "fee": quote_size * self.cfg.fee_rate}

    def market_sell(self, product, base_size, price):
        px = price * (1 - self.cfg.slippage)
        return {"order_id": "paper-" + uuid.uuid4().hex[:12], "price": px,
                "fee": base_size * px * self.cfg.fee_rate}


# ────────────────────────────────────────────────────────────────────────
# Feature engineering + triple-barrier labeling
# ────────────────────────────────────────────────────────────────────────
class FeatureEngine:
    COLS = ["ret1", "ret3", "ret6", "ret12", "rsi", "macd_h", "bb_pos",
            "atr_rel", "vol_z", "vwap_d", "hod_sin", "hod_cos", "dow_sin", "dow_cos"]

    def compute(self, df):
        """df: ascending candles. Returns feature DataFrame aligned to df.index."""
        c, h, l, v = df["close"], df["high"], df["low"], df["volume"]
        f = pd.DataFrame(index=df.index)
        logc = np.log(c)
        for n in (1, 3, 6, 12):
            f[f"ret{n}"] = logc.diff(n)
        # RSI 14
        d = c.diff()
        up = d.clip(lower=0).ewm(alpha=1 / 14, adjust=False).mean()
        dn = (-d.clip(upper=0)).ewm(alpha=1 / 14, adjust=False).mean()
        f["rsi"] = (100 - 100 / (1 + up / (dn + 1e-12))) / 100 - 0.5
        # MACD histogram, ATR-normalized
        macd = c.ewm(span=12).mean() - c.ewm(span=26).mean()
        f["macd_h"] = (macd - macd.ewm(span=9).mean()) / c
        # Bollinger position
        m20, s20 = c.rolling(20).mean(), c.rolling(20).std()
        f["bb_pos"] = ((c - m20) / (2 * s20 + 1e-12)).clip(-2, 2)
        # ATR relative
        tr = pd.concat([h - l, (h - c.shift()).abs(), (l - c.shift()).abs()], axis=1).max(axis=1)
        f["atr_rel"] = tr.ewm(alpha=1 / 14, adjust=False).mean() / c
        # volume z-score
        f["vol_z"] = ((v - v.rolling(48).mean()) / (v.rolling(48).std() + 1e-12)).clip(-4, 4)
        # VWAP distance (rolling 24)
        vwap = (c * v).rolling(24).sum() / (v.rolling(24).sum() + 1e-12)
        f["vwap_d"] = (c - vwap) / c
        # cyclic time encoding
        dt = pd.to_datetime(df["ts"], unit="s", utc=True)
        f["hod_sin"] = np.sin(2 * np.pi * dt.dt.hour / 24)
        f["hod_cos"] = np.cos(2 * np.pi * dt.dt.hour / 24)
        f["dow_sin"] = np.sin(2 * np.pi * dt.dt.dayofweek / 7)
        f["dow_cos"] = np.cos(2 * np.pi * dt.dt.dayofweek / 7)
        return f[self.COLS]

    def atr_abs(self, df):
        c, h, l = df["close"], df["high"], df["low"]
        tr = pd.concat([h - l, (h - c.shift()).abs(), (l - c.shift()).abs()], axis=1).max(axis=1)
        return tr.ewm(alpha=1 / 14, adjust=False).mean()

    def adx(self, df, n=14):
        h, l, c = df["high"], df["low"], df["close"]
        up, dn = h.diff(), -l.diff()
        pdm = np.where((up > dn) & (up > 0), up, 0.0)
        ndm = np.where((dn > up) & (dn > 0), dn, 0.0)
        tr = pd.concat([h - l, (h - c.shift()).abs(), (l - c.shift()).abs()], axis=1).max(axis=1)
        atr = tr.ewm(alpha=1 / n, adjust=False).mean()
        pdi = 100 * pd.Series(pdm, index=df.index).ewm(alpha=1 / n, adjust=False).mean() / (atr + 1e-12)
        ndi = 100 * pd.Series(ndm, index=df.index).ewm(alpha=1 / n, adjust=False).mean() / (atr + 1e-12)
        dx = 100 * (pdi - ndi).abs() / (pdi + ndi + 1e-12)
        return dx.ewm(alpha=1 / n, adjust=False).mean()


class TripleBarrierLabeler:
    """Label 2=long-win (TP first), 0=short-win (SL first), 1=timeout/flat."""

    def __init__(self, cfg: Config):
        self.cfg = cfg

    def label(self, df, atr):
        c, h, l = df["close"].values, df["high"].values, df["low"].values
        n, H, k = len(df), self.cfg.horizon, self.cfg.barrier_atr
        y = np.full(n, 1, dtype=np.int64)
        for i in range(n - H - 1):
            tp = c[i] + k * atr.iloc[i]
            sl = c[i] - k * atr.iloc[i]
            for j in range(i + 1, i + 1 + H):
                if h[j] >= tp:
                    y[i] = 2
                    break
                if l[j] <= sl:
                    y[i] = 0
                    break
        return y


# ────────────────────────────────────────────────────────────────────────
# Models: attention-LSTM + XGBoost primary + XGBoost meta
# ────────────────────────────────────────────────────────────────────────
class AttnLSTM(nn.Module):
    def __init__(self, n_feat, hidden=96, layers=2, n_cls=3):
        super().__init__()
        self.lstm = nn.LSTM(n_feat, hidden, layers, batch_first=True, dropout=0.3)
        self.attn = nn.Linear(hidden, 1)
        self.norm = nn.LayerNorm(hidden)
        self.head = nn.Sequential(nn.Linear(hidden, 64), nn.GELU(),
                                  nn.Dropout(0.2), nn.Linear(64, n_cls))

    def forward(self, x):                      # x: (B, T, F)
        out, _ = self.lstm(x)
        w = torch.softmax(self.attn(out), dim=1)   # attention pooling over time
        ctx = self.norm((w * out).sum(dim=1))
        return self.head(ctx)                  # logits


class ModelEnsemble:
    """XGBoost gives primary direction, LSTM refines, meta-XGB gates entry."""
    VER_KEY = "model_version"

    def __init__(self, cfg: Config, db: Database):
        self.cfg, self.db = cfg, db
        self.fe = FeatureEngine()
        self.labeler = TripleBarrierLabeler(cfg)
        self.lstm = None
        self.xgb_dir = None
        self.xgb_meta = None
        self.feat_mu = self.feat_sd = None
        self.version = "untrained"
        os.makedirs(cfg.model_dir, exist_ok=True)
        self._load()

    # -- persistence (atomic swap) ----------------------------------------
    def _paths(self):
        d = self.cfg.model_dir
        return (f"{d}/lstm.pt", f"{d}/xgb_dir.json", f"{d}/xgb_meta.json", f"{d}/norm.json")

    def _load(self):
        lp, xp, mp, np_ = self._paths()
        try:
            norm = json.load(open(np_))
            self.feat_mu = np.array(norm["mu"])
            self.feat_sd = np.array(norm["sd"])
            self.version = norm["version"]
            self.lstm = AttnLSTM(len(FeatureEngine.COLS))
            self.lstm.load_state_dict(torch.load(lp, map_location="cpu"))
            self.lstm.eval()
            if HAVE_XGB and os.path.exists(xp):
                self.xgb_dir = xgb.XGBClassifier()
                self.xgb_dir.load_model(xp)
                self.xgb_meta = xgb.XGBClassifier()
                self.xgb_meta.load_model(mp)
            self.db.log_event("INFO", f"models loaded, version={self.version}")
        except (FileNotFoundError, json.JSONDecodeError, KeyError):
            self.db.log_event("WARN", "no trained models found — run train first")

    def _save(self):
        lp, xp, mp, np_ = self._paths()
        torch.save(self.lstm.state_dict(), lp + ".tmp")
        os.replace(lp + ".tmp", lp)
        if self.xgb_dir is not None:
            self.xgb_dir.save_model(xp)
            self.xgb_meta.save_model(mp)
        json.dump({"mu": self.feat_mu.tolist(), "sd": self.feat_sd.tolist(),
                   "version": self.version}, open(np_ + ".tmp", "w"))
        os.replace(np_ + ".tmp", np_)

    # -- training ----------------------------------------------------------
    def train(self, df):
        cfg = self.cfg
        if len(df) < cfg.min_train_bars:
            self.db.log_event("WARN", f"train skipped: {len(df)} < {cfg.min_train_bars} bars")
            return False
        feats = self.fe.compute(df)
        atr = self.fe.atr_abs(df)
        y = self.labeler.label(df, atr)
        valid = feats.dropna().index
        valid = valid[valid < len(df) - cfg.horizon - 1]
        X, y = feats.loc[valid].values, y[valid]

        # normalize on train split only (last 15% = validation, walk-forward style)
        split = int(len(X) * 0.85)
        self.feat_mu, self.feat_sd = X[:split].mean(0), X[:split].std(0) + 1e-9
        Xn = (X - self.feat_mu) / self.feat_sd

        # ---- XGBoost primary direction ----
        if HAVE_XGB:
            self.xgb_dir = xgb.XGBClassifier(
                n_estimators=400, max_depth=5, learning_rate=0.03,
                subsample=0.8, colsample_bytree=0.8, objective="multi:softprob",
                num_class=3, eval_metric="mlogloss", early_stopping_rounds=30)
            self.xgb_dir.fit(Xn[:split], y[:split],
                             eval_set=[(Xn[split:], y[split:])], verbose=False)

        # ---- LSTM ----
        T = cfg.seq_len
        seqs = np.lib.stride_tricks.sliding_window_view(Xn, (T, Xn.shape[1]))[:, 0]
        ys = y[T - 1:]
        s_split = split - T + 1
        Xtr = torch.tensor(seqs[:s_split], dtype=torch.float32)
        ytr = torch.tensor(ys[:s_split])
        Xva = torch.tensor(seqs[s_split:], dtype=torch.float32)
        yva = torch.tensor(ys[s_split:])

        self.lstm = AttnLSTM(Xn.shape[1])
        opt = torch.optim.AdamW(self.lstm.parameters(), lr=1e-3, weight_decay=1e-4)
        crit = nn.CrossEntropyLoss()
        best, best_state, patience = 1e9, None, 0
        for ep in range(60):
            self.lstm.train()
            perm = torch.randperm(len(Xtr))
            for i in range(0, len(Xtr), 256):
                idx = perm[i:i + 256]
                opt.zero_grad()
                loss = crit(self.lstm(Xtr[idx]), ytr[idx])
                loss.backward()
                nn.utils.clip_grad_norm_(self.lstm.parameters(), 1.0)
                opt.step()
            self.lstm.eval()
            with torch.no_grad():
                vloss = crit(self.lstm(Xva), yva).item()
            if vloss < best - 1e-4:
                best, best_state, patience = vloss, self.lstm.state_dict(), 0
            else:
                patience += 1
                if patience >= 8:
                    break
        self.lstm.load_state_dict(best_state)
        self.lstm.eval()

        # ---- meta-model: was the primary signal profitable? ----
        if HAVE_XGB:
            proba = self.xgb_dir.predict_proba(Xn)
            sig = proba.argmax(1)
            mask = sig != 1                              # only bars with a direction
            meta_y = (sig[mask] == y[mask]).astype(int)  # 1 = signal was right
            meta_X = np.hstack([Xn[mask], proba[mask]])
            ms = int(len(meta_X) * 0.85)
            self.xgb_meta = xgb.XGBClassifier(
                n_estimators=300, max_depth=4, learning_rate=0.05,
                subsample=0.8, eval_metric="logloss", early_stopping_rounds=30)
            self.xgb_meta.fit(meta_X[:ms], meta_y[:ms],
                              eval_set=[(meta_X[ms:], meta_y[ms:])], verbose=False)

        self.version = datetime.now(timezone.utc).strftime("%Y%m%d-%H%M")
        self._save()
        self.db.log_event("INFO", f"training done, version={self.version}, "
                                  f"lstm_vloss={best:.4f}, bars={len(X)}")
        return True

    # -- inference ----------------------------------------------------------
    def predict(self, df):
        """Returns (p_dir[3], meta_p) or None if not trained / not enough data."""
        if self.lstm is None or self.feat_mu is None:
            return None
        feats = self.fe.compute(df).dropna()
        if len(feats) < self.cfg.seq_len:
            return None
        Xn = (feats.values - self.feat_mu) / self.feat_sd
        with torch.no_grad():
            seq = torch.tensor(Xn[-self.cfg.seq_len:], dtype=torch.float32).unsqueeze(0)
            p_lstm = torch.softmax(self.lstm(seq), dim=1).numpy()[0]
        if HAVE_XGB and self.xgb_dir is not None:
            p_xgb = self.xgb_dir.predict_proba(Xn[-1:])[0]
            p_dir = 0.5 * p_lstm + 0.5 * p_xgb            # blend
            meta_in = np.hstack([Xn[-1:], p_xgb[None, :]])
            meta_p = float(self.xgb_meta.predict_proba(meta_in)[0, 1])
        else:
            p_dir, meta_p = p_lstm, float(max(p_lstm[0], p_lstm[2]))
        return p_dir, meta_p


# ────────────────────────────────────────────────────────────────────────
# Regime filter + risk manager
# ────────────────────────────────────────────────────────────────────────
class RegimeFilter:
    """Trade only when the market trends enough to pay the fees."""

    def __init__(self, cfg: Config):
        self.cfg, self.fe = cfg, FeatureEngine()

    def ok(self, df):
        adx = self.fe.adx(df).iloc[-1]
        atr_rel = self.fe.atr_abs(df).iloc[-1] / df["close"].iloc[-1]
        return bool(adx > 20 and atr_rel > 2.5 * self.cfg.fee_rate)


class RiskManager:
    def __init__(self, cfg: Config, db: Database):
        self.cfg, self.db = cfg, db
        self.equity = db.get("equity", cfg.start_equity)
        self.peak = db.get("equity_peak", self.equity)
        self.day = db.get("day", self._today())
        self.day_start_eq = db.get("day_start_eq", self.equity)
        self.halted = db.get("halted", False)

    def _today(self):
        return datetime.now(timezone.utc).strftime("%Y-%m-%d")

    def _persist(self):
        for k, v in (("equity", self.equity), ("equity_peak", self.peak),
                     ("day", self.day), ("day_start_eq", self.day_start_eq),
                     ("halted", self.halted)):
            self.db.set(k, v)

    def roll_day(self):
        if self._today() != self.day:
            self.day, self.day_start_eq = self._today(), self.equity
            self._persist()
            return True
        return False

    def update_equity(self, delta):
        self.equity += delta
        self.peak = max(self.peak, self.equity)
        dd = 1 - self.equity / self.peak
        if dd >= self.cfg.max_drawdown and not self.halted:
            self.halted = True
            self.db.log_event("ERROR",
                f"KILL SWITCH: drawdown {dd:.1%} >= {self.cfg.max_drawdown:.0%} — HALTED. "
                f"Manual 'reset-halt' required.")
        self._persist()
        return dd

    @property
    def day_pnl(self):
        return self.equity - self.day_start_eq

    def day_limit_hit(self):
        return self.day_pnl <= -self.cfg.daily_loss_limit * self.day_start_eq

    def can_trade(self):
        if self.halted:
            return False, "halted (max drawdown kill switch)"
        if self.day_limit_hit():
            return False, f"daily loss limit hit ({self.day_pnl:.2f})"
        return True, ""

    def position_size(self, price, atr_abs):
        """Volatility targeting with hard cap. Returns base size."""
        atr_rel = atr_abs / price
        risk_eur = self.cfg.target_vol * self.equity
        size_eur = min(risk_eur / (self.cfg.stop_atr * atr_rel),
                       self.cfg.max_position_frac * self.equity)
        return size_eur / price


# ────────────────────────────────────────────────────────────────────────
# Reporter
# ────────────────────────────────────────────────────────────────────────
class Reporter:
    def __init__(self, db: Database, cfg: Config):
        self.db, self.cfg = db, cfg

    def metrics(self):
        eq = pd.read_sql_query("SELECT * FROM equity ORDER BY ts", self.db.con)
        tr = self.db.closed_trades()
        if len(eq) < 2:
            return {"note": "not enough data"}
        r = eq["equity"].pct_change().dropna()
        ann = math.sqrt(365 * 24 * 3600 / max(self.cfg.tf_sec, 1))
        sharpe = ann * r.mean() / (r.std() + 1e-12)
        downside = r[r < 0].std() + 1e-12
        sortino = ann * r.mean() / downside
        wins = tr[tr["pnl"] > 0]["pnl"].sum() if len(tr) else 0
        losses = -tr[tr["pnl"] <= 0]["pnl"].sum() if len(tr) else 0
        return {
            "equity": round(eq["equity"].iloc[-1], 2),
            "max_drawdown": round(eq["drawdown"].max(), 4),
            "sharpe": round(sharpe, 2), "sortino": round(sortino, 2),
            "trades": len(tr),
            "win_rate": round((tr["pnl"] > 0).mean(), 3) if len(tr) else None,
            "profit_factor": round(wins / losses, 2) if losses > 0 else None,
            "fees_paid": round(tr["fees"].sum(), 2) if len(tr) else 0,
        }


# ────────────────────────────────────────────────────────────────────────
# Bot orchestrator — crash-only, restartable
# ────────────────────────────────────────────────────────────────────────
class Bot:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.db = Database(cfg.db_path)
        base = CoinbaseAdapter(cfg) if HAVE_CB else None
        if cfg.paper:
            if base is None:
                raise RuntimeError("paper mode still needs coinbase-advanced-py for candles")
            self.ex = PaperAdapter(cfg, base)
        else:
            self.ex = base
        self.models = ModelEnsemble(cfg, self.db)
        self.regime = RegimeFilter(cfg)
        self.risk = RiskManager(cfg, self.db)
        self.reporter = Reporter(self.db, cfg)
        self.fe = FeatureEngine()
        self.position = self.db.open_position(cfg.product)   # restart: reload open trade
        self.bars_since_train = self.db.get("bars_since_train", 0)
        self._stop = False
        signal.signal(signal.SIGTERM, self._sigterm)
        signal.signal(signal.SIGINT, self._sigterm)
        if self.position:
            self.db.log_event("INFO", f"restart: resumed open {self.position['side']} "
                                      f"position from {self.position['ts_open']}")

    def _sigterm(self, *_):
        self._stop = True

    # -- data ------------------------------------------------------------
    def backfill(self):
        cfg = self.cfg
        last = self.db.last_candle_ts(cfg.product)
        start = last + cfg.tf_sec if last else int(time.time()) - cfg.history_days * 86400
        end = int(time.time())
        if end - start < cfg.tf_sec:
            return
        self.db.log_event("INFO", f"backfill {cfg.product} from {start} to {end}")
        df = self.ex.fetch_candles(cfg.product, cfg.granularity, start, end)
        if len(df):
            self.db.upsert_candles(cfg.product, df)
            self.db.log_event("INFO", f"backfill: {len(df)} candles stored")

    # -- trade lifecycle ---------------------------------------------------
    def _enter(self, ts, price, atr_abs, p_dir, meta_p):
        side = "LONG" if p_dir[2] > p_dir[0] else "SHORT"
        if side == "SHORT" and self.cfg.paper is False:
            # spot exchange: no real shorting — skip live, allow in paper for research
            self.db.log_action(ts, self.cfg.product, "SKIP", "short not possible on spot",
                               price, 0, meta_p)
            return
        size = self.risk.position_size(price, atr_abs)
        fill = (self.ex.market_buy(self.cfg.product, size * price, price) if side == "LONG"
                else self.ex.market_sell(self.cfg.product, size, price))
        sgn = 1 if side == "LONG" else -1
        trade = {"id": uuid.uuid4().hex[:16], "product": self.cfg.product, "side": side,
                 "ts_open": ts, "entry": fill["price"], "size": size,
                 "stop": fill["price"] - sgn * self.cfg.stop_atr * atr_abs,
                 "tp":   fill["price"] + sgn * self.cfg.tp_atr * atr_abs,
                 "fees": fill["fee"]}
        self.db.open_trade(trade)
        self.position = trade
        self.risk.update_equity(-fill["fee"])
        self.db.log_action(ts, self.cfg.product, "BUY" if side == "LONG" else "SELL",
                           f"entry {side} meta={meta_p:.3f}", fill["price"], size, meta_p)

    def _exit(self, ts, price, reason):
        p = self.position
        sgn = 1 if p["side"] == "LONG" else -1
        fill = (self.ex.market_sell(self.cfg.product, p["size"], price) if p["side"] == "LONG"
                else self.ex.market_buy(self.cfg.product, p["size"] * price, price))
        pnl = sgn * (fill["price"] - p["entry"]) * p["size"] - fill["fee"]
        self.db.close_trade(p["id"], ts, fill["price"], pnl, fill["fee"])
        self.risk.update_equity(pnl)
        self.db.log_action(ts, self.cfg.product, reason,
                           f"exit {p['side']} pnl={pnl:.2f}", fill["price"], p["size"], 0)
        self.position = None

    def _check_stops(self, ts, candle):
        """Intrabar stop/TP check on the just-closed candle."""
        p = self.position
        if not p:
            return
        if p["side"] == "LONG":
            if candle.low <= p["stop"]:
                self._exit(ts, p["stop"], "STOP")
            elif candle.high >= p["tp"]:
                self._exit(ts, p["tp"], "TP")
        else:
            if candle.high >= p["stop"]:
                self._exit(ts, p["stop"], "STOP")
            elif candle.low <= p["tp"]:
                self._exit(ts, p["tp"], "TP")

    # -- one bar -----------------------------------------------------------
    def on_bar(self):
        cfg = self.cfg
        self.backfill()                       # also fills gaps after downtime
        df = self.db.load_candles(cfg.product, limit=cfg.seq_len + 200)
        if len(df) < cfg.seq_len + 60:
            self.db.log_event("WARN", "not enough candles yet")
            return
        ts = int(df["ts"].iloc[-1])
        candle = df.iloc[-1]
        price = float(candle.close)
        atr_abs = float(self.fe.atr_abs(df).iloc[-1])

        self._check_stops(ts, candle)
        if self.risk.roll_day():
            self.db.log_event("INFO", f"new UTC day, report: {self.reporter.metrics()}")

        # retrain schedule
        self.bars_since_train += 1
        self.db.set("bars_since_train", self.bars_since_train)
        if self.bars_since_train >= cfg.retrain_every_bars or self.models.lstm is None:
            full = self.db.load_candles(cfg.product)
            if self.models.train(full):
                self.bars_since_train = 0
                self.db.set("bars_since_train", 0)

        # predict + decide
        pred = self.models.predict(df)
        regime_ok = self.regime.ok(df)
        if pred is None:
            self.db.log_action(ts, cfg.product, "HOLD", "model not ready", price)
            return
        p_dir, meta_p = pred
        self.db.log_prediction(ts, cfg.product, p_dir.tolist(), meta_p,
                               regime_ok, self.models.version)

        ok, why = self.risk.can_trade()
        if not ok:
            if self.position:
                self._exit(ts, price, "HALT")
            self.db.log_action(ts, cfg.product, "HALT", why, price)
        elif self.position is None:
            dir_p = max(p_dir[0], p_dir[2])
            if regime_ok and dir_p >= cfg.entry_dir_min and meta_p >= cfg.entry_meta_min:
                self._enter(ts, price, atr_abs, p_dir, meta_p)
            else:
                self.db.log_action(ts, cfg.product, "HOLD",
                                   f"gate: dir={dir_p:.3f} meta={meta_p:.3f} regime={regime_ok}",
                                   price, 0, meta_p)
        else:
            # opposite strong signal → flip out
            sgn_pos = 1 if self.position["side"] == "LONG" else -1
            opp = p_dir[0] if sgn_pos > 0 else p_dir[2]
            if opp > 0.55 and meta_p >= cfg.entry_meta_min:
                self._exit(ts, price, "SELL" if sgn_pos > 0 else "BUY")
            else:
                self.db.log_action(ts, cfg.product, "HOLD", "position open", price)

        # mark-to-market equity for the viewer
        unreal = 0.0
        if self.position:
            sgn = 1 if self.position["side"] == "LONG" else -1
            unreal = sgn * (price - self.position["entry"]) * self.position["size"]
        eq = self.risk.equity + unreal
        dd = 1 - eq / max(self.risk.peak, eq)
        self.db.log_equity(ts, eq, dd, self.risk.day_pnl + unreal)

    # -- main loop -----------------------------------------------------------
    def run(self):
        self.db.log_event("INFO", f"bot start | {self.cfg.product} {self.cfg.granularity} "
                                  f"| paper={self.cfg.paper} | equity={self.risk.equity:.2f}")
        while not self._stop:
            try:
                self.on_bar()
            except Exception as e:
                self.db.log_event("ERROR", f"on_bar: {type(e).__name__}: {e}")
            # heartbeat for external watchdog
            with open(self.cfg.heartbeat_path, "w") as f:
                f.write(str(int(time.time())))
            # sleep until shortly after next candle close
            now = time.time()
            nxt = (int(now) // self.cfg.tf_sec + 1) * self.cfg.tf_sec + 5
            while time.time() < nxt and not self._stop:
                time.sleep(min(5, nxt - time.time()))
        self.db.log_event("INFO", "clean shutdown (state persisted, restart-safe)")


# ────────────────────────────────────────────────────────────────────────
# Entry point
# ────────────────────────────────────────────────────────────────────────
def main():
    cfg = Config()
    tf = {"FIFTEEN_MINUTE": 900, "ONE_HOUR": 3600, "SIX_HOUR": 21600, "ONE_DAY": 86400}
    cfg.tf_sec = tf[cfg.granularity]
    cmd = sys.argv[1] if len(sys.argv) > 1 else "run"

    if cmd == "run":
        Bot(cfg).run()
    elif cmd == "backfill":
        Bot(cfg).backfill()
    elif cmd == "train":
        bot = Bot(cfg)
        bot.backfill()
        bot.models.train(bot.db.load_candles(cfg.product))
    elif cmd == "report":
        db = Database(cfg.db_path)
        print(json.dumps(Reporter(db, cfg).metrics(), indent=2))
    elif cmd == "reset-halt":
        db = Database(cfg.db_path)
        db.set("halted", False)
        db.log_event("WARN", "kill switch manually reset")
    else:
        print(__doc__)


if __name__ == "__main__":
    main()
