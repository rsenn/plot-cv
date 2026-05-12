"""
Coinbase Trading Bot mit PyTorch LSTM-Modell
============================================
Benötigte Pakete:
    pip install torch numpy pandas coinbase-advanced-py scikit-learn

WICHTIG: Nur zu Lernzwecken. Kein echter Finanzrat.
         Teste immer zuerst im Sandbox/Paper-Trading-Modus!
"""

import time
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
from os import getenv
from sklearn.preprocessing import MinMaxScaler
from coinbase.rest import RESTClient  # coinbase-advanced-py
import pprint

logfile = open("coinbase-bot.log", "a")

def logprint(msg):
  print(msg)
  logfile.write(msg+'\n')
  logfile.flush()


# ---------------------------------------------
#1. CONFIGURATION
# ---------------------------------------------
API_KEY    = getenv('COINBASE_API_KEY') or "your_api_key"
API_SECRET = getenv('COINBASE_API_SECRET') or "your_api_secret"

PRODUCT_ID    = "BTC-EUR"   # Trading pair
SEQ_LEN       = 30          # LSTM sequence length (candles)
TRADE_AMOUNT  = 10.0        # EUR per trade
BUY_THRESHOLD  = 0.502      # Model Confidence for Buy
SELL_THRESHOLD = 0.498      # Model Confidence for Selling
LOOP_INTERVAL  = 60         # Seconds between cycles

# ---------------------------------------------
# 2. LSTM-MODEL
# ---------------------------------------------
class PriceLSTM(nn.Module):
    def __init__(self, input_size=1, hidden_size=64, num_layers=2):
        super().__init__()
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True, dropout=0.2)
        self.fc   = nn.Linear(hidden_size, 1)
        self.sig  = nn.Sigmoid()

    def forward(self, x):
        out, _ = self.lstm(x)
        return self.sig(self.fc(out[:, -1, :]))


# ---------------------------------------------
# 3. DATA HELPER
# ---------------------------------------------
def fetch_candles(client: RESTClient, product_id: str, limit: int = 150) -> pd.DataFrame:
    """Gets OHLCV data of the last `limit` 1 minute candles."""
    end   = int(time.time())
    start = end - limit * 60
    resp  = client.get_candles(
        product_id=product_id,
        start=str(start),
        end=str(end),
        granularity="ONE_MINUTE",
    )
    candles = resp["candles"]

    df = pd.DataFrame(c.to_dict() for c in candles)
    df = df.sort_values("start").reset_index(drop=True)
    df["close"] = df["close"].astype(float)
    return df


def prepare_sequence(prices: np.ndarray, seq_len: int, scaler: MinMaxScaler):
    """Normalizes prices and returns the last sequence tensor."""
    scaled = scaler.fit_transform(prices.reshape(-1, 1))
    seq    = scaled[-seq_len:]
    tensor = torch.tensor(seq, dtype=torch.float32).unsqueeze(0)  # (1, seq_len, 1)
    return tensor


# ---------------------------------------------
# 4. ORDER FUNCTIONS
# ---------------------------------------------
# 
"""Buy for `quote_size` EUR at market price."""
def place_market_buy(client: RESTClient, product_id: str, quote_size: float):
    import uuid
    order = client.market_order_buy(
        client_order_id=str(uuid.uuid4()),
        product_id=product_id,
        quote_size=str(quote_size),
    )
    logprint(f"  ✅ BUY-Order: {order['order_id']} | {quote_size} EUR")
    return order


"""Sell `base_size` BTC at market price."""
def place_market_sell(client: RESTClient, product_id: str, base_size: float):
    import uuid
    order = client.market_order_sell(
        client_order_id=str(uuid.uuid4()),
        product_id=product_id,
        base_size=str(round(base_size, 8)),
    )
    logprint(f"  ✅ SELL-Order: {order['order_id']} | {base_size} BTC")
    return order


"""Returns the available BTC balance."""
def get_btc_balance(client: RESTClient) -> float:
    accounts = client.get_accounts()["accounts"]
    for acc in accounts:
        if acc["currency"] == "BTC":
            return float(acc["available_balance"]["value"])
    return 0.0


# ---------------------------------------------
# 5. TRAINING LOOP (Quick Training)
# 
# ---------------------------------------------

"""Trains the model on current market data in the short term."""
def quick_train(model: PriceLSTM, prices: np.ndarray, seq_len: int, scaler: MinMaxScaler, epochs: int = 20):
    scaled = scaler.fit_transform(prices.reshape(-1, 1))
    X, y   = [], []
    for i in range(len(scaled) - seq_len - 1):
        X.append(scaled[i : i + seq_len])
        # Label: 1 if next candle is higher, otherwise 0
        y.append(1.0 if scaled[i + seq_len] > scaled[i + seq_len - 1] else 0.0)

    X = torch.tensor(np.array(X), dtype=torch.float32)
    y = torch.tensor(np.array(y), dtype=torch.float32).unsqueeze(1)

    optimizer = torch.optim.Adam(model.parameters(), lr=1e-3)
    criterion = nn.BCELoss()

    model.train()
    for ep in range(epochs):
        optimizer.zero_grad()
        loss = criterion(model(X), y)
        loss.backward()
        optimizer.step()
        if (ep + 1) % 5 == 0:
            logprint(f"    Epoch {ep+1}/{epochs} | Loss: {loss.item():.4f}")


# ---------------------------------------------
# 6. MAIN BOT LOOP
# ---------------------------------------------
def run_bot():
    logprint("🤖 Coinbase trading bot started")
    logprint(f"   Pair: {PRODUCT_ID} | Sequence: {SEQ_LEN} | Interval: {LOOP_INTERVAL}s\n")

    client  = RESTClient(api_key=API_KEY, api_secret=API_SECRET)
    model   = PriceLSTM()
    scaler  = MinMaxScaler()
    cycle   = 0

    while True:
        cycle += 1
        logprint(f"-- Cycle {cycle} | {time.strftime('%H:%M:%S')} --------------")

        try:
            # --- Fetch data ---
            df     = fetch_candles(client, PRODUCT_ID)
            prices = df["close"].values
            logprint(f"  📊 Last price: {prices[-1]:.2f} EUR")

            # --- Train model (renew every 10 cycles) ---
            if cycle % 10 == 1:
                logprint("  🔄 Training model...")
                quick_train(model, prices, SEQ_LEN, scaler, epochs=20)

            # --- forecast ---
            model.eval()
            with torch.no_grad():
                seq        = prepare_sequence(prices, SEQ_LEN, scaler)
                confidence = model(seq).item()
            logprint(f"  🧠 Model-confidence (upward movement): {confidence:.4f}")

            # --- Trading decision ---
            btc_balance = get_btc_balance(client)

            if confidence > BUY_THRESHOLD:
                logprint("  📈 Signal: BUY")
                place_market_buy(client, PRODUCT_ID, TRADE_AMOUNT)

            elif confidence < SELL_THRESHOLD and btc_balance > 0.00001:
                logprint("  📉 Signal: SELL")
                place_market_sell(client, PRODUCT_ID, btc_balance)

            else:
                logprint("  ⏸️  Signal: HOLD")

        except Exception as e:
            logprint(f"  ❌ Error: {e}")

        logprint(f"  ⏳ Waiting {LOOP_INTERVAL}s...\n")
        time.sleep(LOOP_INTERVAL)


# ---------------------------------------------
# 7. ENTRY POINT
# ---------------------------------------------
if __name__ == "__main__":
    run_bot()
