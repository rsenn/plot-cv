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
from sklearn.preprocessing import MinMaxScaler
from coinbase.rest import RESTClient  # coinbase-advanced-py

# ---------------------------------------------
# 1. KONFIGURATION
# ---------------------------------------------
API_KEY    = os.getenv('COINBASE_API_KEY') or "your_api_key"
API_SECRET = os.getenv('COINBASE_API_SECRET') or "your_api_secret"

PRODUCT_ID    = "BTC-EUR"   # Handelspaar
SEQ_LEN       = 30          # LSTM-Sequenzlänge (Kerzen)
TRADE_AMOUNT  = 10.0        # EUR pro Trade
BUY_THRESHOLD  = 0.502      # Modell-Konfidenz für Kauf
SELL_THRESHOLD = 0.498      # Modell-Konfidenz für Verkauf
LOOP_INTERVAL  = 60         # Sekunden zwischen Zyklen

# ---------------------------------------------
# 2. LSTM-MODELL
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
# 3. DATEN-HELFER
# ---------------------------------------------
def fetch_candles(client: RESTClient, product_id: str, limit: int = 150) -> pd.DataFrame:
    """Holt OHLCV-Daten der letzten `limit` 1-Minuten-Kerzen."""
    end   = int(time.time())
    start = end - limit * 60
    resp  = client.get_candles(
        product_id=product_id,
        start=str(start),
        end=str(end),
        granularity="ONE_MINUTE",
    )
    candles = resp["candles"]
    df = pd.DataFrame(candles, columns=["start", "low", "high", "open", "close", "volume"])
    df = df.sort_values("start").reset_index(drop=True)
    df["close"] = df["close"].astype(float)
    return df


def prepare_sequence(prices: np.ndarray, seq_len: int, scaler: MinMaxScaler):
    """Normalisiert Preise und gibt den letzten Sequenz-Tensor zurück."""
    scaled = scaler.fit_transform(prices.reshape(-1, 1))
    seq    = scaled[-seq_len:]
    tensor = torch.tensor(seq, dtype=torch.float32).unsqueeze(0)  # (1, seq_len, 1)
    return tensor


# ---------------------------------------------
# 4. ORDERFUNKTIONEN
# ---------------------------------------------
def place_market_buy(client: RESTClient, product_id: str, quote_size: float):
    """Kauft für `quote_size` EUR zum Marktpreis."""
    import uuid
    order = client.market_order_buy(
        client_order_id=str(uuid.uuid4()),
        product_id=product_id,
        quote_size=str(quote_size),
    )
    print(f"  ✅ KAUF-Order: {order['order_id']} | {quote_size} EUR")
    return order


def place_market_sell(client: RESTClient, product_id: str, base_size: float):
    """Verkauft `base_size` BTC zum Marktpreis."""
    import uuid
    order = client.market_order_sell(
        client_order_id=str(uuid.uuid4()),
        product_id=product_id,
        base_size=str(round(base_size, 8)),
    )
    print(f"  ✅ VERKAUF-Order: {order['order_id']} | {base_size} BTC")
    return order


def get_btc_balance(client: RESTClient) -> float:
    """Gibt die verfügbare BTC-Balance zurück."""
    accounts = client.get_accounts()["accounts"]
    for acc in accounts:
        if acc["currency"] == "BTC":
            return float(acc["available_balance"]["value"])
    return 0.0


# ---------------------------------------------
# 5. TRAININGS-SCHLEIFE (Schnell-Training)
# ---------------------------------------------
def quick_train(model: PriceLSTM, prices: np.ndarray, seq_len: int,
                scaler: MinMaxScaler, epochs: int = 20):
    """Trainiert das Modell kurzfristig auf aktuellen Marktdaten."""
    scaled = scaler.fit_transform(prices.reshape(-1, 1))
    X, y   = [], []
    for i in range(len(scaled) - seq_len - 1):
        X.append(scaled[i : i + seq_len])
        # Label: 1 wenn nächste Kerze höher, sonst 0
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
            print(f"    Epoch {ep+1}/{epochs} | Loss: {loss.item():.4f}")


# ---------------------------------------------
# 6. HAUPT-BOT-SCHLEIFE
# ---------------------------------------------
def run_bot():
    print("🤖 Coinbase Trading Bot gestartet")
    print(f"   Paar: {PRODUCT_ID} | Sequenz: {SEQ_LEN} | Intervall: {LOOP_INTERVAL}s\n")

    client  = RESTClient(api_key=API_KEY, api_secret=API_SECRET)
    model   = PriceLSTM()
    scaler  = MinMaxScaler()
    cycle   = 0

    while True:
        cycle += 1
        print(f"-- Zyklus {cycle} | {time.strftime('%H:%M:%S')} --------------")

        try:
            # --- Daten holen ---
            df     = fetch_candles(client, PRODUCT_ID)
            prices = df["close"].values
            print(f"  📊 Letzter Preis: {prices[-1]:.2f} EUR")

            # --- Modell trainieren (alle 10 Zyklen neu) ---
            if cycle % 10 == 1:
                print("  🔄 Trainiere Modell...")
                quick_train(model, prices, SEQ_LEN, scaler, epochs=20)

            # --- Vorhersage ---
            model.eval()
            with torch.no_grad():
                seq        = prepare_sequence(prices, SEQ_LEN, scaler)
                confidence = model(seq).item()
            print(f"  🧠 Modell-Konfidenz (Aufwärtsbewegung): {confidence:.4f}")

            # --- Handelsentscheidung ---
            btc_balance = get_btc_balance(client)

            if confidence > BUY_THRESHOLD:
                print("  📈 Signal: KAUFEN")
                place_market_buy(client, PRODUCT_ID, TRADE_AMOUNT)

            elif confidence < SELL_THRESHOLD and btc_balance > 0.00001:
                print("  📉 Signal: VERKAUFEN")
                place_market_sell(client, PRODUCT_ID, btc_balance)

            else:
                print("  ⏸️  Signal: HALTEN")

        except Exception as e:
            print(f"  ❌ Fehler: {e}")

        print(f"  ⏳ Warte {LOOP_INTERVAL}s...\n")
        time.sleep(LOOP_INTERVAL)


# ---------------------------------------------
# 7. EINSTIEGSPUNKT
# ---------------------------------------------
if __name__ == "__main__":
    run_bot()
