"""
╔══════════════════════════════════════════════════════════╗
║         YOLO + OpenCV  –  Foto Kategorisierung Demo      ║
╚══════════════════════════════════════════════════════════╝

Abhängigkeiten installieren:
    pip install ultralytics opencv-python-headless Pillow

Verwendung:
    python photo_categorizer.py                 # Demo-Modus (generierte Testbilder)
    python photo_categorizer.py --input ./fotos # Eigene Bilder
    python photo_categorizer.py --webcam        # Live-Webcam
"""

import argparse
import sys
import time
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

# ── Farben für Terminal-Ausgabe ────────────────────────────────────────────────
RESET  = "\033[0m"
BOLD   = "\033[1m"
GREEN  = "\033[92m"
YELLOW = "\033[93m"
CYAN   = "\033[96m"
RED    = "\033[91m"
GRAY   = "\033[90m"

# ── YOLO-Klassen → übergeordnete Kategorie ────────────────────────────────────
CATEGORY_MAP = {
    "Personen":  {"person"},
    "Fahrzeuge": {"car", "truck", "bus", "motorcycle", "bicycle", "boat",
                  "train", "airplane"},
    "Tiere":     {"cat", "dog", "horse", "sheep", "cow", "elephant", "bear",
                  "zebra", "giraffe", "bird"},
    "Elektronik":{"laptop", "cell phone", "tv", "keyboard", "mouse",
                  "remote", "microwave"},
    "Sport":     {"sports ball", "tennis racket", "baseball bat", "skateboard",
                  "surfboard", "skis", "snowboard"},
    "Essen":     {"banana", "apple", "sandwich", "orange", "broccoli",
                  "carrot", "hot dog", "pizza", "donut", "cake"},
    "Möbel":     {"chair", "couch", "bed", "dining table", "toilet"},
    "Sonstiges": set(),   # Fallback
}

# Invertiertes Lookup: label → kategorie
def build_lookup():
    lookup = {}
    for cat, labels in CATEGORY_MAP.items():
        for lbl in labels:
            lookup[lbl] = cat
    return lookup

LABEL_TO_CAT = build_lookup()


def label_to_category(label: str) -> str:
    return LABEL_TO_CAT.get(label.lower(), "Sonstiges")


# ── Farbkodierung pro Kategorie (BGR für OpenCV) ──────────────────────────────
CATEGORY_COLORS = {
    "Personen":   (255, 100,  50),
    "Fahrzeuge":  ( 50, 200, 255),
    "Tiere":      ( 50, 230, 100),
    "Elektronik": (200,  50, 255),
    "Sport":      (  0, 165, 255),
    "Essen":      ( 50, 255, 200),
    "Möbel":      (180, 130, 255),
    "Sonstiges":  (160, 160, 160),
}


# ══════════════════════════════════════════════════════════════════════════════
#  Demo-Bilder generieren (ohne echte Fotos)
# ══════════════════════════════════════════════════════════════════════════════

def create_demo_images(out_dir: Path) -> list[Path]:
    """Erzeugt synthetische Testbilder mit einfachen Formen."""
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []

    configs = [
        ("strasse.jpg",  [(50, 50, 200, 150, "car"),
                          (220, 80, 370, 170, "truck"),
                          (100, 200, 160, 310, "person")]),
        ("park.jpg",     [(60,  60, 200, 220, "dog"),
                          (250, 50, 380, 190, "person"),
                          (100, 230, 200, 310, "bird")]),
        ("wohnzimmer.jpg",[(30, 80, 250, 250, "couch"),
                           (270, 100, 400, 230, "tv"),
                           (160, 270, 260, 360, "cat")]),
    ]

    for fname, objects in configs:
        img = np.ones((400, 460, 3), dtype=np.uint8) * 235
        # Hintergrundgradient
        for y in range(400):
            img[y] = np.clip(img[y] - y // 6, 100, 235)

        for x1, y1, x2, y2, label in objects:
            cat = label_to_category(label)
            color = CATEGORY_COLORS.get(cat, (128, 128, 128))
            cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 2)
            cv2.putText(img, label, (x1 + 5, y1 + 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)

        p = out_dir / fname
        cv2.imwrite(str(p), img)
        paths.append(p)
        print(f"  {GRAY}Demo-Bild erstellt: {p}{RESET}")

    return paths


# ══════════════════════════════════════════════════════════════════════════════
#  Kern-Logik: Bild analysieren
# ══════════════════════════════════════════════════════════════════════════════

def analyze_image_yolo(model, image_path: Path, conf_thresh: float = 0.35):
    """Führt YOLO-Inferenz aus und gibt Detektionen zurück."""
    results = model(str(image_path), conf=conf_thresh, verbose=False)
    detections = []
    for r in results:
        for box in r.boxes:
            label   = model.names[int(box.cls)]
            conf    = float(box.conf)
            xyxy    = [int(v) for v in box.xyxy[0]]
            category = label_to_category(label)
            detections.append({
                "label":    label,
                "category": category,
                "conf":     conf,
                "bbox":     xyxy,
            })
    return detections


def analyze_image_demo(image_path: Path, conf_thresh: float = 0.35):
    """
    Fallback ohne echtes YOLO-Modell:
    Liest die im Demo-Bild eingebetteten Label-Texte und simuliert Detektionen.
    """
    img = cv2.imread(str(image_path))
    if img is None:
        return []

    # Simulierte Objekte anhand des Dateinamens
    presets = {
        "strasse":     [("car", 0.91), ("truck", 0.87), ("person", 0.78)],
        "park":        [("dog",  0.93), ("person", pip install ultralytics opencv-python-pip install ultralytics opencv-python-pip install ultralytics opencv-python-headless PilloWheadless PilloWheadless PilloW0.82), ("bird", 0.61)],
        "wohnzimmer":  [("couch", 0.95), ("tv",   0.89), ("cat",  0.77)],
    }
    stem = image_path.stem.lower()
    items = presets.get(stem, [("person", 0.70)])

    h, w = img.shape[:2]
    detections = []
    for i, (label, conf) in enumerate(items):
        if conf < conf_thresh:
            continue
        # Grobe simulierte Bounding-Box
        x1 = (i * w // len(items))
        x2 = x1 + w // (len(items) + 1)
        y1, y2 = 50, h - 50
        detections.append({
            "label":    label,
            "category": label_to_category(label),
            "conf":     conf,
            "bbox":     [x1, y1, x2, y2],
        })
    return detections


def draw_detections(image_path: Path, detections: list) -> np.ndarray:
    """Zeichnet Bounding Boxes und Labels ins Bild."""
    img = cv2.imread(str(image_path))
    if img is None:
        return np.zeros((100, 100, 3), dtype=np.uint8)

    for d in detections:
        x1, y1, x2, y2 = d["bbox"]
        color = CATEGORY_COLORS.get(d["category"], (128, 128, 128))
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 3)

        tag = f"{d['label']} {d['conf']:.0%}"
        (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        cv2.rectangle(img, (x1, y1 - th - 10), (x1 + tw + 8, y1), color, -1)
        cv2.putText(img, tag, (x1 + 4, y1 - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

    return img


# ══════════════════════════════════════════════════════════════════════════════
#  Ausgabe & Reporting
# ══════════════════════════════════════════════════════════════════════════════

def print_result(image_path: Path, detections: list):
    cats = defaultdict(list)
    for d in detections:
        cats[d["category"]].append(d)

    print(f"\n  {BOLD}{CYAN}📷  {image_path.name}{RESET}")
    if not detections:
        print(f"    {GRAY}– Keine Objekte erkannt –{RESET}")
        return

    for cat, items in sorted(cats.items()):
        color_code = {
            "Personen": GREEN, "Fahrzeuge": CYAN, "Tiere": YELLOW,
            "Elektronik": "\033[95m", "Sonstiges": GRAY,
        }.get(cat, RESET)
        label_list = ", ".join(
            f"{d['label']} ({d['conf']:.0%})" for d in items
        )
        print(f"    {color_code}{BOLD}[{cat}]{RESET}  {label_list}")


def print_summary(stats: dict):
    print(f"\n{BOLD}{'═'*54}{RESET}")
    print(f"{BOLD}  📊  Zusammenfassung{RESET}")
    print(f"{'═'*cv}")
    total = sum(sum(len(v) for v in cats.values()) for cats in stats.values())
    cat_totals = defaultdict(int)
    for cats in stats.values():
        for cat, items in cats.items():
            cat_totals[cat] += len(items)

    for cat, count in sorted(cat_totals.items(), key=lambda x: -x[1]):
        bar = "█" * count + "░" * max(0, 10 - count)
        print(f"  {BOLD}{cat:<12}{RESET} {bar}  {count} Objekte")
    print(f"\n  {GREEN}✓ {len(stats)} Bilder analysiert, {total} Objekte erkannt.{RESET}\n")


# ══════════════════════════════════════════════════════════════════════════════
#  Webcam-Modus
# ══════════════════════════════════════════════════════════════════════════════

def run_webcam(model, conf_thresh=0.40):
    print(f"\n{YELLOW}Webcam-Modus gestartet – {BOLD}[Q]{RESET}{YELLOW} zum Beenden{RESET}\n")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print(f"{RED}Fehler: Webcam nicht gefunden.{RESET}")
        return

    fps_time = time.time()
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # YOLO-Inferenz
        results = model(frame, conf=conf_thresh, verbose=False)
        for r in results:
            for box in r.boxes:
                label    = model.names[int(box.cls)]
                conf     = float(box.conf)
                category = label_to_category(label)
                color    = CATEGORY_COLORS.get(category, (128, 128, 128))
                x1, y1, x2, y2 = [int(v) for v in box.xyxy[0]]
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                tag = f"{label} {conf:.0%}"
                cv2.putText(frame, tag, (x1, y1 - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

        # FPS
        fps = 1 / max(time.time() - fps_time, 1e-5)
        fps_time = time.time()
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 120), 2)

        cv2.imshow("YOLO Kategorisierung – [Q] beenden", frame)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


# ══════════════════════════════════════════════════════════════════════════════
#  Einstiegspunkt
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description="Foto-Kategorisierung mit YOLOv8 + OpenCV"
    )
    parser.add_argument("--input",   type=str,  help="Pfad zu Bilderordner oder Datei")
    parser.add_argument("--webcam",  action="store_true", help="Live-Webcam-Modus")
    parser.add_argument("--conf",    type=float, default=0.35, help="Konfidenz-Schwelle (0–1)")
    parser.add_argument("--save",    action="store_true", help="Annotierte Bilder speichern")
    parser.add_argument("--demo",    action="store_true", help="Ohne echtes YOLO-Modell (Simulation)")
    args = parser.parse_args()

    print(f"""
{BOLD}{CYAN}╔══════════════════════════════════════════════════════╗
║     YOLOv8 + OpenCV  –  Foto Kategorisierung         ║
╚══════════════════════════════════════════════════════╝{RESET}
""")

    # ── Modell laden ─────────────────────────────────────────────────────────
    model = None
    demo_mode = args.demo

    if not demo_mode:
        try:
            from ultralytics import YOLO
            print(f"  {GRAY}Lade YOLOv8n (wird beim ersten Start heruntergeladen) …{RESET}")
            model = YOLO("yolov8n.pt")
            print(f"  {GREEN}✓ Modell geladen.{RESET}\n")
        except ImportError:
            print(f"  {YELLOW}ultralytics nicht installiert – Simulationsmodus aktiv.{RESET}")
            print(f"  {GRAY}  → pip install ultralytics{RESET}\n")
            demo_mode = True

    # ── Webcam ────────────────────────────────────────────────────────────────
    if args.webcam:
        if demo_mode:
            print(f"{RED}Webcam-Modus benötigt ein echtes YOLO-Modell.{RESET}")
        else:
            run_webcam(model, conf_thresh=args.conf)
        return

    # ── Bilddateien sammeln ───────────────────────────────────────────────────
    EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}

    if args.input:
        p = Path(args.input)
        if p.is_dir():
            image_paths = sorted(f for f in p.iterdir() if f.suffix.lower() in EXTS)
        elif p.is_file():
            image_paths = [p]
        else:
            print(f"{RED}Pfad nicht gefunden: {p}{RESET}")
            sys.exit(1)
    else:
        print(f"  {YELLOW}Kein --input angegeben → Demo-Bilder werden generiert.{RESET}\n")
        demo_dir = Path("demo_images")
        image_paths = create_demo_images(demo_dir)
        demo_mode = True   # Simulierte Detektionen für synthetische Bilder

    if not image_paths:
        print(f"{RED}Keine Bilder gefunden.{RESET}")
        sys.exit(1)

    print(f"  {GREEN}Analysiere {len(image_paths)} Bild(er) …{RESET}\n")
    out_dir = Path("output_annotated")
    if args.save:
        out_dir.mkdir(exist_ok=True)

    stats = {}

    for img_path in image_paths:
        t0 = time.perf_counter()

        if demo_mode:
            detections = analyze_image_demo(img_path, conf_thresh=args.conf)
        else:
            detections = analyze_image_yolo(model, img_path, conf_thresh=args.conf)

        elapsed = (time.perf_counter() - t0) * 1000
        print_result(img_path, detections)
        print(f"    {GRAY}⏱  {elapsed:.1f} ms{RESET}")

        # Kategorien für Zusammenfassung
        cats = defaultdict(list)
        for d in detections:
            cats[d["category"]].append(d)
        stats[img_path.name] = cats

        # Annotiertes Bild speichern
        if args.save:
            annotated = draw_detections(img_path, detections)
            save_path = out_dir / img_path.name
            cv2.imwrite(str(save_path), annotated)

        # Bild anzeigen (2 Sekunden)
        annotated = draw_detections(img_path, detections)
        cv2.imshow(f"Kategorisierung: {img_path.name}", annotated)
        cv2.waitKey(2000)
        cv2.destroyAllWindows()

    print_summary(stats)

    if args.save:
        print(f"  {CYAN}Annotierte Bilder gespeichert in: {out_dir}/{RESET}\n")


if __name__ == "__main__":
    main()


