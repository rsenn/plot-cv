/**
 * photo_categorizer.js
 * Fotokategorisierung mit YOLO + OpenCV via QuickJS
 *
 * Voraussetzungen:
 *   - QuickJS (qjs) >= 0.7.x
 *   - qjs-opencv Bindings: https://github.com/pbosetti/qjs-opencv
 *   - YOLOv8n ONNX-Modell: https://github.com/ultralytics/assets/releases
 *
 * Ausführen:
 *   qjs --std -I ./modules photo_categorizer.js -- ./bilder ./ausgabe
 *
 * Ordnerstruktur nach der Kategorisierung:
 *   ausgabe/
 *     Personen/
 *     Fahrzeuge/
 *     Tiere/
 *     Natur/
 *     Lebensmittel/
 *     Sonstiges/
 */
 
import * as std from "std";
import * as os  from "os";
import {FILLED, FONT_HERSHEY_SIMPLEX, Point, Rect, Scalar, Size, dnn, getTextSize, imread, imwrite,  putText,drawRect as rectangle
}  from "opencv";
 
// ─────────────────────────────────────────────
// KONFIGURATION
// ─────────────────────────────────────────────
const CONFIG = {
  modelPath:     "./models/yolov8n.onnx",
  classesPath:   "./models/coco_classes.txt",
  inputSize:     640,       // YOLO-Eingabegröße (px)
  confThreshold: 0.40,      // Mindest-Konfidenz
  nmsThreshold:  0.45,      // NMS-Schwellwert
  supportedExts: [".jpg", ".jpeg", ".png", ".bmp", ".webp"],
  verbose:       true,
};
 
// COCO-Klassen → Kategorien
const CATEGORY_MAP = {
  // Personen
  person:       "Personen",
 
  // Fahrzeuge
  bicycle:      "Fahrzeuge", car:         "Fahrzeuge",
  motorcycle:   "Fahrzeuge", airplane:    "Fahrzeuge",
  bus:          "Fahrzeuge", train:       "Fahrzeuge",
  truck:        "Fahrzeuge", boat:        "Fahrzeuge",
 
  // Tiere
  bird:         "Tiere",    cat:          "Tiere",
  dog:          "Tiere",    horse:        "Tiere",
  sheep:        "Tiere",    cow:          "Tiere",
  elephant:     "Tiere",    bear:         "Tiere",
  zebra:        "Tiere",    giraffe:      "Tiere",
 
  // Natur / Outdoor
  "potted plant": "Natur",
 
  // Lebensmittel
  banana:       "Lebensmittel", apple:    "Lebensmittel",
  sandwich:     "Lebensmittel", orange:   "Lebensmittel",
  broccoli:     "Lebensmittel", carrot:   "Lebensmittel",
  "hot dog":    "Lebensmittel", pizza:    "Lebensmittel",
  donut:        "Lebensmittel", cake:     "Lebensmittel",
  "wine glass": "Lebensmittel", cup:      "Lebensmittel",
  fork:         "Lebensmittel", knife:    "Lebensmittel",
  spoon:        "Lebensmittel", bowl:     "Lebensmittel",
};
 
// ─────────────────────────────────────────────
// HILFSFUNKTIONEN
// ─────────────────────────────────────────────
 
/** Gibt eine formatierte Log-Zeile aus */
function log(level, msg) {
  const prefix = { info: "ℹ️ ", ok: "✅", warn: "⚠️ ", err: "❌" };
  std.err.puts(`${prefix[level] ?? "  "} ${msg}\n`);
}
 
/** Liest alle Zeilen einer Textdatei in ein Array */
function readLines(path) {
  const f = std.open(path, "r");
  if (!f) throw new Error(`Datei nicht gefunden: ${path}`);
  const lines = [];
  let line;
  while ((line = f.getline()) !== null) {
    const t = line.trim();
    if (t.length > 0) lines.push(t);
  }
  f.close();
  return lines;
}
 
/** Prüft ob ein Pfad ein Verzeichnis ist */
function isDir(path) {
  const [st, err] = os.stat(path);
  return err === 0 && (st.mode & 0o170000) === 0o040000;
}
 
/** Legt ein Verzeichnis an (inkl. Elternverzeichnisse) */
function mkdirp(path) {
  const parts = path.split("/");
  let current = "";
  for (const part of parts) {
    current += (current ? "/" : "") + part;
    if (current && !isDir(current)) {
      const err = os.mkdir(current, 0o755);
      if (err !== 0 && err !== -17 /* EEXIST */) {
        throw new Error(`mkdir fehlgeschlagen für: ${current} (errno ${err})`);
      }
    }
  }
}
 
/** Gibt die Dateiendung in Kleinbuchstaben zurück */
function extname(filename) {
  const dot = filename.lastIndexOf(".");
  return dot >= 0 ? filename.slice(dot).toLowerCase() : "";
}
 
/** Kopiert eine Datei (Fallback für os.rename bei Datei-System-Grenzen) */
function copyFile(src, dst) {
  const inFile  = std.open(src, "rb");
  const outFile = std.open(dst, "wb");
  if (!inFile || !outFile) throw new Error(`Kopieren fehlgeschlagen: ${src} → ${dst}`);
  const CHUNK = 1024 * 64;
  while (true) {
    const buf = inFile.readAsArrayBuffer(CHUNK);
    if (!buf || buf.byteLength === 0) break;
    outFile.write(buf, 0, buf.byteLength);
  }
  inFile.close();
  outFile.close();
}
 
// ─────────────────────────────────────────────
// YOLO-INFERENZ
// ─────────────────────────────────────────────
 
/** Lädt das ONNX-Modell via OpenCV DNN */
function loadModel(modelPath) {
  if (CONFIG.verbose) log("info", `Lade Modell: ${modelPath}`);
  const net = dnn.readNetFromONNX(modelPath);
  if (!net || net.empty) throw new Error("Modell konnte nicht geladen werden.");
 
  // CPU bevorzugen; bei CUDA-Support: dnn.DNN_BACKEND_CUDA
  net.setPreferableBackend(dnn.DNN_BACKEND_OPENCV);
  net.setPreferableTarget(dnn.DNN_TARGET_CPU);
  if (CONFIG.verbose) log("ok", "Modell geladen (CPU).");
  return net;
}
 
/**
 * Führt YOLOv8-Inferenz durch und liefert erkannte Klassenbezeichnungen.
 * Gibt ein Array von { label, confidence, box } zurück.
 */
function detectObjects(net, classNames, imagePath) {
  const img = imread(imagePath);
  if (!img || img.empty) {
    throw new Error(`Bild konnte nicht gelesen werden: ${imagePath}`);
  }
 
  const { inputSize, confThreshold, nmsThreshold } = CONFIG;
 
  // Vorverarbeitung: Blob aus Bild erzeugen
  const blob = dnn.blobFromImage(
    img,
    1 / 255.0,                             // Skalierungsfaktor
    new Size(inputSize, inputSize),      // Zielgröße
    new Scalar(0, 0, 0),               // Mittelwert-Subtraktion
    true,                                  // swapRB
    false                                  // crop
  );
 
  net.setInput(blob);
  const output = net.forward(); // Shape: [1, 84, 8400] für YOLOv8n (COCO)
 
  // YOLOv8-Output-Format parsen: [1, num_classes+4, anchors]
  // Transponieren zu [anchors, num_classes+4]
  const data    = output.reshape(1, output.size[1]);
  const anchors = output.size[2];
  const numCls  = classNames.length;
 
  const boxes       = [];
  const scores      = [];
  const classIds    = [];
 
  const origW = img.cols;
  const origH = img.rows;
  const scaleX = origW / inputSize;
  const scaleY = origH / inputSize;
 
  for (let i = 0; i < anchors; i++) {
    // Klassenwahrscheinlichkeiten (Offset 4..84)
    let maxConf = 0;
    let maxIdx  = -1;
    for (let c = 0; c < numCls; c++) {
      const val = data.at(4 + c, i);
      if (val > maxConf) { maxConf = val; maxIdx = c; }
    }
 
    if (maxConf < confThreshold) continue;
 
    // Box-Koordinaten (cx, cy, w, h) → (x1, y1, w, h)
    const cx = data.at(0, i) * scaleX;
    const cy = data.at(1, i) * scaleY;
    const bw = data.at(2, i) * scaleX;
    const bh = data.at(3, i) * scaleY;
 
    boxes.push(new Rect(
      Math.round(cx - bw / 2),
      Math.round(cy - bh / 2),
      Math.round(bw),
      Math.round(bh)
    ));
    scores.push(maxConf);
    classIds.push(maxIdx);
  }
 
  // Non-Maximum Suppression
  const indices = dnn.NMSBoxes(boxes, scores, confThreshold, nmsThreshold);
 
  const detections = [];
  for (const idx of indices) {
    detections.push({
      label:      classNames[classIds[idx]],
      confidence: scores[idx],
      box:        boxes[idx],
    });
  }
 
  img.delete(); blob.delete(); data.delete(); output.delete();
  return detections;
}
 
// ─────────────────────────────────────────────
// KATEGORISIERUNG
// ─────────────────────────────────────────────
 
/**
 * Bestimmt die Kategorie eines Bildes anhand der YOLO-Erkennungen.
 * Wählt die Klasse mit der höchsten Konfidenz und mappt sie.
 */
function categorize(detections) {
  if (detections.length === 0) return "Sonstiges";
 
  // Nach Konfidenz absteigend sortieren
  detections.sort((a, b) => b.confidence - a.confidence);
 
  for (const det of detections) {
    const cat = CATEGORY_MAP[det.label];
    if (cat) return cat;
  }
  return "Sonstiges";
}
 
// ─────────────────────────────────────────────
// ANNOTIERTES VORSCHAUBILD (optional)
// ─────────────────────────────────────────────
 
/**
 * Zeichnet Bounding Boxes und Beschriftungen ins Bild und
 * speichert es als <name>_annotated.jpg im Ausgabeordner.
 */
function saveAnnotated(imagePath, detections, outDir) {
  const img = imread(imagePath);
  if (!img || img.empty) return;
 
  for (const { label, confidence, box } of detections) {
    // Rahmen zeichnen
    rectangle(img, box, new Scalar(0, 200, 0), 2);
 
    // Beschriftung
    const text = `${label} ${(confidence * 100).toFixed(0)}%`;
    let baseLine;
    const sz = getTextSize(text, FONT_HERSHEY_SIMPLEX, 0.55, 1, v=>baseLine=v);
    rectangle(img,
      new Point(box.x, box.y - sz.height - 6),
      new Point(box.x + sz.width, box.y),
      new Scalar(0, 200, 0), FILLED
    );
    putText(img, text,
      new Point(box.x, box.y - 4),
      FONT_HERSHEY_SIMPLEX, 0.55,
      new Scalar(0, 0, 0), 1
    );
  }
 
  const basename = imagePath.split("/").at(-1).replace(/\.[^.]+$/, "");
  const outPath  = `${outDir}/${basename}_annotated.jpg`;
  imwrite(outPath, img);
  img.delete();
  return outPath;
}
 
// ─────────────────────────────────────────────
// HAUPTPROGRAMM
// ─────────────────────────────────────────────
 
function main() {
  // Argumente parsen: qjs photo_categorizer.js -- <eingabe> <ausgabe> [--annotate]
  const args       = scriptArgs.slice(1);
  const inputDir   = args[0]  ?? "./bilder";
  const outputDir  = args[1]  ?? "./ausgabe";
  const annotate   = args.includes("--annotate");
 
  if (!isDir(inputDir)) {
    log("err", `Eingabeverzeichnis nicht gefunden: ${inputDir}`);
    std.exit(1);
  }
 
  // Ausgabeordner + Kategorie-Unterordner anlegen
  const categories = [...new Set(Object.values(CATEGORY_MAP)), "Sonstiges"];
  mkdirp(outputDir);
  for (const cat of categories) mkdirp(`${outputDir}/${cat}`);
 
  // Modell + Klassen laden
  let net, classNames;
  try {
    classNames = readLines(CONFIG.classesPath);
    net        = loadModel(CONFIG.modelPath);
  } catch (e) {
    log("err", e.message);
    std.exit(1);
  }
 
  // Alle Bilder im Eingabeverzeichnis einlesen
  const [entries, dirErr] = os.readdir(inputDir);
  if (dirErr !== 0) { log("err", `Kann Verzeichnis nicht lesen: ${inputDir}`); std.exit(1); }
 
  const images = entries.filter(f => CONFIG.supportedExts.includes(extname(f)));
  if (images.length === 0) {
    log("warn", "Keine unterstützten Bilder gefunden.");
    std.exit(0);
  }
 
  log("info", `${images.length} Bilder gefunden. Starte Analyse …\n`);
 
  // Statistik
  const stats = Object.fromEntries(categories.map(c => [c, 0]));
  let errors = 0;
 
  for (const filename of images) {
    const srcPath = `${inputDir}/${filename}`;
    let detections = [];
 
    try {
      detections = detectObjects(net, classNames, srcPath);
    } catch (e) {
      log("err", `  ${filename}: ${e.message}`);
      errors++;
      continue;
    }
 
    const category = categorize(detections);
    const dstPath  = `${outputDir}/${category}/${filename}`;
 
    // Datei in Zielordner kopieren
    try {
      copyFile(srcPath, dstPath);
    } catch (e) {
      log("err", `  Kopieren fehlgeschlagen: ${filename} → ${e.message}`);
      errors++;
      continue;
    }
 
    stats[category]++;
 
    if (CONFIG.verbose) {
      const topDets = detections.slice(0, 3)
        .map(d => `${d.label}(${(d.confidence * 100).toFixed(0)}%)`)
        .join(", ") || "–";
      log("ok", `  ${filename.padEnd(40)} → ${category.padEnd(15)}  [${topDets}]`);
    }
 
    // Annotiertes Vorschaubild speichern
    if (annotate && detections.length > 0) {
      saveAnnotated(srcPath, detections, `${outputDir}/${category}`);
    }
  }
 
  // Zusammenfassung
  std.err.puts("\n────────────────────────────────────────\n");
  std.err.puts("📊 Zusammenfassung\n");
  std.err.puts("────────────────────────────────────────\n");
  for (const [cat, count] of Object.entries(stats)) {
    if (count > 0) std.err.puts(`   ${cat.padEnd(20)} ${count} Bild(er)\n`);
  }
  std.err.puts(`   ${"Fehler".padEnd(20)} ${errors}\n`);
  std.err.puts("────────────────────────────────────────\n");
  std.err.puts(`✅ Fertig! Ergebnisse in: ${outputDir}\n`);
 
  // Cleanup
  net.delete?.();
}
 
main();
 
