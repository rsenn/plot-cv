#!/usr/bin/env qjs
//
// circuit2circuitjs.js
//   Best-effort conversion of a schematic photo into a circuitjs file.
//
// Usage:
//   qjs circuit2circuitjs.js <photo.png|jpg> [out.circuitjs]
//

import { existsSync, writeFileSync } from 'fs';
import { basename, extname }          from 'path';
import {
  imread, imwrite,
  Mat, Size, Point, Rect, Scalar,
  cvtColor, GaussianBlur, adaptiveThreshold,
  getStructuringElement, morphologyEx,
  findContours, contourArea, boundingRect, arcLength,
  HoughLinesP, drawRect, putText, drawLine,
  COLOR_BGR2GRAY,
  ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV,
  MORPH_RECT, MORPH_CLOSE,
  RETR_EXTERNAL, CHAIN_APPROX_SIMPLE,
  FONT_HERSHEY_SIMPLEX,
} from 'opencv.so';

// ---------- args ----------
const args = scriptArgs.slice(1);
if (args.length < 1) {
  console.log('usage: qjs circuit2circuitjs.js <photo> [out.circuitjs]');
  throw new Error('missing input image');
}
const inputPath  = args[0];
const outputPath = args[1] ||
  basename(inputPath, extname(inputPath)) + '.circuitjs';
const debugPath  = basename(outputPath, '.circuitjs') + '.debug.png';

if (!existsSync(inputPath))
  throw new Error('input not found: ' + inputPath);

console.log(`reading ${inputPath}`);

// ---------- 1. load + preprocess ----------
const src = imread(inputPath);
if (!src || (src.empty && src.empty()))
  throw new Error('imread returned empty Mat');

console.log(`image: ${src.cols} x ${src.rows}`);

const gray    = new Mat();
const blurred = new Mat();
const bin     = new Mat();
const closed  = new Mat();

cvtColor(src, gray, COLOR_BGR2GRAY);
GaussianBlur(gray, blurred, new Size(3, 3), 0);

// Ink (lines, components) → white (255), paper → black (0).
adaptiveThreshold(
  blurred, bin,
  255,
  ADAPTIVE_THRESH_GAUSSIAN_C,
  THRESH_BINARY_INV,
  21,   // block size — tune for image scale
  10    // C
);

// Close 1–2 px gaps in strokes so contours don't fragment.
const k = getStructuringElement(MORPH_RECT, new Size(2, 2));
morphologyEx(bin, closed, MORPH_CLOSE, k);

// ---------- 2. component detection ----------
const findRes = findContours(closed.clone(), RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
const contours = Array.isArray(findRes) ? findRes[0] : findRes;

const imgArea = src.cols * src.rows;
const MIN_AREA = Math.max(200, imgArea * 0.0002);
const MAX_AREA = imgArea * 0.25;

const components = [];
for (const c of contours) {
  const area = contourArea(c);
  if (area < MIN_AREA || area > MAX_AREA) continue;

  const rect      = boundingRect(c);
  const perim     = arcLength(c, true);
  const ar        = rect.width / rect.height;
  const aspect    = ar >= 1 ? ar : 1 / ar;
  const compact   = (perim * perim) / (4 * Math.PI * area); // 1 = circle
  const fillRatio = area / (rect.width * rect.height);

  components.push({
    rect, area, perim, aspect,
    landscape: ar >= 1,
    compactness: compact,
    fillRatio,
    cx: rect.x + rect.width  / 2,
    cy: rect.y + rect.height / 2,
    kind: 'unknown',
  });
}

function classifyComponent(c) {
  // rough heuristics — swap for an ML model for real accuracy
  if (c.fillRatio < 0.08)                                          return 'wire-frag';
  if (c.rect.width < 16 && c.rect.height < 16)                     return 'node';
  if (c.aspect >= 2.5 && c.compactness > 1.4)                      return 'resistor';
  if (c.aspect >= 1.5 && c.aspect < 2.5 && c.compactness < 1.4)    return 'capacitor';
  if (c.aspect < 1.4 && c.rect.width >= 40 && c.rect.height >= 40) return 'ic';
  if (c.aspect >= 2.5 && c.compactness < 1.4)                      return 'capacitor';
  return 'unknown';
}

for (const c of components) c.kind = classifyComponent(c);

const keep = components.filter(
  c => c.kind !== 'wire-frag' && c.kind !== 'node' && c.kind !== 'unknown'
);

console.log(`contours: ${contours.length}  kept components: ${keep.length}`);
for (const c of keep)
  console.log(`  ${c.kind.padEnd(10)} bbox=${c.rect.x},${c.rect.y} ${c.rect.width}x${c.rect.height}`);

// ---------- 3. wire detection (Hough) ----------
const wireMask = closed.clone();
for (const c of keep) {
  const r = c.rect;
  const pad = 4;
  const x = Math.max(0, r.x - pad);
  const y = Math.max(0, r.y - pad);
  const w = Math.min(wireMask.cols - x, r.width  + 2 * pad);
  const h = Math.min(wireMask.rows - y, r.height + 2 * pad);
  wireMask(new Rect(x, y, w, h)).setTo(new Scalar(0));
}

const hough = new Mat();
HoughLinesP(
  wireMask, hough,
  1,                 // rho
  Math.PI / 180,     // theta
  40,                // threshold votes
  Math.max(20, Math.min(src.cols, src.rows) * 0.04), // min len
  10                 // max gap
);

const rawWires = [];
for (const row of hough) {
  if (!row || row.length < 4) continue;
  rawWires.push([row[0] | 0, row[1] | 0, row[2] | 0, row[3] | 0]);
}
console.log(`hough lines: ${rawWires.length}`);

// ---------- 4. snap + dedup ----------
const GRID = 16;
const snap = v => Math.round(v / GRID) * GRID;

function componentEndpoints(c) {
  const { rect, landscape, cx, cy } = c;
  if (landscape) {
    return [
      { x: snap(rect.x),              y: snap(cy) },
      { x: snap(rect.x + rect.width), y: snap(cy) },
    ];
  } else {
    return [
      { x: snap(cx), y: snap(rect.y)               },
      { x: snap(cx), y: snap(rect.y + rect.height) },
    ];
  }
}

// force wires to H/V (closer axis wins)
function orthogonalize([x1, y1, x2, y2]) {
  const dx = Math.abs(x2 - x1), dy = Math.abs(y2 - y1);
  if (dx >= dy) y2 = y1; else x2 = x1;
  return [snap(x1), snap(y1), snap(x2), snap(y2)];
}

const wireSet = new Set();
const wires = [];
for (const w of rawWires) {
  const [x1, y1, x2, y2] = orthogonalize(w);
  if (x1 === x2 && y1 === y2) continue;
  const key = (x1 < x2 || (x1 === x2 && y1 < y2))
    ? `${x1},${y1},${x2},${y2}`
    : `${x2},${y2},${x1},${y1}`;
  if (wireSet.has(key)) continue;
  wireSet.add(key);
  wires.push([x1, y1, x2, y2]);
}

// ---------- 5. emit circuitjs ----------
const out = [];
out.push('$ 1 0.000005 10.20027730826997 50 5 50 5e-11');

for (const c of keep) {
  const [a, b] = componentEndpoints(c);
  if (a.x === b.x && a.y === b.y) continue;
  switch (c.kind) {
    case 'resistor':
      out.push(`r ${a.x} ${a.y} ${b.x} ${b.y} 0 1000`);
      break;
    case 'capacitor':
      out.push(`c ${a.x} ${a.y} ${b.x} ${b.y} 0 0.000001 0`);
      break;
    case 'ic':
      // No pin recovery → drop a placeholder
      out.push(`r ${a.x} ${a.y} ${b.x} ${b.y} 16 1000`);
      break;
  }
}

for (const [x1, y1, x2, y2] of wires) {
  out.push(`w ${x1} ${y1} ${x2} ${y2} 0`);
}

writeFileSync(outputPath, out.join('\n') + '\n');
console.log(`wrote ${outputPath}  (${out.length - 1} elements)`);

// ---------- debug overlay ----------
const overlay = src.clone();
for (const c of keep) {
  const color =
    c.kind === 'resistor'  ? new Scalar(0,   255, 0)   :
    c.kind === 'capacitor' ? new Scalar(255, 200, 0)   :
    c.kind === 'ic'        ? new Scalar(255, 0,   255) :
                             new Scalar(128, 128, 128);
  const r = c.rect;
  drawRect(overlay, new Point(r.x, r.y),
    new Point(r.x + r.width, r.y + r.height), color, 2);
  putText(overlay, c.kind,
    new Point(r.x, Math.max(12, r.y - 4)),
    FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
}
for (const [x1, y1, x2, y2] of wires) {
  drawLine(overlay, new Point(x1, y1), new Point(x2, y2),
    new Scalar(0, 0, 255), 2);
}
imwrite(debugPath, overlay);
console.log(`wrote ${debugPath}  (visual check)`);

