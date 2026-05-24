/**
 * detect_lines.js
 * Detects line segments in an image using qjs-opencv (LSD / HoughLinesP),
 * draws them on a copy of the image, displays the result, and saves the
 * segment data to a JSON file.
 *
 * Usage:
 *   qjs detect_lines.js <input_image> [output_image] [output_json]
 *
 * Defaults:
 *   output_image  →  lines_output.png
 *   output_json   →  lines_output.json
 *
 * Dependencies (npm / qjsm packages):
 *   qjs-opencv      – OpenCV bindings for QuickJS
 *   qjs-modules     – provides 'fs' for file I/O
 */

import { Scalar, threshold, THRESH_BINARY_INV, THRESH_BINARY, Mat, imread, imshow, imwrite, waitKey, cvtColor, Canny, GaussianBlur, HoughLinesP, drawLine, LineSegmentDetector, COLOR_BGR2GRAY, LINE_AA, } from 'opencv';
import { writeFileSync } from 'fs';

// ─── helpers ────────────────────────────────────────────────────────────────

function die(msg) {
  console.error('[ERROR]', msg);
  throw new Error(msg);
}

function writeJSON(path, data) {
  const text = JSON.stringify(data, null, 2);
  writeFileSync(path, text);
}

// ─── argument parsing ────────────────────────────────────────────────────────

const args = scriptArgs.slice(1); // scriptArgs[0] is the script name
if(args.length < 1) {
  die('Usage: qjs detect_lines.js <input_image> [output_image] [output_json]');
}

const inputPath = args[0];
const outputImg = args[1] ?? 'lines_output.png';
const outputJSON = args[2] ?? 'lines_output.json';

// ─── load image ──────────────────────────────────────────────────────────────

console.log(`Loading image: ${inputPath}`);
const src = imread(inputPath);

if(src.empty) die(`Could not load image: ${inputPath}`);

const [rows, cols] = [src.rows, src.cols];
console.log(`Image size: ${cols}×${rows}`);

// ─── pre-process ─────────────────────────────────────────────────────────────

const gray = new Mat();
cvtColor(src, gray, COLOR_BGR2GRAY);

const blurred = new Mat();
GaussianBlur(gray, blurred, { width: 5, height: 5 }, 0);

// ─── method 1: LSD (Line Segment Detector) ───────────────────────────────────
// LSD tends to give more accurate, sub-pixel segments.

let segments = [];
let usedLSD = false;

//if(0)
try {
  const lsd = new LineSegmentDetector();
  const lsdLines = new Mat();
  lsd.detect(blurred, lsdLines);

  if(!lsdLines.empty) {
    for(let i = 0; i < lsdLines.rows; i++) {
      const v = lsdLines.at(i, 0); // [x1, y1, x2, y2]
      segments.push({ x1: v[0], y1: v[1], x2: v[2], y2: v[3], method: 'LSD' });
    }
    usedLSD = true;
    console.log(`LSD detected ${segments.length} line segment(s).`);
  }
} catch(e) {
  console.warn('LSD unavailable, falling back to HoughLinesP:', e.message);
}

// ─── method 2: HoughLinesP fallback ──────────────────────────────────────────

if(!usedLSD || segments.length === 0) {
  const edges = new Mat(/*blurred.size, blurred.type*/);

  Canny(blurred, edges, 50, 150);
  //
  //blurred.xor(Scalar(255,255,255,255), edges);
  //threshold(blurred,edges,127, 255, THRESH_BINARY);

  const houghLines = new Mat();
  HoughLinesP(edges, houghLines, 1, Math.PI / 180, 80, 30, 10);

  segments = [];
  for(let i = 0; i < houghLines.rows; i++) {
    const v = houghLines.at(i, 0); // [x1, y1, x2, y2]
    segments.push({ x1: v[0], y1: v[1], x2: v[2], y2: v[3], method: 'HoughLinesP' });
  }
  console.log(`HoughLinesP detected ${segments.length} line segment(s).`);
}

// ─── draw segments ───────────────────────────────────────────────────────────

const canvas = src.clone();

// Colour palette – cycle through a few vivid colours so overlapping lines
// are easier to distinguish visually.
const palette = [
  [0, 255, 0], // green
  [0, 128, 255], // orange-blue
  [255, 0, 0], // blue
  [0, 255, 255], // yellow
  [255, 0, 255], // magenta
  [0, 165, 255], // orange
];

segments.forEach((seg, idx) => {
  const color = palette[idx % palette.length];
  drawLine(canvas, { x: Math.round(seg.x1), y: Math.round(seg.y1) }, { x: Math.round(seg.x2), y: Math.round(seg.y2) }, color, 1, LINE_AA);
});

// ─── display ─────────────────────────────────────────────────────────────────

imshow('Detected Line Segments  (press any key to close)', canvas);
waitKey(0);

// ─── save image ──────────────────────────────────────────────────────────────

imwrite(outputImg, canvas);
console.log(`Annotated image saved to: ${outputImg}`);

// ─── save JSON ───────────────────────────────────────────────────────────────

const report = {
  source: inputPath,
  image_width: cols,
  image_height: rows,
  method: usedLSD ? 'LSD' : 'HoughLinesP',
  segment_count: segments.length,
  segments: segments.map((s, i) => ({
    id: i,
    x1: +s.x1.toFixed(2),
    y1: +s.y1.toFixed(2),
    x2: +s.x2.toFixed(2),
    y2: +s.y2.toFixed(2),
    length: +Math.hypot(s.x2 - s.x1, s.y2 - s.y1).toFixed(2),
    angle_deg: +((Math.atan2(s.y2 - s.y1, s.x2 - s.x1) * 180) / Math.PI).toFixed(2),
    method: s.method,
  })),
};

writeJSON(outputJSON, report);
console.log(`Segment data saved to:  ${outputJSON}`);
console.log('Done.');
