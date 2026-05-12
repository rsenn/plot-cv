/**
 * detect_sky.js — Sky region detection using QuickJS + qjs-opencv
 *
 * Usage:
 *   qjs detect_sky.js <input_image> [output_image]
 *
 * Outputs:
 *   - A debug image overlaying the sky mask + largest contour on the original
 *   - Prints bounding rect, contour area, and centroid to stdout
 *
 * Strategy:
 *   1. Convert to HSV colour space.
 *   2. Threshold for blue/grey sky hues — two passes cover clear & overcast skies.
 *   3. Morphologically clean the mask (close gaps, remove noise).
 *   4. Keep only the connected component that touches the TOP edge of the frame
 *      (sky is almost always at the top).
 *   5. Find and draw the external contour of that component.
 */

import { CC_STAT_AREA, CC_STAT_HEIGHT, CC_STAT_LEFT, CC_STAT_TOP, CC_STAT_WIDTH, CCL_DEFAULT, CHAIN_APPROX_SIMPLE, COLOR_BGR2HSV, COLOR_GRAY2BGR, CV_8UC1, CV_32S, MORPH_CLOSE, MORPH_ELLIPSE, MORPH_OPEN, Mat, RETR_EXTERNAL, Scalar, Size, bitwise_or, connectedComponentsWithStats, cvtColor, drawContours, findContours, getStructuringElement, imread, imwrite, inRange, morphologyEx, } from 'opencv';

// ─── CLI args ────────────────────────────────────────────────────────────────
const args = scriptArgs.slice(1); // scriptArgs[0] is the script name
if(args.length < 1) {
  print('Usage: qjs detect_sky.js <input_image> [output_image]');
  std.exit(1);
}
const inputPath = args[0];
const outputPath = args[1] ?? inputPath.replace(/(\.\w+)$/, '_sky$1');

// ─── Load source image ────────────────────────────────────────────────────────
const src = imread(inputPath);
if(src.empty) {
  print(`Error: could not load "${inputPath}"`);
  std.exit(1);
}
print(`Loaded: ${inputPath}  (${src.cols}×${src.rows})`);

// ─── Working matrices ─────────────────────────────────────────────────────────
const hsv = new Mat();
const maskBlue = new Mat(); // clear / blue sky
const maskGrey = new Mat(); // overcast / white-grey sky
const combined = new Mat();
const cleaned = new Mat();

// ─── 1. HSV conversion ───────────────────────────────────────────────────────
cvtColor(src, hsv, COLOR_BGR2HSV);

// ─── 2. Colour thresholds ─────────────────────────────────────────────────────
//
//  Blue sky: H ≈ 90–130°  (OpenCV: 45–65 in [0,180]),  S > 30,  V > 50
//  Overcast / white-grey sky: any H, S < 35,  V > 160
//
inRange(
  hsv,
  Scalar(45, 30, 50), // lower blue
  Scalar(130, 255, 255), // upper blue
  maskBlue,
);

inRange(
  hsv,
  Scalar(0, 0, 160), // lower grey-white
  Scalar(180, 35, 255), // upper grey-white
  maskGrey,
);

bitwise_or(maskBlue, maskGrey, combined);

// ─── 3. Morphological clean-up ───────────────────────────────────────────────
//   Close  → fill small holes inside sky regions
//   Open   → remove isolated noise blobs outside
const kernel = getStructuringElement(MORPH_ELLIPSE, new Size(15, 15));

morphologyEx(combined, cleaned, MORPH_CLOSE, kernel);
morphologyEx(cleaned, cleaned, MORPH_OPEN, kernel);

// ─── 4. Keep only the top-touching component ─────────────────────────────────
//
//  Connected-components labelling; retain any label whose bounding box
//  starts at row 0 (or within a small tolerance).
//
const labels = new Mat();
const stats = new Mat();
const centroids = new Mat();
const numLabels = connectedComponentsWithStats(cleaned, labels, stats, centroids, 8, CV_32S, CCL_DEFAULT);

const TOP_TOLERANCE = Math.round(src.rows * 0.05); // 5 % of image height
const skyMask = Mat.zeros(src.rows, src.cols, CV_8UC1);

let bestLabel = -1;
let bestArea = 0;

for(let lbl = 1; lbl < numLabels; lbl++) {
  // 0 = background

  /*console.log('stats.type', stats.type,CV_32S);
  console.log('stats.size', stats.size);
  console.log('stats', stats);*/
  const top = stats.array[lbl][CC_STAT_TOP];
  const area = stats.array[lbl][CC_STAT_AREA];

  if(top <= TOP_TOLERANCE && area > bestArea) {
    bestArea = area;
    bestLabel = lbl;
  }
}

if(bestLabel === -1) {
  print('Warning: no sky-like region touching the top edge was found.');
  print('Falling back to largest sky-coloured region overall.');

  // Fall back: just pick the largest non-background label
  for(let lbl = 1; lbl < numLabels; lbl++) {
    const area = stats.array[lbl][CC_STAT_AREA];
    if(area > bestArea) {
      bestArea = area;
      bestLabel = lbl;
    }
  }
}

if(bestLabel !== -1) {
  // Build a single-component mask
  for(let row = 0; row < labels.rows; row++) {
    for(let col = 0; col < labels.cols; col++) {
      if(labels.array[row][col] === bestLabel) {
        skyMask.array[row][col] = 255;
        //skyMask.ucharPtr(row, col)[0] = 255;
      }
    }
  }
}

// ─── 5. Find contours ────────────────────────────────────────────────────────
const contours = [];
const hierarchy = new Mat();
findContours(skyMask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

// ─── 6. Annotate & save output ───────────────────────────────────────────────
const output = src.clone();

// Semi-transparent sky fill (blue tint)
const coloured = new Mat();
cvtColor(skyMask, coloured, COLOR_GRAY2BGR);
// Tint masked pixels cyan-blue
for(let row = 0; row < output.rows; row++) {
  for(let col = 0; col < output.cols; col++) {
    if(skyMask.array[row][col] === 255) {
      const px = output.array[row][col];
      px[0] = Math.min(255, px[0] * 0.4 + 100); // B
      px[1] = Math.min(255, px[1] * 0.6 + 40); // G
      px[2] = Math.min(255, px[2] * 0.4); // R
    }
  }
}

// Draw all external contours in bright green
drawContours(output, contours, -1, Scalar(0, 255, 80, 255), 3);

imwrite(outputPath, output);
print(`Output written: ${outputPath}`);

// ─── 7. Report statistics ────────────────────────────────────────────────────
if(bestLabel !== -1) {
  const x = stats.array[bestLabel][CC_STAT_LEFT];
  const y = stats.array[bestLabel][CC_STAT_TOP];
  const w = stats.array[bestLabel][CC_STAT_WIDTH];
  const h = stats.array[bestLabel][CC_STAT_HEIGHT];
  const cx = centroids.array[bestLabel][0].toFixed(1);
  const cy = centroids.array[bestLabel][1].toFixed(1);
  const pct = ((bestArea / (src.rows * src.cols)) * 100).toFixed(1);

  print('─────────────────────────────────');
  print(`Sky region found:`);
  print(`  Bounding rect : x=${x}, y=${y}, w=${w}, h=${h}`);
  print(`  Area          : ${bestArea} px  (${pct}% of frame)`);
  print(`  Centroid      : (${cx}, ${cy})`);
  print(`  Contour count : ${contours.length}`);
  print('─────────────────────────────────');
} else {
  print('No sky region detected.');
}

// ─── Cleanup ─────────────────────────────────────────────────────────────────
src.delete();
hsv.delete();
maskBlue.delete();
maskGrey.delete();
combined.delete();
cleaned.delete();
kernel.delete();
labels.delete();
stats.delete();
centroids.delete();
skyMask.delete();
coloured.delete();
contours.splice(0, contours.length);
hierarchy.delete();
output.delete();
