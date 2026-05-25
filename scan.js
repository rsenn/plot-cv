#!/usr/bin/env qjs
//
// scan.js — A4 document scanner using qjs-opencv
//
// Usage:  qjs scan.js <input-image> [output-prefix]
//
// Produces:
//   <prefix>_contour.json — 4 ordered corners + raw approx points
//   <prefix>_mask.png     — full-frame mask of the detected page
//   <prefix>_scanned.png  — warped → equalised → adaptive-thresholded output
//

import * as fs from 'fs';
import * as path from 'path';
import { Mat, Size, Point, Scalar, Contour, CLAHE, imread, imwrite, cvtColor, GaussianBlur, Canny, findContours, approxPolyDP, arcLength, contourArea, drawContours, getPerspectiveTransform, warpPerspective, adaptiveThreshold, COLOR_BGR2GRAY, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, CV_8UC1, imshow, waitKey, } from 'opencv';

function main() {
  // --- arg parsing ----------------------------------------------------------
  const rawArgs = typeof scriptArgs !== 'undefined' ? scriptArgs : [];
  // scriptArgs[0] is the script path under qjs; strip it if present
  const args = rawArgs[0]?.endsWith?.('.js') ? rawArgs.slice(1) : rawArgs;

  if(args.length < 1) {
    console.log('Usage: qjs scan.js <input-image> [output-prefix]');
    throw new Error('missing input file');
  }

  const inputPath = args[0];
  const prefix = args[1] || path.join(path.dirname(inputPath), path.basename(inputPath, path.extname(inputPath)));

  console.log('input  :', inputPath);
  console.log('prefix :', prefix);

  // --- 1. load source -------------------------------------------------------
  const src = imread(inputPath);
  if(!src || src.cols === 0 || src.rows === 0) throw new Error('could not read image: ' + inputPath);

  const W = src.cols,
    H = src.rows;
  console.log('source :', W, 'x', H);

  // --- 2. preprocess for edge detection -------------------------------------
  const gray = new Mat();
  const blurred = new Mat();
  const edges = new Mat();

  cvtColor(src, gray, COLOR_BGR2GRAY);
  GaussianBlur(gray, blurred, new Size(5, 5), 0);
  Canny(blurred, edges, 75, 200);

  imwrite('edges.png', edges);

  // --- 3. find the page contour (largest 4-vertex approximation) ------------
  const contours = [];
  /*const result =*/ findContours(edges, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  //console.log('contours:', contours);

  const MIN_AREA = W * H * 0.05;
  console.log('MIN_AREA:', MIN_AREA);

  // sort candidates by area, descending, then find first that approximates to a quad
  const ranked = [...contours]
    .map(c => ({ c, area: contourArea(c) }))
    //.filter(o => o.area > MIN_AREA) // ignore noise: < 5 % of frame
    .sort((a, b) => b.area - a.area);
  console.log('ranked[0]:', ranked[0].area);
  console.log('ranked[1]:', ranked[1].area);

  let pageContour = null;
  let pagePoints = null;

  for(const { c, area } of ranked) {
    let peri = arcLength(c, true);
    if(!Number.isFinite(peri)) peri = 10000;
    const approx = new Contour();
    //console.log('peri:', peri);
    c.approxPolyDP(approx, 0.02 * peri, true);
    const pts = [...approx].map(p => ({ x: p.x, y: p.y }));
    if(pts.length === 4) {
      pageContour = approx;
      pagePoints = pts;
      console.log('page area:', area, ' perimeter:', peri.toFixed(1));
      break;
    }
  }
  console.log('pageContour:', pageContour);
  console.log('pagePoints:', pagePoints);

  if(!pageContour) throw new Error('no quadrilateral page outline found');

  // --- 4. order corners as TL, TR, BR, BL -----------------------------------
  const { tl, tr, br, bl } = orderCorners(pagePoints);

  // --- 5. estimate output size, snap to A4 aspect (1 : √2) ------------------
  let outW = Math.max(dist(tr, tl), dist(br, bl));
  let outH = Math.max(dist(bl, tl), dist(br, tr));

  const A4 = Math.SQRT2; // long edge / short edge
  if(outH >= outW)
    outH = outW * A4; // portrait
  else outW = outH * A4; // landscape
  outW = Math.round(outW);
  outH = Math.round(outH);
  console.log('warp to:', outW, 'x', outH, '(A4)');

  // --- 6. perspective transform & warp --------------------------------------
  const srcQuad = [new Point(tl.x, tl.y), new Point(tr.x, tr.y), new Point(br.x, br.y), new Point(bl.x, bl.y)];
  const dstQuad = [new Point(0, 0), new Point(outW - 1, 0), new Point(outW - 1, outH - 1), new Point(0, outH - 1)];

  const M = getPerspectiveTransform(srcQuad, dstQuad);
  const warped = new Mat();
  warpPerspective(src, warped, M, new Size(outW, outH));

  // --- 7. write contour JSON ------------------------------------------------
  const contourJson = {
    source: inputPath,
    imageSize: { width: W, height: H },
    outputSize: { width: outW, height: outH },
    corners: { tl, tr, br, bl }, // ordered
    rawPoints: pagePoints, // approxPolyDP output, original order
  };
  fs.writeFileSync(prefix + '_contour.json', JSON.stringify(contourJson, null, 2));
  console.log('wrote  :', prefix + '_contour.json');

  // --- 8. write mask: filled white quad on black, original frame size -------
  const mask = new Mat(new Size(W, H), CV_8UC1);
  mask.setTo(new Scalar(0));
  drawContours(mask, [pageContour], -1, new Scalar(255), -1); // thickness -1 = filled
  imwrite(prefix + '_mask.png', mask);
  console.log('wrote  :', prefix + '_mask.png');

  // --- 9. post-process the cut-out: gray → CLAHE → adaptiveThreshold --------
  const warpedGray = new Mat();
  const equalised = new Mat();
  const scanned = new Mat();

  cvtColor(warped, warpedGray, COLOR_BGR2GRAY);

  const clahe = new CLAHE(2.0, new Size(8, 8)); // clipLimit, tileGridSize
  clahe.apply(warpedGray, equalised);

  adaptiveThreshold(
    equalised,
    scanned,
    255, // maxValue
    ADAPTIVE_THRESH_GAUSSIAN_C, // method
    THRESH_BINARY, // threshold type
    15, // blockSize (must be odd, >1)
    10, // C subtracted from weighted mean
  );

  imwrite(prefix + '_scanned.png', scanned);
  console.log('wrote  :', prefix + '_scanned.png');

  console.log('done.');
}

// --- helpers ----------------------------------------------------------------
function orderCorners(pts) {
  // top-left     has min(x + y)
  // bottom-right has max(x + y)
  // top-right    has min(y - x)
  // bottom-left  has max(y - x)
  let tl = pts[0],
    tr = pts[0],
    br = pts[0],
    bl = pts[0];
  let minSum = Infinity,
    maxSum = -Infinity;
  let minDif = Infinity,
    maxDif = -Infinity;
  for(const p of pts) {
    const s = p.x + p.y;
    const d = p.y - p.x;
    if(s < minSum) {
      minSum = s;
      tl = p;
    }
    if(s > maxSum) {
      maxSum = s;
      br = p;
    }
    if(d < minDif) {
      minDif = d;
      tr = p;
    }
    if(d > maxDif) {
      maxDif = d;
      bl = p;
    }
  }
  return { tl, tr, br, bl };
}

const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

main();
