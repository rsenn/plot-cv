#!/usr/bin/env qjs
import { existsSync, mkdirSync } from 'fs';
import { basename, extname, join } from 'path';
import { CommandLineParser, Mat, Size, Point, Scalar, imread, imwrite, cvtColor, COLOR_BGR2HSV, inRange, GaussianBlur, bitwise_or, bitwise_and, getStructuringElement, morphologyEx, MORPH_RECT, MORPH_CLOSE, MORPH_OPEN, findContours, drawContours, contourArea, boundingRect, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, drawRect as rectangle, CV_8UC1, LINE_AA, } from 'opencv';

function main(argv) {
  // --------------------------------------------------------------------------
  // CLI definition
  //
  //   {names | default | description}
  //   @name  → positional argument
  //   <none> → required (no default)
  // --------------------------------------------------------------------------
  const keys =
    `{help h usage ? |       | print this message                }` +
    `{@input         | <none>| input image path                  }` +
    `{@output        | .     | output directory                  }` +
    `{top            | 0.6   | top-region prior, 0..1 of height  }` +
    `{blur           | 5     | gaussian pre-blur kernel size     }` +
    `{morph          | 7     | morphology kernel size            }`;

  const parser = new CommandLineParser(argv, keys);
  parser.about('sky_detect v1.0 — HSV sky mask + largest-contour extraction');

  if(parser.has('help')) {
    parser.printMessage();
    return 0;
  }

  const inputPath = parser.get('@input');
  const outputDir = parser.get('@output');
  const topFrac = parser.get('top');
  const blurK = parser.get('blur');
  const morphK = parser.get('morph');

  if(!parser.check()) {
    parser.printErrors();
    return 1;
  }

  if(!existsSync(inputPath)) {
    console.log(`error: input not found: ${inputPath}`);
    return 1;
  }

  if(!existsSync(outputDir)) mkdirSync(outputDir, { recursive: true });

  const stem = basename(inputPath, extname(inputPath));
  const maskPath = join(outputDir, `${stem}_mask.png`);
  const contourPath = join(outputDir, `${stem}_contour.png`);

  // --------------------------------------------------------------------------
  // Load
  // --------------------------------------------------------------------------
  const src = imread(inputPath);
  if(src.empty) {
    console.log(`error: failed to decode image: ${inputPath}`);
    return 1;
  }

  const W = src.cols,
    H = src.rows;
  console.log(`loaded ${inputPath} (${W}x${H})`);

  // --------------------------------------------------------------------------
  // Build a sky mask in HSV
  //   Sky has two flavors:
  //     (a) blue sky: H ≈ 90–135, S moderate, V high
  //     (b) bright/white/overcast sky: very low S, very high V (any H)
  // --------------------------------------------------------------------------
  const blurred = new Mat();
  GaussianBlur(src, blurred, new Size(blurK, blurK), 0);

  const hsv = new Mat();
  cvtColor(blurred, hsv, COLOR_BGR2HSV);

  const maskBlue = new Mat();
  inRange(hsv, new Scalar(90, 30, 120), new Scalar(135, 255, 255), maskBlue);

  const maskBright = new Mat();
  inRange(hsv, new Scalar(0, 0, 200), new Scalar(179, 40, 255), maskBright);

  const maskColor = new Mat();
  bitwise_or(maskBlue, maskBright, maskColor);

  // --------------------------------------------------------------------------
  // Position prior: keep only the top `topFrac` of the frame.
  // --------------------------------------------------------------------------
  const maskTop = new Mat(new Size(W, H), CV_8UC1, new Scalar(0));
  rectangle(maskTop, new Point(0, 0), new Point(W, Math.round(H * topFrac)), new Scalar(255), -1);

  const mask = new Mat();
  bitwise_and(maskColor, maskTop, mask);

  // --------------------------------------------------------------------------
  // Clean: close gaps, then drop speckle.
  // --------------------------------------------------------------------------
  const kernel = getStructuringElement(MORPH_RECT, new Size(morphK, morphK));
  morphologyEx(mask, mask, MORPH_CLOSE, kernel);
  morphologyEx(mask, mask, MORPH_OPEN, kernel);

  // --------------------------------------------------------------------------
  // Largest connected region = sky contour
  // --------------------------------------------------------------------------
  const contours = [];
  findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

  //console.log('contours.length',contours.length);

  if(!contours || contours.length === 0) {
    console.log('no sky region detected');
    imwrite(maskPath, mask);
    imwrite(contourPath, src);
    console.log(`wrote ${maskPath}`);
    console.log(`wrote ${contourPath}`);
    return 2;
  }

  let bestIdx = -1,
    bestArea = 0;
  for(let i = 0; i < contours.length; i++) {
    const a = contourArea(contours[i]);
    if(a > bestArea) {
      bestArea = a;
      bestIdx = i;
    }
  }

  const coverage = bestArea / (W * H);
  const bbox = boundingRect(contours[bestIdx]);
  console.log(`sky contour: idx=${bestIdx}, area=${bestArea.toFixed(0)} px ` + `(${(coverage * 100).toFixed(1)}% of image)`);
  console.log(`bbox: x=${bbox.x}, y=${bbox.y}, w=${bbox.width}, h=${bbox.height}`);

  // --------------------------------------------------------------------------
  // Render outputs
  // --------------------------------------------------------------------------
  imwrite(maskPath, mask);

  const overlay = src.clone();
  drawContours(overlay, contours, bestIdx, new Scalar(0, 255, 0), 2, LINE_AA);
  rectangle(overlay, new Point(bbox.x, bbox.y), new Point(bbox.x + bbox.width, bbox.y + bbox.height), new Scalar(0, 200, 255), 2);
  imwrite(contourPath, overlay);

  console.log(`wrote ${maskPath}`);
  console.log(`wrote ${contourPath}`);
  return 0;
}

main(typeof scriptArgs !== 'undefined' ? scriptArgs : []);
