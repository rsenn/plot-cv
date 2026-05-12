/**
 * YOLO object detection with qjs-opencv
 *
 * Requirements:
 * - QuickJS (qjs) installed
 * - qjs-opencv compiled (opencv.so)
 * - Model files downloaded (see below)
 *
 * Required files:
 * wget https://pjreddie.com/media/files/yolov3.weights
 * wget https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg
 * wget https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names
 *
 * Run:
 * qjs yolo.js --input image.jpg
 * qjs yolo.js --input 0 (webcam)
 * qjs yolo.js --input video.mp4
 */

import { destroyAllWindows, dnn, drawRect, FILLED, FONT_HERSHEY_SIMPLEX, getTextSize, imread, imshow, imwrite, Mat, Point, putText, Rect, Scalar, Size, TickMeter, VideoCapture, waitKey, } from 'opencv';
import * as std from 'std';
import * as os from 'os';

// --- Configuration --------------------------------------------------------------
const CONFIG = {
  weights: 'yolov3.weights',
  config: 'yolov3.cfg',
  names: 'coco.names',
  confThresh: 0.5, // minimum confidence (0-1)
  nmsThresh: 0.4, // Non-maximum suppression threshold
  inputSize: 416, // YOLO input resolution (320 / 416 / 608)
  outputFile: 'output.jpg',
};

// --- Auxiliary functions --------------------------------------------------------

/** Loads the class names from a text file */
function loadClassNames(path) {
  const f = std.open(path, 'r');
  if(!f) throw new Error(`Cannot open ${path}`);
  const names = [];
  let line;

  while((line = f.getline()) !== null) {
    const trimmed = line.trim();
    if(trimmed) names.push(trimmed);
  }

  f.close();
  return names;
}

/** Returns random (but consistent) color for each class */
function classColor(classId, total) {
  const hue = ((classId * 360) / total) % 360;
  // Simple HSV→BGR approximation
  const h = hue / 60;
  const s = 0.9,
    v = 255;
  const i = Math.floor(h);
  const f = h - i;
  const p = v * (1 - s);
  const q = v * (1 - s * f);
  const t = v * (1 - s * (1 - f));
  const rgb = [
    [v, t, p],
    [q, v, p],
    [p, v, t],
    [p, q, v],
    [t, p, v],
    [v, p, q],
  ][i % 6];

  return [Math.round(rgb[2]), Math.round(rgb[1]), Math.round(rgb[0])]; // BGR
}

/** Reads CLI argument */
function getArg(flag) {
  const args = scriptArgs || [];
  const i = args.indexOf(flag);
  return i !== -1 ? args[i + 1] : null;
}

// --- Load network -----------------------------------------------------------------
print('[1/4] Load YOLO-Net ...');
const net = dnn.readNetFromDarknet(CONFIG.config, CONFIG.weights);
net.setPreferableBackend(dnn.DNN_BACKEND_OPENCV);
net.setPreferableTarget(dnn.DNN_TARGET_CPU);

// Determine the names of the output layers
const layerNames = net.getLayerNames();
const unconnected = [...net.getUnconnectedOutLayers()];

const outputLayers = unconnected.map(i => layerNames[i - 1]);

print('[2/4] Load class names ...');
const classNames = loadClassNames(CONFIG.names);
const numClasses = classNames.length;
print(` ${numClasses} Classes loaded (COCO)`);

// --- Open input ---------------------------------------------------------
const inputSrc = getArg('--input') || 'Muehleberg.jpg';
const isCamera = !isNaN(parseInt(inputSrc));
const isVideo = !isCamera && (inputSrc.endsWith('.mp4') || inputSrc.endsWith('.avi') || inputSrc.endsWith('.mkv') || inputSrc.endsWith('.mov'));
const isImage = !isCamera && !isVideo;

print(`[3/4] Opened input: ${inputSrc} (${isCamera ? 'Webcam' : isVideo ? 'Video' : 'Image'})`);

let cap, frame;

if(isImage) {
  frame = imread(inputSrc);
  if(!frame || frame.empty) throw new Error(`Image not found: ${inputSrc}`);
} else {
  cap = new VideoCapture(isCamera ? parseInt(inputSrc) : inputSrc);
  if(!cap.isOpened()) throw new Error(`Cannot open input: ${inputSrc}`);
  frame = new Mat();
}

// --- YOLO inference --------------------------------------------------------

/**
 * Performs YOLO detection on a Mat.
 * Returns array of { classId, className, conf, x, y, w, h }.
 */
function detectYOLO(img) {
  const H = img.rows;
  const W = img.cols;

  //Image → YOLO blob (normalized, scaled, RGB swap)
  const blob = dnn.blobFromImage(
    img,
    1 / 255.0, // scaling factor
    new Size(CONFIG.inputSize, CONFIG.inputSize),
    new Scalar(0, 0, 0), // Mean subtraction
    true, // swapRB (BGR→RGB)
    false, // crop
  );

  net.setInput(blob);
  const outs = [];

  net.forward(outs, outputLayers);

  // Parse detections
  const boxes = [];
  const confidences = [];
  const classIds = [];

  for(const out of outs) {
    // out is [num_detections × (5 + num_classes)]
    for(let r = 0; r < out.rows; r++) {
      const row = out.row(r).array[0]; // Float32Array

      const scores = row.slice(5);

      let maxScore = 0;
      let classId = -1;

      for(let c = 0; c < scores.length; c++) {
        if(scores[c] > maxScore) {
          maxScore = scores[c];
          classId = c;
        }
      }
      if(maxScore < CONFIG.confThresh) continue;

      // YOLO returns normalized center coordinates
      const cx = row[0] * W;
      const cy = row[1] * H;
      const bw = row[2] * W;
      const bh = row[3] * H;

      boxes.push(new Rect(Math.round(cx - bw / 2), Math.round(cy - bh / 2), Math.round(bw), Math.round(bh)));
      confidences.push(maxScore);
      classIds.push(classId);
    }
  }

  // Non-maximum suppression
  const indices = [];

  dnn.NMSBoxes(boxes, confidences, CONFIG.confThresh, CONFIG.nmsThresh, indices);

  return indices.map(i => ({
    classId: classIds[i],
    className: classNames[classIds[i]] ?? `class_${classIds[i]}`,
    conf: confidences[i],
    x: boxes[i].x,
    y: boxes[i].y,
    w: boxes[i].width,
    h: boxes[i].height,
  }));
}

/**
 * Draws bounding boxes + labels on an image.
 */
function drawDetections(img, detections) {
  for(const d of detections) {
    const color = classColor(d.classId, numClasses);
    const pt1 = new Point(d.x, d.y);
    const pt2 = new Point(d.x + d.w, d.y + d.h);

    // Bounding box
    drawRect(img, pt1, pt2, color, 1);

    // Label background
    const label = `${d.className} ${(d.conf * 100).toFixed(0)}%`;
    const baseline = [0];
    const [width, height] = getTextSize(label, FONT_HERSHEY_SIMPLEX, 0.5, 1, baseline);
    const labelY = Math.max(d.y, height + 4);

    drawRect(img, new Point(d.x, labelY - height - 4), new Point(d.x + width + 4, labelY), color, FILLED);

    // Text
    putText(img, label, new Point(d.x + 2, labelY - 2), FONT_HERSHEY_SIMPLEX, 0.5, Scalar(255, 255, 255, 255), 1);
  }
}

// --- Main Loop ---------------------------------------------------------------
print('[4/4] Starting recognition ...\n');

if(isImage) {
  // Single image mode
  const detections = detectYOLO(frame);

  print(`Found objects: ${detections.length}`);
  for(const d of detections) {
    print(` ${('[' + d.className + ']').padEnd(20, ' ')} confidence: ${(d.conf * 100).toFixed(1)}% ` + `box: (${d.x}, ${d.y}, ${d.w}×${d.h})`);
  }

  drawDetections(frame, detections);
  imwrite(CONFIG.outputFile, frame);
  print(`\nResult saved: ${CONFIG.outputFile}`);
} else {
  // Video / webcam mode
  let frameCount = 0;
  const ticker = new TickMeter();

  while(true) {
    ticker.reset();
    ticker.start();

    if(!cap.read(frame) || frame.empty()) {
      print('Input ended.');
      break;
    }

    const detections = detectYOLO(frame);
    drawDetections(frame, detections);

    ticker.stop();
    const fps = (1000 / ticker.getTimeMilli()).toFixed(1);

    // Show FPS
    putText(frame, `FPS: ${fps} Objects: ${detections.length}`, new Point(10, 25), FONT_HERSHEY_SIMPLEX, 0.7, new Scalar(0, 255, 0), 2);

    imshow('YOLO – qjs-opencv (q = quit)', frame);

    frameCount++;
    if(frameCount % 10 === 0) print(`Frame ${frameCount} | FPS: ${fps} | Objects: ${detections.length}`);

    // 'q' or ESC to exit
    const key = waitKey(1) & 0xff;
    if(key === 113 /* q */ || key === 27 /* ESC */) break;
  }

  cap.release();
  destroyAllWindows();
}
