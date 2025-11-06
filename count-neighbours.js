import { COLOR_BGR2GRAY, COLOR_GRAY2BGR, cvtColor, imread, imshow, invert, Mat, skeletonization, pixelNeighborhood, pixelNeighborhoodCross, waitKey, resize, INTER_LINEAR, INTER_NEAREST, INTER_LINEAR_EXACT, dilate, erode, morphologyEx, Point, adaptiveThreshold, threshold, BORDER_ISOLATED, THRESH_BINARY, THRESH_OTSU, ADAPTIVE_THRESH_MEAN_C, ADAPTIVE_THRESH_GAUSSIAN_C, getStructuringElement, MORPH_OPEN, MORPH_CLOSE, MORPH_CROSS, MORPH_RECT, MORPH_ERODE, MORPH_DILATE, MORPH_HITMISS, traceSkeleton, GaussianBlur, paletteApply } from 'opencv';
import { startInteractive } from 'util';

const pal = (globalThis.pal = [
  [0, 0, 0],
  [255, 0, 0],
  [255, 128, 0],
  [255, 255, 0],
  [0, 255, 0],
  [0, 255, 153],
  [0, 204, 255],
  [0, 51, 255],
  [102, 0, 255]
]);

function Show(m, scale = true, usePalette = false) {
  let m2 = new Mat();
  let out = new Mat();

  if(usePalette) {
    paletteApply(m, m2, pal);
  } else {
    m.copyTo(m2);
  }

  if(scale) resize(m2, out, null, 2, 2, INTER_LINEAR);
  else m2.copyTo(out);

  imshow('output', out);
  waitKey(-1);
  destroyWindow('output');
  waitKey(1);
}

function Neighbourhood(m) {
  //cvtColor(m, m, COLOR_BGR2GRAY);
  // m.xor(0xff); // invert;

  let skeleton = new Mat();
  skeletonization(m, skeleton);

  let neigh = new Mat();

sjke  pixelNeighborhood(skeleton, neigh);

  return neigh;
}

function main(...args) {
  let input = (globalThis.input = imread(args[0] ?? '../an-tronics/images/fm/4tr.jpg'));

  cvtColor(input, input, COLOR_BGR2GRAY);
  input.xor(0xff); // invert;

  GaussianBlur(input, input, [3, 3], 0);
  threshold(input, input, 0, 255, THRESH_BINARY | THRESH_OTSU);

  globalThis.input = input;

  let m = (globalThis.img = new Mat());
  input.copyTo(m);

  let k = (globalThis.kernel = getStructuringElement(MORPH_CROSS, [3, 3]));

  morphologyEx(m, m, MORPH_CLOSE, k, new Point(-1, -1), 1);

  let skeleton = (globalThis.skeleton = new Mat());

  skeletonization(m, skeleton);

  let out = new Mat();

  try {
    globalThis.neigh = Neighbourhood(m);
  } catch(e) {
    console.log('error', e?.message ?? e);
  }

  globalThis.r = traceSkeleton(skeleton, (globalThis.polylines = []), null, (globalThis.mapping = new Mat()));

  startInteractive();
}

Object.assign(globalThis, { Show, traceSkeleton });

main(...scriptArgs.slice(1));
