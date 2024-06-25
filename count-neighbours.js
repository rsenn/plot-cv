import { COLOR_BGR2GRAY, COLOR_GRAY2BGR, cvtColor, imread, imshow, invert, Mat, skeletonization, pixelNeighborhood, pixelNeighborhoodCross, waitKey, resize, INTER_NEAREST, dilate, erode, morphologyEx, Point, adaptiveThreshold, threshold, BORDER_ISOLATED, THRESH_BINARY,THRESH_OTSU, ADAPTIVE_THRESH_MEAN_C, ADAPTIVE_THRESH_GAUSSIAN_C, getStructuringElement, MORPH_OPEN, MORPH_CLOSE, MORPH_CROSS ,MORPH_ERODE,MORPH_DILATE, MORPH_HITMISS} from 'opencv';
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

function Show(m) {
  let out = new Mat();

  resize(m, out, m.size.mul(2), 0, 0, INTER_NEAREST);

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

  pixelNeighborhood(skeleton, neigh);

  return neigh;
}

function main(...args) {
  let m = imread('../an-tronics/images/fm/4tr.jpg');

  cvtColor(m, m, COLOR_BGR2GRAY);
  m.xor(0xff); // invert;
  globalThis.img = m;

  //adaptiveThreshold(m, m, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, 3, -2);
  //
  let k = getStructuringElement(MORPH_CROSS, [3, 3]);

  morphologyEx(m, m, MORPH_CLOSE, k, new Point(-1, -1), 1);
 // morphologyEx(m, m,MORPH_HITMISS, k, new Point(-1, -1), 1);

  /*dilate(m, m, new Mat(), new Point(-1, -1), 1);
  erode(m, m, new Mat(), new Point(-1, -1), 1);*/
  //dilate(m, m, new Mat(), new Point(-1, -1));

threshold(m, m, 0,255, THRESH_BINARY |THRESH_OTSU);

  let skeleton = (globalThis.skeleton = new Mat());

  skeletonization(m, skeleton);
  let out = new Mat();

  try {
    globalThis.neigh = Neighbourhood(m);
  } catch(e) {
    console.log('error', e?.message ?? e);
  }

  /*pixelNeighborhood(skeleton, neigh);

  cvtColor(skeleton, out, COLOR_GRAY2BGR);

  imshow('output', out);
  waitKey(-1);*/

  startInteractive();
}

Object.assign(globalThis, { Show });

main(...scriptArgs.slice(1));
