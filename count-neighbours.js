import { filter2D, Mat, invert, imread, imshow, waitKey, COLOR_BGR2GRAY, COLOR_GRAY2BGR, cvtColor, skeletonization } from 'opencv';

function main(...args) {
  let m = imread('../an-tronics/images/fm/4tr.jpg');

  cvtColor(m, m, COLOR_BGR2GRAY);
  invert(m, m);

  let skeleton = new Mat();
  skeletonization(m, skeleton);

  let out = new Mat();
  cvtColor(skeleton, out, COLOR_GRAY2BGR);

  imshow('output', out);
  waitKey(1000);
}

main(...scriptArgs.slice(1));
