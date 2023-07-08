import { COLOR_BGR2GRAY } from 'opencv';
import { COLOR_GRAY2BGR } from 'opencv';
import { cvtColor } from 'opencv';
import { imread } from 'opencv';
import { imshow } from 'opencv';
import { invert } from 'opencv';
import { Mat } from 'opencv';
import { skeletonization } from 'opencv';
import { waitKey } from 'opencv';
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