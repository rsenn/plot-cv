import * as cv from 'opencv';
import Console from 'console';
import * as path from 'path';

function Grayscale(src, dst) {
  let channels = [];
  cv.cvtColor(src, dst, cv.COLOR_BGR2Lab);
  cv.split(dst, channels);
  channels[0].copyTo(dst);
}
function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: 100,
      compact: 1,
      depth: 10
    }
  });
  let ctor_names = Object.getOwnPropertyNames(cv).filter(name => typeof cv[name] == 'function');

  let features2d_names = ctor_names.filter(name => cv[name].prototype && cv[name].prototype[Symbol.toStringTag] == 'Feature2D');

  console.log('cv', features2d_names);

  let img = cv.imread('/home/roman/Dokumente/nokia5510.png');
  let float = new cv.Mat(),
    canny = new cv.Mat();
  let gray = new cv.Mat();

  let channels = [];
  cv.split(img, channels);

  Grayscale(img, gray);
  gray.convertTo(float, cv.CV_32F, 1.0 / 255.0);
  console.log('float', float);
  let na = new Float32Array(float.buffer);
  console.log('na', na);

  let contours, hier;
  cv.Canny(gray, canny, 40, 90, 3);
  cv.findContours(canny, (contours = []), h => (hier = h), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE);

  cv.cvtColor(gray, img, cv.COLOR_GRAY2BGR);
cv.drawContours(img, contours, -1, { r: 0, g: 255, b: 0, a: 255 }, 1, cv.LINE_AA);

  console.log('contours', contours);

  cv.namedWindow('img');
  cv.resizeWindow('img', 640, 480);
  cv.imshow('img', img);

  cv.moveWindow('img', 0, 0);
  cv.waitKey(-1);

  console.log('EXIT');
}
try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
