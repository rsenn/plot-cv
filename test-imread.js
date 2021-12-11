import * as cv from 'opencv';
import Console from 'console';
import * as path from 'path';
import * as misc from 'misc';
import Util from './lib/util.js';

function Grayscale(src, dst) {
  let channels = [];
  cv.cvtColor(src, dst, cv.COLOR_BGR2Lab);
  cv.split(dst, channels);
  channels[0].copyTo(dst);
}

function* TraverseHierarchy(h, s = -1, depth = 0) {
  let a = Array.isArray(h) ? h : [...h];
  let i = a[s] ? s : a.findIndex(([n, p, c, u]) => u == -1);
  while(a[i]) {
    let [n, p, c, u] = a[i];
    yield [i, depth];
    if(a[c]) yield* TraverseHierarchy(h, c, depth + 1);
    i = n;
  }
}

function main(...args) {
  globalThis.console = new Console(process.stdout, {
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: 100,
      compact: 3,
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

  let contours,
    hier,
    lines = new cv.Mat();
  cv.Canny(gray, canny, 40, 90, 3);
  cv.findContours(canny, (contours = []), (hier = new cv.Mat()), cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE);

  console.log('hier', hier);
  console.log('hier.cols', hier.cols);
  console.log('hier.depth', 1 << (hier.depth + 1));
  console.log('hier.channels', hier.channels);

  console.log('contours', contours);
  console.log('contours.length', contours.length);


  cv.cvtColor(gray, img, cv.COLOR_GRAY2BGR);
  cv.drawContours(img, contours, -1, { r: 0, g: 255, b: 0, a: 255 }, 1, cv.LINE_AA);

  for(let [id, depth] of TraverseHierarchy(hier, 0)) {
    //console.log('contour', { id, depth });
    const c = contours[id];
    let contour = [...c];

    //const {area,length}=contour;
     console.log('contour', contour /*, misc.getClassID(c),Object.getPrototypeOf(c)*/);
  }
  return;

  cv.HoughLinesP(canny, lines, 1, cv.CV_PI / 24, 40, 5, 10);

  for(let line of lines) {
    let [x1, y1, x2, y2] = line;
    cv.line(img, [x1, y1], [x2, y2], [255, 0, 255, 255], 1, cv.LINE_AA);
    // console.log('line', line);
  }

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
  console.log(`FAIL: ${error?.message}\n${error?.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
