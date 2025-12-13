import * as path from 'path';
import { range } from 'util';
import Console from 'console';
import * as cv from 'opencv';
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
    let entry = a[i];
    let [, prev, child] = entry;
    yield [i, depth];
    if(a[child]) yield* TraverseHierarchy(h, child, depth + 1);
    i = entry[cv.HIER_NEXT];
  }
}

function* SegmentRect(size, seg = new cv.Size(14, 12)) {
  let rect = size instanceof cv.Size ? new cv.Rect(0, 0, ...size) : new cv.Rect(size);

  let rows = rect.vsplit(...range(0, size.height, seg.height).slice(1, -1));
  let y = 0;
  for(let row of rows) {
    let x = 0;
    let cols = row.hsplit(...range(0, size.width, seg.width).slice(1, -1));

    //yield cols;
    for(let col of cols) {
      yield col;
      x++;
    }
    y++;
  }
}

function Image2ASCII(img, pixelfn = bit => (bit ? '1' : '0')) {
  let rows = [];
  for(let [[row, col], pixel] of img.entries()) {
    rows[row] ??= '';
    rows[row] += pixelfn(pixel[0] < 128);
  }
  return rows.join('\n');
}

function main(...args) {
  globalThis.console = new Console(process.stdout, {
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      compact: 0,
      depth: 10
    }
  });
  let ctor_names = Object.getOwnPropertyNames(cv).filter(name => typeof cv[name] == 'function');

  let features2d_names = ctor_names.filter(name => cv[name].prototype && cv[name].prototype[Symbol.toStringTag] == 'Feature2D');

  console.log('cv', features2d_names);

  args[0] ??= '/home/roman/Dokumente/Urzeitcode/font-14x24.png';
  let name = path.basename(args[0]);
  let dim = new cv.Size([...name.matchAll(/\d+/g)].map(([n]) => +n));
  console.log('dim', dim);

  let img = cv.imread(args[0]);
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

  let dbl = new cv.Mat();

  cv.resize(img, dbl, new cv.Size(img.cols * 2, img.rows * 2), 0, 0, cv.INTER_LINEAR);
  cv.threshold(dbl, dbl, 127, 255, cv.THRESH_BINARY);

  let mrect = new cv.Rect(0, 12, 84, 24);
  let middle = img(mrect);

  let chars = '12345V67890V';

  let segments = [...SegmentRect(img.size, dim)];
  console.log('segments', console.config({ compact: 0 }), segments);
  let i = 0;
  for(let segment of segments) {
    let subsegs = [...SegmentRect(segment, new cv.Size(dim.width, 8))];

    console.log('subsegs', chars[i], console.config({ compact: 0 }), subsegs);
    let j = 0;
    for(let subseg of subsegs) {
      let segimg = img(subseg);

      let asc = Image2ASCII(segimg, bit => (bit ? '██' : '  '));
      console.log(j + '\n' + asc);
      j++;
    }

    i++;
  }

  /*  let contours,
    hier,
    lines = new cv.Mat();
  cv.Canny(gray, canny, 0, 90, 3);
  cv.findContours(canny, (contours = []), (hier = new cv.Mat()), cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE);

  console.log('contours.length', contours.length);

  cv.cvtColor(gray, img, cv.COLOR_GRAY2BGR);
  cv.drawContours(img, contours, -1, { r: 0, g: 255, b: 0, a: 255 }, 1, cv.LINE_AA);
  let rects = new Array(contours.length);
  for(let [id, depth] of TraverseHierarchy(hier, 0)) {

    const c = contours[id];
    let contour = c;

    const r = contour.boundingRect();
    c[4] = rects[id] = r;
    const { tl, br } = r;

    cv.rectangle(img, tl, br, [255, 0, 255, 255], 1, cv.LINE_AA);
  }*/

  cv.namedWindow('img');
  // cv.resizeWindow('img', 1280, 800);
  cv.imshow('img', dbl);

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