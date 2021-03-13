import { Point } from 'point.so';
import { Size } from 'size.so';
import { Rect } from 'rect.so';
import { Mat } from 'mat.so';
import { UMat } from 'umat.so';
import * as cv from 'cv.so';
import { Line } from 'line.so';
import { Contour } from 'contour.so';
import { SliceIterator } from 'slice-iterator.so';
import * as draw from 'draw.so';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';

let filesystem;

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 0
  });
  let types = [
    ['CV_8U', cv.CV_8U],
    ['CV_8UC3', cv.CV_8UC3],
    ['CV_8S', cv.CV_8S],
    ['CV_16U', cv.CV_16U],
    ['CV_16S', cv.CV_16S],
    ['CV_32S', cv.CV_32S],
    ['CV_32F', cv.CV_32F],
    ['CV_64F', cv.CV_64F],
    ['CV_64FC2', cv.CV_64FC2]
  ];

  for(let [k, v] of types) {
    console.log(k,
      v,
      '0x' + v.toString(16),
      '0b' + v.toString(2),
      v >> 1,
      1 << ((v >> 1) + 3),
      1 << (v >> 1)
    );
  }

  let input = cv.imread(args[0] ??
      '../an-tronics/images/fm/Two-Transistor-Regenerative-Receiver-Schematic-Circuit-Diagram.jpg'
  );
  console.log('input.type', '0x' + input.type.toString(16));
  console.log('input.depth', '0x' + input.depth.toString(16));
  console.log('input.channels', '0x' + input.channels.toString(16));
  console.log('input.elemSize1', input.elemSize1);
  console.log('input.total', input.total);
  console.log('input.at', input.at(0, 0));
  let size = input.size;
  console.log('size', size);
  let { width, height } = size;
  let mat = new Mat(input.size, cv.CV_8UC3);
  let thresh = 100;
  const RandomPoint = () => new Point(Util.randInt(0, width - 1), Util.randInt(0, height - 1));
  const RandomColor = () => [Util.randInt(0, 255), Util.randInt(0, 255), Util.randInt(0, 255), 255];

  let gray = new Mat();
  cv.cvtColor(input, gray, cv.COLOR_BGR2GRAY);
  let blur = new Mat();
  cv.blur(gray, blur, new Size(3, 3));

  let canny = new Mat();
  cv.Canny(blur, canny, thresh, thresh * 2, 3);
  let canny2 = new Mat();
  canny.copyTo(canny2);

  let contours = [];
  let hierarchy = [];
  cv.findContours(canny,
    contours,
    hierarchy,
    cv.RETR_TREE,
    cv.CHAIN_APPROX_SIMPLE,
    new Point(0, 0)
  );
  let i = 0;

  function getDepth(idx, id = cv.HIER_PARENT) {
    let parent;
    let depth = 0;
    while(idx != -1) {
      const h = hierarchy[idx];
      depth++;
      idx = h[id];
    }
    return depth;
  }
  console.log(`contours.length`, contours.length);
  // console.log('contours', contours.map(c => [...c].map(({x,y}) =>({x,y}))));
  let lines = new Mat();
  let circles = [];
  let gray_inv = new Mat();
  cv.cvtColor(gray, gray_inv, cv.COLOR_GRAY2BGR);

  gray.xor([255, 255, 255, 0], gray_inv);
  cv.blur(gray_inv, gray_inv, new Size(3, 3));
  //cv.cvtColor(gray_inv, gray_inv, cv.COLOR_BGR2GRAY);
  console.log(`gray_inv`, gray_inv);

  const GetX = x => Math.floor(x / 5);
  const GetY = y => Math.floor(y / 5);

  cv.HoughLinesP(gray_inv, lines, 1, cv.CV_PI / 24, 40, 5, 10);
  cv.HoughCircles(gray_inv, circles, cv.HOUGH_GRADIENT, 1, 10, 200, 80, 1, 100);
  console.log(`circles`, circles);
  let ygrid = Array.from({ length: Math.floor(input.rows / 5) }, () => []);
  let xgrid = Array.from({ length: Math.floor(input.cols / 5) }, () => []);
  console.log(`ygrid`, ygrid);

  for(let elem of lines) {
    const line = new Line(elem);
    const [x1, y1, x2, y2] = elem;

    let angle = Math.floor(line.angle / (cv.CV_PI / 24) * 180)   % 24;
    if(Math.abs(angle) > 2) continue;
    draw.line(mat, line.a, line.b, [255, 0, 0, 255], 1, cv.LINE_AA);
    /*    console.log('line.angle:', (line.angle * 180) / Math.PI);
    console.log('line.length:', line.length);*/

    Util.pushUnique(ygrid[GetY(y1)], line);
    Util.pushUnique(xgrid[GetY(x1)], line);
    Util.pushUnique(ygrid[GetY(y2)], line);
    Util.pushUnique(xgrid[GetY(x2)], line);
  }
  ygrid = ygrid.map((lines, row) => [row * 5, lines]).filter(([row, lines]) => lines.length > 1);

  xgrid = xgrid.map((lines, col) => [col * 5, lines]).filter(([col, lines]) => lines.length > 1);

  //for(let [x] of xgrid) draw.line(mat, new Point(x, 0), new Point(x, input.rows), [255, 0, 255, 255], 1, cv.LINE_AA);

  // for(let [y] of ygrid) draw.line(mat, new Point(0, y), new Point(input.cols, y), [255, 255, 0, 255], 1, cv.LINE_AA);

  for(let [x, y, r] of circles) {
    draw.circle(mat, new Point(x, y), r + 3, [0, 128, 255, 255], 5, cv.LINE_AA);
  }
  for(let contour of contours) {
  console.log("contour.length",contour.length);
  let poly = new Contour();
  contour.approxPolyDP(poly, contour.arcLength()*0.02);

 console.log("poly.length",poly.length,  [...poly]);


 //if(poly.length > 10)continue;
    draw.contours(mat, contours, i, RandomColor() ?? [(i * 255) / contours.length, 0, 0, 0], 2);
    i++;
  }
  // console.log(`lines`, [...lines]);

  //console.log(`hierarchy`, hierarchy);
  i = 0;

  let input2 = new Mat();

  input.copyTo(input2);
  input2.xor([255, 255, 255, 0], input2);
  let input2u = input2.getUMat(cv.ACCESS_RW);
  console.log(`input2u`, input2u);
  cv.imshow('input2', input2u);
  /* console.log(`input.buffer`, input.buffer);
  console.log(`input2.buffer`, input2.buffer);*/

  //for(let i = 0; i < 100; i++) draw.line(mat, RandomPoint(), RandomPoint(), RandomColor(), 1, cv.LINE_AA);

  let out = new Mat(size, cv.CV_8UC3);

  for(let m of [input, mat, out]) console.log('m', m);

  cv.addWeighted(input2, 0.5, mat, 1, 0, out);

  let umat = out.getUMat(cv.ACCESS_READ);

  cv.imshow('lines', umat);

  let key;

  while((key = cv.waitKey(0))) {
    if(key != -1) console.log('key:', key);

    if(key == 'q' || key == 113 || key == '\x1b') break;
  }
}

Util.callMain(main, true);
