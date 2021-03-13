import { Point } from 'point.so';
import { Size } from 'size.so';
import { Rect } from 'rect.so';
import { Mat } from 'mat.so';
import { UMat } from 'umat.so';
import * as cv from 'cv.so';
import { Line } from 'line.so';
import { CLAHE } from 'clahe.so';
import * as draw from 'draw.so';
import inspect from './lib/objectInspect.js';
import path from './lib/path.js';
import PortableFileSystem from './lib/filesystem.js';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';

let filesystem;
function saveMat(name, mat) {
  let ext = mat.channels == 1 ? 'pgm' : 'ppm';
  let p = mat.channels == 1 ? 'P2' : 'P3';
  let colors = [];
  for(let [pos, value] of image) {
    let c = value;
    colors.push(c);
  }
  filesystem.writeFile(`${name}.${ext}`,
    `${p}\n${image.cols} ${image.rows}\n255\n${colors.flat().join('\n')}`
  );
}

function WriteImage(name, mat) {
  cv.imwrite(name, mat);
  console.log(`Wrote '${name}' (${mat.size}).`);
}

function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v]));

  return filesystem.writeFile(Util.getArgv()[1].replace(/\.js$/, '.config.json'),
    JSON.stringify(configObj, null, 2) + '\n'
  );
}

function LoadConfig() {
  let str = filesystem.readFile(Util.getArgv()[1].replace(/\.js$/, '.config.json'), 'utf-8');
  let configObj = JSON.parse(str ?? '{}');

  configObj = Object.fromEntries(Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', configObj);
  return configObj;
}

function dumpMat(name, mat) {
  /* console.log(`${name} = Mat { `,
    ['cols', 'rows', 'depth', 'channels', 'total', 'elemSize', 'elemSize1', 'step']
      .map(prop => `${prop}: ${mat[prop]}`)
      .concat([`step1: ${mat.step1()}`])
      .join(', ') + ' }'
  );*/
  console.log(`${name} =`, mat);
  return;

  console.log(`${name} = Mat { `,
    Object.create(
      Object.prototype,
      ['cols', 'rows', 'depth', 'channels', 'total', 'elemSize', 'elemSize1', 'step'].reduce((acc, prop) => ({
          ...acc,
          [prop]: { value: mat[prop], enumerable: true }
        }),
        { step1: { value: mat.step1(), enumerable: true } }
      )
    )
  );

  return;
  console.log(`${name}`, mat);
  console.log(`${name}.cols`, mat.cols);
  console.log(`${name}.rows`, mat.rows);
  console.log(`${name}.depth`, mat.depth);
  console.log(`${name}.channels`, mat.channels);

  let typeName = Object.entries(cv).filter(([name, value]) => /^CV_[0-9]/.test(name) && value == mat.type
  );

  console.log(`${name}.type`, typeName.length == 1 ? typeName[0][0] : mat.type);
}

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 0
  });
  await PortableFileSystem(fs => (filesystem = fs));

  // console.log('cv', cv);
  //console.log('Object.keys(cv)', Object.keys(cv));
  console.log('Util.getMethodNames(cv)', Util.getMethodNames(cv, Infinity, 0));
  console.log('cv.HoughLines', cv.HoughLines);

  let line = new Line(0, 0, 50, 50);

  console.log('line', line);

  let clahe = new CLAHE();
  console.log('clahe', clahe);

  let image;

  /* for(let windowName of ['gray', 'corners', 'threshold', 'canny'])
    cv.namedWindow(windowName, cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO);*/
  cv.namedWindow('output', cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO);

  //image = cv.imread('../an-tronics/images/5.19.jpg');
  image = cv.imread(args[0] || 'italo-disco.png');
  let { frameShow = 1, ...config } = LoadConfig();
  let outputName, outputMat;

  let params = {
    thres: new NumericParam(config.thres ?? 8, 0, 255),
    max: new NumericParam(config.max ?? 255, 0, 255),
    type: new NumericParam(config.type ?? cv.THRESH_BINARY_INV, 0, 4),
    kernel_size: new NumericParam(config.kernel_size ?? 1, 0, 10),
    k: new NumericParam(config.k ?? 24, 0, 100),
    thres1: new NumericParam(config.thres1 ?? 10, 0, 300),
    thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
    thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
    rho: new NumericParam(config.rho ?? 1, 1, 100),
    theta: new NumericParam(config.theta ?? 2, 0, 240),
    threshold: new NumericParam(config.threshold ?? 30, 0, 100),
    minLineLength: new NumericParam(config.minLineLength ?? 0, 0, 1000),
    maxLineGap: new NumericParam(config.maxLineGap ?? 0, 0, 1000),
    dp: new NumericParam(config.dp ?? 2, 0.1, 100),
    minDist: new NumericParam(config.minDist ?? 100, 1, 1000),
    param1: new NumericParam(config.param1 ?? 200, 1, 1000),
    param2: new NumericParam(config.param2 ?? 100, 1, 1000)
  };

  console.log('thres:', +params.thres);

  let paramNav = new ParamNavigator(params, config.currentParam);
  let pipeline = new Pipeline([
      function AcquireFrame(src, dst) {
        image = cv.imread(args[0] || 'italo-disco.png');
        image.copyTo(dst);
      },
      function Grayscale(src, dst) {
        let channels = [];
        cv.cvtColor(src, dst, cv.COLOR_BGR2Lab);
        cv.split(dst, channels);
        channels[0].copyTo(dst);
      },
      function Threshold(src, dst) {
        cv.threshold(src, dst, +params.thres, +params.max, +params.type);
      },
      function Morphology(src, dst) {
        let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS,
          new Size(+params.kernel_size * 2 + 1, +params.kernel_size * 2 + 1)
        );
        if(+params.type == cv.THRESH_BINARY_INV) src.xor([255, 255, 255, 0], dst);
        else src.copyTo(dst);

        cv.erode(dst, dst, structuringElement);
        console.log('dst:', dst);
        dst.xor([255, 255, 255, 0], dst);
      },
      function Blur(src, dst) {
        const gray = this.outputOf('Grayscale');

        cv.medianBlur(gray, dst, 5);
      },
      function Corners(src, dst) {
        src.convertTo(dst, cv.CV_32FC1);
        let corners = new Mat(dst.rows, dst.cols, cv.CV_32FC1);
        cv.cornerHarris(dst, corners, 2, 3, +params.k);
        corners.convertTo(corners, cv.CV_8UC1);
      },
      function Canny(src, dst) {
        cv.Canny(this.outputOf('Morphology'), dst, +params.thres1, +params.thres2);
      },
      function HoughLinesP(src, dst) {
        const morpho = this.outputOf('Morphology');
        let lines = new Mat();
        cv.HoughLinesP(morpho,
          lines,
          +params.rho,
          +params.theta,
          +params.threshold,
          +params.minLineLength,
          +params.maxLineGap
        );
        // console.log('lines:', lines);
        cv.cvtColor(morpho, dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        for(let elem of lines.values()) {
          const line = new Line(elem);
          draw.line(dst, ...line.toPoints(), [0, 255, 0], 1, cv.LINE_AA);
          ++i;
        }
      },
      function HoughCircles(src, dst) {
        const blur = this.outputOf('Blur');
        let circles = [] ?? new Mat();
        console.log('HoughCircles', { blur, circles });
        cv.HoughCircles(blur,
          circles,
          cv.HOUGH_GRADIENT,
          +params.dp,
          +params.minDist,
          +params.param1,
          +params.param2,
          0,
          0
        );
        console.log('circles:', circles);
        cv.cvtColor(this.outputOf('Grayscale'), dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        for(let [x, y, r] of circles) {
          let p = new Point(x, y);
          draw.circle(dst, p, 1, [0, 100, 100], 3, cv.LINE_AA);
          draw.circle(dst, p, r, [255, 0, 255], 3, cv.LINE_AA);
          console.log('elem:', p.toString(), r);
        }
      }
    ],
    (mat, i, n) => {
      if(frameShow == i) {
        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;
      }

      // let m = (outputMat || mat) ?  (outputMat || mat).dup() : null;
    }
  );

  if(frameShow < 0) frameShow += pipeline.size;
  if(frameShow >= pipeline.size) frameShow -= pipeline.size;

  pipeline();
  console.log('frameShow', pipeline.names[frameShow], frameShow);
  SaveConfig({ frameShow, ...params });

  cv.imshow('output', outputMat);

  console.log('pipeline.names', pipeline.names);
  console.log('pipeline.outputOf("Threshold")', pipeline.outputOf('Threshold'));

  let key;

  while((key = cv.waitKey(0))) {
    if(key != -1) console.log('key:', key);

    if(key == 'q' || key == 113 || key == '\x1b') break;
  }

  return;

  let globalThis = Util.getGlobalObject();
  const moduleNames = ['Rect', 'Point', 'Size', 'Line', 'Mat', 'Contour', 'PointIterator', 'Draw'];
  for(let moduleName of moduleNames)
    Util.tryCatch(() => eval(`globalThis[moduleName] = ${moduleName};`));
  let ctors = new Map(moduleNames.map(name => [name, globalThis[name]]));
  console.log('globalThis:', Object.keys(globalThis));
  console.log('modules:', inspect(ctors));
  if(globalThis.Point) {
    let point = new Point(25, 75);
    console.log(`inspect(point)`, inspect(point));
  }
  function toHex(n, b = 2) {
    let s = (+n).toString(16);
    return '0x' + '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
  }
  let rr;
  if(globalThis.Rect) {
    rr = new Rect(5, 3, 4, 5);
    if(rr) {
      const { x, y, width, height } = rr;
      console.log('rect:', x, y, width, height);
    }
  }
  if(globalThis.Mat) {
    let mat = new Mat(new Size(10, 10), Mat.CV_8UC4);
    console.log(`Mat.CV_8UC3`, toHex(Mat.CV_8UC3), Mat.CV_8UC3);
    console.log(`Mat.CV_8UC4`, toHex(Mat.CV_8UC4), Mat.CV_8UC4);
    console.log(`Mat.CV_8SC3`, toHex(Mat.CV_8SC3), Mat.CV_8SC3);
    console.log(`Mat.CV_8SC4`, toHex(Mat.CV_8SC4), Mat.CV_8SC4);
    console.log(`Mat.CV_32FC1`, toHex(Mat.CV_32FC1), Mat.CV_32FC1);
    console.log(`Mat.CV_32FC4`, toHex(Mat.CV_32FC4), Mat.CV_32FC4);
    console.log(`0x3ff`, toHex(0x3ff));
    console.log(`inspect(mat)`, inspect(mat));
    console.log(`mat.channels`, mat.channels);
    console.log(`mat.depth`, mat.depth);
    console.log(`1 << mat.depth`, 1 << mat.depth);
    console.log(`Mat[DEPTH]`,
      Object.keys(Mat).find(k => Mat[k] === mat.depth)
    );
    console.log(`Mat[TYPE]`,
      Object.keys(Mat).find(k => Mat[k] === mat.type)
    );
    let row0 = mat.row(0);
    let col0 = mat.col(0);
    console.log(`mat.row(0)`, row0);
    for(let r = 0; r < mat.rows; r++)
      for(let c = 0; c < mat.cols; c++) {
        const v = (r << 24) | c;
        console.log(`mat.set(${r},${c},0x${v.toString(16)})`, mat.set(r, c, v));
      }
    console.log(`mat.set(0,1,0xcafebabe)`, mat.set(0, 1, 0xcafebabe));
    console.log(`mat.set(0,2,0xc01dd00d)`, mat.set(0, 2, 0xc01dd00d));
    console.log(`row0.at(0,0)`, row0.at(0, 0));
    console.log(`mat.at(0,0)`, mat.at(0, 0));
    console.log(`mat.at(new Point(0,0))`, mat.at(new Point(0, 0)));
    let it = row0[Symbol.iterator]();
    console.log(`row0[Symbol.iterator]()`, it);
    let step = it.next();
    console.log(`it.next()`, step.done, step.value);
    let i = 0;
    for(let x of row0.values()) {
      console.log(`row0.values()[${i++}]`, x);
    }
    i = 0;
    it = row0.keys();
    console.log(`row0.keys()`, it);
    console.log(`row0.keys().next`, it.next);
    console.log(`row0.keys()[Symbol.iterator]`, it[Symbol.iterator]);
    let v;
    while(true) {
      v = it.next();
      if(v.done) break;
      console.log(`row0.keys() #${i++}`, v.value, v.value.length);
    }
    i = 0;
    for(let [key, value] of row0.entries()) {
      console.log(`row0.entries() #${i++}`,
        key,
        '0x' + ('00000000' + value.toString(16)).slice(-8)
      );
    }
    i = 0;
    for(let [key, value] of col0.entries()) {
      console.log(`col0.entries() #${i++}`,
        key,
        '0x' + ('00000000' + value.toString(16)).slice(-8)
      );
    }
    let range = mat.rowRange(2, 8);
    i = 0;
    for(let [[row, col], value] of range) {
      console.log(`range[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`
      );
    }
    i = 0;
    if(globalThis.Rect) {
      let roi = mat.roi(rr);
      for(let [[row, col], value] of roi) {
        console.log(`roi[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`
        );
      }
      for(let r = 0; r < roi.rows; r++)
        for(let c = 0; c < roi.cols; c++) {
          const v = 0x7f000000 | ((r << 16) | c);
          console.log(`roi.set(${r},${c},0x${v.toString(16)})`, roi.set(r, c, v));
        }
      roi.setTo(...Util.repeat(4 * 5, 0xffffffff));
    }
    i = 0;
    for(let [[row, col], value] of mat) {
      console.log(`mat[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`
      );
    }
    let fmat = new Mat(new Size(10, 10), Mat.CV_32FC1);
    const values = Util.repeat(fmat.rows * fmat.cols, 0.5);
    console.log(`fmat setTo`, values);
    fmat.setTo(...values);
    for(let [[row, col], value] of fmat) {
      console.log(`fmat[${i++}] row=${row} col=${col} value=${value}`);
    }
  }
  if(globalThis.Line) {
    let ll = [new Line(0, 0, 50, 50), new Line(50, 50, 50, 75), new Line(50, 75, 100, 75)];
    for(let line of ll) {
      console.log('line:', line.x1, line.y1, line.x2, line.y2);
      const { a, b } = line;
      console.log('a =', a);
      console.log('b =', b);
      console.log('line[0] =', line[0]);
      console.log('line[1] =', line[1]);
      console.log('line.toString() =', line.toString());
      let i = 0;
      let arr = line.toArray();
      console.log('toArray:', line.toArray().join(','));
      console.log('values(): ', line.values());
      console.log('toPoints(): ',
        [...line.toPoints()].map(p => Util.className(p))
      );
      console.log('toString(): ', line.toString());
      console.log('new Line(50,50,320-50,240-25): ', new Line(50, 50, 320 - 50, 240 - 25));
      let [x1, y1, x2, y2] = arr;
      console.log(`Line{${x1},${y1} ${x2},${y2}}`);
      for(let num of line) {
        console.log('num:', i++, num);
      }
    }
    if(globalThis.Rect) {
      let r = new Rect(50, 100, 350, 200);
      console.log('r.br(): ', r.br());
      console.log('r.tl(): ', r.tl());
      console.log('r.area(): ', r.area());
      if(globalThis.Point) {
        let pt = new Point(75, 150);
        console.log(`r.contains(${pt}): `, r.contains(pt));
        pt = new Point(51, 99);
        console.log(`r.contains(${pt}): `, r.contains(pt));
      }
      r = new Rect(50, 50, 0, 0);
      console.log('r.empty(): ', r.empty());
    }
  }
  if(0) {
    console.log(`std.gc`, std.gc);
    console.log(`args`, args);
    console.log(`path`, inspect(path));
    console.log(`console`, Util.inspect(console));
    console.log(`filesystem.realpath('.')`, filesystem.realpath('.'));
    console.log(`filesystem.chdir('..')`, filesystem.chdir('..'));
    console.log(`filesystem.getcwd('.')`, filesystem.getcwd());
    console.log(`std.gc()`, std.gc());
  }
  return 'done';
}

Util.callMain(main, true);
