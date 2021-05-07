import { Point } from 'point';
import { Size } from 'size';
import { Rect } from 'rect';
import { Mat } from 'mat';
import { UMat } from 'umat';
import * as cv from 'cv';
import { Line } from 'line';
import { CLAHE } from 'clahe';
import * as draw from 'draw';
import inspect from './lib/objectInspect.js';
import path from './lib/path.js';
import PortableFileSystem from './lib/filesystem.js';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';
import { TickMeter } from 'utility';

let filesystem;
let basename = Util.getArgv()[1].replace(/\.js$/, '');

function WriteImage(name, mat) {
  cv.imwrite(name, mat);
  console.log(`Wrote '${name}' (${mat.size}).`);
}

function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v]));
  let file = std.open(basename + '.config.json', 'w+b');
  file.puts(JSON.stringify(configObj, null, 2) + '\n');
  file.close();
  console.log(`Saved config to '${basename + '.config.json'}'`,
    inspect(configObj, { compact: false })
  );
}

function LoadConfig() {
  let str = std.loadFile(basename + '.config.json');
  let configObj = JSON.parse(str ?? '{}');

  configObj = Object.fromEntries(Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:',  inspect(configObj, {compact:false}));
  return configObj;
}

function InspectMat(mat) {
  const { channels, depth, type, cols, rows } = mat;

  return inspect({ channels, depth, type, cols, rows });
}

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 1
  });
  await PortableFileSystem(fs => (filesystem = fs));
  let running = true;

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
  let { frameShow = 1, paramIndex = 0, ...config } = LoadConfig();
  let outputName, outputMat;

  let params = {
    thres: new NumericParam(config.thres ?? 64, 0, 255),
    max: new NumericParam(config.max ?? 255, 0, 255),
    type: new NumericParam(config.type ?? cv.THRESH_BINARY_INV, 0, 4),
    kernel_size: new NumericParam(config.kernel_size ?? 1, 0, 10),
    k: new NumericParam(config.k ?? 24, 0, 100),
    thres1: new NumericParam(config.thres1 ?? 10, 0, 300),
    thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
    thres2: new NumericParam(config.thres2 ?? 20, 0, 300),
    rho: new NumericParam(config.rho ?? 1, 1, 100),
    theta: new NumericParam(config.theta ?? 180, 0, 360),
    threshold: new NumericParam(config.threshold ?? 10, 0, 100),
    minLineLength: new NumericParam(config.minLineLength ?? 2, 0, 1000),
    maxLineGap: new NumericParam(config.maxLineGap ?? 4, 0, 1000),
    dp: new NumericParam(config.dp ?? 2, 0, 10, 0.1),
    minDist: new NumericParam(config.minDist ?? 10, 1, 1000),
    param1: new NumericParam(config.param1 ?? 200, 1, 1000),
    param2: new NumericParam(config.param2 ?? 100, 1, 1000)
  };

  console.log('thres:', +params.thres);
  let lineWidth = 1;
  let lines;
  let circles = [];
  let paramNav = new ParamNavigator(params, paramIndex);
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

        cv.morphologyEx(dst, dst, cv.MORPH_ERODE, structuringElement);
        //   cv.erode(dst, dst, structuringElement);
        dst.xor([255, 255, 255, 0], dst);
      },

      function Skeletonization(src, dst) {
        cv.skeletonization(src, dst);
      },

      function HoughLinesP(src, dst) {
        const skel = this.outputOf('Skeletonization');
        const morpho = this.outputOf('Morphology');
        lines = new Mat();
        cv.HoughLinesP(skel,
          lines,
          +params.rho,
          Math.PI * +params.theta / 180,
          +params.threshold,
          +params.minLineLength,
          +params.maxLineGap
        );
        // console.log('lines:', lines);
        cv.cvtColor(skel, dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        for(let elem of lines.values()) {
          //console.log(`elem #${i}:`, elem);
          const line = new Line(elem);
          draw.line(dst, ...line.toPoints(), [255, 128, 0], lineWidth, cv.LINE_AA);
          draw.line(morpho, ...line.toPoints(), [0, 0, 0], 2, cv.LINE_8);
          draw.line(skel, ...line.toPoints(), [0, 0, 0], lineWidth, cv.LINE_8);
          ++i;
        }
        let kern = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));
        cv.dilate(skel, skel, kern);
        cv.erode(skel, skel, kern);

        cv.dilate(morpho, morpho, kern);
        /*   cv.erode(morpho, morpho, kern);*/

        //cv.morphologyEx(skel, skel, cv.MORPH_DILATE, kern);
      },

      function HoughCircles(src, dst) {
        const morpho = this.outputOf('Morphology');
        const skel = this.outputOf('Skeletonization');
        const paramArray = [+params.dp, +params.minDist, +params.param1, +params.param2, 0, 90];

        let circles1 = [] ?? new Mat();
        let circles2 = [] ?? new Mat();
        cv.HoughCircles(morpho, circles1, cv.HOUGH_GRADIENT, ...paramArray);
        //console.log('circles1:', circles1);
        cv.HoughCircles(skel, circles2, cv.HOUGH_GRADIENT, ...paramArray);
        //console.log('circles2:', circles2);

        this.outputOf('HoughLinesP').copyTo(dst);

        let i = 0;
        /*   for(let elem of lines.values()) {
          const line = new Line(elem);
          draw.line(dst, ...line.toPoints(), [255, 255, 0], lineWidth, cv.LINE_AA);
          ++i;
        }*/
        for(let [x, y, r] of circles1) {
          let p = new Point(x, y);
          draw.circle(dst, p, r, [0, 255, 0], lineWidth, cv.LINE_AA);
          circles.push([x, y, r]);
        }
        for(let [x, y, r] of circles2) {
          let p = new Point(x, y);
          draw.circle(dst, p, r + 2, [255, 0, 0], lineWidth, cv.LINE_AA);
          circles.push([x, y, r]);
        }
      }
    ],
    (mat, i, n) => {
      //console.log('pipeline callback', { i, n });
      if(frameShow == i) {
        cv.imshow('output', mat);
        cv.setWindowTitle('output', `#${i}: ` + pipeline.names[i]);
      }
    }
  );
 // frameShow = config.frameShow ?? 0;

  let key;

  console.log(`pipeline.recalc(${frameShow})`, pipeline.recalc(frameShow));

  while(true) {
    key = cv.waitKeyEx(-1);

    if(key === 'q' || key === 113 || key === '\x1b' || key === 0x100071) break;

    switch (key & 0xfff) {
      case 0xf08 /* backspace */:
      case 0x08 /* backspace */:
        if(frameShow > 0) {
          frameShow--;
          pipeline.step(-1);
        }
        break;
      case 0x3c /* < */:
        paramNav.prev();
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected`);
        break;
      case 0x3e /* > */:
        paramNav.next();
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected`);
        break;

      case 0x2b /* + */:
        paramNav.param.increment();
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline/*.recalc*/(frameShow);
        break;

      case 0xfff /* DELETE */:
      case 0x9f /* numpad DEL */:
      case 0xf9f /* numpad DEL */:
        paramNav.param.reset();
        console.log(`Param ${paramNav.name}: ${inspect(paramNav.param)}`);
        pipeline.recalc(frameShow);
        break;

      case 0x2d /* - */:
      case 0xad /* numpad - */:
      case 0xfad /* numpad - */:
      case 0x2fad /* numpad - */:
        paramNav.param.decrement();
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline/*.recalc*/(frameShow);
        break;

      case 0x31: /* 1 */
      case 0x32: /* 2 */
      case 0x33: /* 3 */
      case 0x34: /* 4 */
      case 0x35: /* 5 */
      case 0x36: /* 6 */
      case 0x37: /* 7 */
      case 0x38: /* 8 */
      case 0x39: /* 9 */
      case 0x30 /* 0 */:
        let v = key & 0xf || 10;
        paramNav.param.alpha = v / 10;
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline.recalc(frameShow);
        break;
      case 0xa7 /* § */:
        paramNav.param.alpha = 0;
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline.recalc(frameShow);
        break;

      case 0x20:
        frameShow = Util.mod(frameShow + 1, pipeline.size);
        console.log(`Back`, { frameShow, size: pipeline.size });
        //frameShow = (frameShow + 1) % pipeline.size;
        pipeline.step();
        break;

      default: {
        if(key !== -1) console.log('key:', '0x' + key.toString(16));
        break;
      }
    }
  }

  SaveConfig({ frameShow, paramIndex: paramNav.index, ...params });

  console.log('EXIT');
}

main(...scriptArgs.slice(1))
  .then(() => console.log('SUCCESS'))
  .catch(error => {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    std.exit(1);
  });

//Util.callMain(main, true);
