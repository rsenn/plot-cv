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
/*function saveMat(name, mat) {
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
}*/

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
    theta: new NumericParam(config.theta ?? 180, 1, 360),
    threshold: new NumericParam(config.threshold ?? 30, 0, 100),
    minLineLength: new NumericParam(config.minLineLength ?? 4, 0, 1000),
    maxLineGap: new NumericParam(config.maxLineGap ?? 30, 0, 1000),
    dp: new NumericParam(config.dp ?? 2, 0.1, 100),
    minDist: new NumericParam(config.minDist ?? 10, 1, 1000),
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

        cv.morphologyEx(dst, dst, cv.MORPH_ERODE, structuringElement);
        //   cv.erode(dst, dst, structuringElement);
        console.log('dst:', dst);
        dst.xor([255, 255, 255, 0], dst);

        console.log('dst:', dst.channels);
      },

      function Skeletonization(src, dst) {
        cv.skeletonization(src, dst);
      },

      function HoughLinesP(src, dst) {
        const skel = this.outputOf('Skeletonization');
        const morpho = this.outputOf('Morphology');
        let lines = new Mat();
        cv.HoughLinesP(skel,
          lines,
          +params.rho,
          Math.PI / +params.theta,
          +params.threshold,
          +params.minLineLength,
          +params.maxLineGap
        );
        // console.log('lines:', lines);
        cv.cvtColor(skel, dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        for(let elem of lines.values()) {
          const line = new Line(elem);
          draw.line(dst, ...line.toPoints(), [0, 255, 0], 1, cv.LINE_AA);
          draw.line(morpho, ...line.toPoints(), [0, 0, 0], 2, cv.LINE_8);
          draw.line(skel, ...line.toPoints(), [0, 0, 0], 1, cv.LINE_8);
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
        console.log('paramArray:', paramArray);

        let circles1 = [] ?? new Mat();
        let circles2 = [] ?? new Mat();
        cv.HoughCircles(morpho, circles1, cv.HOUGH_GRADIENT, ...paramArray);
        console.log('circles1:', circles1);
        cv.HoughCircles(skel, circles2, cv.HOUGH_GRADIENT, ...paramArray);
        console.log('circles2:', circles2);

        cv.cvtColor(morpho, dst, cv.COLOR_GRAY2BGR);
        //  this.outputOf('HoughLinesP').copyTo(dst);
        //  cv.cvtColor(this.outputOf('Grayscale'), dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        for(let [x, y, r] of circles1) {
          let p = new Point(x, y);
          draw.circle(dst, p, r, [0, 255, 0], 1, cv.LINE_AA);
          console.log('elem:', p.toString(), r);
        }
        for(let [x, y, r] of circles2) {
          let p = new Point(x, y);
          // draw.circle(dst, p, 9, [255, 0, 255], 2, cv.LINE_AA);
          draw.circle(dst, p, r + 2, [255, 0, 0], 1, cv.LINE_AA);
          console.log('elem:', p.toString(), r);
        }
      }
    ],
    (mat, i, n) => {
      console.log('output:', InspectMat(mat));
      cv.imshow('output', mat);
      cv.setWindowTitle('output', `#${i}: ` + pipeline.names[i]);
      /* if(frameShow == i) {
        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;

       
      }*/

      // let m = (outputMat || mat) ?  (outputMat || mat).dup() : null;
    }
  );

  if(frameShow < 0) frameShow += pipeline.size;
  if(frameShow >= pipeline.size) frameShow -= pipeline.size;
  SaveConfig({ frameShow, ...params });
  let key;

  while(true) {
    console.log('pipeline.step()', pipeline.step());
    key = cv.waitKey(-1);
    if(key != -1) console.log('key:', key);

    if(key == 8) {
      cv.waitKey(-1);
      pipeline.currentProcessor--;
      console.log('pipeline.currentProcessor', pipeline.currentProcessor);
    }

    if(key == 'q' || key == 113 || key == '\x1b') break;
  }
}

Util.callMain(main, true);
