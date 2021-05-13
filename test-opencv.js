import { Point, Size, Rect, Mat, UMat, Line, CLAHE, TickMeter, Draw } from 'opencv';
import * as cv from 'opencv';
import * as fs from 'fs';
import Console from 'console';
//import * as draw from 'draw';
import * as path from 'path';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle, DrawText } from './cvHighGUI.js';
let basename = Util.getArgv()[1].replace(/\.js$/, '');
function WriteImage(name, mat) {
  cv.imwrite(name, mat);
  console.log("Wrote '" + name + "' (" + mat.size + ').');
}
function SaveConfig(configObj) {
  configObj = Object.fromEntries(Object.entries(configObj).map(([k, v]) => [k, +v]));
  let file = std.open(basename + '.config.json', 'w+b');
  file.puts(JSON.stringify(configObj, null, 2) + '\n');
  file.close();
  console.log("Saved config to '" + basename + '.config.json' + "'",
    inspect(configObj, { compact: false })
  );
}
function LoadConfig() {
  let str = std.loadFile(basename + '.config.json');
  let configObj = JSON.parse(str || '{}');
  configObj = Object.fromEntries(Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', inspect(configObj, { compact: false }));
  return configObj;
}
function InspectMat(mat) {
  const { channels, depth, type, cols, rows } = mat;
  return inspect({ channels, depth, type, cols, rows });
}
function ToHex(number) {
  if(number < 0) number = 0xffffffff + number + 1;
  return '0x' + number.toString(16);
}
function Accumulator(callback) {
  let self;
  let accu = {};
  self = function(name, value) {
    if(name in accu) return;
    accu[name] = value;
    if(typeof callback == 'function') callback(name, value);
  };
  Object.assign(self, {
    accu,
    entries() {
      return Object.entries(accu);
    },
    values() {
      return Object.values(accu);
    },
    keys() {
      return Object.keys(accu);
    },
    *[Symbol.iterator]() {
      for(let key in accu) yield [key, accu[key]];
    },
    clear() {
      for(let key in accu) delete accu[key];
    }
  });
  return self;
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
  let running = true;
  console.log('Util.getMethodNames(cv)', Util.getMethodNames(cv, Infinity, 0));
  console.log('cv.HoughLines', cv.HoughLines);
  let line = new Line(0, 0, 50, 50);
  console.log('line', line);
  let clahe = new CLAHE();
  console.log('clahe', clahe);
  cv.namedWindow('output', cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO);
  let trackbar = '';
  let file = args[0] || '../an-tronics/images/fm/4tr.jpg';
  let image = cv.imread(file);
  let resolution = image.size;
  let scaled;
  console.log('Symbol.inspect', Symbol.inspect);
  console.log('resolution', resolution);
  if(resolution.width > 1200) {
    let f = 1024 / resolution.width;
    scaled = new Size(resolution.width * f, resolution.height * f);
  } else {
    scaled = new Size(resolution);
  }
  let outputRect = new Rect(0, 0, resolution.width, resolution.height);
  let outputMat = new Mat(outputRect.size, cv.CV_8UC3);
  let statusRect = new Rect(0, resolution.height, resolution.width, 200);
  let statusMat = new Mat(statusRect.size, cv.CV_8UC3);
  console.log('statusRect:', statusRect);
  let [textRect, helpRect] = new Rect(statusRect.size).inset(5).vsplit(-20);
  let screenSize = new Size(resolution.width, resolution.height + 200);
  console.log('statusRect', statusRect);
  console.log('textRect', textRect);
  console.log('helpRect:', helpRect);
  let screen = new Mat(screenSize, cv.CV_8UC3);
  cv.imshow('output', screen);
  cv.moveWindow('output', 0, 0);
  cv.resizeWindow('output', screenSize.width);
  let backgroundColor = 0xd0d0d0;
  let shadowColor = 0x404040;
  let textColor = 0xd3d7cf;
  let fonts = [
    '/home/roman/.fonts/gothic.ttf',
    '/home/roman/.fonts/gothicb.ttf',
    '/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf'
  ];
  let fontFace = fonts[2];
  let fontSize = 14;
  fonts.forEach(file => Draw.loadFont(file));
  let config = LoadConfig();
  let { frameShow = 1, paramIndex = 0 } = config;
  let params = {
    thres: new NumericParam(config.thres || 229, 0, 255),
    type: new NumericParam(config.type || cv.THRESH_BINARY_INV, 0, 4),
    blur: new NumericParam(config.blur || 1, 1, 10, 2),
    kernel_size: new NumericParam(config.kernel_size || 0, 0, 9),
    rho: new NumericParam(config.rho || 1, 1, 30, 0.25),
    theta: new NumericParam(config.theta || 1, 0, 90),
    threshold: new NumericParam(config.threshold || 25, 0, 50),
    minLineLength: new NumericParam(config.minLineLength || 3, 0, 30),
    maxLineGap: new NumericParam(config.maxLineGap || 4, 0, 20),
    dp: new NumericParam(config.dp || 2, 0, 10, 0.1),
    minDist: new NumericParam(config.minDist || 10, 1, 1000),
    param1: new NumericParam(config.param1 || 200, 1, 1000),
    param2: new NumericParam(config.param2 || 100, 1, 100),
    minRadius: new NumericParam(config.minRadius || 0, 1, 250),
    maxRadius: new NumericParam(config.maxRadius || 200, 1, 1000)
  };
  let lineWidth = 1;
  let lines = [];
  let circles = [];
  let paramNav = new ParamNavigator(params, paramIndex);
  let paramIndexes = [-1, -1];
  let palette = new Array();
  const black = [0x00, 0x00, 0x00, 0xff];
  for(let i = 0; i < 8; i++)
    palette[i] = [i & 0x04 ? 0xff : 0x00, i & 0x02 ? 0xff : 0x00, i & 0x01 ? 0xff : 0x00, 0xff];
  palette[2] = [0x60, 0x60, 0x60, 0xff];
  palette[3] = [0xff, 0xff, 0x0, 0xff];
  for(let i = 8; i < 16; i++) palette[i] = black;
  let pipeline = new Pipeline([
      function AcquireFrame(src, dst) {
        image = cv.imread(file);
        console.log('AcquireFrame', { image });
        image.copyTo(dst);
        console.log('AcquireFrame', { dst });
      },
      function Grayscale(src, dst) {
        let channels = [];
        console.log('Grayscale', { src });
        cv.cvtColor(src, dst, cv.COLOR_BGR2Lab);
        cv.split(dst, channels);
        channels[0].copyTo(dst);
      },
      function Blur(src, dst) {
        cv.GaussianBlur(src, dst, [+params.blur, +params.blur], 0, 0, cv.BORDER_REPLICATE);
      },
      function Threshold(src, dst) {
        cv.threshold(src, dst, +params.thres, 255, +params.type);
      },
      function Morphology(src, dst) {
        let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS,
          new Size(+params.kernel_size * 2 + 1, +params.kernel_size * 2 + 1)
        );
        src.copyTo(dst);
        cv.morphologyEx(dst, dst, cv.MORPH_ERODE, structuringElement);
        dst.xor([255, 255, 255, 0], dst);
      },
      function Skeletonization(src, dst) {
        cv.skeletonization(src, dst);
      },
      function PixelNeighborhood(src, dst) {
        let neighborhood = new Mat(src.size, cv.CV_8UC1);
        cv.pixelNeighborhood(src, neighborhood);
        let endpoints = cv.pixelFindValue(src, 1);
        console.log('endpoints', endpoints);
        let linepoints = cv.pixelFindValue(src, 2);
        console.log('linepoints', linepoints);
        cv.imwrite('neighborhood.png', neighborhood, palette);
        let im = cv.imread('neighborhood.png');
        im.copyTo(dst);
      },
      function HoughLinesP(src, dst) {
        const skel = this.outputOf('Skeletonization');
        const morpho = this.outputOf('Morphology');
        let output = new Mat();
        if(skel.channels > 1) cv.cvtColor(skel, skel, cv.COLOR_BGR2GRAY);
        if(morpho.channels > 1) cv.cvtColor(morpho, morpho, cv.COLOR_BGR2GRAY);
        cv.HoughLinesP(skel,
          output,
          +params.rho,
          (Math.PI * (+params.theta || 1)) / 180,
          +params.threshold,
          +params.minLineLength,
          +params.maxLineGap
        );
        cv.cvtColor(skel, dst, cv.COLOR_GRAY2BGR);
        let i = 0;
        lines.splice(0, lines.length);
        for(let elem of output.values()) {
          const line = new Line(elem);
          lines.push(line);
          Draw.line(dst, ...line.toPoints(), [255, 128, 0], lineWidth, cv.LINE_AA);
          Draw.line(morpho, ...line.toPoints(), [0, 0, 0], 2, cv.LINE_8);
          Draw.line(skel, ...line.toPoints(), [0, 0, 0], lineWidth, cv.LINE_8);
          ++i;
        }
        lines.sort((a, b) => (a.y1 == b.y1 ? a.x1 - b.x1 : a.y1 - b.y1));
        console.log(`lines`, lines);
        let kern = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));
        cv.dilate(skel, skel, kern);
        cv.erode(skel, skel, kern);
        cv.dilate(morpho, morpho, kern);
      },
      function HoughCircles(src, dst) {
        const morpho = this.outputOf('Morphology');
        const skel = this.outputOf('Skeletonization');
        const paramArray = [
          +params.dp || 1,
          +params.minDist,
          +params.param1,
          +params.param2,
          +params.minRadius,
          +params.maxRadius
        ];
        let circles1 = [] || new Mat();
        let circles2 = [] || new Mat();
        cv.HoughCircles(morpho, circles1, cv.HOUGH_GRADIENT, ...paramArray);
        cv.HoughCircles(skel, circles2, cv.HOUGH_GRADIENT, ...paramArray);
        this.outputOf('HoughLinesP').copyTo(dst);
        let i = 0;
        for(let [x, y, r] of circles1) {
          let p = new Point(x, y);
          Draw.circle(dst, p, r, [0, 255, 0], lineWidth, cv.LINE_AA);
          circles.push([x, y, r]);
        }
        for(let [x, y, r] of circles2) {
          let p = new Point(x, y);
          Draw.circle(dst, p, r + 2, [255, 0, 0], lineWidth, cv.LINE_AA);
          circles.push([x, y, r]);
        }
      }
    ],
    i => {
      if(frameShow == i) {
        let processor = pipeline.getProcessor(i);
        let params = processorParams.get(processor);
        paramIndexes[0] = paramNav.indexOf(params[0]);
        paramIndexes[1] = paramNav.indexOf(params[params.length - 1]);
        if(paramNav.index < paramIndexes[0] || paramNav.index > paramIndexes[1])
          paramNav.current = params[0];
        let mat = pipeline.getImage(i);
        if(mat.channels == 1) cv.cvtColor(mat, outputMat, cv.COLOR_GRAY2BGR);
        else if(mat.channels == 4) cv.cvtColor(mat, outputMat, cv.COLOR_BGRA2BGR);
        else mat.copyTo(outputMat);
        RedrawStatus();
        RedrawWindow();
      }
    }
  );
  function RedrawStatus() {
    //console.log(`pipeline.images =`, new Map(pipeline.imageEntries()));
    let i = pipeline.currentProcessor;
    let processor = pipeline.getProcessor(i);
    let params = processorParams.get(processor);
    let srect = new Rect(statusRect.size);
    Draw.rect(statusMat, srect, backgroundColor, cv.FILLED, true);
    Draw.rect(statusMat, srect.inset(3, 0), 0, cv.FILLED, true);
    const inspectOptions = {
      colors: true,
      hideKeys: ['callback']
    };
    let text =
      `#${i}: ` +
      pipeline.names[i] +
      `\n\n` +
      `params:\n` +
      params
        .map((name, idx) => {
          return `  ${idx + paramIndexes[0] == paramNav.index ? '\x1b[1;31m' : ''}${name.padEnd(13
          )}\x1b[0m   \x1b[1;36m${+paramNav.get(name)}\x1b[0m\n`;
        })
        .join('');
    DrawText(statusMat(textRect), text, textColor, fontFace, fontSize);
    DrawText(statusMat(helpRect),
      '< prev, > next, + increment, - decrement, DEL reset',
      textColor,
      fontFace,
      fontSize
    );
  }
  function RedrawWindow() {
    let i = pipeline.currentProcessor;
    cv.vconcat([outputMat, statusMat], screen);
    cv.imshow('output', screen);
    cv.resizeWindow('output', screenSize.width, screenSize.height);
    cv.setWindowTitle('output', `#${i}: ` + pipeline.names[i]);
  }
  let key;
  let paramAccumulator = paramNav.setCallback(new Accumulator((name, param) => {
      // console.log(`param '${name}' callback`, param);
    })
  );
  let processorParams = Util.weakMapper(processor => []);
  pipeline.before = () => paramAccumulator.clear();
  pipeline.after = () => processorParams.set(pipeline.getProcessor(), paramAccumulator.keys());
  pipeline();
  delete pipeline.before;
  delete pipeline.after;
  console.log(`pipeline.recalc(${frameShow})`, pipeline.recalc(frameShow));
  while(true) {
    key = cv.waitKeyEx(-1);
    if(key === 'q' || key === 113 || key === '\x1b' || key === 0x100071 || key === -1) break;
    switch (key & 0xfff) {
      case 0xf08 /* backspace */:
      case 0x08 /* backspace */:
        if(frameShow > 0) {
          frameShow--;
          pipeline.step(-1);
        }
        break;
      case 0xf52 /* up */:
      case 0x3c /* < */:
        paramNav.prev();
        if(paramIndexes[0] != -1 && paramNav.index < paramIndexes[0])
          paramNav.index = paramIndexes[1];
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected (${+paramNav.param})`);
        RedrawStatus();
        RedrawWindow();
        break;
      case 0xf54 /*down  */:
      case 0x3e /* > */:
        paramNav.next();
        if(paramIndexes[1] != -1 && paramNav.index > paramIndexes[1])
          paramNav.index = paramIndexes[0];
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected (${+paramNav.param})`);
        RedrawStatus();
        RedrawWindow();
        break;
      case 0xf53 /* right */:
      case 0x2b /* + */:
        paramNav.param.increment();
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline.recalc(frameShow);
        break;
      case 0xfff /* DELETE */:
      case 0x9f /* numpad DEL */:
      case 0xf9f /* numpad DEL */:
        paramNav.param.reset();
        console.log(`Param ${paramNav.name}: ${inspect(paramNav.param)}`);
        pipeline.recalc(frameShow);
        break;
      case 0xf51 /* left */:
      case 0x2d /* - */:
      case 0xad /* numpad - */:
      case 0xfad /* numpad - */:
      case 0x2fad /* numpad - */:
        paramNav.param.decrement();
        console.log(`Param ${paramNav.name}: ${+paramNav.param}`);
        pipeline.recalc(frameShow);
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
        pipeline.step();
        break;
      default: {
        if(key !== -1) console.log('key:', ToHex(key));
        break;
      }
    }
  }
  SaveConfig({ frameShow, paramIndex: paramNav.index, ...params });
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
