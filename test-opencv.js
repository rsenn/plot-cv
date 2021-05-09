import { Point } from 'point';
import { Size } from 'size';
import { Rect } from 'rect';
import { Mat } from 'mat';
import { UMat } from 'umat';
import * as cv from 'cv';
import * as fs from 'fs';
import Console from 'console';
import { Line } from 'line';
import { CLAHE } from 'clahe';
import * as draw from 'draw';
import * as path from 'path';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';
import { TickMeter } from 'utility';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './cvHighGUI.js';

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
  console.log('LoadConfig:', inspect(configObj, { compact: false }));
  return configObj;
}

function InspectMat(mat) {
  const { channels, depth, type, cols, rows } = mat;

  return inspect({ channels, depth, type, cols, rows });
}

function Accumulator(callback) {
  let self;
  let accu = {};
  self = function(name, value) {
    accu[name] = value;

    if(typeof callback == 'function') callback(name, value);
  };
  Object.assign(self, {
    accu,
    entries() {
      return Object.entries(this.accu);
    },
    values() {
      return Object.values(this.accu);
    },
    keys() {
      return Object.keys(this.accu);
    },
    *[Symbol.iterator]() {
      for(let key in this.accu) yield [key, this.accu[key]];
    }
  });
  return self;
}

function main(...args) {
  new Console({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 1
  });
  /*  await PortableFileSystem(fs => (filesystem = fs));*/
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
  let statusRect = new Rect(0, resolution.height, resolution.width, 200);
  let textRect = new Rect(statusRect.size).inset(5);
  let screenSize = new Size(resolution.width, resolution.height + 200);
  console.log('statusRect', statusRect);
  console.log('textRect', textRect);

  let screen = new Mat(screenSize, cv.CV_8UC3);
  let output = screen(outputRect);
  let status = screen(statusRect);

  let backgroundColor = 0xd0d0d0;
  let shadowColor = 0x404040;
  let textColor = 0x000000;
  let fonts = [
    '/home/roman/.fonts/gothic.ttf',
    '/home/roman/.fonts/gothicb.ttf',
    '/usr/share/fonts/truetype/ubuntu/UbuntuMono-R.ttf'
  ];
  let fontFace = fonts[0];
  let fontSize = 17;

  fonts.forEach(file => draw.loadFont(file));

  output.setTo([255, 255, 255]);
  status.setTo(backgroundColor);

  let { frameShow = 1, paramIndex = 0, ...config } = LoadConfig();

  let params = {
    thres: new NumericParam(config.thres ?? 229, 0, 255),
    type: new NumericParam(config.type ?? cv.THRESH_BINARY_INV, 0, 4),
    blur: new NumericParam(config.blur ?? 1, 1, 10, 2),
    kernel_size: new NumericParam(config.kernel_size ?? 0, 0, 9),
    rho: new NumericParam(config.rho ?? 1, 1, 30, 0.25),
    theta: new NumericParam(config.theta ?? 1, 0, 90),
    threshold: new NumericParam(config.threshold ?? 25, 0, 50),
    minLineLength: new NumericParam(config.minLineLength ?? 3, 0, 30),
    maxLineGap: new NumericParam(config.maxLineGap ?? 4, 0, 20),
    dp: new NumericParam(config.dp ?? 2, 0, 10, 0.1),
    minDist: new NumericParam(config.minDist ?? 10, 1, 1000),
    param1: new NumericParam(config.param1 ?? 200, 1, 1000),
    param2: new NumericParam(config.param2 ?? 100, 1, 100),
    minRadius: new NumericParam(config.minRadius ?? 0, 1, 250),
    maxRadius: new NumericParam(config.maxRadius ?? 200, 1, 1000)
  };
  let neighborhood;
  console.log('cv.pixelNeighborhood', cv.pixelNeighborhood);
  console.log('thres:', +params.thres);
  let lineWidth = 1;
  let lines;
  let circles = [];
  let paramNav = new ParamNavigator(params, paramIndex);

  let palette = new Uint32Array(256);

  for(let i = 0; i < 256; i++) palette[i] = 0xff000000;

  palette[0] = 0xff000000;
  palette[1] = 0xff00ffff;
  palette[2] = 0xff606060;
  palette[3] = 0xffff0000;
  palette[4] = 0xff00ff00;
  palette[5] = 0xff0080ff;
  palette[6] = 0xff0000ff;

  DrawText(2, 0, 'Test');
  DrawText(1, 1, 'Test');
  DrawText(0, 2, 'Test');
  DrawText(0, 2, 'BLAH');

  function DrawText(x, y, text, color = textColor, thickness = -1) {
    let font = new TextStyle(fontFace, fontSize, thickness);
    let size = font.size(text);
    let pos = new Point((size.width / text.length) * x, (size.height + 3) * (1 + y))
      .add(textRect.x, textRect.y)
      .add(x, y);
    // font.draw(status, text, pos.add(1, 1), shadowColor, thickness + 1);
    font.draw(status, text, pos, color, -1, cv.LINE_AA);
    //draw.text(status, text, pos, fontFace, fontSize, color, -1, cv.LINE_AA);
  }

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
        /*  if(+params.type == cv.THRESH_BINARY_INV) src.xor([255, 255, 255, 0], dst);
        else*/ src.copyTo(dst
        );
        console.log('Morphology dst', dst);

        cv.morphologyEx(dst, dst, cv.MORPH_ERODE, structuringElement);
        //   cv.erode(dst, dst, structuringElement);
        dst.xor([255, 255, 255, 0], dst);
      },

      function Skeletonization(src, dst) {
        console.log('Skeletonization src', src);
        cv.skeletonization(src, dst);
        console.log('Skeletonization dst', dst);
        console.log('Skeletonization dst', this.outputOf('Skeletonization'));
      },

      function PixelNeighborhood(src, dst) {
        neighborhood = new Mat(src.size, src.type);
        let output;

        cv.pixelNeighborhood(src, neighborhood);
        console.log('non-zero:', cv.countNonZero(neighborhood));
        console.log('non-zero:', cv.findNonZero(neighborhood));
        console.log('pixels:', neighborhood.size.area);

        let coords = [[], ...Util.range(1, 6).map(n => cv.pixelFindValue(neighborhood, n))];

        console.log('coords:', Object.fromEntries(coords.map(a => a.length).entries()));

        output = cv.paletteApply(neighborhood, palette);
        output.copyTo(dst);

        cv.imwrite('neighborhood.png', output);
      },

      function HoughLinesP(src, dst) {
        const skel = this.outputOf('Skeletonization');
        const morpho = this.outputOf('Morphology');
        lines = new Mat();

        console.log('Skeletonization dst', this.outputOf('Skeletonization'));
        if(skel.channels > 1) cv.cvtColor(skel, skel, cv.COLOR_BGR2GRAY);
        console.log('HoughLinesP skel', skel);

        if(morpho.channels > 1) cv.cvtColor(morpho, morpho, cv.COLOR_BGR2GRAY);
        console.log('HoughLinesP morpho', morpho);

        cv.HoughLinesP(skel,
          lines,
          +params.rho,
          (Math.PI * (+params.theta || 1)) / 180,
          +params.threshold,
          +params.minLineLength,
          +params.maxLineGap
        );
        // console.log('lines:', lines);
        console.log('\x1b[1;31mskel\x1b[0m', skel);
        cv.cvtColor(skel, dst, cv.COLOR_GRAY2BGR);
        console.log('skel', skel);
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
        const paramArray = [
          +params.dp || 1,
          +params.minDist,
          +params.param1,
          +params.param2,
          +params.minRadius,
          +params.maxRadius
        ];
        console.log('HoughCircles morpho', morpho);
        console.log('HoughCircles skel', skel);

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
        let rect = new Rect(mat.size);
        console.log('rect', rect);
        output = screen(rect);

        if(mat.channels == 1) cv.cvtColor(mat, mat, cv.COLOR_GRAY2BGR);

        mat.copyTo(output);

        draw.rect(screen(statusRect), new Rect(statusRect.size), backgroundColor, cv.FILLED, true);

        DrawText(0, 0, `#${i}: ` + pipeline.names[i], textColor);

        cv.imshow('output', screen);
        cv.resizeWindow('output', screenSize.width, screenSize.height);
        cv.setWindowTitle('output', `#${i}: ` + pipeline.names[i]);
      }
    }
  );

  let key;

  let accu = paramNav.setCallback(new Accumulator((name, param) => {
      console.log(`param '${name}' callback`, param);
    })
  );

  console.log('accu:', [...accu]);

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
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected (${+paramNav.param})`);
        break;
      case 0x3e /* > */:
        paramNav.next();
        console.log(`Param #${paramNav.index} '${paramNav.name}' selected (${+paramNav.param})`);
        break;

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
try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
