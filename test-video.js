import { LoadConfig, SaveConfig } from './config.js';
import { GLFW } from './draw-utils.js';
import { SaveSVG } from './image-helpers.js';
import { WriteJSON } from './io-helpers.js';
import { HSLA } from './lib/color.js';
import { memoize } from './lib/misc.js';
import SvgPath from './lib/svg/path.js';
import { TCPClient } from './midi-tcp.js';
import { EnumParam, NumericParam, ParamNavigator } from './param.js';
import { Mouse, MouseFlags, TextStyle, Window } from './qjs-opencv/js/cvHighGUI.js';
import { Pipeline, Processor } from './qjs-opencv/js/cvPipeline.js';
import { BitsToNames, FindKey, GetOpt, Modulo, Once, Range, WeakMapper } from './qjs-opencv/js/cvUtils.js';
import { VideoSource } from './qjs-opencv/js/cvVideo.js';
import Console from 'console';
import * as cv from 'opencv';
import { CLAHE, Affine3, BoostDesc, BriefDescriptorExtractor, Canny, COLOR_BayerBG2BGR, COLOR_BayerBG2BGRA, COLOR_BayerBG2BGR_EA, COLOR_BayerBG2BGR_VNG, COLOR_BayerBG2GRAY, COLOR_BayerBG2RGB, COLOR_BayerBG2RGBA, COLOR_BayerBG2RGB_EA, COLOR_BayerBG2RGB_VNG, COLOR_BayerGB2BGR, COLOR_BayerGB2BGRA, COLOR_BayerGB2BGR_EA, COLOR_BayerGB2BGR_VNG, COLOR_BayerGB2GRAY, COLOR_BayerGB2RGB, COLOR_BayerGB2RGBA, COLOR_BayerGB2RGB_EA, COLOR_BayerGB2RGB_VNG, COLOR_BayerGR2BGR, COLOR_BayerGR2BGRA, COLOR_BayerGR2BGR_EA, COLOR_BayerGR2BGR_VNG, COLOR_BayerGR2GRAY, COLOR_BayerGR2RGB, COLOR_BayerGR2RGBA, COLOR_BayerGR2RGB_EA, COLOR_BayerGR2RGB_VNG, COLOR_BayerRG2BGR, COLOR_BayerRG2BGRA, COLOR_BayerRG2BGR_EA, COLOR_BayerRG2BGR_VNG, COLOR_BayerRG2GRAY, COLOR_BayerRG2RGB, COLOR_BayerRG2RGBA, COLOR_BayerRG2RGB_EA, COLOR_BayerRG2RGB_VNG, COLOR_BGR2Lab, COLOR_BGR2Luv, COLOR_BGR2YCrCb, COLOR_Lab2BGR, COLOR_Lab2LBGR, COLOR_Lab2LRGB, COLOR_Lab2RGB, COLOR_LBGR2Lab, COLOR_LBGR2Luv, COLOR_LRGB2Lab, COLOR_LRGB2Luv, COLOR_Luv2BGR, COLOR_Luv2LBGR, COLOR_Luv2LRGB, COLOR_Luv2RGB, COLOR_mRGBA2RGBA, COLOR_RGB2Lab, COLOR_RGB2Luv, COLOR_RGB2YCrCb, COLOR_RGBA2mRGBA, COLOR_YCrCb2BGR, COLOR_YCrCb2RGB, COLOR_YUV420p2BGR, COLOR_YUV420p2BGRA, COLOR_YUV420p2GRAY, COLOR_YUV420p2RGB, COLOR_YUV420p2RGBA, COLOR_YUV420sp2BGR, COLOR_YUV420sp2BGRA, COLOR_YUV420sp2GRAY, COLOR_YUV420sp2RGB, COLOR_YUV420sp2RGBA, Contour, Draw, FastFeatureDetector, FastLineDetector, GaussianBlur, GFTTDetector, HarrisLaplaceFeatureDetector, HoughCircles, HoughLines, HoughLinesP, HuMoments, KeyPoint, Laplacian, Line, LineSegmentDetector, Mahalanobis, Mat, MSDDetector, Point, PointIterator, Rect, RotatedRect, Scharr, SimpleBlobDetector, Size, SliceIterator, Sobel, StarDetector, Subdiv2D, TickMeter, UMat, VideoCapture, VideoWriter, } from 'opencv';
import process from 'process';
import inspect from 'inspect';
import * as std from 'std';

let rainbow;
let zoom = 1;
let debug = false;
let basename = (process ? process.argv[1] : scriptArgs[1]).replace(/\.js$/, '');

let simplifyMethods = {
  NTH_POINT: c => c.simplifyNthPoint(2),
  RADIAL_DISTANCE: c => c.simplifyRadialDistance(10),
  PERPENDICULAR_DISTANCE: c => c.simplifyPerpendicularDistance(20),
  REUMANN_WITKAM: c => c.simplifyReumannWitkam(),
  OPHEIM: c => c.simplifyOpheim(),
  LANG: c => c.simplifyLang(),
  DOUGLAS_PEUCKER: c => c.simplifyDouglasPeucker(),
};

function Hierarchy(array) {
  if(array instanceof Int32Array)
    this.index = function(id) {
      return this.array.slice(id * 4, id * 4 + 4);
    };
  else
    this.index = function(id) {
      return this.array[id];
    };
  this.array = array;
}

/* prettier-ignore */
Object.assign(Hierarchy.prototype, {
    parent(id) { const a = this.index(id); return a[cv.HIER_PARENT]; },
    child(id) { const a = this.index(id); return a[cv.HIER_CHILD]; },
    next(id) { const a = this.index(id); return a[cv.HIER_NEXT]; },
    prev(id) { const a = this.index(id); return a[cv.HIER_PREV]; }
  });

function getConstants(names) {
  return Object.fromEntries(names.map(name => [name, '0x' + cv[name].toString(16)]));
}

function findConstant(value, keyCond = k => /^CV/.test(k)) {
  return FindKey(cv, (v, k) => v == value && keyCond(k));
}

function findType(value) {
  return findConstant(value, k => /^CV_[0-9]+[A-Z]+C[0-9]/.test(k));
}

function getBitDepth(mat) {
  switch (mat.depth) {
    case cv.CV_8U:
    case cv.CV_8S:
      return 8;
    case cv.CV_16U:
    case cv.CV_16S:
      return 16;
    case cv.CV_32F:
      return 32;
    case cv.CV_64F:
      return 64;
  }
}

const MakeMatFor = WeakMapper((processor, ...args) => new Mat(...args));

function minMax(mat) {
  const ret = cv.minMaxLoc(mat);
  return [ret.minVal, ret.maxVal];
}

function modifierMap(keyCode) {
  return [
    ['shift', 0x10000],
    ['alt', 0x80000],
    ['ctrl', 0x40000],
  ].map(([modifier, flag]) => [modifier, keyCode & flag ? 1 : 0]);
}

function drawContour(mat, contour, color, thickness = 1, lineType = cv.LINE_AA) {
  cv.drawContours(mat, [contour], 0, color, thickness, lineType);
}

function* getParents(hier, id) {
  while(id != -1) {
    yield id;

    id = hier.parent(id);
  }
}

function getContourDepth(hier, id) {
  return [...getParents(hier, id)].length;
}

function findRoot(hier) {
  return hier.findIndex(h => h[cv.HIER_PREV] == -1 && h[cv.HIER_PARENT] == -1);
}

function* getToplevel(hier) {
  for(let [i, h] of hier.entries()) if(h[cv.HIER_PARENT] == -1) yield i;
}

function* walkContours(hier, id) {
  id = id || findRoot(hier);
  let h;

  while((h = hier[id])) {
    yield id;

    if(h[cv.HIER_CHILD] != -1) yield* walkContours(hier, h[cv.HIER_CHILD]);

    id = h[cv.HIER_NEXT];
  }
}

function main(...args) {
  let start;
  let running = true;
  let paused = false;

  globalThis.console = new Console({
    colors: true,
    depth: 1,
    maxArrayLength: 30,
    compact: 1,
    hideKeys: [Symbol.toStringTag],
  });
  let f = std.open('test-video.log', 'w');
  console.log('f.write', f.write);
  globalThis.log = new Console(f, {
    colors: true,
    depth: 1,
    maxArrayLength: 30,
    compact: 1,
  });

  const { DISPLAY } = globalThis.process ? globalThis.process.env : std.getenviron();
  log.info('DISPLAY', DISPLAY);

  let opts = GetOpt(
    {
      help: [
        false,
        () => {
          console.log(`Usage: ${getArgv()[0]} [OPTIONS] <video|device>`);
          exit(0);
        },
        'h',
      ],
      opengl: [false, null, 'g'],
      input: [true, (file, current) => [...(current || []), file], 'i'],
      driver: [
        true,
        (arg, current, options, results) => {
          let driverId = arg in VideoSource.backends ? arg : current;
          log.info('driver', { arg, current, driverId });
          if(driverId === undefined) {
            const input = results['input'];
            let args = [arg];
            if(input) {
              args = results['input'].concat(args);
              results['input'] = undefined;
            }
            results['@'] = results['@'].concat(args);
          }
          return driverId;
        },
        'd',
      ],
      size: [true, null, 's'],
      trackbars: [false, null, 't'],
      'no-trackbars': [false, null, 'T'],
      '@': 'input,driver',
    },
    args,
  );

  log.info('opts:', opts);
  log.info('opts.size:', opts.size);

  const makeRainbow = steps =>
    Range(0, 360, 360 / steps)
      .slice(0, -1)
      .map(hue => new HSLA(hue, 100, 50))
      .map(h => h.toRGBA());

  const printFlags = flags => [...BitsToNames(MouseFlags)];

  const videos = opts['input'] ? [opts['input']] : opts['@'];
  log.info('Creating VideoSource:', videos);
  let video = new VideoSource(...videos);

  if(opts['size']) {
    video.size = new Size(...opts['size'].split('x'));
  }
  log.info('video.size', video.size);

  let win;

  if(opts.opengl) {
    win = new GLFW(video.size.width, video.size.height, {});
  } else {
    win = new Window('gray', cv.WINDOW_NORMAL | cv.WINDOW_KEEPRATIO /*| cv.WINDOW_KEEPRATIO | */);

    win.setMouseCallback(function (event, x, y, flags) {
      event = Mouse.printEvent(event);
      flags = Mouse.printFlags(flags);

      //console.debug('Mouse event:', console.inspect({ event, x, y, flags }, { multiline: false }));
      //console.debug('Mouse :', { MouseEvents, MouseFlags });
    });
  }

  win.resize(video.size);

  let thickness = 1;
  let font = new TextStyle(cv.FONT_HERSHEY_PLAIN, 1.0, thickness);
  let tSize = font.size(video.time);

  let tPos = new Point(...tSize.div(2))
    .floor()
    .mul(-1)
    .add(50, video.get('frame_height') - tSize.y * 0.8);

  tPos.x = 5;

  // let bgr = new Mat();
  log.info('backend:', video.backend);
  log.info('grab():', video.grab);
  log.info('fps:', video.fps);
  let frameCount = video.get('frame_count');
  let { frameShow, ...config } = LoadConfig();
  log.info('frameShow:', frameShow);

  let contours, hier;
  let contoursDepth;
  let lines, circles;
  let outputMat, outputName;

  let params = {
    ksize: new NumericParam(config.ksize || 3, 1, 13, 2),
    thresh1: new NumericParam(config.thresh1 || 40, 0, 100),
    thresh2: new NumericParam(config.thresh2 || 90, 0, 100),
    threshc: new NumericParam(config.threshc || 50, 0, 100),
    angleResolution: new NumericParam(config.angleResolution || 2, 0.5, 180),
    minLineLength: new NumericParam(config.minLineLength || 3, 0, 50),
    maxLineGap: new NumericParam(config.maxLineGap || 1, 0, 50),
    apertureSize: new NumericParam(config.apertureSize || 3, 3, 7, 2),
    L2gradient: new NumericParam(config.L2gradient || 0, 0, 1),
    dilations: new NumericParam(config.dilations || 0, 0, 10),
    erosions: new NumericParam(config.erosions || 0, 0, 10),
    mode: new EnumParam(config.mode || 3, ['RETR_EXTERNAL', 'RETR_LIST', 'RETR_CCOMP', 'RETR_TREE', 'RETR_FLOODFILL']),
    method: new EnumParam(config.method || 0, ['CHAIN_APPROX_NONE', 'CHAIN_APPROX_SIMPLE', 'CHAIN_APPROX_TC89_L1', 'CHAIN_APPROX_TC89_L189_KCOS']),
    maskColor: new EnumParam(config.maskColor || false, ['OFF', 'ON']),
    lineWidth: new NumericParam(config.lineWidth || 1, 0, 10),
    fontThickness: new NumericParam(config.fontThickness || 1, 0, 10),
  };
  let paramNav = new ParamNavigator(params, config.currentParam);
  let dummyArray = [0, 1, 2, 3, 4, 5, 6, 7];
  log.info('win.imageRect (1)', win.imageRect);

  if(opts['trackbars']) {
    params.apertureSize.createTrackbar('apertureSize', win);
    params.thresh1.createTrackbar('thresh1', win);
    params.thresh2.createTrackbar('thresh2', win);
    log.info('win.imageRect (2)', win.imageRect);
  }

  // cv.createButton('apertureSize', arg => log.info("Button apertureSize", arg), 0, false);

  //log.info('paramNav.param:', paramNav.param);
  //await params.apertureSize.createTrackbar('apertureSize', win);

  //std.exit(0);
  rainbow = makeRainbow(256);

  let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));

  let dst0Size, firstSize, videoSize;
  let clahe = new CLAHE(4, new Size(8, 8));
  let framePos;
  let invert = false;

  let pipeline = new Pipeline(
    [
      Processor(function AcquireFrame(src, dst) {
        const dstEmpty = dst.empty;
        if(dst.empty) dst0Size = dst.size;
        // log.info('video', video.read, video.constructor.name);
        framePos = video.get('pos_frames');
        //log.info('video', video.read, video.constructor.name);
        video.read(dst);
        //log.info('dst', dst.size);

        win.show(dst);
        if(videoSize === undefined || videoSize.empty) videoSize = video.size.area ? video.size : dst.size;
        if(dstEmpty) firstSize = new Size(...videoSize);
        if(dst.size && !videoSize.equals(dst.size)) throw new Error(`AcquireFrame videoSize = ${videoSize} firstSize=${firstSize} dst.size = ${dst.size}`);
      }),
      Processor(function Grayscale(src, dst) {
        let channels = [];
        cv.cvtColor(src, dst, cv.COLOR_BGR2Lab);
        cv.split(dst, channels);
        channels[0].copyTo(dst);
      }),
      Processor(function Norm(src, dst) {
        clahe.apply(src, dst);
      }),
      Processor(function Blur(src, dst) {
        cv.GaussianBlur(src, dst, [+params.ksize, +params.ksize], 0, 0, cv.BORDER_REPLICATE);
      }),
      Processor(function EdgeDetect(src, dst) {
        cv.Canny(src, dst, +params.thresh1, +params.thresh2, +params.apertureSize, +params.L2gradient);
        ////log.info('canny dst: ' +inspectMat(dst), [...dst.row(50).values()]);
      }),
      Processor(function Morph(src, dst) {
        cv.dilate(src, dst, structuringElement, new Point(-1, -1), +params.dilations);
        cv.erode(dst, dst, structuringElement, new Point(-1, -1), +params.erosions);
      }),
      Processor(function Contours(src, dst) {
        cv.findContours(src, (contours = []), h => (hier = h), cv[params.mode], cv[params.method]);
        cv.cvtColor(src, dst, cv.COLOR_GRAY2BGR);

        if(+params.maskColor) {
          let edge = [dst.toString(), pipeline.images[0].toString()];
          dst.and(pipeline.images[0]);
        }
      }),
      Processor(function HoughLines(src, dst) {
        let edges = pipeline.outputOf('EdgeDetect');
        let mat = new Mat(0, 0, cv.CV_32SC4);

        cv.HoughLinesP(edges, mat, 2, (+params.angleResolution * Math.PI) / 180, +params.threshc, +params.minLineLength, +params.maxLineGap);
        lines = [...mat]; //.array;
        // log.info('mat', mat);
        //  log.info('lines', lines.slice(0, 10));
        // log.info('lines.length', lines.length);
        src.copyTo(dst);
      }),
    ],
    (i, n) => {
      if(frameShow == i) {
        let mat = pipeline.getImage(i);

        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;
      }
    },
  );

  log.info(`pipeline.images = { ` + pipeline.images.map(image => '\n  ' + image) + '\n}');

  log.info('Pipeline processor names:', pipeline.names);
  let meter = new TickMeter();
  let prevTime;
  let frameDelay = Math.floor(1000 / video.fps);
  //log.info('frameDelay:', frameDelay);

  if(frameShow === undefined) frameShow = Modulo(-1, pipeline.size);

  log.info(`Trackbar 'frame' frameShow=${frameShow} pipeline.size - 1 = ${pipeline.size - 1}`);

  if(opts['trackbars'])
    cv.createTrackbar('frame', 'gray', frameShow, pipeline.size - 1, function(value, count, name, window) {
      //log.info('Trackbar', { value, count, name, window });
      frameShow = value;
    });

  const resizeOutput = Once(() => {
    let size = outputMat.size.mul(zoom);
    win.resize(size.width, Math.floor(size.height * 1.5));
  });

  let out = new Mat();
  let size;
  let clientRect = win.imageRect;

  const ClearSurface = mat => (mat.setTo([0, 0, 0, 0]), mat);
  const MakeSurface = () => Once((...args) => new Mat(...(args.length == 2 ? args.concat([cv.CV_8UC4]) : args)), null, ClearSurface);
  const MakeComposite = Once(() => new Mat());
  let surface = MakeSurface();
  let keyCode,
    keyTime = Date.now(),
    modifiers,
    modifierList;

  let paramNames = Object.keys(params);
  let paramIndex = -1;
  let controlMap = memoize(controlNumber => {
    let paramName = paramNames[++paramIndex];
    let param = params[paramName];

    console.log('control #' + controlNumber.toString(16) + ' mapped to ' + paramName);
    return value => (param.alpha = value / 127);
  });

  let midi = new TCPClient('tcp://127.0.0.1:6999', event => {
    const { type, param1, param2, channel } = event;

    let control = (channel << 4) | param1;
    let value = param2;

    console.log('MIDI event', { control, value });

    controlMap(control)(value);
  });

  log.info('midi client:', midi);

  while(running) {
    meter.reset();
    meter.start();
    let deadline = Date.now() + frameDelay;
    //console.log('prevTime', prevTime);

    let frameNo = video.get('pos_frames');
    if(frameNo == frameCount) video.set('pos_frames', (frameNo = 0));

    let gray = pipeline();

    //console.log('#0 deadline - Date.now()', deadline - Date.now());

    if(!win.imageRect.equals(clientRect)) {
      log.info(`resized from ${clientRect} to ${win.imageRect}`);
      clientRect = win.imageRect;
    }

    //console.log('#1 deadline - Date.now()', deadline - Date.now());
    showOutput();
    //console.log('#2 deadline - Date.now()', deadline - Date.now());

    while(true) {
      let now = Date.now();
      let sleepMsecs = deadline - now;

      sleepMsecs -= 2;
      let key;
      sleepMsecs = Math.max(1, sleepMsecs);
      //console.log('sleepMsecs',sleepMsecs);
      //
      if((key = cv.waitKeyEx(sleepMsecs)) != -1) {
        keyCode = key;
        keyTime = Date.now();
        modifiers = Object.fromEntries(modifierMap(keyCode));
        modifierList = modifierMap(keyCode).reduce((acc, [modifier, active]) => (active ? [...acc, modifier] : acc), []);
        let ch = String.fromCodePoint(keyCode & 0xff);
        log.info(`keypress [${modifierList}] 0x${(keyCode & ~0xd000).toString(16)} '${ch}'`);
      }
      let keyChar = String.fromCodePoint(key & 0xfff);

      switch (keyChar) {
        case '<' /* < */:
          paramNav.prev();
          break;
        case '>' /* > */:
          paramNav.next();
          break;

        case '\x2b' /* + */:
          paramNav.param.increment();
          break;
        case '\x2d' /* - */:
        case '\x2f' /* numpad - */:
          paramNav.param.decrement();
          break;

        case '1': /* 1 */
        case '2': /* 2 */
        case '3': /* 3 */
        case '4': /* 4 */
        case '5': /* 5 */
        case '6': /* 6 */
        case '7': /* 7 */
        case '8': /* 8 */
        case '9': /* 9 */
        case '0' /* 0 */:
          let v = keyCode & 0xf || 10;
          paramNav.param.alpha = v / 10;
          break;
        case '§' /* § */:
          paramNav.param.alpha = 0;
          break;
        case 'Q': /* Q */
        case 'q': /* q */

        case '\x1b':
          running = false;
          break;
        case ' ':
          paused = !paused;
          break;
        case 'n': /* n */
        case 'N' /* N */:
          frameShow = Modulo(frameShow + 1, pipeline.size);
          break;
        case 'p': /* p */
        case 'P' /* P */:
          frameShow = Modulo(frameShow - 1, pipeline.size);
          break;
        case '\u0f50' /* home */:
          video.set('pos_frames', 0);
          break;
        case '\u0f57' /* end */:
          video.set('pos_frames', video.get('frame_count') - Math.round(video.fps * 3));
          break;
        case '\u0f51': /* left */
        case '\u0f53': /* right */
        case '\u0f52': /* up */
        case '\u0f54': /* down */ {
          const method = keyCode & 0x1 ? 'Frames' : 'Msecs';
          const mod = parseInt(['ctrl', 'shift'].map(n => modifiers[n] | 0).join(''), 2);

          const distance = (keyCode & 0x1 ? 1 : 1000) * (modifiers['ctrl'] ? 1000 : modifiers['shift'] ? 100 : modifiers['alt'] ? 1 : 10);

          const offset = keyCode & 0x2 ? +distance : -distance;

          //log.info('seek', { method, distance, offset });
          video['seek' + method](offset);
          let pos = video.position(method);

          log.info('seek' + method + ' ' + offset + ` distance = ${distance} pos = ${video.position('frames')} time = \x1b[1;36m${video.position('ms')}\x1b[0m (${video.position('%').toFixed(2)}%)`);
          break;
        }
        case 'i' /* invert */:
          invert = !invert;
          break;

        case 's': /* save */ {
          log.info('contours.length', contours.length);
          saveContours(contours, outputMat.size);
          saveLines(lines, outputMat.size);
          break;
        }
        default: {
          if(keyCode !== undefined && key != -1) {
            //log.info('unhandled', console.config({ compact: 2, numberBase: 16 }), { key, keyCode, modifiers });
            //
          }
          break;
        }
      }
      if(sleepMsecs <= 1) break;
    }

    std.gc();

    if(paused) video.seekFrames(-1);

    meter.stop();

    prevTime = meter.timeSec;
  }

  function showOutput() {
    let over = surface(outputMat.rows, outputMat.cols, cv.CV_8UC4);
    let now = Date.now();

    out = outputMat;
    if(outputMat.channels == 1) {
      cv.cvtColor(out, out, cv.COLOR_GRAY2BGRA);
    } else {
      cv.cvtColor(out, out, cv.COLOR_BGR2BGRA);
    }
    const processor = pipeline.processors[frameShow];

    if(processor.functionName == 'HoughLines') {
      for(let line of lines) {
        const { a, b } = line;
        Draw.line(over, line.a, line.b, { r: 255, g: 0, b: 0, a: 255 }, 2, cv.LINE_AA, 0);
      }
    } else if(frameShow == 0 || frameShow == 7) {
      cv.drawContours(over, contours, -1, { r: 0, g: 255, b: 0, a: 255 }, 1, cv.LINE_AA);
    } else {
      let ids = [...getToplevel(hier)];

      let palette = Object.fromEntries([...ids.entries()].map(([i, id]) => [id, rainbow[Math.floor((i * 256) / (ids.length - 1))]]));
      let hierObj = new Hierarchy(hier);
    }
    font.draw(over, video.time + ' ⏩', tPos, { r: 0, g: 255, b: 0, a: 255 }, +params.fontThickness);

    function drawParam(param, y, color) {
      const name = paramNav.nameOf(param);
      const value = param.get() + (param.get() != (param | 0) + '' ? ` (${+param})` : '');
      const arrow = Number.isInteger(y) && paramNav.name == name ? '=>' : '  ';
      const text = `${arrow}${name}` + (Number.isInteger(y) ? `[${param.range.join('-')}]` : '') + ` = ${value}`;
      color = color || {
        r: 0xb7,
        g: 0x35,
        b: 255,
        a: 255,
      };
      y = tPos.y - 20 - (y | 0);
      font.draw(over, text, [tPos.x, y], { r: 0, g: 0, b: 0, a: 255 }, params.fontThickness * 2);
      font.draw(over, text, [tPos.x, y], color, +params.fontThickness);
    }

    let elapsed = now - keyTime;
    let maskRect;
    if((keyCode & 0x3d) == 0x3c && elapsed < 2000) {
      let { index, size } = paramNav;
      let start = 0;
      if(index > 4) {
        start = index - 4;
      }
      let trailing = size - (index + 1);
      if(trailing > 4) {
        size -= trailing - 4;
      }
      let y = 0;
      let h = 0;
      for(let i = size - 1; i >= start; i--) h += 20;
      maskRect = new Rect(tPos.x, tPos.y - 20 - h, 200, h);

      for(let i = size - 1; i >= start; i--) {
        const pos = Modulo(i, size);
        const [name, param] = paramNav.at(pos);
        drawParam(
          param,
          y,
          paramNav.index == pos && {
            r: 255,
            g: 255,
            b: 0,
            a: 255,
          },
        );
        y += 20;
      }
    } else {
      drawParam(paramNav.param);
    }

    font.draw(
      over,
      `#${frameShow + 1}/${pipeline.size}` + (outputName ? ` (${outputName})` : ''),
      [5, 5 + tSize.y],
      {
        r: 255,
        g: 255,
        b: 0,
        a: 255,
      },
      +params.fontThickness,
    );

    //resizeOutput();

    const showOverlay = frameShow != pipeline.size - 1 || now - keyTime < 2000;
    if(maskRect && showOverlay) {
      Draw.rectangle(out, maskRect, [0, 0, 0, 255], -1);
      Draw.rectangle(out, maskRect, [255, 255, 255, 255], 1);
    }
    let composite = MakeMatFor(showOutput);

    if(invert) {
      //      over.xor([255,255,255,0]);
      for(let pixel of over) {
        if(pixel[3] > 0) {
          pixel[0] = 255 - pixel[0];
          pixel[1] = 255 - pixel[1];
          pixel[2] = 255 - pixel[2];
        }
      }
    }
    cv.addWeighted(out, 1, over, showOverlay ? 1 : 0, 0, composite);
    if(maskRect && showOverlay) {
      Draw.rectangle(composite, maskRect, [255, 255, 255, 255], 1);
    }

    win.show(composite);
  }

  function saveContours(contours, size) {
    let points = contours.reduce((acc, contour, i) => {
      //log.info('contour #' + i, contour);
      //contour =simplifyMethods.PERPENDICULAR_DISTANCE(contour);
      //contour = simplifyMethods.RADIAL_DISTANCE(contour);
      let array = contour.toArray();
      //log.info('array #' + i, array.length);
      if(array.length >= 3) {
        let sp = new SvgPath();
        sp.abs();

        for(let i = 0; i < array.length; i += 1) {
          const { x, y } = array[i];
          sp[i == 0 ? 'to' : 'line'](x, y);
        }
        let rsp = sp.toRelative();
        acc.push(rsp.str(2, ' ', ','));
      }
      return acc;
    }, []);
    let children = points.map(d => ({
      tagName: 'path',
      attributes: { d },
    }));
    let viewBox = [0, 0, ...size].join(' ');
    let doc = {
      tagName: 'svg',
      children: [{ tagName: 'g', attributes: { stroke: 'black', fill: 'none' }, children }],
      attributes: {
        xmlns: 'http://www.w3.org/2000/svg',
        viewBox,
      },
    };
    WriteJSON('contours-' + framePos + '.json', doc);
    SaveSVG('contours-' + framePos + '.svg', doc);
  }

  function saveLines(lines, size) {
    let viewBox = [0, 0, ...size].join(' ');
    let children = lines
      .map(coords => new Line(...coords))
      .map(([x1, y1, x2, y2]) => ({
        tagName: 'line',
        attributes: { x1, y1, x2, y2 },
      }));
    let doc = {
      tagName: 'svg',
      children: [{ tagName: 'g', attributes: { stroke: 'black', fill: 'none' }, children }],
      attributes: {
        xmlns: 'http://www.w3.org/2000/svg',
        viewBox,
      },
    };

    WriteJSON('lines-' + framePos + '.json', doc);
    SaveSVG('lines-' + framePos + '.svg', doc);
  }

  const { ksize, thresh1, thresh2, apertureSize, L2gradient, dilations, erosions, mode, method, lineWidth, minLineLength, maxLineGap } = params;
  SaveConfig(
    Object.entries({
      frameShow,
      ksize,
      thresh1,
      thresh2,
      apertureSize,
      L2gradient,
      dilations,
      erosions,
      mode,
      method,
      lineWidth,
      minLineLength,
      maxLineGap,
      currentParam: paramNav.index,
    }).reduce((a, [k, v]) => ({ ...a, [k]: +v }), {}),
  );

  for(let mat of Mat.list || []) {
    let stack = Mat.backtrace(mat)
      .filter(frame => frame.functionName != '<anonymous>' && (frame.lineNumber !== undefined || /test-video/.test(frame.fileName)))
      .map(frame => frame.toString())
      .join('\n  ');

    log.info('mat=' + mat.toString() + '\n  ' + stack);
  }
  log.info('props:', video.dump());
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log('FAIL: ', error && error.message, error && error.stack ? '\n' + error.stack : '');
  std.exit(1);
}

console.log('SUCCESS');
