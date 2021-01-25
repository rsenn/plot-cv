import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import * as draw from 'draw';
import * as std from 'std';
import { Point } from 'point';
import { Size } from 'size';
import { TickMeter } from 'utility';
import { VideoSource } from './cvVideo.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './cvHighGUI.js';
import { Alea } from './lib/alea.js';
import { HSLA } from './lib/color.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import PortableFileSystem from './lib/filesystem.js';
import { Mat as cvMat } from 'mat';
import * as cv from 'cv';

let prng = new Alea(Date.now());
const hr = Util.hrtime;
let filesystem;
let rainbow;
let zoom = 1;

const Mat =
  cvMat ||
  class Mat extends cvMat {
    static map = Util.weakMapper(() => []);
    static list = new Set();

    constructor(...args) {
      super(...args);

      let stack = Mat.map(this);
      for(let frame of Util.stack(null, 3)) stack.push(frame);

      Mat.list.add(this);
    }

    static backtrace(mat) {
      let array = Mat.map(mat);
      return array.length ? array : null;
    }
  };

class Pipeline extends Function {
  constructor(processors = [], callback) {
    let self;
    self = function(mat) {
      let i = 0;
      for(let processor of self.processors) {
        let start = hr();
        mat = processor.call(self, mat, self.images[i], i);
        if(mat) self.images[i] = mat;
        self.times[i] = hr(start);
        if(typeof callback == 'function') callback.call(self, self.images[i], i, self.processors.length);
        i++;
      }
      return mat;
    };
    Util.define(self, {
      processors,
      images: new Array(processors.length),
      callback
    });
    self.times = new Array(processors.length);
    return Object.setPrototypeOf(self, Pipeline.prototype);
    //return Object.assign(self, Pipeline.prototype);
  }

  get size() {
    return this.processors.length;
  }
  get names() {
    return this.processors.map((p) => p.name);
  }
}

function Processor(fn, ...args) {
  let self;
  let mapper = Util.weakMapper(() => new Mat());

  self = function(mat, out, i) {
    if(!out) out = mapper(self);

    fn.call(this, mat, out, ...args);
    return out;
  };
  Util.define(self, { name: Util.fnName(fn) });
  Object.setPrototypeOf(self, Processor.prototype);
  return self;
}
Object.setPrototypeOf(Processor.prototype, Function.prototype);
Object.assign(Pipeline.prototype, {
  setName(name) {
    this.name = name;
  }
});

let symbols = [
  [
    { x: 28.703, y: 28.665 },
    { x: 28.703, y: 21.77 },
    { x: 35.565, y: 21.77 },
    { x: 35.565, y: 28.664 },
    { x: 28.703, y: 28.665 }
  ],
  [
    { x: 34.527, y: 8.103 },
    { x: 34.527, y: 0 },
    { x: 42.604, y: 4.051 },
    { x: 34.527, y: 8.103 }
  ],
  [
    { x: 34.527, y: 18.676 },
    { x: 34.527, y: 10.593999999999998 },
    { x: 38.428, y: 10.593999999999998 },
    { x: 38.428, y: 18.676 },
    { x: 34.527, y: 18.676 }
  ],
  [
    { x: 28.703, y: 39.843 },
    { x: 28.703, y: 31.76 },
    { x: 32.604, y: 31.76 },
    { x: 32.604, y: 39.843 },
    { x: 28.703, y: 39.843 }
  ]
];

const inspectObj = obj => console.inspect(obj, { multiline: false });
const inspectMat = ({ rows, cols, channels, depth, type }) =>
  inspectObj({
    rows,
    cols,
    channels,
    depth,
    type
  });

function SaveConfig(configObj) {
  return filesystem.writeFile(Util.getArgv()[1].replace(/\.js$/, '.config.json'),
    JSON.stringify(configObj, null, 2) + '\n'
  );
}

function LoadConfig() {
  let str = filesystem.readFile(Util.getArgv()[1].replace(/\.js$/, '.config.json')).toString();
  console.log('LoadConfig:', str);
  return JSON.parse(str);
}

function dumpMat(name, mat) {
  console.log(`${name} =`,
    Object.create(
      Mat.prototype,
      ['cols', 'rows', 'depth', 'channels', 'type'].reduce((acc, prop) => ({
          ...acc,
          [prop]: { value: mat[prop], enumerable: true }
        }),
        {}
      )
    )
  );
}

function getConstants(names) {
  return Object.fromEntries(names.map(name => [name, '0x' + cv[name].toString(16)]));
}

function findConstant(value, keyCond = k => /^CV/.test(k)) {
  return Util.findKey(cv, (v, k) => v == value && keyCond(k));
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

const MakeMatFor = Util.weakMapper((...args) => new Mat(...args));

const to32bit = (() => {
  const mapper = Util.weakMapper(() => new Mat());
  return function to32bit(mat) {
    const bits = getBitDepth(mat);
    if(bits == 32) return mat;
    let ret = mapper(mat);
    const max = Math.pow(2, bits) - 1;
    const type = cv.CV_32F | ((mat.channels - 1) << 3);

    mat.convertTo(ret, type, 1.0 / max);
    return ret;
  };
})();

const to8bit = (() => {
  const mapper = Util.weakMapper(() => new Mat());
  return function to8bit(mat) {
    const bits = getBitDepth(mat);
    if(bits == 8) return mat;
    let ret = mapper(mat);
    const type = cv.CV_8U | ((mat.channels - 1) << 3);
    mat.convertTo(ret, type, 255);
    return ret;
  };
})();

const Grayscale = (() => {
  const mapper = Util.weakMapper(() => [new Mat(), new Mat(), new Mat(), new Mat()],
    new WeakMap(),
    (obj, mat) => {
      //console.log('Mat for ', obj, ' = ', mat);
    }
  );
  return function Grayscale(mat) {
    let channels = mapper(mat);
    cv.cvtColor(mat, channels[0], cv.COLOR_BGR2Lab);
    cv.split(channels[0], channels);
    return channels[0];
  };
})();

const getAlpha = (() => {
  const mapper = Util.weakMapper(() => [new Mat(), new Mat(), new Mat(), new Mat()]);
  return function getAlpha(mat) {
    let channels = mapper(mat);
    cv.split(mat, channels);
    return channels[3];
  };
})();

const alphaToMask = (() => {
  const mapper = Util.weakMapper(() => new Mat());
  return function alphaToMask(mat) {
    let mask = mapper(mat);
    cv.merge([mat, mat, mat], mask);
    return mask;
  };
})();

const toBGR = (() => {
  const mapper = Util.weakMapper(() => new Mat());
  return function toBGR(mat) {
    if(mat.channels >= 3) return mat.dup();
    let bgr = mapper(mat);
    cv.cvtColor(mat, bgr, cv.COLOR_GRAY2BGR);
    return bgr;
  };
})();

const toRGBA = (() => {
  const mapper = Util.weakMapper(() => new Mat());
  return function toRGBA(mat) {
    if(mat.channels == 4) return mat.dup();
    let rgba = mapper(mat);
    cv.cvtColor(mat, rgba, cv.COLOR_BGR2BGRA);
    return rgba;
  };
})();

function minMax(mat) {
  const ret = cv.minMaxLoc(mat);
  return [ret.minVal, ret.maxVal];
}

function modifierMap(keyCode) {
  return [
    ['shift', 0x10000],
    ['alt', 0x80000],
    ['ctrl', 0x40000]
  ].map(([modifier, flag]) => [modifier, keyCode & flag ? 1 : 0]);
}

function drawContour(mat, contour, color, thickness = 1, lineType = cv.LINE_AA) {
  cv.drawContours(mat, [contour], 0, color, thickness, lineType);
}

function* getParents(hier, contourId) {
  while(contourId != -1) {
    yield contourId;

    contourId = hier[contourId][cv.HIER_PARENT];
  }
}
function getContourDepth(hier, contourId) {
  return [...getParents(hier, contourId)].length;
}
function findRoot(hier) {
  return hier.findIndex(h => h[cv.HIER_PREV] == -1 && h[cv.HIER_PARENT] == -1);
}
function* getToplevel(hier) {
  for(let [i, h] of hier.entries()) if(h[cv.HIER_PARENT] == -1) yield i;
}
function* walkContours(hier, contourId) {
  contourId = contourId || findRoot(hier);
  let h;

  while((h = hier[contourId])) {
    yield contourId;

    if(h[cv.HIER_CHILD] != -1) yield* walkContours(hier, h[cv.HIER_CHILD]);

    contourId = h[cv.HIER_NEXT];
  }
}

function Profiler(name, ticks = () => cv.getTickCount(), freq = cv.getTickFrequency()) {
  let self,
    i = 0,
    prev,
    start = ticks();

  self = function(label = `#${i}`) {
    let t = ticks();
    let split = t - (prev || start);
    if(prev) console.log(`${name} ${printTime(split).padEnd(6)} ${label}`);
    i++;
    return (prev = t);
  };

  Util.define(self, {
    get elapsed() {
      let t = ticks();
      return t - start;
    },
    get lap() {
      let t = ticks();
      return t - (prev || start);
    }
  });

  function printTime(t) {
    let time, unit;
    if(t < freq * 1e-7) {
      time = t / (freq * 1e-9);
      unit = 'ns';
    } else if(t < freq * 1e-4) {
      time = t / (freq * 1e-6);
      unit = '\u00b5s';
    } else if(t < freq * 1e-1) {
      time = t / (freq * 1e-3);
      unit = 'ms';
    } else {
      time = t / freq;
      unit = 's';
    }
    return time.toFixed(4 - Math.max(1, Math.ceil(Math.log10(time)))) + unit;
  }

  return self;
}

async function main(...args) {
  let start;
  let begin = hr();
  let running = true;
  let paused = false;

  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20,
    multiline: 1,
    alignMap: true
  });
  await PortableFileSystem(fs => (filesystem = fs));

  const makeRainbow = steps =>
    Util.range(0, 360, 360 / steps)
      .slice(0, -1)
      .map(hue => new HSLA(hue, 100, 50))
      .map(h => h.toRGBA());

  let win = new Window('gray', /*cv.WINDOW_AUTOSIZE | cv.WINDOW_NORMAL  |*/ cv.WINDOW_KEEPRATIO);
  console.log('Mouse :', { MouseEvents, MouseFlags });
  //console.log('cv.EVENT_MOUSEMOVE', cv.EVENT_MOUSEMOVE);
  //
  const printFlags = flags => [...Util.bitsToNames(MouseFlags)];
  console.log('printFlags:', printFlags + '');
  console.log('tickFrequency:', cv.getTickFrequency());

  win.setMouseCallback(function (event, x, y, flags) {
    event = Mouse.printEvent(event);
    flags = Mouse.printFlags(flags);

    console.log('Mouse event:', console.inspect({ event, x, y, flags }, { multiline: false }));
  });

  console.log('Setup duration:', hr(begin));

  let video = new VideoSource(...args);
  let thickness = 1;
  let font = new TextStyle(cv.FONT_HERSHEY_PLAIN, 1.0, thickness);
  let tSize = font.size(video.time);

  let tPos = new Point(...tSize.div(2))
    .floor()
    .mul(-1)
    .add(50, video.get('frame_height') - tSize.y * 0.8);

  tPos.x = 5;

  // let bgr = new Mat();
  console.log('backend:', video.backend);
  console.log('grab():', video.grab);
  console.log('fps:', video.fps);
  //console.log('read():', [...Util.repeat(10, () => video.grab())]);
  let frameCount = video.get('frame_count');
  let { frameShow, ...config } = LoadConfig();
  console.log('frameShow:', frameShow);
  let contours, hier;
  let contoursDepth;

  let params = {
    ksize: new NumericParam(config.ksize ?? 3, 1, 13, 2),
    thresh1: new NumericParam(config.thresh1 ?? 40, 0, 100),
    thresh2: new NumericParam(config.thresh2 ?? 90, 0, 100),
    apertureSize: new NumericParam(config.apertureSize ?? 3, 3, 7, 2),
    L2gradient: new NumericParam(config.L2gradient ?? 0, 0, 1),
    numDilations: new NumericParam(config.numDilations ?? 0, 0, 10),
    numErosions: new NumericParam(config.numErosions ?? 0, 0, 10),
    mode: new EnumParam(config.mode ?? 3, ['RETR_EXTERNAL', 'RETR_LIST', 'RETR_CCOMP', 'RETR_TREE', 'RETR_FLOODFILL']),
    method: new EnumParam(config.method ?? 0, [
      'CHAIN_APPROX_NONE',
      'CHAIN_APPROX_SIMPLE',
      'CHAIN_APPROX_TC89_L1',
      'CHAIN_APPROX_TC89_L189_KCOS'
    ]),
    maskColor: new EnumParam(config.maskColor ?? false, ['OFF', 'ON']),
    lineWidth: new NumericParam(config.lineWidth ?? 1, 0, 10),
    fontThickness: new NumericParam(config.fontThickness ??  1, 0, 10)
  };
  let paramNav = new ParamNavigator(params, config.currentParam);
  let dummyArray = [0, 1, 2, 3, 4, 5, 6, 7];
  //console.log('paramNav.map:', paramNav.map);
  //console.log('params.mode:', dummyArray[params.mode]);
  //console.log('params.method:', dummyArray[params.method]);
  //console.log('paramNav.param:', paramNav.param);

  rainbow = makeRainbow(256);

  let outputMat, outputName;
  let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));

  let pipeline = new Pipeline([
      Processor(function AcquireFrame(mat, output) {
        video.read(output);
      }),
      Grayscale,
      Processor(function Norm(src, dst) {
        cv.normalize(src, dst, 255, 0, cv.NORM_MINMAX);
      }),
      Processor(function Blur(src, dst) {
        cv.GaussianBlur(src, dst, [+params.ksize, +params.ksize], 0, 0, cv.BORDER_REPLICATE);
      }),
      Processor(function EdgeDetect(src, dst) {
        cv.Canny(src, dst, +params.thresh1, +params.thresh2, +params.apertureSize, +params.L2gradient);
        ////console.log('canny dst: ' +inspectMat(dst), [...dst.row(50).values()]);
      }),
      Processor(function Morph(src, dst) {
        cv.dilate(src, dst, structuringElement, new Point(-1, -1), +params.numDilations);
        cv.erode(dst, dst, structuringElement, new Point(-1, -1), +params.numErosions);
      }),
      Processor(function Contours(src, dst) {
        cv.findContours(src, (contours = []), (hier = []), cv[params.mode], cv[params.method]);

        cv.cvtColor(src, dst, cv.COLOR_GRAY2BGR);

        if(+params.maskColor) {
          let edge = [dst.toString(), pipeline.images[0].toString()];
          console.log('edge', edge);

          dst.and(pipeline.images[0]);
        }
      })
    ],
    (mat, i, n) => {
      if(frameShow == i) {
        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;
      }

      // let m = (outputMat || mat) ?  (outputMat || mat).dup() : null;
    }
  );

  console.log('Pipeline processor names:', pipeline.names);
  video.seekMsecs(5000);
  let meter = new TickMeter();
  let prevTime;
  let frameDelay = Math.floor(1000 / video.fps);
  console.log('frameDelay:', frameDelay);

  const wrapIndex = Util.mod(pipeline.size);

  if(frameShow === undefined) frameShow = wrapIndex(-1);

  const resizeOutput = Util.once(() => {
    let size = outputMat.size.mul(zoom);
    win.resize(size.width, size.height);
  });

  let out = new Mat();
  let size;

  const ClearSurface = mat => (mat.setTo([0, 0, 0, 0]), mat);
  const MakeSurface = () =>
    Util.once((...args) => new Mat(...(args.length == 2 ? args.concat([cv.CV_8UC4]) : args)), null, ClearSurface);
  const MakeComposite = Util.once(() => new Mat());
  let surface = MakeSurface();
  let keyCode,
    keyTime = Util.now(),
    modifiers,
    modifierList;

  while(running) {
    meter.reset();
    meter.start();
    let deadline = Util.now() + frameDelay;

    let frameNo = video.get('pos_frames');
    if(frameNo == frameCount) video.set('pos_frames', (frameNo = 0));

    let gray = pipeline();
    //console.log('outputMat: ' + outputMat.toString());

    //console.log('pipeline.times:', pipeline.times.map((t, i) => [pipeline.names[i], +t.milliseconds.toFixed(3)]));
    //console.log('pipeline.images:', pipeline.images.map((mat, i) => [pipeline.names[i], mat.toString()]));
    showOutput();
    //console.log('processing time:', Util.now() - (deadline - frameDelay));

    while(true) {
      let sleepMsecs = deadline - Util.now();

      sleepMsecs -= 2;
      //console.log('sleepMsecs:', sleepMsecs, '/', frameDelay);

      if((keyCode = cv.waitKeyEx(Math.max(1, sleepMsecs))) != -1) {
        keyTime = Util.now();
        modifiers = Object.fromEntries(modifierMap(keyCode));
        modifierList = modifierMap(keyCode).reduce((acc, [modifier, active]) => (active ? [...acc, modifier] : acc),
          []
        );
        let ch = String.fromCodePoint(keyCode & 0xff);
        console.log(`keypress [${modifierList}] 0x${(keyCode & ~0xd000).toString(16)} '${ch}'`);
      }

      switch (keyCode & 0xfff) {
        case 0x3c /* < */:
          paramNav.prev();
          break;
        case 0x3e /* > */:
          paramNav.next();
          break;

        case 0x2b /* + */:
          paramNav.param.increment();
          break;
        case 0x2d /* - */:
        case 0x2fad /* numpad - */:
          paramNav.param.decrement();
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
          let v = keyCode & 0xf || 10;
          paramNav.param.alpha = v / 10;
          break;
        case 0xa7 /* § */:
          paramNav.param.alpha = 0;
          break;
        case 0x51: /* Q */
        case 0x71: /* q */

        case 0x1b:
          running = false;
          break;
        case 0x20:
          paused = !paused;
          break;
        case 0x6e: /* n */
        case 0x4e /* N */:
          frameShow = wrapIndex(frameShow + 1);
          break;
        case 0x70: /* p */
        case 0x50 /* P */:
          frameShow = wrapIndex(frameShow - 1);
          break;
        case 0xf50 /* home */:
          video.set('pos_frames', 0);
          break;
        case 0xf57 /* end */:
          video.set('pos_frames', video.get('frame_count') - Math.round(video.fps * 3));
          break;
        case 0xf51: /* left */
        case 0xf53: /* right */
        case 0xf52: /* up */
        case 0xf54: /* down */ {
          const method = keyCode & 0x1 ? 'Frames' : 'Msecs';
          const distance =
            (keyCode & 0x1 ? 1 : 1000) *
            (modifiers['ctrl'] ? 1000 : modifiers['shift'] ? 100 : modifiers['alt'] ? 1 : 10);
          const offset = keyCode & 0x2 ? +distance : -distance;

          //console.log('seek', { method, distance, offset });
          video['seek' + method](offset);
          let pos = video.position(method);

          console.log('seek' + method + ' ' + offset + ' pos =',
            pos,
            ` (${Util.roundTo(video.position('%'), 0.001)}%)`
          );
          break;
        }
        default: {
          break;
        }
      }
      if(sleepMsecs <= 0) break;
    }

    if(paused) video.seekFrames(-1);

    meter.stop();
    //if(prevTime !== undefined) console.log('FPS: ', +(1 / meter.timeSec).toFixed(2), '/', video.fps);

    prevTime = meter.timeSec;
  }

  function showOutput() {
    let mat = surface(outputMat.rows, outputMat.cols, cv.CV_8UC4);
    let now = Util.now();

    //console.log('showOutput:', [now, now - keyTime].map(t => Math.floor(t)));
    out = outputMat;
    if(outputMat.channels == 1) {
      cv.cvtColor(out, out, cv.COLOR_GRAY2BGRA);
    } else {
      cv.cvtColor(out, out, cv.COLOR_BGR2BGRA);
    }

    if(frameShow == 0) {
      cv.drawContours(mat, contours, -1, { r: 0, g: 255, b: 0, a: 255 }, 1, cv.LINE_AA);
    } else {
      let ids = [...getToplevel(hier)];

      let palette = Object.fromEntries([...ids.entries()].map(([i, id]) => [id, rainbow[Math.floor((i * 256) / (ids.length - 1))]])
      );
      contours.forEach((contour, i) => {
        let p = [...getParents(hier, i)];
        let color = palette[p[p.length - 1]];
        drawContour(mat, contour, color, +params.lineWidth);
      });
    }
     font.draw(mat, video.time + ' ⏩', tPos, /*0x00ff00 ||*/ { r: 0, g: 255, b: 0, a: 255 }, +params.fontThickness);
    let paramValue = paramNav.param.get();

    if('' + paramValue != '' + +paramNav.param) paramValue += ` (${+paramNav.param})`;

    let paramStr = `${paramNav.name} [${paramNav.param.range.join('-')}] = ${paramValue}`;
    //console.log('paramStr: ', paramStr);
    font.draw(mat, paramStr, [tPos.x, tPos.y - 20], /*0x00ffff ||*/ { r: 255, g: 0, b: 0, a: 255 }, +params.fontThickness);

    font.draw(mat,
      `#${frameShow + 1}/${pipeline.size}` + (outputName ? ` (${outputName})` : ''),
      [5, 5 + tSize.y],
      /*0xffff00 ||*/ {
        r: 255,
        g: 255,
        b: 0,
        a: 255
      }, +params.fontThickness
    );

    resizeOutput();

    //console.log("row 100:", [...mat.row(100).values()]);

    //let mask = toBGR(getAlpha(mat));
    let composite = MakeMatFor(showOutput);
    const showOverlay = frameShow != pipeline.size - 1 || now - keyTime < 2000;
    cv.addWeighted(out, 1, mat, showOverlay ? 1 : 0, 0, composite);
    win.show(composite);

    //console.log("composite", composite.toString());
    //composite.release();
    //console.log("composite.release()", composite.toString());
  }
  const {
    ksize,
    thresh1,
    thresh2,
    apertureSize,
    L2gradient,
    numDilations,
    numErosions,
    mode,
    method,
    lineWidth
  } = params;
  SaveConfig(Object.entries({
      frameShow,
      ksize,
      thresh1,
      thresh2,
      apertureSize,
      L2gradient,
      numDilations,
      numErosions,
      mode,
      method,
      lineWidth,
      currentParam: paramNav.index
    }).reduce((a, [k, v]) => ({ ...a, [k]: +v }), {})
  );

  for(let mat of Mat.list || []) {
    let stack = Mat.backtrace(mat)
      //.slice(0,-1)
      .filter(frame =>
          frame.functionName != '<anonymous>' && (frame.lineNumber !== undefined || /test-video/.test(frame.fileName))
      )
      .map(frame => frame.toString())
      .join('\n  ');

    console.log('mat=' + mat.toString() + '\n  ' + stack);
  }
  console.log('props:', video.dump());
  console.log('gc:', std.gc());
  console.log('exit:', std.exit(0));
}

Util.callMain(main, true);
