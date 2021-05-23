import { Point, Size, Contour, Rect, Line, TickMeter, Mat, CLAHE, Draw } from 'opencv';
import * as cv from 'opencv';
import { VideoSource } from './qjs-opencv/js/cvVideo.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './qjs-opencv/js/cvHighGUI.js';
import { HSLA } from './lib/color.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import fs from 'fs';
import Console from 'console';
import { Pipeline, Processor } from './qjs-opencv/js/cvPipeline.js';
import { WeakMapper, Modulo, WeakAssign, BindMethods, BitsToNames, FindKey, Define, Once, GetOpt, RoundTo, Range } from './qjs-opencv/js/cvUtils.js';

console.log('process', process);

let rainbow;
let zoom = 1;
let debug = false;
let basename = process.argv[1].replace(/\.js$/, '');

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
  let configObj = JSON.parse(str || '{}');

  configObj = Object.fromEntries(Object.entries(configObj)
      .map(([k, v]) => [k, +v])
      .filter(([k, v]) => !isNaN(v))
  );
  console.log('LoadConfig:', inspect(configObj, { compact: false }));
  return configObj;
}

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

const MakeMatFor = WeakMapper((...args) => new Mat(...args));

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

  Define(self, {
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

function main(...args) {
  let start;
  let running = true;
  let paused = false;

  globalThis.console = new Console({ colors: true, depth: 1, maxArrayLength: 30 });

  let opts = GetOpt({
      input: [true, (file, current) => [...(current || []), file], 'i'],
      driver: [
        true,
        (arg, current, options, results) => {
          let driverId = arg in VideoSource.backends ? arg : current;
          console.log('driver', { arg, current, driverId });
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
        'd'
      ],
      size: [true, null, 's'],
      trackbars: [false, null, 't'],
      'no-trackbars': [false, null, 'T'],
      '@': 'input,driver'
    },
    args
  );
  const makeRainbow = steps =>
    Range(0, 360, 360 / steps)
      .slice(0, -1)
      .map(hue => new HSLA(hue, 100, 50))
      .map(h => h.toRGBA());

  let win = new Window('gray', cv.WINDOW_NORMAL /*| cv.WINDOW_AUTOSIZE | cv.WINDOW_KEEPRATIO*/);
  //console.debug('Mouse :', { MouseEvents, MouseFlags });

  const printFlags = flags => [...BitsToNames(MouseFlags)];
  /*console.log('printFlags:', printFlags + '');
  console.log('tickFrequency:', cv.getTickFrequency());*/

  win.setMouseCallback(function (event, x, y, flags) {
    event = Mouse.printEvent(event);
    flags = Mouse.printFlags(flags);

    //console.debug('Mouse event:', console.inspect({ event, x, y, flags }, { multiline: false }));
  });

  const videos = opts['input'] ? [opts['input']] : opts['@'];
  console.log('Creating VideoSource:', videos);
  let video = new VideoSource(...videos);

  //if(!video.isVideo) video.size = new Size(960, 540);

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
  let frameCount = video.get('frame_count');
  let { frameShow, ...config } = LoadConfig();
  console.log('frameShow:', frameShow);

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
    minLineLength: new NumericParam(config.minLineLength || 30, 0, 500),
    maxLineGap: new NumericParam(config.maxLineGap || 10, 0, 500),
    apertureSize: new NumericParam(config.apertureSize || 3, 3, 7, 2),
    L2gradient: new NumericParam(config.L2gradient || 0, 0, 1),
    dilations: new NumericParam(config.dilations || 0, 0, 10),
    erosions: new NumericParam(config.erosions || 0, 0, 10),
    mode: new EnumParam(config.mode || 3, [
      'RETR_EXTERNAL',
      'RETR_LIST',
      'RETR_CCOMP',
      'RETR_TREE',
      'RETR_FLOODFILL'
    ]),
    method: new EnumParam(config.method || 0, [
      'CHAIN_APPROX_NONE',
      'CHAIN_APPROX_SIMPLE',
      'CHAIN_APPROX_TC89_L1',
      'CHAIN_APPROX_TC89_L189_KCOS'
    ]),
    maskColor: new EnumParam(config.maskColor || false, ['OFF', 'ON']),
    lineWidth: new NumericParam(config.lineWidth || 1, 0, 10),
    fontThickness: new NumericParam(config.fontThickness || 1, 0, 10)
  };
  let paramNav = new ParamNavigator(params, config.currentParam);
  let dummyArray = [0, 1, 2, 3, 4, 5, 6, 7];
  console.log('video.size', video.size);
  console.log('win.imageRect (1)', win.imageRect);

  if(opts['trackbars']) {
    params.apertureSize.createTrackbar('apertureSize', win);
    params.thresh1.createTrackbar('thresh1', win);
    params.thresh2.createTrackbar('thresh2', win);
    console.log('win.imageRect (2)', win.imageRect);
  }

  // cv.createButton('apertureSize', arg => console.log("Button apertureSize", arg), 0, false);

  //console.log('paramNav.param:', paramNav.param);
  //await params.apertureSize.createTrackbar('apertureSize', win);

  //std.exit(0);
  rainbow = makeRainbow(256);

  let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));

  let dst0Size, firstSize, videoSize;
  let clahe = new CLAHE(4, new Size(8, 8));

  let pipeline = new Pipeline([
      Processor(function AcquireFrame(src, dst) {
        const dstEmpty = dst.empty;
        if(dst.empty) dst0Size = dst.size;
        console.log('video', video);
        video.read(dst);
        if(videoSize === undefined || videoSize.empty)
          videoSize = video.size.area ? video.size : dst.size;
        if(dstEmpty) firstSize = new Size(...videoSize);
        if(dst.size && !videoSize.equals(dst.size))
          throw new Error(`AcquireFrame videoSize = ${videoSize} firstSize=${firstSize} dst.size = ${dst.size}`
          );
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
        cv.Canny(src,
          dst,
          +params.thresh1,
          +params.thresh2,
          +params.apertureSize,
          +params.L2gradient
        );
        ////console.log('canny dst: ' +inspectMat(dst), [...dst.row(50).values()]);
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
        lines = new Mat(0, 0, cv.CV_32SC4);

        cv.HoughLinesP(edges,
          lines,
          2,
          (+params.angleResolution * Math.PI) / 180,
          +params.threshc,
          +params.minLineLength,
          +params.maxLineGap
        );
        src.copyTo(dst);
      })
    ],
    (i, n) => {
      if(frameShow == i) {
        let mat = pipeline.getImage(i);

        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;
      }
    }
  );

  console.log(`pipeline.images = { ` + pipeline.images.map(image => '\n  ' + image) + '\n}');

  console.log('Pipeline processor names:', pipeline.names);
  let meter = new TickMeter();
  let prevTime;
  let frameDelay = Math.floor(1000 / video.fps);
  console.log('frameDelay:', frameDelay);

  if(frameShow === undefined) frameShow = Modulo(-1, pipeline.size);

  console.log(`Trackbar 'frame' frameShow=${frameShow} pipeline.size - 1 = ${pipeline.size - 1}`);

  if(opts['trackbars'])
    cv.createTrackbar('frame',
      'gray',
      frameShow,
      pipeline.size - 1,
      function(value, count, name, window) {
        //console.log('Trackbar', { value, count, name, window });
        frameShow = value;
      }
    );

  const resizeOutput = Once(() => {
    let size = outputMat.size.mul(zoom);
    win.resize(size.width, Math.floor(size.height * 1.5));
  });

  let out = new Mat();
  let size;

  const ClearSurface = mat => (mat.setTo([0, 0, 0, 0]), mat);
  const MakeSurface = () =>
    Once((...args) => new Mat(...(args.length == 2 ? args.concat([cv.CV_8UC4]) : args)),
      null,
      ClearSurface
    );
  const MakeComposite = Once(() => new Mat());
  let surface = MakeSurface();
  let keyCode,
    keyTime = Date.now(),
    modifiers,
    modifierList;

  while(running) {
    meter.reset();
    meter.start();
    let deadline = Date.now() + frameDelay;

    let frameNo = video.get('pos_frames');
    if(frameNo == frameCount) video.set('pos_frames', (frameNo = 0));

    let gray = pipeline();

    showOutput();

    while(true) {
      let now = Date.now();
      let sleepMsecs = deadline - now;

      sleepMsecs -= 2;
      let key;
      if((key = cv.waitKeyEx(Math.max(1, sleepMsecs))) != -1) {
        keyCode = key;
        keyTime = Date.now();
        modifiers = Object.fromEntries(modifierMap(keyCode));
        modifierList = modifierMap(keyCode).reduce((acc, [modifier, active]) => (active ? [...acc, modifier] : acc),
          []
        );
        let ch = String.fromCodePoint(keyCode & 0xff);
        console.log(`keypress [${modifierList}] 0x${(keyCode & ~0xd000).toString(16)} '${ch}'`);
      }

      switch (key & 0xfff) {
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
          frameShow = Modulo(frameShow + 1, pipeline.size);
          break;
        case 0x70: /* p */
        case 0x50 /* P */:
          frameShow = Modulo(frameShow - 1, pipeline.size);
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

          console.log('seek' +
              method +
              ' ' +
              offset +
              ` distance = ${distance} pos = ${pos} (${RoundTo(video.position('%'), 0.001)}%)`
          );
          break;
        }
        default: {
          if(keyCode !== undefined && key != -1)
            console.log('unhandled', console.config({ numberBase: 16 }), {
              key,
              keyCode,
              modifiers
            });
          break;
        }
      }
      if(sleepMsecs <= 0) break;
    }

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

      let palette = Object.fromEntries([...ids.entries()].map(([i, id]) => [id, rainbow[Math.floor((i * 256) / (ids.length - 1))]])
      );
      let hierObj = new Hierarchy(hier);
    }
    font.draw(over,
      video.time + ' ⏩',
      tPos,
      { r: 0, g: 255, b: 0, a: 255 },
      +params.fontThickness
    );

    function drawParam(param, y, color) {
      const name = paramNav.nameOf(param);
      const value = param.get() + (param.get() != (param | 0) + '' ? ` (${+param})` : '');
      const arrow = Number.isInteger(y) && paramNav.name == name ? '=>' : '  ';
      const text =
        `${arrow}${name}` +
        (Number.isInteger(y) ? `[${param.range.join('-')}]` : '') +
        ` = ${value}`;
      color = color || { r: 0xb7, g: 0x35, b: 255, a: 255 };
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
        drawParam(param, y, paramNav.index == pos && { r: 255, g: 255, b: 0, a: 255 });
        y += 20;
      }
    } else {
      drawParam(paramNav.param);
    }

    font.draw(over,
      `#${frameShow + 1}/${pipeline.size}` + (outputName ? ` (${outputName})` : ''),
      [5, 5 + tSize.y],
      {
        r: 255,
        g: 255,
        b: 0,
        a: 255
      },
      +params.fontThickness
    );

    //resizeOutput();

    const showOverlay = frameShow != pipeline.size - 1 || now - keyTime < 2000;
    if(maskRect && showOverlay) {
      Draw.rect(out, maskRect, [0, 0, 0, 255], -1);
      Draw.rect(out, maskRect, [255, 255, 255, 255], 1);
    }
    let composite = MakeMatFor(showOutput);

    cv.addWeighted(out, 1, over, showOverlay ? 1 : 0, 0, composite);
    if(maskRect && showOverlay) {
      Draw.rect(composite, maskRect, [255, 255, 255, 255], 1);
    }

    win.show(composite);
  }
  const {
    ksize,
    thresh1,
    thresh2,
    apertureSize,
    L2gradient,
    dilations,
    erosions,
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
      dilations,
      erosions,
      mode,
      method,
      lineWidth,
      currentParam: paramNav.index
    }).reduce((a, [k, v]) => ({ ...a, [k]: +v }), {})
  );

  for(let mat of Mat.list || []) {
    let stack = Mat.backtrace(mat)
      .filter(frame =>
          frame.functionName != '<anonymous>' &&
          (frame.lineNumber !== undefined || /test-video/.test(frame.fileName))
      )
      .map(frame => frame.toString())
      .join('\n  ');

    console.log('mat=' + mat.toString() + '\n  ' + stack);
  }
  console.log('props:', video.dump());
  std.gc();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}
