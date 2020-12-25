import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import * as cv from 'cv';
import * as draw from 'draw';
import { Mat } from 'mat';
import { Point } from 'point';
import { VideoSource } from './cvVideo.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './cvHighGUI.js';
import { Alea } from './lib/alea.js';
import { HSLA } from './lib/color.js';
import { NumericParam, EnumParam } from './param.js';

let prng = new Alea(Date.now());
const hr = Util.hrtime;

let rainbow;
let zoom = 1;

function Pipeline(processors = [], callback) {
  let self;
  self = function(mat) {
    let i = 0;
    for(let processor of self.processors) {
      let start = hr();
      mat = processor.call(self, mat, self.images[i], i);
      if(mat) self.images[i] = mat;
      self.times[i] = hr(start);
      if(typeof callback == 'function')
        callback.call(self, self.images[i], i, self.processors.length);
      i++;
    }
    return mat;
  };
  Util.define(self, { processors, images: new Array(processors.length), callback });
  self.times = new Array(processors.length);
  return Object.setPrototypeOf(self, Pipeline.prototype);
  return Object.assign(self, Pipeline.prototype);
}

Object.setPrototypeOf(Pipeline.prototype, Function.prototype);

Object.defineProperties(Pipeline.prototype, {
  size: {
    get() {
      return this.processors.length;
    }
  },
  names: {
    get() {
      return this.processors.map(p => p.name);
    }
  }
});

function Processor(fn, ...args) {
  let self;
  self = function(mat, out, i) {
    if(!out) out = new Mat();
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

function dumpMat(name, mat) {
  console.log(`${name} =`,
    Object.create(
      Mat.prototype,
      ['cols', 'rows', 'depth', 'channels', 'type'].reduce((acc, prop) => ({ ...acc, [prop]: { value: mat[prop], enumerable: true } }),
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

function to32bit(mat) {
  const bits = getBitDepth(mat);
  if(bits == 32) return mat;
  let ret = new Mat();
  const max = Math.pow(2, bits) - 1;
  const type = cv.CV_32F | ((mat.channels - 1) << 3);

  /* console.log('max:', max);
  console.log('const:', findType(type), type, cv[findType(type)]);
  console.log('channels:', mat.channels);
  console.log('constants:', getConstants(['CV_32F', 'CV_32FC1', 'CV_32FC3', 'CV_32FC4', 'CV_8U', 'CV_8UC1', 'CV_8UC3', 'CV_8UC4']));*/
  mat.convertTo(ret, type, 1.0 / max);
  return ret;
}

function to8bit(mat) {
  const bits = getBitDepth(mat);
  if(bits == 8) return mat;
  let ret = new Mat();
  const type = cv.CV_8U | ((mat.channels - 1) << 3);
  mat.convertTo(ret, type, 255);
  return ret;
}

function toGrayscale(mat) {
  let lab = new Mat();
  let channels = [new Mat(), new Mat(), new Mat()];
  cv.cvtColor(mat, lab, cv.COLOR_BGR2Lab);
  cv.split(lab, channels);
  return channels[0];
}
function getAlpha(mat) {
  let channels = Util.repeat(mat.channels, () => new Mat());
  cv.split(mat, channels);
  return channels[3];
}
function alphaToMask(mat) {
  let bgr = new Mat();
  //    cv.cvtColor(mat, bgr, cv.COLOR_GRAY2BGR);
  cv.merge([mat, mat, mat], bgr);
  return bgr;
}

function toBGR(mat) {
  if(mat.channels >= 3) return mat.clone();

  let bgr = new Mat();
  cv.cvtColor(mat, bgr, cv.COLOR_GRAY2BGR);
  return bgr;
}
function toRGBA(mat) {
  if(mat.channels == 4) return mat.clone();
  let rgba = new Mat();
  cv.cvtColor(mat, rgba, cv.COLOR_BGR2BGRA);
  return rgba;
}

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

function drawContour(mat, contour, color) {
  cv.drawContours(mat, [contour], 0, color, 1, cv.LINE_AA);
}

const inspectObj = obj => console.inspect(obj, { multiline: false });
const inspectMat = ({ rows, cols, channels, depth, type }) =>
  inspectObj({
    rows,
    cols,
    channels,
    depth,
    type
  });

async function main(...args) {
  let start;
  let begin = hr();
  let running = true;
  let paused = false;

  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20,
    multiline: true
  });

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

  win.setMouseCallback(function (event, x, y, flags) {
    event = Mouse.printEvent(event);
    flags = Mouse.printFlags(flags);

    console.log('Mouse event:', { event, x, y, flags });
  });

  console.log('Setup duration:', hr(begin));

  let video = new VideoSource(...args);

  let font = new TextStyle(cv.FONT_HERSHEY_PLAIN, 1.0, 1);
  let tSize = font.size(video.time);

  let tPos = new Point(...tSize.div(2))
    .floor()
    .mul(-1)
    .add(50, video.get('frame_height') - tSize.y * 0.8);

  tPos.x = 5;

  let bgr = new Mat();
  console.log('backend:', video.backend);
  console.log('grab():', video.grab);
  //  console.log('read():', [...Util.repeat(10, () => video.grab())]);
  let frameCount = video.get('frame_count');
  let frameShow = -1;
  let contours, hier;
  let contoursDepth;
  function* getParents(idx) {
    while(idx != -1) {
      yield idx;

      idx = hier[idx][cv.HIER_PARENT];
    }
  }
  function getContourDepth(idx) {
    return [...getParents(idx)].length;
  }
  function findRoot(hier) {
    return hier.findIndex(h => h[cv.HIER_PREV] == -1 && h[cv.HIER_PARENT] == -1);
  }
  function* getToplevel(hier) {
    for(let [i, h] of hier.entries()) if(h[cv.HIER_PARENT] == -1) yield i;
  }
  function* walkContours(hier, idx) {
    idx = idx || findRoot(hier);
    let h;

    while((h = hier[idx])) {
      yield idx;

      if(h[cv.HIER_CHILD] != -1) yield* walkContours(hier, h[cv.HIER_CHILD]);

      idx = h[cv.HIER_NEXT];
    }
  }

  let params = {
    thresh1: new NumericParam(10, 0, 100),
    thresh2: new NumericParam(60, 0, 100),
    mode: new EnumParam(['RETR_EXTERNAL', 'RETR_LIST', 'RETR_CCOMP', 'RETR_TREE', 'RETR_FLOODFILL'],
      3
    ),
    method: new EnumParam([
        'CHAIN_APPROX_NONE',
        'CHAIN_APPROX_SIMPLE',
        'CHAIN_APPROX_TC89_L1',
        'CHAIN_APPROX_TC89_L189_KCOS'
      ],
      0
    )
  };
  console.log('Params:',
    [...Object.entries(params)].map(([name, param]) => [name, param.valueOf()])
  );

  let pipeline = new Pipeline([
      Processor(function acquireFrame(mat, output) {
        video.read(output);
      }),
      toGrayscale,
      Processor(cv.GaussianBlur, [3, 3], 0),
      Processor(function edgeDetect(src, dst) {
        cv.Canny(src, dst, 10, 60, 3);

        cv.findContours(dst, (contours = []), (hier = []), cv[params.mode], cv[params.method]);

        //console.log('hier:', hier .map((h, i) => [i, h]) .filter(([i, h]) => h[cv.HIER_PREV] == -1 && h[cv.HIER_PARENT] == -1));

        // console.log('walkContours:', [...walkContours(hier)]);
      })
    ],
    (mat, i, n) => {
      const showIndex = Util.mod(frameShow, n);
      if(showIndex == i) {
        let name = pipeline.processors[showIndex].name;
        //  mat = toBGR(mat);
        let surface = new Mat(mat.size, cv.CV_8UC4);
        let out = new Mat(/*mat.size, cv.CV_8UC4*/);

        /*    if(mat.channels == 1) cv.cvtColor(mat, mat, cv.COLOR_GRAY2BGR);
        if(mat.depth == cv.CV_32F) mat.convertTo(mat, cv.CV_8UC3);*/

        console.log('mat ' + inspectMat(mat));
        /*  if(out.type != mat.type)
          mat.convertTo(out, out.type, 255);
        else*/
        //        mat.copyTo(out);
        cv.cvtColor(mat, out, cv.COLOR_BGR2BGRA);

        console.log('out ' + inspectMat(out));

        //   mat.convertTo(out, cv.CV_8UC4);

        let ids = [...getToplevel(hier)];

        rainbow = makeRainbow(ids.length);
        let palette = {};
        // console.log('rainbow:', rainbow.map(c => c.hex()));

        for(let [i, id] of ids.entries()) {
          for(let child of walkContours(hier, id)) palette[child] = rainbow[i];
        }

        let depths = contours.map((contour, i) => {
          let p = [...getParents(i)];
          let d = p.length;
          let color = palette[i]; //rainbow[Util.mod(i, rainbow.length)];
          // console.log("drawContour", {d,color});
          drawContour(surface, contour, color);
          return d;
        });
        contoursDepth = depths.length ? Math.max(...depths) : 0;
        //   console.log('contoursDepth:', contoursDepth);

        font.draw(surface, video.time + ' ⏩', tPos, 0xffffff || { r: 0, g: 255, b: 0, a: 255 });
        font.draw(surface,
          `#${showIndex + 1}/${n}` + (name ? ` (${name})` : ''),
          [5, 5 + tSize.y],
          {
            r: 255,
            g: 255,
            b: 0,
            a: 255
          }
        );

        let size = mat.size.mul(zoom);

        win.resize(size.width, size.height);

        /*
        cv.cvtColor(surface, surface, cv.COLOR_BGRA2BGR);
        cv.cvtColor(out, out, cv.COLOR_BGRA2BGR);
*/
        let mask = toBGR(getAlpha(surface));
        console.log('inspectMat',
          ...Object.entries({ out, surface, mask }).map(([n, m]) =>
            [('\n' + n + ':').padEnd(10), inspectMat(m)].join('')
          )
        );
        /*
        cv.bitwise_or(out, surface, out);*/

        //surface.copyTo(out, mask);
        out = Mat.add(out, surface);
        // out = Mat.and(out, toBGR(mask));
        //        cv.bitwise_and(out,mask, out);

        win.show(out);
      }
    }
  );

  console.log('Pipeline processor names:', pipeline.names);
  video.seek_msecs(5000);

  while(running) {
    let frameNo = video.get('pos_frames');
    if(frameNo == frameCount) video.set('pos_frames', 0);

    let gray = pipeline(bgr);

    /* console.log('contours:', contours.length);
    console.log('hier:', hier.length);*/

    let key = cv.waitKeyEx(1000 / video.fps);

    if(key == 27) break;

    let modifiers = Object.fromEntries(modifierMap(key));
    let modifierList = modifierMap(key).reduce((acc, [modifier, active]) => (active ? [...acc, modifier] : acc),
      []
    );

    switch (key & 0xfff) {
      case 0x51:
      case 0x71:
        running = false;
        break;
      case 0x20:
        paused = !paused;
        break;
      case 0x6e:
      case 0x4e:
        frameShow++;
        break;
      case 0x70:
      case 0x50:
        frameShow--;
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
        const method = key & 0x1 ? 'frames' : 'msecs';
        const distance =
          (key & 0x1 ? 1 : 1000) *
          (modifiers['ctrl'] ? 1000 : modifiers['shift'] ? 100 : modifiers['alt'] ? 1 : 10);
        const offset = key & 0x2 ? +distance : -distance;

        //console.log('seek', { method, distance, offset });
        video['seek_' + method](offset);
        let pos = video.position(method);

        console.log('seek_' + method + ' ' + offset + ' pos =',
          pos,
          ` (${Util.roundTo(video.position('%'), 0.001)}%)`
        );
        break;
      }
      default: {
        if(key != -1) console.log(`keypress [${modifierList}] 0x${(key & ~0xd000).toString(16)}`);
        break;
      }
    }

    if(paused) video.seek_frames(-1);
  }

  console.log('props:', video.dump());
}

Util.callMain(main, true);
