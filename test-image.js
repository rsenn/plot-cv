import { SaveSVG } from './image-helpers.js';
import { ImagePipeline } from './imagePipeline.js';
import { WriteJSON } from './io-helpers.js';
import { Rect, Size } from './lib/geom.js';
import { HSLA } from './lib/color.js';
import SvgPath from './lib/svg/path.js';
import { Mouse, MouseFlags, Screen, Window } from './qjs-opencv/js/cvHighGUI.js';
import { BitsToNames, GetOpt, Range } from './qjs-opencv/js/cvUtils.js';
import { VideoSource } from './qjs-opencv/js/cvVideo.js';
import Console from 'console';
import * as cv from 'opencv';
import * as std from 'std';
let simplifyMethods = {
  NTH_POINT: c => c.simplifyNthPoint(2),
  RADIAL_DISTANCE: c => c.simplifyRadialDistance(10),
  PERPENDICULAR_DISTANCE: c => c.simplifyPerpendicularDistance(20),
  REUMANN_WITKAM: c => c.simplifyReumannWitkam(),
  OPHEIM: c => c.simplifyOpheim(),
  LANG: c => c.simplifyLang(),
  DOUGLAS_PEUCKER: c => c.simplifyDouglasPeucker()
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
/*
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
}*/

function main(...args) {
  let start;
  let running = true;
  let paused = false;

  globalThis.console = new Console({
    colors: true,
    depth: 3,
    maxArrayLength: 30,
    compact: 3
  });
  const { DISPLAY } = globalThis.process ? globalThis.process.env : std.getenviron();
  console.log('DISPLAY', DISPLAY);
  console.log(
    'cv.ALIGN_RIGHT',
    Object.getOwnPropertyNames(cv).filter(n => /ALIGN/.test(n))
  );
  let r1 = new Rect(0, 0, 1200, 600),
    r2 = new Size(400, 200);

  let r3 = r2.align(r1, cv.ALIGN_RIGHT | cv.ALIGN_BOTTOM);
  console.log('r3', r3);
  console.log('Screen.size()', Screen.size());

  let opts = GetOpt(
    {
      help: [
        false,
        () => {
          console.log(`Usage: ${getArgv()[0]} [OPTIONS] <video|device>`);
          exit(0);
        },
        'h'
      ],
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

  console.log('cv.getScreenResolution():', cv.getScreenResolution());
  console.log('opts:', opts);
  console.log('opts.size:', opts.size);

  const makeRainbow = steps =>
    Range(0, 360, 360 / steps)
      .slice(0, -1)
      .map(hue => new HSLA(hue, 100, 50))
      .map(h => h.toRGBA());

  let win = new Window('gray', cv.WINDOW_NORMAL /*| cv.WINDOW_AUTOSIZE */ | cv.WINDOW_KEEPRATIO);
  //console.debug('Mouse :', { MouseEvents, MouseFlags });

  const printFlags = flags => [...BitsToNames(MouseFlags)];
  /*console.log('printFlags:', printFlags + '');
  console.log('tickFrequency:', cv.getTickFrequency());*/

  win.setMouseCallback(function (event, x, y, flags) {
    event = Mouse.printEvent(event);
    flags = Mouse.printFlags(flags);

    console.debug('Mouse event:', console.inspect({ event, x, y, flags }, { multiline: false }));
  });

  let contours, hier;
  let contoursDepth;
  let lines, circles;
  let outputMat, outputName;

  const images = opts['input'] ? [opts['input']] : opts['@'];

  let pipeline = new ImagePipeline({});

  for(let image of images) {
    console.log('image', image);
    let mat = cv.imread(image);

    let out = pipeline(mat);

    console.log('mat', mat);
    //let [w, h] = [...mat.size];
    //console.log('mat.size', { w, h });

    //win.move(0, 0);
    win.resize(...mat.size);
    win.align(0);

    win.show(out);
    let k;
    while((k = cv.waitKey(-1))) {
      if(['\n', '\r', 13, 10].indexOf(k) != -1) break;
      if(['s', 'S', 115, 83].indexOf(k) != -1) {
        const { contours } = pipeline;
        console.log('contours.length', contours.length);
        saveContours(contours, out.size);
        // saveLines(lines, out.size);
        continue;
      }
    }
  }
  std.exit(0);

  function saveContours(contours, size) {
    let points = contours.reduce((acc, contour, i) => {
      console.log('contour #' + i, contour);
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
      attributes: { d }
    }));
    let viewBox = [0, 0, ...size].join(' ');
    let doc = {
      tagName: 'svg',
      children: [{ tagName: 'g', attributes: { stroke: 'black', fill: 'none' }, children }],
      attributes: {
        xmlns: 'http://www.w3.org/2000/svg',
        viewBox
      }
    };
    WriteJSON('contours.json', doc);
    SaveSVG('contours.svg', doc);
  }

  function saveLines(lines, size) {
    let viewBox = [0, 0, ...size].join(' ');
    let children = lines
      .map(coords => new Line(...coords))
      .map(([x1, y1, x2, y2]) => ({
        tagName: 'line',
        attributes: { x1, y1, x2, y2 }
      }));
    let doc = {
      tagName: 'svg',
      children: [{ tagName: 'g', attributes: { stroke: 'black', fill: 'none' }, children }],
      attributes: {
        xmlns: 'http://www.w3.org/2000/svg',
        viewBox
      }
    };

    WriteJSON('lines-' + framePos + '.json', doc);
    SaveSVG('lines-' + framePos + '.svg', doc);
  }
  /*   
  let paramNav = new ParamNavigator(params, config.currentParam);
  let dummyArray = [0, 1, 2, 3, 4, 5, 6, 7];
  console.log('win.imageRect (1)', win.imageRect);

  if(opts['trackbars']) {
    params.apertureSize.createTrackbar('apertureSize', win);
    params.thresh1.createTrackbar('thresh1', win);
    params.thresh2.createTrackbar('thresh2', win);
    console.log('win.imageRect (2)', win.imageRect);
  }
 
  rainbow = makeRainbow(256);

  let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));

  let dst0Size, firstSize, videoSize;
  let clahe = new CLAHE(4, new Size(8, 8));
  let framePos;

  let pipeline = new Pipeline(
    [
      Processor(function AcquireFrame(src, dst) {
        const dstEmpty = dst.empty;
        if(dst.empty) dst0Size = dst.size;
        console.log('video', video);
        framePos = video.get('pos_frames');
        video.read(dst);
        console.log('dst', dst);
        win.show(dst);
        if(videoSize === undefined || videoSize.empty) videoSize = video.size.area ? video.size : dst.size;
        if(dstEmpty) firstSize = new Size(...videoSize);
        if(dst.size && !videoSize.equals(dst.size))
          throw new Error(`AcquireFrame videoSize = ${videoSize} firstSize=${firstSize} dst.size = ${dst.size}`);
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

        cv.HoughLinesP(
          edges,
          mat,
          2,
          (+params.angleResolution * Math.PI) / 180,
          +params.threshc,
          +params.minLineLength,
          +params.maxLineGap
        );
        lines = [...mat]; //.array;
        // console.log('mat', mat);
        //  console.log('lines', lines.slice(0, 10));
        // console.log('lines.length', lines.length);
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
  );*/

  //std.gc();
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log('FAIL: ', error && error.message, error && error.stack ? '\n' + error.stack : '');
  std.exit(1);
}

console.log('SUCCESS');