import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
//import { client, server, fetch } from 'net';
import * as cv from 'cv';
import * as draw from 'draw';
import { Mat } from 'mat';
import { Size } from 'size';
import { Point } from 'point';
import { VideoSource } from './cvVideo.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './cvHighGUI.js';
import { Alea } from './lib/alea.js';
//import { drawCircle, drawContour, drawLine, drawPolygon, drawRect } from 'draw';

let prng = new Alea(Date.now());
const hr = Util.hrtime;

function dumpMat(name, mat) {
  console.log(`${name} =`,
    Object.create(
      Mat.prototype,
      ['cols', 'rows', 'depth', 'channels', 'type'].reduce((acc, prop) => ({ ...acc, [prop]: { value: mat[prop], enumerable: true } }), {})
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

  console.log('max:', max);
  console.log('const:', findType(type), type, cv[findType(type)]);
  console.log('channels:', mat.channels);
  console.log('constants:', getConstants(['CV_32F', 'CV_32FC1', 'CV_32FC3', 'CV_32FC4', 'CV_8U', 'CV_8UC1', 'CV_8UC3', 'CV_8UC4']));
  mat.convertTo(ret, type, 1.0 / max);
  return ret;
}

function toGrayscale(mat) {
  let lab = new Mat();
  let channels = [new Mat(), new Mat(), new Mat()];
  cv.cvtColor(mat, lab, cv.COLOR_BGR2Lab);
  cv.split(lab, channels);
  return channels[0];
}

function toBGR(mat) {
  let bgr = new Mat();
  cv.cvtColor(mat, bgr, cv.COLOR_GRAY2BGR);
  return bgr;
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

async function main(...args) {
  let start;
  let begin = hr();
  await ConsoleSetup({ breakLength: 120, maxStringLength: 200, maxArrayLength: 20 });
  console.log('cv:', cv);

  for(let name of ['CV_VERSION_MAJOR', 'NORM_MINMAX', 'COLOR_BGR2Lab', 'RETR_EXTERNAL', 'THRESH_BINARY', 'CAP_FFMPEG', 'CAP_V4L2', 'CAP_PROP_POS_FRAMES', 'CAP_PROP_BACKEND', 'CAP_PROP_CODEC_PIXEL_FORMAT', 'WINDOW_NORMAL', 'WINDOW_AUTOSIZE', 'WINDOW_FULLSCREEN']) console.log(`cv.${name}`, cv[name]);

  let win = new Window('gray', cv.WINDOW_AUTOSIZE);
  //console.log('Mouse :', { MouseEvents, MouseFlags });
  //console.log('cv.EVENT_MOUSEMOVE', cv.EVENT_MOUSEMOVE);

  win.setMouseCallback(function (event, x, y, flags) {
    /* event = Mouse.printEvent(event);*/
    flags = Mouse.printFlags(flags);

    console.log('Mouse event:', { event, x, y, flags });
  });

  console.log('Setup duration:', hr(begin));

  let video = new VideoSource(...args);

  let bgr = new Mat();
  console.log('backend:', video.backend);
  console.log('grab():', video.grab);
  console.log('read():', [...Util.repeat(10, () => video.grab())]);
  let frameCount = video.get('frame_count');

  for(;;) {
    let frameNo = video.get('pos_frames');
    if(frameNo == frameCount) video.set('pos_frames', 0);

    video.read(bgr);

    //dumpMat(`bgr #${frameNo}/${frameCount}`, bgr);

    let gray = toGrayscale(bgr);

    bgr = toBGR(gray);

    draw.circle(bgr, [50, 50], 25, 0xff00ff || { r: 255, g: 0, b: 0 }, 2, cv.LINE_AA);
    let baseY;

    let font = new TextStyle(cv.FONT_HERSHEY_PLAIN, 1.0, 1);
    let tSize = font.size('TEST');

    let tPos = new Point(...tSize.div(2))
      .floor()
      .mul(-1)
      .add(50, 50 + tSize.y * 1.6);

    console.log('tPos:', tPos);

    font.draw(bgr, 'TEST', tPos, 0xffff00 || { r: 255, g: 0, b: 0 });


    win.show(bgr);

    let key = cv.waitKeyEx(1000 / video.fps);

    if(key == 27) break;

    let modifiers = Object.fromEntries(modifierMap(key));
    let modifierList = modifierMap(key).reduce((acc, [modifier, active]) => (active ? [...acc, modifier] : acc), []);

    switch (key & 0xfff) {
      /* left */ case 0xf51:
      /* right */ case 0xf53:
      /* up */ case 0xf52:
      /* down */ case 0xf54: {
        const method = key & 0x1 ? 'frames' : 'msecs';
        const distance = (key & 0x1 ? 1 : 1000) * (modifiers['ctrl'] ? 1000 : modifiers['shift'] ? 100 : modifiers['alt'] ? 1 : 10);
        const offset = key & 0x2 ? +distance : -distance;

        //console.log('seek', { method, distance, offset });
        video['seek_' + method](offset);
        let pos = video.position(method);

        console.log('seek_' + method + ' ' + offset + ' pos =', pos, ` (${Util.roundTo(video.position('%'), 0.001)}%)`);
        break;
      }
      default: {
        if(key != -1) console.log(`keypress [${modifierList}] 0x${(key & ~0xd000).toString(16)}`);
        break;
      }
    }
  }

  console.log('props:', video.dump());
}

Util.callMain(main, true);
