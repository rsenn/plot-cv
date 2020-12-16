import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
//import { client, server, fetch } from 'net';
import * as cv from 'cv';
import { Mat } from 'mat';
import { VideoSource } from './videoSource.js';
import { Alea } from './lib/alea.js';

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

function minMax(mat) {
  const ret = cv.minMaxLoc(mat);
  return [ret.minVal, ret.maxVal];
}

async function main(...args) {
  let start;
  let begin = hr();
  await ConsoleSetup({ breakLength: 120, maxStringLength: 200, maxArrayLength: 20 });

  console.log('Setup duration:', hr(begin));

  let video = new VideoSource(...args);

  let bgr = new Mat();
  console.log('backend:', video.backend);
  console.log('grab():', video.grab);
  console.log('read():', [...Util.repeat(10, () => video.grab())]);

  video.retrieve(bgr);

  dumpMat('bgr', bgr);

  let gray = /*to32bit*/ toGrayscale(bgr);
  dumpMat('gray', gray);
  console.log('gray row(0):', [...gray.col(gray.cols - 1).values()]);
  console.log('gray minMax:', cv.minMaxLoc(gray));

  cv.imshow('gray', gray);

  console.log('props:', video.dump());
}

Util.callMain(main, true);
