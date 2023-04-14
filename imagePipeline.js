import { Point, Size, Contour, Rect, Line, TickMeter, Mat, CLAHE, Draw } from 'opencv';
import * as cv from 'opencv';
import { VideoSource } from './qjs-opencv/js/cvVideo.js';
import { Window, MouseFlags, MouseEvents, Mouse, TextStyle } from './qjs-opencv/js/cvHighGUI.js';
import { HSLA } from './lib/color.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import fs from 'fs';
import { format } from './lib/misc.js';
import * as xml from 'xml';
import Console from 'console';
import { Pipeline, Processor } from './qjs-opencv/js/cvPipeline.js';
import SvgPath from './lib/svg/path.js';
import { WeakMapper, Modulo, WeakAssign, BindMethods, BitsToNames, FindKey, Define, Once, GetOpt, RoundTo, Range } from './qjs-opencv/js/cvUtils.js';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, MapFile, ReadBJSON, WriteFile, WriteJSON, WriteBJSON } from './io-helpers.js';

export function ImagePipeline(/*input,*/ config) {
  let contours = [],
    hier = [];
  let contoursDepth;
  let lines = [],
    circles = [];
  let outputMat = new Mat(),
    outputName;

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
    mode: new EnumParam(config.mode || 3, ['RETR_EXTERNAL', 'RETR_LIST', 'RETR_CCOMP', 'RETR_TREE', 'RETR_FLOODFILL']),
    method: new EnumParam(config.method || 0, ['CHAIN_APPROX_NONE', 'CHAIN_APPROX_SIMPLE', 'CHAIN_APPROX_TC89_L1', 'CHAIN_APPROX_TC89_L189_KCOS']),
    maskColor: new EnumParam(config.maskColor || false, ['OFF', 'ON']),
    lineWidth: new NumericParam(config.lineWidth || 1, 0, 10),
    fontThickness: new NumericParam(config.fontThickness || 1, 0, 10)
  };
  /*  let paramNav = new ParamNavigator(params, config.currentParam);
  let dummyArray = [0, 1, 2, 3, 4, 5, 6, 7];
 
  if(opts['trackbars']) {
    params.apertureSize.createTrackbar('apertureSize', win);
    params.thresh1.createTrackbar('thresh1', win);
    params.thresh2.createTrackbar('thresh2', win);
    console.log('win.imageRect (2)', win.imageRect);
  }
 
  rainbow = makeRainbow(256);
*/
  let structuringElement = cv.getStructuringElement(cv.MORPH_CROSS, new Size(3, 3));

  let clahe = new CLAHE(4, new Size(8, 8));
  let framePos;

  let pipeline;
  pipeline = new Pipeline(
    [
      /*     Processor(function AcquireFrame(src, dst) {
        let dst0Size, firstSize, videoSize;
        const dstEmpty = dst.empty;
        if(dst.empty) dst0Size = dst.size;

       
        framePos = input.get('pos_frames');

        console.log('dst(1)', dst);

        input.read(dst);
        console.log('dst(2)', dst);
       
        if(videoSize === undefined || videoSize.empty) videoSize = input.size.area ? input.size : dst.size;
        if(dstEmpty) firstSize = new Size(...videoSize);
        if(dst.size && !videoSize.equals(dst.size))
          throw new Error(`AcquireFrame videoSize = ${videoSize} firstSize=${firstSize} dst.size = ${dst.size}`);
      }),*/
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
        let mat = new Mat(0, 0, cv.CV_32SC4);

        cv.HoughLinesP(edges, mat, 2, (+params.angleResolution * Math.PI) / 180, +params.threshc, +params.minLineLength, +params.maxLineGap);
        pipeline.lines = [...mat]; //.array;
        // console.log('mat', mat);
        //  console.log('lines', lines.slice(0, 10));
        // console.log('lines.length', lines.length);
        src.copyTo(dst);
      })
    ],
    function(i, n) {
      const { show = 0 } = this;
      if(show === i) {
        let mat = this.getImage(i);

        this.outputName = this.processors[show].name;
        this.outputMat = mat;
      }
    }
  );
  Object.defineProperty(pipeline, 'contours', {
    get() {
      return contours;
    }
  });

  return pipeline;
}
