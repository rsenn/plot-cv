import { LoadConfig } from './config.js';
import { NumericParam, ParamNavigator } from './param.js';
import { Window } from './qjs-opencv/js/cvHighGUI.js';
import { Pipeline } from './qjs-opencv/js/cvPipeline.js';
import Console from 'console';
import * as cv from 'opencv';
function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: 3,
    maxArrayLength: 30,
    compact: 3
  });
  let config = LoadConfig();

  let params = {
    thresh: new NumericParam(config.thresh || 40, 0, 255)
  };
  let paramNav = new ParamNavigator(params, config.currentParam);

  let win = new Window('test', cv.WINDOW_NORMAL | cv.WINDOW_AUTOSIZE | cv.WINDOW_KEEPRATIO);
  params.thresh.createTrackbar('thresh', win);
  let img;
  let frameShow = 2,
    outputName,
    outputMat;
  let roiRect, roiMat;

  let pipeline = new Pipeline(
    [
      function AcquireFrame(src, dst) {
        img.copyTo(dst);
      },
      function Grayscale(src, dst) {
        cv.cvtColor(src, dst, cv.COLOR_BGR2GRAY);

        /*  let hist = new cv.Mat();
    cv.calcHist([dst], 1, 0, new cv.Mat(), hist, 1,  [255],[0,256]);
    console.log('hist', hist);*/
      },
      function Thresh(src, dst) {
        cv.threshold(src, dst, +params.thresh, 255, cv.THRESH_BINARY_INV);

        //dst.div(2);
        dst.xor(0x40);
      }
    ],
    (i, n) => {
      if(frameShow == i) {
        let mat = pipeline.getImage(i);
        //console.log('pipeline show', { i, n });

        outputName = pipeline.processors[frameShow].name;
        outputMat = mat;

        win.show(mat);

        /*if((roiRect=win.selectROI())) {
console.log('roi', roiRect);
roiMat = mat(roiRect);
img =roiMat;
pipeline.recalc();
}*/
      }
    }
  );

  params.thresh.on('change', (newVal, oldVal) => {
    //console.log(`onchange callback: ${oldVal} -> ${newVal}`);
    pipeline.recalc(frameShow);
  });

  for(let arg of args) {
    let key, mat;
    img = cv.imread(arg);
    console.log('img', img);

    mat = pipeline(img);

    do {
      key = cv.waitKey(-1);
      if([13, 10].indexOf(key) != -1) break;
      if(key == 'q'.charCodeAt(0)) std.exit(0);
    } while(key != 13);
  }
}

main(...scriptArgs.slice(1));