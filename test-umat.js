import { Point } from 'point.so';
import { Size } from 'size.so';
import { Rect } from 'rect.so';
import { Mat } from 'mat.so';
import { UMat } from 'umat.so';
import * as cv from 'cv.so';
import { Line } from 'line.so';
import { Contour } from 'contour.so';
import { SliceIterator } from 'slice-iterator.so';
import * as draw from 'draw.so';
import RGBA from './lib/color/rgba.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { NumericParam, EnumParam, ParamNavigator } from './param.js';
import { Pipeline, Processor } from './cvPipeline.js';

let filesystem;

async function main(...args) {
  await ConsoleSetup({
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 0
  });
  let types = [
    ['CV_8U', cv.CV_8U],
    ['CV_8UC3', cv.CV_8UC3],
    ['CV_8S', cv.CV_8S],
    ['CV_16U', cv.CV_16U],
    ['CV_16S', cv.CV_16S],
    ['CV_32S', cv.CV_32S],
    ['CV_32F', cv.CV_32F],
    ['CV_64F', cv.CV_64F],
    ['CV_64FC2', cv.CV_64FC2]
  ];

  for(let [k, v] of types) {
    console.log(k,
      v,
      '0x' + v.toString(16),
      '0b' + v.toString(2),
      v >> 1,
      1 << ((v >> 1) + 3),
      1 << (v >> 1)
    );
  }

  let input = cv.imread(args[0] ??
      '../an-tronics/images/fm/Two-Transistor-Regenerative-Receiver-Schematic-Circuit-Diagram.jpg'
  );
  console.log('input.type', '0x' + input.type.toString(16));
  console.log('input.depth', '0x' + input.depth.toString(16));
  console.log('input.channels', '0x' + input.channels.toString(16));
  console.log('input.elemSize1', input.elemSize1);
  console.log('input.total', input.total);
  console.log('input.at', input.at(0, 0));
  let size = input.size;
  console.log('size', size);
  let { width, height } = size;
  let mat = new Mat(input.size, cv.CV_8UC3);
  let thresh = 100;
  const RandomPoint = () => new Point(Util.randInt(0, width - 1), Util.randInt(0, height - 1));
  const RandomColor = () => [Util.randInt(0, 255), Util.randInt(0, 255), Util.randInt(0, 255), 255];

  let gray = new Mat();
  cv.cvtColor(input, gray, cv.COLOR_BGR2GRAY);
  let blur = new Mat();
  cv.blur(gray, blur, new Size(3, 3));

  let canny = new Mat();
  cv.Canny(blur, canny, thresh, thresh * 2, 3);
  let canny2 = new Mat();
  canny.copyTo(canny2);

  let contours = [];
  let hierarchy = [];
  cv.findContours(canny,
    contours,
    hierarchy,
    cv.RETR_TREE,
    cv.CHAIN_APPROX_SIMPLE,
    new Point(0, 0)
  );
  let i = 0;
  for(let contour of contours) {
    //  console.log(`contours[${i}]`, [...contour].map(({x,y}) => ({x,y})));

    let b = contour.buffer;
    let m = contour.getMat();
    let it = new SliceIterator(b, Float64Array, 2);
    //console.log(`it[${i}]`, [...it].map(([x, y]) => ({ x, y })));
    // console.log(`contour.getMat()`, [...m]);
    //console.log(`cv::boundingRect`, contour.boundingRect());
    // console.log(`cv::aspectRatio`, contour.aspectRatio);
    draw.contours(mat, contours, i, RandomColor() ?? [(i * 255) / contours.length, 0, 0, 0], 2);

    function getDepth(idx, id = cv.HIER_PARENT) {
      let parent;
      let depth = 0;
      while(idx != -1) {
        const h = hierarchy[idx];
        depth++;
        idx = h[id];
      }
      return depth;
    }

    /*console.log(`hierarchy[${i}].depth`, getDepth(i));
    console.log(`hierarchy[${i}].index`, getDepth(i, cv.HIER_PREV));*/
    i++;
  }
  console.log(`contours.length`, contours.length);
   console.log('contours', contours.map(c => [...c].map(({x,y}) =>({x,y}))));
 //console.log(`hierarchy`, hierarchy);
  i = 0;
 
  let input2 = new Mat();

  input.copyTo(input2);
    input2.xor([255,255,255,0], input2);
  let input2u =
  input2.getUMat(cv.ACCESS_RW);
   console.log(`input2u`,input2u);
cv.imshow('input2',input2u);
   /* console.log(`input.buffer`, input.buffer);
  console.log(`input2.buffer`, input2.buffer);*/

  //for(let i = 0; i < 100; i++) draw.line(mat, RandomPoint(), RandomPoint(), RandomColor(), 1, cv.LINE_AA);

  let out = new Mat(size, cv.CV_8UC3);

  for(let m of [input, mat, out]) console.log('m', m);

  cv.addWeighted(input2, 0.5,mat ,1, 0, out);

  let umat = out.getUMat(cv.ACCESS_READ);

  cv.imshow('lines', umat);

  let key;

  while((key = cv.waitKey(0))) {
    if(key != -1) console.log('key:', key);

    if(key == 'q' || key == 113 || key == '\x1b') break;
  }
}

Util.callMain(main, true);
