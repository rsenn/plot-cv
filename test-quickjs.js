import { Rect } from './build/x86_64-linux-gnu/rect.so';
import { Point } from './build/x86_64-linux-gnu/point.so';
import { Size } from './build/x86_64-linux-gnu/size.so';
import { Line } from './build/x86_64-linux-gnu/line.so';
import { Mat } from './build/x86_64-linux-gnu/mat.so';
import { Contour } from './build/x86_64-linux-gnu/contour.so';
import { PointIterator } from './build/x86_64-linux-gnu/point-iterator.so';
import { Draw, drawLine, drawCircle } from './build/x86_64-linux-gnu/draw.so';
import { inspect } from './lib/inspect.js';

//console.log('test:', inspect({ Point, Size, Rect, Mat, PointIterator, Contour }));
import path from './lib/path.js';
import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
///import ConsoleSetup from './consoleSetup.js';

let filesystem;

async function main(...args) {
  //std.print("TEST PRINT\n");
  //  await ConsoleSetup({ breakLength: 120, maxStringLength: 200, maxArrayLength: 20 });

  await PortableFileSystem(fs => (filesystem = fs));
  console.log('start');
  console.log('modules:', inspect({ Point, Size, Rect, Mat, PointIterator, Contour }));

  /*  let rect = new Rect(10, 100, 50, 250);
  const { x, y, width, height } = rect;
  console.log(`rect`, inspect(rect));
  console.log(`{x,y,width,height}`, inspect({ x, y, width, height }));
  console.log(`Object.getPrototypeOf(rect)`, Object.getPrototypeOf(rect));
  console.log(`rect.prototype`, rect.prototype);
  console.log(`rect.__proto__`, rect.__proto__);
  console.log(`Object.getOwnPropertyNames(rect.__proto__)`, Object.getOwnPropertyNames(rect.__proto__));
  console.log(`Object.keys(rect.__proto__)`, Object.keys(rect.__proto__));
  console.log(`Object.keys(Object.getPrototypeOf(rect))`, Object.keys(Object.getPrototypeOf(rect)));
  console.log(`Object.keys(rect)`, Object.keys(rect));
  console.log(`inspect(rect)`, inspect(rect));*/
  let point = new Point(25, 75);
  console.log(`inspect(point)`, inspect(point));

  function toHex(n, b = 2) {
    let s = (+n).toString(16);
    return '0x' + '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
  }

  let rr = new Rect(5, 3, 4, 5);
  if(rr) {
    const { x, y, width, height } = rr;
    console.log('rect:', x, y, width, height);
  }

  let mat = new Mat(new Size(10, 10), Mat.CV_8UC4);
  console.log(`Mat.CV_8UC3`, toHex(Mat.CV_8UC3), Mat.CV_8UC3);
  console.log(`Mat.CV_8UC4`, toHex(Mat.CV_8UC4), Mat.CV_8UC4);
  console.log(`Mat.CV_8SC3`, toHex(Mat.CV_8SC3), Mat.CV_8SC3);
  console.log(`Mat.CV_8SC4`, toHex(Mat.CV_8SC4), Mat.CV_8SC4);
  console.log(`Mat.CV_32FC1`, toHex(Mat.CV_32FC1), Mat.CV_32FC1);
  console.log(`Mat.CV_32FC4`, toHex(Mat.CV_32FC4), Mat.CV_32FC4);
  console.log(`0x3ff`, toHex(0x3ff));
  console.log(`inspect(mat)`, inspect(mat));
  console.log(`mat.channels`, mat.channels);
  console.log(`mat.depth`, mat.depth);
  console.log(`1 << mat.depth`, 1 << mat.depth);
  console.log(`Mat[DEPTH]`,
    Object.keys(Mat).find(k => Mat[k] === mat.depth)
  );
  console.log(`Mat[TYPE]`,
    Object.keys(Mat).find(k => Mat[k] === mat.type)
  );
  let row0 = mat.row(0);
  let col0 = mat.col(0);

  console.log(`mat.row(0)`, row0);

  for(let r = 0; r < mat.rows; r++)
    for(let c = 0; c < mat.cols; c++) {
      const v = (r << 24) | c;
      console.log(`mat.set(${r},${c},0x${v.toString(16)})`, mat.set(r, c, v));
    }
  console.log(`mat.set(0,1,0xcafebabe)`, mat.set(0, 1, 0xcafebabe));
  console.log(`mat.set(0,2,0xc01dd00d)`, mat.set(0, 2, 0xc01dd00d));
  console.log(`row0.at(0,0)`, row0.at(0, 0));
  console.log(`mat.at(0,0)`, mat.at(0, 0));
  console.log(`mat.at(new Point(0,0))`, mat.at(new Point(0, 0)));

  let it = row0[Symbol.iterator]();
  console.log(`row0[Symbol.iterator]()`, it);

  let step = it.next();
  console.log(`it.next()`, step.done, step.value);
  let i = 0;
  for(let x of row0.values()) {
    console.log(`row0.values()[${i++}]`, x);
  }
  i = 0;
  it = row0.keys();
  console.log(`row0.keys()`, it);
  console.log(`row0.keys().next`, it.next);
  console.log(`row0.keys()[Symbol.iterator]`, it[Symbol.iterator]);
  let v;
  while(true) {
    v = it.next();
    if(v.done) break;
    console.log(`row0.keys() #${i++}`, v.value, v.value.length);
  }
  i = 0;
  for(let [key, value] of row0.entries()) {
    console.log(`row0.entries() #${i++}`, key, '0x' + ('00000000' + value.toString(16)).slice(-8));
  }
  i = 0;
  for(let [key, value] of col0.entries()) {
    console.log(`col0.entries() #${i++}`, key, '0x' + ('00000000' + value.toString(16)).slice(-8));
  }

  let range = mat.rowRange(2, 8);
  i = 0;
  for(let [[row, col], value] of range) {
    console.log(`range[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
  }
  i = 0;

  let roi = mat.roi(rr);

  for(let [[row, col], value] of roi) {
    console.log(`roi[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
  }

  for(let r = 0; r < roi.rows; r++)
    for(let c = 0; c < roi.cols; c++) {
      const v = 0x7f000000 | ((r << 16) | c);
      console.log(`roi.set(${r},${c},0x${v.toString(16)})`, roi.set(r, c, v));
    }

  roi.setTo(...Util.repeat(4 * 5, 0xffffffff));

  i = 0;
  for(let [[row, col], value] of mat) {
    console.log(`mat[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
  }

  let fmat = new Mat(new Size(10, 10), Mat.CV_32FC1);
  const values = Util.repeat(fmat.rows * fmat.cols, 0.5);
  console.log(`fmat setTo`, values);
  fmat.setTo(...values);
  for(let [[row, col], value] of fmat) {
    console.log(`fmat[${i++}] row=${row} col=${col} value=${value}`);
  }

  let ll = [new Line(0, 0, 50, 50), new Line(50, 50, 50, 75), new Line(50, 75, 100, 75)];

  for(let line of ll) {
    console.log('line:', line.x1, line.y1, line.x2, line.y2);
    const { a, b } = line;

    console.log('a =', a);
    console.log('b =', b);

    let i = 0;

    console.log('toArray:', line.toArray().join(','));
    console.log('values(): ' + [...line.values()].map(n => '' + n).join(', '));

    let it = line[Symbol.iterator]();
    console.log('[Symbol.iterator]():', it);

    console.log('it:', it);
    for(let a of it) {
      console.log('a:', a);
    }

    //const [start, end] = line;

    //console.log("start,end:",start,end);
  }
  /* let c = new Contour();

  c.push(new Point(0, 0));
  c.push(new Point(10, 0));
  c.push(new Point(20, 0));
  c.push(new Point(30, 0));
  c.push(new Point(30, 10));
  c.push(new Point(30, 20));
  c.push(new Point(30, 30));
  c.push(new Point(20, 30));
  c.push(new Point(10, 30));
  c.push(new Point(0, 30));
  c.push(new Point(0, 20));
  c.push(new Point(0, 10));
  c.push(new Point(0, 0));

  for(let point of c) {
    let { x, y } = point;
    //console.log('point:', point, x, y);
  }

    //console.log('contour:', inspect(c));
*/
  //throw new Error("ERROR");
  if(0) {
    console.log(`std.gc`, std.gc);
    console.log(`args`, args);
    console.log(`path`, inspect(path));
    console.log(`console`, Util.inspect(console));
    console.log(`filesystem.realpath('.')`, filesystem.realpath('.'));
    console.log(`filesystem.chdir('..')`, filesystem.chdir('..'));
    console.log(`filesystem.getcwd('.')`, filesystem.getcwd());
    console.log(`std.gc()`, std.gc());
  }

  return 'done';
}
Util.callMain(main, true);
/*
console.log('TEST\n');
print('TEST\n');
let retVal;
//retVal =  main().catch(err => console.log("Error:", err, err.stack)).then(ret => (console.log("Resolved:", ret),ret));
retVal = main();
console.log('retVal:', retVal);
retVal;
1;
//Util.callMain(main, true);
*/
