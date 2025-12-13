import { className, isBrowser, repeat, tryCatch } from './lib/misc.js';
import inspect from './lib/objectInspect.js';
import * as path from './lib/path.js';
import Console from 'console';
import * as cv from 'opencv';
import * as std from 'std';
function main(...args) {
  //std.print("TEST PRINT\n");
  globalThis.console = new Console({
    inspectOptions: {
      maxStringLength: 200,
      compact: 2,
      depth: Infinity
    }
  });
  console.log('console', className(console));
  console.log('console.log', console.log);
  let entries = Object.fromEntries(Object.entries(cv).filter(([k, v]) => k.startsWith('CV_')));
  console.log(console.config({ depth: 1, compact: 1 }), entries);
  console.log(
    console.config({ compact: 0 }),
    Object.keys(entries).filter(k => /[0-9]S/.test(k))
  );
  console.log(
    console.config({ compact: 0 }),
    Object.keys(entries).filter(k => /[0-9]F/.test(k))
  );

  console.log('start');
  console.log('isBrowser:', isBrowser());
  //console.log('copyTextToClipboard()', await copyTextToClipboard('TEST'));
  // console.log('modules:', inspect({ Point, Size, Rect }));
  const moduleNames = ['Rect', 'Point', 'Size', 'Line', 'Mat', 'Contour', 'PointIterator', 'Draw'];
  for(let moduleName of moduleNames) tryCatch(() => eval(`globalThis[moduleName] = ${moduleName};`));

  let ctors = new Map(moduleNames.map(name => [name, globalThis[name]]));
  console.log('globalThis:', Object.keys(globalThis));
  console.log('modules:', inspect(ctors));

  let c = new Contour();
  console.log('c.keys()', Object.getOwnPropertyNames(c));
  console.log('c.push', c.push);

  c.push(new Point(0, 0));
  c.push({ x: 50, y: 0 });
  c.push({ x: 50, y: 50 });
  c.push({ x: 0, y: 50 });
  c.push({ x: 0, y: 0 });
  console.log('contour[0]', c[0]);
  console.log('contour[1]', c[1]);
  c[4] = new Point(99, 33);
  let fitted = new Line();
  console.log('fitted', fitted);
  c.fitLine(fitted);
  let ellipse = new RotatedRect();
  console.log('ellipse', ellipse);
  ellipse = c.fitEllipse();
  console.log('ellipse', ellipse);

  console.log('fitted', fitted);
  console.log('contour[4]', c[4]);
  console.log('contour:', c);
  console.log('c[Symbol.iterator]:', c[Symbol.iterator]);
  console.log('Object.getOwnPropertyNames(Contour.prototype)', Object.getOwnPropertyNames(Contour.prototype));

  console.log('Object.getOwnPropertyNames(c)', Object.getOwnPropertyNames(c));
  console.log('Object.keys(c)', Object.keys(c));

  let a = [1, 2, 3, 4, 5];
  console.log('Object.getOwnPropertyNames(a)', Object.getOwnPropertyNames(a));
  console.log('Object.keys(a)', Object.keys(a));

  let it = c[Symbol.iterator]();

  console.log('contour[Symbol.iterator]:', c[Symbol.iterator]);
  console.log('contour[Symbol.iterator]():', className(it));
  console.log('contour.get(0):', c[0]);
  console.log('[...contour]:', [...c]);
  console.log('contour.length:', c.length);
  console.log('contour:', className(c));
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
  return;

  let point = new Point(25, 75);
  console.log(`inspect(point)`, inspect(point));
  function toHex(n, b = 2) {
    let s = (+n).toString(16);
    return '0x' + '0'.repeat(Math.ceil(s.length / b) * b - s.length) + s;
  }
  let rr;
  rr = new Rect(5, 3, 4, 5);
  if(rr) {
    const { x, y, width, height } = rr;
    console.log('rect:', x, y, width, height);
  }
  let mat = new Mat(new Size(10, 10), CV_8UC4);
  console.log(`CV_8UC3`, toHex(CV_8UC3), CV_8UC3);
  console.log(`CV_8UC4`, toHex(CV_8UC4), CV_8UC4);
  console.log(`CV_8SC3`, toHex(CV_8SC3), CV_8SC3);
  console.log(`CV_8SC4`, toHex(CV_8SC4), CV_8SC4);
  console.log(`CV_32FC1`, toHex(CV_32FC1), CV_32FC1);
  console.log(`CV_32FC4`, toHex(CV_32FC4), CV_32FC4);
  console.log(`0x3ff`, toHex(0x3ff));
  console.log(`inspect(mat)`, inspect(mat));
  console.log(`mat.channels`, mat.channels);
  console.log(`mat.depth`, mat.depth);
  console.log(`1 << mat.depth`, 1 << mat.depth);
  console.log(
    `Mat[DEPTH]`,
    Object.keys(Mat).find(k => Mat[k] === mat.depth)
  );
  console.log(
    `Mat[TYPE]`,
    Object.keys(Mat).find(k => Mat[k] === mat.type)
  );
  mat.setTo([0xff, 0xff, 0xff, 0x80]);
  let row0 = mat.row(0);
  let col0 = mat.col(0);

  console.log(`mat.at(0,0)`, mat, mat.at(0, 0));
  console.log(`row0`, row0);
  console.log(`mat.row(0)`, row0, row0.cols, [...row0.entries()]);
  console.log(`col0`, col0);
  console.log(`mat.col(0)`, col0, col0.rows, [...col0.entries()]);

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

  it = row0[Symbol.iterator]();
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
    console.log(`row0.entries() #${i++}`, key, value); //'0x' + ('00000000' + value.toString(16)).slice(-8));
  }
  i = 0;
  for(let [key, value] of col0.entries()) {
    console.log(`col0.entries() #${i++}`, key, value);
    //console.log(`col0.entries() #${i++}`, key, '0x' + ('00000000' + value.toString(16)).slice(-8));
  }

  let range = mat.rowRange(2, 8);
  i = 0;
  for(let [[row, col], value] of range.entries()) {
    console.log(`range[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
  }
  i = 0;

  if(globalThis.Rect) {
    let roi = mat.roi(rr);

    for(let [[row, col], value] of roi.entries()) {
      console.log(`roi[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
    }
    console.log(`roi rows=${roi.rows} cols=${roi.cols} depth=${roi.depth} channels=${roi.channels}`);

    for(let r = 0; r < roi.rows; r++)
      for(let c = 0; c < roi.cols; c++) {
        const v = 0x7f000000 | ((r << 16) | c);
        console.log(`roi.set(${r},${c},0x${v.toString(16)})`);
        console.log(`roi.set(${r},${c},0x${v.toString(16)})`, roi.set(r, c, v));
      }

    roi.setTo(...repeat(4 * 5, 0xffffffff));
  }

  i = 0;
  for(let [[row, col], value] of mat.entries()) {
    console.log(`mat[${i++}] row=${row} col=${col} value=0x${('00000000' + value.toString(16)).slice(-8)}`);
  }

  let fmat = new Mat(new Size(10, 10), CV_32FC1);
  const values = repeat(fmat.rows * fmat.cols, 0.5);
  console.log(`fmat setTo`, values);
  fmat.setTo(...values);
  for(let [[row, col], value] of fmat.entries()) {
    console.log(`fmat[${i++}] row=${row} col=${col} value=${value}`);
  }

  let ll = [new Line(0, 0, 50, 50), new Line(50, 50, 50, 75), new Line(50, 75, 100, 75)];

  for(let line of ll) {
    console.log('line:', line.x1, line.y1, line.x2, line.y2);
    const { a, b } = line;

    console.log('a =', a);
    console.log('b =', b);
    console.log('line[0] =', line[0]);
    console.log('line[1] =', line[1]);
    console.log('line.toString() =', line.toString());

    let i = 0;
    let arr = line.toArray();
    console.log('toArray:', line.toArray().join(','));
    console.log('values(): ', line.values());
    console.log(
      'toPoints(): ',
      [...line.toPoints()].map(p => className(p))
    );

    console.log('toString(): ', line.toString());
    console.log('new Line(50,50,320-50,240-25): ', new Line(50, 50, 320 - 50, 240 - 25));
    let [x1, y1, x2, y2] = arr;

    console.log(`Line{${x1},${y1} ${x2},${y2}}`);
    for(let num of line) {
      console.log('num:', i++, num);
    }

    let r = new Rect(50, 100, 350, 200);
    console.log('r.br(): ', r.br());
    console.log('r.tl(): ', r.tl());
    console.log('r.area(): ', r.area());

    if(globalThis.Point) {
      let pt = new Point(75, 150);
      console.log(`r.contains(${pt}): `, r.contains(pt));
      pt = new Point(51, 99);
      console.log(`r.contains(${pt}): `, r.contains(pt));
    }

    r = new Rect(50, 50, 0, 0);
    console.log('r.empty(): ', r.empty());

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
  if(1) {
    console.log(`std.gc`, std.gc);
    console.log(`args`, args);
    console.log(`path`, inspect(path));
    console.log(`console`, console);
    console.log(`std.gc()`, std.gc());
  }

  return 'done';
}

main(...scriptArgs.slice(1));