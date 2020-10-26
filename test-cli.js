import Util from './lib/util.js';
import { Rect } from 'rect';
import { Point } from 'point';
import { Size } from 'size';
import { Mat } from 'mat';
import { PointIterator } from 'point-iterator';
import { Contour } from 'contour';
import { Line } from 'line';
import { Draw, drawLine, drawCircle } from 'draw';
import ConsoleSetup from './lib/consoleSetup.js';

import inspect from './lib/objectInspect.js';

let filesystem;

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  //import { Contour } from "contour";
  const { circle, contour, line, polygon, rect } = Draw;

  console.log('global:', { drawLine, drawCircle });
  console.log('static:', { circle, contour, line, polygon, rect });
  console.log('test:', { Point, Size, Rect, Mat, Contour, Line, Draw });

  const ctors = [Point, Size, Rect, Mat, Contour, Line];

  let objs = [];

  for(let ctor of ctors) {
    objs.push(new ctor(0, 0, 0, 0));
  }
  for(let obj of objs) {
    console.log('keys: ', Util.getMemberNames(obj));
    console.log('obj: ', obj.constructor.name, obj);
  }

  let l = new Line(50, 50, 150, 150);

  console.log('line.a:', l.a);
  console.log('line.b:', l.b);

  let c = new Contour([new Point(0,0),new Point(40,0),new Point(40,20),new Point(0,20),])
  for(let point of [new Point(0,0),new Point(40,0),new Point(40,20),new Point(0,20),])
    c.push(point);


  let it = c[Symbol.iterator]();

  let item;

  for(; (item = it.next(), !item.done); )  
  console.log("Item:", item);

  for(let p of c)
  console.log('p:', p);
}
Util.callMain(main, true);
