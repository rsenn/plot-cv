import { Rect } from './build/x86_64-linux-gnu/rect.so';
import { Point } from './build/x86_64-linux-gnu/point.so';
import { Size } from './build/x86_64-linux-gnu/size.so';
import { Mat } from './build/x86_64-linux-gnu/mat.so';
import { PointIterator } from './build/x86_64-linux-gnu/point-iterator.so';

import { Contour } from './build/x86_64-linux-gnu/contour.so';
import { Line } from './build/x86_64-linux-gnu/line.so';
import { Draw, drawLine, drawCircle } from './build/x86_64-linux-gnu/draw.so';
//import { PointIterator } from "point-iterator";
import { inspect } from './lib/inspect.js';

//import { Contour } from "contour";
const { circle, contour, line, polygon, rect } = Draw;

//contournsole.log('global:', inspect({ drawLine, drawCircle }));
//console.log('static:', inspect({ circle, contour, line, polygon, rect }));
//console.log('test:', inspect({ Point, Size, Rect, Mat, Contour, Line, Draw }));

const ctors = [Point, Size, Rect, Mat, Contour, Line];

let objs = [];

for(let ctor of ctors) {
  objs.push(new ctor(0, 0, 0, 0));
}
for(let obj of objs) {
  //console.log('obj: ', obj.constructor.name, '' + obj);
}

let l = new Line(50, 50, 150, 150);

//console.log('line.a:', l.a);
//console.log('line.b:', l.b);
