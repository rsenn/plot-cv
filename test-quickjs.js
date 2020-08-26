import { Rect } from './build/x86_64-linux-gnu/quickjs-rect.so';
import { Point } from './build/x86_64-linux-gnu/quickjs-point.so';
import { Size } from './build/x86_64-linux-gnu/quickjs-size.so';
import { Mat } from './build/x86_64-linux-gnu/quickjs-mat.so';
import { PointIterator } from './build/x86_64-linux-gnu/quickjs-point-iterator.so';
import { inspect } from './inspect.js';
import { Contour } from './build/x86_64-linux-gnu/quickjs-contour.so';
//console.log('test:', inspect({ Point, Size, Rect, Mat, PointIterator, Contour }));

let c = new Contour();

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

//console.log('contour:', inspect(c));

for (let point of c) {
  let { x, y } = point;
  //console.log('point:', point, x, y);
}
