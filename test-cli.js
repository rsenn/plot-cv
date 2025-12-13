import { Contour, Draw, drawCircle, drawLine, Line, Mat, Point, Rect, Size } from 'opencv';

async function main(...args) {
  //import { Contour } from "contour";
  const { circle, contour, line, polygon, rect } = Draw;

  console.log('global:', { drawLine, drawCircle });
  console.log('static:', { circle, contour, line, polygon, rect });
  console.log('test:', { Point, Size, Rect, Mat, Contour, Line, Draw });

  const ctors = [Point, Size, Rect, /*Mat,*/ Contour, Line];

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

  let ct1 = new Contour();
  let ct2 = new Contour();
  let pl1 = [new Point(0, 0), new Point(40, 0), new Point(40, 20), new Point(0, 20)];
  let pl2 = pl1.map(({ x, y }) => new Point(x + 100, y + 50));
  console.log('pl1:', pl1);
  console.log('pl2:', pl2);

  for(let point of pl1) ct1.push(point);
  for(let point of pl2) ct2.push(point);
  let it = ct1[Symbol.iterator]();

  let item;

  for(; (item = it.next()), !item.done; ) console.log('Item:', item);
  let i = 0;
  for(let p of ct1.concat(ct2)) console.log(`p[${i++}]:`, p);
}

main(...scriptArgs.slice(1));