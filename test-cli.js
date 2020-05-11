import { Rect } from "rect";
import { Point } from "point";
import { Size } from "size";
import { Mat } from "mat";
import { Contour } from "contour";
import { Line } from "line";
import { Draw, drawLine, drawCircle } from "draw";
//import { PointIterator } from "point-iterator";
import { inspect } from "./inspect.js";

//import { Contour } from "contour";
const { circle, contour, line, polygon, rect } = Draw;

console.log("global:", inspect({ drawLine, drawCircle }));
console.log("static:", inspect({ circle, contour, line, polygon, rect }));
console.log("test:", inspect({ Point, Size, Rect, Mat, Contour, Line, Draw }));

const ctors = [Point, Size, Rect, Mat, Contour, Line];

let objs = [];

for(let ctor of ctors) {
  objs.push(new ctor(0, 0, 0, 0));
}
for(let obj of objs) {
  console.log("obj: ", obj.constructor.name, "" + obj);
}

let l = new Line(50, 50, 150, 150);

console.log("line.a:", l.a);
console.log("line.b:", l.b);
