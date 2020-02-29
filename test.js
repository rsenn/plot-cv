//import { Point } from "./point.js";
//import { PointList } from "./pointList.js";
//import { Size } from "./size.js";
//import { Rect } from "./rect.js";
//import { Line } from "./line.js";
//const { Point } = require('./point.js');

function process(contours, hier) {
  console.log("PROCESS contours: ", contours.map(c => "[" + c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ") + "]").join(", "));
  console.log("PROCESS hier: ", "[" + hier.map(h => `[${h.join(",")}]`).join(", "));
}

function inspect(obj) {
  let out = "";
  for (let key in obj) {
    let value = obj[key];
    if (out.length > 0) out += ", ";
    out += key + ": ";
    out += typeof(value) == 'object' ? inspect(value) : value;
  }
  return '{ '+out+' }';
}

function test() {
  var pt = new Point(100, 50);
  var pt2 = new Point(150, 230);
  var pt3 = Point.diff(pt2, pt);
  var contour = [pt, pt, pt, pt, pt, pt, pt];
  var l = new Line(pt, pt2);

  if (this !== undefined && this.drawContour) this.drawContour(contour, [255, 0, 0]);
  console.log("contour: ", contour);
  console.log("args: ", scriptArgs);
  console.log("pt: ", pt.toString(true));
  console.log("diff: ", pt3);
  console.log("angle: ", pt3.toAngle(true));
  console.log("distance: ", pt3.distance());

  console.log("line length: ", l.length());
  console.log("line a: ", l.a);
  console.log("line b: ", l.b);
  let { x1, y1, x2, y2 } = l;
  console.log("line: ", inspect({ x1, y1, x2, y2 }));
  console.log("line slope: ", l.slope());
  console.log("line direction: ", l.direction());

  //console.log("drawContour: ", drawContour);
}

//test();
