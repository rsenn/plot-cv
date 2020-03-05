import { Point } from "./point.js";
import { Size } from "./size.js";
import { Line } from "./line.js";
import { Rect } from "./rect.js";
import { PointList } from "./pointList.js";
import { RGBA } from "./rgba.js";
import { HSLA } from "./hsla.js";
import { Matrix } from "./matrix.js";
import inspect from "./inspect.js";

const lib = { Point, Size, Line, Rect, PointList, RGBA, HSLA, Matrix };

function testPointVector() {
  let pv = new Contour();
  let poly = new Contour();

  pv.push(0,0);
  pv.push({x: 10, y:0});
  pv.push({x: 10, y:20});
  pv.push({x: 0, y:20});
  pv.push({x: 0, y:0});


  pv.approxPolyDP(poly, 2.0, true);

      console.log("poly.length: ", poly.length);
      console.log("poly.boundingRect(): ", poly.boundingRect());
      console.log("pv.length: ", pv.length);
      console.log("pv.get(0): ", pv.get(0));
      console.log("pv.get(1): ", pv.get(1));
      console.log("pv.area: ", pv.area);

/*
  let it = pv[Symbol.iterator]();
    console.log("it: ", it);

  let arr = [...it];
  console.log("arr: ", arr);
*/
}

global.test_array = [1, 2, 3, 4, 5, 6];
global.process = function(contours, hier) {
  var areas = [];

  /*  console.log("contours: ", global.contours[global.contours.length - 1]);
  console.log("hier: ", global.hier[global.contours.length - 1]);*/

  function dumpContour(c) {
    console.log(`contour #${c.id} length=${(c.length + "").padStart(5, " ")} bbox=`, c.bbox, " rect:", c.rect, " area=", c.area);
  }

  function processContours(contours) {
    contours.sort((a, b) => a.length - b.length);
    contours = contours.filter(c => c.length >= 4);
    for(var i = 0; i < contours.length; i++) {
      const [next, prev, child, parent] = hier[i];
      var list = new PointList(contours[i]);
      var bbox = list.bbox();
      var rect = new Rect(bbox);
      contours[i].area = rect.area;
      contours[i].id = i;
      contours[i].bbox = bbox;
      contours[i].rect = rect;
      areas.push(rect.area);
      list = list.map(p => {
        p.x += 2;
        p.y += 2;
        return p;
      });
      drawContour(list, new RGBA(255, 0, 255), 8, false);
    }
    contours.sort((a, b) => b.area - a.area);
    areas = contours.map(c => c.area);
    dumpContour(contours[0]);
    drawContour(contours[0], new RGBA(255, 0, 0), 20, false);
    return contours;
  }

  let polygons = [
    new PointList([
      { x: 0, y: 0 },
      { x: 320, y: 0 },
      { x: 320, y: 240 },
      { x: 0, y: 240 },
      { x: 0, y: 0 }
    ])
  ];

  polygons.push(polygons[0].sum({ x: 320, y: 0 }));
  polygons.push(polygons[0].sum({ x: 320, y: 240 }));
  polygons.push(polygons[0].sum({ x: 0, y: 240 }));
  /*
  drawPolygon(polygons[0], new RGBA(255, 255, 0), 3);
  drawPolygon(polygons[1], new RGBA(0, 255, 0), 3);
  drawPolygon(polygons[2], new RGBA(0, 0, 255), 3);
  drawPolygon(polygons[3], new RGBA(255, 0, 255), 3);
*/
  /*  drawCircle(new Point(300, 150), 110, new RGBA(1, 1, 1), -1);
  drawCircle(new Point(300, 150), 100, new RGBA(255, 255, 0), -1);
*/
  const do_log = false;

  if(do_log) {
    console.log(`polygons: [\n  ${polygons.join(",\n  ")}\n]`);

    console.log("PROCESS contours: ", contours.map(c => "[" + c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ") + "]").join(", "));
    console.log("PROCESS hier: ", "[" + hier.map(h => `[${h.join(",")}]`).join(", "));
  }
};
var ctor = Point_.prototype.constructor;
console.log("Classes: ", inspect(lib));
console.log("Point_: ", inspect(Point_));
console.log("typeof(Point_.prototype.constructor): ", typeof Point_.prototype.constructor == "function");
console.log("typeof(Point_): ", typeof Point_);
console.log("ctor.name: ", ctor.name);
/*console.log("Point_.prototype: ", Point_.prototype);
console.log("Point_.prototype.constructor: ", Point_.prototype.constructor);
*/
let points = [new Point_(0, 0), new Point_(50, 0), new Point_(100, 0), new Point_(100, 50), new Point_(100, 100), new Point_(100, 200)];
console.log("points[0]: ", points[0]);
console.log("points[last]: ", points[points.length - 1]);
console.log("points: ", points.map(p => `{x:${p.x},y:${p.y}}`).join(", "));

testPointVector();
