import { Point as Point_ } from "./point.js";
import { Size as Size_ } from "./size.js";
import { Line } from "./line.js";
import { Rect as Rect_ } from "./rect.js";
import { PointList } from "./pointList.js";
import { RGBA } from "./rgba.js";
import { HSLA } from "./hsla.js";
import { Matrix } from "./matrix.js";
import inspect from "./inspect.js";

const lib = { Point, Size, Line, Rect, PointList, RGBA, HSLA, Matrix };

Point.prototype.atan2 = function() {
  return Math.atan2(this.x, this.y);
};
Object.defineProperty(Point.prototype, "distance", {
  get: function() {
    return Math.sqrt(this.x * this.x + this.y * this.y);
  },
  enumerable: true
});

function testPointVector() {
  let pv = new Contour();
  let poly = new Contour();
  let hull = new Contour();
  let s = new Size(320, 200);
  let mat = new Mat(s, Mat.CV_32FC2);
  let mat2 = new Mat(200, 320, Mat.CV_32FC2);

  console.log("s =", s);
  console.log("mat.rows =", mat.rows);
  console.log("mat.cols =", mat.cols);
  console.log("mat.type =", mat.type);
  console.log("mat.channels =", mat.channels);
  console.log("mat2.rows =", mat2.rows);
  console.log("mat2.cols =", mat2.cols);
  pv.push(0, 0);
  pv.push({ x: 10, y: 0 });
  pv.push({ x: 10, y: 20 });
  pv.push({ x: 0, y: 20 });
  pv.push({ x: 0, y: 0 });

  pv.approxPolyDP(poly, 2.0, true);
  poly.convexHull(hull, true, true);

  let circle = pv.minEnclosingCircle();
  let triangle = pv.minEnclosingTriangle();
  console.log("circle.center: ", circle.center);
  console.log("circle.radius: ", circle.radius);
  console.log("triangle: ", triangle);
  console.log("Mat.CV_8UC4 ", Mat.CV_8UC4);
  console.log("Mat.CV_32FC1 ", Mat.CV_32FC1);
  let a = pv.get(1);
  let b = pv.get(2);

  console.log("a.cross(b): ", a.cross(b));
  console.log("a.dot(b): ", a.dot(b));
  console.log("a.atan2(): ", a.atan2());
  console.log("a.length(): ", a.distance);

  console.log("pv.pointPolygonTest: ", pv.pointPolygonTest(new Point(10, 10)));
  console.log("pv.pointPolygonTest: ", pv.pointPolygonTest(new Point(200, 200)));

  console.log("pv.pointPolygonTest: ", pv.pointPolygonTest(new Point(10, 10), true));
  console.log("pv.pointPolygonTest: ", pv.pointPolygonTest(new Point(200, 200), true));

  console.log("poly.length: ", poly.length);
  console.log("hull.length: ", hull.length);
  console.log("poly.boundingRect(): ", poly.boundingRect());
  console.log("pv.minAreaRect(): ", pv.minAreaRect());
  console.log("pv.fitEllipse(): ", pv.fitEllipse());
  console.log("pv.fitLine(): ", pv.fitLine());
  console.log("hull.boundingRect(): ", hull.boundingRect());
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

  var outlines = {
    contours, hier
  };

  let orig = imgOriginal;

  console.log(typeof contours);
  console.log("contours: ", contours);

  /*
  console.log("orig.rows =", orig.rows);
  console.log("orig.cols =", orig.cols);*/

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
var ctor = Point.prototype.constructor;
console.log("Classes: ", inspect(lib));
console.log("Point: ", inspect(Point));
console.log("typeof(Point.prototype.constructor): ", typeof Point.prototype.constructor == "function");
console.log("typeof(Point): ", typeof Point);
console.log("ctor.name: ", ctor.name);
/*console.log("Point.prototype: ", Point.prototype);
console.log("Point.prototype.constructor: ", Point.prototype.constructor);
*/
let points = [new Point(0, 0), new Point(50, 0), new Point(100, 0), new Point(100, 50), new Point(100, 100), new Point(100, 200)];
console.log("points[0]: ", points[0]);
console.log("points[last]: ", points[points.length - 1]);
console.log("points: ", points.map(p => `{x:${p.x},y:${p.y}}`).join(", "));

testPointVector();
