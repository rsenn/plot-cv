import { Point } from "point.js";
import { Size } from "size.js";
import { Line } from "line.js";
import { Rect } from "rect.js";
import { PointList } from "pointList.js";
const lib = { Point, Size, Line, Rect };

global.process = function(contours, hier) {

  contours.sort((a,b) => a.length - b.length);
  contours = contours.filter(c => c.length  >= 4);

  for(var i = 0; i < contours.length; i++) {
    const [ next, prev, child, parent ] = hier[i];
    var list = new PointList(contours[i]);
    var bbox = list.bbox();
    var rect = new Rect(bbox);
/*
    if(child == -1 && parent == -1)
      continue;*/

      if(parent != -1)
continue;
    console.log(`contour #${i} length=${(contours[i].length+'').padStart(5,' ')} :`, bbox, " rect:", rect, " ", inspect({ next, prev, child, parent }));
  }
  // console.log("Num contours:", contours.length);
  //  console.log("Num hier:", hier.length);
  //  if(do_log) {
  //    console.log("PROCESS contours: ", contours.map(c => "[" + c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ") + "]").join(", "));
  //    console.log("PROCESS hier: ", "[" + hier.map(h => `[${h.join(",")}]`).join(", "));
  //  }
};
global.inspect = function(obj) {
  return (
    "{" +
    Object.entries(obj)
      .map(([key, value]) => {
        let out = value;
        if(typeof out != "string") {
          try {
            if(typeof out == "object") out = inspect(out);
            else out = out + "";
          } catch(err) {
            out = typeof out;
          }
          if(typeof value == "function") {
            let idx = out.indexOf("{");
            out = out.substring(0, idx).trim();
          }
        } else {
          out = '"' + out + '"';
        }
        out = out.replace(/\n/g, "\\n");
        if(out.length > 200) out = out.substring(0, 200) + "...";
        return key + ": " + out;
      })
      .join(", ") +
    "}"
  );
};
//function test() {
//  var classes = { Point_, Line, Size, Rect };
//  console.log("point:", inspect(Point_));
//  console.log("Typeof point:", typeof Point_);
//  console.log("Classes:", inspect(classes));
//  var pt = new Point(100, 50);
//  var pt2 = new Point(150, 230);
//  var pt3 = pt2.diff(pt);
//  var contour = [pt, pt, pt, pt, pt, pt, pt];
//  var l = new Line(pt, pt2);
//  if(this !== undefined && this.drawContour) this.drawContour(contour, [255, 0, 0]);
//  console.log("contour: ", contour);
//  console.log("args: ", scriptArgs);
//  console.log("pt: ", pt.toString(true));
//  console.log("diff: ", pt3);
//  console.log("angle: ", pt3.toAngle(true));
//  console.log("distance: ", pt3.distance());
//  console.log("line length: ", l.length());
//  console.log("line a: ", l.a);
//  console.log("line b: ", l.b);
//  let { x1, y1, x2, y2 } = l;
//  console.log("line: ", inspect({ x1, y1, x2, y2 }));
//  console.log("line slope: ", l.slope());
//  console.log("line direction: ", l.direction());
//  //console.log("drawContour: ", drawContour);
//}
//test();
console.log("classes:", Point, Size, Line, Rect);
//console.log("lib:", inspect(lib));
