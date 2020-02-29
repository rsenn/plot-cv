import { Point } from "point.js";
import { Size } from "size.js";
import { Line } from "line.js";
import { Rect } from "rect.js";
import { PointList } from "pointList.js";
const lib = { Point, Size, Line, Rect };

global.process = function(contours, hier) {
  contours.sort((a, b) => a.length - b.length);
  contours = contours.filter(c => c.length >= 4);
  var areas = [];
  function dumpContour(c) {
    console.log(
      `contour #${c.id} length=${(c.length + "").padStart(5, " ")} bbox=`,
      c.bbox,
      " rect:",
      c.rect,
      " area=",
      c.area
    );
  }
  for(var i = 0; i < contours.length; i++) {
    const [next, prev, child, parent] = hier[i];
    var list = new PointList(contours[i]);
    var bbox = list.bbox();
    var rect = new Rect(bbox);
    /*
    if(child == -1 && parent == -1)
      continue;*/

    //  if(parent != -1) continue;

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

    //drawContour(list, [255, 0, 255, 255], 8, false);
  }

  contours.sort((a, b) => b.area - a.area);
  areas = contours.map(c => c.area);

  dumpContour(contours[0]);
  drawContour(contours[0], [0, 0, 255, 255], 20, false);

  let poly = new PointList([
    [0, 0],
    [320, 0],
    [320, 240],
    [0, 240],
    [0, 0]
  ]);
  console.log("poly: ", poly);
  drawPolygon(poly, [0, 255, 255, 255], false);
  poly.add(320, 240);
  drawPolygon(poly, [255, 0, 255, 255], false);
  poly.sub(320, 0);
  drawPolygon(poly, [0, 255, 0, 255], false);
  poly.sub(-320, 240);
  drawPolygon(poly, [255, 0, 0, 255], false);

  // console.log("Num contours:", contours.length);
  //console.log("Areas:", areas);

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
