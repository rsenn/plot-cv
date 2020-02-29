import { Point } from "./point.js";
import { Size } from "./size.js";
import { Line } from "./line.js";
import { Rect } from "./rect.js";
import { PointList } from "./pointList.js";
import { RGBA } from "./rgba.js";
import { HSLA } from "./hsla.js";

const lib = { Point, Size, Line, Rect, PointList, RGBA, HSLA };

global.process = function(contours, hier) {
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

      drawContour(list, [255, 0, 255, 255], 8, false);
    }

    contours.sort((a, b) => b.area - a.area);
    areas = contours.map(c => c.area);

    dumpContour(contours[0]);
    drawContour(contours[0], [0, 0, 255, 255], 20, false);

    return contours;
  }

  let polygons = [
  new PointList([{ x: 0, y: 0 }, { x: 320, y: 0 }, { x: 320, y: 240 }, { x: 0, y: 240 }, { x: 0, y: 0 } ]),
   ];

   polygons.push(polygons[0].sum({x:320,y:0}));
   polygons.push(polygons[0].sum({x:320,y:240}));
   polygons.push(polygons[0].sum({x:0,y:240}));

  console.log(`polygons: [\n  ${polygons.join(",\n  ")}\n]`);

  drawPolygon(polygons[0], [0, 255, 255, 255], false);
  drawPolygon(polygons[1], [0, 255, 0, 255], false);
  drawPolygon(polygons[2], [255, 0, 0, 255], false);
  drawPolygon(polygons[3], [255, 0, 255, 255], false);
 
  const do_log = false;

  if(do_log) {
    console.log(
      "PROCESS contours: ",
      contours.map(c => "[" + c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ") + "]").join(", ")
    );
    console.log("PROCESS hier: ", "[" + hier.map(h => `[${h.join(",")}]`).join(", "));
  }
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
