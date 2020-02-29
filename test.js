import { Point } from './point.js';
//import Util from './lib/util.js';

function test() {
  var pt = new Point(100,50);
  var pt2 = new Point(150,230);
  var pt3 = Point.diff(pt2, pt);
  var contour = [ pt, pt, pt, pt, pt, pt, pt ];

  if(this !== undefined && this.drawContour)
    this.drawContour(contour, [255,0,0]);
  console.log("contour: ", contour);
  console.log("args: ", scriptArgs);
  console.log("pt: ", pt.toString(true));
  console.log("diff: ", pt3);
  console.log("angle: ", pt3.toAngle(true));
  console.log("distance: ", pt3.distance());

  //console.log("drawContour: ", drawContour);
}

function process(contours, hier) {
  console.log("PROCESS contours: ", contours.map(c => '['+c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ")+']').join(", "));
  console.log("PROCESS hier: ", '['+hier.map(h => `[${h.join(",")}]`).join(", "));
}

test();
