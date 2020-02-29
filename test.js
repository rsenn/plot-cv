import { Point } from './point.js';
//import Util from './lib/util.js';

function test() {
  var contour = [ {x:100,y:50}, {x:100,y:50}, {x:100,y:50}, {x:100,y:50}, {x:100,y:50}, {x:100,y:50}, {x:100,y:50} ];

  if(this !== undefined && this.drawContour)
    this.drawContour(contour, [255,0,0]);
  console.log("contour: ", contour);
  console.log("args: ", scriptArgs);
  //console.log("drawContour: ", drawContour);
}

function process(contours, hier) {
  console.log("PROCESS contours: ", contours.map(c => '['+c.map(pt => `{x:${pt.x},y:${pt.y}}`).join(", ")+']').join(", "));
  console.log("PROCESS hier: ", '['+hier.map(h => `[${h.join(",")}]`).join(", "));
}

test();
