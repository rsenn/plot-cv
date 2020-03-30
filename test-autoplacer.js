import Autoplacer from "./lib/autoplacer/autoplacer.js";
import { BBox } from "./lib/dom/bbox.js";
import { TRBL } from "./lib/dom/trbl.js";
import { Rect } from "./lib/dom/rect.js";
import { Alea } from "./lib/alea.js";

import { Console } from "console";

global.console = new Console({ stdout: process.stdout, stderr: process.stderr, inspectOptions: { depth: 10, colors: true } });

var rg = new Alea(1337);

var bb = new BBox();

var rects = [];

for (var i = 0; i < 10; i++) {
  let r = new Rect(0, 0, Math.round(rg() * 100), Math.round(rg() * 100));

  rects.push(r);
}

var bl = rects.map(rect => {
  const { left, top } = new TRBL(...rect.points());
  const { width, height } = rect;
  console.log({ left, top });
  return [left, top, width, height];
});

console.log("rg:", rg());
console.log("bb:", bb);
console.log("bl:", bl);

var ap = new Autoplacer({ bodies: bl });

console.log("ap.next():", ap.next());

ap.next();

console.log("ap:", ap.bodies_);
