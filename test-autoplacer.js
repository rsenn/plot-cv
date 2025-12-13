import { Alea } from './lib/alea.js';
import Autoplacer from './lib/autoplacer/autoplacer.js';
import { BBox } from './lib/geom/bbox.js';
import { Rect } from './lib/geom/rect.js';
let rg = new Alea(1337);
let bb = new BBox();
let rects = [];

for(let i = 0; i < 10; i++) {
  let r = new Rect(0, 0, Math.round(rg() * 100), Math.round(rg() * 100));

  rects.push(r);
}

let bl = rects.map(rect => {
  const [left, top] = [...rect];
  const { width, height } = rect;
  //console.log({ left, top });
  return [left, top, width, height];
});

//console.log('rg:', rg());
//console.log('bb:', bb);
//console.log('bl:', bl);

let ap = new Autoplacer({ bodies: bl });

console.log('ap.next():', ap.next());

ap.next();

console.log('ap:', ap.bodies_);