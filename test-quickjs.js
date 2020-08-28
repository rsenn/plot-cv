import { Rect } from './build/x86_64-linux-gnu/quickjs-rect.so';
import { Point } from './build/x86_64-linux-gnu/quickjs-point.so';
import { Size } from './build/x86_64-linux-gnu/quickjs-size.so';
import { Mat } from './build/x86_64-linux-gnu/quickjs-mat.so';
import { PointIterator } from './build/x86_64-linux-gnu/quickjs-point-iterator.so';
import { inspect } from './inspect.js';
import { Contour } from './build/x86_64-linux-gnu/quickjs-contour.so';

//console.log('test:', inspect({ Point, Size, Rect, Mat, PointIterator, Contour }));
import path from './lib/path.js';
import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import * as os from 'os';
import * as std from 'std';
let filesystem;

async function main(...args) {
  filesystem = await PortableFileSystem();


  let mat = new Mat();
  let rect = new Rect(10,100,50,250);
  const {x,y,width,height}=rect;
  console.log(`rect`, inspect(rect));
  console.log(`{x,y,width,height}`, inspect({x,y,width,height}));
  console.log(`Object.getPrototypeOf(rect)`, Object.getPrototypeOf(rect));
  console.log(`rect.prototype`, rect.prototype);
  console.log(`rect.__proto__`, rect.__proto__);
  console.log(`Object.getOwnPropertyNames(rect.__proto__)`, Object.getOwnPropertyNames(rect.__proto__));
  console.log(`Object.keys(rect.__proto__)`, Object.keys(rect.__proto__));
  console.log(`Object.keys(Object.getPrototypeOf(rect))`, Object.keys(Object.getPrototypeOf(rect)));
  console.log(`Object.keys(rect)`, Object.keys(rect));
  console.log(`inspect(rect)`, inspect(rect));
  let point = new Point(25,75);
 console.log(`inspect(point)`, inspect(point));
 /* let c = new Contour();

  c.push(new Point(0, 0));
  c.push(new Point(10, 0));
  c.push(new Point(20, 0));
  c.push(new Point(30, 0));
  c.push(new Point(30, 10));
  c.push(new Point(30, 20));
  c.push(new Point(30, 30));
  c.push(new Point(20, 30));
  c.push(new Point(10, 30));
  c.push(new Point(0, 30));
  c.push(new Point(0, 20));
  c.push(new Point(0, 10));
  c.push(new Point(0, 0));

  for(let point of c) {
    let { x, y } = point;
    //console.log('point:', point, x, y);
  }

    //console.log('contour:', inspect(c));
*/


  console.log(`std.gc`, std.gc);
  console.log(`args`, args);
  console.log(`path`, inspect(path));
  console.log(`console`, Util.inspect(console));
  console.log(`filesystem.realpath('.')`, filesystem.realpath('.'));
  console.log(`filesystem.chdir('..')`, filesystem.chdir('..'));
  console.log(`filesystem.getcwd('.')`, filesystem.getcwd());
  console.log(`std.gc()`, std.gc());


}

Util.callMain(main, true);
