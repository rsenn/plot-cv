import { Matrix } from "./js/matrix.js";
import { PointList } from "./js/pointList.js";
import inspect from "./js/inspect.js";

let m = new Matrix();

let rows = m.rows();

console.log("length: ", m.length);

m.scale(2, 2);
m.rotate(Math.PI / 2);
m.translate(50, 50);

console.log("m = ", m);
let dec = m.decompose(false);

let src = new PointList([
  { x: 0, y: 0 },
  { x: 320, y: 0 },
  { x: 320, y: 320 },
  { x: 0, y: 320 }
]);
let dst = new PointList([
  { x: 640, y: 0 },
  { x: 640, y: 640 },
  { x: 0, y: 640 },
  { x: 0, y: 0 }
]);

m.affine_transform(src, dst);
let affine = m.decompose(false);

console.log(
  "affine_transform = ",
  inspect(affine, (v, k) => typeof v != "function" && k != "toString")
);
