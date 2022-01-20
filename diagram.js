import { Point, Size, Rect, Line, TickMeter, Mat, Draw, LINE_AA } from 'opencv';
import { range, mod } from 'util';

export const X = 0;
export const Y = 1;

const COORD = ['x', 'y'];
const SIZE = ['width', 'height'];

export function Max(stops, style, axis) {
  return Math.max(...stops.map(stop => style.size(stop)[SIZE[axis]]));
}

export function AxisPoints(stops, inc = 10, axis, style, size) {
  if(typeof stops == 'number') stops = range(0, stops, inc);
  let positions = [];
  let points = range(0, Math.abs((stops.length - 1) * inc), inc);
  let sizes = [];
  let n = points.length;

  /*for(let stop of stops) {
    sizes.push(style.size(stop)[SIZE[axis]]);
  }*/
  let max = Max(stops, style, axis);
  let pos = max + 2 + 10;

  for(let i = 0; i < n; i++) {
    let point = points[i];
    let y = axis ? point : pos;
    let x = axis ? pos : point;

    positions.push([stops[i], new Point(x, y)]);
  }

  console.log('sizes', sizes);
  return positions;
}

export function DrawLine(mat, start, end, color = [0, 0, 0], width = 1, lineType = LINE_AA) {
  let points = [start, end].map(({ x, y }) => [mod(x, mat.cols), mat.rows - mod(y, mat.rows)]);

  Draw.line(mat, ...points, color, width, lineType);
}

export function DrawAxis(mat, axis, origin) {
  let points = axis.map(([label, point]) => point);
  let diff = Point.diff(points[1], points[0]);
  let norm = diff.norm();
  let xAxis = norm.x ? true : false;
  let add = xAxis ? new Point(0, -10) : new Point(-10, 0);
  let offset = xAxis ? new Point(origin.x, 0) : new Point(0, origin.y);

  points = points.map(p => p.sum(offset));

  let first = points[0],
    last = points[points.length - 1];

  console.log('DrawAxis', { offset, origin });

  DrawLine(mat, first, last);

  for(let point of points) {
    let points = [point, point.sum(add)];
    //  console.log(`${xAxis ? 'x' : 'y'} point`, { points });

    DrawLine(mat, ...points);
  }
}

export function DrawCross(mat, point, radius = 10) {
  DrawLine(mat, point.sum([-10, -10]), point.sum([10, 10]), [0, 0, 255], 2, LINE_AA);
  DrawLine(mat, point.sum([10, -10]), point.sum([-10, 10]), [0, 0, 255], 2, LINE_AA);
}

export function Origin(mat, xAxis, yAxis) {
  const { size } = mat;

  let { x } = yAxis[0][1];
  let { y } = xAxis[0][1];

  return new Point(x, y);
}

export function GetRect(mat, xAxis, yAxis) {
  let origin = Origin(mat, xAxis, yAxis);

  let end = new Point(mat.cols - 1 - origin.x, mat.rows - 1 - origin.y);

  return new Rect(...origin, ...end);
}
