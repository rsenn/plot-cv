import { mod, range } from 'util';
import { CV_64FC1, CV_RGB, Draw, Line, LINE_8, LINE_AA, Mat, Point, Rect } from 'opencv';

export const X = Symbol.for('x');
export const Y = Symbol.for('y');

const PERP = { [X]: Y, [Y]: X };

const COORD = { [X]: 'x', [Y]: 'y' };

const SIZE = { [X]: 'width', [Y]: 'height' };

export function LinearTransform(sx, sy, tx, ty) {
  let m = new Mat(2, 3, CV_64FC1);
  Object.assign(m.array, [sx, 0, tx, 0, sy, ty]);
  return m;
}

export function Max(stops, axis, style) {
  let values;
  if(style) {
    if(Array.isArray(stops[0])) stops = stops.map(([label]) => label);

    values = stops.map(stop => style.size(stop)[SIZE[axis]]);
    //console.log('Max', values);
  } else values = stops.map(stop => stop[COORD[axis]]);

  return Math.max(...values);
}

export function AxisPoints(stops, inc = 10, axis, style, size) {
  if(typeof stops == 'number') stops = range(0, stops, inc);
  let positions = [];
  let points = range(0, Math.abs(stops.length - 1));
  let sizes = [];
  let n = points.length;

  let max = Max(stops, PERP[axis], style);
  let pos = max + 2 + 10 + (axis == X ? 1 : 0);
  //console.log('AxisPoints', { axis, max, pos });

  for(let i = 0; i < n; i++) {
    let point = points[i];
    let y = axis == Y ? point : pos;
    let x = axis == Y ? pos : point;

    positions.push([stops[i], new Point(x, y)]);
  }

  //console.log('sizes', sizes);
  return positions;
}

export function AxisRange(axis) {
  let [start] = axis[0];
  let [end] = axis[axis.length - 1];
  return [start, end];
}

export function DrawLine(mat, start, end, color = CV_RGB(0, 0, 0), width = 1, lineType = LINE_AA) {
  let points = [start, end].map(({ x, y }) => [mod(x, mat.cols), mat.rows - mod(y, mat.rows)]);

  // if(lineType == LINE_AA) throw new Error('LINE_AA');

  //console.log('DrawLine', inspect({ points, width, lineType }, { compact: 2 }));

  Draw.line(mat, ...points, color, width, lineType);
}

export function DrawRect(mat, [x1, y1], [x2, y2], color = CV_RGB(0, 0, 0), width = 1, lineType = LINE_AA) {
  console.log('DrawRect', { x1, y1, x2, y2 }, { color, width, lineType });

  DrawLine(mat, [x1, y1], [x2, y1], color, width, lineType);
  DrawLine(mat, [x2, y1], [x2, y2], color, width, lineType);
  DrawLine(mat, [x2, y2], [x1, y2], color, width, lineType);
  DrawLine(mat, [x1, y2], [x1, y1], color, width, lineType);
}

export function DrawDottedLine(mat, start, end, c = CV_RGB(0, 0, 0)) {
  let a,
    i = 0,
    axis = start.x == end.x ? Y : start.y == end.y ? X : null;
  switch (axis) {
    case X: {
      let r = [start.x, end.x].sort((a, b) => a - b);
      let row = mat.row(mat.rows - start.y);
      a = row.colRange(...r);
      break;
    }
    case Y: {
      let r = [mat.rows - start.y, mat.rows - end.y].sort((a, b) => a - b);
      let col = mat.col(start.x);
      a = col.rowRange(...r);
      break;
    }
  }
  for(let p of a) {
    if(i & 1) {
      p[0] = c[0];
      p[1] = c[1];
      p[2] = c[2];
    }
    i++;
  }
}

export function DrawText(mat, text, pos, style, color = CV_RGB(0, 0, 0)) {
  let pt = new Point(pos.x, mat.rows - pos.y);

  style.draw(mat, text, pt, color);
}

export function DrawAxis(mat, axis, rect, style) {
  let { x1, y1, x2, y2 } = rect;

  let points = axis.map(([label, point]) => point);
  let diff = Point.diff(points[1], points[0]);
  let norm = diff.norm();
  let prop = norm.x ? X : Y;
  let add = prop == X ? new Point(0, -8) : new Point(-8, 0);
  let offset = prop == X ? new Point(rect.x, 0) : new Point(0, rect.y);

  let max = Max(points, prop);

  let maxCoord = rect[SIZE[prop]];
  let factor = maxCoord / max;
  let multiplicator = prop == X ? new Point(factor, 1) : new Point(1, factor);

  //console.log('DrawAxis', { max, factor, multiplicator });
  points = points.map(p => p.prod(multiplicator).round());
  points = points.map(p => p.sum(offset));
  //console.log('DrawAxis', { points });

  let first = points[0],
    last = points[points.length - 1];

  const lineColor = CV_RGB(0, 0, 0);

  let i = 0;
  for(let point of points) {
    let text = axis[i][0] + '';
    let points = [];

    let dimensions = style.size(text);
    let point2 = point.sum(add);

    let disp = {
      [X]: new Point(-dimensions.width / 2 + 1, 0),
      [Y]: new Point(-dimensions.width, dimensions.height / 2 + 2)
    }[prop];
    let textpos = point2.sum(disp.ceil());

    DrawText(mat, text, textpos, style, CV_RGB(0, 0, 0));

    let point3 = { [X]: new Point(point.x, rect.y2), [Y]: new Point(rect.x2, point.y) }[prop];
    // console.log('DrawAxis',prop, {point,point2, point3});

    if(i > 0) DrawDottedLine(mat, point3, point, CV_RGB(200, 200, 200), 1, LINE_8);

    DrawLine(mat, point, point2, lineColor, 1, LINE_8);

    ++i;
  }

  DrawLine(mat, first, last, lineColor, 1, LINE_8);
}

export function DrawCross(mat, point, radius = 10) {
  DrawLine(mat, point.sum([-10, -10]), point.sum([10, 10]), [0, 0, 255], 2, LINE_8);
  DrawLine(mat, point.sum([10, -10]), point.sum([-10, 10]), [0, 0, 255], 2, LINE_8);
}

export function Origin(mat, xAxis, yAxis) {
  const { size } = mat;

  let { x } = yAxis[0][1];
  let { y } = xAxis[0][1];

  return new Point(x, y);
}

export function Flip(mat, geometry) {
  geometry = geometry.clone();
  if(geometry instanceof Rect || geometry instanceof Line) {
    geometry.y = mat.rows - geometry.y2;
  } else if(geometry instanceof Point) {
    geometry.y = mat.rows - geometry.y;
  } else if(types.isArrayLike(geometry) || types.isIterable(geometry)) {
    return [...geometry].map(g => Flip(mat, g));
  } else {
    throw new Error(`No such geometry type: ${geometry}`);
  }
  return geometry;
}

export function GetRect(mat, xAxis, yAxis, style) {
  let origin = Origin(mat, xAxis, yAxis);

  let end = new Point(mat.cols - 1 - origin.x, mat.rows - 1 - origin.y);

  let yAdd = Math.floor(Max(yAxis, Y, style) / 2);
  let xAdd = Math.ceil(Max(xAxis, X, style) / 2);

  return new Rect(...origin, ...end.diff(xAdd, yAdd));
}

export function ClientRect(mat, xAxis, yAxis, style) {
  const rect = GetRect(mat, xAxis, yAxis, style);
  return Flip(mat, rect);
}

export function ClientArea(mat, xAxis, yAxis, style) {
  return mat(ClientRect(mat, xAxis, yAxis, style));
}

export function ClientMatrix(mat, xAxis, yAxis, style) {
  let [minX, maxX] = AxisRange(xAxis);
  let [minY, maxY] = AxisRange(yAxis);

  let clientRect = ClientRect(mat, xAxis, yAxis, style);

  return LinearTransform(clientRect.width / (maxX - minX), -clientRect.height / (maxY - minY), 0, clientRect.height);
}