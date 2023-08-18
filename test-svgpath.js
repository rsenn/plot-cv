import { Matrix, Point } from './lib/dom.js';
import { makeAbsolute, parseSVG } from './lib/svg/path-parser.js';
import require from 'require';

const SvgPath = require('./lib/svg/path.js');
const PointAtLength = require('point-at-length');

let args = [...process.argv].slice(2);

let pathStr =
  args.shift() ||
  'M.1 494.1c-1.1 9.5 6.3 17.8 15.9 17.8l32.3.1c8.1 0 14.9-5.9 16-13.9.7-4.9 1.8-11.1 3.4-18.1H380c1.6 6.9 2.9 13.2 3.5 18.1 1.1 8 7.9 14 16 13.9l32.3-.1c9.6 0 17.1-8.3 15.9-17.8-4.6-37.9-25.6-129-118.9-207.7-17.6 12.4-37.1 24.2-58.5 35.4 6.2 4.6 11.4 9.4 17 14.2H159.7c21.3-18.1 47-35.6 78.7-51.4C410.5 199.1 442.1 65.8 447.9 17.9 449 8.4 441.6.1 432 .1L399.6 0c-8.1 0-14.9 5.9-16 13.9-.7 4.9-1.8 11.1-3.4 18.1H67.8c-1.6-7-2.7-13.1-3.4-18.1-1.1-8-7.9-14-16-13.9L16.1.1C6.5.1-1 8.4.1 17.9 5.3 60.8 31.4 171.8 160 256 31.5 340.2 5.3 451.2.1 494.1z';
let path = parseSVG(pathStr);

let translate = (args.shift() + '').split(',').map(n => parseFloat(n));
if(translate.length == 0 || isNaN(translate[0])) translate = [0, 0];

//console.log('translate:', new Point([...translate]));
makeAbsolute(path);

let newPath = new SvgPath();

newPath.rel();

let t = new Matrix();

t.initTranslate(translate[0], translate[1]);
//console.log(`t:` + t.initTranslate, translate);

for(let i = 0; i < path.length; i++) {
  const c = path[i];
  let { code, command, relative } = c;

  if(relative && i == 0) relative = false;

  let { x, y, x0, y0, x1, y1, x2, y2, rx, ry } = c;
  const prec = 5;

  if(!relative) {
    x0 = 0;
    y0 = 0;
  }

  if(x !== undefined) x = +(x - x0).toFixed(prec);
  if(y !== undefined) y = +(y - y0).toFixed(prec);
  if(x1 !== undefined) x1 = +(x1 - x0).toFixed(prec);
  if(x2 !== undefined) x2 = +(x2 - x0).toFixed(prec);
  if(y1 !== undefined) y1 = +(y1 - y0).toFixed(prec);
  if(y2 !== undefined) y2 = +(y2 - y0).toFixed(prec);

  //console.log(`x:`, { x, x1, x2, rx });
  //console.log(`y:`, { y, y1, y2, ry });

  let points = [new Point(x, y), new Point(x1, y1), new Point(x2, y2)];
  //console.log(`path[${i}]:`, c);

  if(!relative) points = points.map(p => (p.x !== undefined ? t.transformPoint({ x: p.x, y: p.y }) : p));

  if(command == 'A') points[0] = new Point(x, y);

  points.push(new Point(rx, ry));

  relative ? newPath.rel() : newPath.abs();
  points = points.map(p => (p.x !== undefined ? { x: p.x.toFixed(3), y: p.y.toFixed(3) } : p));

  switch (code) {
    case 'M': {
      newPath.to(points[0].x, points[0].y);
      break;
    }
    case 'A': {
      const { xAxisRotation, largeArc, sweep } = c;
      newPath.arc(points[3].x, points[3].y, xAxisRotation, largeArc ? 1 : 0, sweep ? 1 : 0, points[0].x, points[0].y);
      break;
    }
    case 'H': {
      newPath.hline(points[0].x);
      break;
    }
    case 'V': {
      newPath.hline(points[0].y);
      break;
    }
    case 'L':
      {
        newPath.line(points[0].x, points[0].y);
        break;
      }
      '';
    case 'C': {
      newPath.bezier3(points[1].x, points[1].y, points[2].x, points[2].y, points[0].x, points[0].y);
      break;
    }
    case 'Q': {
      newPath.bezier2(points[1].x, points[1].y, points[0].x, points[0].y);
      break;
    }
    case 'Z': {
      newPath.close();
      break;
    }
  }
}

const data = newPath.str().trim();

//console.log(`<path d="${data}" />`);
process.stdout.write('\n' + data + '\n');

function* dataToPoints(d, steps = 10) {
  const pointData = PointAtLength(d);
  let len = pointData.length();

  for(let i = 0; i <= steps; i++) {
    const point = new Point(pointData.at((i / steps) * len));

    yield point.round(0.001);
  }
}

//console.log([...dataToPoints(data)]);