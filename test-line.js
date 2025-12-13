import { LineList, PointList, Rect } from './lib/geom.js';

let rect = new Rect(19, 13, 193, 127);

let pl = rect.toPoints(points => new PointList(points));
let ll = rect.toLines(lines => new LineList(lines));

const sym = Symbol.for('nodejs.util.inspect.custom');

/*console.log("pl[0][sym]:", pl[0][sym]+'');
//console.log("ll[0][sym]:", ll[0][sym]+'');
*/

//let ll = new LineList([new Line(19,13,19,140), new Line(19,140,212,140), new Line(212,140,212,13), new Line(212,13,19,13) ]);

//console.log('pl:', pl);
//console.log('ll:', ll);