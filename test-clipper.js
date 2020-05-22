import ClipperLib, {  FPoint2, JS, ClipperOffset, JoinType, EndType } from './lib/clipper.js';
import SvgPath from './lib/svg/path.js';
import { parse, parseSVG } from './lib/svg/path-parser.js';
const d="M 193.54706,178.86683 163.80521,218.90155 116.21174,233.8085 68.945718,217.89373 40.061173,177.23615 40.591015,127.36556 70.332862,87.330839 117.92634,72.423889 165.19236,88.338658 194.0769,128.99624 Z";

const data = parse(d);
const  path = data.filter(({ x,y }) => x !== undefined).map(({x,y}) => new FPoint2(x,y));
console.log('ClipperLib:', ClipperLib);
let area = JS.AreaOfPolygon(path);
let bounds = JS.BoundsOfPath(path);
let lightened = JS.Lighten(path, 10);
let perimeter = JS.PerimeterOfPath(path, 1);

const offset = new ClipperOffset();

offset.AddPath(path, JoinType.jtRound, EndType.etClosedPolygon);

offset.DoOffset(10);

let outer = offset.m_destPoly;

console.log('output:',{  path,area, bounds, lightened, perimeter, outer });
