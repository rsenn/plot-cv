import ClipperLib from './lib/clipper-lib.js';
import SvgPath from './lib/svg/path.js';
import { parse, parseSVG } from './lib/svg/path-parser.js';
import { Console } from 'console';
import Shape from './lib/clipper.js';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 7, colors: true }
});

const d =
  'M 193.54706,178.86683 163.80521,218.90155 116.21174,233.8085 68.945718,217.89373 40.061173,177.23615 40.591015,127.36556 70.332862,87.330839 117.92634,72.423889 165.19236,88.338658 194.0769,128.99624 Z';

const data = parse(d).filter(({ x, y }) => x !== undefined);

function testOffset() {
  const path = data.map(({ x, y }) => new ClipperLib.DoublePoint(x, y));
  //console.log('ClipperLib:', ClipperLib);

  let area = ClipperLib.JS.AreaOfPolygon(path);
  let bounds = ClipperLib.JS.BoundsOfPath(path);
  let lightened = ClipperLib.JS.Lighten(path, 10);
  let perimeter = ClipperLib.JS.PerimeterOfPath(path, 1);

  const offset = new ClipperLib.ClipperOffset();
  const outer = new ClipperLib.Paths();
  offset.AddPath(path, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etClosedPolygon);

  offset.Execute(outer, 10);
  console.log('testOffset:', { path, area, bounds, lightened, outer });
}
function testClipper() {
  const subjectPaths = [
    [
      { X: 30, Y: 30 },
      { X: 10, Y: 30 },
      { X: 10, Y: 10 },
      { X: 30, Y: 10 }
    ]
  ];
  const clipPaths = [
    [
      { X: 20, Y: 20 },
      { X: 0, Y: 20 },
      { X: 0, Y: 0 },
      { X: 20, Y: 0 }
    ]
  ];

  const result = new ClipperLib.Paths();
  const clipper = new ClipperLib.Clipper();
  clipper.AddPaths(subjectPaths, ClipperLib.PolyType.ptSubject, true);
  clipper.AddPaths(clipPaths, ClipperLib.PolyType.ptClip, true);
  clipper.Execute(ClipperLib.ClipType.ctIntersection, result);

  console.log('testClipper:', result);
}

function testShape() {
  let shape = new Shape([data], true, true);

  let area = shape.totalArea();
  let bounds = shape.shapeBounds();
  let simplified = shape.simplify().mapToLower();
  let rounded = shape.round().mapToLower();
  let outset = shape.offset(10, {
    jointType: 'jtSquare',
    endType: 'etClosedPolygon',
    miterLimit: 2.0,
    roundPrecision: 0.25
  });
  let inset = shape.offset(-10, {
    jointType: 'jtSquare',
    endType: 'etClosedPolygon',
    miterLimit: 2.0,
    roundPrecision: 0.25
  });

  console.log('testShape:', { area, bounds, simplified, rounded, outset, inset });
}

testOffset();
testClipper();
testShape();
