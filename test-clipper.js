import ClipperLib from './lib/clipper-lib.js';
import Shape from './lib/clipper.js';
import { SVG } from './lib/dom.js';
import { Point, PointList } from './lib/geom.js';
import { parse } from './lib/svg/path-parser.js';
import { Console } from 'console';
globalThis.console = new Console({
  inspectOptions: {
    maxStringLength: 200,
    maxArrayLength: 10,
    breakLength: 100,
    compact: 1,
    depth: 10
  }
});

const d =
  'M 193.54706,178.86683 163.80521,218.90155 116.21174,233.8085 68.945718,217.89373 40.061173,177.23615 40.591015,127.36556 70.332862,87.330839 117.92634,72.423889 165.19236,88.338658 194.0769,128.99624 Z';

const d2 = 'M6.13 26.94L16.33 4.5l4.887 25.657 13.16-26.689 5.545 1.948 14.276 4.896-18.561 8.397-7.08 15.744 30.796-4.765 8.73-18.562-1.895-3.904.087-.066';

const data = new PointList(SVG.parsePath(d).commands.filter(({ args }) => args[0] !== undefined && args[2] !== undefined));

const data2 = new PointList(parse(d2).filter(({ x, y }) => x !== undefined && y !== undefined));

function testOffset() {
  console.log('data2:', data2);
  const path = [...data2].map(({ x, y }) => new Point(x, y));
  console.log('path:', path);

  let area = ClipperLib.JS.AreaOfPolygon(path);
  let bounds = ClipperLib.JS.BoundsOfPath(path);
  let lightened = ClipperLib.JS.Lighten(path, 10);
  let perimeter = ClipperLib.JS.PerimeterOfPath(path, 1);

  const offset = new ClipperLib.ClipperOffset();
  const outer = new ClipperLib.Paths();

  offset.AddPath(path, ClipperLib.JoinType.jtRound, ClipperLib.EndType.etOpenRound);
  console.log('offset', offset);

  offset.Execute(outer, 1);
  console.log('outer', outer);

  let points = new PointList(outer[0].map(({ X, Y }) => new Point(X, Y)));
  //console.log('data2:', data2.toPath());
  //console.log('points:', points.toPath());
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

  //console.log('testClipper:', result);
}

function testShape() {
  let shape = new Shape([data2], true, true);

  let area = shape.totalArea();
  let bounds = shape.shapeBounds();
  let simplified = shape.simplify().mapToLower();
  let rounded = shape.round().mapToLower();
  let outset = shape.offset(10, {
    jointType: 'jtSquare',
    endType: 'etOpenSquare',
    miterLimit: 2.0,
    roundPrecision: 0.25
  });
  let points = new PointList(outset.paths[0].map(({ X, Y }) => new Point(X, Y)));

  /*  let inset = shape.offset(-10, {
    jointType: 'jtSquare',
    endType: 'etOpenSquare',
    miterLimit: 2.0,
    roundPrecision: 0.25
  });*/

  let path = points.toPath();

  //console.log('testShape:', { path, outset });
}

testOffset();

testClipper();

testShape();