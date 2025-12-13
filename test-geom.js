import { Matrix, Point, PointList, Voronoi } from './lib/geom.js';

async function main(...args) {
  let file = 'lib/geom/point.js';

  if(args.length == 0) args = ['0,0', '50,100', '100,100', '100,50'];

  args = args.map(arg => arg.split(',').map(n => +n));

  args = args.map(arg => new Point(arg));

  let list = new PointList(args);

  console.log('list:', list); // , getMethodNames(list, 1,1), getPrototypeChain(list));

  list = list.rotate(1);

  console.log('list:', list);
  list = list.rotate(-2).concat([new Point(33, 33), new Point(66, 66)]);
  console.log('list:', list);

  console.log('list:', list.toString());
  let centroid = list.centroid();

  console.log('list.centroid():', list.clone().centroid());
  console.log('list.avg():', list.clone().avg());
  console.log('list.rect():', list.rect());
  console.log('list.xrange():', list.xrange());
  console.log('list.normalizeX():', list.clone().normalizeX());
  console.log('list.yrange():', list.yrange());
  console.log('list.normalizeY():', list.clone().normalizeY());
  console.log('list.bbox():', list.bbox());
  console.log('list.area():', list.area());
  console.log('list.lines():', [...list.lines()]);
  console.log('list.rotate(-2):', list.clone().rotate(-2));
  console.log('list.rotate(1):', list.clone().rotate(1));
  console.log('list.reverse():', list.clone().reverse());
  console.log('list.toPolar():', list.clone().toPolar());
  console.log('list.translate(-50,-50):', list.clone().translate(-50, -50));

  let m = new Matrix()
    .scale(0.5)
    .translate(-50, -50)
    .rotate(Math.PI / 2);

  console.log('list.transform(m):', list.clone().transform(m));
  console.log('list.toString():', list.clone().toString());

  let l = list.map(centroid.diff()).prod(100).floor().quot(100);

  console.log('l.toSource():', l.toSource({ plainObj: true, asString: true }));
  console.log('l:', l);
  let l2 = new PointList('-47.88,5.53 52.120000000000005,55.53 2.12,-44.47 -47.88,-44.47 19.12,22.53 -13.88,-10.47');
  console.log('l2:', l2);
  let bbox = { xl: 0, xr: 800, yt: 0, yb: 600 };

  let v = new Voronoi();
  console.log('v:', v);
  let computation = v.compute(l2, bbox);

  computation.vertices.map(p => Object.setPrototypeOf(p, Point.prototype));
  console.log('compute:', computation);

  let vertices = computation.vertices; /*.map(p => new Point(p))*/

  console.log('vertices:', vertices);
  let result = new PointList(vertices);
  console.log('result:', result);

  console.log('matrix:', m);
  console.log('matrix:', Matrix.identity());
}

main(...scriptArgs.slice(1));