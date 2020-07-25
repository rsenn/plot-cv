import Util from './lib/util.js';
import { Point, PointList, Line, BBox } from './lib/geom.js';
import { SVG } from './lib/dom.js';
import { toXML } from './lib/json.js';
import Voronoi from './lib/geom/voronoi.js';
import { Console } from 'console';
import fs from 'fs';
import { EagleDocument, EagleProject } from './lib/eagle.js';

Error.stackTraceLimit = 1000;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 5, colors: true }
});

const filesystem = {
  readFile(filename) {
    let data = fs.readFileSync(filename).toString();
    return data;
  },
  writeFile(filename, data, overwrite = true) {
    return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' });
  },
  exists(filename) {
    return fs.existsSync(filename);
  },
  realpath(filename) {
    return fs.realpathSync(filename);
  }
};

function testVoronoi(filename) {
  //Util.log('Loading document: ' + filename);
  let doc = new EagleDocument(filesystem.readFile(filename), null, filename);

  //Util.log('doc', doc);
  let points = new PointList();

  for(let element of doc.elements.list) {
    const pkg = element.package;
    let { x, y } = element;
    //Util.log('element:', element, { x, y });
    let origin = new Point(x, y);

    for(let item of pkg.children) {
      if(item.drill !== undefined) {
        let pos = new Point(+item.x, +item.y).add(origin);
        //Util.log('pos:', pos);

        points.push(pos);
      }
    }
  }

  let bb = new BBox();
  for(let item of doc.plain) {
    if(item.layer.number != 47) continue;

    bb.update(item.geometry());

    //Util.log('item:', item);
  }

  var sites = points.map(p => p.toObject());
  //xl, xr means x left, x right
  //yt, yb means y top, y bottom
  //Util.log('bbox:', bb);

  var bbox = { xl: bb.x1, xr: bb.x2, yt: bb.y1, yb: bb.y2 };
  var voronoi = new Voronoi();
  //pass an object which exhibits xl, xr, yt, yb properties. The bounding
  //box will be used to connect unbound edges, and to close open cells
  let result = voronoi.compute(sites, bbox);
  //render, further analyze, etc.
  //Util.log('result:', Object.keys(result).join(', '));

  let { site, cells, edges, vertices, execTime } = result;
  //Util.log('cells:', cells);

  let holes = edges.filter(e => !e.rSite).map(({ lSite, rSite, ...edge }) => new Point(lSite));
  let rlines = edges.filter(e => e.rSite).map(({ lSite, rSite, ...edge }) => new Line(lSite, rSite));
  let vlines = edges.filter(e => e.va && e.vb).map(({ va, vb, ...edge }) => new Line(va, vb).round(0.127, 4));
  let points2 = vertices.map(v => new Point(v).round(0.127, 4));
  const add = (arr, ...items) => [...(Util.isArray(arr) ? arr : []), ...items];

  const factory = SVG.factory({
    create: tag => ({ tagName: tag }),
    append_to: (elem, parent) => (parent.children = add(parent.children, elem)),
    setattr: (elem, name, value) => {
      if(!elem.attributes) elem.attributes = {};
      elem.attributes[name] = value;
    }
  });

  const lines = [...rlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#000', 'stroke-width': 0.1 }]), ...vlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#f00', 'stroke-width': 0.1 }])];

  const circles = [...holes.map(p => ['circle', { cx: p.x, cy: p.y, r: 0.254, fill: 'none', stroke: '#00f', 'stroke-width': 0.1 }]), ...points2.map(p => ['circle', { cx: p.x, cy: p.y, r: 0.254 * 2, fill: 'none', stroke: '#0f0', 'stroke-width': 0.1 }])];

  const polylines = [...cells.reduce((acc, { site, halfedges }) => [...acc, ['polyline', { points: new PointList(halfedges.map(({ site }) => site)).toString(), stroke: '#f0f', 'stroke-width': 0.1 }]], [])];

  const svg = ['svg', { viewBox: bb.toString() }, [['defs'], ...lines, ...circles, ...polylines]];

  //Util.log('factory:', factory);
  const svgFile = toXML(factory(...svg));
  filesystem.writeFile('output.svg', svgFile);
  //Util.log('svg:', svgFile);
}
(() => {
  let args = process.argv.slice(2);
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd');
  for(let arg of args) {
    try {
      let project = testVoronoi(arg);
    } catch(err) {
      //Util.log('Err:', err.message, err.stack);
      throw err;
    }
  }
})();