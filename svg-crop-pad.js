#!/usr/bin/env qjsm
import { kill, SIGUSR1 } from 'os';
import { basename, extname } from 'path';
import { define, getOpt, isObject, mapWrapper, roundTo, showHelp, startInteractive } from 'util';
import { iterateTree } from './dom-helpers.js';
import { WriteFile } from './io-helpers.js';
import { BBox, isBBox } from './lib/geom/bbox.js';
import { Matrix } from './lib/geom/matrix.js';
import { Point } from './lib/geom/point.js';
import { PointList } from './lib/geom/pointList.js';
import { Rect } from './lib/geom/rect.js';
import { isSize, Size } from './lib/geom/size.js';
import { MatrixTransformation, Rotation, Scaling, Transformation, TransformationList, Translation } from './lib/geom/transformation.js';
import { parseSVG } from './lib/svg/path-parser.js';
import { SvgPath } from './lib/svg/path.js';
import { unitConv, unitConvFactor, unitConvFunction, unitConvToMM } from './measure-unit.js';
import { Attr, Comment, CSSStyleDeclaration, Document, Element, Entities, Factory, GetType, Interface, NamedNodeMap, Node, NodeList, nodeTypes, Parser, Prototypes, Serializer, Text, TokenList } from './quickjs/qjs-modules/lib/dom.js';
import * as deep from 'deep';
import extendArray from 'extendArray';
import extendGenerator from 'extendGenerator';
import { TreeIterator } from 'tree_walker';
#!/usr/bin/env qjsm
extendGenerator();
extendArray();

function parseSVGPath(s) {
  let { length, segment } = {
    length: { a: 7, c: 6, h: 1, l: 2, m: 2, q: 4, s: 4, t: 2, v: 1, z: 0 },
    segment: /([astvzqmhlc])([^astvzqmhlc]*)/gi
  };

  const n = /-?[0-9]*\.?[0-9]+(?:e[-+]?\d+)?/gi;
  const parseValues = args => {
    let nm = args.match(n);
    return nm ? nm.map(Number) : [];
  };
  let r = new SvgPath();

  if(typeof s != 'string' && isObject(s) && typeof s.getAttribute == 'function') s = s.getAttribute('d');

  s.replace(segment, (_, c, args) => {
    let t = c.toLowerCase();
    args = parseValues(args);
    //overloaded moveTo
    if(t == 'm' && args.length > 2) {
      r.cmd(...[c].concat(args.splice(0, 2)));
      t = 'l';
      c = c == 'm' ? 'l' : 'L';
    }
    while(true) {
      if(args.length == length[t]) {
        args.unshift(c);
        r.cmd(...args);
        return;
      }
      if(args.length < length[t]) throw new Error(`malformed path data (${args.length} < ${length[t]}): ${c} ${args}`);
      r.cmd(...[c].concat(args.splice(0, length[t])));
    }
  });
  return r;
}

function splitPath(ps) {
  let i = 0,
    cmds = [];
  for(let [m] of ps.matchAll(/([a-zA-Z]|-?[0-9]*(?:\.[0-9]+|))/g)) {
    if(m === '') continue;
    if(!isNaN(+m)) m = +m;
    else if(i == 0) m = m.toUpperCase();
    if(typeof m == 'string') cmds.push([m]);
    else cmds[cmds.length - 1].push(m);
    ++i;
  }
  return cmds;
}

function* pathToAbsolute(path) {
  let prev = [0, 0];
  for(let [cmd, x, y] of splitPath(path)) {
    yield [cmd, x + prev[0], y + prev[1]];

    prev = [x ?? prev[0], y ?? prev[1]];
  }
}

function pathToPoints(path) {
  let result = [],
    splitted = splitPath(path);
  let { length } = splitted;
  let lastmove;
  let prev = new Point(0, 0);
  const isUpper = s => s.toUpperCase() == s;
  for(let i = 0; i < length; i++) {
    let pt,
      cmd = splitted[i];
    const MakePoint = (x, y) => Object.assign(new Point(x, y), { cmd });

    switch (cmd[0].toLowerCase()) {
      case 'z':
        pt = MakePoint(+lastmove[1], +lastmove[2]);
        break;
      case 'v':
        pt = MakePoint(0, +cmd[1]);
        break;
      case 'h':
        pt = MakePoint(+cmd[1], 0);
        break;
      case 'z':
        pt = MakePoint(undefined, undefined);
        break;
      default:
        pt = MakePoint(...cmd.slice(-2).map(n => +n));
        if(cmd[0].toLowerCase() == 'm') lastmove = [...pt];
        break;
    }

    define(pt, { cmd });

    result.push(pt);
  }
  return result;
}

function* pointsToAbsolute(it) {
  let prev = new Point(0, 0),
    i = 0;
  const isUpper = s => s.toUpperCase() == s;

  for(let pt of it) {
    let p = isUpper(pt.cmd[0]) ? pt.clone() : pt.sum(prev);
    p.round();
    p.cmd = pt.cmd;
    yield (prev = p);
  }
}

Object.assign(globalThis, {
  Entities,
  nodeTypes,
  Prototypes,
  Factory,
  Parser,
  Serializer,
  Interface,
  Node,
  NodeList,
  NamedNodeMap,
  Element,
  Document,
  Attr,
  Text,
  Comment,
  TokenList,
  CSSStyleDeclaration,
  GetType
});
Object.assign(globalThis, {
  TreeIterator,
  BBox,
  isBBox,
  Size,
  isSize,
  Matrix,
  Point,
  PointList,
  parseSVGPath,
  splitPath,
  pathToPoints,
  pointsToAbsolute,
  NumericArgs,
  ParentPaths,
  CumulativePaths,
  AllParents,
  getTransformationMatrix,
  getTransformationList,
  AllTransforms,
  ElementTransformMatrix,
  ElementTransformLists,
  ElementTransformList,
  DecomposeTransformList,
  TransformedElements,
  GetXY,
  GetPoints,
  GetMatrix,
  GetTransformedPoints,
  GetSVGPath,
  PositionedElements,
  HasParent,
  GetBounds,
  ProcessPath,
  unitConvToMM,
  unitConv,
  unitConvFactor,
  unitConvFunction,
  getViewBox,
  getWidthHeight,
  XML2String
});

Object.assign(globalThis, {
  save(filename) {
    filename ??= basename(globalThis.file, extname(globalThis.file)) + '.out.svg';
    const str = serializer.serializeToString(document);

    let ret = WriteFile(filename, str);
    console.log(`'${filename}' written.`);
    return ret;
  }
});

function* NumericArgs(s, t = a => a) {
  for(let [m] of s.matchAll(/(\s+|[-+.0-9]+|[^-+.0-9\s]+)[a-z]*/g)) if(m.trim() != '') yield t(m);
}

const deref = p => obj => p.reduce((o, k) => o[k], obj);

function* ParentPaths(p) {
  let plen = p.length;
  for(let i = 2; i <= plen; i += 2) yield p.slice(0, i);
}

function* CumulativePaths(p) {
  let plen = p.length;
  for(let i = 2; i <= plen; i += 2) yield p.slice(i - 2, i);
}

function* AllParents(elem) {
  let obj = Node.document(elem);
  for(let p of CumulativePaths(Node.path(elem))) {
    if(obj.tagName[0] != '?') yield obj;
    obj = deref(p)(obj);
  }
  if(obj) yield obj;
}

function AllTransforms(elem, getter = e => getTransformationMatrix(e)) {
  let t = [];
  for(let e of AllParents(elem)) if(e.hasAttribute('transform')) t.push(getter(e));
  if(elem.hasAttribute('transform')) t.push(getter(elem));
  return t;
}

function ElementTransformLists(elem) {
  let map = new Map();

  for(let e of AllTransforms(elem, e => e)) {
    map.set(e, getTransformationList(e));
  }
  return map;
}

function DecomposeTransformList(elem) {
  let list = getTransformationList(elem);

  if(list.length == 1 && list[0].type == 'matrix') {
    let tl = list.decompose();
    //console.log(`Setting '${list}' to ${tl}`);
    elem.setAttribute('transform', tl + '');
  }
}

function ElementTransformList(elem) {
  let ret = new TransformationList();
  for(let tl of ElementTransformLists(elem).values()) ret = ret.concat(tl);
  return ret;
}

function ElementTransformMatrix(elem) {
  return Matrix.multiply(...AllTransforms(elem, getTransformationMatrix));
}

function GetXY(elem) {
  return new Point(+elem.getAttribute('x'), +elem.getAttribute('y'));
}

function GetPoints(elem) {
  let t = [];

  if(elem.tagName == 'symbol') return GetPoints(elem.firstElementChild);

  if(elem.hasAttribute('xlink:href')) {
    let e, points, xy, m, id;
    id = elem.getAttribute('xlink:href');
    xy = GetXY(elem);
    m = new Matrix().translate(...xy);

    if((e = Node.document(elem).querySelector(id))) return GetPoints(e).map(p => p.transform(m));
  }

  if(elem.hasAttribute('d')) {
    try {
      let pp = pathToPoints(elem.getAttribute('d'));
      return pp;
      /*      let pa = Object.setPrototypeOf(pp, PointList.prototype);

      pa.points = pp;

      return pa;*/
    } catch(e) {
      console.log('ERROR', e.message + '\n' + e.stack);
      return null;
    }

    let svgP = parseSVGPath(elem.getAttribute('d'));
    return new PointList(
      svgP.commands
        .map(c => c.args)
        .filter(a => a.length >= 2 && (!isNaN(+a[0]) || !isNaN(+a[1])))
        .map(([x, y]) => new Point(x, y))
    );
  }
  if(elem.hasAttribute('x') && elem.hasAttribute('y')) return new PointList([GetXY(elem)]);

  throw new Error(`Failed getting point data for element ${XML2String(elem)}`);
}

function GetSVGPath(elem) {
  return parseSVGPath(elem.getAttribute('d'));
}

function PathCmdTransform(matrix, cmd) {
  let args = {
    M: (x, y) => matrix.transformXY(x, y),
    Z: (x, y) => matrix.transformXY(x, y),
    L: (x, y) => matrix.transformXY(x, y),
    H: x => matrix.transformXY(x, 0)[0],
    V: y => matrix.transformXY(0, y)[1],
    C: (x1, y1, x2, y2, x, y) => [...matrix.transformXY(x1, y1), ...matrix.transformXY(x2, y2), ...matrix.transformXY(x, y)],
    S: (x2, y2, x, y) => [...matrix.transformXY(x2, y2), ...matrix.transformXY(x, y)],
    Q: (x1, y1, x, y) => [...matrix.transformXY(x1, y1), ...matrix.transformXY(x, y)],
    T: (x, y) => matrix.transformXY(x, y),
    A: (rx, ry, xAxisRotation, largeArcFlag, sweepFlag, x, y) => [...matrix.transformWH(rx, ry), xAxisRotation, largeArcFlag, sweepFlag, ...matrix.transformXY(x, y)]
  }[cmd.name](...cmd.args);

  //console.log('PathCmdTransform', { cmd, args });
  return { name: cmd.name, args };
}

function PathTransform(matrix, path) {
  let ret = new SvgPath();
  ret.commands = path.commands.map(cmd => PathCmdTransform(matrix, cmd));
  return ret;
}

function GetTransformedPoints(elem) {
  let matrix = ElementTransformMatrix(elem);
  let points = new PointList(GetPoints(elem));

  return points.clone().transform(matrix);
}

function GetMatrix(elem) {
  return Matrix.multiply(...AllTransforms(elem));
}

function IsClipPath(elem) {
  let p = [...AllParents(elem)];
  console.log(
    'IsClipPath',
    p.map(e => e.tagName)
  );

  return p.some(e => e.tagName == 'clipPath');
}

function HasClipPath(elem) {
  let p = [...AllParents(elem), elem];

  return p.some(e => e.hasAttribute('clip-path'));
}

function* PositionedElements(svgElem = svg, skip) {
  skip ??= (() => {
    let defs = svgElem.querySelector('defs');
    let defsPath = Node.path(defs).slice(Node.path(svgElem).length);
    return (v, p) => p.slice(0, defsPath.length).equal(defsPath);
  })();

  for(let [value, path] of deep.iterate(Node.raw(svgElem), e => ['d', 'x', 'y'].some(n => n in e.attributes))) {
    if(skip(value, path)) continue;
    let elem = deref(path)(svgElem);

    /* skip them for now */
    if(HasClipPath(elem)) {
      //console.log('PositionedElements skipping', elem);
      continue;
    }
    yield elem;
  }
}

function* TransformedElements(svgElem = svg) {
  for(let q of deep.iterate(Node.raw(svgElem), e => 'transform' in e.attributes, deep.RETURN_PATH)) yield q.reduce((obj, p) => obj[p], svgElem);
}

function HasParent(elem, other) {
  let e = Node.path(elem);
  let o = Node.path(other);

  for(let i = 0; i < o.length; i++) {
    if(o[i] != e[i]) return false;
  }
  return true;
}

let positioned = (globalThis.positioned = new Set());
let positionedProps = (globalThis.positionedProps = mapWrapper(new WeakMap()));

function GetBounds(svgElem = svg) {
  let bb = new BBox();

  for(let element of PositionedElements(svgElem)) {
    /*let matrix = GetMatrix(element);
    let points = GetPoints(element);

    positioned.add(element);
    positionedProps.set(element, { matrix, points });

    for(let point of points.clone().transform(matrix))*/
    for(let point of GetTransformedPoints(element)) bb.update(point, 0, element);
  }

  return bb;
}

function* ProcessPath(d) {
  let c,
    i = 0,
    a = [];
  for(let [m] of d.matchAll(/(\s+|[-+.0-9]+|[^-+.0-9\s]+)/g)) {
    if(m.trim() == '') continue;
    const n = +m;
    if(isNaN(n)) {
      c = m;
      i = 0;
      a = [];
      continue;
    }
    if(c.toLowerCase() == 'v') i = 1;
    i &= 1;
    a[i] = n;
    if(++i >= 2 || 'VvHh'.indexOf(c) != -1) yield a;
  }
}

/*const ToMillimeter = {
  pt: 3l / 8.5l,
  pc: 25.4l/6l,
  in: 25.4l,
  mil: 1l /25.4e3l,
  cm: 10l,
  mm: 1l,
  px: 25.4l/96l
};

function getUnit(str, defaultUnit) {
  const m = /[a-z]+/g.exec(str);
  return m ? m[0] : defaultUnit;
}
function getValue(str) {
  const m = /[a-z]+/g.exec(str);
  return m ? str.slice(0, m.index) : str;
}

function unitConvToMM(value, defaultUnit = 'px') {
  value = value + '';
  const unit = getUnit(value, defaultUnit);
  value = getValue(value);

  console.log('unixConvToMM', { unit, value });

  if (unit in ToMillimeter) return value * ToMillimeter[unit];

  throw new Error(`No such unit '${unit}'`);
}

const MillimeterTo = {
  pt: 8.5l / 3l,
  pc: 6l/25.4l,
  in: 1l /25.4l,
  px: 96l/25.4l,
  mil: 25.4e3l,
  cm: 0.1l,
  mm: 1l,
  m: 0.001l
};
for(let k of Object.keys(ToMillimeter))
  if(ToMillimeter[k] * MillimeterTo[k] != 1l)
    throw new Error(`Invalid unit conv factor for '${k} (${k} -> mm = ${ToMillimeter[k]}) mm -> ${k} = ${MillimeterTo[k]}`)

function unitConvFactor(from, to) {
  return ToMillimeter[from] * MillimeterTo[to];
}

function unitConvFunction(toUnit = 'mm', fromUnit = 'px') {
  return value => unitConvToMM(value, fromUnit) * MillimeterTo[toUnit];
}

function unitConv(unit) {
  return value => MillimeterTo[unit] * unitConvToMM(value);
}*/

function getViewBox(svgElem = svg) {
  if(svgElem.hasAttribute('viewBox')) {
    let viewBox = svgElem.getAttribute('viewBox');
    return BBox.fromSVG(viewBox + '');
  }

  //return new BBox(0, 0, ...getWidthHeight(svgElem));
}

function XML2String(elem) {
  return new Serializer().serializeToString(elem);
}

function getWidthHeight(svgElem = svg, t = a => a) {
  if(svgElem.hasAttribute('width') && svgElem.hasAttribute('height')) {
    let width = svgElem.getAttribute('width');
    let height = svgElem.getAttribute('height');

    return new Size(t(width, 'px'), t(height,'px'));
  }

  return new Size(...getViewBox(svgElem).size);
}

function getTransformationList(e) {
  if(e.hasAttribute('transform')) return new TransformationList(e.getAttribute('transform'));
}

function getTransformationMatrix(e) {
  let tl;

  if((tl = getTransformationList(e))) return tl.toMatrix();
}

function main(...args) {
  let debug = 0;
  let unit = 'mm';
  let precision = 1e-3;
  let size = 0;
  let padding = 0;

  Object.assign(globalThis, {
    NumericArgs,
    ProcessPath,
    unitConvToMM,
    unitConv,
    unitConvFunction,
    unitConvFactor,
    getViewBox,
    getWidthHeight,
    getTransformationMatrix,
    Transformation,
    Rotation,
    Translation,
    Scaling,
    MatrixTransformation,
    TransformationList,
    deref,
    parseSVGPath,
    parseSVG,
    splitPath,
    PathCmdTransform,
    PathTransform,Rect,iterateTree
  });

  //  globalThis.console = new Console(std.err, { depth: 2, customInspect: false, compact: false, protoChain: true });

  let opts;
  let params = (globalThis.params = getOpt(
    (opts = {
      help: [false, (_x, _y, opts) => showHelp(opts), 'h'],
      debug: [false, () => ++debug, 'x'],
      unit: [true, a => (unit = a), 'u'],
      precision: [true, a => (precision = +a), 'a'],
      'print-size': [false, null, 'P'],
      'print-viewbox': [false, null, 'V'],
      bounds: [false, null, 'b'],
      size: [true, a => (size = unitConvToMM(a)), 's'],
      interactive: [false, null, 'y'],
      padding: [true, null, 'p'],
      '@': 'files'
    }),
    args
  ));
  let files = params['@'];
  let parser = (globalThis.parser = new Parser());
  let serializer = (globalThis.serializer = new Serializer());
  let print = (file, ...args) => std.puts(args.join(' ') + '\n');

  if(files.length == 0) showHelp(opts, 1);
  if(files.length > 1) print = (file, ...args) => std.puts(file + ': ' + args.join(' ') + '\n');

  for(let file of files) {
    if(params.debug >= 1) console.log('Processing:', file);

    let xml, svg;

    try {
      xml = globalThis.document = parser.parseFromFile((globalThis.file = file), 'utf-8');

      // console.log('xml', console.config({ customInspect: false }), xml);
      svg = globalThis.svg = xml.querySelector('svg');
    } catch(e) {
      console.log(`ERROR loading '${file}'`, e.message + '\n' + e.stack);
    }
    let sizeUnit = (globalThis.size = getWidthHeight(svg));
    let size = (globalThis.size = getWidthHeight(svg, unitConvToMM).round(precision));
    let writeUnits = (globalThis.writeUnits = [sizeUnit.units.width, sizeUnit.units.height]);

    let viewBoxOld = (globalThis.viewBoxOld = getViewBox(svg) ?? new BBox(size));
    //console.log('viewBox', { viewBoxOld });
    // console.log('size', { size }, size.units);
    let xfactor = (globalThis.xfactor = viewBoxOld.width / sizeUnit.width);
    let yfactor = (globalThis.yfactor = viewBoxOld.height / sizeUnit.height);

    if(params['print-size']) {
      size.units = ['', ''];
      print(file, size.toString({ separator: ' x ', unit: 'mm' }));
    }

    if(params['print-viewbox']) {
      print(file, viewBoxOld.toSVG());
    }

    if(params['bounds']) {
      let bb = (globalThis.bb = GetBounds(svg));
      print(file, bb.round(n => roundTo(n, precision)).toSVG());
    }

    let newViewBox,
      viewBox = (globalThis.viewBox = viewBoxOld.inset(0));

    if(params.padding) {
      let conv = writeUnits.map(u => unitConv(u));
      console.log('conv', { writeUnits, conv });

      let pad = (globalThis.pad = [...NumericArgs(params.padding)]).map((a, i) => {
        let f = i & 1 ? xfactor : yfactor;
        let u = writeUnits[(i & 1) ^ 1];

        let idx = (i & 1) ^ 1;

        console.log('idx', idx, u, conv[idx]);

        return Number(conv[idx](a) * f);
      });

      console.log('pad', pad);

      newViewBox = globalThis.newViewBox = viewBox.outset(...pad);
  }

    svg.setAttribute('viewBox', (newViewBox ??= viewBox).toRect(Rect.prototype).roundTo(precision).toString());
  console.log('viewBox', {viewBox,pad,precision,newViewBox});

    const { width, height } = newViewBox;
    
   console.log('newViewBox', newViewBox, newViewBox.toSVG());

    let w = (globalThis.w = width / xfactor);
    let h = (globalThis.h = height / yfactor);
    //console.log('attributes', { w, h });

    svg.setAttribute('width', roundTo(w, 0.001) + writeUnits[0]);
    svg.setAttribute('height', roundTo(h, 0.001) + writeUnits[1]);

    for(let transformed of TransformedElements()) DecomposeTransformList(transformed);

    save(basename(file, '.svg') + '.out.svg');
    //WriteFile(basename(file, '.svg') + '.out.svg', serializer.serializeToString(document));

    if(params.interactive) kill(process.pid, SIGUSR1);
  }
}

try {
  main(...scriptArgs.slice(1));
} catch(e) {
  console.log('ERROR', e.message + '\n' + e.stack);
  startInteractive();
}