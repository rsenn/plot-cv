#!/usr/bin/env qjsm
import { Console } from 'console';
import { kill, SIGUSR1 } from 'os';
import { getOpt, showHelp, isObject, mapWrapper } from 'util';
import { basename, extname } from 'path';
import { Entities, nodeTypes, Prototypes, Factory, Parser, Serializer, Interface, Node, NodeList, NamedNodeMap, Element, Document, Attr, Text, Comment, TokenList, CSSStyleDeclaration, GetType } from './quickjs/qjs-modules/lib/dom.js';
//import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import { BBox, isBBox } from './lib/geom/bbox.js';
import { Size, isSize } from './lib/geom/size.js';
import { TreeIterator } from 'tree_walker';
import { WriteFile } from './io-helpers.js';
import { Matrix, isMatrix, ImmutableMatrix } from './lib/geom/matrix.js';
import { Point } from './lib/geom/point.js';
import { PointList } from './lib/geom/pointList.js';
import { SvgPath } from './lib/svg/path.js';
import extendGenerator from 'extendGenerator';
import extendArray from 'extendArray';
import { basename } from './lib/path.js';
import { read as readXML, write as writeXML } from 'xml';
import * as xml from 'xml';

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
  return new PointList(
    r.commands
      .map(c => c.args)
      .filter(a => a.length >= 2 && (!isNaN(+a[0]) || !isNaN(+a[1])))
      .map(([x, y]) => new Point(x, y))
  );
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
  NumericArgs,
  ParentPaths,
  CumulativePaths,
  AllParents,
  GetTransform,
  AllTransforms,
  GetXY,
  GetPoints,
  GetMatrix,
  PositionedElements,
  HasParent,
  GetBounds,
  ProcessPath,
  unitConvToMM,
  unitConv,
  unitConvTo,
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
}

function GetTransform(elem) {
  if(elem.hasAttribute('transform')) return Matrix.fromCSS(elem.getAttribute('transform'));
}

function AllTransforms(elem) {
  let t = [];
  for(let e of AllParents(elem)) if(e.hasAttribute('transform')) t.push(GetTransform(e));
  if(elem.hasAttribute('transform')) t.push(GetTransform(elem));
  return t;
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

    if((e = Node.document(elem).querySelector(id))) return GetPoints(e).transform(m);
  }

  if(elem.hasAttribute('d')) return parseSVGPath(elem.getAttribute('d'));
  if(elem.hasAttribute('x') && elem.hasAttribute('y')) return new PointList([GetXY(elem)]);

  throw new Error(`Failed getting point data for element ${XML2String(elem)}`);
}

function GetMatrix(elem) {
  return Matrix.multiply(...AllTransforms(elem));
}

function* PositionedElements(svgElem = svg, skip) {
  skip ??= (() => {
    let defs = svgElem.querySelector('defs');
    let defsPath = Node.path(defs).slice(Node.path(svgElem).length);
    return (v, p) => p.slice(0, defsPath.length).equal(defsPath);
  })();

  for(let [value, path] of deep.iterate(Node.raw(svgElem), e => ['d', 'x', 'y'].some(n => e.attributes[n]))) {
    if(skip(value, path)) continue;
    yield deref(path)(svgElem);
  }
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
    let matrix = GetMatrix(element);
    let points = GetPoints(element);

    positioned.add(element);
    positionedProps.set(element, { matrix, points });

    for(let point of points.clone().transform(matrix)) bb.update(point, 0, element);
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

function unitConvToMM(value) {
  if(/pt\s*$/i.test(value)) return (+value.replace(/\s*pt\s*$/gi, '') * 3) / 8.5;
  if(/pc\s*$/i.test(value)) return +value.replace(/\s*pc\s*$/gi, '') * 4.23333;
  if(/in\s*$/i.test(value)) return +value.replace(/\s*in\s*$/gi, '') * 25.4;
  if(/mil\s*$/i.test(value)) return +value.replace(/\s*mil\s*$/gi, '') * 0.0254;
  if(/cm\s*$/i.test(value)) return +value.replace(/\s*cm\s*$/gi, '') * 10;
  if(/mm\s*$/i.test(value)) return +value.replace(/\s*mm\s*$/gi, '');
  if(/px\s*$/i.test(value) || !isNaN(+value)) return +(value + '').replace(/\s*px\s*$/gi, '') / 3.77952755953127906261;
}

const MillimeterTo = {
  pc: mm => mm * 0.23622,
  px: mm => mm * 3.77952755953127906261,
  pt: mm => (mm * 8.5) / 3,
  in: mm => mm / 25.4,
  mil: mm => mm / 0.0254,
  cm: mm => mm * 1e-1,
  mm: mm => mm,
  m: mm => mm * 1e-3
};

function unitConv(unit) {
  return value => MillimeterTo[unit](unitConvToMM(value));
}

function unitConvTo(value, unit) {
  let mm = unitConvToMM(value);
  return MillimeterTo[unit](mm) + unit;
}

function getViewBox(svgElem = svg) {
  if(svgElem.hasAttribute('viewBox')) {
    let viewBox = svgElem.getAttribute('viewBox');
    return BBox.fromString(viewBox + '');
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

    return new Size(t(width), t(height));
  }

  return new Size(...getViewBox(svgElem).size);
}

function main(...args) {
  let debug = 0;
  let unit = 'mm';
  let precision = 1;
  let size = 0;
  let padding = 0;

  Object.assign(globalThis, {
    NumericArgs,
    ProcessPath,
    unitConvToMM,
    unitConvTo,
    MillimeterTo,
    getViewBox,
    getWidthHeight
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

    let xml = (globalThis.document = parser.parseFromFile((globalThis.file = file), 'utf-8'));

    let svg = (globalThis.svg = xml.querySelector('svg'));

    let sizeUnit = (globalThis.size = getWidthHeight(svg));
    let size = (globalThis.size = getWidthHeight(svg, unitConvToMM).round(precision));
    let writeUnits = (globalThis.writeUnits = [sizeUnit.units.width, sizeUnit.units.height]);

    let viewBoxOld = (globalThis.viewBoxOld = getViewBox(svg) ?? size);
    console.log('viewBox', { viewBoxOld });
    console.log('size', { size }, size.units);
    let xfactor = (globalThis.xfactor = viewBoxOld.width / sizeUnit.width);
    let yfactor = (globalThis.yfactor = viewBoxOld.height / sizeUnit.height);

    if(params['print-size']) {
      size.units = ['', ''];
      print(file, size.toString({ separator: 'x' }));
    } else {
      let viewBox = (globalThis.viewBox = viewBoxOld.inset(0));

      if(params.padding) {
        let conv = writeUnits.map(unitConv);
        console.log('writeUnits', writeUnits);
        console.log('conv', conv[0](1), conv[1](1));
        let pad = (globalThis.pad = [...NumericArgs(params.padding)]).map((a, i) => (i & 1 ? xfactor : yfactor) * conv[(i & 1) ^ 1](a));
        console.log('pad', pad);

        viewBox = globalThis.viewBox = viewBox.outset(...pad);
      }

      svg.setAttribute('viewBox', viewBox.toSVG());

      const { width, height } = viewBox;
      console.log('viewBox', { width, height });
      console.log('viewBox', viewBox);

      let w = (globalThis.w = width / xfactor);
      let h = (globalThis.h = height / yfactor);
      console.log('attributes', { w, h });

      svg.setAttribute('width', w + writeUnits[0]);
      svg.setAttribute('height', h + writeUnits[1]);

      WriteFile(basename(file, '.svg') + '.out.svg', serializer.serializeToString(document));
    }

    if(params.interactive) kill(process.pid, SIGUSR1);
  }
}

main(...scriptArgs.slice(1));
