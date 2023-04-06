#!/usr/bin/env qjsm
import { IfDebug, LogIfDebug, ReadFd, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';
import { Console } from 'console';
import { kill, SIGUSR1 } from 'os';
import { getOpt, showHelp } from 'util';
import { basename, extname } from 'path';
import { nodeTypes, Parser, Node, NodeList, NamedNodeMap, Element, Document, Attr, Text, TokenList, Factory, Serializer } from './quickjs/qjs-modules/lib/dom.js';
//import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import { BBox, isBBox } from './lib/geom/bbox.js';
import { Size, isSize } from './lib/geom/size.js';
import { TreeWalker, TreeIterator } from 'tree_walker';

let debug = 0;

Object.assign(globalThis, {
  nodeTypes,
  Parser,
  Node,
  NodeList,
  NamedNodeMap,
  Element,
  Document,
  Attr,
  Text,
  TokenList,
  Factory,
  Serializer,
  TreeIterator,
  BBox,
  isBBox,
  ProcessPath,
  getViewBox,
  getWidthHeight,
  Size,
  isSize
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

function* NumericArgs(s) {
  for(let [m] of s.matchAll(/(\s+|[-+.0-9]+|[^-+.0-9\s]+)/g)) yield +m;
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
  if(/pt\s*$/i.test(value)) return +value.replace(/\s*pt\s*$/gi, '') / 2.83464566929133858267;
  if(/pc\s*$/i.test(value)) return +value.replace(/\s*pc\s*$/gi, '') * 4.23333;
  if(/in\s*$/i.test(value)) return +value.replace(/\s*in\s*$/gi, '') * 25.4;
  if(/in\s*$/i.test(value)) return +value.replace(/\s*in\s*$/gi, '') * 25.4;
  if(/mil\s*$/i.test(value)) return +value.replace(/\s*mil\s*$/gi, '') * 0.0254;
  if(/cm\s*$/i.test(value)) return +value.replace(/\s*cm\s*$/gi, '') * 10;
  if(/px\s*$/i.test(value) || !isNaN(+value)) return +(value + '').replace(/\s*px\s*$/gi, '') / 3.77952755953127906261;
}

function unitConvToPx(value) {
  return MillimeterToPixel(unitConvToMM(value));
}

function MillimeterToPixel(value) {
  return value * 3.77952755953127906261;
}

function getViewBox(svgElem = svg) {
  if(svgElem.hasAttribute('viewBox')) {
    let viewBox = svgElem.getAttribute('viewBox');
    return BBox.fromString(viewBox + '');
  }

  return new BBox(0, 0, ...getWidthHeight(svgElem));
}

function getWidthHeight(svgElem = svg) {
  if(svgElem.hasAttribute('width') && svgElem.hasAttribute('height')) {
    let width = svgElem.getAttribute('width');
    let height = svgElem.getAttribute('height');

    return new Size(unitConvToPx(width), unitConvToPx(height));
  }

  return new Size(...getViewBox(svgElem).size);
}

function main(...args) {
  globalThis.console = new Console({ depth: 2, customInspect: true, compact: false });

  let opts;
  let params = getOpt(
    (opts = {
      help: [false, (_x, _y, opts) => showHelp(opts), 'h'],
      debug: [false, () => ++debug, 'x'],
      size: [true, null, 's'],
      padding: [true, null, 'p'],
      '@': 'files'
    }),
    args
  );
  let files = params['@'];
  let parser = (globalThis.parser = new Parser());
  let serializer = (globalThis.serializer = new Serializer());

  if(files.length == 0) showHelp(opts, 1);

  for(let file of files) {
    console.log('Processing:', file);

    let xml = (globalThis.document = parser.parseFromFile((globalThis.file = file), 'utf-8'));

    let svg = (globalThis.svg = xml.querySelector('svg'));

    let vbOld = getViewBox(svg) ?? getWidthHeight(svg);

    let vbNew = vbOld.inset(0);

    if(params.padding) {
      let padding = [...NumericArgs(params.padding)];
      vbNew = vbNew.outset(...padding);
    }

    console.log('viewBox', { vbOld, vbNew });
    svg.setAttribute('viewBox', vbNew.toSVG());

    const { width, height } = vbNew;
    console.log('viewBox', { width, height });
    console.log('viewBox', width.toString);

    svg.setAttribute('width', width);
    svg.setAttribute('height', height);

    kill(process.pid, SIGUSR1);
  }
}

main(...scriptArgs.slice(1));
