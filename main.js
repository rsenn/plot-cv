import { AddLayer, BoardToGerber, ClearCache, FetchURL, GcodeToPolylines, GerberToGcode, GetLayer, ListProjects, default as commands } from './commands.js';
import { Button, ButtonGroup, Chooser, ColorWheel, Conditional, CrossHair, DisplayList, DropDown, DynamicLabel, Fence, File, FileList, FloatingPanel, Panel, Ruler, Slider, Toggle } from './components.js';
import { AlignAll, GetElements, GetInstances, GetPositions, scientific, UpdateMeasures } from './eagle-commands.js';
import Alea from './lib/alea.js';
import debounce from './lib/async/debounce.js';
import asyncHelpers from './lib/async/helpers.js';
import { makeLocalStorage } from './lib/autoStore.js';
import { classNames } from './lib/classNames.js';
import { ClipperLib } from './lib/clipper-lib.js';
import Shape from './lib/clipper.js';
import { HSLA, ImmutableHSLA, ImmutableRGBA, isHSLA, isRGBA, RGBA } from './lib/color.js';
import { BinaryTree } from './lib/container/binaryTree.js';
import * as deep from './lib/deep.js';
import { devtools } from './lib/devtools.js';
import * as dom from './lib/dom-old.js';
import { Cache } from './lib/dom/cache.js';
import { CacheStorage } from './lib/dom/cacheStorage.js';
import { eventIterator, iterator } from './lib/dom/iterator.js';
import keysim from './lib/dom/keysim.js';
import { h, render, useState, Fragment, Portal, ReactComponent } from './lib/dom/preactComponent.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { BoardRenderer, EagleSVGRenderer, LibraryRenderer, Renderer, SchematicRenderer } from './lib/eagle.js';
import { DereferenceError, EagleParser,EagleFactory,EagleDocument, EagleElement, EagleProject, Node as EagleNode} from './lib/eagle-dom.js';
import { Instance } from './lib/eagle/components/instance.js';
import { SchematicSymbol } from './lib/eagle/components/symbol.js';
import { EventIterator } from './lib/iterator/event.js';
import { Slot, SlotProvider } from './slots.js';
import { Wire } from './lib/eagle/components/wire.js';
import { Alignment, AlignmentAngle, AlignmentAttrs, CalculateArcRadius, CartesianToPolar, ClampAngle, DEBUG, DEG2RAD, EagleAlignments, ElementToClass, EscapeClassName, HORIZONTAL, HORIZONTAL_VERTICAL, InvertY, LayerAttributes, LayerToClass, LinesToPath, log, MakeCoordTransformer, MakeRotation, PinSizes, PolarToCartesian, RAD2DEG, RenderArc, RotateTransformation, setDebug, SVGAlignments, UnescapeClassName, useAttributes, useTrkl, VERTICAL } from './lib/eagle/renderUtils.js';
import * as ecmascript from './lib/ecmascript.js';
import { BG, digit2color, GetColorBands, GetExponent, GetFactor, GetMantissa, GetMultipliers, NumberToValue, ValueToNumber } from './lib/eda/colorCoding.js';
import { FetchCached, NormalizeResponse, ResponseData } from './lib/fetch.js';
import { fnmatch, PATH_FNM_MULTI } from './lib/fnmatch.js';
import { Functional } from './lib/functional.js';
import { GCodeLineStream, GcodeObject, GcodeParser, gcodetogeometry, gcodeToObject, Interpreter, InterpretGcode, noop, objectToGcode, parseFile, parseFileSync, parseGcode, parseStream, parseString, parseStringSync } from './lib/gcode.js';
import { Arc, BBox, Circle, isBBox, LineList, Polygon, default as geom } from './lib/geom.js';
import { MatrixTransformation, Rotation, Scaling, Transformation, TransformationList, Translation } from './lib/geom/transformation.js';
import Voronoi from './lib/geom/voronoi.js';
import GerberParser from './lib/gerber/parser.js';
import { useDimensions } from './lib/hooks/useDimensions.js';
import { useDoubleClick } from './lib/hooks/useDoubleClick.js';
import { Iterator } from './lib/iterator.js';
import { Pointer as ImmutablePath } from './lib/pointer.js';
import { arrayDiff, /*ImmutablePath, */ MutablePath, objectDiff, toXML } from './lib/json.js';
import { lazyInitializer } from './lib/lazyInitializer.js';
import LogJS from './lib/log.js';
import { BaseCache, brcache, CachedFetch, lscache } from './lib/lscache.js';
import { camelize, clamp, className, define, entries, escape, filter, functionName, getMethods, getset, isArray, isObject, isoDate, keys, mapAdapter, mapFunction, memoize, once, properties, range, repeater, roundTo, split, toString, tryCatch, tryFunction, values, weakDefine } from './lib/misc.js';
import { WebSocketClient } from './lib/net/websocket-async.js';
import objectInspect from './lib/objectInspect.js';
import * as path from './lib/path.js';
import renderToString from './lib/preact-render-to-string.js';
import * as React from './lib/preact.mjs';
import { useResult } from './lib/repeater/react-hooks.js';
import { Repeater } from './lib/repeater/repeater.js';
import * as Timers from './lib/repeater/timers.js';
import { FixedMedium } from './lib/svg/fixedMedium.js';
import { SvgPath } from './lib/svg/path.js';
import { normalizePath, reverseNormalizedPath, reversePath, reverseSubPath } from './lib/svg/pathReverse.js';
import tlite from './lib/tlite.js';
import { TouchListener } from './lib/touchHandler.js';
import { trkl } from './lib/trkl.js';
import tXml from './lib/tXml.js';
import { XmlAttr, XmlObject } from './lib/xml-old.js';
import { ImmutableXPath } from './lib/xml/xpath.js';
import { Message } from './message.js';
import * as rpc2 from './quickjs/qjs-net/js/rpc.js';
import serial from './serial.js';
import { PipeTo, AsyncRead, AsyncWrite, DebugTransformStream, TextEncodeTransformer, TextEncoderStream, TextDecodeTransformer, TextDecoderStream, TransformStreamSink, TransformStreamSource, TransformStreamDefaultController, TransformStream, ArrayWriter, readStream, WriteToRepeater, LogSink, RepeaterSink, StringReader, LineReader, ChunkReader, ByteReader, PipeToRepeater, WritableStream, ReadFromIterator } from './lib/stream.js?ts=<?TS?>';
//const React = {Component, Fragment, create: h, html, render, useLayoutEffect, useRef, useState };

const {
  Align,
  AlignToString,
  Anchor,
  CSS,
  Event,
  CSSTransformSetters,
  Element,
  ElementPosProps,
  ElementRectProps,
  ElementRectProxy,
  ElementSizeProps,
  ElementTransformation,
  ElementWHProps,
  ElementXYProps,
  isElement,
  isLine,
  isMatrix,
  isPoint,
  isRect,
  isSize,
  Line,
  Matrix,
  Point,
  PointList,
  Polyline,
  Rect,
  Select,
  Size,
  SVG,
  Transition,
  TransitionList,
  TRBL,
  Tree
} = { ...dom, ...geom };
//import rpc from './quickjs/qjs-net/js/rpc.js';

const elementDefaultAttributes = {
  stroke: 'red',
  fill: 'none',
  'stroke-linecap': 'round',
  'stroke-linejoin': 'round',
  'stroke-width': 0.1
};

const is = {
  on: arg => ['yes', 'on', 1, true].indexOf(arg) != -1
};

/* prettier-ignore */
//extend(window, { React, ReactComponent, WebSocketClient, html }, { dom, keysim }, geom, { Iterator, Functional }, { EagleNodeList, EagleDocument, EagleElement }, { toXML, XmlObject, XmlAttr }, { CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token, ReactComponent, ClipperLib, Shape, isRGBA, RGBA, ImmutableRGBA, isHSLA, HSLA, ImmutableHSLA, Alea, Message }, { Chooser, useState, useLayoutEffect, useRef, Polygon, Circle } );
const Timer = { ...Timers, once: dom.Timer };

const putError = error => console.log('ERROR: ' + error.message + '\n' + error.stack);

let currentProj = trkl.property(window, 'project');
let layerList = trkl.property(window, 'layers', { value: [] });
let gcode = trkl(null);
let wantAuthorization = trkl(null);

let open = trkl();
let showSearch = trkl(true);
let dump = trkl({});
let cache = new lscache();

let projectName = 'Headphone-Amplifier-ClassAB-alt3';
let palette = null;
let svgElement;
let brdXml, schXml, brdDom, schDom;
let board, schematic;
let loadedProjects = [];
let container;

let projectFiles = [];
let activeFile;
let transform = trkl(new TransformationList());
let sizeListener = trkl({});
let aspectListener = trkl(1);
const documentTitle = trkl('');
const documentSize = trkl('');
const loading = trkl(false);
const filePanel = trkl(false);

const SaveConfig = debounce(() => {
  let obj = store.toObject();

  return fetch('config', {
    method: 'POST',
    headers: { 'content-type': 'application/octet-stream' },
    body: JSON.stringify(obj)
  }).then(res => res.json());
}, 5 * 1000);

const LoadConfig = once(() =>
  fetch('config')
    .then(ResponseData)
    .then(r => (console.log('config:', r), r))
    //    .then(r => r.json())
    .then(r => ({
      ...r,
      entries() {
        return Object.entries(r);
      }
    }))
);

let store = (window.store = makeLocalStorage());

let projects = trkl([]);
let socket = trkl();
let config = {
  listURL: trkl(store.get('url') || null),
  searchFilter: trkl(store.get('filter') || '*'),
  zoomLog: trkl(store.get('zoom') || null),
  logSize: trkl(store.get('console') || null),
  debugFlag: trkl(store.get('debug') || false),
  credentials: trkl(store.get('auth') || {}),
  showGrid: trkl(store.get('grid') || true),
  sortOrder: trkl(store.get('sortOrder') || -1),
  sortKey: trkl(store.get('sortKey') || 'name'),
  currentProject: trkl(store.get('currentProject') || null)
};

const GetProject = arg => {
  let ret = typeof arg == 'number' ? projects()[arg] : typeof arg == 'string' ? projects().find(p => p.name == arg) : arg;
  if(typeof ret == 'string') ret = { name: ret };
  return ret;
};
let elementChildren = null;
let elementGeometries = null;
let zoomValue = getset(
  () => ZoomFactor(config.zoomLog()),
  value => config.zoomLog(ZoomLog(value))
);
//
/*let zoomValue= getset(
  () => ZoomFactor(config.zoomLog()),
  v => config.zoomLog(ZoomLog(v))
  );*/
//let zoomValue = deriveGetSet(config.zoomLog, ZoomFactor, ZoomLog);

config.zoomLog.subscribe(AdjustZoom);

const add = (arr, ...items) => [...(arr ? arr : []), ...items];

const useSlot = (arr, i) => [() => arr[i], v => (arr[i] = v)];
const trklGetSet = (get, set) => value => value !== undefined ? set(value) : get();
//const useTrkl = trkl => [() => trkl(), value => trkl(value)];

/*const MouseEvents = h => ({
  onMouseDown: h,
  onMouseOut: h,
  onMouseUp: h
});*/

tlite(() => ({
  grav: 'nw',
  attrib: ['data-tlite', 'data-tooltip', 'title', 'data-filename']
}));

const utf8Decoder = new TextDecoder('utf-8');
let svgOwner, parent;

const svgFactory = memoize((parent, delegate) => {
  parent = parent ? Element.find(parent) : project.svgElement.parentElement;
  const factory = SVG.factory({
    ...delegate,
    append_to(elem, p) {
      if(delegate.append_to) delegate.append_to(elem, p || parent);
    }
  });
  let rect = calcViewBox(parent);
  let zIndex = maxZIndex() + 1;
  const svg = [
    'svg',
    {
      viewBox: rect.toString(),
      style: `position: absolute; left: 0; top: 0; z-index: ${zIndex}; stroke: #000, fill: none;`
    },
    [
      ['defs'],
      [
        'g',
        {
          transform: ` scale(1,-1) translate(0,1.27) translate(0,${-rect.y2}) `
        },
        [['rect', { ...rect.toObject(), fill: 'hsla(0,0%,50%,0.3333)' }]]
      ]
    ]
  ];
  const element = (svgOwner = factory(...svg));
  factory.root = parent = element.lastElementChild;
  return factory;
});

function DrawSVG(...args) {
  const factory = svgFactory('body', {
    append_to(elem, p) {
      //console.log('append_to', this, { elem, p });
      (p || this.root).appendChild(elem);
      adjustViewBox(elem);
    }
  });
  let e;
  /*let parent = project.svg.parentElement.lastElementChild;
    const append = e => parent.appendChild(e);*/
  let c = RGBA.random();
  let [tag, attrs, children] = args;
  if(typeof tag == 'string') {
    //console.log('draw(', ...args, ')');
    e = factory(tag, { stroke: c.hex(), 'stroke-width': 0.1, ...attrs }, children);
  } else if(Array.isArray(args[0])) {
    let items = args.shift();
    // document.querySelector('#main > div > div > div > svg:nth-child(2) > g');
    //   setViewBox(factory.root.ownerSVGElement||factory.root, BBox.from(items));
    for(let item of items) {
      let line;
      if(isLine(item)) line = new Line(item);
      if(line) {
        e = factory('line', {
          ...line.toObject(),
          stroke: c.hex(),
          'stroke-width': 0.1
        });
      }
    }

    return;
  }

  function adjustViewBox(e) {
    let ownerSVG;

    if(!(ownerSVG = e.ownerSVGElement)) return;
    let rect = new Rect(ownerSVG.getBBox());
    //console.log('ownerSVG:', ownerSVG, 'rect:', rect);
    ownerSVG.setAttribute('viewBox', rect + '');
    if(!ownerSVG.style.maxHeight) Element.setCSS(ownerSVG, { maxWidth: '100vw', maxHeight: '100vh' });
  }

  return e || factory;
}

function calcViewBox(box) {
  box = box || (project && project.doc && BBox.from(project.doc.getMeasures(true) || project.doc.getBounds(0)));
  box = box || Element.rect('.aspect-ratio-box-inside');
  const { width, height, x, y } = box;
  let { x1, y1, x2, y2 } = new Rect(x, y, width, height);
  const rect = new BBox(x1, y1 - y2, x2 - x1, y2);
  console.log('calcViewBox', rect);
  return rect;
}

function setViewBox(svgOwner, box) {
  svgOwner = svgOwner || [...Element.findAll('svg', Element.find('#main'))].reverse()[0];
  const rect = box; // instanceof BBox ? box : DrawSVG.calcViewBox(box);
  rect.y1 -= rect.y2;
  rect.x2 -= rect.x1;
  //console.log('setViewBox', { svgOwner, rect, box });
  svgOwner.setAttribute('viewBox', rect.toString());
  svgOwner.lastElementChild.setAttribute('transform', `scale(1,-1)  translate(0,${-rect.height})`);
  Element.attr(svgOwner.lastElementChild.firstElementChild, {
    ...rect.toRect()
  });
}

const ElementToXML = (e, predicate) => {
  if(globalThis.XMLSerializer) return new XMLSerializer().serializeToString(e);

  const x = Element.toObject(e, { predicate });

  for(let [value, path] of deep.iterate(x, (v, k) => k[k.length - 1] == 'd')) {
    deep.set(x, path, value.trim().replace(/\s+/g, ' '));
  }

  for(let [value, path] of deep.iterate(x, (v, k) => k[k.length - 1] == 'id' && v == 'rects')) deep.unset(x, path.slice(0, -1));
  for(let [value, path] of deep.iterate(x, (v, k) => /(^data-|^class$)/.test(k[k.length - 1]))) deep.unset(x, path);
  //console.log('x:', x);
  return Element.toString(x, { newline: '\n' });
};

const filesystem = {
  async readFile(filename) {
    return await FetchURL(`static/${filename}`);
  },
  async writeFile(filename, data, overwrite = true) {
    return await fetch('save', {
      method: 'post',
      headers: {
        'Content-Type': 'application/octet-stream',
        'Content-Disposition': `attachment; filename="${filename}"`
      },
      body: data + ''
    });
  },
  async exists(filename) {},
  async realpath(filename) {}
};

async function LoadFile(file) {
  let { url, name } = typeof file == 'string' ? { url: file, name: file.replace(/.*\//g, '') } : GetProject(file);

  // url = /:\/\//.test(url) ? url : /^(tmp|data|static)\//.test(url) ? '/' + url : `/data/${name}`;
  url = `file?action=load&file=${file}`; // /:\/\//.test(url) ? url : /^(tmp|data|static)\//.test(url) ? '/' + url : `/data/${name}`;
  console.log(`LoadFile ${url}`);

  let response = await FetchURL(url);
  let xml = await response.text();

 console.log(`LoadFile ${name}`, { xml });

  let doc = new EagleParser(new EagleFactory()).parseFromString(xml, name);
  //let doc = new EagleDocument(xml, null, name, null, filesystem);

  //console.log(`LoadFile ${name}`, { doc, xml });
  if(/\.brd$/.test(name)) window.board = doc;
  if(/\.sch$/.test(name)) window.schematic = doc;
  if(/\.lbr$/.test(name)) window.libraries = add(window.libraries, doc);
  return doc;
}

async function SaveFile(filename, data, contentType) {
  if(!data.endsWith('\n')) data += '\n';
  let response = await fetch('save', {
    method: 'post',
    headers: {
      'Content-Type': contentType || 'application/xml',
      'Content-Disposition': `attachment; filename="${filename}"`
    },
    body: data
  });
  let { status, statusText } = response;
  //console.log('SaveFile', {filename,data, response});
  globalThis.saveResponse = response;
  let body = await response
    .text()
    .then(s => JSON.parse(s))
    .catch(() => null);
  let headers = [...(await response.headers.entries())];
  //console.log('SaveFile', { body, headers });
  const result = { status, statusText };
  const request = new Request(`/tmp/${filename}`);
  response = new Response(data, { status, statusText });
  let cache = await caches.open('fetch');
  let r = await cache.put(request, response);
  LogJS.info(`${filename} saved & cached (${body.size} bytes)`);
  return result;
}

async function SaveSVG(filename, layers = [1, 16, 20, 21, 22, 23, 25, 27, 47, 48, 51]) {
  const { doc } = project;
  const { basename, typeName } = doc;
  if(!filename) filename = `${doc.basename}.${doc.typeName}.svg`;
  //console.log('SaveSVG(', filename, ', ', layers, ')');
  let predicate = element => {
    /*  if(element.hasAttribute) {if(!element.hasAttribute('data-layer')) return true;
    const layer = element.getAttribute('data-layer');
    let [number, name] = layer.split(/\ /);
    if(number !== undefined && name !== undefined) return layers.indexOf(+number) != -1 || layers.indexOf(name) != -1;
  }*/
    return true;
  };
  let data = ElementToXML(project.svgElement, predicate);
  //let data = ElementToXML(project.svgElement);
  return await SaveFile(filename.replace(/\.svg$/i, '.svg'), data);
}

async function LoadSVG(filename) {
  let data = await FetchURL(filename).then(ResponseData);
  let xml = tXml(data.replace(/<\?xml[^>]*>/, ''));
  let element = Element.create('div', { style: { display: 'inline-block' } }, 'body');
  let component = ReactComponent.fromObject(xml[0]);
  React.render(component, element);
  return element.firstElementChild;
}

function LoadImage(filename) {
  let element = Element.create('img', { src: filename }, 'body');

  //Element.setCSS(element, { width: `50%`, height: 'auto', 'z-index': 100000000, position: 'fixed', bottom: '4em', right: '4em', 'box-shadow': '0 0 4px 2px #000' });
  //Element.setCSS(element, { transform: `scale(2)` });

  Element.setCSS(element, { position: 'fixed', bottom: '4em', right: '4em', 'z-index': 100000000 });

  return element;
}

const RenderComponent = (() => {
  let id = 1;
  return function RenderComponent(component) {
    let element = Element.create('div', { id: `react-${id++}`, style: { display: 'inline-block' } }, 'body');
    React.render(component, element);
    return element;
  };
})();

const ModifyColors = fn => e => {
  const { type, buttons } = e;
  if(type.endsWith('down')) {
    if(!window.c) window.c = SVG.allColors(project.svgElement);
    let { c } = window;
    c.dump();
    fn(c);
  }
};

const FindLayer = name_or_id => {
  for(let id of (name_or_id + '').split(/\s+/g).map(n => (isNaN(n) ? n : +n))) {
    const layer = layers.find(l => l.i == id || l.name == id);
    if(layer) return layer;
  }
};

const GerberLayers = {
  GTL: 'Top (copper) Layer',
  GBL: 'Bottom (copper) Layer ',
  GTO: 'Top Overlay',
  GBO: 'Bottom Overlay ',
  GTP: 'Top Paste Mask ',
  GBP: 'Bottom Paste Mask ',
  GTS: 'Top Solder Mask ',
  GBS: 'Bottom Solder Mask ',
  GKO: 'Keep-Out Layer ',
  GML: 'Mill layer',
  gpi: 'Photoplotter info file',
  TXT: 'Drill file'
};

let svgDocFactory = memoize((id = '#geom') =>
  SVG.factory(Element.find(id)).initialize('svg', {
    width: window.innerWidth,
    height: window.innerHeight /*, style: "pointer-events: none;"*/
  })
);
let svgGroupFactory = memoize((props = {}) =>
  svgDocFactory().setRoot('g', {
    stroke: '#f00',
    'stroke-width': 3,
    fill: 'none',
    ...props
  })
);

const maxZIndex = () =>
  Math.max(
    ...Element.findAll('*')
      .map(e => Element.getCSS(e, 'z-index'))
      .filter(z => !/(auto)/.test(z))
      .map(z => +z)
  );

const groupProps = memoize(() => {
  let transform = `translate(300,-50)`;
  const prng = new Alea(234234800);
  const randomColor = () => HSLA.random([240, -120], [100, 100], [30, 75], [1, 1], prng).hex();
  return [
    { stroke: randomColor(), transform },
    { stroke: randomColor(), transform }
  ];
});

function DrawArc(start, end, angle) {
  let [r, g, b] = groupProps().map(props => svgGroupFactory(props).clear());

  let [p1, p2] = [start, end].map(p => new Point(p));

  let line = new Line(p1, p2);
  let radius = Arc.radius(angle, p1, p2);
  let length = Arc.length(angle, p1, p2);
  let center = Arc.center(p1.x, p1.y, p2.x, p2.y, radius);

  let rect = new Rect({ x: 50, y: 350, width: 300, height: 300 });
  let middle = line.pointAt(0.5);
  let points = [
    ...line,
    middle,
    center
    //...[0.25, 0.75].map(a => line.pointAt(a))
  ];
  let degA = (angle * 180) / Math.PI;
  let a_b = (360 - degA) / 2;
  let angles = [90, -a_b, a_b];
  //console.log('angles:', angles);
  let matrices = angles.map(a => new Rotation(a).toMatrix());

  let slopes = matrices.map(m => new Point(m.transformPoint(line.slope)).normal());

  slopes[1].mul(-1);
  //console.log('slopes:', slopes);

  // r('rect', rect.toObject());
  let rot = new TransformationList([new Translation(middle.x, middle.y), new Rotation(90), new Translation(-middle.x, -middle.y)]);
  //console.log('rot:', rot + '');
  let pivots = [middle, line.a, line.b];
  let colors = ['#EB1F00', '#F0CC11', '#34DB05', '#0078F0', '#8D1AE6'];
  let compound = ['#2E17B3', '#554D85', '#3578E6', '#E9B470', '#B35917'];
  let palette = ['ff595e', 'ffca3a', '8ac926', '1982c4', '6a4c93'].map(hex => `#${hex}`);
  let rainbow = ['#E64535', '#E6AA4C', '#D0E634', '#1EE67B', '#3394E8'];
  let lines2 = [new Line(p1, center), new Line(p2, center)];

  let norms = [p1, p2].map(p => p.diff(center)).map(p => p.normal());
  angles = norms.map(p => mod(p.toAngle(true), 360));
  //console.log('angles:', angles);
  //console.log('angles abs:', Math.abs(angles[0] - angles[1]));
  //console.log('angle :', angle);
  //console.log('norms:', norms);
  //console.log('center:', center);

  let range = norms.map(({ x, y }) => new Point(x * radius, y * radius).sum(center));
  let deg = (angle * 180) / Math.PI;
  let approx = range(0, deg, 10).map(a => Point.fromAngle((a * Math.PI) / 180 - angle, radius - 30));

  //console.log('range:', range);
  //console.log('approx:', approx);
  //range = range.map(v => v. sum(center));

  points.forEach(({ x, y }, i) =>
    b('circle', {
      cx: x,
      cy: y,
      r: 10,
      fill: rainbow[i],
      'stroke-width': 2,
      stroke: 'black'
    })
  );

  let svg = g('line', { ...line.toObject() }).ownerSVGElement;
  //console.log('svg:', svg);
  svg.addEventListener('click', e => {
    //console.log('clicked:', e.target);
    svg.style.setProperty('display', 'none');
  });
  lines2.forEach((l, i) => g('line', { ...l.toObject(), stroke: compound[i] }));

  range.forEach((p, i) => b('circle', { cx: p.x, cy: p.y, r: 15, stroke: '#0ff' }));
  approx.forEach((p, i) =>
    b('circle', {
      cx: p.x,
      cy: p.y,
      r: 9,
      fill: '#ff0',
      stroke: '#000',
      'stroke-width': 1.5,
      transform: `translate(${center})`
    })
  );

  b('polyline', {
    points: new PointList(approx),
    fill: 'none',
    stroke: '#000',
    'stroke-width': 1.5,
    transform: `translate(${center})`
  });

  r('path', { d: `M ${p1} A ${radius} ${radius} 0 0 1 ${p2}` });
  r('path', {
    d: `M ${p1} A ${radius} ${radius} 0 1 0 ${p2}`,
    stroke: '#00dd0080'
  });

  //g('line', { ...line2.toObject(), stroke: '#0f0' });
}

const DrawBinaryTree = (tree, draw = DrawSVG()) => {
  let a = [];
  const add = (level, item) => (a[level] ? a[level].push(item) : (a[level] = [item]));

  RecurseBinaryNode(tree.root);

  let svg = Element.find('body > svg');
  [...svg.children].forEach(Element.remove);

  function RecurseBinaryNode(node, parent = null, depth = 0) {
    let item = { node, parent, y: depth * 20 };
    add(depth, item);

    if(node.left) RecurseBinaryNode(node.left, item, depth + 1);
    if(node.right) RecurseBinaryNode(node.right, item, depth + 1);
  }

  //console.log('a:', a);
  a.forEach((nodes, i, level) => {
    let fx = j => j * 20 - ((nodes.length - 1) * 20) / 2;
    nodes.map((node, j) => (node.x = fx(j)));
  });
  a.flat().forEach(item => (DrawNode(item.x, item.y, item.node), item.parent && DrawEdge(item, item.parent)));

  function DrawNode(x, y, node) {
    draw('circle', {
      cx: x,
      cy: y,
      r: 5,
      stroke: '#000',
      fill: '#fff',
      'stroke-width': 0.4
    });
    draw(
      'text',
      {
        x: x + 0.2,
        y: y + 0.5,
        ...AlignmentAttrs({ x: 0, y: 0 }),
        'font-size': '6px'
      },
      node.value + ''
    );
  }
  function DrawEdge(item, parent) {
    let points = [item, parent].map(Point);
    let line = new Line(...points);
    let slope = line.slope.normal().mul(5);

    points[0].add(slope);
    points[1].sub(slope);

    //console.log('points:', points);
    draw('line', {
      ...Line(...points).round(0.001),
      stroke: '#000',
      'stroke-width': 0.4
    });
  }
};
DrawBinaryTree.bt = new BinaryTree(
  new BinaryTree.Node('A', new BinaryTree.Node('B', new BinaryTree.Node('D')), new BinaryTree.Node('C', new BinaryTree.Node('E', null, new BinaryTree.Node('G')), new BinaryTree.Node('F')))
);
DrawBinaryTree.bt = new BinaryTree(
  new BinaryTree.Node(
    'V',
    new BinaryTree.Node('H', new BinaryTree.Node(2), new BinaryTree.Node(1)),
    new BinaryTree.Node(
      'H',
      new BinaryTree.Node(
        'H',
        new BinaryTree.Node('V', new BinaryTree.Node(6), new BinaryTree.Node(7)),
        new BinaryTree.Node('V', new BinaryTree.Node(4), new BinaryTree.Node(5)),
        new BinaryTree.Node('V')
      ),
      new BinaryTree.Node(3)
    )
  )
);

function GetPaths(query, parent = project.svgElement) {
  return Element.findAll(query, parent).reduce((a, e) => a.concat(e.tagName != 'path' ? Element.findAll('path', e) : [e]), []);
}

function PathToPolylines(path, step = 0.01) {
  let poly,
    polys = [];
  [...SVG.pathIterator(path, { step })].forEach(p => {
    if(p.move || !poly) polys.push((poly = []));

    poly.push(new Point(p).round(0.001));
  });

  return polys
    .filter(poly => poly.length > 1)
    .map(poly => {
      let transforms = new TransformationList(
        Element.walkUp(path, (p, d, set, stop) => (p.parentElement.tagName == 'svg' ? stop() : p.hasAttribute('transform') && set(p.getAttribute('transform')))).reverse()
      ).collapse();
      //console.log('transforms', transforms);
      return new Polyline(poly).transform(transforms);
    });
}

function PathToPolyline(path, step = 0.01) {
  let poly = [...SVG.pathIterator(path, { step })];

  let transforms = new TransformationList(
    Element.walkUp(path, (p, d, set, stop) => (p.parentElement.tagName == 'svg' ? stop() : p.hasAttribute('transform') && set(p.getAttribute('transform')))).reverse()
  ).collapse();
  //console.log('transforms', transforms);
  return new Polyline(poly).transform(transforms);
}

function PathsToPolylines(paths, step = 0.01) {
  if(typeof paths == 'string') paths = GetPaths(...paths.split(' '));
  if(!Array.isArray(paths)) paths = [paths];
  return new Map(paths.map(path => [path, PathToPolyline(path, step)]));
}

function OutsetPath(path, offset, miterLimit = 2, arcTolerance = 0.01) {
  let co = new ClipperLib.ClipperOffset(miterLimit, arcTolerance);
  let output = (window.output = new ClipperLib.Paths());
  co.AddPath(
    path.closed ? path.slice(0, -1) : path,
    ClipperLib.JoinType[path.closed ? 'jtRound' : 'jtSquare'],
    ClipperLib.EndType[path.closed ? 'etClosedLine' /*'etClosedPolygon' */ : 'etOpenSquare' || 'etOpenRound']
  );
  co.Execute(output, offset);
  //console.log('output:', output);
  output.toPolylines = function() {
    return this.map(p => new Polyline(p.map(({ X, Y }) => new Point(X, Y))).close());
  };
  return output;
}

//
//  OutsetPaths(PathsToPolylines('path.top .R8'), 0.1);
//
function OutsetPaths(paths, offset, miterLimit = 2, arcTolerance = 0.25) {
  let ret;
  // if(typeof paths == 'string') paths = PathsToPolylines(paths);
  if(typeof paths.values == 'function') paths = [...paths.values()];

  //console.log('OutsetPaths:', { paths, ret });

  ret = paths.map(path => OutsetPath(path, offset, miterLimit, arcTolerance));

  ret = ret.slice(1).reduce((a, p) => ClipPath(a, p), ret[0]);

  let [outer, clip] = ret.toPolylines();

  let f = project.makeFactory();
  let d = [outer.toPath(), clip.toPath()].join(' ');

  //console.log('d:', d);

  /* outer.toSVG(f, { 'stroke-width': 0.0508, stroke: 'black', fill: 'none' });
  clip.toSVG(f, { 'stroke-width': 0.0508, stroke: 'black', fill: 'none' });
*/
  f('path', { d, 'stroke-width': 0.0508, stroke: 'none', fill: 'red' });
  return ret;
}

function ClipPath(path, clip, mode = ClipperLib.ClipType.ctUnion) {
  let cl = new ClipperLib.Clipper();
  let output = new ClipperLib.Paths();
  const add = (p, clip = false) => (Array.isArray(p[0]) ? cl.AddPaths : cl.AddPath).call(cl, p, clip ? ClipperLib.PolyType.ptClip : ClipperLib.PolyType.ptSubject, true);

  add(path, false);
  add(clip, true);

  cl.Execute(mode, output);
  output.toPolylines = function() {
    return this.map(p => new Polyline(p.map(({ X, Y }) => new Point(X, Y))).close());
  };
  return output;
}

function saveItemStates(itemList, get = item => is.on(item.visible())) {
  return itemList.map(item => [item, get(item)]);
}

function restoreItemStates(itemStates, /* prettier-ignore */ set = (item, value) => item.visible(value ? 'yes' : 'no')) {
  for(let [item, state] of itemStates) set(item, state);
}

function EagleMaps(project) {
  let transformPath = p => p.replace(/\s*➟\s*/g, '/').replace(/\/([0-9]+)/g, '/[$1]');
  let dom2path = [...Element.findAll('*[data-path]', project.object)].map(e => [e, new ImmutableXPath(transformPath(e.getAttribute('data-path')))]);
  //console.debug('dom2path:', dom2path);
  dom2path = mapFunction(new WeakMap(dom2path));

  let dom2eagle = node => {
    let p;
    if((p = dom2path(node))) return project.doc.lookup(p);
  };
  let eagle2dom = elem => Element.find(`[data-type=${elem.tagName}][data-name=${elem.name}]`, project.element);
  let path2dom = p => Element.find(`[data-path='${CSS.escape(p)}']`, project.element);

  let mapElements = {
    /*    eagle: unique(eagle2dom.map(([e, d]) => e)),
    dom: unique(eagle2dom.map(([e, d]) => d), (a, b) => a.isSameNode(b))*/
  };
  let maps = { dom2path, dom2eagle, eagle2dom, path2dom };
  /*  maps.eagle2dom = mapFunction(new WeakMap(mapElements.eagle.map(eagle => [
        eagle,
        eagle2dom.filter(([e, d]) => e === eagle).map(([e, d]) => d)
      ])
    )
  );
  //console.debug('maps.eagle2dom:', maps.eagle2dom);*/
  //) maps.dom2eagle = mapFunction(new WeakMap(eagle2dom.map(([k, v]) => [v, k])));
  const [path2component, component2path] = project.renderer.maps.map(mapFunction);
  const { /*path2obj, obj2path, */ path2eagle, eagle2path /*, eagle2obj, obj2eagle */ } = project.doc.maps;
  const [component2eagle, eagle2component] = [
    mapAdapter((key, value) => (value === undefined ? path2eagle(component2path(key)) : undefined)),
    mapAdapter((key, value) => (value === undefined ? path2component(eagle2path(key) + '') : undefined))
  ];
  weakDefine(maps, {
    path2eagle,
    eagle2path,
    path2component,
    component2path
  });

  /* const [component2dom, dom2component] = [
    mapAdapter((key, value) =>
      value === undefined ? eagle2dom(component2eagle(key)) : undefined
    ),
    mapAdapter((key, value) =>
      value === undefined ? eagle2component(dom2eagle(key)) : undefined
    )
  ]; */
  Object.assign(maps, {
    /*dom2component,
 component2dom, */
    component2eagle,
    component2path,
    eagle2component,
    path2component
  });
  Object.assign(project, { maps });
  return maps;
}

function* PackageNames(doc = project.doc) {
  const tokenize = matchAll(/([A-Za-z]+|[0-9,]+|[^0-9A-Za-z]+)/g);
  let packages = doc.packages && doc.packages.length ? doc.packages : [...doc.getAll('package')];
  let names = packages
    .map(e => [e, e.getBounds()])
    .map(([e, b]) => [e, b.width, b.height, Math.max(b.width, b.height), b.height > b.width])
    .map(([e, w, h, m, v]) => [e, e.name, [...tokenize(e.name)], roundTo(w, 0.01), roundTo(h, 0.01), Math.floor(m), v ? 'V' : '']);

  for(let [element, name, matches, w, h, size, orientation] of names) {
    let tokens = matches.map(({ index, ...match }) => match[0] + '');
    let [index] = matches
      .reduce(
        ({ s, v, indexes }, match, i) => {
          if(Number.isFinite(s) && indexes.length) return { s, v, indexes };
          if(match[0] == '/' && !Number.isFinite(s)) s = i;
          if(match[0] == 'V') v = i;
          if(!isNaN(+(match[0] + ''))) indexes.push(i);
          return { s, v, indexes };
        },
        { indexes: [] }
      )
      .indexes.reverse();
    let sIndex = tokens.findIndex(([token, index]) => token == '/');
    let vhIndex = tokens.findIndex(([token, index]) => token == 'V' || token == 'H');
    let tokIndex = vhIndex != -1 ? vhIndex - 1 : sIndex + 1;

    if(tokens[vhIndex]) {
      while(tokIndex >= 0 && isNaN(tokens[tokIndex])) tokIndex--;
    }
    if(isNaN(+tokens[tokIndex])) tokIndex = index;

    let token = tokens[tokIndex];
    let vertical = tokens[vhIndex] == 'V';
    let number = typeof token == 'string' ? +token.replace(/[^-.0-9]/g, '') : NaN;
    //console.log('names:', { index, sIndex, vhIndex, token, vertical, number });

    /*  if(number != size || vertical != (orientation == 'V')) */

    if(token) {
      let fromTo = [`${token}${tokens[vhIndex] || ''}`, `${size}${vertical ? 'V' : ''}`];
      let newName = name.replace(...fromTo);

      yield Object.assign([name, newName, ...fromTo], {
        w,
        h,
        size,
        orientation
      });
    }
  }
}

//let projectIndex;

function NextDocument(n = 1) {
  let i;
  const { projects } = globalThis;
  if(typeof globalThis.projectIndex != 'number') globalThis.projectIndex = projects.indexOf(project);
  const cond = isObject(n) && n instanceof RegExp ? (idx, i) => !n.test(projects[idx]?.name) : (idx, i) => i < n;
  let start = projectIndex;
  for(i = 0; cond(++projectIndex, i); ++i) {
    //LogJS.verbose(`NextDocument skip ${i} [${projectIndex}] ${projects[projectIndex].name}`);

    projectIndex %= projects.length;
    if(projectIndex == start) break;
  }
  LogJS.verbose(`NextDocument skipped ${projectIndex - start}`);

  let next = projects[projectIndex];
  config.currentProject(next.name);

  //console.log('NextDocument', `[${projectIndex}]`, projects[projectIndex].name);
  return LoadDocument(next);
}

async function LoadDocument(project, parentElem) {
  console.log('LoadDocument', project.name);
  open(false);
  gcode(null);
  if(typeof project == 'string') project = GetProject(project);

  config.currentProject(project.name);


  project.doc = await LoadFile(project.name).catch(err => console.error(err));
  //project.doc = new EagleParser().parseFromString(project.data);

 console.log('project.doc', project.doc);

  currentProj(project);
  LogJS.info(`${project.name} loaded.`);
  const topPlace = 'tPlace';
  elementChildren = memoize(() => ElementChildren(topPlace, ent => Object.fromEntries(ent)));
  elementGeometries = memoize(() => ElementGeometries(topPlace, ent => Object.fromEntries(ent)));

  tryCatch(() => {
    documentTitle(project.doc.file.replace(/.*\//g, ''));
    let s = project.doc.type != 'lbr' && project.doc.dimensions;
    if(s) documentSize(s.round(0.01).toString({ unit: 'mm' }));
  });

  const { doc } = project;
  window.eagle = doc;
  window.project = project;
  Element.remove('#fence');
  let docElem = Element.find('#doc');
  docElem.innerHTML = '';
  define(
    window,
    properties(
      {
        renamePackages() {
          let names = [...PackageNames(doc)];
          let changes = names.filter(a => a[0] != a[1]);
          return names;
        }
      },
      { memoize: true, configurable: true }
    )
  );

  await RenderProject(project);

  return project;
}

async function RenderProject(project = globalThis.project) {
  const { doc } = project;

  project.renderer = new Renderer(doc, ReactComponent.append, config.debugFlag());

  config.showGrid = trkl(true);
  config.showGrid.subscribe(value => {
    let obj = { ...project.renderer.grid, visible: value };
    project.renderer.grid = obj;
  });

  let style = { width: '100%', height: '100%', position: 'relative' };

  let Component = project.renderer.render(doc, null, {});

  let usedLayers = [...doc.layers.list]; /*.filter(layer => layer.elements.size > 0)*/

  Timer.once(250).then(() =>
    layerList(
      usedLayers.map(layer => ({
        i: layer.number,
        name: layer.name,
        color: layer.getColor(),
        element: layer,
        visible: (() => {
          let fn;
          const handler = layer.handlers.visible;
          fn = function(v) {
            if(v !== undefined) handler(is.on(v) ? 'yes' : 'no');
            else return handler();
          };
          fn.subscribe = handler.subscribe;
          fn.unsubscribe = handler.unsubscribe;
          return fn;
        })()
      }))
    )
  );
  LogJS.info(`${project.name} rendered.`);
  window.component = project.component = Component;
  let element = Element.find('#main');

  if(project.renderer) {
    let r = project.renderer.rect || project.renderer.bounds;
    let size = (project.dimensions = project.renderer.size);
    let aspectRatio = 1;
    if(project.doc.type != 'lbr') {
      if(r) {
        aspectRatio = r.width / r.height;
        sizeListener(size);
      }
    } else {
      sizeListener({});
    }
    aspectListener(aspectRatio);

    Component = h(
      Fence,
      {
        style: {},
        sizeListener,
        aspectListener,
        listener: transform,
        'data-name': project.name
      },
      [Component]
    );
  }
  let svgElement;

  if(window.component) {
    React.render(Component, element);

    let object = ReactComponent.toObject(Component);
    project.object = object;
    let rendered = object.children[0];

    setTimeout(() => SaveSVG(), 500);

    project.maps = {
      ...project.doc.maps,
      ...EagleMaps(project)
    };

    project.rendered = rendered;

    window.project.element = element;
    window.project.svgElement = svgElement = Element.find('svg', element);

    project.grid = Element.find('g.grid', project.element);
    project.bbox = SVG.bbox(project.grid);
    project.aspectRatio = aspect;
  }

  let svg = Element.find('svg', '#main');

  if(svg) {
    project.makeGroup = function({ transform, ...props } = {}) {
      let e;
      if(props.id && (e = Element.find(`#${props.id}`))) return e;
      let groupElement = Element.find('g.elements', svg) || Element.find('g.instances', svg);
      transform = (groupElement ? groupElement.getAttribute('transform') : '') + (transform ? ' ' + transform : '');
      return (e = SVG.create('g', { ...props, transform }, svg));
    };
    project.makeFactory = memoize(id =>
      SVG.factory(() =>
        project.makeGroup({
          ...((id !== undefined && { id }) || {}),
          'stroke-width': 0.127 / 4
        })
      )
    );
    project.makeFactory();

    let center = SVG.bbox(svgElement).center.round();
    let defaultTransform = `translate(${center.x},${center.y}) scale(2.54,2.54)`;
    function xx() {
      let g = SVG.create('g', {});
      project.svgElement.appendChild(g);
      let ll = geometries.R4 && geometries.R4.lines.toSVG(ReactComponent.append, () => h('g', { ...elementDefaultAttributes, defaultTransform }));
      render(ll, g);
    }
    window.AddElement = (function (transform) {
      const root = project.svgElement;
      let list = [];

      return (tag, attr, children = []) => {
        let e = SVG.create(tag, { ...elementDefaultAttributes, transform, ...attr }, root);
        list.push(e);
        let d = trkl.property(e, 'd');
        d.subscribe(value => e.setAttribute('d', value));
        return e;
      };
    })(defaultTransform);
  }

  /*  tryCatch(async () => {
    let { name, data, doc, svg, bbox } = project;
    let bounds = doc.getBounds();
    let rect = bounds.toRect(Rect.prototype);
    let size = new Size(rect.size);
    // currentProj(project);
    size.mul(doc.type == 'brd' ? 2 : 1.5);
    let svgrect = SVG.bbox(project.svgElement);
    let measures = (doc.measures || doc.getBounds()).rect;
    //console.debug('measures:', measures);
    Element.attr(project.svgElement, {
      'data-filename': project.name,
      'data-aspect': project.aspectRatio
    });
    // let css = size.div(0.26458333333719).toCSS({ width: 'px', height: 'px' });
    //  window.size = project.doc.type == 'lbr' ? {} : css;
    AdjustZoom();
    project.status = SaveSVG();
  }, putError);*/

  if(svgElement) {
    let viewBox = new Rect(svgElement.getAttribute('viewBox').split(/\s+/g));

    let bgRects = [...svgElement.querySelector('#bg').children];

    const setRect = rect => {
      svgElement.setAttribute('viewBox', rect + '');
      ['x', 'y', 'width', 'height'].forEach(k => bgRects.forEach(elem => elem.setAttribute(k, rect[k])));
    };

    let r = (globalThis.viewBox = {
      /* prettier-ignore */ get x() { return viewBox.x; },
      /* prettier-ignore */ set x(value) { viewBox.x = value; setRect(viewBox); },
      /* prettier-ignore */ get y() { return viewBox.y; },
      /* prettier-ignore */ set y(value) { viewBox.y = value; setRect(viewBox); },
      /* prettier-ignore */ get x1() { return viewBox.x1; },
      /* prettier-ignore */ set x1(value) { viewBox.x1 = value; setRect(viewBox); },
      /* prettier-ignore */ get y1() { return viewBox.y1; },
      /* prettier-ignore */ set y1(value) { viewBox.y1 = value; setRect(viewBox); },
      /* prettier-ignore */ get x2() { return viewBox.x2; },
      /* prettier-ignore */ set x2(value) { viewBox.x2 = value; setRect(viewBox); },
      /* prettier-ignore */ get y2() { return viewBox.y2; },
      /* prettier-ignore */ set y2(value) { viewBox.y2 = value; setRect(viewBox); },
      /* prettier-ignore */ get width() { return viewBox.width; },
      /* prettier-ignore */ set width(value) { viewBox.width = value; setRect(viewBox); },
      /* prettier-ignore */ get height() { return viewBox.height; },
      /* prettier-ignore */ set height(value) { viewBox.height = value; setRect(viewBox); }
    });
  }
  {
    let { name, data, doc, svg, bbox } = project;

    let bounds = doc.getBounds();

    //console.log('bounds', bounds);
    let rect = bounds.toRect(Rect.prototype);

    //console.log('rect', rect);

    const { width, height } = rect;

    if(width && height) {
      let size = new Size(width, height);

      size.mul(doc.type == 'brd' ? 2 : 1.5);

      let svgrect = SVG.bbox(project.svgElement);
      let measures = (doc.measures || doc.getBounds()).rect;

      Element.attr(project.svgElement, {
        'data-filename': project.name,
        'data-aspect': project.aspectRatio
      });
      AdjustZoom();
      project.status = SaveSVG();
    }
  } /*,
    a => a,
    err => console.log('ERROR: ' + err.message + '\n' + err.stack)
  );*/

  return project;
}
async function ChooseDocument(project, i) {
  let r;
  if(i == undefined) i = project.i || projectFiles.indexOf(project);
  const box = Element.findAll('.file')[i];
  LogJS.info('ChooseDocument:', { project, i, box });
  LogJS.info(`${project.name} selected.`);

  if(!project.loaded) {
    loading(true);
    let data = await LoadDocument(project, box);
    project.loaded = true;
    //console.log('loaded:', project);
    loading(false);
  }

  return project.loaded;
}

/* gerber=await BoardToGerber(project.name); gc=await GerberToGcode('tmp/7seg-2.54.GBL'); geom=gcodetogeometry(gc.data);lines = geom.lines.map(({start,end}) => new Line(start,end)) */

const GenerateVoronoi = () => {
  //console.log('Loading document: ' + filename);
  let { doc } = project;
  //console.log('doc', doc);
  let points = new PointList();

  for(let element of doc.elements.list) {
    let { x, y, package: pkg } = element;
    //console.log('element:', element, { x, y });
    let origin = new Point(x, y);
    for(let item of pkg.children) {
      if(item.drill !== undefined) {
        let pos = new Point(+item.x, +item.y).add(origin);
        //console.log('pos:', pos);
        points.push(pos);
      }
    }
  }

  let bb = doc.getBounds();
  let rect = bb.toRect(Rect.prototype);
  //console.log('bb:', bb);
  //console.log('rect:', rect);
  rect.outset(1.27);
  window.tmprect = rect;
  let sites = points.map(p => p.toObject());
  let bbox = { xl: bb.x1, xr: bb.x2, yt: bb.y1, yb: bb.y2 };
  let voronoi = new Voronoi();
  //pass an object which exhibits xl, xr, yt, yb properties. The bounding
  //box will be used to connect unbound edges, and to close open cells
  let result = voronoi.compute(sites, bbox);
  //render, further analyze, etc.
  //console.log('result:', Object.keys(result).join(', '));
  let { site, cells, edges, vertices, execTime } = result;
  //console.log('cells:', cells);
  let holes = edges.filter(e => !e.rSite).map(({ lSite, rSite, ...edge }) => new Point(lSite));
  let rlines = edges.filter(e => e.rSite).map(({ lSite, rSite, ...edge }) => new Line(lSite, rSite));
  let vlines = edges.filter(e => e.va && e.vb).map(({ va, vb, ...edge }) => new Line(va, vb).round(0.127, 4));
  let points2 = vertices.map(v => new Point(v).round(0.127, 4));
  const add = (arr, ...items) => [...(Array.isArray(arr) ? arr : []), ...items];
  const factory = SVG.factory();
  const lines = [
    ...rlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#000', 'stroke-width': 0.01 }]),
    ...vlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#f00', 'stroke-width': 0.01 }])
  ];
  const circles = [
    ...holes.map(p => [
      'circle',
      {
        cx: p.x,
        cy: p.y,
        r: 0.254,
        fill: 'none',
        stroke: '#00f',
        'stroke-width': 0.3
      }
    ]) /* ...points2.map(p => [ 'circle', { cx: p.x, cy: p.y, r: 0.254 * 2, fill: 'none', stroke: 'rgba(0,255,255,0.75)', 'stroke-width': 0.1 } ])*/
  ];
  const polylines = [
    ...cells.reduce(
      (acc, { site, halfedges }) => [
        ...acc,
        [
          'polyline',
          {
            points: new PointList(halfedges.map(({ site }) => site)).toString(),
            stroke: '#f0f',
            'stroke-width': 0.1
          }
        ]
      ],
      []
    )
  ];
  //console.log('polylines:', polylines);
  //console.log('cells:', cells);
  window.cells = cells;
  Element.setCSS(svgElem, {
    position: 'absolute',
    left: 0,
    top: 0,
    width: '100%',
    height: 'auto'
  });
  //filesystem.writeFile('output.svg', svgFile);
  //console.log('svg:', svgElem);
};

function PackageChildren(element, layer) {
  let children = [...element.children].map((c, i) => [i, c]).filter(([i, p]) => p.layer && p.layer.name == 'tPlace' && p.tagName == 'wire');
  children.xml = children.map(([i, e]) => e.toXML()).join('\n');
  return children;
}

function ElementChildren(layer = 'tPlace', rfn = ent => new Map(ent)) {
  const { elements = [] } = project.doc || {};

  return rfn([...elements].map(([name, element]) => [name, PackageChildren(element, layer)]));
}

function ElementGeometries(layer = 'tPlace', rfn = ent => new Map(ent)) {
  return rfn(
    ElementChildren(layer, ent => ent)
      .map(([name, children]) => [
        name,
        new LineList(
          children.map(([i, e]) => {
            let line = e.geometry;
            if(e.curve !== undefined) line.curve = e.curve;
            line.element = e;
            line.xml = e.toXML();
            return line;
          })
        )
      ])
      .map(([name, lines]) => [name, lines, lines.slice().toPolygons(pts => new Polyline(pts))])
      .map(([name, lines, polygons]) => [name, { lines, polygons }])
  );
}

function NewPath(path) {
  let elem = SVG.create('path');
  project.svgElement.appendChild(elem);
}

const MakeFitAction = index => async event => {
  // window.transform='';
  const { buttons, type, target } = event;
  if(!type.endsWith('down') || buttons == 0) return false;
  //console.debug(`FitAct(${index})`, { buttons, type, target });
  let oldSize = Element.rect('#fence');
  let matrix = transform().invert().toMatrix();
  oldSize = matrix.transformRect(oldSize);
  let topBar = Element.rect('.buttons');
  let clientArea = Element.rect('#main');

  //console.log('MakeFitAction', clientArea);

  let f = oldSize.fit(clientArea);
  let factors = new Size(oldSize).fitFactors(new Size(clientArea));
  let t = new TransformationList().scale(factors[index], factors[index]);
  matrix = t.toMatrix();
  let newSize = matrix.transformRect(new Rect(oldSize));
  let align = 0;
  if(newSize.width > clientArea.width) align |= Align.LEFT;
  else align |= Align.CENTER;
  if(newSize.height > clientArea.height) align |= Align.TOP;
  else align |= Align.MIDDLE;
  newSize.align(clientArea, align);
  matrix = Matrix.getAffineTransform(oldSize.toPoints(), newSize.toPoints());
  //console.debug(`FitAction(${index})`, { oldSize, newSize, clientArea }, AlignToString(align), matrix.decompose());
  transform(t);
};

function ZoomFactor(val = config.zoomLog()) {
  return +Math.pow(10, val / 200).toFixed(5);
}

function ZoomLog(factor) {
  return Math.log10(factor) * 200;
}

function AdjustZoom(l = config.zoomLog()) {
  let zoomFactor = ZoomFactor(l);
  let t = new TransformationList(window.transform);
  if(!t.scaling) t.scale(zoomFactor, zoomFactor);
  else {
    t.scaling.x = zoomFactor;
    t.scaling.y = zoomFactor;
  }
  window.transform = t;
}

const CreateGrblSocket = async (port = 'tnt1') => {
  let url = makeURL({
    location: '/serial',
    protocol: 'ws',
    query: { port }
  });
  let ws = new WebSocketClient();
  await ws.connect(url);
  LogJS.info('Grbl Connected:', ws.connected);
  let output = await AsyncRead(ws);
  for await(let data of output) {
    //console.log('data:', data);
  }
};

function HandleMessage(msg) {
  const { type, origin, recipient, body } = msg;

  switch (type) {
    case 'CONTOURS': {
      let { frame, width, height, contours } = body;
      //console.log('HandleMessage', { contours });

      let lists = (typeof contours == 'string' ? contours.split(/\s*\|\s*/g) : contours).map(pointStr => new Polyline(pointStr));

      window.lists = lists;
      //console.log('HandleMessage', { type, width, height, frame }, lists);

      break;
    }
  }
}

const CreateWebSocket = async (socketURL, log, socketFn = () => {}) => {
  // log = log || ((...args) => console.log(...args));
  socketURL =
    socketURL ||
    makeURL({
      location: parseURL(window.location.href).location + 'ws',
      protocol: window.location.href.startsWith('https') ? 'wss' : 'ws'
    });
  let ws = new WebSocketClient();
  let send = ws.send;
  ws.send = (...args) => {
    let [msg] = args;
    if(!(msg instanceof Message)) msg = new Message(...args);
    //console.log('send:', msg.data);
    return send.call(ws, msg.data);
  };
  window.socket = ws;
  LogJS.info('New WebSocket: ' + socketURL);
  await ws.connect(socketURL);
  LogJS.info('WebSocket Connected:', ws.connected);
  socketFn(ws);
  ws.send('PING main.js:data!');
  let data;
  //console.log('ws', ws);

  for await(event of ws) {
    //console.log('WebSocket event:', event);
    if(event.type == 'message') {
      const { data } = event;
      //console.log('data:', abbreviate(data, 40));
      let msg = new Message(data);
      window.msg = msg;
      // LogJS.info('WebSocket recv: ' + inspect(msg));
      HandleMessage(msg);
      ws.dataAvailable !== 0;
    } else {
      //console.log(`${event.type}:`, event);
      break;
    }
  }
  await ws.disconnect();
};

const AuthorizationDialog = ({ onAuth, ...props }) => {
  const [username, setUsername] = useState(props.username || '');
  const [password, setPassword] = useState(props.password || '');

  return h(
    Portal,
    { into: '#portal' },
    h(
      'div',
      { class: 'auth-portal' },
      h('div', { class: 'auth-dialog' }, [
        h('h1', {}, 'Authorization'),
        h(
          'form',
          { action: '', method: '', onSubmit: () => false },
          h('div', { class: 'auth-form' }, [
            h('input', {
              name: 'username',
              placeholder: 'Username',
              type: 'text',
              size: 30,
              onInput: e => {
                setUsername(e.target.value);
              },
              value: username
            }),
            h('input', {
              name: 'password',
              placeholder: 'Password',
              type: 'password',
              size: 30,
              onInput: e => {
                setPassword(e.target.value);
              },
              value: password
            }),
            h('input', {
              name: 'Ok',
              type: 'submit',
              onClick: e => {
                e.preventDefault();
                onAuth({ username, password });
                return false;
              }
            })
          ])
        )
      ])
    )
  );
};

const BindGlobal = once(arg => trkl.bind(window, arg));

const AppMain = (window.onload = async () => {
  const { sortOrder, sortKey } = config;
  //prettier-ignore

  const imports = {Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList, dom, ReactComponent, iterator, eventIterator, keysim, geom, isBBox, BBox, LineList, Polygon, Circle, TouchListener, trkl, ColorMap, ClipperLib, Shape, devtools, tlite, debounce, tXml, deep, Alea, path,  Timers, asyncHelpers, Cache, CacheStorage, InterpretGcode, gcodetogeometry, GcodeObject, gcodeToObject, objectToGcode, parseGcode, GcodeParser, GCodeLineStream, parseStream, parseFile, parseFileSync, parseString, parseStringSync, noop, Interpreter, Iterator, Functional, makeLocalStorage, Repeater, useResult, LogJS, useDimensions, toXML, MutablePath, ImmutablePath, MutablePath,arrayDiff, objectDiff,  XmlObject, XmlAttr, RGBA, isRGBA, ImmutableRGBA, HSLA, isHSLA, ImmutableHSLA, React, Fragment,  FileList, Message, WebSocketClient,    PipeTo, AsyncRead, AsyncWrite,   DebugTransformStream, TextEncodeTransformer, TextEncoderStream, TextDecodeTransformer, TextDecoderStream, TransformStreamSink, TransformStreamSource, TransformStreamDefaultController, TransformStream, ArrayWriter, readStream, WriteToRepeater, LogSink, RepeaterSink, StringReader, LineReader, ChunkReader, ByteReader, PipeToRepeater,ReadFromIterator, WritableStream, useTrkl, RAD2DEG, DEG2RAD, VERTICAL, HORIZONTAL, HORIZONTAL_VERTICAL, DEBUG, log, setDebug, PinSizes, EscapeClassName, UnescapeClassName, LayerToClass, ElementToClass, ClampAngle, AlignmentAngle, MakeRotation, EagleAlignments, Alignment, SVGAlignments, AlignmentAttrs, RotateTransformation, LayerAttributes, InvertY, PolarToCartesian, CartesianToPolar, RenderArc,
 CalculateArcRadius, LinesToPath, MakeCoordTransformer, useAttributes , Wire, Instance, SchematicSymbol,  Slot, SlotProvider, Voronoi, GerberParser, lazyInitializer, LibraryRenderer, BoardRenderer, DereferenceError, EagleFactory,EagleDocument, EagleNode, EagleElement, EagleProject, EagleSVGRenderer, Renderer, SchematicRenderer, brcache, lscache, BaseCache, CachedFetch, NormalizeResponse, ResponseData, FetchCached, GetProject, ListProjects, GetLayer, AddLayer, BoardToGerber, GerberToGcode, GcodeToPolylines, 
 classNames , BinaryTree, normalizePath, reverseNormalizedPath, reverseSubPath, reversePath, ...commands,  DEBUG, objectInspect, SvgPath, renderToString , ...ecmascript };

  Object.assign(globalThis, {
    open,
    ClearCache,
    brcache,
    lscache,
    BaseCache,
    CachedFetch,
    FetchURL,
    roundTo,
    define,
    properties,
    className,
    functionName,
    keys,
    entries,
    values,
    tryCatch,
    tryFunction
  });

  const localFunctions = {
    PackageChildren,
    ElementChildren,
    Timer,
    DrawSVG,
    ElementToXML,
    filesystem,
    LoadFile,
    SaveFile,
    SaveSVG,
    LoadSVG,
    LoadImage,
    RenderComponent,
    ModifyColors,
    GerberLayers,
    LoadDocument,
    NextDocument,
    ChooseDocument,
    GenerateVoronoi,
    MakeFitAction,
    CreateWebSocket,
    CreateGrblSocket,
    BindGlobal,
    AppMain,
    serial,
    FindLayer,
    OutsetPath,
    OutsetPaths,
    ClipPath,
    PathToPolyline,
    PathsToPolylines,
    GetPaths,
    DrawArc,
    maxZIndex,
    ClearCache,
    PackageNames,
    DrawBinaryTree,
    EagleMaps,
    SaveConfig,
    LoadConfig,
    FixedMedium,
    ...rpc2,
    ...commands,
    fnmatch,
    PATH_FNM_MULTI,

    ...{ GetColorBands, GetElements, GetInstances, GetPositions, UpdateMeasures, AlignAll, scientific },
    ...{
      GetExponent,
      GetMantissa,
      ValueToNumber,
      NumberToValue,
      GetMultipliers,
      GetFactor,
      GetColorBands,
      BG,
      digit2color
    }
  };

  if(store.keys().length == 0) {
    await LoadConfig()
      .then(response => {
        for(let [key, value] of response.entries()) {
          //console.log(`Initializing store set('${key}',`, value, `)`);
          store.set(key, value);
        }

        for(let key of store.keys()) {
          const value = store.get(key);
          switch (key) {
            case 'url':
              config.listURL(value);
              break;
            case 'filter':
              config.searchFilter(value);
              break;
            case 'zoom':
              config.zoomLog(value);
              break;
            case 'console':
              config.logSize(value);
              break;
            case 'debug':
              config.debugFlag(value);
              break;
            case 'auth':
              config.credentials(value);
              break;
          }
        }
      })
      .catch(e => {});
  }
  const importedNames = Object.keys(imports);
  //console.debug('Dupes:', getMemberNames(window).filter(m => importedNames.indexOf(m) != -1));

  //prettier-ignore
  weakDefine(window, { rpc:rpc2 });
  weakDefine(window, imports);
  weakDefine(window.Element, getMethods(dom.Element));
  weakDefine(window, dom, geom, imports, localFunctions);
  weakDefine(window, {
    functions: filter(localFunctions, v => typeof v == 'function'),
    dom,
    geom,
    config,
    loading,
    filePanel
  });
  Error.stackTraceLimit = 100;

  weakDefine(window, {
    TestArc: () => timer(2000).then(() => DrawArc({ x: 50, y: 150 }, { x: 350, y: 300 }, 120 * (Math.PI / 180)))
  });

  const timestamps = new Repeater(async (push, stop) => {
    push(Date.now());
    const interval = setInterval(() => push(Date.now()), 1000);
    await stop;
    clearInterval(interval);
  });

  const logger = new Repeater(async (push, stop) => {
    push(['DEBUG', null, null, 'Load ready!']);
    window.pushlog = push;
    await stop;
  });
  logger.push = window.pushlog;

  //window.focusSearch = trkl();
  window.currentSearch = trkl(null);

  window.keystroke =
    target =>
    (key, modifiers = 0) =>
      keysim.Keyboard.US_ENGLISH.dispatchEventsForKeystroke(new keysim.Keystroke(modifiers, key), target);

  window.focusSearch = state => {
    const input = currentSearch();
    //console.log('focusSearch', input.tagName, state);
    input[state ? 'focus' : 'blur']();
  };

  // prettier-ignore
  BindGlobal({ projects, socket, transform, size: sizeListener, aspect: aspectListener, showSearch,   watched: dump, 
    children: () => elementChildren(),
     geometries: () => elementGeometries(),
    ...config });

  currentSearch.subscribe(value => {
    if(value) {
      focusSearch(false);
      Timer.once(1000).then(() => focusSearch(true));
    }
  });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = props => html` <div>This is a test</div> `;

  let c = h(testComponent, {});
  window.testComponent = c;

  const UpdateProjectList = async (opts = config.listURL() ? { url: config.listURL(), ...credentials } : {}) => {
    let list = [];
    //console.log('opts:', opts);
    let { url, ...restOfOpts } = opts;
    let urls = url ? url.split(/\n/g) : [null];
    for(url of urls) {
      //console.log('UpdateProjectList:', { ...opts, ...credentials, url });
      // let data = await ListProjects({ ...opts, ...credentials, url });

      let data = {
        files: await fetch('files?filter=.*.(brd|sch|lbr|GBL|GKO|GTL)$&flat=1', {
          method: 'get' /*,
          body: JSON.stringify({ filter: '.*.(brd|sch|lbr|GBL|GKO|GTL)$' })*/
        })
          .then(resp => resp.text())
          .then(json => JSON.parse(json))
      };

      if(typeof data == 'string') data = JSON.parse(data);

      console.log('UpdateProjectList', data);

      let files = (typeof data == 'object' && data != null && data.files) || [];

      if(isObject(files) && 'files' in files) files = files.files;

      //console.log('files', files);

      function File(obj, i) {
        const { name } = obj;
        let file = this instanceof File ? this : Object.create(File.prototype);
        let data = trkl({ percent: NaN });
        Object.assign(file, obj);
        file.name = name;
        file.i = i;
        trkl.bind(file, { data });
        //console.info(`Got file '${name.replace(/.*:\/\//g, '').replace(/raw.githubusercontent.com/, 'github.com') || name.replace(/.*\//g, '')}'`);

        return file;
      }
      File.prototype.toString = function() {
        return this.name;
      };
      File.prototype.path = function() {
        return this.dir + '/' + this.name;
      };

   /*   weakDefine(File.prototype, {get doc() {
        return LoadFile(this.name);
      }})*/

      if(files) {
        list = list.concat(files.sort((a, b) => a.name.localeCompare(b.name)).map((obj, i) => new File(obj, i)));
        let svgs = list.reduce((acc, file) => {
          if(/\.lbr$/i.test(file.name)) return acc;
          file.svg = `${path.basename(file.name)}.${path.extname(file.name)}.svg`;
          //console.log(`file.svg = '${file.svg}'`);
          return [...acc, file.svg];
        }, []);

        data = await ListProjects({ descriptions: false, names: svgs });
        files = globalThis.files = data || [];

        for(let svgFile of files) {
          if(isObject(svgFile) && svgFile.mtime !== undefined) {
            const f = list.find(i => i.svg === svgFile.name);
            if(isObject(f) && f.mtime !== undefined) {
              const delta = svgFile.mtime - f.mtime;

              f.modified = delta < 0;
            }
          }
        }
      }
    }

    LogJS.info(`retrieved project list. Got ${list.length} items.`);

    projects(list);
  };

  UpdateProjectList();

  /*(async function() {
    while(true) {
      await CreateWebSocket(null, null, ws => (window.socket = ws)).catch(console.error);
      await waitFor(1000);
    }
  })();*/

  const crosshair = { show: trkl(false), position: trkl({ x: 0, y: 0 }) };

  window.crosshair = trkl.bind({}, crosshair);

  config.credentials.subscribe(value => {
    store.set('auth', value);
    LogJS.info(`config.credentials`, value);
  });
  config.searchFilter.subscribe(value => {
    store.set('filter', value);
    LogJS.info(`config.searchFilter is ${value}`);
  });

  config.listURL.subscribe(value => {
    store.set('url', value);
    LogJS.info(`config.listURL is '${value}'`);
  });
  config.debugFlag.subscribe(value => store.set('debug', value));

  config.logSize.subscribe(value => {
    const { width, height } = value;

    if(width === undefined || height === undefined) {
      throw new Error('config.logSize undefined');
    }
    store.set('console', value);
    //LogJS.info(`config.logSize is ${value.width} x ${value.height}`);
  });

  //trkl.bind(window, { config.searchFilter, config.listURL });
  //trkl.bind(window, { svgFactory });

  //trkl.bind(window, { config.zoomLog, zoom: zoomValue, config.logSize });

  config.zoomLog.subscribe(value => {
    let factor = ZoomFactor(value);
    //console.info('zoomFactor changed', value, factor);
    store.set('zoom', value);
    if(value === 1) throw new Error(value);
  });

  const updateIfChanged = (trkl, newValue, callback) => {
    const oldValue = trkl() || [];
    //console.info('updateIfChanged ', { oldValue, newValue });
    if(!Array.prototype.every.call(oldValue, (elem, i) => newValue[i] === elem)) return false;
    trkl(newValue);
    if(typeof callback == 'function') callback(trkl, oldValue, newValue);
    return true;
  };

  const changeInput = e => {
    const { target } = e;
    LogJS.info('changeInput:', target.value);
    let { value } = target;
    let parts = value.split(/\s+/g);
    let urls = parts.filter(p => /\:\/\//.test(p)).join('\n');
    updateIfChanged(config.listURL, urls, arg => {
      //console.debug('updateIfChanged:', arg);
    });
    config.listURL(urls);
    //    value = parts.filter(p => !/\:\/\//.test(p)).join(' ');
    config.searchFilter(value == '' ? '*' : value.split(/\s*\|\s*/g).join(' | '));
  };

  const Consumer = props => {
    const result = useResult(async function* () {
      for await(let time of timestamps) {
        yield time;
      }
    });
    return h(
      'div',
      {
        className: 'vcenter fixed grow no-select',
        style: {
          flex: '1 0 auto',
          justifyContent: 'flex-end',
          color: 'white',
          height: '60px',
          width: '200px',
          padding: '0 10px 0 0'
        }
      },
      [result && new Date(result.value).toLocaleTimeString('de-CH')]
    );
  };
  LogJS.addAppender(
    class extends LogJS.BaseAppender {
      log(type, time, msg) {
        let d = new Date(time);
        if(typeof window.pushlog == 'function') window.pushlog([type, isoDate(d).replace(/-/g, ''), d.toLocaleTimeString(navigator.language || 'de'), msg]);
      }
    }
  );
  let loggerRect = new Rect();
  const Logger = props => {
    const [lines, setLines] = useState([]);
    const [ref, rect] = useDimensions();

    const r = new Rect(rect);
    if(!loggerRect.equals(r)) {
      loggerRect = r;
    }
    const result = useResult(async function* () {
      for await(let msg of logger) {
        //console.debug("msg:", msg);
        yield msg;
      }
    });
    if(result) {
      lines.push(result.value);
    }
    return h(
      'table',
      { className: 'logger', ref },
      lines.slice(-100, lines.length).map(([type, d, t, m], i) =>
        h('tr', {}, [
          h(
            'td',
            { className: 'log sign' },
            h('img', {
              className: 'log sign',
              src: `static/${type.toLowerCase() || 'warn'}.svg`,
              style: { height: '14px', width: 'auto', marginTop: '-1px' }
            })
          ),
          h('td', { className: 'log message' }, m + '')
        ])
      )
    );
  };

  dump({ ...dump(), test: 123 });

  const Dumper = props => {
    const [values, setValues] = useState(dump());
    let lines = [];
    dump.subscribe(value => setValues(value));
    for(let [key, value] of Object.entries(values)) lines.push([key, value]);
    return h(
      'table',
      { border: '0', cellpadding: 3, cellspacing: 0, className: 'dumper' },
      lines.map(([k, v], i) => h('tr', { className: 'watch' }, [h('td', { className: 'name' }, k + ''), h('td', { className: 'value' }, v + '')]))
    );
  };

  const Commander = ({ onCommand, ...props }) => {
    const [inputText, setInputText] = useState('');
    const handler = e => {
      const { target } = e;
      if(e.type.endsWith('down') && e.keyCode == 13) {
        const value = target.value || inputText;
        if(value != '') {
          if(typeof onCommand == 'function') onCommand(value);
          setInputText('');
        }
      } else {
        setInputText(target.value);
      }
    };
    return h(
      'input',
      {
        type: 'text',
        className: 'commander',
        value: inputText,
        onKeyDown: handler,
        autofocus: true
      },
      []
    );
  };

  const layersDropDown = trkl(false);

  const toggle = trkl => trkl(!trkl());
  let setTo;

  const Layer = ({ title, name, label, i, color, element, className, ...props }) => {
    let setVisible = props.visible || element.handlers.visible,
      visible = useTrkl(setVisible);

    const isVisible = visible === true || (visible !== false && { yes: true }[visible]);

    if(isObject(element) && 'visible' in element) setVisible = value => (element.visible = value);
    let [solo, setSolo] = useState(null);

    const onMouseDown = debounce(e => {
      //console.log('onMouseDown', e);
      /* if(e.buttons & 1)*/ {
        setVisible((setTo = !isVisible));
        return true;
      }
    }, 200);
    //console.log(`Layer #${i} ${name} isVisible=${isVisible}`);
    return h(
      'div',
      {
        className: classNames(className, !isVisible && 'gray'),
        id: `layer-${i}`,
        'data-layer': `${(element && element.number) || i} ${(element && element.name) || name}`,
        onClick: useDoubleClick(
          e => {
            let { target } = e;

            while(!target.hasAttribute('id') && target.parentElement) target = target.parentElement;
            //console.log('Double click', { solo, i, target });
            let layers = [...layerList()];
            let visibleLayers = layers.filter(l => is.on(l.visible()));
            let hiddenLayers = layers.filter(l => !is.on(l.visible()));
            //console.log('Layer.onClick', { visibleLayers, hiddenLayers, solo });

            if(solo) {
              //onMouseDown.clear();
              let restoreData = solo;

              setSolo(null);
              restoreItemStates(restoreData, (item, value) => item.visible(value ? 'yes' : 'no'));

              //console.debug('restoreData:', restoreData);
            } else {
              let saved = saveItemStates(layers, item => is.on(item.visible()));
              //console.debug('saved:', saved);
              setSolo(saved);
              //console.debug('layers:', layers);
              const states = layers.map(l => [l, l.i == i]);
              //console.debug('states:', states);
              states.forEach(([l, state]) => l.visible(state ? 'yes' : 'no'));
              //    for(let l of layers) l.visible(l.name == name ? 'no' : 'yes');
              setVisible(true);
            }
            layerList(layers);
          },
          onMouseDown ||
            (e => {
              let layers = [...layerList()];
              if(solo) {
              } else {
              }

              layerList(layers);
            }),
          { timeout: 40 }
        ),
        onMouseMove: e => {
          if(e.buttons & 1 && setTo !== undefined) setVisible(setTo);
        },
        onMouseUp: e => {
          setTo = null;
        } /*,
        onMouseDown*/
      },
      [
        h(
          'span',
          {
            className: classNames(className, 'number'),
            style: {
              background: color || (isObject(element) && element.color)
            },
            ...props
          },
          `${i}`
        ),
        h(
          'span',
          {
            className: classNames(className, 'name', !isVisible && 'gray', solo && 'bold'),
            ...props
          },
          `${name}`
        ),
        h('img', {
          className: classNames(className, 'visible'),
          ...props,
          style: { height: '1em', width: 'auto' },
          src: `static/svg/${isVisible ? 'show' : 'hide'}.svg`
        })
      ]
    );
  };

  class DocumentList {
    constructor() {
      this.repeater = new Repeater((push, stop) => {
        this.push = push;
        this.stop = stop;
      });
      this.map = new Map();
    }

    add(name, component) {
      const { map } = this;

      if(!ReactComponent.isComponent(component)) component = h(component, {}, []);

      map.set(name, component);
      this.update();
    }

    addEntry([name, component]) {
      return this.add(name, component);
    }

    remove(name) {
      if(map.has(name)) {
        map.delete(name);
        this.update();
        return true;
      }
    }

    update() {
      const { map } = this;
      this.push([...map.values()]);
    }
  }
  let data;

  console.log('DUMMY');

  window.documentList = data = new DocumentList();
  React.render(h(DisplayList, { data }), Element.find('#display'));

  let preactComponent = h(SlotProvider, {}, [
    h(Panel, { className: classNames('buttons', 'no-select'), tag: 'header' }, [
      h(Button, {
        image: 'static/svg/browse.svg',
        state: open,
        fn: e => {
          if(e.type.endsWith('down')) {
            //console.log('file list push', e);
            open(!open());
          }
        }
      }),

      /* h(Button, {
          caption: 'Random',
          fn: ModifyColors(c => c.replaceAll(c => HSLA.random()))
        }),
        h(Button, {
          caption: 'Invert',
          fn: ModifyColors(c => c.replaceAll(c => c.invert()))
        }),*/
      h(DropDown, {}, []),
      h(Button, {
        //  caption: '↔',
        fn: MakeFitAction(VERTICAL & 1),
        image: 'static/svg/fit-vertical.svg'
      }),
      h(Button, {
        //  caption: '↕',
        fn: MakeFitAction(HORIZONTAL & 1),
        image: 'static/svg/fit-horizontal.svg'
      }),
      h(Conditional, { signal: currentProj }, [
        h(Button, {
          //  caption: '↕',
          fn: () => config.showGrid(!config.showGrid()),
          state: config.showGrid,
          toggle: true,
          image: 'static/svg/grid.svg'
        }),
        h(
          DropDown,
          {
            isOpen: layersDropDown.subscribe(open => console.log('layers dropdown', { open }))
            // into: '#portal'
          },
          [
            props =>
              h(Button, {
                ...props,
                toggle: true,
                state: layersDropDown,
                image: 'static/svg/layers.svg'
              }),
            props =>
              h(
                Chooser,
                {
                  ...props,
                  className: 'layers',
                  itemClass: 'layer',
                  itemComponent: Layer,
                  items: layerList
                },
                []
              )
          ]
        ),
        h(Button, {
          fn: debounce(async e => {
            /*console.log("CAM button",{e});
              if(e.type.endsWith('up')) return false;*/
            let r;
            project.gerber = {};
            project.gcode = {};
            //console.debug('CAM Button');
            for(let side of ['back', 'front', 'drill', 'outline']) {
              let gerber = await BoardToGerber(project, {
                side,
                [side]: true,
                fetch: ['drill', 'outline'].indexOf(side) != -1
              });

              if(gerber) {
                //console.debug(`project.gerber['${side}'] =`, gerber);
                project.gerber[side] = gerber;
                if(gerber && gerber.data) {
                  gerber.cmds = await GerberParser.parse(gerber.data);
                  gerber.unit = gerber.cmds.find(i => i.prop == 'units');

                  gerber.points = gerber.cmds.filter(i => i.coord).map(({ coord }) => new Point(coord.x, coord.y));
                }
                //console.debug('BoardToGerber side =', side, ' file =', gerber.file);
              }
            }
            const sides = /*Object.fromEntries*/ ['back', 'front', 'drill', 'outline'].map(side => [side, project.gerber[side].file]);
            //console.debug('  sides = ', sides);
            //console.debug('  project = ', project);
            let allGcode = {};
            for(let [side, file] of sides) {
              let gcode = await GerberToGcode(project, {
                side,
                file,
                nog64: true,
                'fill-outline': true,
                voronoi: true,
                /*'zero-start': true,*/ nog81: true
              });
              allGcode[side] = gcode;
              //project.gcode[side] = gcode.data && gcode.data.data ? gcode.data.data : gcode.data;
            }
            //console.debug('GerberToGcode allGcode = ', allGcode);
            let bbox;

            for(let side of ['outline', 'back', 'front', 'drill']) {
              try {
                let gerber = project.gerber[side];
                console.debug('GerberToGcode  ', { gerber, allGcode, side });
                let data = allGcode[side];
                console.debug('GerberToGcode  ', { data });
                let file = gerber.file || allGcode.data.files[side];
                //console.debug('GerberToGcode  ', { gerber, data, file });

                if(data) {
                  let gc = { data, file };

                  if(side != 'drill') {
                    let processed = file.replace(/\.ngc$/, '.svg');
                    //console.debug('processed', processed);
                    gc.svg = await FetchURL(processed).then(ResponseData);
                    let pos;

                    if(gc.svg) {
                      if((pos = gc.svg.indexOf('<svg ')) != -1) gc.svg = gc.svg.substring(pos);

                      if(side == 'outline') {
                        //console.debug('outline', gc.svg);
                        let xmlData = tXml(gc.svg);
                        console.debug('xmlData', xmlData);
                        const tail = a => (a.length ? a[a.length - 1] : null);
                        let svgPath = tail(xmlData[0].children).children[0];
                        let points = SVG.pathToPoints(svgPath.attributes);
                        //console.debug('points:', points);
                        bbox = new Rect(new BBox().update(points)).round(0.001);
                        //console.debug('bbox:', bbox);
                        //

                        continue;
                      }
                      //console.debug('gc.svg ',gc.svg );
                      let layer = GetLayer({
                        name: makeLayerName('processed', side),
                        'data-filename': processed,
                        create: (project, props = {}) => {
                          let g = SVG.create('g', { innerHTML: gc.svg, ...props }, project.svgElement);
                          g.innerHTML = gc.svg;
                          if(g.firstElementChild && g.firstElementChild.tagName == 'svg') {
                            let svg = g.firstElementChild;
                            ['width', 'height', 'xmlns', 'xmlns:xlink', 'version'].forEach(a => svg.removeAttribute(a));
                            svg.setAttribute('viewBox', bbox);
                          }
                          Element.findAll('path', g)
                            .filter(e => e.style['fill-opacity'] == 1)
                            .forEach(e => (e.style.display = 'none'));

                          ['fill', 'stroke'].forEach(name =>
                            Element.findAll(`[style*="${name}:"]`, g).forEach(e => {
                              const value = e.style[name];
                              if(value != 'rgb(0, 0, 0)' && value != 'none') {
                                e.setAttribute(name, value);
                                e.style.removeProperty(name);
                              }
                            })
                          );

                          return g;
                        }
                      });
                      /*
                      layer.sublayers = histogram(Element.walk(layer.dom, (e, acc) => (e.tagName.endsWith('g') ? acc : [...acc, e]), []),
                        e => e.getAttribute('style'),
                        new Map(),
                        () => new Set()
                      );*/
                    }
                  }

                  //console.debug('GerberToGcode side =', side, ' gc =', gc.file, ' svg =', abbreviate(gc.svg));
                }
              } catch(err) {
                console.error('ERROR: ' + err.message);
                console.error('STACK:', err.stack);
              }
            }
            gcode(project.gcode);

            function makeLayerName(name, side) {
              const prefix = side == 'front' ? 't-' : side == 'back' ? 'b-' : '';
              return camelize(prefix + path.basename(name, /\.[^.]+$/).replace(new RegExp(`_${side}`), ''));
            }
          }, 100),
          'data-tooltip': 'Generate Gerber RS274-X CAM data',
          image: 'static/svg/cnc-obrabeni.svg'
        })
      ]),

      h(Conditional, { signal: gcode }, [
        h(Button, {
          fn: () => {
            const colors = {
              front: 'hsl(300,100%,70%)',
              back: 'hsl(230,100%,70%)'
            };
            for(let side of ['back', 'front']) {
              let gc = project.gcode[side];
              if(gc) {
                //console.debug(`${side} gcode gc =`, gc);
                GcodeToPolylines(gc.data, {
                  fill: false,
                  color: colors[side],
                  side
                });
              }
            }
          },
          'data-tooltip': 'Create Voronoi diagram',
          image: 'static/svg/voronoi.svg'
        })
      ]),
      h(Toggle, {
        state: sortOrder,
        images: ['static/svg/sort-asc.svg', 'static/svg/sort-desc.svg'],
        //disable: trkl(true),
        visible: open
      }),
      h(DynamicLabel, {
        className: 'vcenter pad-lr',
        caption: documentTitle
      }),
      h(DynamicLabel, {
        className: 'vcenter pad-lr',
        caption: documentSize
      }),
      h(Consumer, {})
    ]),

    /*  h('div', { style: { display: 'inline-flex', flexFlow: 'row', alignItems: 'stretch', height: '100px', padding: '10px' } }, [
        h(ColorWheel, {}),
        h(Slider, {
          min: 0,
          max: 100,
          value: 100,
          orient: 'vertical',
          name: 'S',
          length: '10px',
          style: { flex: '0 1 auto' },
          onChange: value => {
            //console.log('value:', value);
          }
        }),
        h(Slider, {
          min: 0,
          max: 100,
          orient: 'vertical',
          name: 'L',
          length: '10px',
          style: { flex: '0 1 auto' },
          onChange: value => {
            //console.log('value:', value);
          }
        })
      ]),*/
    h(
      FileList,
      {
        listTag: 'nav',
        files: projects,
        onActive: open,
        onChange: debounce(async (e, p, i) => await ChooseDocument(p, i), 5000, {
          leading: true
        }),
        filter: config.searchFilter,
        showSearch,
        changeInput,
        focusSearch,
        sortKey,
        sortOrder,
        makeSortCompare: key =>
          key == 'name' || !key
            ? function(a, b) {
                let nameA = a.name,
                  nameB = b.name;
                let extA = path.extname(nameA),
                  extB = path.extname(nameB);
                if(extA == '.lbr' && extB != '.lbr') return -1;
                if(extA != '.lbr' && extB == '.lbr') return 1;
                return nameA.localeCompare(nameB);
              }
            : function(a, b) {
                let valueA = a[key],
                  valueB = b[key];
                return valueA < valueB ? -1 : valueA > valueB ? 1 : 0;
              },
        currentInput: currentSearch
      },
      [
        h(
          ButtonGroup,
          {
            className: 'small',
            onChange(event) {
              let { currentTarget, target } = event;
              let key = target.getAttribute('data-key');

              console.log('Sort order changed', key);
              config.sortKey(key);
            }
          },
          [
            props =>
              h('img', {
                src: 'static/svg/sort-name-2.svg',
                alt: 'Name',
                'data-key': 'name',
                ...props
              }),
            props =>
              h('img', {
                src: 'static/svg/sort-time-2.svg',
                alt: 'Modification time',
                'data-key': 'mtime',
                ...props
              }),
            props =>
              h('img', {
                src: 'static/svg/sort-size-2.svg',
                alt: 'Size',
                'data-key': 'size',
                ...props
              })
          ]
        )
      ]
    ),

    h(CrossHair, { ...crosshair }),
    h(FloatingPanel, { onSize: config.logSize, className: 'no-select', id: 'console' }, [
      /*h(div, {}, [ */ h(Logger, {}),
      h(Dumper, {}),
      h(Commander, {
        onCommand: cmdStr => {
          let fn = new Function(`return ${cmdStr};`);

          //console.log('Command:', cmdStr);
          LogJS.info(`> ${cmdStr}`);
          let result = fn();
          LogJS.info(`= ${toSource(result)}`);
        }
      }) /*])*/
    ]),
    h(Slot, { name: 'layers' }),
    h(Conditional, { signal: wantAuthorization }, h(AuthorizationDialog, { onAuth: config.credentials })),
    h(Ruler, {
      class: 'ruler-container vertical ',
      handleChange: e => {
        //console.log('Ruler changed:', e);
      },
      style: {
        position: 'absolute',
        right: 0
      }
    })
  ]);
  console.log('DUMMY', (window.preactComponent = preactComponent));

  render(preactComponent, Element.find('#preact'));

  let move, resize;
  let box;
  container = Element.find('#main');

  let touchHandler = trkl();
  let moveHandler = trkl();

  //Element.find('.transformed-element-size').setAttribute('id', 'transformed-element');

  TouchListener(touchHandler, { element: window });

  window.addEventListener('pointermove', moveHandler);

  let rects = (window.rects = new Map());
  let elems = (window.elems = new Set());
  window.addEventListener('pointerdown', event => {
    window.elements = [...elems].filter(e => e.tagName == 'path');
  });
  let css = {
    cursor: undefined,
    'pointer-events': undefined,
    'user-select': undefined
  };

  //moveHandler.subscribe(MoveEvent);

  touchHandler.subscribe(TouchEvent);

  function MoveEvent(event, prevEvent) {
    const { x, y, clientX, clientY, index, buttons, start, type, target } = event;
    window.lastMoveEvent = event;
    event.elements = document.elementsFromPoint(x, y);
    function* WalkUp(e) {
      while(e) {
        yield e;
        e = e.parentElement;
      }
    }
    let zIndex = find(
      map(WalkUp(event.target), e => e.style.getPropertyValue('z-index')),
      z => /^[0-9]/.test(z)
    );
    if(zIndex > 0) clear(event.elements);
    for(let e of event.elements)
      Element.walkUp(e)
        .slice(1)
        .forEach(p => remove(event.elements, p));
    remove(event.elements, document.documentElement);

    event.layers = new Map(
      event.elements.map(e => [
        e,
        Element.walkUp(e, e => {
          if(e.hasAttribute('data-layer')) throw e.getAttribute('data-layer');
        })
      ])
    );
    event.colors = new Map();
    for(let [e, layer] of event.layers) {
      if(!layer || /(Measure|Dimension)/.test(layer)) continue;
      let l = FindLayer(layer);
      if(l) event.colors.set(e, l.color.setOpacity(0.8) || '#000');
    }
    event.classes = new Map(
      event.elements.map(e => [
        e,
        ifThenElse(
          v => v,
          l => l.map(e => e.classList.value),
          () => ''
        )(Element.walkUp(e, (e, depth) => !e.classList.value.startsWith('aspect') && e.classList.value))
      ])
    );
    removeIf(event.classes, classes => classes == '');
    removeIf(event.elements, e => e.tagName == 'polyline');
    removeIf(event.elements, e => !(event.classes.has(e) || event.colors.has(e)));
    const group =
      project &&
      project.makeGroup &&
      project.makeGroup({
        id: 'rects',
        stroke: '#ff6f00',
        'stroke-width': 0.127,
        fill: 'none',
        'stroke-linecap': 'square',
        'vector-effect': 'non-scaling-stroke',
        'pointer-events': 'none'
      });

    if(prevEvent && group) {
      let u = union(prevEvent.elements, event.elements, (a, b) => a.isSameNode(b));
      let [remove, add] = difference(prevEvent.elements, event.elements, (a, b) => a.findIndex(Node.prototype.isSameNode, b) != -1);

      //console.log('difference:', [remove,add], 'union:', u);
      //console.log('add:', add);

      const bboxes = new Map(add.map(e => [e, new Rect(e.getBBox ? e.getBBox() : e.getBoundingClientRect())]));

      for(let [e, rect] of bboxes) {
        let transforms =
          Element.walkUp(e, (p, d, set, stop) =>
            p.parentElement == null || p.parentElement.isSameNode(p.ownerSVGElement) ? stop() : p.hasAttribute('transform') && set(p.getAttribute('transform'))
          ) || [];
        transforms = transforms.reverse();
        elems.add(e);
        let props = {
          ...rect.round(0.001).toObject(),
          transform: transforms.join(' ')
        };
        rects.set(e, [
          // SVG.create('rect', { ...props, stroke: '#000', 'stroke-width': 0.127 * 2 }, group),
          SVG.create('rect', { ...props, 'stroke-dasharray': '0.508 0.508', stroke: '#000' }, group),
          SVG.create(
            'rect',
            {
              ...props,
              'stroke-dasharray': '0.508 0.508',
              'stroke-dashoffset': 0.508,
              stroke: '#ff0'
            },
            group
          )
        ]);
      }
      /*
      add.forEach(e => {
        elems.add(e);
        rects.set(e, devtools.rect(new Rect(e.getBoundingClientRect()), event.colors.get(e) || '#00000000', event.colors.get(e)));
      });*/
      remove.forEach(e => {
        let rect = rects.get(e);
        rects.delete(e);

        if(Array.isArray(rect)) rect.forEach(e => Element.remove(e));
      });

      if(bboxes.size) {
        /* console.log('event.elements:', event.elements);
        //console.log('event.classes:', event.classes);
        //console.log('event.target:', zIndex);*/
        //console.log('rects:', clone(bboxes));
      }
    }
  }
  function TouchEvent(event) {
    const { x, y, index, buttons, start, type, target } = event;
    //console.log('touchHandler', event);
    if(type.endsWith('end') || type.endsWith('up')) return cancel();
    if(event.buttons === 0 && type.endsWith('move')) return cancel();
    // if(event.index > 0) console.log('touch', { x, y, index, buttons, type, target }, container);
    if(!move && !resize) {
      let elemId;
      //console.log('target:', target);
      box = (e => {
        do {
          elemId = e.getAttribute('id');
          if(['fence', 'console'].indexOf(elemId) != -1) return e;
        } while((e = e.parentElement));
      })(target);
      //console.log('box:', box);
      if(event.buttons && event.buttons != 1) {
        if('preventDefault' in event) event.preventDefault();
        if(!resize && box) {
          let edges = Element.rect(box).toPoints();
          let corners = [edges[0], edges[2]].map((p, i) => [i, p.distance(new Point(start).sum(x, y)), p]);
          let edge = corners.sort((a, b) => a[1] - b[1])[0];

          window.resize = resize = Element.resizeRelative(box, null, edge[0] ? -1 : 1, size => {
            //console.log('resizeRelative:', { elemId, size });
            if(elemId == 'console') config.logSize(size);
          });

          box.style.cursor = `nwse-resize`;
          //console.log('RESIZE:', { resize, box, corners, edge });
          return true;
        }
        return cancel();
      }

      //        let box = Element.find('#main').firstElementChild;
      const id = box && box.getAttribute('id');

      if(id == 'console') {
        const rects = [true, false].map(border => Element.rect(box, { border }));
        let p = new Point(start.x + x, start.y + y);
        //console.log('', p);
        const inside = rects.map(r => r.inside(p));
        const inBorder = inside[0] && !inside[1];
        function mod(n, m) {
          return ((n % m) + m) % m;
        }
        let rad = p.diff(rects[0].center).toAngle();
        let deg = Math.round((rad * 180) / Math.PI);
        let sector = mod(Math.floor(((180 - deg) * 8) / 360), 8);
        let directions = ['n', 'ne', 'e', 'se', 's', 'sw', 'w', 'nw'];
        let norm = Point.fromAngle(rad, 1);
        //console.log('box: ', id, ...inside, inBorder, p, { sector, deg });
        let compass = directions[sector];
      }
      if(box) {
        let translation = new Translation();
        let transformList = new TransformationList([translation]).concat(transform());

        let setStyle = once(() =>
          Element.setCSS(box, {
            cursor: 'move',
            'pointer-events': 'none',
            'user-select': 'none'
          })
        );

        window.move = move = Element.moveRelative(box, null, id == 'console' ? ['right', 'bottom'] : ['left', 'top'], (pos, last, first) => {
          if(pos && first) {
            let rel = Point.diff(pos, first);
            if(rel.distanceSquared() > 0) {
              setStyle();

              translation.x = rel.x;
              translation.y = rel.y;
              transform(transformList.collapse());
              //console.log('TouchHandler transform:', transform());
            }
          }
        });
      }
      return true;
    }
    if((move || resize) && event.buttons == 0) {
      return cancel();
    }

    if(event.index > 0) {
      let rel = new Point(event).sub(event.start);
      let absolute = new Point(start).add(rel);

      if(resize) {
        if(event.buttons > 0) resize(-rel.x, -rel.y);
        else resize = resize.jump();
      } else if(move) {
        /*  window.crosshair.show = true;
          window.crosshair.position = absolute;*/

        //console.log('move', { rel, absolute });
        if(event.buttons > 0) move(rel.x, rel.y);
        else move = move.jump();
      }
    }
    function cancel() {
      move = null;
      resize = null;
      window.crosshair.show = false;

      if(box && box.style) Element.setCSS(box, css);
      /*return*/ event.cancel();
      return false;
    }
  }
  window.oncontextmenu = function(e) {
    const { x, y, index, buttons, start, type, target } = event;
    let rect = Element.rect('.transformed-element-size');
    let cons = Element.rect('#console');
    if(rect && cons) if (rect.inside(event) && !cons.inside(event)) return true;
    if(e.shiftKey && e.altKey) return true;
    //console.log('oncontextmenu',  event);
    return false;
  };
  window.processEvents = async function eventLoop() {
    for await(let e of new EventIterator('touch')) {
      const {
        altKey,
        bubbles,
        button,
        buttons,
        cancelBubble,
        cancelable,
        clientX,
        clientY,
        composed,
        ctrlKey,
        detail,
        eventPhase,
        fromElement,
        isTrusted,
        layerX,
        layerY,
        metaKey,
        movementX,
        movementY,
        offsetX,
        offsetY,
        pageX,
        pageY,
        path,
        region,
        relatedTarget,
        returnValue,
        screenX,
        screenY,
        shiftKey,
        srcElement,
        target,
        timeStamp,
        toElement,
        type,
        view,
        which,
        x,
        y,
        ...event
      } = e;
      // LogJS.info(`${type} ` + /* toSource(e)+ */ ` ${x},${y} → ${Element.xpath(target)}`);
    }
  };
  processEvents();

  //  eventLoop();

  window.styles = CSS.create('head');

  /* document.addEventListener('keydown', event => {
    const { ctrlKey, shiftKey, altKey, metaKey } = event;

    if(true || ctrlKey || shiftKey || altKey || metaKey) {
      const { key, code, keyCode } = event;
      const { target, currentTarget } = event;
      //console.log('keydown: ', (window.keyEvent = event));
    }
  });*/

  window.addEventListener('wheel', event => {
    const { wheelDelta, deltaMode, deltaX, deltaY, screenX, screenY, clientX, clientY, pageX, pageY, x, y, offsetX, offsetY, layerX, layerY } = event;

    window.wheelEvent = event;

    const clientArea = Element.rect('body > div');
    const sideBar = Element.rect('.sidebar');

    if(sideBar.x2 > clientArea.x1) {
      clientArea.width -= sideBar.x2;
      clientArea.x = sideBar.x2;
      clientArea.width = window.innerWidth - clientArea.x;
    }
    clientArea.height = window.innerHeight;
    clientArea.x += container.parentElement.scrollLeft;

    const clientCenter = clientArea.center;
    const { target, currentTarget, buttons, altKey, ctrlKey, shiftKey } = event;
    const pos = new Point(clientX, clientY);

    if(
      Element.walkUp(target)
        .map(e => [...e.classList])
        .flat()
        .indexOf('ruler-container') != -1
    )
      return;

    //console.log('wheel:', { deltaY, deltaMode, wheelDelta, target });

    if(!pos.inside(clientArea)) return;

    const dy = Math.sign(event.deltaY) * 53;

    const wheelPos = -dy.toFixed(2);
    let zoomVal = config.zoomLog();

    zoomVal = altKey || ctrlKey || shiftKey ? 0 : clamp(-100, 300, zoomVal + wheelPos * 0.1);
    config.zoomLog(zoomVal);
    AdjustZoom();
  });

  //console.error('AppMain done');

  //console.log(globalThis);

  /*  for(let path of [...Element.findAll('path')]) {
    let points = new PointList([...SVG.pathIterator(path, 30, p => p.toFixed(3))]);
  }*/
});
const Module = {
  noInitialRun: true,
  onRuntimeInitialized: () => {
    //console.log('initialized');
    let myString = prompt('Enter a string:');
    Module.callMain([myString]);
  },
  print: txt => alert(`The MD5 hash is: ${txt}`)
};
