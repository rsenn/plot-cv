// prettier-ignore-start
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom from './lib/dom.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { iterator, eventIterator } from './lib/dom/iterator.js';
import keysim from './lib/dom/keysim.js';
import geom, { BBox, Polygon, Circle, LineList } from './lib/geom.js';
import { TouchListener } from './lib/touchHandler.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { ClipperLib } from './lib/clipper-lib.js';
import Shape from './lib/clipper.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tlite from './lib/tlite.js';
import { debounceAsync } from './lib/async/debounce.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Alea from './lib/alea.js';
import path from './lib/path.js';
import Timers, { TimeoutError } from './lib/repeater/timers.js';
import asyncHelpers from './lib/async/helpers.js';
import { Cache } from './lib/dom/cache.js';
import { CacheStorage } from './lib/dom/cacheStorage.js';
/* prettier-ignore */ import { InterpretGcode, gcodetogeometry, GcodeObject, gcodeToObject, objectToGcode, parseGcode, GcodeParser, GCodeLineStream, parseStream, parseFile, parseFileSync, parseString, parseStringSync, noop, Interpreter } from './lib/gcode.js';
import { Iterator } from './lib/iterator.js';
import { Functional } from './lib/functional.js';
import { makeLocalStorage } from './lib/autoStore.js';
import { Repeater } from './lib/repeater/repeater.js';
import { useResult } from './lib/repeater/react-hooks.js';
import LogJS from './lib/log.js';
import { useDimensions } from './useDimensions.js';
import { toXML, ImmutablePath, arrayDiff, objectDiff } from './lib/json.js';
import { XmlObject, XmlAttr, ImmutableXPath } from './lib/xml.js';
import { RGBA, isRGBA, ImmutableRGBA, HSLA, isHSLA, ImmutableHSLA, ColoredText } from './lib/color.js';
//import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import components, { Chooser, DynamicLabel, Button, FileList, Panel, SizedAspectRatioBox, TransformedElement, Canvas, ColorWheel, Slider, CrossHair, FloatingPanel, DropDown, Conditional } from './components.js';
import { Message } from './message.js';
import { WebSocketClient } from './lib/net/websocket-async.js';
/* prettier-ignore */ import { CTORS, ECMAScriptParser, estree, Factory, Lexer, Position, Range, ESNode, Parser, PathReplacer, Printer, Stack, Token, ArrayBindingPattern, ArrayLiteral, ArrowFunction, AssignmentExpression, AwaitExpression, BinaryExpression, BindingPattern, BindingProperty, LabelledStatement, BlockStatement, BreakStatement, CallExpression, ClassDeclaration, ConditionalExpression, ContinueStatement, Declaration, DecoratorExpression, DoStatement, EmptyStatement, Expression, ExpressionStatement, ForInStatement, ForStatement, FunctionLiteral, FunctionDeclaration, Identifier, ComputedPropertyName, IfStatement, SwitchStatement, CaseClause, ImportStatement, ExportStatement, JSXLiteral, Literal, TemplateLiteral, LogicalExpression, MemberExpression, InExpression, NewExpression, ObjectBindingPattern, ObjectLiteral, PropertyDefinition, MemberVariable, Program, RestOfExpression, ReturnStatement, SequenceExpression, SpreadElement, Statement, StatementList, ThisExpression, ThrowStatement, YieldStatement, TryStatement, UnaryExpression, UpdateExpression, VariableDeclaration, VariableDeclarator, WhileStatement, WithStatement } from './lib/ecmascript.js';
import {
  PipeTo,
  AsyncRead,
  AsyncWrite,
  AcquireReader,
  AcquireWriter,
  DebugTransformStream,
  TextEncodeTransformer,
  TextEncoderStream,
  TextDecodeTransformer,
  TextDecoderStream,
  TransformStreamSink,
  TransformStreamSource,
  TransformStreamDefaultController,
  TransformStream,
  ArrayWriter,
  readStream,
  WriteToRepeater,
  LogSink,
  RepeaterSink,
  StringReader,
  LineReader,
  ChunkReader,
  ByteReader,
  PipeToRepeater,
  WritableStream
} from './lib/stream.js?ts=<?TS?>';
import { PrimitiveComponents, ElementNameToComponent, ElementToComponent } from './lib/eagle/components.js';
import { SVGAlignments, AlignmentAttrs, Alignment, AlignmentAngle, Arc, CalculateArcRadius, ClampAngle, EagleAlignments, HORIZONTAL, HORIZONTAL_VERTICAL, InvertY, LayerAttributes, LinesToPath, MakeCoordTransformer, PolarToCartesian, RotateTransformation, VERTICAL, useTrkl } from './lib/eagle/renderUtils.js';
import { Wire } from './lib/eagle/components/wire.js';
import { Instance } from './lib/eagle/components/instance.js';
import { SchematicSymbol } from './lib/eagle/components/symbol.js';
import { Emitter, EventIterator } from './events.js';
import { Slot, SlotProvider } from './slots.js';
import Voronoi from './lib/geom/voronoi.js';
import GerberParser from './lib/gerber/parser.js';
import { lazyInitializer } from './lib/lazyInitializer.js';
import { BoardRenderer, DereferenceError, EagleDocument, EagleElement, EagleNode, EagleNodeList, EagleNodeMap, EagleProject, EagleRef, EagleReference, EagleSVGRenderer, Renderer, SchematicRenderer, makeEagleElement, makeEagleNode } from './lib/eagle.js';
//import PureCache from 'pure-cache';
import { brcache, lscache, BaseCache, CachedFetch } from './lib/lscache.js'; //const React = {Component, Fragment, create: h, html, render, useLayoutEffect, useRef, useState };
import { NormalizeResponse, ResponseData, FetchURL, FetchCached, GetProject, ListProjects, GetLayer, AddLayer, BoardToGerber, GerberToGcode, GcodeToPolylines, ListGithubRepo, ListGithubRepoServer } from './commands.js';
// prettier-ignore-end

/* prettier-ignore */ const { Align, Anchor, CSS, Event, CSSTransformSetters, Element, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementTransformation, ElementWHProps, ElementXYProps, isElement, isLine, isMatrix, isNumber, isPoint, isRect, isSize, Line,Matrix, Node, Point, PointList, Polyline, Rect, Select, Size, SVG, Transition, TransitionList, TRBL, Tree } = { ...dom, ...geom };

import { classNames } from './lib/classNames.js';

Util.colorCtor = ColoredText;
const elementDefaultAttributes = { stroke: 'red', fill: 'none', 'stroke-linecap': 'round', 'stroke-linejoin': 'round', 'stroke-width': 0.1 };

/* prettier-ignore */
//Util.extend(window, { React, ReactComponent, WebSocketClient, html }, { dom, keysim }, geom, { Iterator, Functional }, { EagleNodeList, EagleNodeMap, EagleDocument, EagleReference, EagleNode, EagleElement }, { toXML, XmlObject, XmlAttr }, { CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token, ReactComponent, ClipperLib, Shape, isRGBA, RGBA, ImmutableRGBA, isHSLA, HSLA, ImmutableHSLA, ColoredText, Alea, Message }, { Chooser, useState, useLayoutEffect, useRef, Polygon, Circle } );
const Timer = { ...Timers, once: dom.Timer };

let currentProj = trkl.property(window, 'project');
let layerList = trkl.property(window, 'layers', { value: [] });
let gcode = trkl(null);

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

let store = (window.store = makeLocalStorage());

let projects = trkl([]);
let socket = trkl();
let listURL = trkl(store.get('url') || null);
let searchFilter = trkl(store.get('filter') || '*');
let zoomLog = trkl(store.get('zoom') || null);
let logSize = trkl(store.get('console') || null);
let debugFlag = trkl(store.get('debug') || false);
let elementChildren = null;
let elementGeometries = null;
let showGrid;

const add = (arr, ...items) => [...(arr ? arr : []), ...items];

const useSlot = (arr, i) => [() => arr[i], v => (arr[i] = v)];
const trklGetSet = (get, set) => value => (value !== undefined ? set(value) : get());
//const useTrkl = trkl => [() => trkl(), value => trkl(value)];

const MouseEvents = h => ({
  onMouseDown: h,

  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h
});

tlite(() => ({ grav: 'nw', attrib: ['data-tlite', 'data-tooltip', 'title', 'data-filename'] }));

const utf8Decoder = new TextDecoder('utf-8');
let svgOwner, parent;

const svgFactory = lazyInitializer(() => {
  parent = project.svg.parentElement;
  const factory = SVG.factory({
    append_to(elem, p) {
      (p || parent).appendChild(elem);
    }
  });
  let rect = DrawSVG.calcViewBox();
  const svg = [
    'svg',
    { viewBox: rect.toString(), style: 'position: absolute; left: 0; top: 0;' },
    [
      ['defs'],
      [
        'g',
        {
          transform: ` scale(1,-1) translate(0,1.27) translate(0,${-rect.y2})
       `
        }, [['rect', { ...rect.toObject(), fill: 'hsla(0,0%,50%,0.3333)' }]]
      ]
    ]
  ];
  const element = (svgOwner = factory(...svg));
  factory.root = parent = element.lastElementChild;
  return factory;
});

const DrawSVG = (...args) => {
  const factory = svgFactory();
  let e;
  try {
    let parent = project.svg.parentElement.lastElementChild;
    const append = e => parent.appendChild(e);
    let c = RGBA.random();
    let [tag, attrs, children] = args;
    if(typeof tag == 'string') {
      // console.log('draw(', ...args, ')');
      e = factory(tag, { stroke: c.hex(), 'stroke-width': 0.1, ...attrs }, children);
    } else if(Util.isArray(args[0])) {
      let items = args.shift();
      document.querySelector('#main > div > div > div > svg:nth-child(2) > g');
      DrawSVG.setViewBox(BBox.from(items));
      for(let item of items) {
        let line;
        if(isLine(item)) line = new Line(item);
        if(line) {
          e = factory('line', { ...line.toObject(), stroke: c.hex(), 'stroke-width': 0.1 });
          append(e);
          //   console.log('e:', e);
        }
      }
      return;
    }
    if(e) append(e);
  } catch(error) {
    Util.putError(error);
    throw error;
  }
  return e;
  function calcViewBox(box) {
    box = box || (project && project.doc && BBox.from(project.doc.getMeasures(true)));
    box = box || Element.rect('.aspect-ratio-box-inside');
    const { width, height, x, y } = box;
    let { x1, y1, x2, y2 } = new Rect(x, y, width, height);
    const rect = new BBox(x1, y1 - y2, x2 - x1, y2);
    return rect;
  }
  function setViewBox(box) {
    svgOwner = svgOwner || [...Element.findAll('svg', Element.find('#main'))].reverse()[0];
    const rect = box; // instanceof BBox ? box : DrawSVG.calcViewBox(box);
    rect.y1 -= rect.y2;
    rect.x2 -= rect.x1;
    // console.log('setViewBox', { svgOwner, rect, box });
    svgOwner.setAttribute('viewBox', rect.toString());
    svgOwner.lastElementChild.setAttribute('transform', `scale(1,-1)  translate(0,${-rect.height})`);
    Element.attr(svgOwner.lastElementChild.firstElementChild, { ...rect.toRect() });
  }
};

const ElementToXML = (e, predicate) => {
  const x = Element.toObject(e, { predicate });
  //console.log('x:', x);
  return Element.toString(x, { newline: '\n' });
};

const FileSystem = {
  async readFile(filename) {
    return await FetchURL(`/static/${filename}`);
  },
  async writeFile(filename, data, overwrite = true) {
    return await fetch('/save', {
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

const LoadFile = async file => {
  let { url, name: filename } = typeof file == 'string' ? { url: file, name: file.replace(/.*\//g, '') } : GetProject(file);
  LogJS.info(`LoadFile ${url}`);
  url = /:\/\//.test(url) ? url : /^tmp\//.test(url) ? '/' + url : `/static/${filename}`;
  //console.log('LoadFile url=', url);
  let response = await FetchURL(url);
  // console.debug('LoadFile response=', response);
  let xml = await response.text();
  let doc = new EagleDocument(await xml, null, filename, null, FileSystem);
  if(/\.brd$/.test(filename)) window.board = doc;
  if(/\.sch$/.test(filename)) window.schematic = doc;
  if(/\.lbr$/.test(filename)) window.libraries = add(window.libraries, doc);
  LogJS.info('LoadFile', doc.file);
  return doc;
};

const SaveFile = async (filename, data, contentType) => {
  if(!data.endsWith('\n')) data += '\n';
  let { status, statusText, body } = await fetch('/save', {
    method: 'post',
    headers: {
      'Content-Type': contentType || 'application/xml',
      'Content-Disposition': `attachment; filename="${filename}"`
    },
    body: data
  });
  const result = { status, statusText, body };
  LogJS.info(`${filename} saved.`);
  return result;
};

const SaveSVG = async function save(filename, layers = [1, 16, 20, 21, 22, 23, 25, 27, 47, 48, 51]) {
  const { doc } = project;
  const { basename, typeName } = doc;
  if(!filename) filename = `${doc.basename}.${doc.typeName}.svg`;
  let predicate = element => {
    if(!element.hasAttribute('data-layer')) return true;
    const layer = element.getAttribute('data-layer');
    let [number, name] = layer.split(/\ /);
    if(number !== undefined && name !== undefined) return layers.indexOf(+number) != -1 || layers.indexOf(name) != -1;
    return true;
  };
  let data = ElementToXML(project.svg, predicate);
  return await SaveFile(filename.replace(/\.svg$/i, '.svg'), data);
};

const ModifyColors = fn => e => {
  const { type, buttons } = e;
  if(type.endsWith('down')) {
    if(!window.c) window.c = SVG.allColors(project.svg);
    let { c } = window;
    c.dump();
    fn(c);
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

const LoadDocument = async (project, parentElem) => {
  //console.log('project:', project);
  open(false);
  gcode(null);

  try {
    project.doc = await LoadFile(project);
  } catch(error) {
    console.error(error);
    throw error;
  }
  LogJS.info(`${project.doc.basename} loaded.`);
  const topPlace = 'tPlace';
  elementChildren = Util.memoize(() => ElementChildren(topPlace, ent => Object.fromEntries(ent)));
  elementGeometries = Util.memoize(() => ElementGeometries(topPlace, ent => Object.fromEntries(ent)));
  //polygonGeometries = Util.memoize(() => Object.entries(elementGeometries()).map(([name, lineList]) => [name, lineList.toPolygon((pts) => new Polyline(pts))]));

  documentTitle(project.doc.file.replace(/.*\//g, ''));
  let s = project.doc.dimensions;

  documentSize(s.toString({ unit: 'mm' }));
  window.eagle = project.doc;
  window.project = project;
  Element.remove('#fence');
  let docElem = Element.find('#doc');
  docElem.innerHTML = '';
  console.log('project.doc:', project.doc.basename);
  project.renderer = new Renderer(project.doc, ReactComponent.append, debug);

  showGrid = trkl(true);
  showGrid.subscribe(value => {
    let obj = { ...project.renderer.grid, visible: value };
    console.log('showGrid:', obj);
    project.renderer.grid = obj;
  });

  console.log('project.renderer', project.renderer);
  let style = { width: '100%', height: '100%', position: 'relative' };
  let component = project.renderer.render(project.doc, null, {});
  let usedLayers = [...project.doc.layers.list].filter(layer => layer.elements.size > 0);

  Timer.once(250).then(() =>
    layerList(usedLayers.map(layer => ({
        i: layer.number,
        name: layer.name,
        element: layer,
        visible: layer.handlers.visible
      }))
    )
  );
  LogJS.info(`${project.name} rendered.`);
  window.component = project.component = component;
  //console.debug('testRender:', component);
  let element = Element.find('#main');
  let r = project.renderer.rect || project.renderer.bounds;
  //console.debug('project.renderer:', project.renderer);
  //console.debug('r:', r);
  let aspectRatio = 1;
  if(r) {
    aspectRatio = r.width / r.height;
    sizeListener({ width: r.width });
  }
  aspectListener(aspectRatio);
  //console.debug('aspectRatio:', aspectRatio);
  const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
    const [dimensions, setDimensions] = useState(sizeListener());
    const [aspect, setAspect] = useState(aspectListener());
    if(sizeListener && sizeListener.subscribe) sizeListener.subscribe(value => setDimensions(value));
    if(aspectListener && aspectListener.subscribe) aspectListener.subscribe(value => setAspect(value));
    console.debug('Fence dimensions:', dimensions);
    return h(TransformedElement,
      {
        id: 'fence',
        type: SizedAspectRatioBox,
        aspect,
        listener: transform,
        style: {
          position: 'relative',
          minWidth: '100px',
          'data-name': project.name,
          ...style,
          ...dimensions
        },
        ...props
      },
      children
    );
  };
  component = h(Fence,
    {
      style: {},
      sizeListener,
      aspectListener
    }, [component]
  );

  React.render(component, element);

  let object = ReactComponent.toObject(component);
  project.object = object;
  let rendered = object.children[0];

  //console.debug('rendered:', rendered);

  let eagle2dom = [...Element.findAll('*[data-path]')];

  eagle2dom = eagle2dom.map(e => [e.getAttribute('data-path'), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [new ImmutablePath(p), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [p, p.apply(project.doc.raw), e]);
  eagle2dom = eagle2dom.map(([p, r, e]) => [EagleElement.get(project.doc, p, r), e]);

  //console.debug('eagle2dom:', eagle2dom);

  let dom2eagle = Util.mapFunction(new WeakMap(eagle2dom.map(([k, v]) => [v, k])));

  eagle2dom = Util.mapFunction(new WeakMap(eagle2dom));
  //console.debug('eagle2dom:', eagle2dom);

  const [path2component, component2path] = project.renderer.maps.map(Util.mapFunction);
  const { path2obj, obj2path, path2eagle, eagle2path, eagle2obj, obj2eagle } = project.doc.maps;

  const [component2eagle, eagle2component] = [Util.mapAdapter((key, value) => (value === undefined ? path2eagle(component2path(key)) : undefined)), Util.mapAdapter((key, value) => (value === undefined ? path2component(eagle2path(key)) : undefined))];

  const [component2dom, dom2component] = [Util.mapAdapter((key, value) => (value === undefined ? eagle2dom(component2eagle(key)) : undefined)), Util.mapAdapter((key, value) => (value === undefined ? eagle2component(dom2eagle(key)) : undefined))];

  //path2eagle: path2obj, eagle2path: obj2path

  project.maps = {
    ...project.doc.maps,
    path2component,
    component2path,
    dom2eagle,
    eagle2dom,
    component2eagle,
    eagle2component,
    component2dom,
    dom2component
  };

  project.rendered = rendered;
  project.element = element;
  project.svg = Element.find('svg', '#main');
  project.grid = Element.find('g.grid', project.element);
  project.bbox = SVG.bbox(project.grid);
  project.aspectRatio = aspect;

  let center = SVG.bbox(project.svg).center.round();
  let defaultTransform = `translate(${center.x},${center.y}) scale(2.54,2.54)`;

  function xx() {
    let g = SVG.create('g', {});

    project.svg.appendChild(g);
    let ll = geometries.R4.lines.toSVG(ReactComponent.append, () => h('g', { ...elementDefaultAttributes, defaultTransform }));

    render(ll, g);
  }
  xx();
  window.AddElement = (function (transform) {
    const root = project.svg;

    let list = [];

    return (tag, attr, children = []) => {
      let e = SVG.create(tag, { ...elementDefaultAttributes, transform, ...attr }, root);
      list.push(e);
      let d = trkl.property(e, 'd');
      d.subscribe(value => e.setAttribute('d', value));
      return e;
    };
  })(defaultTransform);

  let { name, data, doc, svg, bbox } = project;
  let bounds = doc.getBounds();
  let rect = bounds.toRect(Rect.prototype);
  let size = new Size(r);
  currentProj(project);
  size.mul(doc.type == 'brd' ? 2 : 1.5);
  let svgrect = SVG.bbox(project.svg);
  let measures = doc.measures.rect;
  //console.debug('measures:', measures);
  Element.attr(project.svg, { 'data-filename': project.name, 'data-aspect': project.aspectRatio });
  let css = size.div(0.26458333333719).toCSS({ width: 'px', height: 'px' });
  window.size = css;
  AdjustZoom();
  project.status = SaveSVG();
  return project;
};

const ChooseDocument = async (project, i) => {
  let r;
  if(i == undefined) i = project.i || projectFiles.indexOf(project);
  const box = Element.findAll('.file')[i];
  LogJS.info('ChooseDocument:', { project, i, box });
  LogJS.info(`${project.name} selected.`);
  try {
    if(!project.loaded) {
      let data = await LoadDocument(project, box);
      project.loaded = true;
      //console.log('loaded:', project);
    }
    r = project.loaded;
  } catch(err) {
    Util.putError(err);
    console.error(err);
  }
  return r;
};

/* gerber=await BoardToGerber(project.name); gc=await GerberToGcode('tmp/7seg-2.54.GBL'); geom=gcodetogeometry(gc.data);lines = geom.lines.map(({start,end}) => new Line(start,end)) */

const GenerateVoronoi = () => {
  //console.log('Loading document: ' + filename);
  let { doc } = project;
  console.log('doc', doc);
  let points = new PointList();
  for(let element of doc.elements.list) {
    const pkg = element.package;
    let { x, y } = element;
    console.log('element:', element, { x, y });
    let origin = new Point(x, y);
    for(let item of pkg.children) {
      if(item.drill !== undefined) {
        let pos = new Point(+item.x, +item.y).add(origin);
        console.log('pos:', pos);
        points.push(pos);
      }
    }
  }
  let bb = doc.getBounds();
  let rect = bb.toRect(Rect.prototype);
  console.log('bb:', bb);
  console.log('rect:', rect);
  rect.outset(1.27);
  window.tmprect = rect;
  let sites = points.map(p => p.toObject());
  let bbox = { xl: bb.x1, xr: bb.x2, yt: bb.y1, yb: bb.y2 };
  let voronoi = new Voronoi();
  //pass an object which exhibits xl, xr, yt, yb properties. The bounding
  //box will be used to connect unbound edges, and to close open cells
  let result = voronoi.compute(sites, bbox);
  //render, further analyze, etc.
  console.log('result:', Object.keys(result).join(', '));
  let { site, cells, edges, vertices, execTime } = result;
  console.log('cells:', cells);
  let holes = edges.filter(e => !e.rSite).map(({ lSite, rSite, ...edge }) => new Point(lSite));
  let rlines = edges.filter(e => e.rSite).map(({ lSite, rSite, ...edge }) => new Line(lSite, rSite));
  let vlines = edges.filter(e => e.va && e.vb).map(({ va, vb, ...edge }) => new Line(va, vb).round(0.127, 4));
  let points2 = vertices.map(v => new Point(v).round(0.127, 4));
  const add = (arr, ...items) => [...(Util.isArray(arr) ? arr : []), ...items];
  const factory = SVG.factory();
  const lines = [...rlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#000', 'stroke-width': 0.01 }]), ...vlines.map(l => ['line', { ...l.toObject(t => t + ''), stroke: '#f00', 'stroke-width': 0.01 }])];
  const circles = [
    ...holes.map(p => ['circle', { cx: p.x, cy: p.y, r: 0.254, fill: 'none', stroke: '#00f', 'stroke-width': 0.3 }])

    /* ...points2.map(p => [
      'circle',
      { cx: p.x, cy: p.y, r: 0.254 * 2, fill: 'none', stroke: 'rgba(0,255,255,0.75)', 'stroke-width': 0.1 }
    ])*/
  ];
  const polylines = [
    ...cells.reduce((acc, { site, halfedges }) => [
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
  console.log('polylines:', polylines);
  console.log('cells:', cells);
  window.cells = cells;
  Element.setCSS(svgElem, { position: 'absolute', left: 0, top: 0, width: '100%', height: 'auto' });
  //filesystem.writeFile('output.svg', svgFile);
  console.log('svg:', svgElem);
};

function PackageChildren(element, layer) {
  let children = [...element.children].map((c, i) => [i, c]).filter(([i, p]) => p.layer && p.layer.name == 'tPlace' && p.tagName == 'wire');
  children.xml = children.map(([i, e]) => e.toXML()).join('\n');
  return children;
}
function ElementChildren(layer = 'tPlace', rfn = ent => new Map(ent)) {
  return rfn([...project.doc.elements].map(([name, element]) => [name, PackageChildren(element, layer)]));
}

function ElementGeometries(layer = 'tPlace', rfn = ent => new Map(ent)) {
  return rfn(ElementChildren(layer, ent => ent)
      .map(([name, children]) => [
        name,
        new LineList(children.map(([i, e]) => {
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
  project.svg.appendChild(elem);
}

const MakeFitAction = index => async event => {
  // window.transform='';
  const { buttons, type, target } = event;

  if(!type.endsWith('down') || buttons == 0) return false;
  console.debug(`FitAct(${index})`, { buttons, type, target });

  let container = Element.find('.transformed-element-size');
  let oldSize = Element.rect(container);
  let topBar = Element.rect('.buttons');
  let clientArea = Element.rect('body');
  clientArea.y1 += topBar.height;
  const zoom = ZoomFactor();
  // console.debug(`FitA
  let f = oldSize.fit(clientArea);
  // console.debug(`FitAction(${index})`,  { oldSize, clientArea }, "\n", ...f);

  let factors = new Size(oldSize).fitFactors(new Size(clientArea));
  let factor = factors[index];
  let oldTransform = new TransformationList(window.transform);
  console.debug(`FitAction(${index})`, { factor, oldTransform });
  let oldScaling = oldTransform.scaling;
  let newTransform = oldTransform.slice(0, 1);

  if(!newTransform.scaling) newTransform = newTransform.scale(factor, factor);
  else {
    newTransform.scaling.x *= factor;
    newTransform.scaling.y *= factor;
  }

  console.debug(`FitAction(${index})`, { oldScaling, newTransform });

  Element.setCSS(container, { transform: newTransform });

  let newSize = Element.rect(container);
  let size = new Rect(newSize).align(clientArea);
  console.debug(`FitAction <->`, oldSize, ' -> ', size);

  let points = [size, newSize].map(s => new Point(s));
  let delta = Point.diff(...points);
  console.debug(`FitAction <->`, ...points);
  console.debug(`FitAction -`, Point.diff(...points));
  let newScaling = newTransform.scaling;

  if(newTransform.translation) {
    newTransform.translation.x += delta.x;
    newTransform.translation.y += delta.y;
  } else newTransform = newTransform.translate(delta.x / newScaling.x, delta.y / newScaling.y, 'px');
  console.debug(`FitAction newTransform=`, newTransform, newTransform + '');
  //  newTransform =
  AdjustZoom(ZoomLog(newTransform.scaling.x));

  window.transform = newTransform.toString('px', 'deg');
};

function ZoomFactor(val = zoomLog()) {
  return Math.pow(10, val / 200).toFixed(5);
}

function ZoomLog(factor) {
  return Math.log10(factor) * 200;
}

function AdjustZoom(l = zoomLog()) {
  let zoomFactor = ZoomFactor(l);
  let t = new TransformationList(window.transform);
  if(!t.scaling) t.scale(zoomFactor, zoomFactor);
  else {
    t.scaling.x = zoomFactor;
    t.scaling.y = zoomFactor;
  }
  window.transform = t;
}

const CreateWebSocket = async (socketURL, log, socketFn = () => {}) => {
  // log = log || ((...args) => console.log(...args));
  socketURL = socketURL || Util.makeURL({ location: '/ws', protocol: 'ws' });
  let ws = new WebSocketClient();
  let send = ws.send;
  ws.send = (...args) => {
    let [msg] = args;
    if(!(msg instanceof Message)) msg = new Message(...args);
    return send.call(ws, msg.data);
  };
  window.socket = ws;
  LogJS.info('New WebSocket:', ws);
  await ws.connect(socketURL);
  LogJS.info('WebSocket Connected:', ws.connected);
  socketFn(ws);
  ws.send('main.js data!');
  let data;
  for await (data of ws) {
    let msg = new Message(data);
    window.msg = msg;
    LogJS.info('WebSocket recv: ' + Util.toString(msg)); //fmsg[Symbol.toStringTag]());
    ws.dataAvailable !== 0;
  }
  await ws.disconnect();
};

const BindGlobal = Util.once(arg => trkl.bind(window, arg));

const AppMain = (window.onload = async () => {
  Util(globalThis);
  //prettier-ignore
  const imports = {Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList, dom, ReactComponent, iterator, eventIterator, keysim, geom, BBox, LineList, Polygon, Circle, TouchListener, trkl, ColorMap, ClipperLib, Shape, devtools, Util, tlite, debounceAsync, tXml, deep, Alea, path, TimeoutError, Timers, asyncHelpers, Cache, CacheStorage, InterpretGcode, gcodetogeometry, GcodeObject, gcodeToObject, objectToGcode, parseGcode, GcodeParser, GCodeLineStream, parseStream, parseFile, parseFileSync, parseString, parseStringSync, noop, Interpreter, Iterator, Functional, makeLocalStorage, Repeater, useResult, LogJS, useDimensions, toXML, ImmutablePath, arrayDiff, objectDiff, XmlObject, XmlAttr, ImmutableXPath, RGBA, isRGBA, ImmutableRGBA, HSLA, isHSLA, ImmutableHSLA, ColoredText, React, h, html, render, Fragment, Component, useState, useLayoutEffect, useRef, components, Chooser, DynamicLabel, Button, FileList, Panel, SizedAspectRatioBox, TransformedElement, Canvas, ColorWheel, Slider, CrossHair, FloatingPanel, DropDown, Conditional, Message, WebSocketClient, CTORS, ECMAScriptParser, estree, Factory, Lexer, Position, Range, Factory, ESNode, ArrayBindingPattern, ArrayLiteral, ArrowFunction, AssignmentExpression, AwaitExpression, BinaryExpression, BindingPattern, BindingProperty, LabelledStatement, BlockStatement, BreakStatement, CallExpression, ClassDeclaration, ConditionalExpression, ContinueStatement, Declaration, DecoratorExpression, DoStatement, EmptyStatement, Expression, ExpressionStatement, ForInStatement, ForStatement, FunctionLiteral, FunctionDeclaration, Identifier, ComputedPropertyName, IfStatement, SwitchStatement, CaseClause, ImportStatement, ExportStatement, JSXLiteral, Literal, TemplateLiteral, LogicalExpression, MemberExpression, InExpression, NewExpression, ObjectBindingPattern, ObjectLiteral, PropertyDefinition, MemberVariable, Program, RestOfExpression, ReturnStatement, SequenceExpression, SpreadElement, Statement, StatementList, ThisExpression, ThrowStatement, YieldStatement, TryStatement, UnaryExpression, UpdateExpression, VariableDeclaration, VariableDeclarator, WhileStatement, WithStatement, Parser, PathReplacer, Printer, Stack, Token, PipeTo, AsyncRead, AsyncWrite, AcquireReader, AcquireWriter, DebugTransformStream, TextEncodeTransformer, TextEncoderStream, TextDecodeTransformer, TextDecoderStream, TransformStreamSink, TransformStreamSource, TransformStreamDefaultController, TransformStream, ArrayWriter, readStream, WriteToRepeater, LogSink, RepeaterSink, StringReader, LineReader, ChunkReader, ByteReader, PipeToRepeater, WritableStream, PrimitiveComponents, ElementNameToComponent, ElementToComponent, SVGAlignments, AlignmentAttrs, Alignment, AlignmentAngle, Arc, CalculateArcRadius, ClampAngle, EagleAlignments, HORIZONTAL, HORIZONTAL_VERTICAL, InvertY, LayerAttributes, LinesToPath, MakeCoordTransformer, PolarToCartesian, RotateTransformation, VERTICAL, useTrkl, Wire, Instance, SchematicSymbol, Emitter, EventIterator, Slot, SlotProvider, Voronoi, GerberParser, lazyInitializer, BoardRenderer, DereferenceError, EagleDocument, EagleElement, EagleNode, EagleNodeList, EagleNodeMap, EagleProject, EagleRef, EagleReference, EagleSVGRenderer, Renderer, SchematicRenderer, makeEagleElement, makeEagleNode, brcache, lscache, BaseCache, CachedFetch, NormalizeResponse, ResponseData, FetchURL, FetchCached, GetProject, ListProjects, GetLayer, AddLayer, BoardToGerber, GerberToGcode, GcodeToPolylines, ListGithubRepo, ListGithubRepoServer, classNames };
  const localFunctions = { PackageChildren, ElementChildren, Timer, MouseEvents, DrawSVG, ElementToXML, FileSystem, LoadFile, SaveFile, SaveSVG, ModifyColors, GerberLayers, LoadDocument, ChooseDocument, GenerateVoronoi, MakeFitAction, CreateWebSocket, BindGlobal, AppMain };

  const importedNames = Object.keys(imports);
  console.debug('Dupes:',
    Util.getMemberNames(window).filter(m => importedNames.indexOf(m) != -1)
  );

  //prettier-ignore
  Util.weakAssign(window,geom);
  Util.weakAssign(window, imports);
  Util.weakAssign(window, localFunctions);
  Error.stackTraceLimit = 100;

  const timestamps = new Repeater(async (push, stop) => {
    push(Date.now());
    const iRnterval = setInterval(() => push(Date.now()), 1000);
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

  window.keystroke = target => (key, modifiers = 0) => keysim.Keyboard.US_ENGLISH.dispatchEventsForKeystroke(new keysim.Keystroke(modifiers, key), target);

  window.focusSearch = state => {
    const input = currentSearch();
    //console.log('focusSearch', input.tagName, state);
    input[state ? 'focus' : 'blur']();
  };

  // prettier-ignore
  BindGlobal({ projects, socket, transform, size: sizeListener, aspect: aspectListener, showSearch, logDimensions: logSize, watched: dump, 
    children: () => elementChildren(),
     geometries: () => elementGeometries(),
     debug: debugFlag });

  currentSearch.subscribe(value => {
    if(value) {
      focusSearch(false);
      Timer.once(1000).then(() => focusSearch(true));
    }
  });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = props => html` <div>This is a test</div> `;

  let c = testComponent({});
  window.testComponent = c;
  let credentials = { username: 'rsenn', password: 'tjIDznHp9' };

  const UpdateProjectList = async (opts = listURL() ? { url: listURL(), ...credentials } : {}) => {
    let list = [];
    //console.log('opts:', opts);
    let { url, ...restOfOpts } = opts;
    let urls = url ? url.split(/\n/g) : [null];
    for(url of urls) {
      let data = await ListProjects({ ...opts, url });
      let { files } = data;
      //console.log(`Got ${files.length} files`, files);
      function File(obj, i) {
        const { name } = obj;
        let file = this instanceof File ? this : Object.create(File.prototype);
        let data = trkl({ percent: NaN });
        Object.assign(file, obj);
        file.name = name;
        file.i = i;
        trkl.bind(file, { data });
        LogJS.info(`Got file '${name.replace(/.*:\/\//g, '').replace(/raw.githubusercontent.com/, 'github.com') || name.replace(/.*\//g, '')}'`);

        return file;
      }
      File.prototype.toString = function() {
        return this.name;
      };
      list = list.concat(files.sort((a, b) => a.name.localeCompare(b.name)).map((obj, i) => new File(obj, i)));
      let svgs = list.reduce((acc, file) => {
        if(/\.lbr$/i.test(file.name)) return acc;
        file.svg = `${EagleDocument.baseOf(file.name)}.${EagleDocument.typeOf(file.name)}.svg`;
        //console.log(`file.svg = '${file.svg}'`);
        return [...acc, file.svg];
      }, []);

      data = await ListProjects({ descriptions: false, names: svgs });
      files = (data && data.files) || [];
      //      console.log('filesData:', files);

      for(let svgFile of files) {
        if(Util.isObject(svgFile) && svgFile.mtime !== undefined) {
          const f = list.find(i => i.svg === svgFile.name);
          if(Util.isObject(f) && f.mtime !== undefined) {
            const delta = svgFile.mtime - f.mtime;

            f.modified = delta < 0;
          }
        }
      }
    }

    LogJS.info(`retrieved project list. Got ${list.length} items.`);

    projects(list);
  };

  UpdateProjectList();
  CreateWebSocket(null, null, ws => (window.socket = ws));

  const crosshair = {
    show: trkl(false),
    position: trkl({ x: 0, y: 0 })
  };

  window.crosshair = trkl.bind({}, crosshair);

  searchFilter.subscribe(value => {
    store.set('filter', value);
    LogJS.info(`searchFilter is ${value}`);
  });

  listURL.subscribe(value => {
    store.set('url', value);
    LogJS.info(`listURL is '${value}'`);
  });
  debugFlag.subscribe(value => store.set('debug', value));

  logSize.subscribe(value => {
    const { width, height } = value;

    if(width === undefined || height === undefined) {
      throw new Error('logSize undefined');
    }
    store.set('console', value);
    //LogJS.info(`logSize is ${value.width} x ${value.height}`);
  });

  trkl.bind(window, { searchFilter, listURL });
  trkl.bind(window, { svgFactory });

  trkl.bind(window, { zoomLog, logSize });

  zoomLog.subscribe(value => {
    let factor = ZoomFactor(value);
    // console.info('zoomFactor changed', value, factor);
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

    updateIfChanged(listURL, urls, () => {});
    listURL(urls);

    //    value = parts.filter(p => !/\:\/\//.test(p)).join(' ');

    searchFilter(value == '' ? '*' : value.split(/\s*\|\s*/g).join(' | '));
  };

  const Consumer = props => {
    const result = useResult(async function* () {
      for await (let time of timestamps) {
        yield time;
      }
    });
    return h('div',
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
      }, [result && new Date(result.value).toLocaleTimeString('de-CH')]
    );
  };
  LogJS.addAppender(class extends LogJS.BaseAppender {
      log(type, time, msg) {
        let d = new Date(time);
        if(typeof window.pushlog == 'function') window.pushlog([type, Util.isoDate(d).replace(/-/g, ''), d.toLocaleTimeString(navigator.language || 'de'), msg]);
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
      for await (let msg of logger) yield msg;
    });
    if(result) {
      lines.push(result.value);
    }
    return h('table',
      { className: 'logger', ref },
      lines.slice(-100, lines.length).map(([type, d, t, m], i) =>
        h('tr', {}, [
          h('td', { className: 'log sign' },
            h('img', {
              className: 'log sign',
              src: `/static/${type.toLowerCase() || 'warn'}.svg`,
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
    return h('table',
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
    return h('input',
      {
        type: 'text',
        className: 'commander',
        value: inputText,
        onKeyDown: handler,
        autofocus: true
      }, []
    );
  };

  const layersDropDown = trkl(false);

  const Toggle = trkl => trkl(!trkl());
  let setTo;

  const Layer = ({ title, name, label, i, color, element, className, ...props }) => {
    let setVisible = props.visible || element.handlers.visible,
      visible = useTrkl(setVisible);
    const isVisible = Util.is.on(visible);
    if(Util.isObject(element) && 'visible' in element) setVisible = value => (element.visible = value);

    console.log(`Layer #${i} ${name} isVisible=${isVisible}`);
    return h('div',
      {
        className,
        onMouseMove: e => {
          if(e.buttons & 1 && setTo !== undefined) setVisible(setTo);
        },
        onMouseUp: e => {
          setTo = null;
        },
        onMouseDown: e => {
          if(e.buttons & 1) {
            setVisible((setTo = !isVisible));
            return true;
          }

          // if(e.buttons & 2)
        }
      }, [
        h('span', {
            className: classNames(className, 'number'),
            style: { background: color || (Util.isObject(element) && element.color) },
            ...props
          },
          `${i}`
        ),
        h('span', { className: classNames(className, 'name'), ...props }, `${name}`),
        h('img', {
          className: classNames(className, 'visible'),
          ...props,
          style: { height: '1em', width: 'auto' },
          src: `static/svg/${isVisible ? 'show' : 'hide'}.svg`
        })
      ]
    );
  };

  React.render(h(SlotProvider, {}, [
      Panel('buttons no-select', [
        h(Button, {
          image: 'static/svg/browse.svg',
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
            fn: () => showGrid(!showGrid()),
            //  state: showGrid,
            toggle: true,
            image: 'static/svg/grid.svg'
          }),
          h(DropDown, {
              isOpen: layersDropDown.subscribe(open => console.log('layers dropdown', { open }))
            }, [
              h(Button, {
                toggle: true,
                state: layersDropDown,
                //    fn: (e,state) => /*(e.buttons && e.type.endsWith('down')) &&*/ state && layersDropDown(state) || true,
                image: 'static/svg/layers.svg'
              }),
              props =>
                h(Chooser, {
                    ...props,
                    className: 'layers',
                    itemClass: 'layer',
                    itemComponent: Layer,
                    items: layerList
                  }, []
                )
            ]
          ),
          h(Button, {
            fn: debounceAsync(async e => {
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
                  console.debug('BoardToGerber gerber =', gerber);
                  project.gerber[side] = gerber;
                  if(gerber.data) {
                    gerber.cmds = await GerberParser.parse(gerber.data);
                    gerber.unit = gerber.cmds.find(i => i.prop == 'units');

                    gerber.points = gerber.cmds.filter(i => i.coord).map(({ coord }) => new Point(coord.x, coord.y));
                  }
                  console.debug('BoardToGerber side =', side, ' file =', gerber.file);
                }
              }
              const sides = Object.fromEntries(['back', 'front', 'drill', 'outline'].map(side => [side, project.gerber[side].file]));
              console.debug('  sides = ', sides);
              const allGcode = await GerberToGcode(project.name, {
                ...sides,
                nog64: true,
                'fill-outline': true,
                voronoi: true,
                /*'zero-start': true,*/ nog81: true
              });
              console.debug('GerberToGcode allGcode = ', allGcode);
              let bbox;
              for(let side of ['outline', 'back', 'front', 'drill']) {
                let gerb = project.gerber[side];
                let data = allGcode[side];
                let file = allGcode.files[side];

                if(data) {
                  let gc = (project.gcode[side] = { data, file });

                  if(side != 'drill') {
                    let processed = file.replace(/\.ngc$/, '.svg');
                    gc.svg = await FetchURL(processed).then(ResponseData);
                    let pos;
                    if((pos = gc.svg.indexOf('<svg ')) != -1) gc.svg = gc.svg.substring(pos);

                    console.debug('processed', processed, Util.abbreviate(gc.svg));

                    if(side == 'outline') {
                      let xmlData = tXml(gc.svg);
                      let svgPath = Util.tail(xmlData[0].children).children[0];
                      let points = SVG.pathToPoints(svgPath.attributes);
                      console.debug('points:', points);
                      bbox = new Rect(new BBox().update(points)).round(0.001);
                      console.debug('bbox:', bbox);

                      continue;
                    }
                    //  console.debug('gc.svg ',gc.svg );
                    let layer = GetLayer({
                      name: makeLayerName('processed', side),
                      'data-filename': processed,
                      create: (project, props = {}) => {
                        let g = SVG.create('g', { innerHTML: gc.svg, ...props }, project.svg);
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

                    layer.sublayers = Util.histogram(Element.walk(layer.dom, (e, acc) => (e.tagName.endsWith('g') ? acc : [...acc, e]), []),
                      e => e.getAttribute('style'),
                      new Map(),
                      () => new Set()
                    );
                  }

                  console.debug('GerberToGcode side =', side, ' gc =', gc.file, ' svg =', Util.abbreviate(gc.svg));
                }
              }
              gcode(project.gcode);

              function makeLayerName(name, side) {
                const prefix = side == 'front' ? 't-' : side == 'back' ? 'b-' : '';
                return Util.camelize(prefix +
                    path
                      .basename(name)
                      .replace(new RegExp(`_${side}`), '')
                      .replace(/\.[A-Za-z0-9]{3}$/, '')
                );
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
                //console.debug('GcodeToPolylines =', { side }, colors[side], gc);
                GcodeToPolylines(gc.data, { fill: false, color: colors[side], side });
              }
            },
            'data-tooltip': 'Create Voronoi diagram',
            image: 'static/svg/voronoi.svg'
          })
        ]),
        h(DynamicLabel, { className: 'vcenter pad-lr', caption: documentTitle }),
        h(DynamicLabel, { className: 'vcenter pad-lr', caption: documentSize }),
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
      h(FileList, {
        files: projects,
        onActive: open,
        onChange: debounceAsync(async (e, p, i) => await ChooseDocument(p, i), 5000, {
          leading: true
        }),
        filter: searchFilter,
        showSearch,
        changeInput,
        focusSearch,
        currentInput: currentSearch
      }),

      h(CrossHair, { ...crosshair }),
      h(FloatingPanel, { onSize: logSize, className: 'no-select', id: 'console' }, [
        /*h(div, {}, [ */ h(Logger, {}),
        h(Dumper, {}),
        h(Commander, {
          onCommand: cmdStr => {
            let fn = new Function(`return ${cmdStr};`);

            //console.log('Command:', cmdStr);
            LogJS.info(`> ${cmdStr}`);
            let result = fn();
            LogJS.info(`= ${Util.toSource(result)}`);
          }
        }) /*])*/
      ]),
      h(Slot, { name: 'layers' })
    ]),
    Element.find('#preact')
  );

  let move, resize;
  let box;
  container = Element.find('#main');

  //Element.find('.transformed-element-size').setAttribute('id', 'transformed-element');

  TouchListener(/*Util.printReturnValue*/ event => {
      const { x, y, index, buttons, start, type, target } = event;

      if(type.endsWith('end') || type.endsWith('up')) return cancel();
      if(event.buttons === 0 && type.endsWith('move')) return cancel();
      // if(event.index > 0) console.log('touch', { x, y, index, buttons, type, target }, container);
      if(!move && !resize) {
        let elemId;
        //  console.log('target:', target);
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
              //    console.log('resizeRelative:', { elemId, size });
              if(elemId == 'console') logSize(size);
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
          window.move = move = Element.moveRelative(box, null, id == 'console' ? ['right', 'bottom'] : ['left', 'top']);
          box.style.cursor = `move`;
        }
        return true;
      }
      if((move || resize) && event.buttons == 0) {
        return cancel();
      }

      if(event.index > 0) {
        let rel = new Point(event);
        let absolute = new Point(start).add(rel);

        if(resize) {
          if(event.buttons > 0) resize(-rel.x, -rel.y);
          else resize = resize.jump();
        } else if(move) {
          /*  window.crosshair.show = true;
          window.crosshair.position = absolute;*/

          //          console.log('move', ...[...rel], ...[...absolute]);
          if(event.buttons > 0) move(rel.x, rel.y);
          else move = move.jump();
        }
      }
      function cancel() {
        move = null;
        resize = null;
        window.crosshair.show = false;

        if(box && box.style) box.style.cursor = `default`;
        /*return*/ event.cancel();
        return false;
      }
    },
    { element: window }
  );

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
    for await (let e of new EventIterator('touch')) {
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
        //  curFcrentTarget,
        //    defaultPrevented,
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
        //     sourceCapabilities,
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
      // LogJS.info(`${type} ` + /* Util.toSource(e)+ */ ` ${x},${y} → ${Element.xpath(target)}`);
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
    const { deltaX, deltaY, screenX, screenY, clientX, clientY, pageX, pageY, x, y, offsetX, offsetY, layerX, layerY } = event;

    //console.log('wheel:', { deltaX, deltaY, screenX, screenY, clientX, clientY, pageX, pageY, x, y, offsetX, offsetY, layerX, layerY });
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

    //console.log('wheel:', { sideBar, clientArea });

    const clientCenter = clientArea.center;
    const { target, currentTarget, buttons, altKey, ctrlKey, shiftKey } = event;
    const pos = new Point(clientX, clientY);

    if(!pos.inside(clientArea)) return;

    const wheelPos = -event.deltaY.toFixed(2);
    let zoomVal = zoomLog();

    zoomVal = altKey || ctrlKey || shiftKey ? 0 : Util.clamp(-100, 300, zoomVal + wheelPos * 0.1);
    zoomLog(zoomVal);
    AdjustZoom();
  });

  console.error('AppMain done');

  //console.log(Util.getGlobalObject());

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
