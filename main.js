///prettier-ignore-ignore-start
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom from './lib/dom.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { iterator, eventIterator } from './lib/dom/iterator.js';
import keysim from './lib/dom/keysim.js';
import geom from './lib/geom.js';
import { BBox } from './lib/geom/bbox.js';
import { Polygon } from './lib/geom/polygon.js';
import { TouchListener } from './lib/touchHandler.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { ClipperLib } from './lib/clipper-lib.js';
import Shape from './lib/clipper.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tlite from './lib/tlite.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Alea from './lib/alea.js';
import { Cache } from './lib/dom/cache.js';
import { CacheStorage } from './lib/dom/cacheStorage.js';
import { gcodetogeometry, GcodeObject, gcodeToObject, objectToGcode, parseGcode, GcodeParser as GcodeInterpreter } from './lib/gcode.js';
import { Iterator } from './lib/iterator.js';
import { Functional } from './lib/functional.js';
import { makeLocalStorage } from './lib/autoStore.js';
import { Repeater } from './lib/repeater/repeater.js';
import { useResult } from './lib/repeater/react-hooks.js';
import LogJS from './lib/log.js';
import { useDimensions } from './useDimensions.js';
import { toXML, ImmutablePath } from './lib/json.js';
import { XmlObject, XmlAttr, ImmutableXPath } from './lib/xml.js';
import { RGBA, isRGBA, ImmutableRGBA, HSLA, isHSLA, ImmutableHSLA, ColoredText } from './lib/color.js';
//import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import React, { h, html, render, Fragment, Component, useState, useLayoutEffect, useRef } from './lib/dom/preactComponent.js';
import components, { Chooser, DynamicLabel, Button, FileList, Panel, SizedAspectRatioBox, TransformedElement, Canvas, ColorWheel, Slider, BrowseIcon, CrossHair, FloatingPanel, DropDown, Conditional } from './components.js';
import { Message } from './message.js';
import { WebSocketClient } from './lib/net/websocket-async.js';
import { CTORS, ECMAScriptParser, estree, Factory, Lexer, ESNode, Parser, PathReplacer, Printer, Stack, Token } from './lib/ecmascript.js';

import KolorWheel from './lib/KolorWheel.js';
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
/* prettier-ignore */ import { BoardRenderer, DereferenceError, EagleDocument, EagleElement, EagleNode, EagleNodeList, EagleNodeMap, EagleProject, EagleRef, EagleReference, EagleSVGRenderer, Renderer, SchematicRenderer, makeEagleElement, makeEagleNode
} from './lib/eagle.js';
//import PureCache from 'pure-cache';
import { brcache, lscache, BaseCache, CachedFetch } from './lib/lscache.js'; //const React = {Component, Fragment, create: h, html, render, useLayoutEffect, useRef, useState };

/* prettier-ignore */ /* prettier-ignore */ const { Align, Anchor, CSS, Event, CSSTransformSetters, Element, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementTransformation, ElementWHProps, ElementXYProps, isElement, isLine, isMatrix, isNumber, isPoint, isRect, isSize, Line, Matrix, Node, Point, PointList, Polyline, Rect, Select, Size, SVG, Timer, Transition, TransitionList, TRBL, Tree } = { ...dom, ...geom };
Util.colorCtor = ColoredText;
/* prettier-ignore */ Util.extend(window, { React, ReactComponent, WebSocketClient, html }, { dom, keysim }, geom, { Iterator, Functional }, { EagleNodeList, EagleNodeMap, EagleDocument, EagleReference, EagleNode, EagleElement }, { toXML, XmlObject, XmlAttr }, { CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token, ReactComponent, ClipperLib, Shape, isRGBA, RGBA, ImmutableRGBA, isHSLA, HSLA, ImmutableHSLA, ColoredText, Alea, Message }, { Chooser, useState, useLayoutEffect, useRef, Polygon } );

const prng = new Alea(1598127218);
let currentProj = trkl.property(window, 'project');
let layerList = trkl.property(window, 'layers', { value: [] });

let open = trkl();
let showSearch = trkl(true);
let logSize = trkl({});
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
let debug = true;
const documentTitle = trkl('');

let store = (window.store = makeLocalStorage());

let projects = trkl([]);
let socket = trkl();
let listURL = trkl(store.get('url') || null);
let searchFilter = trkl(store.get('filter') || '*');
let zoomLog = trkl(store.get('zoom') || null);

const add = (arr, ...items) => [...(arr ? arr : []), ...items];

const useSlot = (arr, i) => [() => arr[i], (v) => (arr[i] = v)];
const trklGetSet = (get, set) => (value) => (value !== undefined ? set(value) : get());
//const useTrkl = trkl => [() => trkl(), value => trkl(value)];

const classNames = (...args) => args.filter((arg) => typeof arg == 'string' && arg.length > 0).join(' ');

const MouseEvents = (h) => ({
  onMouseDown: h,

  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h
});
//console.log('running');
//console.log("dom", { Rect, Element });

window.dom = { Element, SVG };

tlite(() => ({ grav: '-|', attrib: ['data-tlite', 'data-tooltip', 'title', 'data-filename'] }));

/* prettier-ignore */
/*    const CreateSelect = (obj, node = document.body) => {
                            let elem = Select.create(Object.entries(obj));
                            node.insertBefore(elem, node.firstElementChild);
                          };*/
const utf8Decoder = new TextDecoder('utf-8');
let svgOwner, parent;

const svgFactory = lazyInitializer(() => {
  parent = project.svg.parentElement;

  const factory = SVG.factory({
    append_to(elem, p) {
      (p || parent).appendChild(elem);
    }
  });
  //const {width,height}=Element.rect('.aspect-ratio-box-inside');
  let rect = DrawSVG.calcViewBox();

  console.log('rect:', rect);

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
  //console.log('factory:', factory);

  const element = (svgOwner = factory(...svg));

  factory.root = parent = element.lastElementChild;

  //project.svg.parentElement.appendChild(element);

  return factory;
});

const GeneratePalette = (numColors) => {
  let ret = [];
  let base = new HSLA(Util.randInt(0, 360, prng), 100, 50).toRGBA();
  let offsets = Util.range(1, numColors).reduce((acc, i) => [...acc, ((acc[acc.length - 1] || 0) + Util.randInt(20, 80)) % 360], []);
  offsets = offsets.sort((a, b) => a - b);
  //offsets = Util.shuffle(offsets, prng);
  //Util.log('offsets:', offsets);

  new KolorWheel(base.hex()).rel(offsets, 0, 0).each(function () {
    const hex = this.getHex();
    const rgba = new RGBA(hex);
    const hsla = rgba.toHSLA();
    //Util.log(hex, rgba.toString(), hsla.toString());
    ret.push(hsla);
  });
  return ret;
};

function DrawSVG(...args) {
  const factory = svgFactory();

  let e;
  try {
    let parent = project.svg.parentElement.lastElementChild;
    const append = (e) => parent.appendChild(e);
    let c = RGBA.random();

    let [tag, attrs, children] = args;

    if(typeof tag == 'string') {
      console.log('draw(', ...args, ')');

      e = factory(tag, { stroke: c.hex(), 'stroke-width': 0.1, ...attrs }, children);
    } else if(Util.isArray(args[0])) {
      let items = args.shift();

      document.querySelector('#main > div > div > div > svg:nth-child(2) > g');

      DrawSVG.setViewBox(BBox.from(items));

      /*
items = items.map(i => isLine(i) ? new Line(i) : null);

console.log("items:",items);
*/
      for(let item of items) {
        // console.log('item:', item);
        let line;
        if(isLine(item)) line = new Line(item);

        //console.log('line:', line);

        if(line) {
          e = factory('line', { ...line.toObject(), stroke: c.hex(), 'stroke-width': 0.1 });
          append(e);
          console.log('e:', e);
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
}
DrawSVG.calcViewBox = (box) => {
  box = box || (project && project.doc && BBox.from(project.doc.getMeasures(true)));
  box = box || Element.rect('.aspect-ratio-box-inside');

  const { width, height, x, y } = box;

  let { x1, y1, x2, y2 } = new Rect(x, y, width, height);

  const rect = new BBox(x1, y1 - y2, x2 - x1, y2);
  return rect;
};

DrawSVG.setViewBox = (box) => {
  svgOwner = svgOwner || [...Element.findAll('svg', Element.find('#main'))].reverse()[0];
  const rect = box; // instanceof BBox ? box : DrawSVG.calcViewBox(box);

  rect.y1 -= rect.y2;
  rect.x2 -= rect.x1;

  console.log('setViewBox', { svgOwner, rect, box });
  svgOwner.setAttribute('viewBox', rect.toString());
  svgOwner.lastElementChild.setAttribute('transform', `scale(1,-1)  translate(0,${-rect.height})`);
  Element.attr(svgOwner.lastElementChild.firstElementChild, { ...rect.toRect() });
};

const GetProject = (arg) => (typeof arg == 'number' ? projects()[arg] : typeof arg == 'string' ? projects().find((p) => p.name == arg) : arg);

const ListProjects = async function (opts = {}) {
  const { url, descriptions = true, names, filter } = opts;

  //console.log('ListProjects', { url, descriptions, names, filter });

  let response;

  if(!url) {
    response = await fetch('/files.html', {
      //      nocache: true,
      method: 'post',
      mode: 'cors', // no-cors, *cors, same-origin
      cache: 'no-cache', // *default, no-cache, reload, force-cache, only-if-cached
      credentials: 'same-origin', // include, *same-origin, omit
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ descriptions, names, filter })
    });

    if(typeof response.text == 'function') response = await response.text();

    console.log('response:', Util.abbreviate(response));

    if(response) response = JSON.parse(response);
  } else {
    response = await ListGithubRepo(url, null, null, '\\.(brd|sch|lbr)$', opts);
    //console.log('response:', response);
    let fileList = response.map((file, i) => ({ ...file, name: response.at(i) }));

    response = { files: fileList };
  }

  return response;
};

const ElementToXML = (e, predicate) => {
  const x = Element.toObject(e, { predicate });
  //console.log('x:', x);
  return Element.toString(x, { newline: '\n' });
};

const FetchCached = Util.cachedFetch();

const FetchURL = async (url, allOpts = {}) => {
  let { nocache = false, ...opts } = allOpts;
  let result;
  let ret;

  if(opts.method && opts.method.toUpperCase() == 'POST') nocache = true;

  let fetch = nocache ? window.fetch : FetchCached;

  if(/tmp\//.test(url)) {
    url = url.replace(/.*tmp\//g, '/static/tmp/');
  } else if(/^\//.test(url)) {
  } else if(/:\/\//.test(url)) {
  } else {
    url = '/static/' + url;
  }
  try {
    //    if(!nocache) ret = cache.get(url);

    if(!ret) {
      ret = result = await fetch(url, opts);

      //  if(!nocache && typeof ret == 'string' && ret.length > 0) cache.set(url, ret);
    }
  } catch(error) {
    Util.putError(error);
    throw error;
  }

  //  if(typeof ret.text == 'function') ret = await result.text();

  return ret;
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

const LoadFile = async (file) => {
  //console.log('file:', file.url, file.name);
  let { url, name: filename } = GetProject(file);
  url = /:\/\//.test(filename) ? filename : `/static/${filename}`;
  console.log('LoadFile url=', url);
  let response = await FetchURL(url);
  console.log('LoadFile response=', response);

  let xml = /* await FetchURL(url);*/ await response.text(); //(typeof file.
  // console.log('LoadFile xml=', await xml);
  //file= await FetchURL(url);  //'function' && (await file.fetch())) || (await FetchURL(url, {}));
  //console.log('LoadFile', { url, xml });
  //console.log('xml: ', xml.substring(0, 100));
  //let dom = new DOMParser().parseFromString(xml, 'application/xml');

  let doc = new EagleDocument(await xml, null, filename, null, FileSystem);

  if(/\.brd$/.test(filename)) window.board = doc;
  if(/\.sch$/.test(filename)) window.schematic = doc;
  if(/\.lbr$/.test(filename)) window.libraries = add(window.libraries, doc);

  LogJS.info('LoadFile', doc.file);

  return doc;
};

const SaveFile = async (filename, data, contentType) => {
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

const SaveSVG = (window.save = async function save(filename, layers = [1, 16, 20, 21, 22, 23, 25, 27, 47, 48, 51]) {
  const { doc } = project;
  const { basename, typeName } = doc;
  if(!filename) filename = `${doc.basename}.${doc.typeName}.svg`;

  let predicate = (element) => {
    if(!element.hasAttribute('data-layer')) return true;
    //console.log('element:', element);
    const layer = element.getAttribute('data-layer');
    let [number, name] = layer.split(/ /);
    if(number !== undefined && name !== undefined) return layers.indexOf(+number) != -1 || layers.indexOf(name) != -1;
    return true;
  };
  let data = ElementToXML(project.svg, predicate);

  return await SaveFile(filename.replace(/\.svg$/i, '.svg'), data);
});

const ModifyColors = (fn) => (e) => {
  const { type, buttons } = e;
  if(type.endsWith('down')) {
    if(!window.c) window.c = SVG.allColors(project.svg);
    let { c } = window;
    //console.log('ModifyColors', fn);

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

const BoardToGerber = async (board = project.name, opts = { fetch: true }) => {
  let proj = GetProject(board);

  let request = { ...opts, board: proj.name },
    response;
  try {
    response = await FetchURL('/gerber', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request)
    }).then((r) => r.json());
    //response = JSON.parse(response);
    if(opts.fetch && response.file) {
      proj.gerber = response.data = await FetchURL(`static/${response.file.replace(/^\.\//, '')}`).then((r) => r.text());
    }
  } catch(err) {
    response.error = err;
  }
  return response;
};

const GerberToGcode = async (gerber = project.name, opts = {}) => {
  let proj = GetProject(gerber);
  let request = { ...opts, gerber: proj.gerber.file, fetch: true, /*voronoi: '1', */ 'isolation-width': '1mm' },
    response;
  try {
    response = await FetchURL('/gcode', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request)
    }).then((r) => r.json());
    /// response = JSON.parse(response);
    if(opts.fetch && response.file) proj.gcode = response.data = await FetchURL(`static/${response.file.replace(/^\.\//, '')}`).then((r) => r.text());
    // response.polylines = GcodeToPolylines(proj, request);
  } catch(err) {
    response.error = err;
  }
  response.opts = opts;
  return response;
};

const GcodeToPolylines = (gcode = project.name, opts = {}) => {
  const { fill = false, color } = opts;
  let proj = GetProject(gcode);
  let gc = Util.filter(parseGcode(project.gcode.data), (g) => /G0[01]/.test(g.command + '') && 'x' in g.args && 'y' in g.args);
  let polylines = [];
  let polyline = null;
  let bb = new BBox();
  const NewPolyline = () => {
    if(polyline) polylines.push(polyline);
    polyline = new Polyline([]);
  };
  const f = new Point(-25.4, 25.4);
  for(let g of gc) {
    if(g.command == 'G00') NewPolyline();
    if(isPoint(g.args)) {
      let p = new Point(g.args.x, g.args.y).prod(f).round(0.01, 2);
      bb.updateXY(p.x, p.y);
      polyline.push(p);
    }
  }
  NewPolyline();
  let palette = GeneratePalette(polylines.length);
  let ret = { polylines, bbox: bb, palette };
  //console.log('polylines(1):', polylines);
  let remove = new Set();
  let props = (polyline, i) => ({ stroke: color || palette[i], fill: fill ? palette[i].prod(1, 1, 1, 0.5) : 'none' });
  let grp = SVG.create('g', { fill: 'none', stroke: 'magenta', 'stroke-width': 0.1, transform: ` scale(1,-1) translate(${0},${-bb.y2}) translate(-0.3175,0)  translate(0,-2.54)` }, proj.svg);
  let paths = [];

  if(fill) {
    polylines = polylines.map((pl) => pl.toMatrix().flat());
    //console.log('polylines(2):', polylines);
    polylines = polylines.map((pl) => geom.simplify(pl, 0.02, true));
    //console.log('polylines(3):', polylines);
    polylines = polylines.map((pl) => Util.chunkArray(pl, 2).map((pt) => new Point(...pt)));
    //console.log('polylines(4):', polylines);
    polylines = polylines.map((pl) => new Polyline([]).push(...pl));
    let inside = new Map(polylines.map((polyline2, i) => [polyline2, polylines.filter((polyline, j) => polyline !== polyline2 && i !== j && Polyline.inside(polyline, polyline2))]));
    let insideOf = polylines.map((polyline, i) => [
      i,
      polylines
        .map((polyline2, j) => [inside.get(polyline2).length, j, polyline2])
        .filter(([n, j, polyline2]) => i !== j && inside.get(polyline2).indexOf(polyline) != -1)
        .sort(([a], [b]) => a - b)
    ]);
    console.log('insideOf:', insideOf);
    let holes = polylines.map((polyline, i) => new Set());
    insideOf.filter(([i, list]) => list.length == 1).map(([i, list]) => holes[list[0][1]].add(i));
    console.log('holes:', holes);
    let remove = new Set();
    for(let [i, inner] of holes.entries()) {
      let ids = [i, ...inner];
      //console.log('polygon', { i, ids, inner });
      const polyline = polylines[i];
      inner = [...inner].map((ip) => polylines[ip].counterClockwise);
      if(inner.length == 0) continue;
      //console.log('polygon', { polyline, inner });
      let list = [polyline, ...inner];
      paths.push([i, list.map((pl) => pl.toPath()).join('\n')]);
      ids.forEach((id) => remove.add(id));
    }
  }
  let ids = polylines.map((pl, i) => i).filter((i) => !remove.has(i));
  let polys = [...ids.map((i) => polylines[i].toSVG((...args) => args, { ...props(polylines[i], i), id: `polyline-${i}` }, grp, 0.01)), ...paths.map(([i, d]) => ({ ...props(polyline, i), id: `polygon-${polylines.indexOf(polyline)}`, d })).map((p, i) => ['path', p, grp])];
  //console.log('polys:', polys);
  let elements = polys.map((args) => SVG.create(...args));
  return { ...ret, group: grp, elements };
};

const ListGithubRepo = async (owner, repo, dir, filter, opts = {}) => {
  const { username, password } = opts;
  let host, path;
  if(new RegExp('://').test(owner) || (repo == null && dir == null)) {
    const url = owner;
    let parts = url
      .replace(/.*:\/\//g, '')
      .replace('/tree/master', '')
      .split('/');
    while(!/github.com/.test(parts[0])) parts = parts.slice(1);
    [host, owner, repo, ...path] = parts;

    dir = path.join('/');
  }

  const url = `https://api.github.com/repos/${owner}/${repo}/contents/${dir}`;
  //console.log('ListGithubRepo', { host, owner, repo, dir, filter, url });
  const headers = {
    Authorization: 'Basic ' + window.btoa(`${username}:${password}`)
  };
  let response = await FetchURL(url, {
    //method: 'get', // credentials: 'include',
    headers
  });
  let result = JSON.parse(await response.text());

  if(!Util.isArray(result)) return result;

  if(filter) {
    const re = new RegExp(filter, 'g');
    result = result.filter(({ name, type }) => type == 'dir' || re.test(name));
  }
  //  console.log('result:', result);
  const firstFile = result.find((r) => !!r.download_url);
  const base_url = firstFile ? firstFile.download_url.replace(/\/[^\/]*$/, '') : '';
  const files = result.map(({ download_url = '', html_url, name, type, size, path, sha }) => ({
    url: (download_url || html_url || '').replace(base_url + '/', ''),
    name,
    type,
    size,
    path,
    sha
  }));
  const at = (i) => {
    let url = files[i].url;
    if(!/:\/\//.test(url)) url = base_url + '/' + url;
    return url;
  };

  return Object.assign(files.map((file, i) => {
      file.toString = () => at(i);
      if(file.type == 'dir') file.list = async (f = filter) => await ListGithubRepo(at(i), null, null, f, {});
      else {
        let getter = async function () {
          let data = await fetch(at(i), {});
          //         console.log('data:', data);
          this.buf = await data.text();
          return this.buf;
        };

        let text = function () {
          return typeof this.buf == 'string' && this.buf.length > 0 ? this.buf : this.get();
        }; //Util.memoize((...args) => file.fetch(...args));

        file.get = getter;
        file.getText = text;
        Object.defineProperty(file, 'text', { get: text, enumerable: true, configurable: true });

        //Util.defineGetter(file, 'text', text);
      }
      return file;
    }),
    {
      base_url,
      at,
      async get(i) {
        const url = at(i);
        //console.log('url:', url);
        return await FetchURL(url, {});
      },
      get files() {
        return files.filter((item) => item.type != 'dir');
      },
      get dirs() {
        return files.filter((item) => item.type == 'dir');
      }
    }
  );
};

const ListGithubRepoServer = async (owner, repo, dir, filter) => {
  let response;
  let request = { owner, repo, dir, filter };
  try {
    response = await FetchURL('/github', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(request)
    });
  } catch(err) {}
  let ret = JSON.parse(response);
  ret.at = function (i) {
    return this.base_url + '/' + this.files[i];
  };
  ret.get = async function (i) {
    let data = await FetchURL(this.at(i));
    return data;
  };
  return ret;
};

const LoadDocument = async (project, parentElem) => {
  //console.log('project:', project);
  open(false);
  try {
    project.doc = await LoadFile(project);
  } catch(error) {
    console.error(error);
    throw error;
  }
  LogJS.info(`${project.doc.basename} loaded.`);
  documentTitle(project.doc.file.replace(/.*\//g, ''));
  window.eagle = project.doc;
  window.project = project;
  Element.remove('#fence');
  let docElem = Element.find('#doc');
  docElem.innerHTML = '';
  console.log('project.doc:', project.doc.basename);
  project.renderer = new Renderer(project.doc, ReactComponent.append, debug);
  console.log('project.renderer', project.renderer);
  let style = { width: '100%', height: '100%', position: 'relative' };

  let component = project.renderer.render(project.doc, null, {});

  setTimeout(() => layerList([...project.doc.layers.list].filter((layer) => layer.elements.size > 0).map((layer) => ({ i: layer.number, name: layer.name, element: layer }))), 250);

  LogJS.info(`${project.name} rendered.`);
  window.component = project.component = component;
  console.log('testRender:', component);
  let element = Element.find('#main');
  let r = project.renderer.rect || project.renderer.bounds;
  console.log('project.renderer:', project.renderer);
  console.log('r:', r);
  let aspectRatio = 1;
  if(r) {
    aspectRatio = r.width / r.height;
    sizeListener({ width: r.width });
  }
  aspectListener(aspectRatio);
  console.log('aspectRatio:', aspectRatio);

  const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
    const [dimensions, setDimensions] = useState(sizeListener());
    const [aspect, setAspect] = useState(aspectListener());
    if(sizeListener && sizeListener.subscribe) sizeListener.subscribe((value) => setDimensions(value));
    if(aspectListener && aspectListener.subscribe) aspectListener.subscribe((value) => setAspect(value));
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

  console.log('rendered:', rendered);

  let eagle2dom = [...Element.findAll('*[data-path]')];

  eagle2dom = eagle2dom.map((e) => [e.getAttribute('data-path'), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [new ImmutablePath(p), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [p, p.apply(project.doc.raw), e]);
  eagle2dom = eagle2dom.map(([p, r, e]) => [EagleElement.get(project.doc, p, r), e]);

  console.log('eagle2dom:', eagle2dom);

  let dom2eagle = Util.mapFunction(new WeakMap(eagle2dom.map(([k, v]) => [v, k])));

  eagle2dom = Util.mapFunction(new WeakMap(eagle2dom));
  console.log('eagle2dom:', eagle2dom);

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

  let { name, data, doc, svg, bbox } = project;
  let bounds = doc.getBounds();
  let rect = bounds.toRect(Rect.prototype);
  let size = new Size(r);
  currentProj(project);
  size.mul(doc.type == 'brd' ? 2 : 1.5);
  let svgrect = SVG.bbox(project.svg);

  let measures = new BBox().update(doc.getMeasures(true)).toRect(Rect.prototype);
  console.log('measures:', measures);
  //project.aspectRatio = svgrect.aspect();

  Element.attr(project.svg, {
    'data-filename': project.name,
    'data-aspect': project.aspectRatio,
    'data-width': size.width + 'mm',
    'data-height': size.height + 'mm'
  });

  //project.svg.setAttribute('data-aspect', project.aspectRatio);
  let css = size.div(0.26458333333719).toCSS({ width: 'px', height: 'px' });

  window.size = css;
  //console.log('css:', css);
  /*  Object.assign(project.svg.style, {
    'min-width': `${size.width}mm`
  });
  Element.setCSS(project.svg, { left: 0, top: 0, position: 'relative' });
  Element.setCSS(project.svg, { left: 0, top: 0, position: 'relative' });
  //console.log('LoadDocument:', project.svg);*/
  /*  } catch(err) {
    console.error('Render ERROR:',
      err,
      [...err.stack].map(f => (f + '').replace(Util.getURL() + '/', ''))
    );
  }*/

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

/*
gerber=await BoardToGerber(project.name); gc=await GerberToGcode('tmp/7seg-2.54.GBL'); geom=gcodetogeometry(gc.data);lines = geom.lines.map(({start,end}) => new Line(start,end))

*/

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
  //bb.outset(1.27);
  /*new BBox();
  for(let item of doc.plain) {
    if(item.layer.number != 47) continue;

    bb.update(item.geometry());

    //console.log('item:', item);
  }*/

  let sites = points.map((p) => p.toObject());
  //xl, xr means x left, x right
  //yt, yb means y top, y bottom
  //console.log('bbox:', bb);

  let bbox = { xl: bb.x1, xr: bb.x2, yt: bb.y1, yb: bb.y2 };
  let voronoi = new Voronoi();
  //pass an object which exhibits xl, xr, yt, yb properties. The bounding
  //box will be used to connect unbound edges, and to close open cells
  let result = voronoi.compute(sites, bbox);
  //render, further analyze, etc.
  console.log('result:', Object.keys(result).join(', '));

  let { site, cells, edges, vertices, execTime } = result;
  console.log('cells:', cells);

  let holes = edges.filter((e) => !e.rSite).map(({ lSite, rSite, ...edge }) => new Point(lSite));
  let rlines = edges.filter((e) => e.rSite).map(({ lSite, rSite, ...edge }) => new Line(lSite, rSite));
  let vlines = edges.filter((e) => e.va && e.vb).map(({ va, vb, ...edge }) => new Line(va, vb).round(0.127, 4));
  let points2 = vertices.map((v) => new Point(v).round(0.127, 4));
  const add = (arr, ...items) => [...(Util.isArray(arr) ? arr : []), ...items];

  const factory = SVG.factory(/*{
    create: tag => ({ tagName: tag }),
    append_to: (elem, parent) => (parent.children = add(parent.children, elem)),
    setattr: (elem, name, value) => {
      if(!elem.attributes) elem.attributes = {};
      elem.attributes[name] = value;
    }
  }*/);

  const lines = [...rlines.map((l) => ['line', { ...l.toObject((t) => t + ''), stroke: '#000', 'stroke-width': 0.01 }]), ...vlines.map((l) => ['line', { ...l.toObject((t) => t + ''), stroke: '#f00', 'stroke-width': 0.01 }])];

  const circles = [
    ...holes.map((p) => ['circle', { cx: p.x, cy: p.y, r: 0.254, fill: 'none', stroke: '#00f', 'stroke-width': 0.3 }])

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

const MakeFitAction = (index) => async () => {
  let parent = Element.find('#main');
  let prect = Element.rect(parent);
  let svg = Element.find('svg', parent);
  let container = [...Element.findAll('.aspect-ratio-box-size', parent)].reverse()[0];
  //console.log('container:', container);
  let oldSize = Element.rect(container);
  let brect = Element.rect('.buttons');
  let srect = Element.rect(svg);
  prect.y += brect.height;
  prect.height -= brect.height;
  let rects = [prect, oldSize, srect];
  prect.scale(0.8);
  //console.log('resize rects', { oldSize, prect, srect });
  let f = srect.fit(prect);
  let newSize = f[index].round(0.0001);
  let affineTransform = Matrix.getAffineTransform(oldSize.toPoints(), newSize.toPoints());

  /*console.log('oldSize:', oldSize);
  //console.log('newSize:', newSize);*/

  let oldTransform = window.transform.clone();
  // console.log('oldTransform:', oldTransform);
  let oldFactor = oldTransform.scaling.x;
  //console.log('oldFactor:', oldFactor);

  const zoom = Math.pow(10, zoomVal / 200).toFixed(5);
  //console.log('zoom:', zoom);

  let transform = affineTransform.decompose();
  //console.log('transform:', transform);
  //console.log(`fitAction(${index})`, { oldSize, newSize, transform });
  let factor = transform.scale.x;
  //console.log('factor:', factor);
  let newFactor = zoom * factor;
  //console.log('newFactor:', newFactor);

  let newTransform = new TransformationList().scale(newFactor, newFactor);
  //console.log('newTransform:', newTransform);

  /*
let newScaling = newTransform.scaling;
  //console.log('newScaling:', newScaling);
newScaling.x *= factor;
newScaling.y *= factor;
newTransform[0] = newScaling;
*/
  let delay = Math.abs(Math.log(factor) * 1000);
  //Element.setCSS(container, { ...oldSize.toCSS(), transform: '', position: 'absolute' });
  //await Element.transition(container, { ...newSize.toCSS()/*, transform: '', position: 'absolute'*/ }, delay + 'ms', 'linear');
  await Element.transition(container, { transform: newTransform }, delay + 'ms', 'linear');
  window.transform = newTransform;
  zoomVal = Math.log10(newFactor) * 200;
};

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
    LogJS.info('WebSocket data:', msg[Symbol.toStringTag]());
    ws.dataAvailable !== 0;
  }
  await ws.disconnect();
};

const BindGlobal = Util.once((arg) => trkl.bind(window, arg));

const AppMain = (window.onload = async () => {
  Util(globalThis);

  //prettier-ignore
  Object.assign(window, { Repeater, BBox, ChooseDocument, classNames, ColorMap, components, CSS, deep, EagleDocument, EagleElement, EagleNode, ImmutablePath, ImmutableXPath, EagleReference, eventIterator, h, HSLA, html, isLine, isPoint, isRect, isSize, iterator, Line, LoadDocument, LoadFile, Matrix, MatrixTransformation, ModifyColors, Point, PointList, React, Rect,  Rotation, Scaling, Size, SVG, Transformation, TransformationList, Translation, tXml, Util, MouseEvents, ElementToXML, LoadFile, ModifyColors, MakeFitAction, CreateWebSocket, AppMain, Canvas, BoardToGerber, ListGithubRepo, ListGithubRepoServer, brcache, lscache, BaseCache, FetchCached, CachedFetch ,GerberToGcode });
  Object.assign(window, { cache, tlite, FetchURL, GcodeToPolylines, geom },
    { classes: { Cache, CacheStorage, Response, Request } },
    {
      PrimitiveComponents,
      ElementNameToComponent,
      ElementToComponent,
      Wire,
      Instance,
      SchematicSymbol
    },
    { Emitter, EventIterator }
  );
  Object.assign(window, { LogJS },
    { Element, devtools, dom, RGBA, HSLA, draw: DrawSVG },
    { Voronoi, GerberParser, GenerateVoronoi, gcodetogeometry, GcodeObject, gcodeToObject, objectToGcode, parseGcode, GcodeInterpreter, GcodeParser },
    {
      SVGAlignments,
      AlignmentAttrs,
      Alignment,
      AlignmentAngle,
      Arc,
      CalculateArcRadius,
      ClampAngle,
      EagleAlignments,
      HORIZONTAL,
      HORIZONTAL_VERTICAL,
      InvertY,
      LayerAttributes,
      LinesToPath,
      MakeCoordTransformer,
      PolarToCartesian,
      RotateTransformation,
      VERTICAL
    }
  );
  Object.assign(window, { ListProjects, LoadDocument, LoadFile, ChooseDocument, SaveSVG });

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

  window.keystroke = (target) => (key, modifiers = 0) => keysim.Keyboard.US_ENGLISH.dispatchEventsForKeystroke(new keysim.Keystroke(modifiers, key), target);

  window.focusSearch = (state) => {
    const input = currentSearch();
    //console.log('focusSearch', input.tagName, state);
    input[state ? 'focus' : 'blur']();
  };

  BindGlobal({
    projects,
    socket,
    transform,
    size: sizeListener,
    aspect: aspectListener,
    showSearch,
    logDimensions: logSize,
    watched: dump
  });

  currentSearch.subscribe((value) => {
    if(value) {
      focusSearch(false);

      setTimeout(() => {
        //console.log('currentSearch:', value);
        focusSearch(true);
      }, 1000);
    }
  });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = (props) => html` <div>This is a test</div> `;

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
      File.prototype.toString = function () {
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
          const f = list.find((i) => i.svg === svgFile.name);
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
  CreateWebSocket(null, null, (ws) => (window.socket = ws));

  const crosshair = {
    show: trkl(false),
    position: trkl({ x: 0, y: 0 })
  };

  window.crosshair = trkl.bind({}, crosshair);

  searchFilter.subscribe((value) => {
    store.set('filter', value);
    LogJS.info(`searchFilter is ${value}`);
  });
  zoomLog.subscribe((value) => {
    store.set('zoom', value);
  });

  listURL.subscribe((value) => {
    store.set('url', value);
    LogJS.info(`listURL is '${value}'`);
  });

  trkl.bind(window, { searchFilter, listURL });
  trkl.bind(window, { svgFactory });

  trkl.bind(window, { zoomVal: zoomLog });

  zoomLog.subscribe((value) => {
    const zoomFactor = ZoomFactor(value);
    console.log('zoomFactor changed', zoomFactor);
    if(value === 1) throw new Error(value);
  });

  const updateIfChanged = (trkl, newValue, callback) => {
    const oldValue = trkl();
    if(!Array.prototype.every.call(oldValue, (elem, i) => newValue[i] === elem)) return false;
    trkl(newValue);
    if(typeof callback == 'function') callback(trkl, oldValue, newValue);
    return true;
  };

  const changeInput = (e) => {
    const { target } = e;
    LogJS.info('changeInput:', target.value);

    let { value } = target;

    let parts = value.split(/\s+/g);

    let urls = parts.filter((p) => /\:\/\//.test(p)).join('\n');

    updateIfChanged(listURL, urls, () => {});
    listURL(urls);

    //    value = parts.filter(p => !/\:\/\//.test(p)).join(' ');

    searchFilter(value == '' ? '*' : value.split(/\s*\|\s*/g).join(' | '));
  };

  const Consumer = (props) => {
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
  const Logger = (props) => {
    const [lines, setLines] = useState([]);

    const [ref, { x, y, width, height }] = useDimensions();

    const r = new Rect({ x, y, width, height });
    if(!loggerRect.equals(r)) {
      //console.log('Logger.dimensions:', r);
      loggerRect = r;
      logSize({ width });
    }

    const result = useResult(async function* () {
      for await (let msg of logger) yield msg;
    });
    if(result) {
      //     console.log("result:",result);
      lines.push(result.value);
    }
    return h('table',
      { className: 'logger', ref },
      lines.slice(-10, lines.length).map(([type, d, t, m], i) =>
        h('tr', {}, [
          h('td', { className: 'log sign' },
            h('img', {
              className: 'log sign',
              src: `/static/${type.toLowerCase() || 'warn'}.svg`,
              style: { height: '14px', width: 'auto', marginTop: '-1px' }
            })
          ),
          h('td', { className: 'log message' }, /*333*/ m + '')
        ])
      )
    );
  };
  dump({ ...dump(), test: 123 });

  const Dumper = (props) => {
    const [values, setValues] = useState(dump());
    let lines = [];
    dump.subscribe((value) => setValues(value));
    for(let [key, value] of Object.entries(values)) lines.push([key, value]);
    return h('table',
      { border: '0', cellpadding: 3, cellspacing: 0, className: 'dumper' },
      lines.map(([k, v], i) => h('tr', { className: 'watch' }, [h('td', { className: 'name' }, k + ''), h('td', { className: 'value' }, v + '')]))
    );
  };

  const Commander = ({ onCommand, ...props }) => {
    const [inputText, setInputText] = useState('');
    const handler = (e) => {
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

  const Toggle = (trkl) => trkl(!trkl());

  const Layer = ({ title, name, label, i, element, className, ...props }) => {
    const [visible, setVisible] = useTrkl(element.handlers.visible);
    console.log('Layer props=', props);
    return h('div',
      {
        className,
        onClick: (e) => {
          setVisible(element.visible ? 'no' : 'yes');
        }
      }, [
        h('span', {
            className: classNames(className, 'number'),
            style: { background: element.color },
            ...props
          },
          `${i}`
        ),
        h('span', { className: classNames(className, 'name'), ...props }, `${name}`),
        h('img', {
          className: classNames(className, 'visible'),
          ...props,
          style: { height: '1em', width: 'auto' },
          src: `static/svg/${element.visible ? 'show' : 'hide'}.svg`
        })
      ]
    );
  };

  React.render(h(SlotProvider, {}, [
      Panel('buttons', [
        h(Button, {
            caption: BrowseIcon(),

            fn: (e) => {
              if(e.type.endsWith('down')) {
                //console.log('file list push', e);
                open(!open());
              }
            }
          },
          html`
            <svg height="28" width="28" style=${{ height: 'auto', position: 'relative', width: '100%' }} viewBox="131 -131 512 512" xmlns="http://www.w3.org/2000/svg">
              <path d="M131-70.3v390.6h432.4L643 76.2H207.4l-48.8 145.7 33.1-170.2h378V2.9H326.3l-47.2-73.2z" fill="#fff" />
            </svg>
          `
        ),

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
          fn: MakeFitAction(0),
          image: 'static/fit-vertical.svg'
        }),
        h(Button, {
          //  caption: '↕',
          fn: MakeFitAction(1),
          image: 'static/fit-horizontal.svg'
        }),
        h(Conditional, { signal: currentProj },
          h(DropDown, { isOpen: layersDropDown.subscribe((open) => console.log('layers dropdown', { open })) }, [
            h(Button, {
              toggle: true,
              state: layersDropDown,
              //    fn: (e,state) => /*(e.buttons && e.type.endsWith('down')) &&*/ state && layersDropDown(state) || true,
              image: 'static/svg/layers.svg'
            }),
            (props) =>
              h(Chooser, {
                  ...props,
                  className: 'layers',
                  itemClass: 'layer',
                  itemComponent: Layer,
                  items: layerList
                }, []
              )
          ])
        ),
        h(DynamicLabel, { className: 'vcenter pad-lr', caption: documentTitle }),
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
      html` <${FileList} files=${projects} onActive=${open} onChange=${(e, p, i) => ChooseDocument(p, i)} filter=${searchFilter} showSearch=${showSearch} changeInput=${changeInput} focusSearch=${focusSearch} currentInput=${currentSearch} /> `,
      h(CrossHair, { ...crosshair }),
      h(FloatingPanel, { onSize: logSize, className: 'no-select', id: 'console' }, [
        h(Logger, {}),
        h(Dumper, {}),
        h(Commander, {
          onCommand: (cmdStr) => {
            let fn = new Function(`return ${cmdStr};`);

            //console.log('Command:', cmdStr);
            LogJS.info(`> ${cmdStr}`);
            let result = fn();
            LogJS.info(`= ${Util.toSource(result)}`);
          }
        })
      ]),
      h(Slot, { name: 'layers' })
    ]),
    Element.find('#preact')
  );

  let move, resize;
  let box;
  container = Element.find('#main');

  TouchListener(/*Util.printReturnValue*/ (event) => {
      const { x, y, index, buttons, start, type, target } = event;

      if(type.endsWith('end') || type.endsWith('up')) return cancel();
      if(event.buttons === 0 && type.endsWith('move')) return cancel();
      // if(event.index > 0) console.log('touch', { x, y, index, buttons, type, target }, container);
      if(!move && !resize) {
        console.log('target:', target);
        box = ((e) => {
          do {
            if(['main', 'console'].indexOf(e.getAttribute('id')) != -1) return e;
          } while((e = e.parentElement));
        })(target);
        console.log('box:', box);

        if(event.buttons && event.buttons != 1) {
          if('preventDefault' in event) event.preventDefault();
          if(!resize && box) {
            let edges = Element.rect(box).toPoints();
            let corners = [edges[0], edges[2]].map((p, i) => [i, p.distance(new Point(start).sum(x, y)), p]);

            let edge = corners.sort((a, b) => a[1] - b[1])[0];

            window.resize = resize = Element.resizeRelative(box, null, edge[0] ? -1 : 1);
            console.log('RESIZE:', { resize, box, corners, edge });
            return true;
          } else {
            return cancel();
          }
        }

        //        let box = Element.find('#main').firstElementChild;
        const id = box && box.getAttribute('id');

        if(id == 'console') {
          const rects = [true, false].map((border) => Element.rect(box, { border }));
          let p = new Point(start.x + x, start.y + y);
          //console.log('', p);
          const inside = rects.map((r) => r.inside(p));
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
          box.style.cursor = `${compass}-resize`;
        }

        if(box) window.move = move = Element.moveRelative(box, null, id == 'console' ? ['right', 'bottom'] : ['left', 'top']);
        return true;
      }
      if((move || resize) && event.buttons == 0) return cancel();

      if(event.index > 0) {
        let rel = new Point(event);
        let absolute = new Point(start).add(rel);

        if(resize) {
          if(event.buttons > 0) resize(-rel.x, -rel.y);
        } else if(move) {
          /*  window.crosshair.show = true;
          window.crosshair.position = absolute;*/

          //          console.log('move', ...[...rel], ...[...absolute]);
          if(true || event.buttons > 0) move(rel.x, rel.y);
          else move = move.jump();
        }
      }
      function cancel() {
        move = null;
        resize = null;
        window.crosshair.show = false;

        /*return*/ event.cancel();
        return false;
      }
    },
    { element: window }
  );

  window.oncontextmenu = function (e) {
    const { x, y, index, buttons, start, type, target } = event;
    let rect = Element.rect('.transformed-element');
    let cons = Element.rect('#console');
    if(rect.inside(event) && !cons.inside(event)) return true;
    //console.log('oncontextmenu', { target }, event);
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

  window.addEventListener('wheel', (event) => {
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
    zoomVal = altKey || ctrlKey || shiftKey ? 0 : Util.clamp(-100, 200, zoomVal + wheelPos * 0.1);
    AdjustZoom();
  });

  console.error('AppMain done');

  //console.log(Util.getGlobalObject());

  /*  for(let path of [...Element.findAll('path')]) {
    let points = new PointList([...SVG.pathIterator(path, 30, p => p.toFixed(3))]);
  }*/
});
function ZoomFactor(val) {
  return Math.pow(10, val / 200).toFixed(5);
}
function AdjustZoom() {
  const zoom = ZoomFactor(zoomVal);

  let t = window.transform;
  //console.log('t:', t);

  if(!t.scaling) t.scale(zoom, zoom);
  else {
    t.scaling.x = zoom;
    t.scaling.y = zoom;
  }
  //console.log('window.transform:', window.transform);
  window.transform = new TransformationList(t);
}

const Module = {
  noInitialRun: true,
  onRuntimeInitialized: () => {
    //console.log('initialized');
    let myString = prompt('Enter a string:');
    Module.callMain([myString]);
  },
  print: (txt) => alert(`The MD5 hash is: ${txt}`)
};
