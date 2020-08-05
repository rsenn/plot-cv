//prettier-ignore-ignore-start
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom from './lib/dom.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { iterator, eventIterator } from './lib/dom/iterator.js';
import keysim from './lib/dom/keysim.js';
import geom from './lib/geom.js';
import { BBox } from './lib/geom/bbox.js';
import { Polygon } from './lib/geom/polygon.js';
import { ScrollDisabler } from './lib/scrollHandler.js';
import { TouchListener } from './lib/touchHandler.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { ClipperLib } from './lib/clipper-lib.js';
import Shape from './lib/clipper.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Alea from './lib/alea.js';
import { Iterator } from './lib/iterator.js';
import { Functional } from './lib/functional.js';
import { makeLocalStorage } from './lib/autoStore.js';
import { Repeater } from './lib/repeater/repeater.js';
import { useValue, useResult, useAsyncIter } from './lib/repeater/react-hooks.js';
import LogJS from './lib/log.js';
import { useDimensions } from './useDimensions.js';
import { toXML, ImmutablePath } from './lib/json.js';
import { XmlObject, XmlAttr, ImmutableXPath } from './lib/xml.js';
import { RGBA, isRGBA, ImmutableRGBA, HSLA, isHSLA, ImmutableHSLA, ColoredText } from './lib/color.js';
//import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './lib/dom/preactComponent.js';
import components, { Chooser, DynamicLabel, Label, Container, Button, FileList, Panel, AspectRatioBox, SizedAspectRatioBox, TransformedElement, Canvas, ColorWheel, Slider, BrowseIcon, CrossHair, FloatingPanel } from './components.js';
import { Message } from './message.js';
import { WebSocketClient } from './lib/net/websocket-async.js';
import { CTORS, ECMAScriptParser, estree, Factory, Lexer, ESNode, Parser, PathReplacer, Printer, Stack, Token } from './lib/ecmascript.js';

import { PrimitiveComponents, ElementNameToComponent, ElementToComponent } from './lib/eagle/components.js';
import { SVGAlignments, AlignmentAttrs, Alignment, AlignmentAngle, Arc, CalculateArcRadius, ClampAngle, EagleAlignments, HORIZONTAL, HORIZONTAL_VERTICAL, InvertY, LayerAttributes, LinesToPath, MakeCoordTransformer, PolarToCartesian, RotateTransformation, VERTICAL } from './lib/eagle/renderUtils.js';
import { Wire } from './lib/eagle/components/wire.js';
import { Instance } from './lib/eagle/components/instance.js';
import { SchematicSymbol } from './lib/eagle/components/symbol.js';

/* prettier-ignore */ import { BoardRenderer, DereferenceError, EagleDocument, EagleElement, EagleNode, EagleNodeList, EagleNodeMap, EagleProject, EagleRef, EagleReference, EagleSVGRenderer, Renderer, SchematicRenderer, makeEagleElement, makeEagleNode
 } from './lib/eagle.js';
/* prettier-ignore */ const React = {Component, createContext, create: h, html, render, useCallback, useContext, useDebugValue, useEffect, useImperativeHandle, useLayoutEffect, useMemo, useReducer, useRef, useState };
/* prettier-ignore */ const { Align, Anchor, CSS, Event, CSSTransformSetters, Element, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementTransformation, ElementWHProps, ElementXYProps, isElement, isLine, isMatrix, isNumber, isPoint, isRect, isSize, Line, Matrix, Node, Point, PointList, Polyline, Rect, Select, Size, SVG, Timer, Transition, TransitionList, TRBL, Tree } = {...dom, ...geom };
Util.colorCtor = ColoredText;
/* prettier-ignore */ Util.extend(window, { React, ReactComponent, WebSocketClient, html }, { dom, keysim }, geom, { Iterator, Functional }, { EagleNodeList, EagleNodeMap, EagleDocument, EagleReference, EagleNode, EagleElement }, { toXML, XmlObject, XmlAttr }, {CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token, ReactComponent, ClipperLib, Shape, isRGBA, RGBA, ImmutableRGBA, isHSLA, HSLA, ImmutableHSLA, ColoredText, Alea, Message }, { Chooser, useState, useLayoutEffect, useRef, Polygon } );

Error.stackTraceLimit = 100;

let currentProj = trkl.property(window, 'project');
let open = trkl();
let showSearch = trkl(true);
let logSize = trkl({});
let dump = trkl({});

let projectName = 'Headphone-Amplifier-ClassAB-alt3';
let palette = null;
let svgElement;
let brdXml, schXml, brdDom, schDom;
let board, schematic;
let loadedProjects = [];
let zoomVal = 0;
let container;

let projectFiles;
let activeFile;
let transform = trkl(new TransformationList());
let sizeListener = trkl({});
let aspectListener = trkl(1);
let debug = false;
const documentTitle = trkl('');

let store = (window.store = makeLocalStorage());

const add = (arr, ...items) => [...(arr ? arr : []), ...items];

const useSlot = (arr, i) => [() => arr[i], v => (arr[i] = v)];
const trklGetSet = (get, set) => value => (value !== undefined ? set(value) : get());
const useTrkl = trkl => [() => trkl(), value => trkl(value)];

const classNames = (...args) => args.filter(arg => typeof arg == 'string' && arg.length > 0).join(' ');

const MouseEvents = h => ({
  onMouseDown: h,

  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h
});
//console.log('running');
//console.log("dom", { Rect, Element });

window.dom = { Element, SVG };

/* prettier-ignore */
/*    const CreateSelect = (obj, node = document.body) => {
                            let elem = Select.create(Object.entries(obj));
                            node.insertBefore(elem, node.firstElementChild);
                          };*/
const utf8Decoder = new TextDecoder('utf-8');

const ListProjects = (window.list = async function(url = '/files.html') {
  let response = await fetch(url, {
    method: 'POST',
    mode: 'cors', // no-cors, *cors, same-origin
    cache: 'no-cache', // *default, no-cache, reload, force-cache, only-if-cached
    credentials: 'same-origin', // include, *same-origin, omit
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ descriptions: true })
  });
  const reader = await (await response.body).getReader();
  let { value: chunk, done: readerDone } = await reader.read();
  chunk = chunk ? await utf8Decoder.decode(chunk) : '';
  return chunk;
});

const ElementToXML = (e, predicate) => {
  const x = Element.toObject(e, { predicate });
  //console.log('x:', x);
  return Element.toString(x, { newline: '\n' });
};

const FileSystem = {
  async readFile(filename) {
    return await fetch(`/static/${filename}`).then(async res => await (await res).text());
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

const LoadFile = async filename => {
  let xml = await fetch(`/static/${filename}`).then(async res => await (await res).text());
  //console.log('xml: ', xml.substring(0, 100));
  //let dom = new DOMParser().parseFromString(xml, 'application/xml');

  let doc = new EagleDocument(xml, null, filename, null, FileSystem);

  if(/\.brd$/.test(filename)) window.board = doc;
  if(/\.sch$/.test(filename)) window.schematic = doc;
  if(/\.lbr$/.test(filename)) window.libraries = add(window.libraries, doc);
  Util.log('LoadFile', doc.file);

  return doc;
};

const SaveSVG = (window.save = async function save(filename = projectName, layers = [1, 16, 20, 21, 22, 23, 25, 27, 47, 48, 51]) {
  let predicate = element => {
    if(!element.hasAttribute('data-layer')) return true;
    console.log('element:', element);
    const layer = element.getAttribute('data-layer');
    let [number, name] = layer.split(/ /);
    if(number !== undefined && name !== undefined) return layers.indexOf(+number) != -1 || layers.indexOf(name) != -1;
    return true;
  };
  let data = ElementToXML(project.svg, predicate);

  let { status, statusText, body } = await fetch('/save', {
    method: 'post',
    headers: {
      'Content-Type': 'application/xml',
      'Content-Disposition': `attachment; filename="${filename.replace(/\.svg$/i, '')}.svg"`
    },
    body: data
  });
  const result = { status, statusText, body };
  console.log('saved', result);
  return result;
});

const ModifyColors = fn => e => {
  const { type, buttons } = e;
  if(type.endsWith('down')) {
    if(!window.c) window.c = SVG.allColors(project.svg);
    let { c } = window;
    //console.log('ModifyColors', fn);

    c.dump();
    fn(c);
  }
};

const LoadDocument = async (project, parentElem) => {
  Util.log('project.name:', project.name);

  project.doc = await LoadFile(project.name);

  documentTitle(project.doc.file.replace(/.*\//g, ''));

  window.eagle = project.doc;
  window.project = project;

  Element.remove('#fence');

  let docElem = Element.find('#doc');
  docElem.innerHTML = '';

  Util.log('project.doc:', project.doc.file);

  project.renderer = new Renderer(project.doc, ReactComponent.append, debug);
  Util.log('project.renderer', project.renderer);

  let style = { width: '100%', height: '100%', position: 'relative' };
  let component = project.renderer.render(project.doc, null, {});

  window.component = project.component = component;

  Util.log('testRender:', component);

  let element = Element.find('#main');
  let r = project.renderer.rect || project.renderer.bounds;

  Util.log('project.renderer:', project.renderer);
  Util.log('r:', r);

  let aspectRatio = 1;

  if(r) {
    aspectRatio = r.width / r.height;
    sizeListener({ width: r.width });
  }

  aspectListener(aspectRatio);

  const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
    const [dimensions, setDimensions] = useState(sizeListener());
    const [aspect, setAspect] = useState(aspectListener());

    if(sizeListener && sizeListener.subscribe) sizeListener.subscribe(value => setDimensions(value));
    if(aspectListener && aspectListener.subscribe) aspectListener.subscribe(value => setAspect(value));

    return h(
      TransformedElement,
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

  component = h(
    Fence,
    {
      style: {},
      sizeListener,
      aspectListener
    },
    [component]
  );

  React.render(component, element);

  let object = ReactComponent.toObject(component);
  project.object = object;
  let rendered = object.children[0];
  Util.log('rendered:', rendered);
  /*
  for(let [item, path] of deep.iterate(object, v => Util.isObject(v) && v['data-path'])) {
    let p = path.reduce((a, i) => (i == 'children' ? [...a, 'props', 'children'] : [...a, +i]), []); //, {tagField: 'type', specialFields: ['props']});
    let o = path.slice(0, 4 * 2 - 1).reduce((a, i) => a && a[i], object);
    let c = p.slice(0, 4 * 3 - 1).reduce((a, i) => a && a[i], component);
  }
*/
  let eagle2dom = [...Element.findAll('*[data-path]')];

  eagle2dom = eagle2dom.map(e => [e.getAttribute('data-path'), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [new ImmutablePath(p), e]);
  eagle2dom = eagle2dom.map(([p, e]) => [p, p.apply(project.doc.raw), e]);
  eagle2dom = eagle2dom.map(([p, r, e]) => [EagleElement.get(project.doc, p, r), e]);

  //  console.log('eagle2dom:', eagle2dom);

  let dom2eagle = Util.mapFunction(new WeakMap(eagle2dom.map(([k, v]) => [v, k])));

  eagle2dom = Util.mapFunction(new WeakMap(eagle2dom));

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
  let rect = bounds.rect;
  let size = new Size(r);
  currentProj(project);
  size.mul(doc.type == 'brd' ? 2 : 1.5);
  let svgrect = SVG.bbox(project.svg);

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
    console.error(
      'Render ERROR:',
      err,
      [...err.stack].map(f => (f + '').replace(Util.getURL() + '/', ''))
    );
  }*/

  return project;
};

const ChooseDocument = async (e, proj, i) => {
  let r;
  const { type } = e;
  const box = Element.findAll('.file')[i];
  Util.log('ChooseDocument:', { e, proj, i, box });

  try {
    if(!proj.loaded) {
      let data = await LoadDocument(proj, box);
      proj.loaded = true;

      open(false);

      //console.log('loaded:', proj);
    }
    r = proj.loaded;
  } catch(err) {
    Util.putError(err);
  }

  return r;
};

const MakeFitAction = index => async () => {
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
  let transform = affineTransform.decompose();
  //console.log(`fitAction(${index})`, { oldSize, newSize, transform });
  let factor = transform.scale.x;
  //console.log('zoom factor:', factor);
  let delay = Math.abs(Math.log(factor) * 1000);
  //console.log('transition delay:', delay);
  await Element.transition(container, { ...newSize.toCSS(), transform: '', position: 'absolute' }, delay + 'ms', 'linear');
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

  Util.log('New WebSocket:', ws);
  await ws.connect(socketURL);
  Util.log('WebSocket Connected:', ws.connected);
  socketFn(ws);
  ws.send('main.js data!');
  let data;
  for await (data of ws) {
    let msg = new Message(data);
    window.msg = msg;
    Util.log('WebSocket data:', msg[Symbol.toStringTag]());
    ws.dataAvailable !== 0;
  }
  await ws.disconnect();
};

let projects = trkl([]);
let socket = trkl();

const BindGlobal = Util.once(arg => trkl.bind(window, arg));

const AppMain = (window.onload = async () => {
  Object.assign(
    window,
    { LogJS },
    { Element, devtools, dom,RGBA,HSLA },
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
  Object.assign(window, { SaveSVG });

  Error.stackTraceLimit = 100;

  const timestamps = new Repeater(async (push, stop) => {
    push(Date.now());
    const interval = setInterval(() => push(Date.now()), 1000);
    await stop;
    clearInterval(interval);
  });

  const logger = new Repeater(async (push, stop) => {
    push('Load ready!');
    window.pushlog = push;
    await stop;
  });
  logger.push = window.pushlog;

  const RegisterEventHandler = (events = []) =>
    new Repeater(async (push, stop) => {
      let handler = e => push(e);
      events.forEach(ev => window.addEventListener(ev, handler));
      console.log('registered');
      await stop;
      events.forEach(ev => window.removeEventListener(ev, handler));
      console.log('unregistered');
    });
  const touchEvents = RegisterEventHandler(['touchmove', 'touchstart', 'touchcancel', 'mousemove', 'mouseup', 'mousedown']);

  //window.focusSearch = trkl();
  window.currentSearch = trkl(null);

  window.keystroke = target => (key, modifiers = 0) => keysim.Keyboard.US_ENGLISH.dispatchEventsForKeystroke(new keysim.Keystroke(modifiers, key), target);

  window.focusSearch = state => {
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

  currentSearch.subscribe(value => {
    if(value) {
      focusSearch(false);

      setTimeout(() => {
        //console.log('currentSearch:', value);
        focusSearch(true);
      }, 1000);
    }
  });

  Util(globalThis);

  //prettier-ignore
  Object.assign(window, { BBox, ChooseDocument, classNames, ColorMap, components, CSS, deep, EagleDocument, EagleElement, EagleNode, ImmutablePath, ImmutableXPath, EagleReference, eventIterator, h, HSLA, html, isLine, isPoint, isRect, isSize, iterator, Line, LoadDocument, LoadFile, Matrix, MatrixTransformation, ModifyColors, Point, PointList, React, Rect,  Rotation, Scaling, Size, SVG, Transformation, TransformationList, Translation, tXml, Util, MouseEvents, ElementToXML, LoadFile, ModifyColors, MakeFitAction, CreateWebSocket, AppMain, Canvas });
  Object.assign(window, {
    PrimitiveComponents,
    ElementNameToComponent,
    ElementToComponent,
    Wire,
    Instance,
    SchematicSymbol
  });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = props =>
    html`
      <div>This is a test</div>
    `;

  let c = testComponent({});
  window.testComponent = c;

  ListProjects('/files.html').then(response => {
    let data = JSON.parse(response);
    let { files } = data;
    //console.log(`Got ${files.length} files`);
    function File(obj, i) {
      const { name } = obj;
      let file = this instanceof File ? this : Object.create(File.prototype);
      let data = trkl({ percent: NaN });
      Object.assign(file, obj);
      file.name = name;
      file.i = i;
      trkl.bind(file, { data });

      return file;
    }
    File.prototype.toString = function() {
      return this.name;
    };
    projectFiles = window.files = files.sort((a, b) => a.name.localeCompare(b.name)).map((obj, i) => new File(obj, i));
    projects(projectFiles);
  });

  CreateWebSocket(null, null, ws => (window.socket = ws));

  const searchFilter = trkl(store.get('filter') || '*');
  const crosshair = {
    show: trkl(false),
    position: trkl({ x: 0, y: 0 })
  };

  window.crosshair = trkl.bind({}, crosshair);

  searchFilter.subscribe(value => {
    store.set('filter', value);
    Util.log('searchFilter is ', value);
  });

  const changeInput = e => {
    const { target } = e;
    Util.log('changeInput:', target.value);

    let { value } = target;

    searchFilter(value == '' ? '*' : value.split(/\s*\|\s*/g).join(' | '));
  };

  const Consumer = props => {
    const result = useResult(async function*() {
      for await (let time of timestamps) {
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
        if(typeof window.pushlog == 'function') window.pushlog(type + ' ' + Util.isoDate(d).replace(/-/g, '') + ' ' + d.toLocaleTimeString(navigator.language || 'de') + ' ' + msg);
      }
    }
  );
  let loggerRect = new Rect();
  const Logger = props => {
    const [lines, setLines] = useState([]);

    const [ref, { x, y, width, height }] = useDimensions();

    const r = new Rect({ x, y, width, height });
    if(!loggerRect.equals(r)) {
      console.log('Logger.dimensions:', r);
      loggerRect = r;
      logSize({ width });
    }

    const result = useResult(async function*() {
      for await (let msg of logger) yield msg;
    });
    if(result) lines.push(result.value);
    return h(
      'div',
      { className: 'logger', ref },
      lines.slice(-10, lines.length).map((l, i) => h('p', {}, /*333*/ l + ''))
    );
  };
  dump({ ...dump(), test: 123 });

  const Dumper = props => {
    const [values, setValues] = useState(dump());
    let lines = [];

    dump.subscribe(value => setValues(value));

    for(let [key, value] of Object.entries(values)) {
      lines.push([key, value]);
    }
    return h(
      'table',
      { border: '0',cellpadding: 3  ,cellspacing: 0, className: 'dumper' },
      lines.map(([k,v]  , i) => h('tr', { className: 'watch' }, [h('td', { className: 'name'}, k + ''), h('td',  { className: 'value'}, v + '')]))
    );
  };

  React.render(
    [
      Panel('buttons', [
        h(Button, {
          caption: BrowseIcon(),
          fn: e => {
            if(e.type.endsWith('down')) {
              //console.log('file list push', e);
              open(!open());
            }
          }
        }),
        h(Button, {
          caption: 'Random',
          fn: ModifyColors(c => c.replaceAll(c => HSLA.random()))
        }),
        h(Button, {
          caption: 'Invert',
          fn: ModifyColors(c => c.replaceAll(c => c.invert()))
        }),
        h(Button, {
          caption: '↔',
          fn: MakeFitAction(0)
        }),
        h(Button, {
          caption: '↕',
          fn: MakeFitAction(1)
        }),
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
      html`
        <${FileList} files=${projects} onActive=${open} onChange=${ChooseDocument} filter=${searchFilter} showSearch=${showSearch} changeInput=${changeInput} focusSearch=${focusSearch} currentInput=${currentSearch} />
      `,
      h(CrossHair, { ...crosshair }),
      h(FloatingPanel, { onSize: logSize }, [h(Logger, {}), h(Dumper, {})])
    ],
    Element.find('#preact')
  );

  let move;
  container = Element.find('#main');

  TouchListener(
    event => {
      const { x, y, index, buttons, start, type, target } = event;

      if(type.endsWith('end') || type.endsWith('up')) return cancel();
      if(event.buttons === 0 && type.endsWith('move')) return cancel();
      // if(event.index > 0) Util.log('touch', { x, y, index, buttons, type, target }, container);
      if(event.buttons & 2) return cancel();
      if(!move) {
        let box = Element.find('#main').firstElementChild;
        window.move = move = Element.moveRelative(box);
      } else if(move && event.buttons == 0) {
        cancel();
      } else if(event.index > 0) {
        let rel = new Point(event);
        let absolute = new Point(start).add(rel);

        if(move) {
          window.crosshair.show = true;
          window.crosshair.position = absolute;

          //          Util.log('move', ...[...rel], ...[...absolute]);
          if(true || event.buttons > 0) move(rel.x, rel.y);
          else move = move.jump();
        }
      }
      function cancel() {
        move = null;
        window.crosshair.show = false;

        return event.cancel();
      }
    },
    { element: window }
  );

  window.processEvents = async function eventLoop() {
    for await (let e of touchEvents) {
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
        //  currentTarget,
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
      LogJS.info(`${type} ` + /* Util.toSource(e)+ */ ` ${x},${y} → ${Element.xpath(target)}`);
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
    zoomVal = altKey || ctrlKey || shiftKey ? 0 : Util.clamp(-100, 100, zoomVal + wheelPos * 0.1);
    const zoom = Math.pow(10, zoomVal / 100).toFixed(5);

    let t = window.transform;

    if(!t.scaling) t.scale(zoom, zoom);
    else {
      t.scaling.x = zoom;
      t.scaling.y = zoom;
    }

    window.transform = new TransformationList(t);
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
