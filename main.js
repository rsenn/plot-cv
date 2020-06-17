// prettier-ignore-start
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom from './lib/dom.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { iterator, eventIterator } from './lib/dom/iterator.js';
import geom from './lib/geom.js';
import { BBox } from './lib/geom/bbox.js';
import { ScrollDisabler } from './lib/scrollHandler.js';
import { TouchListener } from './lib/touchHandler.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './modules/htm/preact/standalone.mjs';
import components, { Chooser, Container, Button, FileList, Panel, AspectRatioBox, SizedAspectRatioBox, TransformedElement, Canvas, ColorWheel, Slider } from './static/components.js';
import { WebSocketClient } from './lib/websocket-client.js';
import { CTORS, ECMAScriptParser, estree, Factory, Lexer, ESNode, Parser, PathReplacer, Printer, Stack, Token } from './lib/ecmascript.js';
import {
  AlignmentAngle,
  Arc,
  BoardRenderer,
  CalculateArcRadius,
  ClampAngle,
  DereferenceError,
  EagleDocument,
  EagleElement,
  EagleInterface,
  EagleNode,
  EagleNodeList,
  EagleNodeMap,
  EaglePath,
  EagleProject,
  EagleRef,
  EagleReference,
  EagleSVGRenderer,
  HORIZONTAL,
  HORIZONTAL_VERTICAL,
  InvertY,
  LayerAttributes,
  LinesToPath,
  MakeCoordTransformer,
  PolarToCartesian,
  Renderer,
  RotateTransformation,
  SchematicRenderer,
  VERTICAL,
  makeEagleElement,
  makeEagleNode,
  makeEagleNodeList,
  makeEagleNodeMap,
  renderDocument
} from './lib/eagle.js';

const React = { cloneElement, Component, createContext, createRef, Fragment, create: h, html, hydrate, isValidElement, render, toChildArray, useCallback, useContext, useDebugValue, useEffect, useImperativeHandle, useLayoutEffect, useMemo, useReducer, useRef, useState };
const { Align, Anchor, CSS, CSSTransformSetters, Element, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementTransformation, ElementWHProps, ElementXYProps, HSLA, isElement, isHSLA, isLine, isMatrix, isNumber, isPoint, isRect, isRGBA, isSize, Line, Matrix, Node, Point, PointList, Polyline, Rect, RGBA, Select, Size, SVG, Timer, Transition, TransitionList, TRBL, Tree } = {
  ...dom,
  ...geom
};
Object.assign(window, { React, ReactComponent, WebSocketClient, html }, dom, geom, { CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token, ReactComponent }, { Chooser });

let currentProj = trkl.property(window, 'project');
let open = trkl();

let projectName = 'Headphone-Amplifier-ClassAB-alt3';
let palette = null;
let svgElement;
let brdXml, schXml, brdDom, schDom;
let board, schematic;
let loadedProjects = [];
let zoomVal = 0;
let container;

let projects, projectFiles;
let activeFile;
let transform = trkl(new TransformationList());
let sizeListener = trkl({});
let aspectListener = trkl(1);

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
console.log('running');
//console.log("dom", { Rect, Element, parseSchematic });

window.dom = { Element, SVG };
/* prettier-ignore */
/*    const CreateSelect = (obj, node = document.body) => {
                            let elem = Select.create(Object.entries(obj));
                            node.insertBefore(elem, node.firstElementChild);
                          };*/
const utf8Decoder = new TextDecoder("utf-8");

const ListProjects = (window.list = async function(url) {
  let response = await fetch('/files.html', { method: 'get' });
  const reader = await (await response.body).getReader();
  let { value: chunk, done: readerDone } = await reader.read();
  chunk = chunk ? await utf8Decoder.decode(chunk) : '';
  return chunk;
});

const ElementToXML = e => {
  const x = Element.toObject(e);
  console.log('x:', x);
  return Element.toString(x);
};

const LoadFile = async filename => {
  let xml = await fetch(`/static/${filename}`).then(async res => {
    return await (await res).text();
  });
  //console.log('xml: ', xml.substring(0, 100));
  //let dom = new DOMParser().parseFromString(xml, 'application/xml');

  let doc = new EagleDocument(xml);

  if(/\.brd$/.test(filename)) window.board = doc;
  if(/\.sch$/.test(filename)) window.schematic = doc;

  return doc;
};

const SaveSVG = (window.save = async function save(filename = projectName) {
  let body = ElementToXML(window.svg);
  let result = await fetch('/save', {
    method: 'post',
    headers: {
      'Content-Type': 'application/xml',
      'Content-Disposition': `attachment; filename="${projectName}.svg"`
    },
    body
  });
  console.log('saved', result);
});

const ModifyColors = fn => e => {
  const { type, buttons } = e;
  if(type.endsWith('down')) {
    if(!window.c) window.c = SVG.allColors(project.svg);
    let { c } = window;
    console.log('ModifyColors', fn);

    c.dump();
    fn(c);
  }
};

const loadDocument = async (proj, parentElem) => {
  console.log(`load project #${proj.i}:`, proj);
  proj.doc = await LoadFile(proj.name);
  window.eagle = proj.doc;
  window.project = proj;

  Element.remove('#fence');

  proj.renderer = new Renderer(proj.doc, ReactComponent.append);

  if(!proj.renderer || !proj.renderer.render) return;

  let style = { width: '100%', height: '100%', position: 'relative' };
  let svgXml = proj.renderer.render(proj.doc, null, { style });
  console.log('testRender:', svgXml);
  let component = proj.renderer.render(proj.doc, null, { style });
  window.component = component;

  let element = Element.find('#main');
  console.log('h', h);
  console.log('component', component);

  let r = proj.renderer.rect || proj.renderer.bounds;
  console.log('r', r);
  let aspectRatio = r.width / r.height;
  console.log('aspectRatio', aspectRatio);

  aspectListener(aspectRatio);

  const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
    const [dimensions, setDimensions] = useState(sizeListener());
    const [aspect, setAspect] = useState(aspectListener());

    if(sizeListener && sizeListener.subscribe) sizeListener.subscribe(value => setDimensions(value));
    if(aspectListener && aspectListener.subscribe) aspectListener.subscribe(value => setAspect(value));

    console.log('Fence.render', { dimensions, aspect });

    return h(TransformedElement, { id: 'fence', type: SizedAspectRatioBox, aspect, listener: transform, style: { position: 'relative', minWidth: '100px', 'data-name': proj.name, ...style, ...dimensions }, ...props }, children);
  };

  component = h(
    Fence,
    {
      style: {
        /*border: '0.001em dashed red'*/
      },
      sizeListener,
      aspectListener
    },
    [component]
  );

  React.render(component /*html`<${Fence}>${component}</${Fence}>`*/, element);

  let rendered = [...element.children];

  window.rendered = rendered;
  console.log('window.rendered', window.rendered);
  proj.element = rendered[0];
  proj.svg = Element.find('svg', '#main');
  proj.grid = Element.find('g.grid', proj.element);
  proj.bbox = SVG.bbox(proj.grid);
  proj.aspectRatio = aspect;
  console.log('proj.svg', proj.svg);
  console.log('project', proj);

  let { name, data, doc, svg, bbox } = proj;
  let bounds = doc.getBounds();
  let rect = bounds.rect;
  let size = new Size(rect);
  currentProj(proj);
  size.mul(doc.type == 'brd' ? 2 : 1.5);
  let svgrect = SVG.bbox(proj.svg);

  //  proj.aspectRatio = svgrect.aspect();

  Element.attr(proj.svg, {
    'data-filename': proj.name,
    'data-aspect': proj.aspectRatio,
    'data-width': size.width + 'mm',
    'data-height': size.height + 'mm'
  });

  // proj.svg.setAttribute('data-aspect', proj.aspectRatio);
  let css = size.div(0.26458333333719).toCSS({ width: 'px', height: 'px' });

  window.size = css;
  //  console.log("css:", css);
  /*  Object.assign(proj.svg.style, {
    'min-width': `${size.width}mm`
  });
  Element.setCSS(proj.svg, { left: 0, top: 0, position: 'relative' });
  Element.setCSS(proj.svg, { left: 0, top: 0, position: 'relative' });
  console.log('loadDocument:', proj.svg);*/
  return proj;
};

const chooseDocument = async (e, proj, i) => {
  let r;
  try {
    const { type } = e;
    const box = Element.findAll('.file')[i];
    console.log('chooseDocument:', { e, proj, i, box });
    if(!proj.loaded) {
      let data = await loadDocument(proj, box);
      proj.loaded = true;

      open(false);

      console.log('loaded:', proj);
    }
    r = proj.loaded;
  } catch(err) {
    console.log('err:', err.message, err.stack);
  }

  return r;
};

const MakeFitAction = index => async () => {
  let parent = Element.find('#main');
  let prect = Element.rect(parent);
  let svg = Element.find('svg', parent);
  let container = [...Element.findAll('.aspect-ratio-box-size', parent)].reverse()[0];
  console.log('container:', container);
  let oldSize = Element.rect(container);
  let brect = Element.rect('.buttons');
  let srect = Element.rect(svg);
  prect.y += brect.height;
  prect.height -= brect.height;
  let rects = [prect, oldSize, srect];
  prect.scale(0.8);
  console.log('resize rects', { oldSize, prect, srect });
  let f = srect.fit(prect);
  let newSize = f[index].round(0.0001);
  let affineTransform = Matrix.getAffineTransform(oldSize.toPoints(), newSize.toPoints());
  let transform = affineTransform.decompose();
  console.log(`fitAction(${index})`, { oldSize, newSize, transform });
  let factor = transform.scale.x;
  console.log('zoom factor:', factor);
  let delay = Math.abs(Math.log(factor) * 1000);
  console.log('transition delay:', delay);
  await Element.transition(container, { ...newSize.toCSS(), transform: '', position: 'absolute' }, delay + 'ms', 'linear');
};

const CreateWebSocket = async (socketURL, log, socketFn = () => {}) => {
  log = log || ((...args) => console.log(...args));
  socketURL = socketURL || Util.makeURL({ location: '/ws', protocol: 'ws' });
  let ws = new WebSocketClient();
  log('New WebSocket:', ws);
  await ws.connect(socketURL);
  log('Connected:', ws.connected);
  socketFn(ws);
  ws.send('hello!');
  let data;
  for await (data of ws) {
    log('WebSocket data:', data);
    ws.dataAvailable !== 0;
  }
  await ws.disconnect();
};

const AppMain = (window.onload = async () => {
  Object.assign(window, { Element, devtools, dom });
  let projects = trkl([]);
  let socket = trkl();
  trkl.bind(window, { projects, socket, transform, size: sizeListener, aspect: aspectListener });

  Util(globalThis);

  // prettier-ignore
  Object.assign(window, {BBox, chooseDocument, classNames, ColorMap, components, CSS, deep, EagleDocument, EagleElement, EagleInterface, EagleNode, EaglePath, EagleReference, eventIterator, h, HSLA, html, isLine, isPoint, isRect, isSize, iterator, Line, loadDocument, LoadFile, Matrix, MatrixTransformation, ModifyColors, Point, PointList, React, Rect, RGBA, Rotation, Scaling, Size, SVG, Transformation, TransformationList, Translation, tXml, Util, MouseEvents, ElementToXML, LoadFile, ModifyColors, MakeFitAction, CreateWebSocket, AppMain, Canvas });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = props =>
    html`
      <div>This is a test</div>
    `;

  let c = testComponent({});
  window.testComponent = c;
  console.log('testComponent', ReactComponent.toObject(c));

  /*console.realLog = console.log;
  console.log = function(...args) {
    let out = [''];
    for(let arg of args) {
      if(typeof arg != 'string') {
        if(arg[inspectSym]) {
          out = concat(out, arg[inspectSym]());
          continue;
        } else if(arg.toString && !Util.isNativeFunction(arg.toString)) {
          //  console.realLog("toString: "+arg.toString);
          out = concat(out, [arg.toString()]);
        } else if(Util.isObject(arg)) {
          out.push(arg);
          continue;
          out[0] += Util.inspect(arg, { indent: '', newline: '', depth: 2, spacing: '' });
        }
      }
      out[0] += ' ' + arg;
    }
    //  for(let i in out)
    //  console.realLog("out:",out);
    this.realLog(...out);
  };*/

  ListProjects('/files.html').then(response => {
    let data = JSON.parse(response);
    let { files } = data;
    console.log(`Got ${files.length} files`);
    projectFiles = window.files = files;
    projects(
      projectFiles.map(({ name }, i) => {
        let data = trkl({ percent: NaN });
        let proj = { name, i };
        trkl.bind(proj, { data });
        return proj;
      })
    );
  });

  CreateWebSocket(null, null, ws => (window.socket = ws));

  React.render(
    [
      Panel('buttons', [
        h(Button, {
          caption: '📂',
          fn: e => {
            if(e.type.endsWith('down')) {
              console.log('file list push', e);
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
        })
      ]),
      h(ColorWheel, {}),
      h(Slider, { min: 0, max: 100, orient: 'horizontal', name: 'saturation', length: '100px',  onChange: value => { console.log("value:", value); } }),
      h(Slider, { min: 0, max: 100, orient: 'vertical', name: 'lightness',  length: '100px', onChange: value => { console.log("value:", value); } }),
      html`
        <${FileList} files=${projects} onActive=${open} onChange=${chooseDocument} />
      `
    ],
    Element.find('#preact')
  );

  let move;
  container = Element.find('#main');

  TouchListener(
    event => {
      //  if(event.index > 0 && event.buttons > 0) console.log('touch', event, container);
      if(!move) {
        let container = Element.find('#main');

        move = Element.moveRelative(container);
      } else if(event.index > 0) {
        let rel = new Point(event);
        if(move) {
          if(event.buttons > 0) move(rel.x, rel.y);
          else move = move.jump();
        }
      }
    },
    { element: window }
  );

  window.styles = CSS.create('head');

  window.addEventListener('wheel', event => {
    //console.log("event:",event);
    const clientArea = Element.rect('body > div');
    clientArea.x += container.parentElement.scrollLeft;
    const clientCenter = clientArea.center;
    const { clientX, clientY, target, currentTarget, buttons, altKey, ctrlKey, shiftKey } = event;
    const pos = new Point(clientX, clientY);
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
    /*    const transform = ` scale(${zoom},${zoom}) `;
    const origin = `${pos.toString(1, 'px', ' ')}`;
    let list = [...Element.skip(target)].find(p => p.classList.contains('list'));
    if(false && list) {
      let parent = list.parentElement;
      let rects = [Element.rect(parent), Element.rect(list)];
      let screen = new Rect(0, 0, window.innerWidth, window.innerHeight);
      rects[0].y2 = screen.y2;
      rects[1].y2 = screen.y2;
      Element.setRect(list, rects[1]);
    } else {
      window.transform;
      Element.setCSS(container, { transform });
      Element.setCSS(container, { 'transform-origin': origin });
    }*/
  });

  console.log(Util.getGlobalObject());

  for(let path of [...Element.findAll('path')]) {
    let points = new PointList([...SVG.pathIterator(path, 30, p => p.toFixed(3))]);
  }
});

const Module = {
  noInitialRun: true,
  onRuntimeInitialized: () => {
    console.log('initialized');
    let myString = prompt('Enter a string:');
    Module.callMain([myString]);
  },
  print: txt => alert(`The MD5 hash is: ${txt}`)
};
