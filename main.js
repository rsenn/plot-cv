// prettier-ignore-start
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
import { Iterator } from './lib/iterator.js';
import { Functional } from './lib/functional.js';
import { makeLocalStorage } from './lib/autoStore.js';

import { toXML, ImmutablePath } from './lib/json.js';
import { XmlObject, XmlAttr, ImmutableXPath } from './lib/xml.js';
import { RGBA, isRGBA, HSLA, isHSLA } from './lib/color.js';
import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './modules/htm/preact/standalone.module.js';
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
  EagleNode,
  EagleNodeList,
  EagleNodeMap,
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
  makeEagleNode
} from './lib/eagle.js';

const React = {
  cloneElement,
  Component,
  createContext,
  createRef,
  Fragment,
  create: h,
  html,
  hydrate,
  isValidElement,
  render,
  toChildArray,
  useCallback,
  useContext,
  useDebugValue,
  useEffect,
  useImperativeHandle,
  useLayoutEffect,
  useMemo,
  useReducer,
  useRef,
  useState
};
const { Align, Anchor, CSS, Event, CSSTransformSetters, Element, ElementPosProps, ElementRectProps, ElementRectProxy, ElementSizeProps, ElementTransformation, ElementWHProps, ElementXYProps, isElement, isLine, isMatrix, isNumber, isPoint, isRect, isSize, Line, Matrix, Node, Point, PointList, Polyline, Rect, Select, Size, SVG, Timer, Transition, TransitionList, TRBL, Tree } = {
  ...dom,
  ...geom
};
Util.extend(
  window,
  { React, ReactComponent, WebSocketClient, html },
  { dom, keysim },
  geom,
  { Iterator, Functional },
  { EagleNodeList, EagleNodeMap, EagleDocument, EagleReference, EagleNode, EagleElement },
  { toXML, XmlObject, XmlAttr },
  {
    CTORS,
    ECMAScriptParser,
    ESNode,
    estree,
    Factory,
    Lexer,
    Parser,
    PathReplacer,
    Printer,
    Stack,
    Token,
    ReactComponent,
    ClipperLib,
    Shape,
    RGBA,
    isHSLA
  },
  { Chooser, useState, useLayoutEffect, useRef, Polygon }
);

  Error.stackTraceLimit = 100;

let currentProj = trkl.property(window, 'project');
let open = trkl();
let showSearch = trkl(true);

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

let store = (window.store = makeLocalStorage());

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

const ListProjects = (window.list = async function(url) {
  let response = await fetch('/files.html', { method: 'get' });
  const reader = await (await response.body).getReader();
  let { value: chunk, done: readerDone } = await reader.read();
  chunk = chunk ? await utf8Decoder.decode(chunk) : '';
  return chunk;
});

const ElementToXML = e => {
  const x = Element.toObject(e);
  //console.log('x:', x);
  return Element.toString(x);
};

const LoadFile = async filename => {
  let xml = await fetch(`/static/${filename}`).then(async res => await (await res).text());
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
  //console.log('saved', result);
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

const loadDocument = async (project, parentElem) => {
  //console.log(`load project #${project.i}:`, project);
  project.doc = await LoadFile(project.name);

  //console.log('project.doc', project.doc);

  window.eagle = project.doc;
  window.project = project;

  Element.remove('#fence');

  let docElem = Element.find('#doc');
  docElem.innerHTML = '';

  /*  let docNode = Util.parseXML(project.doc.toXML());
  let eagleNode = docNode.firstElementChild;
  docNode.removeChild(eagleNode);
  docElem.appendChild(eagleNode);*/

  project.renderer = new Renderer(project.doc, ReactComponent.append, false);
  //console.log('project.renderer', project.renderer);

  // if(!project.renderer || !project.renderer.render) return;

  let style = { width: '100%', height: '100%', position: 'relative' };
  /*  let svgXml = project.renderer.render(project.doc, null, {});
  //console.log('testRender:', svgXml);*/
  let component = project.renderer.render(project.doc, null, {});
  window.component = component;
  project.component = component;

  let element = Element.find('#main');
  //console.log('h', h);
  //
  let r = project.renderer.rect || project.renderer.bounds;
  //console.log('r', r);
  let aspectRatio = r.width / r.height;
  //console.log('aspectRatio', aspectRatio);

  sizeListener({ width: r.width });
  aspectListener(aspectRatio);

  const Fence = ({ children, style = {}, sizeListener, aspectListener, ...props }) => {
    const [dimensions, setDimensions] = useState(sizeListener());
    const [aspect, setAspect] = useState(aspectListener());

    if(sizeListener && sizeListener.subscribe) sizeListener.subscribe(value => setDimensions(value));
    if(aspectListener && aspectListener.subscribe) aspectListener.subscribe(value => setAspect(value));

    //console.log('Fence.render', { dimensions, aspect });

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
      style: {
        /*border: '0.001em dashed red'*/
      },
      sizeListener,
      aspectListener
    },
    [component]
  );

  React.render(component, element);

  let rendered = [...element.children];

  window.rendered = rendered;
  //console.log('window.rendered', window.rendered);
  project.element = rendered[0];
  project.svg = Element.find('svg', '#main');
  project.grid = Element.find('g.grid', project.element);
  project.bbox = SVG.bbox(project.grid);
  project.aspectRatio = aspect;
  //console.log('project.svg', project.svg);
  //console.log('project', project);

  let { name, data, doc, svg, bbox } = project;
  let bounds = doc.getBounds();
  let rect = bounds.rect;
  let size = new Size(r);
  currentProj(project);
  size.mul(doc.type == 'brd' ? 2 : 1.5);
  let svgrect = SVG.bbox(project.svg);

  //  project.aspectRatio = svgrect.aspect();

  Element.attr(project.svg, {
    'data-filename': project.name,
    'data-aspect': project.aspectRatio,
    'data-width': size.width + 'mm',
    'data-height': size.height + 'mm'
  });

  // project.svg.setAttribute('data-aspect', project.aspectRatio);
  let css = size.div(0.26458333333719).toCSS({ width: 'px', height: 'px' });

  window.size = css;
  //console.log('css:', css);
  /*  Object.assign(project.svg.style, {
    'min-width': `${size.width}mm`
  });
  Element.setCSS(project.svg, { left: 0, top: 0, position: 'relative' });
  Element.setCSS(project.svg, { left: 0, top: 0, position: 'relative' });
  //console.log('loadDocument:', project.svg);*/
  return project;
};

const chooseDocument = async (e, proj, i) => {
  let r;
  try {
    const { type } = e;
    const box = Element.findAll('.file')[i];
    //console.log('chooseDocument:', { e, proj, i, box });
    if(!proj.loaded) {
      let data = await loadDocument(proj, box);
      proj.loaded = true;

      open(false);

      //console.log('loaded:', proj);
    }
    r = proj.loaded;
  } catch(err) {
    console.log('err:', err.message);
    console.log('stack:', [...err.stack].join("\n"));
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

let projects = trkl([]);
let socket = trkl();

const BindGlobal = Util.once(arg => trkl.bind(window, arg));

const AppMain = (window.onload = async () => {
  Object.assign(window, { Element, devtools, dom });

  // window.focusSearch = trkl();
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
    showSearch
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

  // prettier-ignore
  Object.assign(window, { BBox, chooseDocument, classNames, ColorMap, components, CSS, deep, EagleDocument, EagleElement, EagleNode, ImmutablePath, ImmutableXPath, EagleReference, eventIterator, h, HSLA, html, isLine, isPoint, isRect, isSize, iterator, Line, loadDocument, LoadFile, Matrix, MatrixTransformation, ModifyColors, Point, PointList, React, Rect,  Rotation, Scaling, Size, SVG, Transformation, TransformationList, Translation, tXml, Util, MouseEvents, ElementToXML, LoadFile, ModifyColors, MakeFitAction, CreateWebSocket, AppMain, Canvas });

  const inspectSym = Symbol.for('nodejs.util.inspect.custom');

  const testComponent = props =>
    html`
      <div>This is a test</div>
    `;

  let c = testComponent({});
  window.testComponent = c;
  //console.log('testComponent', ReactComponent.toObject(c));

  /*console.realLog = console.log;
  //console.log = function(...args) {
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
    //console.log(`Got ${files.length} files`);
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

  const searchFilter = trkl(store.get('filter') || '.*');

  //console.log('searchFilter is ', searchFilter());

  searchFilter.subscribe(value => {
    store.set('filter', value);
  });

  const changeInput = e => {
    const { target } = e;
    //console.log('changeInput:', target.value);

    searchFilter(target.value);
  };

  React.render(
    [
      Panel('buttons', [
        h(Button, {
          caption: '📂',
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
        })
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
        <${FileList} files=${projects} onActive=${open} onChange=${chooseDocument} filter=${searchFilter} showSearch=${showSearch} changeInput=${changeInput} focusSearch=${focusSearch} currentInput=${currentSearch} />
      `
    ],
    Element.find('#preact')
  );

  let move;
  container = Element.find('#main');

  TouchListener(
    event => {
      //if(event.index > 0 && event.buttons > 0) console.log('touch', event, container);
      if(!move) {
        let box = Element.find('#main').firstElementChild;

        move = Element.moveRelative(box);
      } else if(move && event.buttons == 0) {
        move = null;
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
  /* document.addEventListener('keydown', event => {
    const { ctrlKey, shiftKey, altKey, metaKey } = event;

    if(true || ctrlKey || shiftKey || altKey || metaKey) {
      const { key, code, keyCode } = event;
      const { target, currentTarget } = event;
      //console.log('keydown: ', (window.keyEvent = event));
    }
  });*/

  window.addEventListener('wheel', event => {
    //console.log('event:', event);
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
    const { clientX, clientY, target, currentTarget, buttons, altKey, ctrlKey, shiftKey } = event;
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
