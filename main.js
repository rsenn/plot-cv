// prettier-ignore-start
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom from './lib/dom.js';
import { ReactComponent } from './lib/dom/preactComponent.js';
import { iterator, eventIterator } from './lib/dom/iterator.js';
import geom from './lib/geom.js';
import { BBox } from './lib/geom/bbox.js';
import { ScrollDisabler } from './lib/scrollHandler.js';
import { TouchListener } from './lib/touchHandler.js';
import { parseSchematic } from './lib/eagle/parser.js';
import { EagleDocument } from './lib/eagle/document.js';
import { EagleNode } from './lib/eagle/node.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { EagleElement } from './lib/eagle/element.js';
import { EaglePath, EagleReference } from './lib/eagle/locator.js';
import { toXML, EagleInterface } from './lib/eagle/common.js';
import { Renderer } from './lib/eagle/renderer.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './modules/htm/preact/standalone.mjs';
import components, { Chooser, Container, Button, FileList } from './static/components.js';
import { WebSocketClient } from './lib/websocket-client.js';
import { CTORS, ECMAScriptParser, estree, Factory, Lexer, ESNode, Parser, PathReplacer, Printer, Stack, Token } from './lib/ecmascript.js';
const React = { cloneElement, Component, createContext, createRef, Fragment, create: h, html, hydrate, isValidElement, render, toChildArray, useCallback, useContext, useDebugValue, useEffect, useImperativeHandle, useLayoutEffect, useMemo, useReducer, useRef, useState };
const {
  Align,
  Anchor,
  CSS,
  CSSTransformSetters,
  Element,
  ElementPosProps,
  ElementRectProps,
  ElementRectProxy,
  ElementSizeProps,
  ElementTransformation,
  ElementWHProps,
  ElementXYProps,
  HSLA,
  isElement,
  isHSLA,
  isLine,
  isMatrix,
  isNumber,
  isPoint,
  isRect,
  isRGBA,
  isSize,
  Line,
  Matrix,
  Node,
  Point,
  PointList,
  Polyline,
  Rect,
  RGBA,
  Select,
  Size,
  SVG,
  Timer,
  Transition,
  TransitionList,
  TRBL,
  Tree
} = { ...dom, ...geom };
Object.assign(window, { React, ReactComponent, WebSocketClient, html }, dom, geom, { CTORS, ECMAScriptParser, ESNode, estree, Factory, Lexer, Parser, PathReplacer, Printer, Stack, Token }, { Chooser });

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
  return toXML(x);
};

const LoadFile = async filename => {
  let xml = await fetch(`/static/${filename}`).then(async res => {
    return await (await res).text();
  });
  console.log('xml: ', xml.substring(0, 100));
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
    console.log('ModifyColors', e);
    if(!window.c) window.c = SVG.allColors('svg');
    c.dump();
    fn(c);
  }
};

const Panel = (name, children) => html`<${Container} className="${name}">${children}</${Container}>`;

const loadDocument = async (proj, parentElem) => {
  console.log(`load project #${proj.i}:`, proj);

  proj.doc = await LoadFile(proj.name);
  window.eagle = proj.doc;

  Element.remove('#fence');
  proj.element = Element.create('div', { id: 'fence', style: { position: 'relative', 'data-name': proj.name, border: '1px dotted black' } }, Element.find('#container'));
  //let svg = Element.create('svg', { viewBox:  } , element);



    proj.renderer = new Renderer(proj.doc, ReactComponent.append);

  let svgXml = proj.renderer.render(proj.doc);
  console.log('testRender:', svgXml);

  let component = proj.renderer.render(proj.doc, null);

  window.component = component;
  window.rendered = React.render(component, proj.element);

  console.log('window.rendered', window.rendered);

  proj.svg = Element.find('svg', proj.element);
  proj.grid = Element.find('g.grid', proj.element);
  proj.bbox = SVG.bbox(proj.grid);
  let { name, data, element, doc, svg, bbox } = proj;

  let bounds = doc.getBounds();
  let rect = bounds.rect;
  let size = new Size(rect);

  currentProj(proj);

  size.mul(doc.type == 'brd' ? 2 : 1.5);
  //    let aspectRatios = [Element.rect(element).aspect(), ];

  let svgrect = SVG.bbox(proj.svg);
  proj.aspectRatio = svgrect.aspect();

  Element.attr(element, {
    'data-filename': proj.name,
    'data-aspect': proj.aspectRatio,
    'data-width': size.width + 'mm',
    'data-height': size.height + 'mm'
  });

  proj.svg.setAttribute('data-aspect', proj.aspectRatio);

  //  console.log("setRect", { svg, grid, bbox, bounds, rect });

  let css = size.toCSS({ width: 'mm', height: 'mm' });

  //console.log("size.toCSS", css);
  //  Element.move(svg, Point(0,0), 'relative');
  //
  //console.log("setRect", svg);
  //console.log("setRect", css);
  //

  Object.assign(proj.svg.style, {
    'min-width': `${size.width}mm`
    // "min-height": `${size.height}mm`
  });
  Element.setCSS(proj.svg, { left: 0, top: 0, position: 'relative' });

  let w, h;

  Element.setCSS(proj.svg, { left: 0, top: 0, position: 'relative' });

  console.log('loadDocument:', proj.svg);

  return proj;
};

const chooseDocument = async (e, proj, i) => {
  const { type } = e;
  const box = Element.findAll('.file')[i];
  console.log('chooseDocument:', { e, proj, i, box });
  if(!proj.loaded) {
    let data = await loadDocument(proj, box);
    proj.loaded = true;

    open(false);

    console.log('loaded:', proj);
  }
  return proj.loaded;
};

const AppStart = (window.onload = async () => {
  Util(globalThis);

  Object.assign(window, {
    BBox,
    chooseDocument,
    classNames,
    ColorMap,
    components,
    CSS,
    deep,
    EagleDocument,
    EagleElement,
    EagleInterface,
    EagleNode,
    EaglePath,
    EagleReference,
    eventIterator,
    h,
    HSLA,
    html,
    isLine,
    isPoint,
    isRect,
    isSize,
    iterator,
    Line,
    loadDocument,
    LoadFile,
    Matrix,
    MatrixTransformation,
    ModifyColors,
    Point,
    PointList,
    React,
    Rect,
    RGBA,
    Rotation,
    Scaling,
    Size,
    SVG,
    toXML,
    Transformation,
    TransformationList,
    Translation,
    tXml,
    Util
  });
  Object.assign(window, { Element, devtools, dom });

  let projects = trkl([]);
  let socket = trkl();

  trkl.bind(window, { projects, socket });

  ListProjects('/files.html').then(response => {
    //console.log("files", response);
    let data = JSON.parse(response);
    let { files } = data;
    console.log(`Got ${files.length} files`);
    projectFiles = window.files = files;
    //  CreateSelect(files);
    projects(
      projectFiles.map(({ name }, i) => {
        let data = trkl({ percent: NaN });
        let proj = { name, i };
        trkl.bind(proj, { data });
        //console.log("project:", proj);
        return proj;
      })
    );
  });

  let socketURL = Util.makeURL({ location: '/ws', protocol: 'ws' });

  (async () => {
    let ws = (window.socket = new WebSocketClient());
    console.log('New WebSocket:', ws);
    await ws.connect(socketURL);
    console.log('Connected:', ws.connected);
    ws.send('hello!');
    let data;
    for await (data of ws) {
      console.log('WebSocket data:', data);
      // See if there are any more messages received.
      ws.dataAvailable !== 0;
    }
    // Close the connection.
    await ws.disconnect();
    console.log('WebSocket connected: ', ws.connected);
  })();

  const MakeFitAction = index => () => {
    let container = Element.find('#container');
    let parent = Element.find('#main');
    let prect = Element.rect(parent);
    let rect = Element.rect(container);

    let svg = Element.find('svg', container);
    let brect = Element.rect('.buttons');
    let srect = Element.rect(svg);

    prect.y += brect.height;
    prect.height -= brect.height;

    let rects = [prect, rect, srect];
    console.log('resize rects', rects);

    let f = srect.fit(prect);
    console.log('fitted:', f);

    Element.setRect(container, f[index], 'absolute');
    Element.setCSS(container, { transform: '', position: 'absolute' });
  };

  React.render(
    [
      Panel('buttons', [
        h(Button, {
          caption: '📂', // 📁
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
      html`
        <${FileList} files=${projects} onActive=${open} onChange=${chooseDocument} />
      `
    ],
    Element.find('#preact')
  );
  let move;

  TouchListener(
    event => {
      let container = Element.find('#container');
      if(!move) {
        move = Element.moveRelative(container);
      } else if(event.index > 0) {
        let rel = new Point(event);
        //console.log("touch x/y:", event);

        if(move) {
          //console.log("move pos:", move.pos);
          if(event.buttons > 0) move(rel.x, rel.y);
          else move = move.jump();
        }
      }
    },
    { element: window }
  );

  container = Element.find('#container');
  window.styles = CSS.create('head');

  // new ScrollDisabler(() => true, window);

  window.addEventListener('wheel', event => {
    const clientArea = Element.rect('body > div');
    clientArea.x += container.parentElement.scrollLeft;
    const clientCenter = clientArea.center;

    const { clientX, clientY, target, currentTarget } = event;
    const pos = new Point(clientX, clientY);
    //console.log("zoom:", { target, currentTarget });
    const wheelPos = -event.deltaY.toFixed(2);
    zoomVal = Util.clamp(-100, 100, zoomVal + wheelPos * 0.1);
    const zoom = Math.pow(10, zoomVal / 100).toFixed(5);
    const transform = ` scale(${zoom},${zoom}) `;
    const origin = `${pos.toString(1, 'px', ' ')}`;

    let list = [...Element.skip(target)].find(p => p.classList.contains('list'));

    if(list) {
      let parent = list.parentElement;
      let rects = [Element.rect(parent), Element.rect(list)];
      let screen = new Rect(0, 0, window.innerWidth, window.innerHeight);

      rects[0].y2 = screen.y2;
      rects[1].y2 = screen.y2;

      Element.setRect(list, rects[1]);

      /*   console.log("rects:", [...rects, screen]);
            console.log("list:", { pos, wheelPos, zoomVal, zoom });*/
    } else {
      Element.setCSS(container, { transform });

      Element.setCSS(container, { 'transform-origin': origin });
    }
  });
  console.log(Util.getGlobalObject());

  //   LoadProject(projectName);

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
