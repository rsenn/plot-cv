// prettier-ignore-start
import { isPoint, Point } from './lib/geom/point.js';
import { PointList } from './lib/geom/pointList.js';
import { isRect, Rect } from './lib/geom/rect.js';
import { isSize, Size } from './lib/geom/size.js';
import { isLine, Line } from './lib/geom/line.js';
import { Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList } from './lib/geom/transformation.js';
import dom, { Element } from './lib/dom.js';
import { Matrix } from './lib/geom/matrix.js';
import { SVG } from './lib/dom/svg.js';
import { BBox } from './lib/geom/bbox.js';
import { ScrollDisabler } from './lib/scrollHandler.js';
import { RGBA, HSLA, CSS } from './lib/dom.js';
import { TouchListener } from './lib/touchHandler.js';
import { parseSchematic } from './lib/eagle/parser.js';
import { EagleDocument } from './lib/eagle/document.js';
import { EagleNode } from './lib/eagle/node.js';
import { trkl } from './lib/trkl.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { EagleElement } from './lib/eagle/element.js';
import { EaglePath, EagleReference } from './lib/eagle/locator.js';
import { toXML, EagleInterface } from './lib/eagle/common.js';
import { renderDocument, EagleSVGRenderer } from './lib/eagle/renderer.js';
import { devtools } from './lib/devtools.js';
import Util from './lib/util.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import { hydrate, Fragment, createRef, isValidElement, cloneElement, toChildArray } from './modules/preact/dist/preact.mjs';
import { h, html, render, Component, createContext, useState, useReducer, useEffect, useLayoutEffect, useRef, useImperativeHandle, useMemo, useCallback, useContext, useDebugValue } from './modules/htm/preact/standalone.mjs';
import components, { Container, Button, FileList } from './static/components.js';

const React = {
  cloneElement,
  Component,
  createContext,
  createRef,
  Fragment,
  h,
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

var projectName = 'Headphone-Amplifier-ClassAB-alt3';
var palette = null;
var svgElement;
var brdXml, schXml, brdDom, schDom;
var board, schematic;
var loadedProjects = [];
let projects, projectFiles;
let activeFile;

const MouseHandler = callback => e => {
  if(e.type) {
    const pressed = e.type.endsWith('down');

    callback(e, pressed);
  }
};

const classNames = (...args) => args.filter(arg => typeof arg == 'string' && arg.length > 0).join(' ');

const MouseEvents = h => ({
  onMouseDown: h,
  /*  onBlur: h,*/
  onMouseOut: h,
  onMouseUp: h
});
console.log('running');
console.log('dom', { Rect, Element, parseSchematic });

window.dom = { Element, SVG };
/* prettier-ignore */ Object.assign(window, {EagleDocument, EagleElement, EagleNode, EaglePath, EagleReference, EagleSVGRenderer, EagleInterface, Element, SVG, CSS, RGBA, HSLA, toXML, isPoint, Point, PointList, isLine, Line, isRect, Rect, isSize, Size, Matrix, Transformation, Rotation, Translation, Scaling, MatrixTransformation, TransformationList, tXml, deep, BBox, React, h, html,
        ColorMap,
        devtools,
        components, dom  });
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
  console.log('x:', x);
  return toXML(x);
};

const LoadFile = filename =>
  new Promise(async (resolve, reject) => {
    let xml = await fetch(`/static/${filename}`).then(async res => {
      return await (await res).text();
    });
    console.log('xml: ', xml.substring(0, 100));
    //let dom = new DOMParser().parseFromString(xml, 'application/xml');

    let doc = new EagleDocument(xml);

    if(/\.brd$/.test(filename)) window.board = doc;
    if(/\.sch$/.test(filename)) window.schematic = doc;

    resolve(doc);
  });

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

const loadDocument = (proj, parentElem) =>
  new Promise(async (resolve, reject) => {
    console.log(`load project:`, proj);
    let { name, signal, svg, element } = proj;
    signal.subscribe(project => resolve(project));
    let doc = await LoadFile(name);
    window.eagle = doc;
    //console.log('document:', doc);
    /*if(!element)*/

    Element.remove('#fence');

    element = Element.create(
      'div',
      {
        id: 'fence',
        style: { position: 'relative', 'data-name': name, border: '1px dotted black' }
      },
      Element.find('#container')
    );
    //let svg = Element.create('svg', { viewBox:  } , element);
    window.renderer = renderDocument(doc, element);
    if(!svg) svg = Element.find('svg', element);
    let grid = Element.find('g.grid', element);
    let bbox = SVG.bbox(grid);
    let bounds = doc.getBounds();
    let rect = bounds.rect;
    let size = new Size(rect);

    size.mul(doc.type == 'brd' ? 2 : 1.5);
    //    let aspectRatios = [Element.rect(element).aspect(), ];

    let svgrect = SVG.bbox(svg);
    let aspectRatio = svgrect.aspect();

    Element.attr(element, {
      'data-filename': name,
      'data-aspect': aspectRatio,
      'data-width': size.width + 'mm',
      'data-height': size.height + 'mm'
    });

    svg.setAttribute('data-aspect', aspectRatio);

    //  console.log("setRect", { svg, grid, bbox, bounds, rect });

    let css = size.toCSS({ width: 'mm', height: 'mm' });

    //console.log("size.toCSS", css);
    //  Element.move(svg, Point(0,0), 'relative');
    //
    //console.log("setRect", svg);
    //console.log("setRect", css);
    //

    Object.assign(svg.style, {
      'min-width': `${size.width}mm`,
      'min-height': `${size.height}mm`
    });
    Element.setCSS(svg, { left: 0, top: 0, position: 'relative' });

    let w, h;

    //h = (100 / aspectRatio).toFixed(4)+'%';

    //   let rect = Element.rect(element);

    //let padding = (100 / aspect).toFixed(4)+'%';
    //   Element.setCSS(element, { position: 'relative', width: '100%', height, padding: `0 0 0 0` });

    Element.setCSS(svg, { left: 0, top: 0, position: 'relative' });

    signal({ ...proj, loaded: true, bbox, element, svg });
    return signal();
  });

const chooseDocument = async (...args) => {
  const [e, proj, i] = args;
  const { type } = e;
  const box = Element.findAll('.file')[i];
  console.log('chooseDocument:', { e, proj, box, i });
  if(!proj.loaded) {
    let data = await loadDocument(proj, box);
    proj.loaded = true;
    console.log('loaded:', proj);
  }
  return proj.loaded;
};

const AppStart = (window.onload = async () => {
  Object.assign(window, {
    AppStart,

    chooseDocument,
    classNames,
    loadDocument,
    LoadFile,
    ModifyColors,
    MouseEvents,
    MouseHandler,
    devtools
  });

  window.Util = Util;
  Util(globalThis);

  let projects = trkl([]);

  ListProjects('/files.html').then(response => {
    console.log('files', response);
    let data = JSON.parse(response);
    let { files } = data;
    console.log(`Got ${files.length} files`);
    projectFiles = window.files = files;
    //  CreateSelect(files);
    projects(
      projectFiles.map(({ name }, i) => {
        let signal = trkl({ percent: NaN });
        let proj = { name, i, signal };
        trkl.bind(proj, 'data', signal);
        //console.log("project:", proj);
        return proj;
      })
    );
  });

  let open = trkl();

  const MakeFitAction = index => () => {
    let container = Element.find('#container');
    let parent = container.parentElement;
    let prect = Element.rect(parent);
    let rect = Element.rect(container);

    let svg = Element.find('svg', container);
    let srect = Element.rect(svg);

    let rects = [prect, rect, srect];
    console.log('resize rects', rects);

    let f = srect.fit(prect);
    console.log('fitted:', f);

    Element.setRect(container, f[index], 'absolute');
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
  var move;

  TouchListener(
    event => {
      let container = Element.wrap('#container');
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

  window.container = Element.find('#container');
  window.styles = CSS.create('head');

  // new ScrollDisabler(() => true, window);

  var zoomVal = 0;
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

      console.log('rects:', [...rects, screen]);
      console.log('list:', { pos, wheelPos, zoomVal, zoom });
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
