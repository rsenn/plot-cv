import { crosskit, CANVAS } from './lib/crosskit.js';
import { RGBA, HSLA } from './lib/color.js';
import { timer } from './lib/async/helpers.js';
import { WebSocketIterator, WebSocketURL, CreateWebSocket, ReconnectingWebSocket, StreamReadIterator } from './lib/async/websocket.js';
import { once, streamify, throttle, distinct, subscribe } from './lib/async/events.js';
import { propertyLookup, tryCatch, tryFunction, chain, chainRight, getset, gettersetter, lazyProperties, memoize, define, isUndefined, properties, keys, unique, randStr, randInt, waitFor, isFunction, getPrototypeChain, getConstructorChain, isCFunction, isJSFunction, isObject } from './lib/misc.js';
import { isStream, AcquireReader, AcquireWriter, ArrayWriter, readStream, PipeTo, WritableRepeater, WriteIterator, AsyncWrite, AsyncRead, ReadFromIterator, WriteToRepeater, LogSink, StringReader, LineReader, DebugTransformStream, CreateWritableStream, CreateTransformStream, RepeaterSource, RepeaterSink, LineBufferStream, TextTransformStream, ChunkReader, ByteReader, PipeToRepeater, Reader, ReadAll } from './lib/stream/utils.js';
import { Intersection, Matrix, isRect, Rect, Size, Point, Line, TransformationList, Vector } from './lib/geom.js';
import { Element, isElement, SVG } from './lib/dom.js';
import React, { h, html, render, Fragment, Component, createRef, useState, useLayoutEffect, useRef, toChildArray } from './lib/dom/preactComponent.js';
import miscfixed6x13 from './static/json/miscfixed6x13.js';
import { List } from './lib/list.js';
//import { fire } from './fire/build/fire-debug.js';
import { LinkedList } from './lib/container/linkedList.js';
import { wasmBrowserInstantiate } from './wasm-helpers.js';
//import lscache from './lib/lscache.js';

let lsgs = (globalThis.lsgs = getset([key => localStorage.getItem(key), (key, value) => localStorage.setItem(key, value)]));

const lstore = (globalThis.lstore = propertyLookup(
  ...(globalThis.lsgs2 = lsgs.transform(
    tryFunction(
      v => JSON.parse(v ?? ''),
      v => v,
      () => null
    ),
    o => JSON.stringify(o || null)
  ))
));

// For random numbers, use "x = 181 * x + 359" from
// Tom Dickens "Random Number Generator for Microcontrollers"
// https://web.archive.org/web/20170323204917/http://home.earthlink.net/~tdickens/68hc11/random/68hc11random.html
let scratch = 0;

function RandomByte() {
  const value = 181 * scratch + 359;
  scratch = value >>> 0;
  return (value >>> 8) & 0xff;
}

function Modulo(n, m) {
  return ((n % m) + m) % m;
}

function isNativeObject(obj) {
  return isCFunction(obj.constructor);
}

function Object2JSON(obj) {
  if(typeof obj.tagName == 'string') return ElementName(obj);
  if(obj instanceof EventTarget) return '#' + TargetName(obj).toLowerCase();
}

function CopyObject(obj) {
  let ret = {};
  for(let proto of [obj, ...getPrototypeChain(obj)].reverse()) {
    for(let prop of Object.getOwnPropertyNames(proto)) {
      if(prop == '__proto__') continue;
      if(typeof prop == 'symbol') continue;
      if(isFunction(obj[prop])) continue;
      let v = obj[prop];
      if(isObject(v)) {
        v = Object2JSON(v);

        v ??= isNativeObject(v) ? {} : CopyObject(v);
      }
      ret[prop] = v;
    }
  }
  return ret;
}

function CreatePalette() {
  const colors = new Array(256);

  for(let i = 0; i < 64; i++) {
    const value = i * 4;

    colors[i] = new RGBA(value, 0, 0);
    colors[i + 64] = new RGBA(255, value, 0);
    colors[i + 128] = new RGBA(255, 255, value);
    colors[i + 192] = new RGBA(255, 255, 255);
  }
  return colors;
}

function CreatePaletteHSL() {
  const colors = new Array(256);

  const hues = [new HSLA(0, 100, 0), new HSLA(0, 100, 50), new HSLA(30, 100, 50), new HSLA(60, 100, 50), new HSLA(60, 100, 100), new HSLA(60, 100, 100)];

  const breakpoints = [0, 51, 80, 154, 205, 256];
  console.log('breakpoints:', breakpoints);

  for(let i = 0; i < 256; i++) {
    const hue = (v => (v == -1 ? () => hues.length - 2 : v => v))(breakpoints.findIndex(b => i < b));
    const range = breakpoints[hue] - 1 - breakpoints[hue - 1];

    colors[i] = HSLA.blend(hues[hue - 1], hues[hue], (i - breakpoints[hue - 1]) / range).toRGBA();
  }
  return colors;
}

class DrawList extends LinkedList {
  constructor() {
    super();
  }

  insert(item) {
    return this.insertBefore(item, it => it.time > item.time);
  }

  prev(upto = performance.now()) {
    let prev;
    for(let item of this) {
      if(item.time >= upto) return prev;
      prev = item;
    }
  }

  next(upto = performance.now()) {
    for(let item of this) if(item.time >= upto) return item;
  }

  diff(upto = performance.now()) {
    let p = this.prev(upto),
      n = this.next(upto);

    return {
      x: n.x - p.x,
      y: n.y - p.y,
      time: n.time - p.time
    };
  }

  *dequeue(upto = performance.now()) {
    let head,
      predicate = typeof upto == 'number' ? it => it.time < upto : upto;
    while((head = this.head)) {
      if(!predicate(head)) break;
      yield this.removeHead();
    }
  }

  queue(trail, t = performance.now()) {
    trail[0].time = 0;
    for(let item of trail) {
      t += item.time;
      this.insert({ ...item, time: t });
    }
  }
}

let drawList = (globalThis.drawList = new DrawList());

function* AllParents(elem) {
  let obj = elem.ownerDocument;

  while(elem) {
    yield elem;
    if(elem.isSameNode(obj)) break;
    elem = elem.parentElement;
  }
}

function getTransformationList(e) {
  let css = Element.getCSS(e);
  if(css.transform) return new TransformationList(css.transform);
}
function DecomposeTransformList(elem) {
  let list = getTransformationList(elem);

  if(list && list.length == 1 && list[0].type == 'matrix') {
    let tl = list.decompose();
    console.log(`Setting '${list}' to ${tl}`);
    elem.style.transform = tl;
  }
}

function drawRect(rect, stroke = '#0f0') {
  let svg = svgLayer;
  const { x, y, width, height } = new Rect(rect);
  return SVG.create('rect', { x, y, width, height, stroke, 'stroke-width': '1', fill: 'none' }, svg);
}

function GetElementMatrix(element) {
  let { transform } = Element.getCSS(element);

  return Matrix.fromCSS(transform);
}

function SetCrosshair(pos) {
  let ch = Element.find('#crosshair');
  let rect = Element.getRect(ch);

  console.log('SetCrosshair', { ch, rect, pos });
  rect.x = pos.x - rect.width / 2;
  rect.y = pos.y - rect.height / 2;

  Element.setRect(ch, rect);
}

function EventPositions(eventOrTouch) {
  let positions = unique(
    keys(eventOrTouch, 2)
      .filter(n => typeof n == 'string' && /[XY]$/.test(n))
      .map(n => n.slice(0, -1))
  );

  return positions.reduce((acc, key) => {
    acc[key] = new Point(eventOrTouch[key + 'X'], eventOrTouch[key + 'Y']);
    return acc;
  }, {});
}

function PositionMatrix(canvas = Element.find('canvas'), rect) {
  rect ??= Element.rect(canvas);
  let vertical = rect.aspect() < 1;
  let topLeft = rect.toPoints()[vertical ? 1 : 0];
  let m = Matrix.identity().scale(canvas.width, canvas.height);

  if(vertical) m = m.rotate(-Math.PI / 2);

  m = m.scale(1 / rect.width, 1 / rect.height);
  m = m.translate(-topLeft.x, -topLeft.y);

  return m;
}

function PositionProcessor(canvas = Element.find('canvas'), rect) {
  rect ??= Element.rect(canvas);
  let m = PositionMatrix(canvas, rect);
  return pos => new Point(...m.transformPoint(new Point(pos))).round(1);
}

function ProcessPosition(pos) {
  return PositionProcessor()(pos);
}

function TouchTransformer(tfn = (x, y) => [x, y]) {
  return touch => {
    let [x, y] = tfn(touch.clientX, touch.clientY);

    touch.x = x;
    touch.y = y;
    return touch;
  };
}

async function* TouchPrinter(iter) {
  for await(let event of iter) {
    let n = event.touches?.length;
    console.log('TouchPrinter', event);
    for(let i = 0; i < n; i++) {
      const { x, y } = event.touches[i];
      console.log('touch #' + i, { x, y });
    }
    yield event;
  }
}

function MouseToTouch(event) {
  switch (event.type) {
    case 'mouseup':
      delete event.type;
      Object.defineProperty(event, 'type', { value: 'touchend', configurable: true, enumerable: true });
      break;
    case 'mousedown':
      delete event.type;
      Object.defineProperty(event, 'type', { value: 'touchstart', configurable: true, enumerable: true });
      break;
    case 'mousemove':
      delete event.type;
      Object.defineProperty(event, 'type', { value: 'touchmove', configurable: true, enumerable: true });
      break;
  }
  if(!('touches' in event)) {
    event.touches = [{ clientX: event.clientX, clientY: event.clientY, x: event.x, y: event.y }];
  }

  return event;
}
async function* CatchIterator(it) {
  try {
    for await(let item of it) yield item;
  } catch(error) {
    console.log('CatchIterator ERROR: ' + error.message + '\n' + error.stack);
  }
}

async function* TouchIterator(element, t) {
  let rect = Element.rect(element);

  if(!t) {
    let matrix = PositionMatrix();
    //GetElementMatrix(canvasElement.parentElement).invert();
    // console.log('TouchIterator', { matrix }, matrix.decompose());

    t = TouchTransformer((x, y) => matrix.transformXY(x, y).map(Math.floor));
  }
  //console.log('TouchIterator', { rect, element });

  let ev = MouseToTouch(await once(element, ['mousedown', 'touchstart']));
  let type = ev.type.slice(0, 5);
  if(ev.touches) [...ev.touches].forEach(t);
  yield ev;

  for await(let event of streamify(['touchend', 'touchmove', 'mouseup', 'mousemove'], element)) {
    event.preventDefault();

    MouseToTouch(event);

    if(event.touches) [...event.touches].forEach(t);

    yield event;

    if(event.type.endsWith('end')) break;
  }
}

async function* MovementIterator(element) {
  let ev = await once(element, 'mousedown', 'touchstart');
  let type = ev.type.slice(0, 5);
  yield ev;
  console.log(type + ' start');
  for await(let event of streamify(['mouseup', 'mousemove', 'touchend', 'touchmove'], element)) {
    //event.preventDefault();
    if('touches' in event) {
      if(event.touches.length) {
        globalThis.mouseEvent = event;

        for(let touch of event.touches) {
          globalThis.touch = touch;
          const { force, radiusX, radiusY, clientX: x, clientY: y, ...obj } = touch;
          yield { ...obj, type: 'touch', force, radiusX, radiusY, x, y };
        }

        // yield* [...event.touches].map(EventPositions);
      }
    } else {
      const { type, clientX: x, clientY: y } = event;
      let obj = { type, x, y };
      yield obj;
    }
    if(/(up|end)$/.test(event.type)) {
      console.log(type + ' end');
      break;
    }
  }
}

async function* MoveIterator(eventIterator) {
  for await(let event of eventIterator) {
    if('touches' in event) {
      if(event.touches.length) {
        globalThis.touchEvent = event;

        for(let touch of event.touches) {
          globalThis.touch = touch;
          const { force, radiusX, radiusY, clientX: x, clientY: y, ...obj } = touch;
          yield { ...obj, type: 'touch', force, radiusX, radiusY, x, y };
        }
      }
    } else throw new Error(`No such property: touches`);
  }
}

function main() {
  lazyProperties(globalThis, {
    svgLayer: () => {
      let e = SVG.create('svg', { xmlns: 'http://www.w3.org/2000/svg', viewBox: new Rect(0, 0, window.innerWidth, window.innerHeight) }, document.body);
      Element.move(e, new Point(0, 0), 'absolute');
      Element.setCSS(e, { 'pointer-events': 'none' });
      return e;
    }
  });

  define(
    globalThis,
    { Bresenham, LinkedList, List, AllParents, TransformationList, getTransformationList, DecomposeTransformList, drawRect, miscfixed6x13 },
    //    {fire},
    properties(
      {
        cid: () => lstore.cid || (lstore.cid = MakeClientID()),
        currentURL: () => new URL(import.meta.url),
        currentFile: () => globalThis.currentURL.pathname.replace(/^\//, '')
      },
      { memoize: true }
    )
  );

  LoadWASM();

  const width = 320;
  const height = 200;
  const parent = document.body;

  crosskit.init({
    renderer: CANVAS,
    parent,
    w: width,
    h: height,
    alpha: false
  });

  function Reparent(canvas = document.getElementsByTagName('canvas')[0]) {
    canvas.parentElement.removeChild(canvas);
    document.getElementById('canvas').appendChild(canvas);
  }

  Reparent();

  crosskit.clear();
  crosskit.rect({
    x: 0,
    y: 0,
    width,
    height,
    fill: 'black',
    stroke: 'black',
    angle: 0
  });

  const buffer = new ArrayBuffer(width * (height + 2));
  const palette = CreatePalette();
  const paletteHSL = CreatePaletteHSL();

  const pixels = Array.from({ length: height + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * width, width));
  const { context } = crosskit;
  const image = context.createImageData(width, height);

  const { now, waitFor, animationFrame } = Util;
  const fps = 50;
  const matrix = new Matrix().translate(160, 100).scale(0.5);

  Object.assign(globalThis, {
    buffer,
    palette,
    paletteHSL,
    pixels,
    context,
    image,
    fps,
    matrix,
    Reparent,
    dom: { Element },
    geom: { Rect },
    MouseToTouch,
    CatchIterator,
    TouchIterator,
    TouchPrinter,
    MovementIterator,
    GetElementMatrix,
    SetCrosshair,
    EventPositions,
    PositionProcessor,
    PositionMatrix,
    ProcessPosition,
    PutArray,
    waitFor,
    ReplayTrail,
    Blaze
  });

  async function Loop() {
    const delay = 1000 / fps;
    const log = (t, name) => globalThis.doLog && console.log(`${name} timing: ${t.toFixed(3)}ms`);
    const fire = (...args) => Fire(...args);
    const redraw = (...args) => Redraw(...args);

    await once(window, 'load');

    Init();

    for(;;) {
      fire();
      redraw();
      await animationFrame(delay);
    }
  }

  function Fire() {
    for(let x = 0; x < width; x++) {
      pixels[height][x] = 255 - (RandomByte() % 128);
      pixels[height + 1][x] = 255 - (RandomByte() % 128);
    }

    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        const sum = [pixels[y + 1][Modulo(x - 1, width)], pixels[y + 1][x], pixels[y + 1][Modulo(x + 1, width)], pixels[y + 2][x]].reduce((a, p) => a + (p | 0), 0);

        pixels[y][x] = (sum * 15) >>> 6;
      }
    }

    for(let draw of drawList.dequeue()) {
      draw.value = 255 - 0x10; //(RandomByte() % 128);
      if(draw.size > 1) {
        if(draw.x > 0) pixels[draw.y][draw.x - 1] = draw.value;
        if(draw.x < 319) pixels[draw.y][draw.x + 1] = draw.value;
        if(draw.y > 0) pixels[draw.y - 1][draw.x] = draw.value;

        if(draw.y < 199) pixels[draw.y + 1][draw.x] = draw.value;
      }
      pixels[draw.y][draw.x] = draw.value;

      //Blaze(draw.x, draw.y, 255 - (RandomByte() % 128));
    }
  }

  async function Redraw() {
    const { data } = image;

    let i = 0;
    let t = [...matrix];

    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        const c = palette[pixels[y][x]];
        data[i++] = c.r;
        data[i++] = c.g;
        data[i++] = c.b;
        data[i++] = c.a;
      }
    }

    context.putImageData(image, 0, 0);
  }

  let element, rect, rc, mouseTransform;

  function Draw(x, y, time = performance.now()) {
    return drawList.insert({ x, y, time });
  }

  function* Bresenham(x0, y0, x1, y1) {
    var dx = Math.abs(x1 - x0),
      dy = Math.abs(y1 - y0),
      sx = x0 < x1 ? 1 : -1,
      sy = y0 < y1 ? 1 : -1,
      err = dx - dy;

    while(x0 != x1 || y0 != y1) {
      var e2 = 2 * err;
      if(e2 > dy * -1) {
        err -= dy;
        x0 += sx;
      }
      if(e2 < dx) {
        err += dx;
        y0 += sy;
      }
      yield [x0, y0];
    }
  }

  function Blaze(x, y, r) {
    let v = pixels[y][x];
    r ??= 255 - v;
    PutArray(x - 1, y - 1, [
      [0, r, 0],
      [r, r, r],
      [0, r, 0]
    ]);

    /*    for(let ty = y - 1; ty < y + 1; ty++) 
      for(let tx = x - 1; tx < x + 1; tx++) 
        pixels[ty][tx] = r;*/

    // pixels[y + 1][x] = r;
  }

  function ReplayTrail(trail, time = performance.now() + 20) {
    let prev,
      ret = [];

    for(let pt of trail) {
      // console.log('ReplayTrail', {pt});

      if(prev) {
        let index = ret.length;
        let i = 0;
        for(let [x, y] of Bresenham(prev.x, prev.y, pt.x, pt.y)) {
          /*      if(i++ > 0) */ ret.push({ x, y, time: 0, size: 2 });
        }
        if(ret[index] !== undefined) {
          ret[index].time = pt.time;
        }
        if(ret[ret.length - 1]) ret[ret.length - 1].size = 2;
      } else {
        pt.size = 2;
        ret.push(pt);
      }
      prev = pt;
    }

    if(ret[0] !== undefined) {
      ret[0].time = 0;
      drawList.queue(ret, time);
    }
  }

  function PutArray(x, y, a) {
    let rows = a.length;
    let cols = a[0].length;
    let w = pixels[0].length;
    let h = pixels.length;

    for(let ty = 0; ty < rows; ++ty) {
      for(let tx = 0; tx < cols; ++tx) {
        pixels[(y + ty) % h][(x + tx) % w] += a[ty][tx] / 2;
      }
    }
  }

  function PutArray2(x, y, a) {
    let rows = a.length * 2;
    let cols = a[0].length * 2;
    let w = pixels[0].length;
    let h = pixels.length;

    for(let ty = 0; ty < rows; ++ty) {
      for(let tx = 0; tx < cols; ++tx) {
        pixels[(y + ty) % h][(x + tx) % w] = a[ty >> 1][tx >> 1];
      }
    }
  }

  function ResizeHandler(event) {
    let { body } = document;
    let rect = Element.rect('canvas');

    Element.setCSS(body, { width: rect.x2 + 'px', height: rect.y2 + 'px' });
    let { width, height } = Element.getCSS(body);
    console.log('ResizeHandler', { event, rect, width, height });

    SendWS({ type: 'event', event });

    mouseTransform = PositionProcessor();
  }

  function OrientationChange(e) {
    rect = canvasRect;
    mouseTransform = PositionProcessor();
    console.log('OrientationChange', { e, rect });
  }

  Object.assign(globalThis, { RandomByte });

  function Init() {
    window.canvas = element = document.querySelector('canvas');

    define(
      globalThis,
      properties({
        canvasElement: () => Element.find('canvas'),
        divElement: () => Element.find('body > div:first-child')
      }),
      properties({
        windowRect: () => new Rect(window.innerWidth, window.innerHeight),
        bodyRect: () => Element.rect('body').round(0),
        canvasRect: () => Element.rect('canvas').round(0),
        divRect: () => Element.rect('body > div:first-child').round(0)
      }),
      {
        getRect
      },
      properties({
        transform: [() => new TransformationList(Element.getCSS('body > div:first-child').transform), value => Element.setCSS('body > div:first-child', { transform: value + '' })]
      })
    );

    const SVGComponent = props => {
      const rect = Element.rect(document.body).round(1);
      const { center, width, height } = rect;

      return h('svg', { version: '1.1', xmlns: 'http://www.w3.org/2000/svg', viewBox: [...rect].join(' '), width, height }, [
        h('circle', {
          cx: center.x,
          cy: center.y,
          r: 250,
          stroke: '#0f0',
          'stroke-width': 1,
          fill: `rgba(80,80,80,0.3)`
        })
      ]);
    };

    let svgContainer = Element.create('div', {}, document.body);
    Element.setCSS(svgContainer, { position: 'absolute', top: 0, left: 0, zIndex: 99999999, pointerEvents: 'none' });

    /*render(h(SVGComponent), svgContainer);*/

    rect = canvasRect;
    mouseTransform = PositionProcessor();

    (async function() {
      for await(let event of streamify(['orientationchange', 'resize'], document)) {
        console.log(event.type, event);
        globalThis[event.type] = event;
      }
    })();

    window.addEventListener('resize', ResizeHandler, true);
    window.addEventListener('orientationchange', OrientationChange, true);

    ResizeHandler();
    /*  
    subscribe(MovementIterator(element), handler);*/
  }

  globalThis.ws = (globalThis.rws ??= NewWS({
    onOpen() {
      console.log('WS connected!');
      /* if(!globalThis.cid) 
        globalThis.cid = lstore.cid || (lstore.cid=MakeClientID());*/
      const { cid } = globalThis;
      SendWS({ type: 'hello', cid });

      SendWS({ type: 'rects', cid, rects: GetRects() });
    }
  })).ws;

  function handleEvents(a, handler) {
    for(let type of a) window.addEventListener(type, handler);
  }

  handleEvents(['reset', 'resize', 'scroll'], e => {
    e.preventDefault();
    globalThis.event = e;
    let event = CopyObject(e);
    console.log('e.target', e.target);
    console.log('event', event);

    if(e.type == 'scroll') {
      const { scrollX, scrollY } = window;
      event.scrollX = scrollX;
      event.scrollY = scrollY;

      window.scrollY = 0;
      window.scrollX = 0;
    }

    SendWS({ type: 'event', event });
    return false;
  });

  let str = '';
  let xpos = 0;
  function KeyHandler(key) {
    //str+=key;
    console.log('KeyHandler', { xpos, key });

    if(key in miscfixed6x13) {
      PutArray2(xpos, 100, miscfixed6x13[key]);
      xpos += 14;
    }
  }

  (async () => {
    for await(let e of streamify(['keydown', 'keyup'], window)) {
      const { type, key, keyCode, repeat, ctrlKey, shiftKey, altKey, metaKey } = (globalThis.keyEvent = e);

      if(!ctrlKey && !altKey && !metaKey) e.preventDefault();

      // console.log('event', { type, key, charCode: key.codePointAt(0), keyCode, repeat, ctrlKey, shiftKey, altKey, metaKey });

      //if(key in miscfixed6x13 || keyCode < 0x20) {
      KeyHandler(key);
    }
  })();

  /* (globalThis.it = CatchIterator(TouchPrinter(TouchIterator(window)))),
    (globalThis.events = []),
    (async () => {
      for await(let v of it) events.push(v);
    })();*/

  //false &&
  //
  InputHandler();

  //
  async function InputHandler() {
    const timegen = (() => {
      let t = Date.now();
      let prev = 0;
      return Object.assign(
        () => {
          let tmp = t;
          t = Date.now() - prev;
          prev += t;
          return t;
        },
        { start: t }
      );
    })();
    const trail = (globalThis.trail = []);
    for(;;) {
      let prev,
        pt,
        start = timegen(),
        last;
      for await(let ev of /*CatchIterator*/ MoveIterator(/*TouchPrinter*/ TouchIterator(window))) {
        globalThis.movement = ev;
        if(rect.inside(ev)) {
          let pt = (globalThis.mousePos = mouseTransform(ev));
          let diff = pt,
            t;
          if(prev) diff = pt.diff(prev);
          let time = timegen();
          let obj = { ...pt, time };
          trail.push(obj);
          if(prev) {
            for(let [x, y] of Bresenham(prev.x, prev.y, pt.x, pt.y)) {
              drawList.insert({ x, y, time });
            }
          } else drawList.insert(obj);
          last = t;
          try {
          } catch(e) {}
          prev = pt;
        }
      }
      SendTrail();

      function SendTrail(start = timegen.start) {
        if(trail.length) {
          trail[0].time = 0;
          const payload = (globalThis.payload = { type: 'blaze', cid: globalThis.cid, start, trail: [...trail] });
          console.log('SendTrail', payload);
          SendWS(payload);
        }
        trail.splice(0, trail.length);
        start = undefined;
        prev = undefined;
      }
    }
  }

  Loop();
}

function getRect(elem) {
  return new Rect((elem ?? divElement).getBoundingClientRect()).round(1);
}

function ParseJSON(s) {
  let r;
  try {
    r = JSON.parse(s);
  } catch(error) {
    console.log('JSON parse error: ' + error.message + '\n' + error.stack);
  }
  return r;
}

function NewWS(handlers) {
  let url = WebSocketURL('/ws', { mirror: currentFile });
  let ws = new ReconnectingWebSocket(url, 'lws-mirror-protocol', handlers ?? {});

  (async function() {
    let chunks = '';
    for await(let chunk of ws) {
      chunks += chunk;

      if(/}\s*$/.test(chunks)) {
        let data = (globalThis.received = ParseJSON(chunks));

        if(data.type != 'event') console.log('WS receive:', data);

        switch (data.type) {
          case 'event':
            break;
          case 'blaze': {
            ReplayTrail(data.trail);
            break;
          }
          case 'eval': {
            if(data.cid && globalThis.cid != data.cid) break;

            let result, exception;

            try {
              result = eval(data.code);
            } catch(error) {
              exception = error;
            }

            SendWS({ type: 'result', ...(exception ? { error: exception.message } : { result }) });
            break;
          }
        }

        chunks = '';
      }
    }
  })();

  return (globalThis.ws = ws);
}

function MakeUUID(rng = Math.random) {
  return [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef'), rng).join('-');
}
function MakeClientID(rng = Math.random) {
  return [4, 4, 4, 4].map(n => randStr(n, ['ABCDEFGHIJKLMNOPQRSTUVWXYZ', 'abcdefghijklmnopqrstuvwxyz', '.-$'][randInt(0, 3)]), rng).join('');
}

async function LoadWASM(file = 'fire/build/fire.wasm') {
  let t = Date.now();
  let {
    module,
    instance: { exports }
  } = await wasmBrowserInstantiate(file);

  console.log(`WASM module loaded. Took ${Date.now() - t}ms`);

  return (globalThis.wasm = { module, exports });
}

function ElementName(e) {
  let s = e.outerHTML.substring(1).substring(0, e.tagName.length);
  if(e.id) s += '#' + e.id;
  return s;
}

function TargetName(e) {
  return getConstructorChain(e)[0].name.replace('HTML', '');
}

function GetRects() {
  let rects = [];
  for(let element of [...AllParents(Element.find('canvas'))]) {
    rects.push([ElementName(element), Element.rect(element)]);
  }

  return rects;
}

function SendWS(msg) {
  if(ws.readyState != ws.OPEN) return;
  if(!('cid' in msg)) msg = { cid: globalThis.cid, ...msg };

  console.log('WS send:', msg);
  if(typeof msg != 'string') msg = JSON.stringify(msg);

  return ws.send(msg);
}
define(globalThis, { crosskit, RGBA, HSLA, Util, Matrix, TransformationList });
define(globalThis, { WebSocketIterator, WebSocketURL, CreateWebSocket, NewWS, ReconnectingWebSocket });
define(globalThis, { define, isUndefined, properties, keys });
define(globalThis, { once, streamify, throttle, distinct, subscribe });
define(globalThis, { Intersection, Matrix, isRect, Rect, Size, Point, Line, TransformationList, Vector });
define(globalThis, {
  LoadWASM,
  timer,
  MakeUUID,
  MakeClientID,
  isStream,
  AcquireReader,
  AcquireWriter,
  ArrayWriter,
  readStream,
  PipeTo,
  WritableRepeater,
  WriteIterator,
  AsyncWrite,
  AsyncRead,
  ReadFromIterator,
  WriteToRepeater,
  LogSink,
  StringReader,
  LineReader,
  DebugTransformStream,
  CreateWritableStream,
  CreateTransformStream,
  RepeaterSource,
  RepeaterSink,
  LineBufferStream,
  TextTransformStream,
  ChunkReader,
  ByteReader,
  PipeToRepeater,
  Reader,
  ReadAll,
  StreamReadIterator
});

define(globalThis, {
  Element,
  isElement,
  SVG,
  React,
  h,
  html,
  render,
  Fragment,
  Component,
  createRef,
  useState,
  useLayoutEffect,
  useRef,
  toChildArray,
  randInt,
  wasmBrowserInstantiate,
  CopyObject,
  getPrototypeChain,
  getConstructorChain,
  isJSFunction,
  isCFunction,
  ElementName,
  TargetName,
  GetRects,
  SendWS,
  getset,
  gettersetter,
  chain,
  chainRight
});

main();
