import { once, streamify } from './lib/async/events.js';
//import { timer } from './lib/async/helpers.js';
import { ReconnectingWebSocket, WebSocketURL } from './lib/async/websocket.js';
import { HSLA, RGBA } from './lib/color.js';
import { LinkedList } from './lib/container/linkedList.js';
import { CANVAS, crosskit } from './lib/crosskit.js';
import { Element, SVG } from './lib/dom-old.js';
import { Fragment, h, default as React } from './lib/dom/preactComponent.js';
import { Matrix, Point, Rect, Size, TransformationList } from './lib/geom.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { List } from './lib/list.js';
import { chainRight, define, getConstructorChain, getPrototypeChain, getset, isCFunction, isFunction, isObject, keys, memoize, properties, propertyLookup, randInt, randStr, tryCatch, tryFunction, unique, } from './lib/misc.js';
//import { AcquireReader, AcquireWriter, ArrayWriter, AsyncRead, AsyncWrite, ByteReader, ChunkReader, CreateTransformStream, CreateWritableStream, DebugTransformStream, isStream, LineBufferStream, LineReader, LogSink, PipeTo, PipeToRepeater, ReadAll, Reader, ReadFromIterator, readStream, RepeaterSink, RepeaterSource, StringReader, TextTransformStream, WritableRepeater, WriteIterator, WriteToRepeater, } from './lib/stream/utils.js';
import trkl from './lib/trkl.js';
import miscfixed6x13 from './static/json/miscfixed6x13.js';
import { wasmBrowserInstantiate } from './wasm-helpers.js';

console.log('fire.js loaded!');

let lsgs = (globalThis.lsgs = getset([key => localStorage.getItem(key), (key, value) => localStorage.setItem(key, value)]));

const lstore = (globalThis.lstore = propertyLookup(
  ...(globalThis.lsgs2 = lsgs.transform(
    tryFunction(
      v => JSON.parse(v ?? ''),
      v => v,
      () => null,
    ),
    o => JSON.stringify(o || null),
  )),
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
      time: n.time - p.time,
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
let fixedList = (globalThis.fixedList = new Array());

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

  return Matrix.fromCSS(transform || '');
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
      .map(n => n.slice(0, -1)),
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
  let m = Matrix.identity();

  m = m.scale(canvas.width, canvas.height);

  if(vertical) m = m.rotate(-Math.PI / 2);

  m = m.scale(1 / rect.width, 1 / rect.height);
  m = m.translate(-topLeft.x, -topLeft.y);

  return m;
}

function PositionMatrix2(element = canvasElement) {
  let { rotate } = GetElementMatrix(element).decompose();
  let m = Matrix.identity();

  m = m.scale(element.width, element.height);
  m = m.scale(1 / element.offsetWidth, 1 / element.offsetHeight);

  m = m.rotate(-rotate);

  let { x, y } = element.getBoundingClientRect();

  m = m.translate(-x, -y);

  if(Math.abs(rotate) > Number.EPSILON) m = m.translate(-canvasElement.offsetHeight, 0);

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
    let t = [];

    for(let i = 0; i < n; i++) {
      const { x, y } = event.touches[i];
      t.push({ x, y });
    }

    console.log('TouchPrinter', ...t);
    yield event;
  }
}

async function* GenericPrinter(iter) {
  for await(let item of iter) {
    console.log('GenericPrinter', item);
    yield item;
  }
}

function Transformer(t) {
  return async function* (iter) {
    for await(let item of iter) yield t(item);
  };
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

  if(!('touches' in event)) event.touches = [{ clientX: event.clientX, clientY: event.clientY, x: event.x, y: event.y }];

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
  let ev = MouseToTouch(await once(element, ['mousedown', 'touchstart']));
  let type = ev.type.slice(0, 5);

  let { buttons } = ev;

  globalThis.pressed = true;

  if(!t) {
    let matrix = PositionMatrix2();
    //console.log('TouchIterator', { matrix }, matrix.decompose());
    t = TouchTransformer((x, y) => matrix.transformXY(x, y).map(Math.floor));
  }

  if(ev.touches) [...ev.touches].forEach(t);
  yield ev;

  for await(let event of streamify(['touchend', 'touchmove', 'mouseup', 'mousemove'], element, {
    passive: false,
    capture: true,
  })) {
    if(event.cancelable) event.preventDefault();
    event.stopPropagation();
    MouseToTouch(event);

    if(event.touches) [...event.touches].forEach(t);

    yield event;

    if(event.type.endsWith('end')) break;
  }

  globalThis.pressed = false;
}

async function* MoveIterator(eventIterator) {
  for await(let event of eventIterator) {
    if('touches' in event) {
      if(event.touches.length) {
        let i = 0;

        for(let touch of event.touches) {
          const { force, radiusX, radiusY, ...obj } = touch;
          yield { ...obj, type: 'touch', index: i, force, radiusX, radiusY, buttons: event.buttons };
          ++i;
        }
      }
    } else throw new Error(`No such property: touches`);
  }
}

function main() {
  define(
    globalThis,
    {
      Bresenham,
      LinkedList,
      List,
      AllParents,
      TransformationList,
      getTransformationList,
      DecomposeTransformList,
      drawRect,
      miscfixed6x13,
    },
    properties(
      {
        cid: () => lstore.cid || (lstore.cid = randStr(16, '0123456789abcdef')),
        currentURL: () => new URL(import.meta.url),
        currentFile: () => globalThis.currentURL.pathname.replace(/^\//, ''),
      },
      { memoize: true },
    ),
  );

  LoadWASM();

  document.addEventListener('contextmenu', event => event.preventDefault());

  const width = 320;
  const height = 200;
  const parent = document.body;

  crosskit.init({
    renderer: CANVAS,
    parent,
    w: width,
    h: height,
    alpha: false,
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
    angle: 0,
  });

  const buffer = new ArrayBuffer(width * (height + 2));
  const palette = CreatePalette();
  const paletteHSL = CreatePaletteHSL();

  const pixels = Array.from({ length: height + 2 }).map((v, i) => new Uint8ClampedArray(buffer, i * width, width));
  const seedlist = (globalThis.seedlist = new DrawList());
  const { context } = crosskit;
  const image = context.createImageData(width, height);
  const fps = 50;
  const matrix = new Matrix().translate(160, 100).scale(0.5);

  const animationFrame = (minDelay = 0) => {
    if(minDelay <= 0) return new Promise(resolve => requestAnimationFrame(resolve));
    const start = performance.now();

    return new Promise(resolve => {
      requestAnimationFrame(animationFrame);

      function animationFrame(t) {
        if(t - start >= minDelay) resolve(t);
        requestAnimationFrame(animationFrame);
      }
    });
  };

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

  function DrawPixel(draw, value = 255 - 0x10) {
    try {
      pixels[draw.y][draw.x] = value;

      if(draw.size > 1) {
        if(draw.x > 0) pixels[draw.y][draw.x - 1] = value;
        if(draw.x < 319) pixels[draw.y][draw.x + 1] = value;
        if(draw.y > 0) pixels[draw.y - 1][draw.x] = value;

        if(draw.y < 199) pixels[draw.y + 1][draw.x] = value;
      }
    } catch(e) {}
  }

  function Fire() {
    for(let x = 0; x < width; x++) {
      pixels[height][x] = 255 - (RandomByte() % 128);
      pixels[height + 1][x] = 255 - (RandomByte() % 128);
    }

    for(let seed of seedlist) {
      try {
        pixels[seed.y][seed.x] = 255 - (RandomByte() % 128);
      } catch(e) {}
    }

    for(let seed of seedlist.dequeue()) {
    }

    for(let draw of drawList.dequeue()) {
      DrawPixel(draw);

      draw.time += 40;

      seedlist.insert(draw);

      //Blaze(draw.x, draw.y, 255 - (RandomByte() % 128));
    }

    for(let group of fixedList) for (let draw of group) DrawPixel(draw, (randInt() & 0xff) | 0x80);

    for(let y = 0; y < height; y++) {
      for(let x = 0; x < width; x++) {
        /* prettier-ignore */ const sum = [
          pixels[y + 1][Modulo(x - 1, width)], 
          pixels[y + 1][x], 
          pixels[y + 1][Modulo(x + 1, width)], 
          pixels[y + 2][x]
        ].reduce((a, p) => a + (p | 0), 0);

        pixels[y][x] = (sum * 15) >>> 6;
      }
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

  function FixedPoints(points = []) {
    fixedList.push(points.map(({ x, y }) => ({ x, y, time: performance.now() })));
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
      [0, r, 0],
    ]);
  }

  function PutArray(x, y, a) {
    let rows = a.length;
    let cols = a[0].length;
    let w = pixels[0].length;
    let h = pixels.length;

    for(let ty = 0; ty < rows; ++ty) for (let tx = 0; tx < cols; ++tx) pixels[(y + ty) % h][(x + tx) % w] += a[ty][tx] / 2;
  }

  function PutArray2(x, y, a) {
    let rows = a.length * 2;
    let cols = a[0].length * 2;
    let w = pixels[0].length;
    let h = pixels.length;

    for(let ty = 0; ty < rows; ++ty) for (let tx = 0; tx < cols; ++tx) pixels[(y + ty) % h][(x + tx) % w] = a[ty >> 1][tx >> 1];
  }

  Object.assign(globalThis, { randInt, Draw, FixedPoints, Bresenham, RandomByte, Blaze });

  NewWS({
    onOpen() {
      console.log('WS connected!');

      const { cid } = globalThis;

      SendWS({ type: 'hello', cid });
      /*SendWS({ type: 'rects', cid, rects: GetRects() });*/
    },
  });

  let str = '';
  let xpos = 0;

  function KeyHandler(key, pressed) {
    console.log('KeyHandler', { xpos, key, pressed });

    if(key == 'Shift') {
      globalThis.shiftPressed = pressed;
    } else if(key in miscfixed6x13) {
      PutArray2(xpos, 100, miscfixed6x13[key]);
      xpos += 14;
    }
  }

  (async () => {
    for await(let e of streamify(['keydown', 'keyup'], window, { passive: false })) {
      const { type, key, keyCode, repeat, ctrlKey, shiftKey, altKey, metaKey } = (globalThis.keyEvent = e);

      if(!ctrlKey && !altKey && !metaKey) if (e.cancelable) e.preventDefault();

      KeyHandler(key, type.endsWith('down'));
    }
  })();

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
        { start: t },
      );
    })();

    const trail = (globalThis.trail = []);

    for(;;) {
      let prev,
        pt,
        start = timegen(),
        last,
        iter = chainRight(MoveIterator, /*GenericPrinter, */ TouchIterator)(window);

      for await(let ev of iter) {
        globalThis.movement = ev;

        //console.log(ev);

        let a;

        if(ev.buttons & 2) fixedList.push((a = []));
        let add = ev.buttons & 2 ? pt => a.push(pt) : pt => drawList.insert(pt);

        if(gfxRect.inside(ev)) {
          let pt = (globalThis.mousePos = new Point(ev));
          let diff = pt,
            t;

          if(prev) diff = pt.diff(prev);

          let time = timegen();
          let obj = { ...pt, time };

          trail.push(obj);

          if(prev) {
            for(let [x, y] of Bresenham(prev.x, prev.y, pt.x, pt.y)) add({ x, y, time });
          } else add(obj);

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

          //console.log('SendTrail', payload);

          SendWS(payload);
        }

        trail.splice(0, trail.length);
        start = undefined;
        prev = undefined;
      }
    }
  }

  tryCatch(
    () => Loop(),
    () => null,
    error => SendWS({ type: 'exception', cid: globalThis.cid, message: error.message, stack: error.stack }),
  );
}

function Init() {
  // window.canvas = element = document.querySelector('canvas');

  define(
    globalThis,
    properties(
      {
        canvasElement: () => Element.find('canvas'),
        divElement: () => Element.find('body > div:first-child'),
        htmlElement: () => document.documentElement,
        gfxRect: () => new Rect(0, 0, canvasElement.width, canvasElement.height),
      },
      { memoize: true },
    ),
    properties({
      windowRect: () => new Rect(0, 0, window.innerWidth, window.innerHeight),
      windowSize: () => new Size(window.innerWidth, window.innerHeight),
      scrollPos: () => new Point(window.scrollX, window.scrollY),
      bodyRect: () => Element.rect('body').round(0),
      canvasRect: () => Element.rect('canvas').round(0),
    }),
    properties({
      transform: [() => new TransformationList(Element.getCSS('body > div:first-child').transform), value => Element.setCSS('body > div:first-child', { transform: value + '' })],
    }),
  );

  const SetLocked = ToggleClass(htmlElement, 'is-locked');
  const SetPressed = ToggleClass(canvasElement, 'pressed');

  define(globalThis, properties({ pressed: [SetPressed, SetPressed] }));

  globalThis.circle = trkl(new Point(0, 0));
  globalThis.points = trkl([]);

  const SVGPolyline = ({ points, ...props }) => h('polyline', { points: points.map(pt => [...pt].join(',')).join(' '), ...props });

  const SVGComponent = ({ circle, points, ...props }) => {
    let rect = new Rect(0, 0, canvasElement.width, canvasElement.height);

    const { r = 10, x, y, width = '1', stroke = '#0f0', fill = `rgba(80,80,80,0.3)` } = useTrkl(circle);

    return h(
      'svg',
      {
        version: '1.1',
        xmlns: 'http://www.w3.org/2000/svg',
        viewBox: [...rect].join(' '),
        width: rect.width,
        height: rect.height,
      },
      [
        h('circle', {
          cx: x,
          cy: y,
          r,
          stroke,
          'stroke-width': width,
          fill,
        }),
        h(SVGPolyline, { points: useTrkl(points), stroke, 'stroke-width': width }),
      ],
    );
  };

  /*  let svgContainer = (globalThis.svgContainer = Element.create('div', { class: 'overlay' }, document.body));

  render(h(SVGComponent, { circle: globalThis.circle, points: globalThis.points }), svgContainer);

  globalThis.svgElement = svgContainer.firstElementChild;*/

  //  Element.setCSS(svgContainer, { position: 'absolute', left: canvasRect.x + 'px', top: canvasRect.y + 'px' });
  /*  rect = canvasRect;
    mouseTransform = PositionProcessor();*/

  (async function() {
    ResizeHandler();

    for await(let event of streamify(['orientationchange', 'resize'], window)) {
      ResizeHandler();
    }
  })();

  syncHeight();
  window.addEventListener('resize', syncHeight);

  SetLocked(true);
}

function ResizeHandler() {
  let { width, height } = Element.rect('body');

  console.log('ResizeHandler', { width, height });

  if(globalThis.svgElement)
    Element.setCSS(globalThis.svgElement, {
      width: canvasElement.offsetWidth + 'px',
      height: canvasElement.offsetHeight + 'px',
      transform: 'rotate(90deg)',
    });
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

function NewWS(handlers) {
  let url = WebSocketURL('/ws', { mirror: currentFile });
  let rws = new ReconnectingWebSocket(url, 'lws-mirror-protocol', handlers ?? {});

  define(globalThis, {
    get ws() {
      return rws.socket;
    },
  });

  (async function() {
    let chunks = '',
      data;

    for await(let chunk of rws) {
      chunks += chunk;

      if(/}\s*$/.test(chunks)) {
        if(!(data = ParseJSON(chunks))) {
          chunks = chunks.slice(chunks.indexOf('{'));
          continue;
        }

        if(data.type != 'event') if (!data.cid || globalThis.cid != data.cid) console.log('WS receive:', data);

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
          default: {
            console.log('WS received:', data);
            break;
          }
        }

        chunks = '';
      }
    }
  })();

  return rws;
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
    instance: { exports },
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

  for(let element of [...AllParents(Element.find('canvas'))]) rects.push([ElementName(element), Element.rect(element), new Point(element.scrollLeft, element.scrollTop)]);

  rects.push(['window', new Rect(0, 0, window.innerWidth, window.innerHeight), new Point(window.scrollX, window.scrollY)]);

  return rects;
}

function SendWS(msg) {
  if(ws.readyState != ws.OPEN) return;
  if(!('cid' in msg)) msg = { cid: globalThis.cid, ...msg };

  //console.log('WS send:',msg);

  if(typeof msg != 'string') msg = JSON.stringify(msg);

  return ws.send(msg);
}

function ToggleClass(element, name) {
  return (...args) => element.classList[args.length > 0 ? (args[0] ? 'add' : 'remove') : 'contains'](name);
}

function syncHeight() {
  htmlElement.style.setProperty('--window-inner-height', `${window.innerHeight}px`);
}

main();
