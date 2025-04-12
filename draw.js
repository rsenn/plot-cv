import { streamify, subscribe } from './lib/async/events.js';
import { getLocalStorage, logStoreAdapter, makeAutoStoreHandler, makeDummyStorage, makeLocalStorage, makeLocalStore } from './lib/autoStore.js';
import { HSLA, RGBA } from './lib/color.js';
import { Element } from './lib/dom/element.js';
import { Fragment, h, render } from './lib/dom/preactComponent.js';
import { SVG } from './lib/dom/svg.js';
import { SVG as SVGComponent } from './lib/eagle/components/svg.js';
import { Align, Point, PointList, Rect, Size, Line, BBox } from './lib/geom.js';
import { Arc, ArcTo } from './lib/geom/arc.js';
import { Matrix } from './lib/geom/matrix.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { KolorWheel } from './lib/KolorWheel.js';
import Cache from './lib/lscache.js';
import { define, range, unique, waitFor, isObject } from './lib/misc.js';
import trkl from './lib/trkl.js';
import { ReconnectingWebSocket, WebSocketURL } from './lib/async/websocket.js';
import { getMethodNames } from './lib/misc.js';

let ref = (globalThis.ref = trkl(null));
let svgElem;
let screenCTM, svgCTM;
let anchorPoints = (globalThis.anchorPoints = trkl([]));
let arc = (globalThis.arc = Tracked({ start: { x: 10, y: 10 }, end: { x: 20, y: 20 }, curve: 90 }));
let ls = (globalThis.ls = new Storage());

function* R(s = 0, inc = 1, e) {
  for(let i = s; i < e; i += inc) yield i;
}

function* Up(e, k = 'parentNode') {
  for(; e; ) {
    yield e;
    if(e[Symbol.toStringTag] == 'HTMLDocument' && e[k] == null) e = window;
    else e = e[k];
  }
}

function Log(...args) {
  if(globalThis.ws) {
    try {
      sendMessage({ command: 'log', args });
    } catch(error) {
      console.error('Log() error:', error);
    }
  }
  console.log(...args);
}

function isComponent(obj) {
  if(typeof obj == 'object' && obj !== null) return 'props' in obj;
}

function Storage() {
  let ls = localStorage;
  return new Proxy(
    {},
    {
      get(target, prop, receiver) {
        if(typeof prop == 'string') {
          let value = ls.getItem(prop);
          if(typeof value == 'string') return JSON.parse(value);
        }
      },
      set(target, prop, value) {
        if(typeof prop == 'string') ls.setItem(prop, JSON.stringify(value));
      },
      ownKeys(target) {
        let i,
          ret = [];
        for(i = 0; i < ls.length; i++) ret.push(ls.key(i));
        return ret;
      }
    }
  );
}

function* WalkUp(elem) {
  do {
    yield elem;
  } while((elem = elem.parentElement));
}

function IterateSome(iter, pred = elem => false) {
  let i = 0;
  for(let elem of iter) {
    let ret;
    if((ret = pred(elem, i++))) return ret;
  }
}

function GetAllPropertyNames(obj) {
  const ret = new Set();

  do {
    if(obj == Object.prototype) break;

    for(let name of Object.getOwnPropertyNames(obj)) ret.add(name);
  } while((obj = Object.getPrototypeOf(obj)));

  return [...ret];
}

function CloneObject(obj, pred = (key, value) => true) {
  let names = GetAllPropertyNames(obj);
  let ret = {};

  for(let name of names) {
    if(!pred(name, obj[name])) continue;
    let value = obj[name];
    if(typeof value == 'function' && name != 'constructor') value = (...args) => value.call(obj, ...args);
    ret[name] = value;
  }
  return ret;
}

const Table = props => {
  return h(
    'table',
    { cellspacing: 0, cellpadding: 0 },
    (props.rows ?? props.children).map(row =>
      Array.isArray(row)
        ? h(
            'tr',
            {},
            row.map(cell => (isComponent(cell) ? cell : h('td', {}, [cell])))
          )
        : row
    )
  );
};

function Observable(obj, change = (prop, value) => {}) {
  return new Proxy(obj, {
    get(target, prop, receiver) {
      if(prop == 'wrapped') return obj;
      return Reflect.get(target, prop, receiver);
    },
    set(target, prop, value) {
      let oldValue = target[prop];
      Reflect.set(target, prop, value);
      if(oldValue != value) change.call(obj, prop, value);
    }
  });
}

function Tracked(obj) {
  let ret, obsrv;
  obsrv = Observable(obj, (prop, value) => {
    ret(obsrv);
  });
  ret = trkl(obsrv);
  return ret;
}

const AnchorPoints = ({ points, ...props }) => {
  let data = useTrkl(points);
  return h(
    'g',
    {},
    data.map((pt, i) => {
      let j = data.length - i;
      return h('circle', {
        'data-index': `point-${i}`,
        cx: pt.x,
        cy: pt.y,
        r: devicePixelRatio > 1 ? 3 : 1,
        stroke: 'black',
        'stroke-width': devicePixelRatio > 1 ? 0.2 : 0.1,
        fill: `hsla(${data.length > 1 ? (i * 300) / (data.length - 1) : 0}, 100%, 50%, 0.8)`
      });
    })
  );
};

const Path = ({ points, lineCommand = 'L', ...props }) => {
  let data = [...useTrkl(points)];
  if(data.length == 0) return null;

  let d = `M ${data.shift()}` + data.reduce((acc, pt) => acc + ` ${(pt.lineCommand ?? lineCommand).toUpperCase()} ${pt}`, '');
  return h('path', { d, fill: 'none', stroke: 'black', 'stroke-width': devicePixelRatio > 1 ? 0.2 : 0.1 });
};

const EllipticArc = ({ data, ...props }) => {
  let { start, end, curve } = useTrkl(data);
  return h('path', {
    d: `M ${new Point(start)} ` + ArcTo(end.x - start.x, end.y - start.y, curve),
    fill: 'none',
    stroke: 'black',
    'stroke-width': 0.1
  });
};

function AddPoint(pt) {
  const l = anchorPoints();

  //Log('AddPoint', l.length, pt);
  sendMessage({ type: 'add', index: l.length, point: { ...pt.toObject() } });

  anchorPoints(l.concat([pt]));
}

function GetElementSignal(elem) {
  return elem.getAttribute('data-signal');
}

function GetElementsBySignal(signalName) {
  return Element.findAll('*[data-signal=' + signalName.replace(/(\$)/g, '\\$1') + ']');
}

/*function SortElementsByPosition(elements) {
  let entries = elements.map(e => [e, new Point(Element.rect(e).corners()[0])]);
  globalThis.entries = entries;
  entries.sort((a, b) => a[1].valueOf(-16) - b[1].valueOf(-16));
  Log('SortElementsByPosition', { entries });
  return entries.map(([e, rect]) => e);
}*/

/*function CreateElement(pos, size) {
  let { x, y } = new Point(pos);
  let { width, height } = size ? new Size(size) : {};
  let style = { background: 'red', left: `${x}px`, top: `${y}px`, position: 'fixed', margin: `0 0 0 0` };

  if(width) style.width = `${width}px`;
  if(height) style.height = `${height}px`;
  return Element.create(
    'div',
    {
      innerHTML: 'x',
      style
    },
    document.body
  );
}*/

function GetCirclePosition(element) {
  let x = +element.getAttribute('cx');
  let y = +element.getAttribute('cy');
  return new Point(x, y);
}

function FindPoint(pos, f = (i, d, p) => p) {
  let [i, d, pt] = PointList.prototype.nearest.call(anchorPoints(), pos, (i, d, p) => [i, d, p]);
  return f(i, d, pt);
}

function MouseEvents(element) {
  const isTouch = 'ontouchstart' in element;
  return streamify(isTouch ? ['touchmove', 'touchend'] : ['mousemove', 'mouseup'], element, isTouch ? e => !e.type.endsWith('end') : e => !e.type.endsWith('up'));
}

async function* TouchEvents(element) {
  let touch;

  for await(let event of MouseEvents(element)) {
    const { type } = event;

    if(!type.endsWith('end')) {
      if(event.touches)
        for(let t of [...event.touches]) {
          touch = CloneObject(t);

          yield define(CloneObject(event), touch);
        }
      else yield CloneObject(event);
    } else {
      try {
        define(event, touch);

        yield event;
      } catch(e) {}
    }
  }
}

function GetSignalNames() {
  return unique(Element.findAll(`*[data-signal]`).map(e => e.getAttribute('data-signal'))).sort((a, b) => (a == 'GND' ? -1 : a == 'VCC' ? (b != 'GND' ? -1 : 1) : 1 ?? a.localeCompare(b)));
}

function GetSignalColor(signalName) {
  for(let elem of Element.findAll('*[data-signal=' + signalName.replace(/(\$)/g, '\\$1') + ']')) {
    let stroke = elem.getAttribute('stroke');
    if(stroke) return stroke;
  }
}

function* GetSignalEntries() {
  for(let name of GetSignalNames()) yield [name, GetSignalColor(name)];
}

function ColorSignals() {
  let names = GetSignalNames();
  let palette = MakePalette(names.length);
  globalThis.palette(palette);

  Log('ColorSignals', { names, palette });
  let i = 0;
  for(let name of names) {
    ColorSignal(name, palette[i++]);
  }
}

function RenderPalette(props) {
  let data = useTrkl(props.data);
  let names = GetSignalNames();
  return h(
    Table,
    {},
    (data ?? []).map((color, i) => [names[i] ?? i + '', h('td', { style: { background: color, 'min-width': '1em' } })])
  );
}

function ColorSignal(signalName, color) {
  for(let elem of Element.findAll('*[data-signal=' + signalName.replace(/(\$)/g, '\\$1') + ']')) {
    elem.setAttribute('stroke', color);
  }
}

function ZoomHandler(e) {
  const { deltaY, screenX: x, screenY: y } = e;

  this.zoomPos = (this.zoomPos ?? 0) - deltaY / 1000;

  const factor = 10 ** this.zoomPos;

  Element.setCSS(this, {
    transform: `translate(${-x}px, ${-y}px) scale(${factor}, ${factor}) translate(${x / factor}px, ${y / factor}px) `
  });

  define(this, {
    get zoomFactor() {
      return this.getScreenCTM().a / this.getCTM().a;
    }
  });
}

async function LoadSVG(filename) {
  let response = await fetch(filename);
  let data = await response.text();
  let elem = Element.create('div', { class: 'svg' }, document.body);
  const { body } = document;
  elem.innerHTML = data;
  elem.insertBefore(Element.create('h4', { innerHTML: filename }, []), elem.children[0]);
  const isTouch = 'ontouchstart' in elem;

  elem.onmousewheel = ZoomHandler;

  elem[isTouch ? 'ontouchstart' : 'onmousedown'] = MoveHandler;

  async function MoveHandler(e) {
    let { clientX: startX, clientY: startY, type, touches, target, currentTarget } = e;
    startX ??= touches[0].clientX;
    startY ??= touches[0].clientY;

    Log('MoveHandler', { type, startX, startY, touches });
    let rect = new Rect(Element.getRect(elem));

    for await(let ev of TouchEvents(document.body)) {
      let { clientX: x, clientY: y } = ev;
      Log('MoveHandler', { x, y });
      let rel = new Point({ x: x - startX, y: y - startY });
      let pos = new Rect(...rel.sum(rect.upperLeft), rect.width, rect.height);
      Element.move(elem, pos.upperLeft);
    }
  }

  Element.setCSS(elem, { position: 'absolute', left: '0px', top: '0px' });
  Element.setCSS(elem.lastElementChild, { transform: 'scale(-1, 1)' });

  return elem;
}

function MakePalette2(num) {
  let result = [];
  new KolorWheel('#0000ff').abs(0, -1, -1, num - 1).each(function () {
    result.push(this.getHex());
  });
  result.push('#000000');
  result.reverse();
  return result;
}

function MakePalette(num) {
  let signalColors = `#000000
#e02231
#ff9b00
#e4e20b
#68db19
#197dff
#a138ff
#b6b6b6`.split('\n');

  let result = [];

  signalColors.shift();
  signalColors.pop();

  for(let i = 0; i < num - 2; i++) result.push(signalColors[i % signalColors.length]);

  result.unshift('#e02231');
  result.unshift('#000000');
  globalThis.palette(result);

  return result;
}
function sendMessage(msg) {
  return globalThis.ws.send(JSON.stringify(msg));
}

async function CreateSocket(endpoint) {
  const url = WebSocketURL('/ws?mirror=draw.js');
  const rws = (globalThis.rws = new ReconnectingWebSocket(url, 'ws', {
    onOpen({ target: ws }) {
      console.log('ReconnectingWebSocket connected!', ws);
    },
    onMessage(e) {
      console.log('onMessage', e);
    }
  }));

  define(globalThis, {
    get ws() {
      return rws.socket;
    }
  });

  await rws.connect(endpoint);

  //ws.addEventListener('close', () => (ws.sendMessage = null));

  return rws;
}

Object.assign(globalThis, {
  Matrix,
  Point,
  PointList,
  Line,
  BBox,
  Align,
  Rect,
  Element,
  SVG,
  GetCirclePosition,
  FindPoint,
  MouseEvents,
  TouchEvents,
  Arc,
  ArcTo,
  Tracked,
  LoadSVG,
  ColorSignal,
  GetSignalNames,
  GetSignalColor,
  KolorWheel,
  MakePalette,
  RenderPalette,
  ColorSignals,
  Table,
  GetSignalEntries,
  define,
  range,
  WalkUp,
  IterateSome,
  GetAllPropertyNames,
  CloneObject,
  Cache,
  makeLocalStorage,
  logStoreAdapter,
  makeLocalStore,
  makeDummyStorage,
  getLocalStorage,
  makeAutoStoreHandler,
  Storage,
  GetElementSignal,
  GetElementsBySignal,
  h,
  isComponent,
  RGBA,
  HSLA,
  waitFor,
  CreateSocket,
  sendMessage,
  Log,
  Up,
  getMethodNames,
  streamify,
  R,
  EllipticArc,
  ZoomHandler
});

define(globalThis, {
  get points() {
    return new PointList(anchorPoints());
  },
  get lines() {
    return [...points.lines()];
  }
});
window.addEventListener('contextmenu', event => event.preventDefault());

window.addEventListener('load', async e => {
  let element = document.querySelector('#preact');
  document.querySelector('span').style.setProperty('display', 'none');

  await CreateSocket();
  ref.subscribe(value => {
    svgElem = globalThis.svgElem = value.base;
    Log('ref', value);

    svgCTM = globalThis.svgCTM = Matrix.fromDOM(svgElem.getScreenCTM()).invert();
    screenCTM = globalThis.screenCTM = Matrix.fromDOM(svgElem.getScreenCTM());

    const isTouch = 'ontouchstart' in svgElem;
    if(isTouch) svgElem.addEventListener('touchstart', ev => ev.preventDefault());

    Log('isTouch', isTouch);

    (async function() {
      for await(let e of streamify(isTouch ? 'touchstart' : 'mousedown', svgElem)) {
        const { type } = e;
        const { clientX: x, clientY: y, target, currentTarget } = e;

        let xy,
          pt,
          d,
          index,
          pos = new Point(...svgCTM.transformXY(x, y));

        if(pos.x === undefined || pos.y === undefined) pos = new Point(...svgCTM.transformXY(e.touches[0].clientX, e.touches[0].clientY));

        [index, d, pt] = FindPoint(pos, (i, d, pt) => [i, d, pt]);

        if(d >= 10) (index = undefined), (pt = undefined);

        if(target.tagName != 'svg') {
          xy = GetCirclePosition(target);

          index = +(target.getAttribute('data-index') ?? '--1').replace(/^[^-]*-/g, '');
          pt = anchorPoints()[index];
        }

        if(pt) {
          if(e.buttons == 2) {
            let l = anchorPoints();
            l.splice(index, 1);
            anchorPoints([...l]);
            continue;
          }

          Log('Move', { pos, index, d, pt });
        }
        globalThis.ev = e;
        globalThis.pos = pos;

        if(index === undefined) index = anchorPoints().length;

        if(!pt) AddPoint((pt = pos.clone()));

        await MovePoint(index, pt);

        async function MovePoint(index, p) {
          for await(let ev of TouchEvents(svgElem)) {
            let { clientX, clientY, type } = ev;
            let tpos = new Point(...svgCTM.transformXY(clientX ?? x, clientY ?? y));
            let diff = tpos.diff(pos);

            pt.x = tpos.x;
            pt.y = tpos.y;

            p.x = pt.x;
            p.y = pt.y;

            if(diff.distance()) sendMessage({ type: 'move', index, point: { ...p.toObject() } });

            anchorPoints([...anchorPoints()]);

            console.log(`touch[${type}] ${(pt + '').padStart(20)} ${(diff + '').padStart(20)}`);

            if(!type.endsWith('move')) {
              break;
            }
          }
        }
      }
    })();
  });

  let component = h(Fragment, {}, [
    h(SVGComponent, { ref, viewBox: new BBox(0, 0, 160, 100), height: '100vh', width: '100vw' }, [
      // h('circle', { cx: 30, cy: 30, r: 2, stroke: 'red', 'stroke-width': 0.1, fill: 'none' }),
      h(Path, { points: anchorPoints, lineCommand: 'L' }, []),
      h(AnchorPoints, { points: anchorPoints }, [])
      // h(EllipticArc, { data: arc }, [])
    ]),
    h(RenderPalette, { data: (globalThis.palette = trkl([])) })
  ]);

  render(component, element);

  //false &&
  (async () => {
    await LoadSVG('Mind-Synchronizing-Generator-PinHdrPot-board.svg');
    await waitFor(1000);

    Element.find('svg > #bg').style.setProperty('pointer-events', 'none');
    ColorSignals();
  })();

  for await(let event of streamify(['mousemove', 'touchmove'], document.body, e => true)) {
    let { clientX: x, clientY: y, target, currentTarget, type } = event;

    if(event.touches) {
      const touches = [...event.touches];
      if(touches.length) {
        x ??= touches[touches.length - 1].clientX;
        y ??= touches[touches.length - 1].clientY;
      }
    }

    let elements = [...document.elementsFromPoint(x, y)].filter(IsSVGElement);

    // Log('Touch', { elements });

    if(elements.length) {
      let signals = elements.map(e => [e, GetElementSignal(e)]).filter(([e, sig]) => sig != null);
      let signal = (signals[0] ?? [])[1];
      if(signal) globalThis.elements = GetElementsBySignal(signal);
    }

    function IsSVGElement(elem) {
      return IterateSome(WalkUp(elem), (e, i) => i > 0 && e.tagName == 'svg');
    }
  }
});
