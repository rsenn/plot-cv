import React, { Fragment, h, render, Component } from './lib/dom/preactComponent.js';
import { SVG as SVGComponent } from './lib/eagle/components/svg.js';
import { BBox } from './lib/geom/bbox.js';
import trkl from './lib/trkl.js';
import { Matrix } from './lib/geom/matrix.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { Align, Point, Rect, PointList, Size } from './lib/geom.js';
import { Element } from './lib/dom/element.js';
import { SVG } from './lib/dom/svg.js';
import { HSLA, RGBA } from './lib/color.js';
import { streamify, map, subscribe } from './lib/async/events.js';
import { Arc, ArcTo } from './lib/geom/arc.js';
import { unique, define, range } from './lib/misc.js';
import { KolorWheel } from './lib/KolorWheel.js';
import Cache from './lib/lscache.js';
import { makeLocalStorage, logStoreAdapter, makeLocalStore, makeDummyStorage, getLocalStorage, makeAutoStoreHandler } from './lib/autoStore.js';

let ref = (globalThis.ref = trkl(null));
let svgElem;
let screenCTM, svgCTM;
let anchorPoints = (globalThis.anchorPoints = trkl([]));
let arc = (globalThis.arc = Tracked({ start: { x: 10, y: 10 }, end: { x: 20, y: 20 }, curve: 90 }));
let ls = (globalThis.ls = new Storage());

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
    data.map(pt => {
      return h('circle', {
        cx: pt.x,
        cy: pt.y,
        r: 0.8,
        stroke: 'black',
        'stroke-width': 0.1,
        fill: 'rgba(255,0,0,0.8)'
      });
    })
  );
};

const Path = ({ points, ...props }) => {
  let data = [...useTrkl(points)];

  if(data.length == 0) return null;

  let d = `M ${data.shift()}` + data.reduce((acc, pt) => acc + ` L ${pt}`, '');

  return h('path', { d, fill: 'none', stroke: 'black', 'stroke-width': 0.1 });
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
  anchorPoints(anchorPoints().concat([pt]));
}

function GetElementSignal(elem) {
  return elem.getAttribute('data-signal');
}

function GetElementsBySignal(signalName) {
  return Element.findAll(`*[data-signal=${signalName.replace(/([\$])/g, '\\$1')}]`);
}

function SortElementsByPosition(elements) {
  let entries = elements.map(e => [e, new Point(Element.rect(e).corners()[0])]);
  globalThis.entries = entries;
  entries.sort((a, b) => a[1].valueOf(-16) - b[1].valueOf(-16));
  console.log('SortElementsByPosition', { entries });
  return entries.map(([e, rect]) => e);
}

function CreateElement(pos, size) {
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
}

function GetPosition(element) {
  let x = +element.getAttribute('cx');
  let y = +element.getAttribute('cy');
  return new Point(x, y);
}

function FindPoint(pos) {
  let pt = anchorPoints().find(pt2 => Point.equals(pt2, pos));
  return pt;
}

function MouseEvents(element) {
  const isTouch = 'ontouchstart' in element;
  return streamify(
    isTouch ? ['touchmove', 'touchend'] : ['mousemove', 'mouseup'],
    element,
    isTouch ? e => !e.type.endsWith('end') : e => !e.type.endsWith('up')
  );
}

async function* TouchEvents(element) {
  let touches;
  for await(let event of MouseEvents(element)) {
    const { type } = event;
    // console.log('TouchEvents', { type,event });

    if('touches' in event && event.touches.length) {
      touches = [...event.touches];

      for(let touch of touches) {
        //define(globalThis, { event, touch });
        yield define(CloneObject(event), CloneObject(touch));
      }
    } else {
      if(type.endsWith('end')) event = define(event, CloneObject(touches[touches.length - 1]));

      yield event;
    }
  }
}

function GetSignalNames() {
  return unique(Element.findAll(`*[data-signal]`).map(e => e.getAttribute('data-signal'))).sort((a, b) =>
    a == 'GND' ? -1 : a == 'VCC' ? (b != 'GND' ? -1 : 1) : 1 ?? a.localeCompare(b)
  );
}

function GetSignalColor(signalName) {
  for(let elem of Element.findAll(`*[data-signal=${signalName.replace(/([\$])/g, '\\$1')}]`)) {
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

  console.log('ColorSignals', { names, palette });
  let i = 0;
  for(let name of names) {
    ColorSignal(name, palette[i++]);
  }
}

function RenderPalette(props) {
  let data = useTrkl(props.data);
  return h(
    Table,
    {},

    (data ?? []).map((color, i) => [i + '', h('td', { style: { background: color, 'min-width': '1em' } })])
  );
}

function ColorSignal(signalName, color) {
  for(let elem of Element.findAll(`*[data-signal=${signalName.replace(/([\$])/g, '\\$1')}]`)) {
    //console.log('ColorSignal', elem);
    elem.setAttribute('stroke', color);
  }
}

async function LoadSVG(filename) {
  let response = await fetch(filename);
  let data = await response.text();

  //console.log('LoadSVG', { filename, data });

  let elem = Element.create('div', { class: 'svg' }, document.body);
  const { body } = document;
  elem.innerHTML = data;
  elem.insertBefore(Element.create('h4', { innerHTML: filename }, []), elem.children[0]);
  let zoomPos = 0,
    zoomFactor = 1;
  const isTouch = 'ontouchstart' in elem;

  elem.onmousewheel = ZoomHandler;

  elem[isTouch ? 'ontouchstart' : 'onmousedown'] = MoveHandler;

  function ZoomHandler(e) {
    const { deltaY, screenX: x, screenY: y } = e;
    zoomPos -= deltaY / 1000;
    zoomFactor = Math.pow(10, zoomPos);

    //console.log('wheel', { x, y, zoomFactor });

    Element.setCSS(elem, {
      transform: `translate(${-x}px, ${-y}px) scale(${zoomFactor}, ${zoomFactor}) translate(${x / zoomFactor}px, ${
        y / zoomFactor
      }px) `
    });
  }

  async function MoveHandler(e) {
    let { clientX: startX, clientY: startY, type, touches, target, currentTarget } = e;
    startX ??= touches[0].clientX;
    startY ??= touches[0].clientY;

    console.log('MoveHandler', { type, startX, startY, touches });
    let rect = new Rect(Element.getRect(elem));
    for await(let ev of TouchEvents(document.body)) {
      let { clientX: x, clientY: y } = ev;
      console.log('MoveHandler', { x, y });
      let rel = new Point({ x: x - startX, y: y - startY });
      let pos = new Rect(...rel.sum(rect.upperLeft), rect.width, rect.height);
      Element.move(elem, pos.upperLeft);
    }
  }

  Element.setCSS(elem, { position: 'absolute', left: '0px', top: '0px' });

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
  //let cl=new range(0,350, (360/num)).reverse().map(r => new HSLA(r, 100,50,1));
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
  signalColors.shift();
  signalColors.pop();

  for(let i = 0; i < num - 2; i++) result.push(signalColors[i % signalColors.length]);

  //let result=cl.map(c => c.toRGBA()+'')
  result.unshift('#e02231');
  result.unshift('#000000');
  //result.reverse();
  globalThis.palette(result);
  return result;
}

Object.assign(globalThis, {
  Matrix,
  Point,
  PointList,
  Align,
  Rect,
  Element,
  SVG,
  CreateElement,
  GetPosition,
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
  SortElementsByPosition,
  h,
  isComponent,
  RGBA,
  HSLA
});

window.addEventListener('load', async e => {
  let element = document.querySelector('#preact');

  ref.subscribe(value => {
    svgElem = globalThis.svgElem = value.base;
    console.log('ref', value);

    svgCTM = globalThis.svgCTM = Matrix.fromDOM(svgElem.getScreenCTM()).invert();
    screenCTM = globalThis.screenCTM = Matrix.fromDOM(svgElem.getScreenCTM());

    const isTouch = 'ontouchstart' in svgElem;

    svgElem.addEventListener(isTouch ? 'touchstart' : 'mousedown', async e => {
      const { type } = e;
      console.log('on' + type, e);

      const { clientX: x, clientY: y, target, currentTarget } = e;

      let xy,
        pt,
        pos = new Point(...svgCTM.transform_xy(x, y));

      if(target.tagName != 'svg') {
        Object.assign(globalThis, { target });

        xy = GetPosition(target);
        pt = FindPoint(xy);
      }

      if(!pt) AddPoint(pos);
      else MovePoint();
      //   console.log('touchstart', { xy, pt });

      async function MovePoint() {
        for await(let ev of TouchEvents(svgElem)) {
          let { clientX, clientY } = ev;
          let tpos = new Point(...svgCTM.transform_xy(clientX, clientY));

          let diff = tpos.diff(pos);

          pt.x = tpos.x;
          pt.y = tpos.y;
          anchorPoints([...anchorPoints()]);

          console.log('touch', { pt, diff });
        }
      }

      console.log('mousedown', { x, y, target, currentTarget });
    });
  });

  let component = h(Fragment, {}, [
    h(SVGComponent, { ref, viewBox: new BBox(0, 0, 160, 100) }, [
      h('circle', { cx: 30, cy: 30, r: 2, stroke: 'red', 'stroke-width': 0.1, fill: 'none' }),
      h(Path, { points: anchorPoints }, []),
      h(AnchorPoints, { points: anchorPoints }, []),
      h(EllipticArc, { data: arc }, [])
    ]),
    h(RenderPalette, { data: (globalThis.palette = trkl([])) })
  ]);

  render(component, element);

  setTimeout(() => {
    LoadSVG('Mind-Synchronizing-Generator-PinHdrPot-board.svg').then(() => {
      setTimeout(() => {
        Element.find('svg > #bg').style.setProperty('pointer-events', 'none');

        ColorSignals();
      }, 1000);
    }, 100);
  });

  for await(let event of streamify(['mousemove', 'touchmove'], document.body, e => true)) {
    const { clientX: x, clientY: y, target, currentTarget } = event;

    let elements = [...document.elementsFromPoint(x, y)].filter(IsSVGElement);

    if(elements.length) {
      let signals = elements.map(e => [e, GetElementSignal(e)]).filter(([e, sig]) => sig != null);
      let signal = (signals[0] ?? [])[1];

      if(signal) globalThis.elements = GetElementsBySignal(signal);

      console.log('move', { x, y, signal });
    }

    function IsSVGElement(elem) {
      return IterateSome(WalkUp(elem), (e, i) => i > 0 && e.tagName == 'svg');
    }
  }
});
