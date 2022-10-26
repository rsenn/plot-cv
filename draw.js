import React, { h, html, render, Fragment, Component, createRef, useState, useLayoutEffect, useRef, toChildArray } from './lib/dom/preactComponent.js';
import { SVG } from './lib/eagle/components/svg.js';
import { BBox } from './lib/geom/bbox.js';
import { PinSizes, Circle } from './lib/eagle/components/circle.js';
import trkl from './lib/trkl.js';
import { Matrix, isMatrix, ImmutableMatrix } from './lib/geom/matrix.js';
import { useTrkl } from './lib/hooks/useTrkl.js';
import { Point, isPoint, ImmutablePoint } from './lib/geom/point.js';
import { Element, isElement } from './lib/dom/element.js';
import { once, streamify, filter, map, throttle, distinct, subscribe } from './lib/async/events.js';
import { Arc, ArcTo } from './lib/geom/arc.js';

let ref = (globalThis.ref = trkl(null));
let svgElem;
let screenCTM, svgCTM;
let anchorPoints = (globalThis.anchorPoints = trkl([]));
let arc = (globalThis.arc = Tracked({ start: { x: 10, y: 10 }, end: { x: 20, y: 20 }, curve: 90 }));

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

function CreateElement(pos) {
  let { x, y } = pos;

  Element.create(
    'div',
    {
      innerHTML: 'x',
      style: { background: 'red', left: `${x}px`, top: `${y}px`, position: 'fixed', margin: `0 0 0 0` }
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

function TouchEvents(element) {
  return streamify(['mousemove', 'mouseup'], element, e => !e.type.endsWith('up'));
}

Object.assign(globalThis, {
  Matrix,
  Point,
  Element,
  CreateElement,
  GetPosition,
  FindPoint,
  TouchEvents,
  Arc,
  ArcTo,
  Tracked
});

window.addEventListener('load', e => {
  let element = document.querySelector('#preact');

  ref.subscribe(value => {
    svgElem = globalThis.svgElem = value.base;
    console.log('ref', value);

    svgCTM = globalThis.svgCTM = Matrix.fromDOM(svgElem.getScreenCTM()).invert();
    screenCTM = globalThis.screenCTM = Matrix.fromDOM(svgElem.getScreenCTM());

    svgElem.addEventListener('mousedown', async e => {
      const { clientX: x, clientY: y, target, currentTarget } = e;

      let pos = new Point(...svgCTM.transform_xy(x, y));

      if(target.tagName != 'svg') {
        Object.assign(globalThis, { target });

        let xy = GetPosition(target);
        let pt = FindPoint(xy);

        if(pt) {
          console.log('touchstart', { xy, pt });

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
      } else {
        AddPoint(pos);
        //CreateElement(new Point(x,y));
      }

      console.log('mousedown', { x, y, target, currentTarget });
    });
  });
  let component = h(SVG, { ref, viewBox: new BBox(0, 0, 160, 100) }, [
    h('circle', { cx: 30, cy: 30, r: 2, stroke: 'red', 'stroke-width': 0.1, fill: 'none' }),
    h(Path, { points: anchorPoints }, []),
    h(AnchorPoints, { points: anchorPoints }, []),
    h(EllipticArc, { data: arc }, [])
  ]);

  render(component, element);
});
