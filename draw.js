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

Object.assign(globalThis, {
  Matrix,
  Point,
  Element,
  CreateElement,
  GetPosition,
  FindPoint,
  TouchEvents
});

let ref = (globalThis.ref = trkl(null));
let svgElem;
let screenCTM, svgCTM;
let anchorPoints = (globalThis.anchorPoints = trkl([]));

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
    h(AnchorPoints, { points: anchorPoints }, [])
  ]);

  render(component, element);
});
