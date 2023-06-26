import { read, write } from 'xml';
import { WriteFile, WriteFd, WriteClose, WriteAny, WriteJSON, WriteXML } from './io-helpers.js';
import { Rect, Size, Point, Line, Align } from './lib/geom.js';
import { getUnit, getValue, unitConvToMM, unitConvFactor, unitConvFunction, unitConv } from './measure-unit.js';
import { SvgPath } from './lib/svg/path.js';

const px = n => unitConvToMM(`${n}px`, 'mm').toFixed(6);
const pt = (n, to = 'mm') => unitConvToMM(`${n}pt`, 'mm').toFixed(6);

const strokeStyle = (width = '0.5px', unit = '') => ({
  stroke: 'black',
  'stroke-width': unitConv(unit || 'mm')(width) + unit,
  fill: 'none'
});

const markerStyle = () => ({
  stroke: 'black',
  fill: 'none'
});

const measureStyle = {
  'marker-start': 'url(#a)',
  'marker-end': 'url(#b)'
};

function toChildArray(a) {
  if(!a) a = [];
  else if(!Array.isArray(a)) a = [a];
  return a;
}

function xml(tagName, attributes = {}, c = undefined) {
  return { tagName, attributes, children: toChildArray(c) };
}

function text(str, attrs = {}, spanAttrs = {}) {
  return xml(
    'text',
    {
      ...attrs,
      'font-weight': 500,
      'font-size': '0.5mm',
      //    'font-family': 'Fixed',
      'font-family': 'Century Gothic',
      'letter-spacing': 0,
      'word-spacing': 0,
      'text-anchor': 'middle',
      'dominant-baseline': 'middle',
      'stroke-width': 0,
      fill: 'black'
    },
    [xml('tspan', spanAttrs, [str])]
  );
}

/*function line(...args) {
  let l = new Line(...args);
  let p = new SvgPath();

  p.to(...l.a);
  p.line(...l.b);

  return xml('path', { d: p.str(), ...strokeStyle(), ...measureStyle });
}*/

function measure(...args) {
  let l = new Line(...args);
  let p = new SvgPath();

  p.to(...l.a);
  p.line(...l.b);

  let xy = l.pointAt(0.5).sum(0, 0.5);

  return xml('g', {}, [
    text(
      `${l.getLength()}mm`,
      {
        transform: `translate(${xy}) rotate(${l.y1 == l.y2 ? 0 : 90}) translate(0, ${
          l.y1 == l.y2 ? 1 : 1.5
        }) `
      },
      {}
    ),
    xml('path', { d: p.str(), ...strokeStyle(), ...measureStyle })
  ]);
}

function rect(...args) {
  let r = new Rect(...args);
  let p = new SvgPath();

  let pts = r.toPoints();

  p.to(...pts.shift());
  p.line(...pts.shift());
  p.line(...pts.shift());
  p.line(...pts.shift());
  p.close();

  return xml('path', { d: p.str(), ...strokeStyle() });
}

function main(...args) {
  let [width = 100, height = 45, depth = 36, thickness = 4] = args.map(a =>
    Number(unitConvToMM(a, 'mm'))
  );

  console.log('svg-box', { width, height, depth });

  console.log('1px', px(1));
  let dimensions = { width: '210mm', height: '297mm' };
  let size = new Size(dimensions.width, dimensions.height);
  const f = 8;
  const g = f - px(1);
  const d = +px(2);
  let obj = xml(
    'svg',
    {
      xmlns: 'http://www.w3.org/2000/svg',
      version: '1.1',
      viewBox: [0, 0, ...size].join(' '),
      ...dimensions
    },
    xml('defs', {}, [
      xml(
        'marker',
        { id: 'a', orient: 'auto' },
        xml('path', {
          d: `M 0,${f} v ${-2 * f} M ${d + g},${g} l ${-g},${-g} l ${g},${-g}`,
          ...markerStyle()
        })
      ),
      xml(
        'marker',
        { id: 'b', orient: 'auto' },
        xml('path', {
          d: `M 0,${f} v ${-2 * f} M ${-d - g},${g} l ${g},${-g} l ${-g},${-g}`,
          ...markerStyle()
        })
      )
    ])
  );

  const pushrect = (r, mx = true, my = true) =>
    obj.children.push(
      rect(r),
      ...(mx ? [measure(r.x, r.y - 10, r.x2, r.y - 10)] : []),
      ...(my ? [measure(r.x - 10, r.y, r.x - 10, r.y2)] : [])
    );

  let sz = new Size(210, 297);
  let r = new Rect(0, 0, width, height);
  // console.log('svg-box', { width, height, depth });

  r.align(sz, Align.CENTER | Align.TOP);
  r.y += 20;

  pushrect(r);

  let r2 = new Rect(r.x, r.y2 + 10, width, depth - thickness * 2);

  pushrect(r2, false, true);

  let r3 = new Rect(r2.x, r2.y2 + 20, height - thickness * 2, depth - thickness * 2);

  pushrect(r3);

  r3.x += r3.width;

  pushrect(r3, true, false);

  WriteFile('output.svg', write(obj));
}

main(...scriptArgs.slice(1));
