import { read, write } from 'xml';
import { WriteFile, WriteFd, WriteClose, WriteAny, WriteJSON, WriteXML } from './io-helpers.js';
import { Rect, Size, Point, Line, Align, BBox } from './lib/geom.js';
import { getUnit, getValue, unitConvToMM, unitConvFactor, unitConvFunction, unitConv } from './measure-unit.js';
import { SvgPath } from './lib/svg/path.js';
import { getOpt } from './lib/misc.js';

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
      'font-weight': 500,
      'font-size': 3,
      'font-family': 'Century Gothic',
      'letter-spacing': 0,
      'word-spacing': 0,
      'text-anchor': 'middle',
      'dominant-baseline': 'middle',
      'stroke-width': 0,
      fill: 'black',
      ...attrs
    },
    [xml('tspan', spanAttrs, [str])]
  );
}

function line(...args) {
  let l = new Line(...args);
  let p = new SvgPath();

  p.to(...l.a);
  p.line(...l.b);

  return xml('path', {
    d: p.str(),
    ...strokeStyle(),
    'stroke-dasharray': '0.52916667,0.52916667',
    'stroke-dashoffset': 0
  });
}

function measure(...args) {
  let l = new Line(...args);
  let p = new SvgPath();

  p.to(0, 0);
  p.line(...l.slope);

  let xy = l.pointAt(0.5).sum(0, 0.5);

  return xml('g', {}, [
    text(
      `${(l.getLength() / 10).toFixed(1).replace('.', ',')}cm`,
      {
        transform: `translate(${xy}) rotate(${l.y1 == l.y2 ? 0 : 90}) translate(0, ${(l.y1 == l.y2 ? -l.slope.normalize().x : 0.8) * 1.5})`,
        'dominant-baseline': l.y1 == l.y2 && l.slope.normalize().x == 1 ? 'text-bottom' : 'hanging'
      },
      {}
    ),
    xml('path', {
      d: p.str(),
      ...strokeStyle(),
      ...measureStyle,
      transform: `translate(${l.a}) scale(${l.y1 == l.y2 ? 1 : -1},1) `
    })
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
  let orientation = 'landscape';
  let arrows = true;
  let params = getOpt(
    {
      landscape: [false, () => (orientation = 'landscape'), 'l'],
      portrait: [false, () => (orientation = 'portrait'), 'p'],
      inner: [false, null, 'i'],
      'no-arrows': [false, () => (arrows = false), 'A'],
      output: [true, null, 'o'],
      'bottom-inside': [false, null, 'b'],
      gap: [true, null, 'g'],
      '@': 'args'
    },
    args
  );
  let { gap = 1, output = 'output.svg', 'bottom-inside': bottomInside = false } = params;
  let [width = 210, height = 45, depth = 36, thickness = 4] = params['@'].map(a => Number(unitConvToMM(a, 'mm')));

  if(params.inner) {
    width += thickness * 2;
    height += thickness * 2;
    depth += thickness * 2;
  }

  console.log('svg-box', { width, height, depth });

  console.log('1px', px(1));

  let dimensions = orientation == 'landscape' ? { width: '297mm', height: '210mm' } : { width: '210mm', height: '297mm' };
  let size = new Size(dimensions.width, dimensions.height);

  const f = 32;
  const g = f - px(1);
  const d = +px(2);
  const e = Math.sqrt(0.5) * 0.5 * f;

  let elements = [];

  const pushrect = (r, mx = -1, my = -1) =>
    elements.push(
      rect(r),
      ...(mx ? [mx < 0 || mx === true ? measure(r.x, r.y - 10, r.x2, r.y - 10) : measure(r.x2, r.y2 + 10, r.x, r.y2 + 10)] : []),
      ...(my ? [my < 0 || my === true ? measure(r.x - 10, r.y, r.x - 10, r.y2) : measure(r.x2 + 10, r.y2, r.x2 + 10, r.y)] : [])
    );

  const pushlines = (...l) => elements.push(...l.map(o => line(o)));

  const pushtext = (t, p) => elements.push(text(t, { transform: `translate(${p})`, 'font-size': 6 }));

  let sz = new Size(...size);
  let r = new Rect(0, 0, width, height);
  let r2, r3;
  // console.log('svg-box', { width, height, depth });
  const bb = new BBox();

  //r.align(sz, Align.CENTER | Align.TOP);
  r.x += 20;
  r.y += 20;

  pushrect(r);
  pushtext(bottomInside ? 'Deckel' : 'Deckel/Boden', r.center);

  let [t, , u] = r.clone().inset(thickness, 0).toLines();

  pushlines(t, u);

  let [, w, , v] = r.clone().inset(thickness, thickness).toLines();

  pushlines(v, w);

  if(bottomInside) {
    let bottom = r.clone().inset(thickness, thickness);
    bottom.x = r.x2 + +gap;
    pushrect(bottom, -1, 1);
    pushtext('Boden', bottom.center);
    /*r=bottom;
  r.y += 20;*/
  }

  frontBack(r.x, r.y2 + 10);

  function frontBack(x, y) {
    let r = new Rect(x, y, width, depth - thickness * (bottomInside ? 1 : 2));

    pushrect(r, false, true);
    pushtext('Vorne', r.center);

    let [, u, , t] = r.clone().inset(0, thickness).toLines();

    pushlines(t, u);

    if(bottomInside) pushlines(r.clone().inset(thickness, thickness).toLines()[2]);

    r2 = r.clone();

    r.y += r.height + +gap;

    pushrect(r, false, true);
    pushtext('Hinten', r.center);

    [, u, , t] = r.clone().inset(0, 4).toLines();

    pushlines(t, u);

    if(bottomInside) pushlines(r.clone().inset(thickness, thickness).toLines()[2]);
    bb.updateXY(r.x, r.y);
    return r;
  }

  leftRight(r2.x2 + +gap, r2.y);

  function leftRight(x, y) {
    let r = new Rect(x, y, height - thickness * 2, depth - thickness * (bottomInside ? 1 : 2));

    pushrect(r, 1, false);

    if(bottomInside) pushlines(r.clone().inset(4, 0).toLines()[2]);

    pushtext('Links', r.center);

    r3 = r.clone();

    r.x += r.width + +gap;

    pushrect(r, 1, 1);

    if(bottomInside) pushlines(r.clone().inset(4, 0).toLines()[2]);
    pushtext('Rechts', r.center);
    bb.updateXY(r.x, r.y);
  }

  const br = new Rect(bb.toRect());
  console.log('br', br + '');
  console.log('br.aspect', br.aspect());

  while(br.height > size.height) size.height *= 2;
  while(br.width > size.width) size.width *= 2;

  /* console.log('bb', bb);
  console.log('size',size.toObject());
  console.log('bb.width', bb.width);
  console.log('bb.height', bb.height);
*/
  const obj = xml(
    'svg',
    {
      xmlns: 'http://www.w3.org/2000/svg',
      version: '1.1',
      viewBox: [0, 0, ...size].join(' '),
      ...size.toObject(true)
    },
    [
      xml('defs', {}, [
        xml(
          'marker',
          { id: 'a', orient: 'auto' },
          xml('path', {
            d: `M 0,${f * -0.25} v ${1.25 * f} M 0,0 h ${-0.25 * f} M ${-e},${-e} l ${e * 2},${e * 2}`,
            //d: `M 0,${f} v ${-2 * f}` + (arrows ? ` M ${d + g},${g} l ${-g},${-g} l ${g},${-g}` : ''),
            ...markerStyle()
          })
        ),
        xml(
          'marker',
          { id: 'b', orient: 'auto' },
          xml('path', {
            d: `M 0,${f * -0.25} v ${1.25 * f} M 0,0 h ${0.25 * f} M ${-e},${-e} l ${e * 2},${e * 2}`,
            // d: `M 0,${f} v ${-2 * f}`+ (arrows ? ` M ${-d - g},${g} l ${g},${-g} l ${-g},${-g}` : ''),
            ...markerStyle()
          })
        )
      ]),
      ...elements
    ]
  );

  WriteFile(output, write(obj));
}

main(...scriptArgs.slice(1));
