import { h, Fragment } from './lib/preact.js';

const SchematicPalette = [
  '#ffffff', '#7f7fff', '#7fff7f', '#7fffff', '#ff7f7f', '#ff7fff', '#ffff7f', '#000000',
  '#4b4bff', '#4bff4b', '#4bffff', '#ff4b4b', '#ff4bff', '#ffff4b', '#4b4b4b', '#a5a5a5', '#000000',
];

const BoardPalette = [
  '#000000', '#0000a5', '#00a500', '#00a5a5', '#a50000', '#a500a5', '#a5a500', '#a5a5a5',
  '#4b4b4b', '#4b4bff', '#4bff4b', '#4bffff', '#ff4b4b', '#ff4bff', '#ffff4b', '#ffffff', '#000000',
];

const num = (el, name, dflt = 0) => {
  const v = el.getAttribute(name);
  return v == null || v === '' ? dflt : +v;
};

const str = (el, name, dflt = '') => el.getAttribute(name) ?? dflt;

const yesNo = (el, name) => el.getAttribute(name) === 'yes';

const findChild = (el, tag) => [...el.children].find(c => c.tagName === tag);

const filterChildren = (el, tag) => [...el.children].filter(c => c.tagName === tag);

function parseRot(rot) {
  if(!rot) return { angle: 0, mirror: false, spin: false };
  const s = rot + '';
  return {
    mirror: /M/.test(s),
    spin: /S/.test(s),
    angle: +s.replace(/[MSR]/g, '') || 0,
  };
}

function transformString(x, y, rot, extraScale = null) {
  const { angle, mirror } = parseRot(rot);
  const parts = [`translate(${x},${y})`];
  if(mirror) parts.push('scale(-1,1)');
  if(angle) parts.push(`rotate(${angle})`);
  if(extraScale) parts.push(`scale(${extraScale.x},${extraScale.y})`);
  return parts.join(' ');
}

function layerByName(doc, name) {
  return doc.querySelector(`layer[name="${name}"]`);
}

function layerByRef(doc, ref) {
  if(ref == null || ref === '') return null;
  const layers = doc.eagle?.drawing?.layers;
  if(!layers) return null;
  return [...layers.children].find(l => l.getAttribute('number') === ref + '' || l.getAttribute('name') === ref);
}

function paletteFor(type) {
  return type === 'board' ? BoardPalette : SchematicPalette;
}

function layerStroke(doc, layer, palette) {
  if(!layer) return '#a5a5a5';
  const color = +layer.getAttribute('color');
  return palette[color] || palette[15] || '#a5a5a5';
}

class BBox {
  constructor() {
    this.x1 = Infinity;
    this.y1 = Infinity;
    this.x2 = -Infinity;
    this.y2 = -Infinity;
  }
  add(x, y) {
    if(x < this.x1) this.x1 = x;
    if(y < this.y1) this.y1 = y;
    if(x > this.x2) this.x2 = x;
    if(y > this.y2) this.y2 = y;
  }
  update(other) {
    if(!other) return;
    if(other.x1 < this.x1) this.x1 = other.x1;
    if(other.y1 < this.y1) this.y1 = other.y1;
    if(other.x2 > this.x2) this.x2 = other.x2;
    if(other.y2 > this.y2) this.y2 = other.y2;
  }
  outset(d) {
    this.x1 -= d; this.y1 -= d;
    this.x2 += d; this.y2 += d;
  }
  get valid() {
    return isFinite(this.x1) && isFinite(this.y1) && isFinite(this.x2) && isFinite(this.y2);
  }
  get width() { return this.x2 - this.x1; }
  get height() { return this.y2 - this.y1; }
}

function bboxOfPrimitive(el, into) {
  const t = el.tagName;
  switch(t) {
    case 'wire':
    case 'rectangle':
    case 'dimension':
    case 'frame':
      into.add(num(el, 'x1'), -num(el, 'y1'));
      into.add(num(el, 'x2'), -num(el, 'y2'));
      break;
    case 'circle': {
      const cx = num(el, 'x'), cy = -num(el, 'y'), r = num(el, 'radius');
      into.add(cx - r, cy - r);
      into.add(cx + r, cy + r);
      break;
    }
    case 'pad':
    case 'via':
    case 'hole':
    case 'text':
    case 'junction':
    case 'pin':
    case 'label':
    case 'attribute':
    case 'vertex':
      into.add(num(el, 'x'), -num(el, 'y'));
      break;
    case 'smd': {
      const x = num(el, 'x'), y = -num(el, 'y');
      const dx = num(el, 'dx'), dy = num(el, 'dy');
      into.add(x - dx / 2, y - dy / 2);
      into.add(x + dx / 2, y + dy / 2);
      break;
    }
    case 'polygon':
      for(const v of filterChildren(el, 'vertex'))
        into.add(num(v, 'x'), -num(v, 'y'));
      break;
  }
}

function bboxOfContainer(el, into = new BBox()) {
  if(!el) return into;
  for(const c of el.children) bboxOfPrimitive(c, into);
  return into;
}

function measuresBBox(doc, plain) {
  const bb = new BBox();
  const dimLayer = layerByName(doc, 'Dimension');
  const measLayer = layerByName(doc, 'Measures');
  const wanted = new Set([dimLayer?.getAttribute('number'), measLayer?.getAttribute('number')].filter(Boolean));
  let seen = 0;
  for(const c of plain.children) {
    if(c.tagName !== 'wire') continue;
    if(!wanted.has(c.getAttribute('layer'))) continue;
    bboxOfPrimitive(c, bb);
    seen++;
  }
  if(!seen) return null;
  return bb;
}

function renderWire(w, doc, palette, opts = {}) {
  const layer = layerByRef(doc, w.getAttribute('layer'));
  const stroke = opts.color ?? layerStroke(doc, layer, palette);
  const width = num(w, 'width', 0.1) || 0.1;
  const curve = num(w, 'curve');
  const x1 = num(w, 'x1'), y1 = -num(w, 'y1');
  const x2 = num(w, 'x2'), y2 = -num(w, 'y2');
  if(curve) {
    const dx = x2 - x1, dy = y2 - y1;
    const chord = Math.hypot(dx, dy);
    const theta = curve * Math.PI / 180;
    const r = Math.abs(chord / (2 * Math.sin(theta / 2)));
    const large = Math.abs(curve) > 180 ? 1 : 0;
    const sweep = curve > 0 ? 0 : 1;
    return h('path', {
      class: 'wire', d: `M ${x1} ${y1} A ${r} ${r} 0 ${large} ${sweep} ${x2} ${y2}`,
      fill: 'none', stroke, 'stroke-width': width, 'stroke-linecap': 'round',
    });
  }
  return h('line', {
    class: 'wire', x1, y1, x2, y2, stroke, 'stroke-width': width, 'stroke-linecap': 'round',
  });
}

function renderRectangle(r, doc, palette) {
  const layer = layerByRef(doc, r.getAttribute('layer'));
  const fill = layerStroke(doc, layer, palette);
  const x1 = num(r, 'x1'), y1 = -num(r, 'y1');
  const x2 = num(r, 'x2'), y2 = -num(r, 'y2');
  const x = Math.min(x1, x2), y = Math.min(y1, y2);
  return h('rect', {
    class: 'rectangle', x, y,
    width: Math.abs(x2 - x1), height: Math.abs(y2 - y1),
    fill, stroke: 'none',
  });
}

function renderCircle(c, doc, palette) {
  const layer = layerByRef(doc, c.getAttribute('layer'));
  const stroke = layerStroke(doc, layer, palette);
  return h('circle', {
    class: 'circle', cx: num(c, 'x'), cy: -num(c, 'y'),
    r: num(c, 'radius'), fill: 'none', stroke, 'stroke-width': num(c, 'width', 0.1) || 0.1,
  });
}

function renderPolygon(p, doc, palette) {
  const layer = layerByRef(doc, p.getAttribute('layer'));
  const fill = layerStroke(doc, layer, palette);
  const verts = filterChildren(p, 'vertex');
  if(!verts.length) return null;
  const d = verts.map((v, i) => `${i ? 'L' : 'M'} ${num(v, 'x')} ${-num(v, 'y')}`).join(' ') + ' Z';
  return h('path', {
    class: 'polygon', d, fill, stroke: fill,
    'stroke-width': num(p, 'width', 0.05) || 0.05, 'fill-opacity': 0.3,
  });
}

function renderText(t, doc, palette, overrideText) {
  const layer = layerByRef(doc, t.getAttribute('layer'));
  const fill = layerStroke(doc, layer, palette);
  const size = num(t, 'size', 1) || 1;
  const rot = t.getAttribute('rot') || '';
  const align = str(t, 'align', 'bottom-left');
  const { angle, mirror } = parseRot(rot);
  const x = num(t, 'x'), y = -num(t, 'y');
  const anchor = { 'bottom-left': 'start', 'bottom-center': 'middle', 'bottom-right': 'end',
    'center-left': 'start', 'center': 'middle', 'center-right': 'end',
    'top-left': 'start', 'top-center': 'middle', 'top-right': 'end' }[align] || 'start';
  const baseline = { 'bottom-left': 'baseline', 'bottom-center': 'baseline', 'bottom-right': 'baseline',
    'center-left': 'central', 'center': 'central', 'center-right': 'central',
    'top-left': 'hanging', 'top-center': 'hanging', 'top-right': 'hanging' }[align] || 'baseline';
  const tx = [`translate(${x},${y})`];
  if(mirror) tx.push('scale(-1,1)');
  if(angle) tx.push(`rotate(${-angle})`);
  tx.push('scale(1,-1)');
  const text = overrideText ?? (t.textContent || '');
  return h('text', {
    class: 'text', transform: tx.join(' '), fill, stroke: 'none',
    'font-size': size, 'font-family': 'monospace',
    'text-anchor': anchor, 'dominant-baseline': baseline,
  }, text);
}

function renderPin(pin, doc, palette) {
  const x = num(pin, 'x'), y = -num(pin, 'y');
  const rot = pin.getAttribute('rot') || 'R0';
  const { angle } = parseRot(rot);
  const lengthName = str(pin, 'length', 'long');
  const lenMap = { long: 3, middle: 2, short: 1, point: 0 };
  const veclen = (lenMap[lengthName] ?? 3) * 2.54;
  const rad = angle * Math.PI / 180;
  const dx = Math.cos(rad) * veclen;
  const dy = -Math.sin(rad) * veclen;
  const name = str(pin, 'name');
  const visible = str(pin, 'visible', 'both');
  const showName = visible === 'both' || visible === 'pin';
  return h('g', { class: 'pin' }, [
    h('line', {
      x1: x, y1: y, x2: x + dx, y2: y + dy,
      stroke: '#a54b4b', 'stroke-width': 0.15, 'stroke-linecap': 'round',
    }),
    showName && name && h('text', {
      x: x + dx * 1.2, y: y + dy * 1.2,
      transform: `scale(1,-1) translate(0, ${-2 * (y + dy * 1.2)})`,
      fill: '#a5a54b', stroke: 'none', 'font-size': 0.6, 'font-family': 'monospace',
      'text-anchor': 'middle', 'dominant-baseline': 'central',
    }, name),
  ]);
}

function renderJunction(j) {
  return h('circle', {
    class: 'junction', cx: num(j, 'x'), cy: -num(j, 'y'),
    r: 0.5, fill: '#4ba54b', stroke: 'none',
  });
}

function renderPad(p, doc, palette) {
  const name = str(p, 'name');
  const x = num(p, 'x'), y = -num(p, 'y');
  const drill = num(p, 'drill', 0.6);
  const diameter = num(p, 'diameter') || drill * 1.6;
  const shape = str(p, 'shape', 'round');
  const ro = diameter / 2;
  let d;
  switch(shape) {
    case 'square':
      d = `M ${-ro} ${-ro} L ${ro} ${-ro} L ${ro} ${ro} L ${-ro} ${ro} Z`;
      break;
    case 'octagon': {
      const pts = [];
      for(let i = 0; i < 8; i++) {
        const a = (Math.PI * i) / 4 + Math.PI / 8;
        pts.push([Math.cos(a) * ro * 1.08, Math.sin(a) * ro * 1.08]);
      }
      d = pts.map(([px, py], i) => `${i ? 'L' : 'M'} ${px} ${py}`).join(' ') + ' Z';
      break;
    }
    case 'long': {
      const w = ro;
      d = `M ${-w} ${-ro} L ${w} ${-ro} A ${ro} ${ro} 0 0 1 ${w} ${ro} L ${-w} ${ro} A ${ro} ${ro} 0 0 1 ${-w} ${-ro} Z`;
      break;
    }
    default:
      d = `M 0 ${-ro} A ${ro} ${ro} 0 0 1 0 ${ro} A ${ro} ${ro} 0 0 1 0 ${-ro} Z`;
  }
  const ri = drill / 2;
  return h('g', { class: 'pad', transform: `translate(${x},${y})`, 'data-name': name }, [
    h('path', { d, fill: '#4ba54b', stroke: 'none' }),
    h('circle', { cx: 0, cy: 0, r: ri, fill: '#000', stroke: 'none' }),
  ]);
}

function renderVia(v, doc, palette) {
  const x = num(v, 'x'), y = -num(v, 'y');
  const drill = num(v, 'drill', 0.4);
  const ro = drill * 0.8;
  return h('g', { class: 'via', transform: `translate(${x},${y})` }, [
    h('circle', { cx: 0, cy: 0, r: ro, fill: '#7f7f7f' }),
    h('circle', { cx: 0, cy: 0, r: drill / 2, fill: '#000' }),
  ]);
}

function renderSmd(s, doc, palette) {
  const layer = layerByRef(doc, s.getAttribute('layer'));
  const fill = layerStroke(doc, layer, palette);
  const x = num(s, 'x'), y = -num(s, 'y');
  const dx = num(s, 'dx'), dy = num(s, 'dy');
  return h('rect', {
    class: 'smd', x: x - dx / 2, y: y - dy / 2,
    width: dx, height: dy, fill, stroke: 'none',
  });
}

function renderHole(h_) {
  const x = num(h_, 'x'), y = -num(h_, 'y');
  const drill = num(h_, 'drill', 1);
  return h('circle', { class: 'hole', cx: x, cy: y, r: drill / 2, fill: '#000' });
}

function renderPrimitive(el, doc, palette, opts = {}) {
  switch(el.tagName) {
    case 'wire': return renderWire(el, doc, palette, opts);
    case 'rectangle': return renderRectangle(el, doc, palette);
    case 'circle': return renderCircle(el, doc, palette);
    case 'polygon': return renderPolygon(el, doc, palette);
    case 'text': return renderText(el, doc, palette);
    case 'pin': return renderPin(el, doc, palette);
    case 'junction': return renderJunction(el);
    case 'pad': return renderPad(el, doc, palette);
    case 'via': return renderVia(el, doc, palette);
    case 'smd': return renderSmd(el, doc, palette);
    case 'hole': return renderHole(el);
    case 'dimension': return renderWire(el, doc, palette, opts);
    case 'frame': return renderRectangle(el, doc, palette);
    case 'label': return renderText(el, doc, palette, opts.labelText);
    case 'attribute': return renderText(el, doc, palette, opts.attrText);
    default: return null;
  }
}

function renderCollection(el, doc, palette, opts = {}) {
  if(!el) return [];
  return [...el.children].map(c => renderPrimitive(c, doc, palette, opts)).filter(Boolean);
}

function renderSymbol(symbol, doc, palette) {
  return h('g', { class: 'symbol', 'data-name': symbol.getAttribute('name') },
    renderCollection(symbol, doc, palette),
  );
}

function renderPackage(pkg, doc, palette) {
  return h('g', { class: 'package', 'data-name': pkg.getAttribute('name') },
    renderCollection(pkg, doc, palette),
  );
}

function q(root, selector) {
  try { return root.querySelector(selector); } catch { return null; }
}

function findLibrary(doc, name) {
  return q(doc, `schematic > libraries > library[name="${name}"]`) ||
    q(doc, `board > libraries > library[name="${name}"]`);
}

function findSymbol(doc, libName, symName) {
  const lib = findLibrary(doc, libName);
  return lib && q(lib, `symbols > symbol[name="${symName}"]`);
}

function findPackage(doc, libName, pkgName) {
  const lib = findLibrary(doc, libName);
  return lib && q(lib, `packages > package[name="${pkgName}"]`);
}

function findPart(doc, partName) {
  return q(doc, `schematic > parts > part[name="${partName}"]`);
}

function findGateSymbol(doc, part, gateName) {
  if(!part) return null;
  const libName = part.getAttribute('library');
  const dsName = part.getAttribute('deviceset');
  const lib = findLibrary(doc, libName);
  if(!lib) return null;
  const gate = q(lib, `devicesets > deviceset[name="${dsName}"] > gates > gate[name="${gateName}"]`);
  if(!gate) return null;
  return findSymbol(doc, libName, gate.getAttribute('symbol'));
}

function renderInstance(inst, doc, palette) {
  const x = num(inst, 'x'), y = -num(inst, 'y');
  const rot = inst.getAttribute('rot') || 'R0';
  const partName = inst.getAttribute('part');
  const gateName = inst.getAttribute('gate');
  const part = findPart(doc, partName);
  const symbol = findGateSymbol(doc, part, gateName);
  if(!symbol) return null;
  const { angle, mirror } = parseRot(rot);
  const parts = [`translate(${x},${y})`];
  if(mirror) parts.push('scale(-1,1)');
  if(angle) parts.push(`rotate(${-angle})`);
  return h('g', {
    class: 'instance', transform: parts.join(' '),
    'data-part': partName, 'data-gate': gateName,
  }, renderCollection(symbol, doc, palette));
}

function renderElement(elt, doc, palette) {
  const x = num(elt, 'x'), y = -num(elt, 'y');
  const rot = elt.getAttribute('rot') || 'R0';
  const libName = elt.getAttribute('library');
  const pkgName = elt.getAttribute('package');
  const pkg = findPackage(doc, libName, pkgName);
  if(!pkg) return null;
  const { angle, mirror } = parseRot(rot);
  const parts = [`translate(${x},${y})`];
  if(mirror) parts.push('scale(-1,1)');
  if(angle) parts.push(`rotate(${-angle})`);
  return h('g', {
    class: 'element', transform: parts.join(' '),
    'data-name': elt.getAttribute('name'), 'data-library': libName, 'data-package': pkgName,
  }, renderCollection(pkg, doc, palette));
}

function renderNet(net, doc, palette) {
  const name = net.getAttribute('name');
  const segments = filterChildren(net, 'segment');
  const children = [];
  for(const seg of segments) {
    for(const c of seg.children) {
      const opts = c.tagName === 'label' ? { labelText: name } : {};
      const el = renderPrimitive(c, doc, palette, opts);
      if(el) children.push(el);
    }
  }
  return h('g', { class: `net`, 'data-name': name }, children);
}

function renderSignal(sig, doc, palette) {
  const name = sig.getAttribute('name');
  const children = renderCollection(sig, doc, palette);
  return h('g', { class: 'signal', 'data-name': name }, children);
}

function renderSheet(sheet, doc, palette) {
  const plain = findChild(sheet, 'plain');
  const instances = findChild(sheet, 'instances');
  const nets = findChild(sheet, 'nets');
  const children = [];
  if(plain) children.push(h('g', { class: 'plain' }, renderCollection(plain, doc, palette)));
  if(nets) children.push(h('g', { class: 'nets' }, filterChildren(nets, 'net').map(n => renderNet(n, doc, palette))));
  if(instances) children.push(h('g', { class: 'instances' }, filterChildren(instances, 'instance').map(i => renderInstance(i, doc, palette))));
  return h('g', { class: 'sheet' }, children);
}

function computeBounds(doc) {
  const bb = new BBox();
  const drawing = doc.eagle?.drawing;
  if(!drawing) return bb;
  if(doc.type === 'schematic') {
    const sheet = findChild(drawing.schematic, 'sheets')?.children[0];
    if(sheet) {
      bboxOfContainer(findChild(sheet, 'plain'), bb);
      for(const inst of filterChildren(findChild(sheet, 'instances') || sheet, 'instance'))
        bb.add(num(inst, 'x'), -num(inst, 'y'));
      for(const net of filterChildren(findChild(sheet, 'nets') || sheet, 'net'))
        for(const seg of filterChildren(net, 'segment'))
          bboxOfContainer(seg, bb);
    }
    const plain = drawing.schematic && findChild(drawing.schematic, 'plain');
    if(plain) bboxOfContainer(plain, bb);
  } else if(doc.type === 'board') {
    const board = drawing.board;
    const plain = findChild(board, 'plain');
    if(plain) {
      const meas = measuresBBox(doc, plain);
      if(meas) bb.update(meas);
      else bboxOfContainer(plain, bb);
    }
    const elts = findChild(board, 'elements');
    if(elts) for(const e of elts.children)
      if(e.tagName === 'element') bb.add(num(e, 'x'), -num(e, 'y'));
  } else if(doc.type === 'library') {
    const lib = drawing.library;
    const symbols = findChild(lib, 'symbols');
    const packages = findChild(lib, 'packages');
    if(symbols) for(const s of symbols.children) bboxOfContainer(s, bb);
    if(packages) for(const p of packages.children) bboxOfContainer(p, bb);
  }
  if(bb.valid) bb.outset(2);
  return bb;
}

function svgWrapper(bounds, children) {
  const width = bounds.valid ? bounds.width : 100;
  const height = bounds.valid ? bounds.height : 100;
  const x = bounds.valid ? bounds.x1 : 0;
  const y = bounds.valid ? bounds.y1 : 0;
  return h('svg', {
    xmlns: 'http://www.w3.org/2000/svg',
    viewBox: `${x} ${y} ${width} ${height}`,
    width: `${width}mm`, height: `${height}mm`,
    'font-family': 'monospace', 'font-size': '0.7',
    'stroke-linejoin': 'round', 'stroke-linecap': 'round',
  }, children);
}

class SchematicRenderer {
  constructor(doc) { this.doc = doc; this.palette = SchematicPalette; }
  render() {
    const doc = this.doc;
    const sheets = filterChildren(findChild(doc.eagle.drawing.schematic, 'sheets'), 'sheet');
    const sheet = sheets[0];
    const bounds = computeBounds(doc);
    const children = [];
    if(sheet) children.push(renderSheet(sheet, doc, this.palette));
    return svgWrapper(bounds, children);
  }
}

class BoardRenderer {
  constructor(doc) { this.doc = doc; this.palette = BoardPalette; }
  render() {
    const doc = this.doc;
    const board = doc.eagle.drawing.board;
    const plain = findChild(board, 'plain');
    const elementsC = findChild(board, 'elements');
    const signalsC = findChild(board, 'signals');
    const bounds = computeBounds(doc);
    const children = [];
    if(plain) children.push(h('g', { class: 'plain' }, renderCollection(plain, doc, this.palette)));
    if(signalsC) children.push(h('g', { class: 'signals' }, filterChildren(signalsC, 'signal').map(s => renderSignal(s, doc, this.palette))));
    if(elementsC) children.push(h('g', { class: 'elements' }, filterChildren(elementsC, 'element').map(e => renderElement(e, doc, this.palette))));
    return svgWrapper(bounds, children);
  }
}

class LibraryRenderer {
  constructor(doc) { this.doc = doc; this.palette = SchematicPalette; }
  render() {
    const doc = this.doc;
    const lib = doc.eagle.drawing.library;
    const symbols = findChild(lib, 'symbols');
    const packages = findChild(lib, 'packages');
    const bounds = computeBounds(doc);
    const children = [];
    if(symbols) children.push(h('g', { class: 'symbols' }, filterChildren(symbols, 'symbol').map(s => renderSymbol(s, doc, this.palette))));
    if(packages) children.push(h('g', { class: 'packages' }, filterChildren(packages, 'package').map(p => renderPackage(p, doc, BoardPalette))));
    return svgWrapper(bounds, children);
  }
}

export function Renderer(doc) {
  switch(doc.type) {
    case 'schematic': return new SchematicRenderer(doc);
    case 'board': return new BoardRenderer(doc);
    case 'library': return new LibraryRenderer(doc);
    default: throw new Error(`Unsupported document type: ${doc.type}`);
  }
}

export { SchematicRenderer, BoardRenderer, LibraryRenderer };
