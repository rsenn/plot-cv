/**
 * eagle-renderer.js
 *
 * SVG renderer for EAGLE documents, built on the proven preact components
 * in lib/eagle/components.js.
 *
 * Consumes the DOM-based EagleDocument from ./eagle.js; an adapter (below)
 * presents DOM elements to the components with the EagleElement-style API
 * they expect (numeric attributes, layer objects with palette colors,
 * getColor()/getLayer(), part/symbol resolution, ...).
 *
 * render() returns preact vdom; serialize with lib/preact-render-to-string.js.
 */
import { h } from './lib/preact.js';
import { Rect, TransformationList } from './lib/geom.js';
import { RGBA } from './lib/color/rgba.js';
import { Palette } from './lib/eagle/common.js';
import { Background, Element, ElementToComponent, Grid, Instance, Pattern, Package, SchematicSymbol, Signals, SVG, Text, } from './lib/eagle/components.js';

const num = (el, name, dflt = 0) => {
  const v = el.getAttribute(name);
  return v == null || v === '' ? dflt : +v;
};

const str = (el, name, dflt = '') => el.getAttribute(name) ?? dflt;

const findChild = (el, tag) => [...el.children].find(c => c.tagName === tag);

const filterChildren = (el, tag) =>
  [...el.children].filter(c => c.tagName === tag);

function* descendants(el) {
  for(const c of el.children) {
    if(c.nodeType != null && c.nodeType !== 1) continue;
    yield c;
    yield* descendants(c);
  }
}

/* ------------------------------------------------------------------ *
 * Adapter: DOM element -> the API surface used by lib/eagle/components
 * ------------------------------------------------------------------ */

const NUMERIC_ATTRS = new Set([
  'x',
  'y',
  'x1',
  'y1',
  'x2',
  'y2',
  'x3',
  'y3',
  'width',
  'size',
  'drill',
  'diameter',
  'dx',
  'dy',
  'radius',
  'ratio',
  'textsize',
  'columns',
  'rows',
]);

/* async-iterable that never yields; keeps the components' useValue() harmless */
const silentRepeater = {
  [Symbol.asyncIterator]() {
    return { next: () => new Promise(() => {}) };
  },
};

class EagleAdapter {
  constructor(doc, palette) {
    this.doc = doc;
    this.palette =
      palette ??
      Palette[doc.type == 'board' ? 'board' : 'schematic'](
        (r, g, b) => new RGBA(r, g, b),
      );

    const eagle = doc.querySelector('eagle');
    this.drawing = findChild(eagle, 'drawing');
    this.layersEl = findChild(this.drawing, 'layers');

    this.cache = new WeakMap();
    this.layerCache = new Map();

    this.docFacade = {
      type: doc.type,
      getLayer: ref => this.getLayer(ref),
      layers: new Proxy(
        {},
        {
          get: (_, prop) =>
            typeof prop == 'string' ? this.getLayer(prop) : undefined,
        },
      ),
    };
  }

  getLayer(ref) {
    if(ref == null || ref === '') return undefined;
    const key = ref + '';

    let layer = this.layerCache.get(key);
    if(layer) return layer;

    const el = [...this.layersEl.children].find(
      l => l.getAttribute('number') == key || l.getAttribute('name') == key,
    );

    layer = {
      number: el ? +el.getAttribute('number') : +key || 0,
      name: el ? el.getAttribute('name') : key,
      color:
        (el && this.palette[+el.getAttribute('color')]) ?? this.palette[16],
      handlers: { visible: (el && el.getAttribute('visible')) ?? 'yes' },
    };

    this.layerCache.set(key, layer);
    return layer;
  }

  findLibrary(name) {
    for(const section of ['board', 'schematic', 'library']) {
      const s = findChild(this.drawing, section);
      const libs = s && findChild(s, 'libraries');
      const lib =
        libs && [...libs.children].find(l => l.getAttribute('name') == name);
      if(lib) return lib;
    }
  }

  layerOf(el) {
    switch (el.tagName) {
      case 'pad':
        return this.getLayer('Pads');
      case 'via':
        return this.getLayer('Vias');
      case 'hole':
        return this.getLayer('Holes');
    }
    return this.getLayer(el.getAttribute('layer'));
  }

  attributesOf(el) {
    const obj = {};
    for(const name of el.getAttributeNames())
      obj[name] = el.getAttribute(name);
    return obj;
  }

  wrap(el) {
    if(!el || typeof el != 'object') return el;

    let w = this.cache.get(el);
    if(w) return w;

    const adapter = this;

    w = new Proxy(el, {
      get(target, prop) {
        switch (prop) {
          case 'tagName':
            return target.tagName;
          case 'raw':
            return target;
          case 'attributes':
          case 'handlers':
            return adapter.attributesOf(target);
          case 'children':
            return [...target.children].map(c => adapter.wrap(c));
          case 'text':
            return target.textContent ?? '';
          case 'document':
            return adapter.docFacade;
          case 'path':
            return [];
          case 'parentNode':
            return target.parentNode ? adapter.wrap(target.parentNode) : null;
          case 'repeater':
            return silentRepeater;
          case 'layer':
            return adapter.layerOf(target);
          case 'getLayer':
            return () => adapter.layerOf(target);
          case 'getColor':
            return () => adapter.layerOf(target)?.color ?? adapter.palette[16];
          case 'get':
            return pred => {
              for(const d of descendants(target)) {
                const f = adapter.wrap(d);
                if(pred(f)) return f;
              }
            };
          case 'getAll':
            return pred => {
              const out = [];
              for(const d of descendants(target)) {
                const f = adapter.wrap(d);
                if(pred(f)) out.push(f);
              }
              return out;
            };
          case 'radius':
            if(target.tagName == 'pad') {
              const drill = num(target, 'drill', 0.6);
              return (num(target, 'diameter') || drill * 1.6) / 2;
            }
            break;
          case 'diameter':
            if(target.tagName == 'via' && !target.hasAttribute('diameter'))
              return 'auto';
            break;
          case 'part':
          case 'gate':
            if(target.tagName == 'instance' || target.tagName == 'pinref')
              return adapter.wrap(target[prop]);
            break;
          case 'symbol':
            if(target.tagName == 'instance')
              return adapter.wrap(target.gate?.symbol);
            if(target.tagName == 'gate') return adapter.wrap(target.symbol);
            break;
          case 'deviceset':
            if(target.tagName == 'part') return adapter.wrap(target.deviceset);
            break;
          case 'library':
            if(target.tagName == 'element' || target.tagName == 'part')
              return adapter.wrap(
                adapter.findLibrary(target.getAttribute('library')),
              );
            break;
        }

        if(typeof prop == 'string' && target.hasAttribute(prop)) {
          const v = target.getAttribute(prop);
          return NUMERIC_ATTRS.has(prop) ? +v : v;
        }

        return undefined;
      },
    });

    this.cache.set(el, w);
    return w;
  }
}

/* ------------------------------------------------------------------ *
 * Bounds computation (on the raw DOM document, SVG coordinates: y down)
 * ------------------------------------------------------------------ */

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
    this.x1 -= d;
    this.y1 -= d;
    this.x2 += d;
    this.y2 += d;
  }
  get valid() {
    return (
      isFinite(this.x1) &&
      isFinite(this.y1) &&
      isFinite(this.x2) &&
      isFinite(this.y2)
    );
  }
  get width() {
    return this.x2 - this.x1;
  }
  get height() {
    return this.y2 - this.y1;
  }
}

function bboxOfPrimitive(el, into) {
  switch (el.tagName) {
    case 'wire':
    case 'rectangle':
    case 'dimension':
    case 'frame':
      into.add(num(el, 'x1'), -num(el, 'y1'));
      into.add(num(el, 'x2'), -num(el, 'y2'));
      break;
    case 'circle': {
      const cx = num(el, 'x'),
        cy = -num(el, 'y'),
        r = num(el, 'radius');
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
      const x = num(el, 'x'),
        y = -num(el, 'y');
      const dx = num(el, 'dx'),
        dy = num(el, 'dy');
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

function measuresBBox(adapter, plain) {
  const bb = new BBox();
  const wanted = new Set(
    ['Dimension', 'Measures']
      .map(name => adapter.getLayer(name))
      .filter(Boolean)
      .map(l => l.number + ''),
  );
  let seen = 0;
  for(const c of plain.children) {
    if(c.tagName !== 'wire') continue;
    if(!wanted.has(c.getAttribute('layer'))) continue;
    bboxOfPrimitive(c, bb);
    seen++;
  }
  return seen ? bb : null;
}

/* ------------------------------------------------------------------ *
 * Renderers
 * ------------------------------------------------------------------ */

export class EagleSVGRenderer {
  constructor(doc) {
    this.doc = doc;
    this.adapter = new EagleAdapter(doc);
    this.transform = new TransformationList().scale(1, -1);
  }

  renderItem(el, opts = {}) {
    switch (el.tagName) {
      case 'junction':
        return h('circle', {
          class: 'junction',
          fill: '#4ba54b',
          stroke: 'none',
          cx: num(el, 'x'),
          cy: num(el, 'y'),
          r: 0.5,
        });
      case 'label':
        return this.renderLabel(el, opts);
      case 'frame': {
        const layer = this.adapter.layerOf(el);
        const x1 = num(el, 'x1'),
          y1 = num(el, 'y1');
        const x2 = num(el, 'x2'),
          y2 = num(el, 'y2');
        return h('rect', {
          class: 'frame',
          x: Math.min(x1, x2),
          y: Math.min(y1, y2),
          width: Math.abs(x2 - x1),
          height: Math.abs(y2 - y1),
          fill: 'none',
          stroke: layer?.color,
          'stroke-width': 0.15,
        });
      }
      case 'pinref':
      case 'contactref':
      case 'description':
        return null;
    }

    const comp = ElementToComponent(el);
    if(!comp) return null;

    return h(comp, {
      data: this.adapter.wrap(el),
      opts: { transformation: this.transform, ...opts },
    });
  }

  renderLabel(el, { labelText = '' } = {}) {
    const layer = this.adapter.layerOf(el);

    return h(Text, {
      className: 'label',
      x: num(el, 'x'),
      y: num(el, 'y'),
      rot: str(el, 'rot', ''),
      alignment: str(el, 'align', 'bottom-left'),
      text: labelText,
      color: layer?.color,
      opts: { transformation: this.transform },
      style: { 'font-size': num(el, 'size', 1.778) * 1.5 },
    });
  }

  renderCollection(collection, opts = {}) {
    const items = [...collection];

    return [
      ...items.filter(el => el.tagName != 'text'),
      ...items.filter(el => el.tagName == 'text'),
    ]
      .map(el => this.renderItem(el, opts))
      .filter(Boolean);
  }

  /* bounds: BBox in final SVG coordinates (y down) */
  renderSVG(children, bounds) {
    const { x1, y1, width, height } = bounds;
    const rect = new Rect(x1, y1, width, height);
    const grid = findChild(this.adapter.drawing, 'grid');

    return h(
      SVG,
      {
        viewBox: `${x1} ${y1} ${width} ${height}`,
        width: `${width}mm`,
        height: `${height}mm`,
        defs: grid
          ? h(Pattern, { data: this.adapter.wrap(grid), id: 'grid' })
          : [],
      },
      [
        h(Background, { rect, attrs: { color: '#ffffff', visible: true } }),
        grid
          ? h(Grid, {
              data: this.adapter.wrap(grid),
              id: 'grid',
              rect,
              attrs: { color: '#0000aa', visible: true },
            })
          : null,
        h(
          'g',
          {
            class: 'drawing',
            transform: this.transform,
            'font-family': 'Fixed',
            'font-size': 0.6,
            'stroke-linecap': 'round',
            'stroke-linejoin': 'round',
          },
          children,
        ),
      ],
    );
  }
}

export class SchematicRenderer extends EagleSVGRenderer {
  render(sheetNo = 0) {
    const { adapter, transform } = this;
    const schematic = findChild(adapter.drawing, 'schematic');
    const sheet = filterChildren(findChild(schematic, 'sheets'), 'sheet')[
      sheetNo
    ];
    const plain = sheet && findChild(sheet, 'plain');
    const nets = sheet && findChild(sheet, 'nets');
    const instances = sheet && findChild(sheet, 'instances');

    const bounds = new BBox();
    if(plain) bboxOfContainer(plain, bounds);
    if(instances)
      for(const inst of filterChildren(instances, 'instance'))
        bounds.add(num(inst, 'x'), -num(inst, 'y'));
    if(nets)
      for(const net of filterChildren(nets, 'net'))
        for(const seg of filterChildren(net, 'segment'))
          bboxOfContainer(seg, bounds);
    if(!bounds.valid) bounds.add(0, 0);
    bounds.outset(2.54);

    const children = [];

    if(plain)
      children.push(
        h('g', { class: 'plain' }, this.renderCollection(plain.children)),
      );

    if(nets)
      children.push(
        h(
          'g',
          { class: 'nets' },
          filterChildren(nets, 'net').map(net =>
            h(
              'g',
              { class: 'net', 'data-name': net.getAttribute('name') },
              filterChildren(net, 'segment')
                .map(seg =>
                  this.renderCollection(seg.children, {
                    labelText: net.getAttribute('name'),
                  }),
                )
                .flat(),
            ),
          ),
        ),
      );

    if(instances)
      children.push(
        h(
          'g',
          { class: 'instances' },
          filterChildren(instances, 'instance').map(inst =>
            h(Instance, {
              data: adapter.wrap(inst),
              opts: { transformation: transform },
            }),
          ),
        ),
      );

    return this.renderSVG(children, bounds);
  }
}

export class BoardRenderer extends EagleSVGRenderer {
  render() {
    const { adapter, transform } = this;
    const board = findChild(adapter.drawing, 'board');
    const plain = findChild(board, 'plain');
    const signals = findChild(board, 'signals');
    const elements = findChild(board, 'elements');

    let bounds = plain && measuresBBox(adapter, plain);
    if(!bounds) {
      bounds = new BBox();
      if(plain) bboxOfContainer(plain, bounds);
      if(elements)
        for(const e of filterChildren(elements, 'element'))
          bounds.add(num(e, 'x'), -num(e, 'y'));
      if(signals)
        for(const sig of filterChildren(signals, 'signal'))
          bboxOfContainer(sig, bounds);
      if(!bounds.valid) bounds.add(0, 0);
    }
    bounds.outset(1.27);

    const children = [];

    if(plain)
      children.push(
        h('g', { class: 'plain' }, this.renderCollection(plain.children)),
      );

    if(signals)
      children.push(
        h(Signals, {
          data: adapter.wrap(signals),
          opts: { transformation: transform },
        }),
      );

    if(elements)
      children.push(
        h(
          'g',
          { class: 'elements' },
          filterChildren(elements, 'element').map(el =>
            h(Element, {
              data: adapter.wrap(el),
              opts: { transformation: transform },
            }),
          ),
        ),
      );

    return this.renderSVG(children, bounds);
  }
}

export class LibraryRenderer extends EagleSVGRenderer {
  render() {
    const { adapter, transform } = this;
    const library = findChild(adapter.drawing, 'library');
    const boardAdapter = new EagleAdapter(
      this.doc,
      Palette.board((r, g, b) => new RGBA(r, g, b)),
    );
    const gap = 2.54;

    const bounds = new BBox();
    const children = [];
    let x = 0;

    for(const [tag, component, itemAdapter] of [
      ['symbol', SchematicSymbol, adapter],
      ['package', Package, boardAdapter],
    ]) {
      const collection = findChild(library, tag + 's');
      if(!collection) continue;

      for(const item of filterChildren(collection, tag)) {
        const bb = bboxOfContainer(item);
        if(!bb.valid) bb.add(0, 0);

        const dx = x - bb.x1;

        children.push(
          h(
            'g',
            {
              class: `${tag} ${item.getAttribute('name')}`,
              transform: `translate(${dx},0)`,
            },
            [
              h(component, {
                data: itemAdapter.wrap(item),
                opts: { transformation: transform },
              }),
            ],
          ),
        );

        bounds.add(bb.x1 + dx, bb.y1);
        bounds.add(bb.x2 + dx, bb.y2);
        x += bb.width + gap;
      }
    }

    if(!bounds.valid) bounds.add(0, 0);
    bounds.outset(2.54);

    return this.renderSVG(children, bounds);
  }
}

export function Renderer(doc) {
  switch (doc.type) {
    case 'sch':
    case 'schematic':
      return new SchematicRenderer(doc);
    case 'brd':
    case 'board':
      return new BoardRenderer(doc);
    case 'lbr':
    case 'library':
      return new LibraryRenderer(doc);
    default:
      throw new Error(`Unsupported document type: ${doc.type}`);
  }
}
