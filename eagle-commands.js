import { LineList, Rect, Point } from './lib/geom.js';
import Util from './lib/util.js';
import { BG, digit2color, GetColorBands, PartScales } from './lib/eda/colorCoding.js';
export { GetColorBands } from './lib/eda/colorCoding.js';
import { map, consume, reduce } from './lib/iterable.js';

export function GetParts(schematic, t = entries => Object.fromEntries(entries)) {
  return t(map(schematic.getAll('part'), elem => [elem.name, elem]));
}

export function GetElements(board, t = entries => Object.fromEntries(entries)) {
  return t(map(board.getAll('element'), elem => [elem.name, elem]));
}

export function GetInstances(schematic, t = entries => /*Object.fromEntries*/ entries) {
  let entries = Object.entries(
    reduce(
      schematic.getAll('instance'),
      (accu, elem) => {
        const { name } = elem.part;
        if(!(name in accu)) accu[name] = [];

        accu[name].push(elem);
        return accu;
      },
      {}
    )
  );

  return t(entries);
}

export function GetPositions(doc) {
  let entries = { sch: d => GetInstances(d), brd: d => GetElements(d, e => e).map(([name, elem]) => [name, [elem]]) }[
    doc.type
  ](doc);

  return entries.map(([name, arr]) => [name, arr.map(({ x, y }) => new Point(x, y))]);
}

export function UpdateMeasures(board) {
  if(!board) return false;
  let bounds = board.getBounds();
  let measures = board.getMeasures();
  if(measures) {
    console.log('got measures:', measures);
  } else {
    let rect = new Rect(bounds.rect);
    let lines = rect.toLines(lines => new LineList(lines));
    let { plain } = board;
    plain.remove(e => e.tagName == 'wire' && e.attributes.layer == '47');
    plain.append(
      ...lines.map(line => ({
        tagName: 'wire',
        attributes: { ...line.toObject(), layer: 47, width: 0 }
      }))
    );
  }
  return !measures;
}

export function AlignItem(item) {
  console.debug('AlignItem', item);
  let geometry = item.geometry;
  let oldPos = geometry.clone();
  let newPos = geometry.clone().round(1.27, 2);
  let diff = newPos.diff(oldPos).round(0.0001, 5);
  let before = item.parentNode.toXML();
  geometry.add(diff);
  let changed = !diff.isNull();
  if(changed) {
    console.log('before:', Util.abbreviate(before));
    console.log('after:', Util.abbreviate(item.parentNode.toXML()));
    console.log('align\n', item.xpath(), '\n newPos:', newPos, '\n diff:', diff, '\n attr:', item.raw.attributes);
  }
  return changed;
}

export function AlignAll(doc) {
  if(!doc) return false;
  let items = doc.getAll(doc.type == 'brd' ? 'element' : 'instance');
  let changed = false;
  for(let item of items) changed |= AlignItem(item);
  let signals_nets = doc.getAll(/(signals|nets)/);

  for(let net of signals_nets) for (let item of net.getAll('wire')) changed |= AlignItem(item);
  return !!changed;
}

export function scientific(value) {
  let sci = [GetMantissa(value), GetExponent(value)];

  define(sci, {
    toString() {
      let sign = Math.sign(this[1]) < 0 ? '-' : '+';
      return `${this[0]}e${sign}${(Math.abs(this[1]) + '').padStart(2, '0')}`;
    }
  });
  return sci;
}
const text = (() => {
  const ansi = Util.coloring(true);
  return (...args) => ansi.text(...args);
})();

const verticalRectangles = ['█', '□', '\u2588\u258d' /*'\u2589'*/ /*'▮'*/, '□', '▯'];
const largeSquares = ['■', '□'];

export function num2color(num, square = false) {
  let sym = square ? largeSquares : verticalRectangles;
  let bands = typeof num == 'number' ? GetColorBands(num) : num;
  const { ansi, rgb } = digit2color;
  let a = n => ansi[n]?.slice(1) ?? [];
  let ret = []; //bands.map(n => );
  //ret.push(text('\u2595', 38, 2, ...digit2color.rgb[BG]));

  for(let band of bands) {
    if(ret.length) ret.push(text(' ', 48, 2, ...rgb[BG]));
    //
    //  ret.push(text(sym[0], 38));
    if(band) {
      band = text(sym[0], 38, ...a(band));
      band = text(band, 48, 2, ...rgb[BG]);
    } else {
      band = text(sym[1], 38, 2, 255, 255, 255);
    }
    ret.push(band); //
  }
  ret.push(text(' ', 48, 2, ...rgb[BG]));
  //console.log('num2color', { bands, ret });
  return ret.join('');
}
