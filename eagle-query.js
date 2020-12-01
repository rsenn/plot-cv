import { EagleDocument, EagleProject } from './lib/eagle.js';
import PortableFileSystem from './lib/filesystem.js';
import { LineList, Rect } from './lib/geom.js';
import { toXML } from './lib/json.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Graph } from './lib/fd-graph.js';
import ptr from './lib/json-ptr.js';
import LogJS from './lib/log.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tXml from './lib/tXml.js';

let filesystem,
  documents = [];

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, '>\n    <') : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

function updateMeasures(board) {
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
    plain.append(...lines.map(line => ({
        tagName: 'wire',
        attributes: { ...line.toObject(), layer: 47, width: 0 }
      }))
    );
  }
  return !measures;
}

function alignItem(item) {
  console.debug('alignItem', item);
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

function alignAll(doc) {
  if(!doc) return false;
  let items = doc.getAll(doc.type == 'brd' ? 'element' : 'instance');
  let changed = false;
  for(let item of items) changed |= alignItem(item);
  let signals_nets = doc.getAll(/(signals|nets)/);
  //console.log('signals_nets:', signals_nets);
  for(let net of signals_nets) for (let item of net.getAll('wire')) changed |= alignItem(item);
  return !!changed;
}

function exponent(value) {
  const suffix = value.replace(/[^KkMmnpuμ]/g, '');
  let exp = 0;
  if(suffix.length > 1) throw new Error(`Suffix '${suffix}' length > 1`);
  switch (suffix) {
    case 'M':
      exp = 6;
      break;
    case 'K':
    case 'k':
      exp = 3;
      break;
    case 'm':
      exp = -3;
      break;
    case 'μ':
    case 'u':
      exp = -6;
      break;
    case 'n':
      exp = -9;
      break;
    case 'p':
      exp = -12;
      break;
  }
  return exp;
}

function mantissa(value) {
  let mantissa = value.replace(/[KkMmnpuμ]$/, '');
  if(isNaN(+mantissa)) throw new Error(`Mantissa '${mantissa}' not a valid number`);
  return +mantissa;
}

function scientific(value) {
  let sci = [mantissa(value), exponent(value)];

  Util.define(sci, {
    toString() {
      let sign = Math.sign(this[1]) < 0 ? '-' : '+';
      return `${this[0]}e${sign}${(Math.abs(this[1]) + '').padStart(2, '0')}`;
    }
  });
  return sci;
}

function rational(value) {
  let exp = exponent(value);
  let man = mantissa(value);

  return man * 10 ** exp;
}

const multipliers = [10, 100, 1e3, 1e4, 1e5, 1e6, 1e7];

function factor(num) {
  //console.log('num:', num);
  let multipliers = [10, 100, 1e3, 1e4, 1e5, 1e6, 1e7];
  let i = -1;
  for(let max of multipliers) {
    //console.log('', { num, max, i });
    if(num >= max) i++;
    else break;
  }
  return i;
  //  return upper.reduce((acc, max,i) => (num < max ? i : acc), -1);
}

function bands(num) {
  let f = factor(num);
  let multiplier = multipliers[f];
  let x = num / multiplier;

  //console.log('bands', { f, x });
  let a = Math.floor(x);
  let b = Math.round((x % 1) * 10);

  return [a, b, f];
}

const bandColors = [
  [48, 5, 16], // black
  [48, 5, 94], // brown
  [48, 5, 160], // red
  [48, 5, 208], // orange
  [48, 5, 226], // yellow
  [48, 5, 40], // green
  [48, 5, 27], // blue
  [48, 5, 63], // violet
  [48, 5, 241], // grey
  [48, 5, 231], // white
  [48, 5, 172], // gold
  //  [48,5,251],
  [48, 5, 249] // silver
];

const color = Util.coloring(true);

const verticalRectangles = ['▮', '▯'];
const largeSquares = ['⬛', '⬜'];

function num2color(num, square = true) {
  let sym = square ? largeSquares : verticalRectangles;
  let c = typeof num == 'number' ? bands(num) : num;
  /*console.log('c:', c);
  console.log('num:', num);*/
  return c
    .map(n => color.text(n ? sym[0] : color.text(sym[1], 38, 5, 236), n ? 38 : 48, ...bandColors[n].slice(1)))
    .join('');
}

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity, breakLength: 100 });
  await PortableFileSystem(fs => (filesystem = fs));

  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt2.brd');

  args = Util.unique(args);

  for(let arg of args) {
    let data = filesystem.readFile(arg);
    console.log(`loaded '${arg}' length: ${data.length}`);
    let doc = new EagleDocument(data, null, arg);
    documents.push(doc);
  }
  //console.log('documents:', documents);

  let components = {
    C: [],
    L: [],
    R: []
  };

  for(let doc of documents) {
    let main = doc.mainElement;
     console.log('main:', main);


    let parts = [...(main.elements || main.parts)].map(([name, elem]) => [name, elem.value]);

    let matchers = [
      [/^R/, /^[0-9.]+([kKmM]Ω?|Ω)(|\/[0-9.]+W)$/],
      [/^C/, /^[0-9.]+([pnuμm]F?|F)(|\/[0-9.]+V)$/],
      [/^L/, /^[0-9.]+([nuμm]H?|H)$/]
    ];

    let nameValueMap = new Map(parts.filter(([name, value]) => matchers.some(m => m[0].test(name) && m[1].test(value)))
    );

    for(let [name, value] of nameValueMap) {
      //  Util.insertSorted(components[name[0]], value); X
      components[name[0]].push(value.replace(/[ΩFH]$/, '').replace(/^\./, '0.'));
    }
    // console.log('nameValueMap:',nameValueMap);
  }
  let histograms = {};
  let values = {};

  for(let key in components) {
    components[key].sort();
    let hist = Util.histogram(components[key], (item, i) => [item[1], i], new Map());
    histograms[key] = new Map([...hist].sort((a, b) => b[1] - a[1]));
    values[key] = [...histograms[key]]
      .map(([value, count]) =>
        //rational(value).toExponential()
        [value || scientific(value).toString(), rational(value), count]
      )
      .sort((a, b) => a[1] - b[1])
      .map(([val, rat, count]) =>
        [(val + '').padStart(4, ' '), /*rat, factor(rat),*/ rat >= 1 ? num2color(rat) : '', `  × ${count}`].join(' ')
      );
  }

  /*console.log('components:', components);
  console.log('histograms:', histograms);*/
  console.log('values:\n   '+
    Object.entries(values)
      .map(([key, list]) => `${key}:\n\t${list.join('\n\t')}`)
      .join('\n   ')
  );
}

Util.callMain(main, true);
