import { EagleDocument } from './lib/eagle.js';
import { LineList, Rect } from './lib/geom.js';
import { toXML } from './lib/json.js';
import fs from 'fs';
import Util from './lib/util.js';
import { Console } from 'console';
import { digit2color, GetFactor, GetColorBands, ValueToNumber, NumberToValue, GetExponent, GetMantissa } from './lib/eda/colorCoding.js';
import { UnitForName } from './lib/eda/units.js';
import { updateMeasures, alignItem, alignAll } from './eagle-commands.js';

let documents = [];

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, '>\n    <') : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

function scientific(value) {
  let sci = [GetMantissa(value), GetExponent(value)];

  Util.define(sci, {
    toString() {
      let sign = Math.sign(this[1]) < 0 ? '-' : '+';
      return `${this[0]}e${sign}${(Math.abs(this[1]) + '').padStart(2, '0')}`;
    }
  });
  return sci;
}
const color = Util.coloring(true);

const verticalRectangles = ['▮ ', '▯ '];
const largeSquares = ['■', '□'];

function num2color(num, square = false) {
  let sym = square ? largeSquares : verticalRectangles;
  let c = typeof num == 'number' ? GetColorBands(num) : num;

  return c.map(n => color.text(n ? sym[0] : color.text(sym[1], 38, 5, 236), n ? 38 : 48, ...digit2color.ansi[n].slice(1))).join('');
}
const SubstChars = str => str.replace(/\xCE\xBC/g, '\u00B5').replace(/\xCE\xA9/g, '\u2126');

async function main(...args) {
  globalThis.console = new Console({ inspectOptions: { breakLength: 100, colors: true, depth: Infinity, compact: 2, customInspect: true } });
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt2.brd');
  args = Util.unique(args);
  for(let arg of args) {
    let data = fs.readFileSync(arg);
    console.log(`loaded '${arg}' length: ${data.length}`);
    let doc = new EagleDocument(data, null, arg);
    documents.push(doc);
  }
  let components = {
    C: [],
    L: [],
    R: []
  };
  for(let doc of documents) {
    let main = doc.mainElement;
    console.log('main:', main);
    let parts = [...(main.elements || main.parts)].map(([name, elem]) => [name, typeof elem.value == 'string' ? SubstChars(elem.value) : elem.value]);
    console.log('parts', parts);
    let matchers = [
      [/^R/, /^[0-9.]+([kKmM][Ω\u03A9]?|[Ω\u03A9]?)(|\/[0-9.]+W)/],
      [/^C/, /^[0-9.]+([pnuμm]F?|F?)(|\/[0-9.]+V)/],
      [/^L/, /^[0-9.]+([nuμm]H?|H?)/]
    ];

    let nameValueMap = new Map(parts.filter(([name, value]) => matchers.some(m => m[0].test(name) && m[1].test(value))));
    console.log(
      'nameValueMap',
      new Map(
        [...nameValueMap].map(([n, v]) => [
          n,
          v
        ])
      )
    );
    for(let [name, value] of nameValueMap) {
      value = value.replace(/[\u0000-\u001F\u007F-\uFFFF]/g, '');
      components[name[0]].push(value.replace(/[ΩFH]$/, '').replace(/^\./, '0.'));
    }
  }
  let histograms = {};
  let values = {};
  console.log('components', components);
  for(let value of [2.2, 4.7e3]) {
    console.log(`GetColorBands(${value}, 3)`, GetColorBands(value, 2));
  }
  for(let key in components) {
    components[key].sort();
    let hist = Util.histogram(components[key],  new Map());

    histograms[key] = new Map([...hist].sort((a, b) => b[1] - a[1]));
    values[key] = [...histograms[key]]
      .map(([value, count]) => {


        return [value || scientific(value).toString(), ValueToNumber(value), count];
      })
      .sort((a, b) => a[1] - b[1])
      .map(([val, rat, count]) => [(val + '').padStart(4, ' ') + UnitForName(key),  rat >= 1 ? num2color(rat) : '', `  × ${count}`].join(' '));
  }
  console.log('components:', components);


  console.log(
    'values:\n   ' +
      Object.entries(values)
        .map(([key, list]) => `${key}:\n\t${list.join('\n\t')}`)
        .join('\n   ')
  );

  return;
  console.log('digit2color:', digit2color);
  for(let value of [33000, 1800, 470, 1e6, 4.7e3]) {
    console.log('GetColorBands', GetColorBands(value, 3));
    console.log('GetColorBands', GetColorBands(value, 2));
    console.log('NumberToValue', NumberToValue(value));
  }
  for(let value of ['33k', '1.8k', '470', '1.8k', '47k', '10M', '4.7k']) {
    console.log(`ValueToNumber(${value})`, ValueToNumber(value));
  }
}

Util.callMain(main, true);

