import { unique } from './lib/misc.js';
import { EagleDocument } from './lib/eagle.js';
import { toXML } from './lib/json.js';
import fs from 'fs';
import Util from './lib/util.js';
import { Console } from 'console';
import { digit2color, GetColorBands, ValueToNumber, NumberToValue, PartScales } from './lib/eda/colorCoding.js';
import { UnitForName } from './lib/eda/units.js';
import { num2color, scientific } from './eagle-commands.js';

let documents = [];

function xmlize(obj, depth = 2) {
  return obj.toXML ? obj.toXML().replace(/>\s*</g, '>\n    <') : EagleDocument.toXML(obj, depth).split(/\n/g)[0];
}

const SubstChars = str => str.replace(/\xCE\xBC/g, '\u00B5').replace(/\xCE\xA9/g, '\u2126');

async function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { breakLength: 100, colors: true, depth: Infinity, compact: 2, customInspect: true }
  });
  if(args.length == 0) args.unshift('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt2.brd');
  args = unique(args);
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
    //console.log('main:', doc);
     let parts = [...(doc.elements || doc.parts)].map(([name, elem]) => [
      name,
      typeof elem.value == 'string' ? SubstChars(elem.value) : elem.value
    ]);
    //console.log('parts', console.config({ compact: false }), Object.fromEntries(parts));
    let matchers = [
      [/^R/, /^[0-9.]+([kKmM][Ω\u03A9]?|[Ω\u03A9]?)(|\/[0-9.]+W)/],
      [/^C/, /^[0-9.]+([pnuμ\u03bcm]F?|F?)(|\/[0-9.]+V)/],
      [/^L/, /^[0-9.]+([nuμ\u03bcm]H?|H?)/]
    ];

    let nameValueMap = new Map(
      parts.filter(([name, value]) => matchers.some(m => m[0].test(name) && m[1].test(value)))
    );
    //console.log('nameValueMap', new Map([...nameValueMap].map(([n, v]) => [n, v])));
    for(let [name, value] of nameValueMap) {
      value = (value ? '' + value : '').replace(/[\u0000-\u001F]/g, '')/*.replace(/[\u007F-\uFFFF]/g, '')*/;
      components[name[0]].push(value.replace(/[ΩFH]$/, '').replace(/^\./, '0.'));
    }
  }
  let histograms = {};
  let values = {};
  //
  /* for(let value of [2.2, 4.7e3]) {
    console.log(`GetColorBands(${value}, 3)`, GetColorBands(value, 2));
  }*/
  for(let key in components) {
    components[key].sort();
    //console.log(`component ${key}`, components[key]);
    let hist = Util.histogram(components[key], new Map());

    histograms[key] = new Map([...hist].sort((a, b) => b[1] - a[1]));
    values[key] = [...histograms[key]]
      .map(([value, count]) => {
        return [value || scientific(value).toString(), ValueToNumber(value), count];
      })
      .sort((a, b) => a[1] - b[1])
      .map(([val, rat, count]) => {
        //console.log('c', { val, rat, count });
        const scal = PartScales[key[0]];

        let bands =
          key[0] == 'C' ? [' '] /*?? GetColorBands(rat * scal, 2).map(b => `[${b}]`)*/ : [num2color(rat * scal)];
        //  console.log('c', { bands });

        return [
          key,
          (val + '').substring(0, 10).padStart(10, ' ') + UnitForName(key),
          bands.join(' '),
          `  × ${count}`
        ].join(' ');
      });
  }
  // console.log('components:', components);

  console.log(
    'values:\n   ' +
      Object.entries(values)
        .map(([key, list]) => `${key}:\n\t${list.join('\n\n\t')}`)
        .join('\n   ')
  );
  console.log(`\r\nFinished querying`, ...args);

  return;
  /* console.log('digit2color:', digit2color);
  for(let value of [33000, 1800, 470, 1e6, 4.7e3]) {
    console.log('GetColorBands', GetColorBands(value, 3));
    console.log('GetColorBands', GetColorBands(value, 2));
    console.log('NumberToValue', NumberToValue(value));
  }
  for(let value of ['33k', '1.8k', '470', '1.8k', '47k', '10M', '4.7k']) {
    console.log(`ValueToNumber(${value})`, ValueToNumber(value));
  }*/
}

Util.callMain(main, true);
