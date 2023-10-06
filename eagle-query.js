import { num2color, scientific } from './eagle-commands.js';
import { ReadFile } from './io-helpers.js';
import { HSLA } from './lib/color.js';
import { EagleDocument } from './lib/eagle.js';
import { PartScales, ValueToNumber } from './lib/eda/colorCoding.js';
import { UnitForName } from './lib/eda/units.js';
import { toXML } from './lib/json.js';
import { unique } from './lib/misc.js';
import { Console } from 'console';
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
    let data = ReadFile(arg);
    console.log(`loaded '${arg}'`);
    let doc = new EagleDocument(data, null, arg);
    documents.push(doc);
  }
  let components = {
    C: [],
    L: [],
    R: []
  };
  for(let doc of documents) {
    let parts = [...(doc.elements || doc.parts)].map(([name, elem]) => [name, typeof elem.value == 'string' ? SubstChars(elem.value) : elem.value]);
    let matchers = [
      [/^R/, /^[0-9.]+([kKmM][Ω\u03A9]?|[Ω\u03A9]?)(|\/[0-9.]+W)/],
      [/^C/, /^[0-9.]+([pnuμ\u03bcm]F?|F?)(|\/[0-9.]+V)/],
      [/^L/, /^[0-9.]+([nuμ\u03bcm]H?|H?)/]
    ];
    let nameValueMap = new Map(parts.filter(([name, value]) => matchers.some(m => m[0].test(name) && m[1].test(value))));
    for(let [name, value] of nameValueMap) {
      value = (value ? '' + value : '').replace(/[\u0000-\u001F]/g, '');
      components[name[0]].push(value.replace(/[ΩFH]$/, '').replace(/^\./, '0.'));
    }
  }
  let histograms = {};
  let values = {};
  for(let key in components) {
    components[key].sort();
    let hist = histogram(components[key], new Map());

    histograms[key] = new Map([...hist].sort((a, b) => b[1] - a[1]));
    values[key] = [...histograms[key]]
      .map(([value, count]) => {
        return [value || scientific(value).toString(), ValueToNumber(value), count];
      })
      .sort((a, b) => a[1] - b[1])
      .map(([val, rat, count]) => {
        const scal = PartScales[key[0]];
        let inductorColor = new HSLA({ h: 161, s: 60, l: 50, a: 1 }).toRGBA();
        //inductorColor=new HSLA({ h: 90, s: 90, l: 50, a: 1 }).toRGBA();
        //inductorColor=new HSLA({ h: 117, s: 70, l: 60, a: 1 }).toRGBA();

        let bands = key[0] == 'C' ? [' '] : [num2color(rat * scal, key[0] == 'L' ? [...inductorColor].slice(0, 3) : undefined)];
        return [key, (val + '').substring(0, 10).padStart(10, ' ') + UnitForName(key), bands.join(' '), `  × ${count}`].join(' ');
      });
  }
  console.log(
    //'values:\n' +
    '   ' +
      Object.entries(values)
        .map(([key, list]) => `${key}:\n\t${list.join('\n\n\t')}`)
        .join('\n   ')
  );
  console.log(`\r\nFinished querying`, ...args);
  return;
}

main(...scriptArgs.slice(1));