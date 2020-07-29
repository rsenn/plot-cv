import fs, { promises as fsPromises } from 'fs';
import util from 'util';
import Util from './lib/util.js';
import { XMLIterator } from './lib/xml/util.js';
import deep from './lib/deep.js';
import tXml from './lib/tXml.js';
import { RGBA, HSLA } from './lib/color.js';
import { Iterator, IteratorForwarder } from './lib/iterator.js';
import toSource from './lib/tosource.js';
import { toXML, Path } from './lib/json.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { Console } from 'console';
import Alea from './lib/alea.js';
import { Functional } from './lib/functional.js';
import KolorWheel from './lib/KolorWheel.js';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});

//prettier-ignore
const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); },
  stat(filename) {return fs.statSync(filename); }
};

function readXML(filename) {
  //Util.log('readXML', filename);
  let data = filesystem.readFile(filename);
  let xml = tXml(data);
  //Util.log('xml:', xml);
  return xml;
}
//TODO: Test with tmScheme (XML) and ColorMap

const push_back = (arr, ...items) => [...(arr || []), ...items];
const push_front = (arr, ...items) => [...items, ...(arr || [])];

function main(...args) {
  let colors, keys;
  let attributes = new Map();
  let numeric = new Set();
  const printSet = set => [...set.values()].map(n => "'" + n + "'").join(', ');

  const setAttributes = (tag, attrs) => {
    if(!attributes.has(tag)) attributes.set(tag, new Set());

    let s = attributes.get(tag);

    for(let name in attrs) {
      s.add(name);

      const value = attrs[name];

      if(Util.isNumeric(value)) numeric.add(name);
    }
  };

  try {
    for(let filename of args) {
      let xml = readXML(filename);

      for(let [element, path] of XMLIterator.iterate(xml[0])) {
        //console.log('element:',element.tagName, element.attributes);
        setAttributes(element.tagName, element.attributes);
      }
    }
  } catch(err) {
    Util.putError(err);
  }

  for(let [tag, attr] of attributes) {
    let names = [...attr.values()];
    if(names.length == 0) continue;

    console.log(` ${tag}: [${printSet(attr)}],`);
  }

  console.log('numeric: ' + printSet([...numeric.values()].sort()));
}
main(...process.argv.slice(2));
