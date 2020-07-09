import tXml from './lib/tXml.js';
import { XPath } from './lib/xml.js';
import fs, { promises as fsPromises } from 'fs';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import toSource from './lib/tosource.js';

// prettier-ignore
const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); }
};

function readXML(filename) {
  let data = filesystem.readFile('HoerMalWerDaHaemmert.html');

  let xml = tXml(data);
  console.log('xml:', xml);

  return xml;
}
// TODO: Test with tmScheme (XML) and ColorMap

function main(...args) {
  const filename = 'HoerMalWerDaHaemmert.html';
  let xml = readXML(filename);
  let json = JSON.stringify(xml);
  //  let js = toSource(xml);
  filesystem.writeFile('HoerMalWerDaHaemmert.json', json);
  //filesystem.writeFile('HoerMalWerDaHaemmert.js', js);
  let flat = deep.flatten(
    xml[0],
    new Map(),
    (v, p) => p.findIndex(part => /^(attributes|tagName)$/.test(part)) == -1 && p[p.length - 1] != 'children', // Util.isObject(v) && v.tagName !== undefined,
    p => p.join('.'),
    e => {
      if(e.tagName !== undefined) {
        const { tagName, attributes = {}, children } = e;
        const { style, ...rest } = attributes;
        return { tagName, attributes: rest /*, children*/ };
      }
      return e;
    }
  );
  console.log('flat:', flat);
}
main(process.argv.slice(2));
