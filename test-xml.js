import fs, { promises as fsPromises } from 'fs';

import Util from './lib/util.js';
import { XPath } from './lib/xml.js';
import deep from './lib/deep.js';
import tXml from './lib/tXml.js';
import { RGBA, HSLA } from './lib/color.js';
import { Iterator, IteratorForwarder } from './lib/iterator.js';
import toSource from './lib/tosource.js';
import { toXML, Path } from './lib/json.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 8, colors: true }
});

// prettier-ignore
const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); }
};

function readXML(filename) {
  console.log('readXML', filename);
  let data = filesystem.readFile(filename);

  let xml = tXml(data);
  /// console.log('xml:', xml);

  return xml;
}
// TODO: Test with tmScheme (XML) and ColorMap

function main(...args) {
  console.log('main', args);
  if(args.length == 0) args = ['/home/roman/.config/sublime-text-3/Packages/Babel/Next.tmTheme' /*  */];
  for(let filename of args)
    try {
      let xml = readXML(filename);
      let json = JSON.stringify(xml);
      let basename = filename.replace(/.*\//g, '').replace(/\.[^.]*$/, '');
      console.log('basename ', basename);
      //  let js = toSource(xml);
      filesystem.writeFile(basename + '.json', json);
      //filesystem.writeFile('HoerMalWerDaHaemmert.js', js);
      let flat = deep.flatten(
        xml[0],
        new Map(),
        (v, p) => typeof v != 'object',
        (p, v) => [new Path(p), v]
      );
      let colors = new Map([...Iterator.filter(flat, ([path, value]) => /^#[0-9A-Fa-f]*$/.test(value))].map(([path, value]) => [path, new RGBA(value)]));
      console.log('colors:', colors);
      let obj = {};
      for(let [path, value] of flat) {
        //        console.log('path:', path, ' value:', value);
        deep.set(obj, path, value);
      }
      filesystem.writeFile(basename + '.xml', toXML(obj));
      console.log('methods:', Util.getMethodNames(Iterator));
      let it = new IteratorForwarder(colors.entries());
      console.log('it.map', it.map + '');
      let paths = it.map(([path, value]) => [XPath.from(path, xml[0]), path]);
      let o = it.map(([path, value]) => {
        const key = path.up(2).prevSibling;
        let paths = [
          ...key.walk(p => {
            if(p.at(-1) > 0) return p.up(1).down(0);
            return p.length > 2 && p.up(2);
          })
        ];
        return [
          paths
            .map(p => {
              let ret = deep.get(xml[0], p);
              while(ret.children && typeof ret.children[0] == 'object') {
                ret = ret.children[0];
                p = p.down('children', 0);
              }
              return p;
            })
            .filter(Util.uniquePred(Path.equal))
            .map(p => deep.get(xml[0], p.down('children', 0)))
            .filter(key => ['settings', 'name'].indexOf(key) == -1),
          value
        ];
      });
      console.log('o:', o);
      colors = [...o].map(([name, value]) => value);
      console.log('colors:', colors);
      console.log('colors:', colors.values());
      let palette = new ColorMap([...colors.values()]);
      console.log('palette:', palette);
    } catch(err) {
      console.log('err:', err);
    }
}

main(...process.argv.slice(2));
