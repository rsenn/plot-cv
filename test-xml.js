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
import Alea from './lib/alea.js';

const prng = new Alea(Date.now());

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

const push_back = (arr, ...items) => [...(arr || []), ...items];
const push_front = (arr, ...items) => [...items, ...(arr || [])];

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
        (v, p) => (typeof v != 'object' && p.indexOf('attributes') == -1) || p.indexOf('attributes') == p.length - 1,
        (p, v) => [new Path(p), v]
      );
      let colors = new Map([...Iterator.filter(flat, ([path, value]) => /^#[0-9A-Fa-f]*$/.test(value))].map(([path, value]) => [path, new RGBA(value)]));
      //console.log('colors:', colors);
      let obj = {};
      //console.log('methods:', Util.getMethodNames(Iterator));
      let it = new IteratorForwarder(colors.entries());
      // console.log('it.map', it.map + '');
      let paths = Iterator.map(colors, ([path, value]) => [XPath.from(path, xml[0]), deep.get(xml[0], path), path]);
      let o = it.map(([path, value]) => {
        const key = path.up(0);
        let prev = new Path([]),
          prevValue = {},
          list = [];
        console.log('paths:', path);
        let paths = [
          ...key.walk((p, i, abort, skip) => {
            let r;
            list = push_front(list, p);

            let value = deep.get(xml[0], p);
            if((value + '').startsWith('#')) skip();

            if(0 && Util.isObject(value) && value.tagName !== undefined) {
              if(value.tagName == 'dict' || value.tagName == 'array') skip();
              const text = value.children && value.children[0];
              if(text == 'settings') skip();
              if(value.tagName == 'string' && text == 'Next') skip();
              if((text + '').startsWith('#')) skip();
              if(prev.length == p.length) {
                if(value.tagName == 'key' && prev.tagName == 'string') {
                  skip();
                  prev = p;
                  return p.up(2);
                }
              }
            }

            return p.prevSibling || p.parentNode;

            if(p.last > 0) r = p.left(1);
            else if(p.length > 4) r = p.up(2);
            prev = p;
            return r;
          })
        ];
        paths = paths.filter(Util.uniquePred(Path.equal));
        paths = paths.map(p => [p, deep.get(xml[0], p)]);
        paths = paths.map(([path, value]) => {
          return [path, value.children && typeof value.children[0] == 'string' ? { [value.tagName]: value.children[0] } : Util.filterOutKeys(value, ['children', 'attributes'])];
        });
        return [new Map(paths), path, value];
      });
      o = [...o].filter(([paths, path, value]) => !/background/i.test(paths));
      console.log('o:', [...o]);

      colors = new Map([...o].map(([paths, path, value]) => [path, value]));
      console.log('colors:', colors);
      //  console.log('colors:', colors.values());

      let palette = new ColorMap(HSLA, colors);

      for(let item of palette) {
        let [key, color] = item;

        //console.log("old:",key,color);
        color = HSLA.random([0, 360], [color.s, 255 - (255 - color.s) / 2], [(color.l * 2) / 3, 255 - ((255 - color.l) * 3) / 4], [color.a, color.a], prng);

        flat.set(key, color.toRGBA().hex());

        //console.log("new:",key,color);
      }

      const newObj = {};
      console.log('palette:', palette);
      console.log('paths:', [...paths]);

      for(let [path, value] of flat) {
        console.log('path:', path, ' value:', value);
        deep.set(newObj, path, value);
      }
      console.log('newObj:', newObj);
      filesystem.writeFile(basename + '.xml', toXML(newObj));
    } catch(err) {
      console.log('err:', err);
    }
}

main(...process.argv.slice(2));
