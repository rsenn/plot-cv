import fs, { promises as fsPromises } from 'fs';
import util from 'util';
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
//prettier-ignore
const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); }
};
function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFile(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}
//TODO: Test with tmScheme (XML) and ColorMap

const push_back = (arr, ...items) => [...(arr || []), ...items];
const push_front = (arr, ...items) => [...items, ...(arr || [])];

function main(...args) {
  let colors, keys;

  //console.log('main', args);
  if(args.length == 0) args = ['/home/roman/.config/sublime-text-3/Packages/Babel/Next.tmTheme' /*  */];
  for(let filename of args)
    try {
      let xml = readXML(filename);
      let json = JSON.stringify(xml);
      let basename = filename.replace(/.*\//g, '').replace(/\.[^.]*$/, '');
      //console.log('basename ', basename);
      //let js = toSource(xml);
      filesystem.writeFile(basename + '.json', json);
      //filesystem.writeFile('HoerMalWerDaHaemmert.js', js);
      let flat = deep.flatten(
        xml[0],
        new Map(),
        (v, p) => (typeof v != 'object' && p.indexOf('attributes') == -1) || (p.length && p.indexOf('attributes') == p.length - 1),
        (p, v) => [new Path(p), v]
      );

      colors = new Map([...Iterator.filter(flat, ([path, value]) => /^#[0-9A-Fa-f]*$/.test(value))].map(([path, value]) => [path, new RGBA(value)]));
      console.log('colors:', colors); //[...colors].map(([path,value ]) => [path, value]));
      let obj = {};
      //console.log('methods:', Util.getMethodNames(Iterator));
      let it = new IteratorForwarder(colors.entries());
      //console.log('it.map', it.map + '');
      let paths = Iterator.map(colors, ([path, value]) => [XPath.from(path, xml[0]), deep.get(xml[0], path), path]);
      let o = it.map(([path, value]) => {
        const key = path.up(0);
        let prev = new Path([]),
          prevValue = {},
          list = [],
          numString = 0;
        //console.log('paths:', path);
        let paths = [
          ...key.walk((p, i, abort, skip) => {
            let r;
            let value = deep.get(xml[0], p);
            const children = value.children ? value.children : [];
            const text = typeof children[0] == 'string' ? children[0] : '';
            if(['Next', 'settings', 'scope', 'name'].indexOf(text) != -1 || /* text.startsWith('#') ||*/ typeof children[0] != 'string') {
              skip();
            }

            if(text.startsWith('#')) {
              skip();
              numString++;
            }

            if(numString > 1) skip();
            if(numString == 2 || ('' + (prevValue.children && prevValue.children[0]))[0] == '#') {
              //skip() ;
              prev = p;
              prevValue = value;
              numString = 0;
              return p.up(2);
            }
            //console.log('p:', p, util.inspect(value, { depth: 1 }));
            if(p.last > 0 && text != 'scope') r = p.left(1);
            else if(p.length > 2) r = p.up(2);
            prev = p;
            prevValue = value;
            return r;
          })
        ];
        /*   paths = paths.filter(Util.uniquePred(Path.equal));
        paths = paths.map(p => [p, deep.get(xml[0], p)]);*/
        /*        paths = paths.map(([path, value]) => [
          path,
          value.children && typeof value.children[0] == 'string'
            ? { [value.tagName]: value.children[0] }
            : Util.filterOutKeys(value, ['children', 'attributes'])
        ]);*/
        //paths = paths.filter(item => !((item[1].tagName == 'key' && item[1].children[0] == 'settings') || item[1].children[0].startsWith('#') ) );
        //paths = paths.reduce((acc, [path, value]) => [...(acc || []), [path, value]]);

        /*  paths = paths.filter(([path, value]) => value.tagName == 'key' && value.children[0] != 'settings');
         */
        paths = paths.map(path => path.concat(['children', 0]));
        return [
          path,
          paths
            .map(p => deep.get(xml[0], p))
            .reverse()
            .join('/'),
          value
        ];
      });
      o = [...o].filter(([p, k, v]) => !/background/i.test(k));
      keys = new Map(o.map(([p, k, v]) => [p, k]));
      colors = new Map(o.map(([p, k, v]) => [p, v]));
      //console.log('colors:', [...colors].slice(0, 10));
      let palette = new ColorMap(HSLA, colors);

      const lexOrder = key => {
        let i = 0;
        let r = [];
        for(i = 0; i < 10; i++) {
          const c = i < key.length ? key.charCodeAt(i) : 0;
          const x = c >= 0x30 && c <= 0x39 ? c - 0x30 : c >= 0x41 && c <= 0x5a ? c - 0x41 + 10 : c >= 0x61 && c <= 0x7a ? c - 0x61 + 36 : 0;
          r.push(x);
        }
        return r;
      };
      const hash = key => ((a, b) => a / b)(...lexOrder(key).reduce((acc, c) => [acc[0] * (10 + 26 + 26) + c, acc[1] * (10 + 26 + 26)], [0, 1]));

      let i = 0;
      for(let item of palette) {
        let [path, color] = item;
        const key = keys.get(path);
        const hashes = key
          .split(/\//g)
          .slice(0)
          .map(k => [k, k.split(/,? /g).map(k => k.split(/\./g).map(h => hash(h)))])
          .flat()
          .filter(i => typeof i != 'string')
          .flat();
        //console.log("hash:", hashes);

        if(/background/i.test(key)) continue;
        const { h, s, l, a } = color;
        //  console.log(`palette[${i++}] =`, key, color);

        let la = [l * 0.9, 255 - (255 - color.l) * 0.9];
        let hues = [hashes[0][0] - 0.4, hashes[0][0] + 0.4];
        let luminances = [hashes[0][1] || 0.4, hashes[0][1] || 0.8];
        color = HSLA.random([hues[0] * 360, hues[1] * 360], [s, 255 - (255 - s) / 2], la || [luminances[0] * 255, luminances[1] * 255], [a, a], prng);
        const rgba = color.toRGBA();
        //  console.log("",rgba);
        //    flat.set(path, color);
      }

      keys = [...palette.keys()];
      let values = [...palette.values()];

      let channels = values.reduce((a, c) => [a[0].concat(c.h), a[1].concat(c.s), a[2].concat(c.l), a[3].concat(c.a)], [[], [], [], []]);

      //    Util.shuffle(channels[0], prng);

      channels[0] = channels[0].map((c, i) => Util.randInt(0, 360));

      values = channels[0].map((c0, i) => new HSLA(channels[0][i], channels[1][i], channels[2][i], channels[3][i]));

      let shuffled = new Map(keys.reduce((acc, key, i) => [...acc, [key, values[i].toRGBA()]], []));

      for(let [path, value] of shuffled) {
        flat.set(path, value.hex());
      }

      //flat = [...shuffled.entries()];
      console.log('flat:', flat);

  let generated = ColorMap.generate(10, 3, prng);
      console.log('generated:', generated);

      const newObj = {};
      for(let [path, value] of flat) {
        deep.set(newObj, path, value);
      }
      filesystem.writeFile(basename + '.xml', toXML(newObj));
    } catch(err) {
      console.log('err:', err);
    }
}
main(...process.argv.slice(2));
