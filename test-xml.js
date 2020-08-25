import Util from './lib/util.js';
import ConsoleSetup from './consoleSetup.js';
import { XPath } from './lib/xml.js';
import deep from './lib/deep.js';
import tXml from './lib/tXml.js';
import { RGBA, HSLA } from './lib/color.js';
import { Iterator, IteratorForwarder } from './lib/iterator.js';
import toSource from './lib/tosource.js';
import { toXML, Path } from './lib/json.js';
import { ColorMap } from './lib/draw/colorMap.js';
import Alea from './lib/alea.js';
import { Functional } from './lib/functional.js';
import KolorWheel from './lib/KolorWheel.js';

let filesystem;

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

const GeneratePalette = (counts = { h: 3, s: 2, l: 5 }, deltas = { h: 360, s: 100, l: 30 }, prng) => {
  let ret = [];
  let base = new HSLA(Util.randInt(-deltas.h / 2, deltas.h / 2, prng), 100, 50).toRGBA();

  const makeRange = (count, delta) =>
    Util.range(0, count - 1)
      .map((v) => (v * delta) / (count - 1) - delta / 2)
      .map(Math.round);

  let ranges = { h: makeRange(counts.h, deltas.h), s: makeRange(counts.s, deltas.s), l: makeRange(counts.l, deltas.l) };
  //const numColors = ranges.reduce((acc, r) => acc * r.length, 1);

  //ranges.push(numColors);
  console.log('ranges:', ranges);

  new KolorWheel(base.hex())
    .rel(ranges.h, 0, 0)
    .rel(0, ranges.s, 0)
    .rel(0, 0, ranges.l)
    .each(function () {
      const hex = this.getHex();
      const rgba = new RGBA(hex);
      const hsla = rgba.toHSLA();
      //console.log(hex, rgba.toString(), hsla.toString());
      ret.push(hsla);
    });
  //ret = ret.sort((a,b) => a.compareTo(b));
  //ret=ret.filter((hsla,i,arr) => arr.findIndex(item => item.compareTo(hsla) == 0) == i);
  return ret; //Util.unique(ret, (a,b) => a.compareTo(b));
};

async function main(...args) {
  await ConsoleSetup();
  let colors, keys;
  filesystem = await PortableFileSystem();

  //console.log('main', args);
  if(args.length == 0) args = ['/home/roman/.config/sublime-text-3/Packages/Babel/Next.tmTheme' /*  */];
  let [filename, outfile, ...cmds] = args;

  try {
    let xml = readXML(filename);
    let json = JSON.stringify(xml, null, '  ');

    let { rdev, ino, mtime, atime } = filesystem.stat(filename);

    const prng = new Alea().seed((ino * 256 + rdev) ^ mtime.valueOf() ^ Date.now());

    console.log('prng.uint32():', prng.uint32());
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
    //console.log('colors:', colors); //[...colors].map(([path,value ]) => [path, value]));
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
      paths = paths.map((path) => path.concat(['children', 0]));
      return [
        path,
        paths
          .map((p) => deep.get(xml[0], p))
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
    console.log('palette.getMinMax():', palette.getMinMax());

    const lexOrder = (key) => {
      let i = 0;
      let r = [];
      for(i = 0; i < 10; i++) {
        const c = i < key.length ? key.charCodeAt(i) : 0;
        const x = c >= 0x30 && c <= 0x39 ? c - 0x30 : c >= 0x41 && c <= 0x5a ? c - 0x41 + 10 : c >= 0x61 && c <= 0x7a ? c - 0x61 + 36 : 0;
        r.push(x);
      }
      return r;
    };
    const hash = (key) => ((a, b) => a / b)(...lexOrder(key).reduce((acc, c) => [acc[0] * (10 + 26 + 26) + c, acc[1] * (10 + 26 + 26)], [0, 1]));

    let i = 0;
    let sz = 1 << Math.ceil(Math.log2(palette.size));

    let gcd = [3, 4, 6, 8, 12, 16, 32].map((n) => Util.greatestCommonDenominator(sz, n));
    let numHues = 6;
    let step = Math.ceil(Math.pow(sz, 1 / 2) / 1.85);
    console.log('sz:', sz);
    console.log('step:', step);
    let newPal = GeneratePalette({ l: step, s: 3, h: Math.ceil(sz / step / 3) });
    //console.log('newPal:', newPal);

    console.log('palette.size:', palette.size);
    console.log('newPal.length:', newPal.length);
    /*const drawColor = Util.draw(newPal, prng);
      console.log('drawColor:', drawColor + '');*/
    /*   console.log('prng.int32():', prng.int32());
      console.log('prng.uint32():', prng.uint32());
*/
    const newObj = {};
    for(let [path, value] of flat) {
      deep.set(newObj, path, value);
    }

    for(let item of palette) {
      let [path, color] = item;
      const key = keys.get(path);
      const hashes = key
        .split(/\//g)
        .slice(0)
        .map((k) => [k, k.split(/,? /g).map((k) => k.split(/\./g).map((h) => hash(h)))])
        .flat()
        .filter((i) => typeof i != 'string')
        .flat();
      //console.log("hash:", hashes);
      //console.log("color1:", color);

      if(/(background)/i.test(key)) continue;
      //if(/(activeGuide|bracketsBackground|bracketsOptions|caret|guide|gutter|invisibles|lineHighlight|multiEditHighlight|searchHighlight|selection|stackGuide)/i.test(key)) continue;
      //console.log(`palette[${i++}] =`, key, color);
      prng();
      color = Util.draw(newPal, 1, prng); //||

      if(true || !color) {
        let hues = [hashes[0][0] - 0.4, hashes[0][0] + 0.4];
        let la = [30, 70];
        /*color = HSLA.random([hues[0] * 360, hues[1] * 360], [25,75], la || [luminances[0] * 255, luminances[1] * 255], [255, 255], prng);
        let luminances = [hashes[0][1] || 0.4, hashes[0][1] || 0.8];*/
        color = HSLA.random([0, 360], [50, 100], [50, 85], [1, 1], prng);
      }

      console.log('color2:', color);
      const rgba = color.toRGBA();
      const hex = rgba.hex();
      console.log('deep.set:', { newObj, path, hex });

      palette.set(path, color);
    }
    //  console.log(`palette.getChannel('h'):`, palette.getChannel('h'));
    console.log(`palette.getMinMax():`, palette.getMinMax());
    const mm = palette.getMinMax();

    if(cmds.length == 0) {
      cmds.unshift(['remap', 'h', 0, 360]);
      cmds.unshift(['remap', 's', 50, 100]);
      cmds.unshift(['remap', 'l', 25, 75]);
    }

    cmds = cmds.map((cmdStr) => (Util.isArray(cmdStr) ? cmdStr : cmdStr.split(/[^-A-Za-z0-9\.\/]/g)));

    let handlers = {
      shuffle() {
        const keys = [...palette.keys()];
        let values = keys.reduce((acc, key) => [...acc, palette.get(key)], []);
        values = Util.shuffle(values, prng);

        keys.forEach((key, i) => palette.set(key, values[i]));
      },
      remap(channel, start, end) {
        const mapper = Util.remap(mm[channel], start, end);
        return palette.remapChannel(channel, mapper);
      },
      grayscale() {
        //console.log('grayscale');
        return this.remap('s', 0, 0);
      },
      invert() {
        this.remap((c, k) => {
          let rgba = c.toRGBA();
          return rgba.invert();
        });

        console.log('invert');
      }
    };
    console.log('cmds:', cmds);

    cmds.forEach((cmd) => handlers[cmd[0]](...cmd.slice(1)));

    console.log('palette', palette);

    /*
    palette.remapChannel('h', Util.remap(mm.h, [0, 360]));
    palette.remapChannel('l', Util.remap(mm.l, [25, 75]));
    palette.remapChannel('s', Util.remap(mm.s, [50, 100]));
*/

    for(let [path, color] of palette.entries()) {
      //console.log("palette.entries()",{path,color});

      color = color && color.toRGBA ? color.toRGBA() : color;
      let [obj, key] = path.bottom(newObj);

      obj[key] = color.hex();

      flat.set(path, color);
      deep.set(newObj, path, color.hex());
    }
    //console.log('flat:', [...flat.entries()].filter(([path,value]) => value instanceof RGBA));

    // console.log('newObj:', toXML(newObj));
    /*
      keys = [...palette.keys()];
      let values = [...palette.values()];
      let channels = values.reduce((a, c) => [a[0].concat(c.h), a[1].concat(c.s), a[2].concat(c.l), a[3].concat(c.a)], [[], [], [], []]);
          //Util.shuffle(channels[0], prng);
      channels[0] = channels[0].map((c, i) => Util.randInt(0, 360));
      values = channels[0].map((c0, i) => new HSLA(channels[0][i], channels[1][i], channels[2][i], channels[3][i]));
      let shuffled = new Map(keys.reduce((acc, key, i) => [...acc, [key, values[i].toRGBA()]], []));
      for(let [path, value] of shuffled) {
        flat.set(path, value.hex());
      }*/

    //flat = [...shuffled.entries()];

    //let generated = ColorMap.generate(10, 3, prng);
    //console.log('generated:', generated);

    outfile = outfile || basename + '.xml';
    filesystem.writeFile(outfile, toXML(newObj));

    let c = Functional.curry(function (a, b, c) {
      return a * b * c;
    });
    let arity = Functional.arityof(c);
    //console.log('arity:', arity, c(2)(3)(4));
    Functional.compose(Functional.trim, Functional.split(/\//g))('test/blah');
  } catch(err) {
    console.log('err:', err);
  }
}
main(...Util.getArgs());
