import Alea from './lib/alea.js';
import { HSLA, RGBA } from './lib/color.js';
import distanceChecker from './lib/color/distanceChecker.js';
import * as deep from './lib/deep.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { Iterator, IteratorForwarder } from './lib/iterator.js';
import { Path, toXML } from './lib/json.js';
import KolorWheel from './lib/KolorWheel.js';
import { isObject } from './lib/misc.js';
import * as path from './lib/path.js';
import tXml from './lib/tXml.js';
import { XPath } from './lib/xml.js';
import inspect from 'inspect';
let prng = new Alea().seed(Date.now());

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}

//TODO: Test with tmScheme (XML) and ColorMap

const push_back = (arr, ...items) => [...(arr || []), ...items];

const push_front = (arr, ...items) => [...items, ...(arr || [])];

const GeneratePalette = (counts = { h: 3, s: 3, l: 5 }, deltas = { h: 360, s: 100, l: 30 }, prng) => {
  let ret = [];
  let base = new HSLA(randInt(-deltas.h / 2, deltas.h / 2, prng), 100, 50).toRGBA();
  const makeRange = (count, delta) =>
    range(0, count - 1)
      .map(v => (v * delta) / (count - 1) - delta / 2)
      .map(Math.round);
  let ranges = {
    h: makeRange(counts.h, deltas.h),
    s: makeRange(counts.s, deltas.s),
    l: makeRange(counts.l, deltas.l)
  };
  //const numColors = ranges.reduce((acc, r) => acc * r.length, 1);
  //ranges.push(numColors);
  console.log('ranges:', ranges);
  console.log('ranges:', chunkArray(ranges.h, 3));
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
  return ret; //unique(ret, (a,b) => a.compareTo(b));
};

async function main(...args) {
  let colors, keys;

  let params = getOpt(
    {
      output: [true, null, 'o'],
      '@': 'input,'
    },
    args
  );

  console.log('main', params);
  if(params['@'].length == 0) params['@'] = ['/home/roman/.config/sublime-text-3/Packages/Babel/Next.tmTheme' /*  */];
  let filename = params['@'].shift();
  let basename = path.basename(filename, /\.[^.]+$/g);
  let outfile;

  outfile = params.output ?? basename + '.xml';
  if(outfile == filename) outfile = basename + '.out.xml';

  let name = path.basename(outfile, /\..*/g);

  let cmds = params['@'];
  let newObj = {};
  let xmlData;
  let hex2idx, idx2hue, idx2path;

  try {
    let xml = readXML(filename);
    let json = JSON.stringify(xml, null, '  ');
    let st;
    let { rdev, ino, mtime, atime } = (st = filesystem.stat(filename));
    console.log('stat:', inspect(st), Object.keys(st));
    //prng = new Alea().seed((ino * 256 + rdev) ^ mtime.valueOf());

    console.log('prng.uint32():', prng.uint32());
    let basename = path.basename(filename, /\.[^.]+$/);
    filesystem.writeFile(basename + '.json', json);
    xmlData = xml[0];
    newObj = deep.clone(xml[0]);
    let flat = deep.flatten(
      xml[0],
      new Map(),
      (v, p) => (typeof v != 'object' && p.indexOf('attributes') == -1) || (p.length && p.indexOf('attributes') == p.length - 1),
      (p, v) => [new Path(p), v]
    );
    colors = new Map([...Iterator.filter(flat, ([path, value]) => /^#[0-9A-Fa-f]*$/.test(value))].map(([path, value]) => [path, new RGBA(value)]));
    let it = new IteratorForwarder(colors.entries());
    let paths = Iterator.map(colors, ([path, value]) => [XPath.from(path, xml[0]), deep.get(xml[0], path), path]);
    let o = it.map(([path, value]) => {
      path = new Path(path);
      console.log('it.map', { path, value });
      const key = path.up(0);
      let prev = new Path([]),
        prevValue = {},
        list = [],
        numString = 0;
      let paths = [
        ...key.walk((p, i, abort, skip) => {
          let r;
          let value = deep.get(xml[0], p);
          const children = isObject(value) && value.children ? value.children : [];
          const text = typeof children[0] == 'string' ? children[0] : '';
          if(['Next', 'settings', 'scope', 'name', 'gutter'].indexOf(text) != -1 || /* text.startsWith('#') ||*/ typeof children[0] != 'string') {
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
      paths = paths.map(path => path.concat(['children', 0]));
      /* prettier-ignore */ return [ path, paths.map((p) => deep.get(xml[0], p)) .reverse() .join('/'), value ];
    });
    o = [...o].filter(([p, k, v]) => !/background/i.test(k));
    keys = new Map(o.map(([p, k, v]) => [p, k]));
    colors = new Map(o.map(([p, k, v]) => [p, v]));
    colors = new Map(
      [...colors.entries()]
        .map(([p, c]) => [p, p.up(2).prevSibling.down('children', 0), c])
        .map(([p, f, c]) => [p, f.apply(xml[0]), c])
        .filter(([p, f, c]) => !/(gutter|guide)/.test(f))
        .map(([p, f, c]) => [p, c])
    );
    let palette = new ColorMap(HSLA, colors);
    idx2path = [...palette.keys()];
    // console.log('idx2path:', idx2path );

    console.log('palette.getMinMax():', palette.getMinMax());

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
    //    console.log("colors:",colors);

    const hash = key => ((a, b) => a / b)(...lexOrder(key).reduce((acc, c) => [acc[0] * (10 + 26 + 26) + c, acc[1] * (10 + 26 + 26)], [0, 1]));

    console.log(`palette.getMinMax():`, palette.getMinMax());
    const mm = palette.getMinMax();

    if(cmds.length == 0) {
      //   cmds.unshift(['remap', 'h', 0, 360]);
      //  cmds.unshift(['remap', 's', 75, 100]);
      cmds.unshift(['normalize']);
      //cmds.unshift(['remap', 'l', 50, 60]);
    }
    const UpdatePalette = pal => {
      pal = pal || palette;
      // console.log('UpdatePalette:', pal);
      for(let [path, color] of pal) {
        color = typeof color == 'function' ? color() : color;
        //  color = color && color.toRGBA ? color.toRGBA() : color;
        // console.log('UpdatePalette:', newObj, path, inspect(color, { multiline: false }));
        let key = path.last;
        let parent = deep.get(xmlData, path.up());
        let oldValue = parent[key];
        // console.log('UpdatePalette:', {  oldValue,key,parent});
        let oldColor = RGBA.fromHex(oldValue).toHSLA();
        let rgbaColors = [oldColor, color].map(c => c.toRGBA());
        let distance = RGBA.distance(...rgbaColors);
        //if(distance > 0) console.log('UpdatePalette:', oldColor, ` -> #${idx2path.indexOf(path)}`, color, ` Δ = ${distance}`);
        //console.log('UpdatePalette:', { color, oldValue, oldColor });
        //let [obj, key] = path.bottom(newObj, true);
        parent[key] = color.hex();
        flat.set(path, color);
        if(newObj) deep.set(newObj, path, color.hex());
      }

      /*idx2path = [...palette.keys()];
        console.log(`idx2path`, idx2path);*/
    };

    let handlers = {
      shuffle(...seed) {
        let rng = new Alea(...seed);
        //if(seed) rng.mash(...seed);
        const keys = [...palette.keys()];
        let values = keys.reduce((acc, key) => [...acc, palette.get(key)], []);
        values = shuffle(values, rng);
        keys.forEach((key, i) => palette.set(key, values[i]));
      },
      remap(channel, start, end) {
        return palette.remapChannel(channel, remap(mm[channel], start, end));
      },
      set(...args) {
        let colors = '#' + args.join(',');
        console.log('colors:', colors);
        let re = /[^0-9a-fA-F]([0-9a-fA-F][0-9a-fA-F]+)/g;
        //  console.log("matchAll:",[...colors.matchAll(re)]);
        let result;
        let a = [];
        while((result = re.exec(colors))) {
          const r = [...result].slice(1);
          a.push('#' + r[0]);
        }
        let newColors = a.map(p => new RGBA(p));
        console.log('newColors:', newColors);
        for(let [path, color] of palette) {
          console.log(`path=`, path, ` color=`, color);
          let path2 = path.up(4);
          if(path2.last > 2) {
            path2 = path2.left(path2.last - 1);
            let obj = path2.apply(newObj, true);
            console.log(`path2=${path2} obj=`, obj);
          }
          const { value, index, distance } = RGBA.nearestColor(color.toRGBA(), newColors, distanceChecker);
          console.log(`value=`, value);
          palette.set(path, value.toHSLA());
        }
      },
      grayscale(amount = 1.0) {
        return palette.remapChannel('s', s => clamp(0, 100, 100 - (100 - s) * amount));
      },
      normalize() {
        return this.remap('l', 50, 75);
      },
      huerotate(by) {
        return palette.remapChannel('h', v => mod(v + by, 360));
      },
      lighten(by) {
        return palette.remapChannel('l', l => clamp(0, 100, l + by));
      },
      brighter(by) {
        return palette.remapChannel('l', l => clamp(0, 100, 100 - (100 - l) * by));
      },
      darker(by) {
        return palette.remapChannel('l', l => clamp(0, 100, l * by));
      },
      desaturate(by) {
        return palette.remapChannel('s', s => clamp(0, 100, s - s * by));
      },
      saturate(by) {
        return palette.remapChannel('s', s => clamp(0, 100, 100 - (100 - s) * by));
      },
      generate(...seed) {
        let rng = prng.clone();
        if(seed) rng.mash(...seed);
        let i = 0;
        let sz = 1 << Math.ceil(Math.log2(palette.size));
        let gcd = [3, 4, 6, 8, 12, 16, 32].map(n => greatestCommonDenominator(sz, n));
        let numHues = 8;
        let step = Math.ceil(Math.pow(sz, 1 / 2) / 1.85);
        console.log('sz:', sz);
        console.log('step:', step);
        let newPal = GeneratePalette({
          l: step,
          s: 3,
          h: numHues || Math.ceil(sz / step / 3)
        });
        console.log('palette.size:', palette.size);
        console.log('newPal.length:', newPal.length);
        newObj = {};
        for(let [path, value] of flat) {
          deep.set(newObj, path, value);
        }

        for(let item of palette) {
          let [path, color] = item;
          const key = keys.get(path);
          const hashes = key
            .split('/')
            .slice(0)
            .map(k => [k, k.split(/,? /g).map(k => k.split(/\./g).map(h => hash(h)))])
            .flat()
            .filter(i => typeof i != 'string')
            .flat();
          if(/(background)/i.test(key)) continue;
          prng();
          color = draw(newPal, 1, rng);
          if(false)
            if(true || !color) {
              let hues = [hashes[0][0] - 0.4, hashes[0][0] + 0.4];
              let la = [30, 70];
              color = HSLA.random([0, 360], [80, 100], [50, 60], [1, 1], rng);
            }
          color = color || new HSLA(0, 0, 0, 0);
          if(color.l < 60) {
            let rem = 60 - color.l;
            color.l += rem / 3;
          }
          if(Math.abs(100 - color.s) >= 10) {
            let rem = 100 - color.s;
            color.s = 100 - rem / 2;
          }
          const rgba = color.toRGBA();
          const hex = rgba.hex();
          palette.set(path, color);
          palette.remapChannel('l', remap(mm.l, [50, 75]));
        }
        return palette;
      },
      invert() {
        this.remap((c, k) => {
          let rgba = c.toRGBA();
          return rgba.invert();
        });
        console.log('invert');
      },
      dump() {
        console.info('palette:', palette);
      },
      reducehues() {
        let changed = new Set();
        let i = 0;
        for(let [path, color] of palette) {
          const prevColor = prevPalette.get(path);
          if(!prevColor.equals(color)) changed.add(i);
          i++;
        }

        const getHSLA = idx_or_hex => colors[typeof idx_or_hex == 'string' ? hex2idx[idx_or_hex] : idx_or_hex] || palette.get(idx2path[idx_or_hex]);
        console.log('changed ', [...changed].join(', '));
        colors = [...palette.entries()].map(([path, color], idx) => color);
        /* prettier-ignore */ console.log('colors = ', inspect(colors.map((c) => [...c]), { multiline: false, colors: false }));
        // let idx2hex = colors.map( (color,i) =>  color.hex());
        hex2idx = Object.fromEntries(colors.map((color, i) => [color.hex(), i]));
        console.log(`hex2idx`, hex2idx);

        idx2hue = colors.map((color, i) => color.h);
        console.log(`idx2hue`, idx2hue);
        /* prettier-ignore */ const getIds4Hue = (hue) => idx2hue.map((h, i) => [i, h]) .filter(([i, h]) => h == hue).map(([i, h]) => i);
        /* prettier-ignore */ let hues = histogram(colors, (v, i) => [v.h, v], new Map(), () => new Set(), (v, i) => [v.h, i]);
        // console.log(`hues`, hues);

        let hueIds = [...hues.entries()]
          .map(([hue, colorCodes], idx) => [+hue, /* [...colorCodes].join(',') || */ colorCodes || new Set([...colorCodes].map(hex => getHSLA(hex)))])
          .map(([hue, ids]) => [hue, [...ids] + '']);
        console.log(`hueIds`, hueIds);

        let hueCounts = /*new Map*/ hueIds.map(([hue, ids], i) => [i, [hue, ids.split(',').map(id => +id)]]).map(([idx, [hue, ids]]) => [idx, ids.length]);

        console.log(`hueCounts`, hueCounts);

        let hueData = hueIds
          .map(([hue, ids], idx) => [idx, hue, ids /*.split(',').map((v) => +v)*/])
          .reduce((acc, [idx, hue, ids = getIds4Hue(hue)]) => [...acc, [idx, hue, ids.split(',').map(p => +p)]], []);
        //console.log(`hueData`, hueData);

        //console.log(`zhistogram`, histogram(colors, (c, i) => [c.h, i], new Map(), () => new Set(), i => [colors[i].h, i] ) ); //new Map(Object.entries(idx2hue).map(([idx,hue]) => [+idx,+hue])));
        console.log(`hueCounts`, hueCounts);

        const removeHues = hueCounts.filter(([idx, count]) => count < 2);
        console.log(`removeHues`, removeHues);
        const removeIds = removeHues.map(([idx]) => idx).sort((a, b) => b - a);
        console.log(`removeIds.reverse()`, /*histogram*/ [...removeIds].reverse());

        //console.log(`removeIds`, removeIds);
        const colorsRGBA = [...palette.entries()].map(([path, c], i) => [i, c.toRGBA()]); //[...palette.entries()].map(([path,c], i) => [i, c.toRGBA()]).filter(([i, c]) => removeIds.indexOf(i) == -1);
        //  console.log(`[...palette.entries()].:`, );
        console.log(`colorsRGBA:`, colorsRGBA.length);
        let rgba = colorsRGBA.filter(([i, c]) => removeIds.indexOf(i) == -1);
        let newPalette = [...new Map(rgba).values()];

        const idx2hsla = colorsRGBA.map(([i]) => palette.get(idx2path[i]));
        //console.log(`rgba:`, newPalette);

        let rem = removeIds
          .map(id => [id, colors[id].toRGBA()])
          .map(([i, c]) => {
            let path = idx2path[i];
            let oldColor = /*new RGBA*/ deep.get(newObj, path);
            //    console.log('oldColor', { oldColor });
            const r = RGBA.nearestColor(oldColor, newPalette, distanceChecker);
            //    console.log('r', r , {oldColor});

            if(isObject(r) && r.index !== undefined) {
              const { value, index, distance } = r;
              // console.log('i=', i, 'r', inspect({ value, index, distance }, { multiline: false }));

              const nearColor = getHSLA(index) || idx2hsla[index];
              let [removedColor] = colors.splice(i, 1, nearColor);
              let newColor = nearColor.hex();
              deep.set(newObj, path, newColor);
              let [fromColor, toColor] = [removedColor.hex(), newColor].map(c => new RGBA(c).toHSLA());

              console.info(`changed #${i}`, fromColor, ` -> #${index} `, toColor, ` Δ ${roundTo(distance, 0.1, 2)}`);
              return removedColor;
            }
          });
        let modifyIds = colors.map((c, idx) => [idx2path[idx], c]).map(([path, color]) => [path, color, new RGBA(deep.get(xml[0], path)).toHSLA()]);
      }
    };
    let prevPalette = new Map(palette.entries());
    cmds = cmds.map(cmdStr =>
      Array.isArray(cmdStr)
        ? cmdStr
        : cmdStr
            .split(/[^-A-Za-z0-9\.\/]/g)
            .map(p => (!isNaN(+p) ? +p : p))
            .map(p => (typeof p == 'string' ? p.toLowerCase() : p))
    );

    cmds.forEach(cmd => {
      cmd = cmd.map(p => (/^.?time?$/i.test(p) ? +mtime : /^now$/i.test(p) ? Date.now() : /^p?r?a?n[dg]o?m?$/i.test(p) ? prng.uint32() : p));
      console.log('Command ', cmd);
      //  putStack();
      let ret = handlers[cmd[0]](...cmd.slice(1));

      if(ret) palette = ret;

      UpdatePalette();
      // console.log('New Palette ', ret);
    });

    // Change UUID
    const mkuuid = () => [8, 4, 4, 4, 12].map(n => randStr(n, '0123456789abcdef')).join('-');

    const change = (key, value) => {
      let ptr = new Path(deep.find(newObj, (v, p) => v.children && v.children[0] == key)?.path);

      if(ptr) {
        let valuePath = ptr.nextSibling.down('children', 0);
        deep.set(newObj, valuePath, value);
      }
    };

    change('uuid', mkuuid());
    change('name', name);
    change('background', '#000000');

    outfile = outfile || basename + '.xml';
    filesystem.writeFile(outfile, toXML(newObj));
  } catch(err) {
    let st = stack(err.stack);
    // console.log(err.message, '\n', st.toString()); //st.map(f =>  inspect(f)));
    throw err;
  }
}

main(...scriptArgs.slice(1));