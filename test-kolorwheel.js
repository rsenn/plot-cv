import Alea from './lib/alea.js';
import { HSLA, RGBA } from './lib/color.js';
import KolorWheel from './lib/KolorWheel.js';
import { unique } from './lib/misc.js';

const prng = new Alea(Date.now());

const layerToColorIndex = {
  Top: 4,
  Bottom: 1,
  Pads: 2,
  Vias: 2,
  Unrouted: 6,
  Dimension: 24,
  tPlace: 16,
  bPlace: 14,
  tOrigins: 15,
  bOrigins: 15,
  tNames: 7,
  bNames: 7,
  tValues: 7,
  bValues: 7,
  tStop: 7,
  bStop: 7,
  tCream: 7,
  bCream: 7,
  tFinish: 6,
  bFinish: 6,
  tGlue: 7,
  bGlue: 7,
  tTest: 7,
  bTest: 7,
  tKeepout: 4,
  bKeepout: 1,
  tRestrict: 4,
  bRestrict: 1,
  vRestrict: 2,
  Drills: 7,
  Holes: 7,
  Milling: 3,
  Measures: 7,
  Document: 7,
  Reference: 7,
  tDocu: 6,
  bDocu: 7,
  Nets: 2,
  Busses: 1,
  Pins: 2,
  Symbols: 4,
  Names: 7,
  Values: 7,
  Route2: 1,
  Route3: 4,
  Route4: 1,
  Route5: 4,
  Route6: 1,
  Route7: 4,
  Route8: 1,
  Route9: 4,
  Route10: 1,
  Route11: 4,
  Route12: 1,
  Route13: 4,
  Route14: 1,
  Route15: 4,
  Modules: 5,
  Info: 7,
  Guide: 6,
  SpiceOrder: 7,
  trash: 7
};
const layerColors = {
  Top: 'rgb(0,23,185)',
  Route2: 'rgb(3,0,5)',
  Route3: 'rgb(0,23,185)',
  Route4: 'rgb(3,0,5)',
  Route5: 'rgb(0,23,185)',
  Route6: 'rgb(3,0,5)',
  Route7: 'rgb(0,23,185)',
  Route8: 'rgb(3,0,5)',
  Route9: 'rgb(0,23,185)',
  Route10: 'rgb(3,0,5)',
  Route11: 'rgb(0,23,185)',
  Route12: 'rgb(3,0,5)',
  Route13: 'rgb(0,23,185)',
  Route14: 'rgb(3,0,5)',
  Route15: 'rgb(0,23,185)',
  Bottom: 'rgb(252,245,38)',
  Pads: 'hsl(131,100%,24.7%)',
  Vias: 'rgb(252,245,38)',
  Unrouted: 'rgb(62,46,25)',
  Dimension: 'rgb(175,175,175)',
  tPlace: 'hsl(20,100%,60%)',
  bPlace: 'rgb(255,180,83)',
  tOrigins: 'rgb(178,27,0)',
  bOrigins: 'rgb(178,27,0)',
  tNames: 'rgb(255,38,0)',
  bNames: 'rgb(189,133,64)',
  tValues: 'rgb(255,38,0)',
  bValues: 'rgb(189,133,64)',
  tStop: 'rgb(189,133,64)',
  bStop: 'rgb(189,133,64)',
  tCream: 'rgb(189,133,64)',
  bCream: 'rgb(189,133,64)',
  tFinish: 'rgb(62,46,25)',
  bFinish: 'rgb(62,46,25)',
  tGlue: 'rgb(189,133,64)',
  bGlue: 'rgb(189,133,64)',
  tTest: 'rgb(189,133,64)',
  bTest: 'rgb(189,133,64)',
  tKeepout: 'rgb(0,23,185)',
  bKeepout: 'rgb(3,0,5)',
  tRestrict: 'rgb(0,23,185)',
  bRestrict: 'rgb(3,0,5)',
  vRestrict: 'rgb(252,245,38)',
  Drills: 'rgb(189,133,64)',
  Holes: 'rgb(0,255,255)',
  Milling: 'rgb(0,126,24)',
  Measures: 'rgb(189,133,64)',
  Document: 'rgb(255,180,83)',
  Reference: 'rgb(189,133,64)',
  tDocu: 'rgb(255,180,83)',
  bDocu: 'rgb(255,180,83)',
  Modules: 'rgb(79,9,0)',
  Nets: 'rgb(252,245,38)',
  Busses: 'rgb(3,0,5)',
  Pins: 'rgb(252,245,38)',
  Symbols: 'rgb(0,23,185)',
  Names: 'rgb(189,133,64)',
  Values: 'rgb(189,133,64)',
  Info: 'rgb(189,133,64)',
  Guide: 'rgb(62,46,25)',
  SpiceOrder: 'rgb(189,133,64)',
  trash: 'rgb(189,133,64)'
};

const findAllKeys = color => {
  let keys = [];
  for(let name in layerColors) {
    const c = layerColors[name];
    if(c == color) keys.push(name);
  }
  return keys;
};

for(let name in layerColors) {
  const c = RGBA.fromString(layerColors[name]).hex();
  layerColors[name] = c;
}

const allColors = unique(Object.values(layerColors));
//console.log('allColors: ' + allColors);

let keyList = [];

for(let color of allColors) {
  let keys = findAllKeys(color);
  keyList.push(keys);
  //
}

//console.log("keyList:", keyList);

const GeneratePalette = numColors => {
  let ret = [];
  let base = new HSLA(randInt(0, 360, prng), 100, 50).toRGBA();
  let offsets = range(1, numColors).reduce((acc, i) => [...acc, ((acc[acc.length - 1] || 0) + randInt(20, 80)) % 360], []);
  offsets = offsets.sort((a, b) => a - b);
  //offsets = shuffle(offsets, prng);
  //console.log('offsets:', offsets);

  new KolorWheel(base.hex()).rel(offsets, 0, 0).each(function () {
    const hex = this.getHex();
    const rgba = new RGBA(hex);
    const hsla = rgba.toHSLA();
    //console.log(hex, rgba.toString(), hsla.toString());
    ret.push(hsla);
  });
  return ret;
};
function* Gradient(start, end, steps = 10) {
  var base = new KolorWheel(start);
  var target = base.abs(end, steps);

  for(var n = 0; n < steps; n++) yield target.get(n).getHex();
}

// for gradient
async function main(...args) {
  const palette = GeneratePalette(5);
  console.log('palette:', palette);
  //console.log('keyList.length:', keyList.length);

  let s = '';
  for(let i = 0; i < keyList.length; i++) {
    //console.log(`keys[${i}]:`, keyList[i]);

    for(let key of keyList[i]) {
      if(s != '') s += ', ';
      s += `${key}: palette[${i}]`;
    }
  }
  console.log('const palette = [ ' + palette.map(c => c.toSource()).join(', ') + ' ];\n renderer.colors = {' + s + '};');
  let colors = [...Gradient('#9ceaff', '#000088', 7)].map(c => new RGBA(c));

  colors = (function () {
    let ret = [];
    new KolorWheel([180, 100, 50]).rel(60, 0, 0, 7).each(function () {
      ret.push(new RGBA(this.getHex()));
    });
    return ret;
  })();

  console.log(colors);
  console.log(colors.map(c => c.toHSLA()));
  console.log(
    colors
      .map(c => c.hex())
      .map(c => `"${c}"`)
      .join(',\n')
  );
  console.log(new RGBA(0, 0, 255).toHSLA());
  console.log(new RGBA('#006dcf').toHSLA());
  console.log();
  let rainbow = [];
  for(let h of range(0, 300, 300 / 100)) rainbow.push(new HSLA(h, 100, 60));

  console.log(rainbow);
  console.log(
    chunkArray(
      rainbow.map(c => c.hex()).map(c => `"${c}"`),
      8
    )
      .map(a => a.join(', '))
      .join('\n')
  );
}

main(...scriptArgs.slice(1));