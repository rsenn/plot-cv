import { RGBA } from './lib/color.js';
import { range } from './lib/misc.js';
function extendBits(arr, bits) {
  const mask = (1 << bits) - 1;
  let r = new arr.constructor(arr.length);
  for(let i = 0; i < arr.length; i++) {
    let n = 0;
    for(let j = 0; j < 8; j++) {
      const bit = 1 << j;
      const v = !!(arr[i] & bit);
      n |= (mask * v) << (j * bits);
    }
    r[i] = n;
  }
  return r;
}

function andBinary(arr, value, n, s = 0) {
  n = n || arr.length;
  for(let i = s; i < s + n; i++) {
    arr[i] &= value;
  }
}

function orBinary(arr, value, n, s = 0) {
  n = n || arr.length;
  for(let i = s; i < s + n; i++) {
    arr[i] |= value;
  }
}

function printBinary(arr, base = 2) {
  const prefix = base == 2 ? '0b' : base == '16' ? '0x' : base == 8 ? '0o' : '';
  const pad = base == 2 ? 24 : base == '16' ? 6 : base == 8 ? '0o' : 9;
  let i = 0;
  for(let num of arr) {
    console.log((i++ + '').padStart(3, ' ') + ': ' + prefix + ('0'.repeat(pad) + Math.abs(num).toString(base)).slice(-pad));
  }
}

let arr = new Uint32Array(range(0, 15));
let bits = extendBits(arr, 8);

andBinary(bits, 0x7f7f7f, 8);
orBinary(bits, 0x4b4b4b, 8, 8);

printBinary(bits, 16);
console.log('bits:', bits);
let colors = [...bits].map(num => `#${('000000' + (+num).toString(16)).slice(-6)}`).map(c => new RGBA(c));

console.log('colors:', colors);
console.log('colors:\n', colors.map(c => c.toSource()).join(',\n'));
let palette = [new RGBA(0x4b, 0xff, 0x4b), new RGBA(0x4b, 0xff, 0xff), new RGBA(0xff, 0x4b, 0x4b), new RGBA(0xff, 0xff, 0x4b)];
console.log('palette:', palette);
console.log('palette:', palette.map(color => color.hex()).join(','));