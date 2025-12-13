import { ReadFile } from './io-helpers.js';
function* BytesToUTF8(bytes) {
  let state = 0,
    val = 0;
  for(const c of bytes) {
    if(state !== 0 && c >= 0x80 && c < 0xc0) {
      val = (val << 6) | (c & 0x3f);
      state--;
      if(state === 0) yield val;
    } else if(c >= 0xc0 && c < 0xf8) {
      state = 1 + (c >= 0xe0) + (c >= 0xf0);
      val = c & ((1 << (6 - state)) - 1);
    } else {
      state = 0;
      yield c;
    }
  }
}

function* CodePointsToChars(codePoints) {
  for(let c of codePoints) yield String.fromCodePoint(c);
}

function CodePointsToString(codePoints) {
  let s = '';
  for(let c of codePoints) s += String.fromCodePoint(c);
  return s;
}

function main(...args) {
  let data = ReadFile(args[0] ?? 'utf8.txt', null);
  console.log('data:', data);

  let bytes = new Uint8Array(data);

  let result = [...bytes].reduce(
    (acc, c) => {
      let [out, state, val] = acc;
      if(state !== 0 && c >= 0x80 && c < 0xc0) {
        val = (val << 6) | (c & 0x3f);
        state--;
        if(state === 0) {
          out.push(val);
        }
      } else if(c >= 0xc0 && c < 0xf8) {
        state = 1 + (c >= 0xe0) + (c >= 0xf0);
        val = c & ((1 << (6 - state)) - 1);
      } else {
        state = 0;
        out.push(c);
      }
      return [out, state, val];
    },
    [[], 0, 0]
  )[0];

  console.log('result:', result);
  let g = BytesToUTF8(bytes);
  console.log('g:', typeof g);
  console.log('g:', g.next);
  console.log('g:', g);

  console.log('BytesToUTF8:', CodePointsToString(g));
}

main(...scriptArgs.slice(1));