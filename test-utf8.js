import parse from './lib/xml/parse.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import tXml from './lib/tXml.js';
import { toXML } from './lib/json.js';

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

async function main(...args) {
  await ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  await PortableFileSystem();

  let data = filesystem.readFile(args[0] ?? 'utf8.txt', null);
  console.log('data:', data);

  let bytes = new Uint8Array(data);

  let result = [...bytes].reduce((acc, c) => {
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
    }, [[], 0, 0]
  )[0];

  console.log('result:', result);
  let g = BytesToUTF8(bytes);
  console.log('g:', typeof g);
  console.log('g:', g.next);
  console.log('g:', g);

  console.log('BytesToUTF8:', CodePointsToString(g));
}

Util.callMain(main, true);