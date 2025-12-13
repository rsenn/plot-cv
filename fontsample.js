import { WriteFile } from './io-helpers.js';
import { Console } from 'console';
function main() {
  globalThis.console = new Console(process.stdout, {
    inspectOptions: {
      colors: true,
      depth: 2,
      maxArrayLength: 400,
      breakLength: 100,
      compact: 2
    }
  });

  let a = new Uint8Array(256 + 16);
  let s = '';
  let j = 0;

  for(let i = 0; i < 256; i++) {
    let code = i < 32 || (i >= 0x7f && i < 0xa0) ? 0x20 : i;
    a[j++] = code;

    s += String.fromCodePoint(code);

    if((i & 0x0f) == 0x0f) {
      a[j++] = 0x0a;
      s += '\n';
    }
  }
  console.log('a', a);
  WriteFile('output.txt', s);
  // WriteFile('output.txt', a.buffer);
}

main();