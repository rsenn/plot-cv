import * as os from 'os';
import * as std from 'std';
os.ttySetRaw(0);

let ab = new ArrayBuffer(1024);
let u8 = new Uint8Array(ab);
std.puts('\x1b[?1l');

for(;;) {
  let r = os.read(0, ab, 0, 1024);

  console.log('data:', r >= 0 ? ab.slice(0, r) : null);

  if(r == 1 && u8[0] == 3) break;
}