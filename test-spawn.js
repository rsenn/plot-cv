import { escape } from './lib/misc.js';
import PortableSpawn from './lib/spawn.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import inspect from './lib/objectInspect.js';

//prettier-ignore
let filesystem, spawn;

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 80, depth: Infinity });
  await PortableFileSystem(fs => (filesystem = fs));
  spawn = await PortableSpawn();

  console.log('spawn:', spawn);

  let child = spawn(['ls', '-la' /*, 'CMakeLists.txt'*/]);

  console.log('test:', 234);
  console.log('child:', inspect(child));
  console.log('child.wait():', await child.wait());
  const bufSize = 100;
  let ab = new ArrayBuffer(bufSize);
  let r;

  while((r = await child.stdout.read(ab, 0, bufSize))) {
    if(!(r > 0 && r == bufSize)) console.log('r:', r);
    console.log('data:', escape(ArrayBufToString(ab, 0, r)));
  }
}

main(...scriptArgs.slice(1));

function ArrayBufToString(buf, offset, length) {
  let arr = new Uint8Array(buf, offset, length);
  return arr.reduce((s, code) => s + String.fromCodePoint(code), '');
}
