import Util from './lib/util.js';
import PortableSpawn from './lib/spawn.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import inspect from './lib/objectInspect.js';
import { mmap, munmap, PROT_READ, PROT_WRITE, MAP_PRIVATE, MAP_SHARED, MAP_ANONYMOUS } from 'mmap.so';

async function main(...args) {
  await ConsoleSetup({ breakLength: 80, depth: Infinity });
  await PortableFileSystem();
  await PortableSpawn();

  let fd = filesystem.open(Util.getArgv()[1], filesystem.O_RDONLY);
  let size = filesystem.size(fd);
  console.log('fd', fd);
  console.log('size', size);
  console.log('seek()', filesystem.seek(0, 10, filesystem.SEEK_SET));
  console.log('typeof 1n', typeof 1n);

  let map = mmap(0, size, PROT_READ, MAP_PRIVATE, fd, 0);
  console.log('map', Util.getMembers(map, Infinity, 0));
  console.log('map', map + '');

  console.log('map =', ArrayBufToString(map));

  munmap(map);
  console.log('map', Util.getMembers(map, Infinity, 0));
}

Util.callMain(main, true);

function ArrayBufToString(buf, offset, length) {
  let arr = new Uint8Array(buf, offset, length);
  return arr.reduce((s, code) => s + String.fromCodePoint(code), '');
}
