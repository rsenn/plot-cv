import Util from './lib/util.js';
import PortableFileSystem, { SEEK_SET, SEEK_END } from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import ObjectInspect from './lib/objectInspect.js';

let filesystem;
let tmpdir;
let buffer, buffer2;
let handle;
let data = 'TEST\nabcdefg\n123456789';
let data2;

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  await PortableFileSystem((fs) => (filesystem = fs));

  let files = filesystem.readdir('src').map((file) => `src/${file}`);
  let sources = files.filter((path) => /\.[ch]/.test(path));

  console.log('readdir', sources);
}

Util.callMain(main, true);
