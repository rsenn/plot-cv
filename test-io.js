import Util from './lib/util.js';
import PortableFileSystem, { SEEK_SET, SEEK_END } from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import ObjectInspect from './lib/objectInspect.js';
import { Repeater } from './lib/repeater/repeater.js';
//import TinyTest, { run, assert, assertEquals } from './lib/tinyTest.js';

let filesystem;
let tmpdir;
let buffer, buffer2;
let handle;
let data = 'TEST\nabcdefg\n123456789';
let data2;

function FileReader(path, bufferSize = 1024) {
  let file = filesystem.open(path, 'r');
  let buf = filesystem.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    while((ret = filesystem.read(file, buf)) > 0) await push(buf.slice(0, ret));
    filesystem.close(file);
  });
}

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  await PortableFileSystem(fs => (filesystem = fs));

  for await (let data of await FileReader('test-io.js', 64)) console.log('data:', filesystem.bufferToString(data));

  return;
  console.log(Util.getMethodNames(filesystem)
      .map(n => `  'filesystem.${n}': null,`)
      .join('\n')
  );
}

Util.callMain(main, true);
