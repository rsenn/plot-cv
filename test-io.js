import { Repeater } from './lib/repeater/repeater.js';
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
  for await(let data of await FileReader('test-io.js', 64)) console.log('data:', filesystem.bufferToString(data));

  return;
  console.log(
    getMethodNames(filesystem)
      .map(n => `  'filesystem.${n}': null,`)
      .join('\n')
  );
}

main(...scriptArgs.slice(1));