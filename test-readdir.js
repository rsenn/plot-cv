import { DirIterator, ReadDirRecursive, RecursiveDirIterator } from './dir-helpers.js';
import extendGenerator from './quickjs/qjs-modules/lib/extendGenerator.js';
let tmpdir;
let buffer, buffer2;
let handle;
let data = 'TEST\nabcdefg\n123456789';
let data2;

extendGenerator();

function* Filter(gen, regEx = /.*/) {
  for(let item of gen) if(regEx.test(item)) yield item;
}

Object.assign(globalThis, { DirIterator, RecursiveDirIterator, ReadDirRecursive });

function main(...args) {
  console.log('readdir', [...Filter(ReadDirRecursive('.', 2), /\.[ch]$/)]);

  os.kill(process.pid, os.SIGUSR1);
}

main(...scriptArgs.slice(1));