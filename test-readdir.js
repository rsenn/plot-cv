import filesystem from 'fs';
import ObjectInspect from './lib/objectInspect.js';

let tmpdir;
let buffer, buffer2;
let handle;
let data = 'TEST\nabcdefg\n123456789';
let data2;

function* Filter(gen, regEx = /.*/) {
  for(let item of gen) if(regEx.test(item)) yield item;
}

function* ReadDirRecursive(dir, maxDepth = Infinity) {
  for(let file of filesystem.readdir(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;

    let entry = `${dir}/${file}`;
    let isDir = false;
    let st = filesystem.stat(entry);

    isDir = st && st.isDirectory();

    yield isDir ? entry + '/' : entry;

    if(maxDepth > 0 && isDir) yield* ReadDirRecursive(entry, maxDepth - 1);
  }
}

async function main(...args) {
  /*
  let files = filesystem.readdir('src').map(file => `src/${file}`);

  let sources = files.filter(path => /\.[ch]/.test(path));

*/

  console.log('readdir', [...Filter(ReadDirRecursive('.', 2), /\.[ch]$/)]);
}

main(...scriptArgs.slice(1));
