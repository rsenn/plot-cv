import { Spawn } from './io-helpers.js';
import { keys, quote, toString } from 'util';
import { readAll,readSync, readAllSync } from 'fs';
import { Console } from 'console';
import { read } from 'os';
import { ReadFile } from './test-readfile.js';
import { _get_osfhandle } from 'misc';

globalThis.console = new Console({ inspectOptions: {  colors: true, compact: false } });

function ReadProcess(...args) {
  let child = Spawn(args.shift(), args, {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });
  let {stdout}=child;
  console.log('stdout', stdout);
    console.log('child.stdout', child.stdout);
  console.log('stdout.fileno()', stdout.fileno());
let ab = new ArrayBuffer(1024);
let u32 = new Uint32Array(2);
let hnd = _get_osfhandle(stdout.fileno());
console.log('hnd', hnd);

let r = ReadFile(hnd, ab, 1024, u32.buffer, 0);
console.log('r', r);
console.log('u32', u32);
console.log('output', toString(ab.slice(0,u32[0])));

  child.wait();

  
  let data = readAllSync(stdout.fileno());
console.log('data', data);

  return data;
}

console.log('data', await ReadProcess('gcc', '-M', '-I.', 'sigval.c'));

//console.log('child', child);
//console.log('child', keys(child).reduce((acc, k) => ({ ...acc, [k]: child[k] }), {}));
//console.log('child', console.config({ customInspect: false }), child);
//
//console.log('data', quote(data));
//console.log('child.wait() =', child.wait());
