import { readAllSync } from 'fs';
import { toString } from 'util';
import { Spawn } from './os-helpers.js';
import { ReadFile } from './test-readfile.js';
import { Console } from 'console';
import { _get_osfhandle } from 'misc';
globalThis.console = new Console({ inspectOptions: { colors: true, compact: false } });

function ReadProcess(...args) {
  let child = Spawn(args.shift(), args, {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });
  let { stdout } = child;
  console.log('stdout', stdout);
  console.log('child.stdout', child.stdout);
  console.log('stdout.fileno()', stdout.fileno());
  let ab = new ArrayBuffer(1024);
  let u32 = new Uint32Array(2);
  let hnd = _get_osfhandle(stdout.fileno());
  console.log('hnd', hnd);

  for(;;) {
    //let r = read(stdout.fileno(), ab, 1024);

    let r = ReadFile(hnd, ab, 1024, u32.buffer, 0);
    //let r = stdout.read(ab, 0, 1024);
    console.log('r', r);

    console.log('u32', u32);
    if(u32[0] == 0) break;

    console.log('output', toString(ab.slice(0, u32[0])));
  }
  console.log('done!');

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