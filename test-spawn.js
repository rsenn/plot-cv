import { Spawn } from './io-helpers.js';
import { keys, quote } from 'util';
import { readAll } from 'fs';

async function ReadProcess(...args) {
  let child = Spawn(args.shift(), args, {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });
  let data = await readAll(child.stdio[1]);

  child.wait();
  return data;
}



console.log('data',  await ReadProcess('gcc', '-M', '-I.', 'sigval.c'));

//console.log('child', child);
//console.log('child', keys(child).reduce((acc, k) => ({ ...acc, [k]: child[k] }), {}));
//console.log('child', console.config({ customInspect: false }), child);
//
//console.log('data', quote(data));
//console.log('child.wait() =', child.wait());
