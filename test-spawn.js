import { Spawn } from './io-helpers.js';
import { keys, quote } from 'util';
import { readAllSync } from 'fs';

let child = Spawn('readlink', ['-f', `/proc/${process.pid}/exe`], {
  block: false,
  stdio: ['inherit', 'pipe', 'inherit']
});

console.log('child', child);

console.log(
  'child',
  keys(child).reduce((acc, k) => ({ ...acc, [k]: child[k] }), {})
);
console.log('child', console.config({ customInspect: false }), child);

let data = readAllSync(child.stdio[1]);

console.log('data', quote(data));
console.log('child.wait() =', child.wait());
