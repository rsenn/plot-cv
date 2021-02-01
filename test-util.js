import { Util } from './lib/util.js';
import { sleep } from 'os';

/*console.log(Util.escape(read('/proc/self/cmdline')));
console.log(Util.escape(read('/etc/hosts')));*/
//console.log(globalThis.options());
async function main(...args) {
  console.log('Util.getPlatform():', Util.getPlatform());
  console.log('Util.getArgs():', Util.getArgs());
  console.log('Util.getArgv():', Util.getArgv());
  console.log('Util.scriptName():', Util.scriptName());
  //  console.log('Util.now:', await Util.now);
  console.log('Util.now:', Util.now);
  for(let i = 0; i < 1000; i += 100) console.log('sleep(100):', sleep(100));
  let now;
  console.log('Util.now:', (now = Util.now));
  console.log('Util.getNow():', Util.getNow());
  console.log('Util.isAsync(Util.now):', Util.isAsync(Util.now));
  console.log('now:', now);
  console.log('Util.now():', await now());
  let obj = JSON.parse('{"a":1,"b":2}');

  console.log('obj:', obj);
  console.log(`obj=${obj}`);
  console.log(`{a:1,b:2}: ${{ a: 1, b: 2 }}`);
}

Util.callMain(main, true);
