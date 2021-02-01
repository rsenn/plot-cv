import { Util } from './lib/util.js';
import { sleep } from 'os';
import ConsoleSetup from './lib/consoleSetup.js';

/*console.log(Util.escape(read('/proc/self/cmdline')));
console.log(Util.escape(read('/etc/hosts')));*/
//console.log(globalThis.options());
async function main(...args) {
  await ConsoleSetup({ breakLength: 120, depth: 10 });

  console.log('Util.getPlatform():', Util.getPlatform());
  console.log('Util.getArgs():', Util.getArgs());
  console.log('Util.getArgv():', Util.getArgv());
  console.log('Util.scriptName():', Util.scriptName());
  //  console.log('Util.now:', await Util.now);
  console.log('Util.now:', Util.now);
  console.log('Util.waitFor:', Util.waitFor);
  for(let i = 0; i < 100; i += 10) console.log('Util.waitFor(10):', await Util.waitFor(10));
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
  console.log(`await import('os'):`, Object.keys(await import('os')));
  console.log(`await import('std'):`, await import('std').catch(err => (console.log(err), err)));
  //console.log(`await import('ffi.so'):`, await import('ffi.so'));
  //
  Util.exit(0);
}

Util.callMain(main, true);
