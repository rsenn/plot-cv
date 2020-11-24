import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

async function main(...args) {
  await ConsoleSetup({ depth: 10 });
  console.log('stackFrame:', Util.getStackFrame(1));
  let st = Util.getCallerStack();
  console.log('getCallerStack:', st);
  /*console.log('test:', 1);*/
}

Util.callMain(main, true);
