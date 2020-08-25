import Util from './lib/util.js';

async function main(args) {
  console.log('stackFrame:', Util.getStackFrame(1));
  console.log('getCallerStack:', Util.getCallerStack());
  Util.log('test:', 1);
}

Util.callMain(main);
