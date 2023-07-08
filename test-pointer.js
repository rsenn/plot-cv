import { getMethodNames } from 'util';
import { Console } from 'console';
import * as deep from 'deep.dll';
import { Pointer } from 'pointer.dll';
function main(...args) {
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      compact: 1,
      depth: Infinity,
      maxArrayLength: 100,
      breakLength: 10000
    }
  });
  let ptr = new Pointer(['children', 3]);
  let obj = {};
  let obj2 = { children: [, , , { tag: 'TEST' }] };
  console.log('ptr', ptr);
  console.log('deep', deep);
  console.log('ptr', getMethodNames(ptr, 1, 1));

  deep.set(obj, ptr.concat(['children']), [1, 1234]);
  console.log('obj', obj);
  console.log('ptr.deref(obj2)', ptr.deref(obj2));
}

let error;
try {
  main(...process.argv.slice(1));
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    console.log('FAIL');
    process.exit(1);
  } else {
    console.log('SUCCESS');
  }
}