import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';

class Callable extends Function {
  constructor(...args) {
    const isFunction = typeof args[0] == 'function';

    super(...(isFunction ? [] : args));

    let obj = isFunction
      ? Object.setPrototypeOf(args.shift(), Callable.prototype)
      : this;

    return obj;
  }
}

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: Infinity });
  await PortableFileSystem();

  let fn = new Callable((...args) => args);
  let fn2 = new Callable('...args', 'return args;');

  let ret = fn();
  let ret2 = fn2([1, 2, 3]);

  console.log('fn:', fn);
  console.log('ret:', ret);
  console.log('ret2:', ret2);
}

Util.callMain(main, true);
