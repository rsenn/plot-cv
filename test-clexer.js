import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tokenize from './tokenize.js';
  import PortableFileSystem from './lib/filesystem.js';


async function main(...args) {
  await ConsoleSetup({ depth: Infinity });
     await PortableFileSystem();

let code = filesystem.readFile('pa_devs.c', 'utf-8');
console.log(Util.abbreviate(code));

let tokens = tokenize(code);
console.log('tokens',tokens);;

}

Util.callMain(main, true);
