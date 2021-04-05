import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tokenize from './tokenize.js';
import PortableFileSystem from './lib/filesystem.js';

const consoleOpts = {
  depth: Infinity,
  compact: 2,
  hideKeys: [],
  maxArrayLength: Infinity
};

async function main(...args) {
  await ConsoleSetup(consoleOpts);
  await PortableFileSystem();
  console.options = consoleOpts;

  let code = filesystem.readFile(args[0] ?? 'pa_devs.c', 'utf-8');
  console.log(Util.abbreviate(code));
  let i = 0;

  for await(let token of tokenize(code)) {
    console.log(`token #${i}`, token);
    i++;
  }
}

Util.callMain(main, true);
