import { abbreviate } from './lib/misc.js';
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
  console.log(abbreviate(code));
  let i = 0;

  for await(let token of tokenize(code)) {
    console.log(`token #${i}`, token);
    i++;
  }
}

main(...scriptArgs.slice(1));
