import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import tokenize from './tokenize.js';
import PortableFileSystem from './lib/filesystem.js';
const consoleOpts = { depth: Infinity, compact: 5, hideKeys: ['pos'] };

async function main(...args) {
  await ConsoleSetup(consoleOpts);

  await PortableFileSystem();
  console.options = { ...consoleOpts, depth: 3, compact: 1, hideKeys: ['offset'] };

  let code = filesystem.readFile('pa_devs.c', 'utf-8');
  console.log(Util.abbreviate(code));

  let tokens = tokenize(code);
  console.log('tokens', tokens);
}

Util.callMain(main, true);
