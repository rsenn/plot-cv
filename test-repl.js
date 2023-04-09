import { atexit } from './lib/misc.js';
import REPL from './repl.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import * as Terminal from './terminal.js';

let repl;

async function CommandLine() {
  let repl = (globalThis.repl = new REPL('AST'));

  repl.exit = n => std.exit(n);
  repl.directives = {
    c(...args) {
      console.log('c', { args });
      return Compile(...args);
    }
  };

  Util.atexit(() => {
    Terminal.mousetrackingDisable();
    console.log(`EXIT`);
  });
  Terminal.mousetrackingEnable();

  await repl.run();
  console.log('REPL done');
}

async function main(...args) {
  await ConsoleSetup({ breakLength: 80, depth: Infinity });
  await PortableFileSystem();

  await CommandLine();
}

main(...scriptArgs.slice(1));
