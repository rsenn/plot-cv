import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import parse from 'bash-parser';

const consoleOpts = { colors: true, depth: Infinity, compact: 5, hideKeys: ['pos'] };

async function main(...args) {
  await ConsoleSetup();
  await PortableFileSystem();

  console.options = consoleOpts;

  const data = filesystem.readFile('../cfg.sh');

  const ast = parse(data ?? 'echo ciao;');
  console.log('ast:', ast);
}
Util.callMain(main, true);
