
import PortableFileSystem from './lib/filesystem.js';
import { ConsoleSetup } from './lib/consoleSetup.js';

let filesystem;
let globalThis;

main(...scriptArgs.slice(1));
  let { message, stack } = error;

  console.log('ERROR message =', message);
  console.log('ERROR stack:\n' + stack);
});

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));

  console.log('main args =', args);

  await ConsoleSetup({ depth: 3 });

  globalThis = globalThis;

  throw new Error('This is an error');
}
