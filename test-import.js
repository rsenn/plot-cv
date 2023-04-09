import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import path from './lib/path.js';
import * as bjson from 'bjson';

let filesystem;

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: 4 });

  console.log('bjson:', bjson);
  let ffi = await import('ffi');
  console.log('ffi:', ffi);
  return;
}

main(...scriptArgs.slice(1));
