import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import path from './lib/path.js';

let filesystem;

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));

  console.log('main', args);
 console.log('path.sep', path.sep);
  console.log('path.delimiter', path.delimiter);
 
  let filename = process.argv[1];
  console.log('path.basename', path.basename);
  console.log('filename', filename);
  let basename = path.basename(filename, /\.[^.]*$/);
  console.log('basename', basename);
}

Util.callMain(main, true);
