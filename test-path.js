import PortableFileSystem from './lib/filesystem.js';
import Util from './lib/util.js';
import { Console } from 'console';
import path from './lib/path.js';

let filesystem;

function main(...args) {
globalThis.console =  new Console({ inspectOptions: { colors: true, maxArrayLength: Infinity, compact: 1, breakLength: Infinity }});
  console.log('main(', ...args, ')');
  //console.log('main', args);
  console.log('path', Object.getOwnPropertyNames(path));
  console.log('path', Util.getMethodNames(path));

  let filename = Util.getArgv()[1];
 
   
  console.log('path.basename', path.basename);
  console.log('filename', filename);
  let basename = path.basename(filename, /\.[^.]*$/);
  console.log('basename', basename);
}

main();
