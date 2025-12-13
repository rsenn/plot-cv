import { getMethodNames } from './lib/misc.js';
import * as path from './lib/path.js';
import { Console } from 'console';
let filesystem;

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: { colors: true, maxArrayLength: Infinity, compact: 1, breakLength: Infinity }
  });
  console.log('main(', ...args, ')');
  //console.log('main', args);
  console.log('path', Object.getOwnPropertyNames(path));
  console.log('path', getMethodNames(path));

  let filename = process.argv[1];
  let cwd = path.absolute('.');
  console.log(`path.relative('/', '${cwd}')`, path.relative('/', cwd));

  console.log('path.sep', path.sep);
  console.log('path.delimiter', path.delimiter);
  console.log('path.basename', path.basename);
  console.log('filename', filename);
  let basename = path.basename(filename, /\.[^.]*$/);
  console.log('basename', basename);
}

main();