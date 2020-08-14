import { SourceMap } from './lib/sourceMap.js';

import fs from 'fs';
import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});
//prettier-ignore
const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); },
  stat(filename) {return fs.statSync(filename); }
};

/*async*/ function main(...args) {
  console.log('sourceMap');

  if(args.length == 0) args = ['htm/dist/htm.module.js.map', 'htm/htm.js.map', 'htm/index.js.map', 'htm/preact.js.map', 'htm/preact/standalone.modern.js.map', 'htm/standalone.js.map'];
  console.log('args:', args);
  for(let arg of args) {
    let map = SourceMap.fromMapFileComment(`//# sourceMappingURL=${arg} \r\n`, '.', filesystem);

    console.log('map.toBase64():', map.toBase64());
    console.log('map.toComment():', map.toComment());
  }
}

main(...process.argv.slice(2));
/*  .then(() => process.exit(0))
  .catch(() => process.exit(1));
*/
