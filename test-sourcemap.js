import { SourceMap } from './lib/sourceMap.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';

//prettier-ignore
let filesystem;

async function main(...args) {
  await ConsoleSetup();
  filesystem = await PortableFileSystem();

  console.log('sourceMap');

  if(args.length == 0)
    args = [
      'htm/dist/htm.module.js.map',
      'htm/htm.js.map',
      'htm/index.js.map',
      'htm/preact.js.map',
      'htm/preact/standalone.modern.js.map',
      'htm/standalone.js.map'
    ];
  console.log('args:', args);
  for(let arg of args) {
    let map = SourceMap.fromMapFileComment(`//# sourceMappingURL=${arg} \r\n`, '.', filesystem);

    console.log('map.toBase64():', map.toBase64());
    console.log('map.toComment():', map.toComment());
  }
}

Util.callMain(main);

/*  .then(() => Util.exit(0))
  .catch(() => Util.exit(1));
*/
