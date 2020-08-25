import PortableFileSystem from './lib/filesystem.js';
import util from 'util';
import Util from './lib/util.js';
import { XPath } from './lib/xml.js';
import deep from './lib/deep.js';
import tXml from './lib/tXml.js';
import { RGBA, HSLA } from './lib/color.js';
import { Iterator, IteratorForwarder } from './lib/iterator.js';
import toSource from './lib/tosource.js';
import { toXML, Path } from './lib/json.js';
import { ColorMap } from './lib/draw/colorMap.js';
import { Console } from 'console';
import Alea from './lib/alea.js';
import { Functional } from './lib/functional.js';
import KolorWheel from './lib/KolorWheel.js';
import { ColoredText } from './lib/color/coloredText.js';
Util.colorCtor = ColoredText;
global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
});
//prettier-ignore
let filesystem ;

async function main(...args) {
  filesystem = await PortableFileSystem();

  let file;
  let str;
  try {
    for(file of args) {
      str = filesystem.readFile(file);

      function getDesc(str) {
        let r = [...Util.matchAll('<(/)?(board|schematic|library)[ >]', str)]
          .map((m) => m.index)
          .sort((a, b) => a - b)
          .slice(0, 2);
        let chunk = str.substring(...r);
        let a = ['<description>', '</description>'];
        let indexes = a
          .map((s) => new RegExp(s))
          .map((re) => re.exec(chunk))
          .map((m) => m && m.index);
        indexes[0] += a[0].length;
        return chunk.substring(...indexes);
      }

      let description = getDesc(str);

      console.log(`description:\n` + description);
      //r = [...Util.matchAll('<\\/?(description)[^>]*>', str)];
    }
  } catch(err) {
    console.log('err:', err);
  }
}
main(...process.argv.slice(2));
