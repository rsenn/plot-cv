import { ColoredText } from './lib/color/coloredText.js';
Util.colorCtor = ColoredText;
//prettier-ignore
let filesystem ;

async function main(...args) {

  let file;
  let str;
  try {
    for(file of args) {
      str = filesystem.readFile(file);

      function getDesc(str) {
        let r = [...Util.matchAll('<(/)?(board|schematic|library)[ >]', str)]
          .map(m => m.index)
          .sort((a, b) => a - b)
          .slice(0, 2);
        let chunk = str.substring(...r);
        let a = ['<description>', '</description>'];
        let indexes = a
          .map(s => new RegExp(s))
          .map(re => re.exec(chunk))
          .map(m => m && m.index);
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
main(...scriptArgs.slice(1));
