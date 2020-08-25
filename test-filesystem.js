import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';

async function main() {
  let outputFile = 'test.txt';
  let filesystem = await PortableFileSystem();
  let err;
  let st;

  try {
    // console.log(`filesystem :`, filesystem);
    console.log(`filesystem.writeFile('${outputFile}', ...):`, filesystem.writeFile(outputFile, 'BLAH\nthis is a test!\n\n'));
    console.log(`filesystem.readFile('test-filesystem.js'):`, Util.abbreviate(filesystem.readFile('test-filesystem.js')));
    console.log(`filesystem.realpath('/proc/self'):`, filesystem.realpath('/proc/self'));
    console.log(`filesystem.exists('${outputFile}'):`, filesystem.exists(outputFile));
    console.log(`filesystem.size('${outputFile}'):`, filesystem.size(outputFile));
    console.log(`filesystem.exists('blah.txt'):`, filesystem.exists('blah.txt'));
    st = filesystem.stat('test.txt');
    console.log(
      `filesystem.stat('test.txt'):`,
      Util.toString(
        Util.filterOutMembers(st, Util.isFunction),
        { multiline: true }
      )
    );
    console.log(`st.isFile():`, st.isFile());
    console.log(`filesystem.stat('/proc/self').isSymbolicLink():`, filesystem.stat('/proc/self').isSymbolicLink());
    console.log(`filesystem.stat('/proc/self',true).isDirectory():`, filesystem.stat('/proc/self', true).isDirectory());
  } catch(error) {
    err = error;
  }
  if(err) console.log(`error:`, err);
}

main();
