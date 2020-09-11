import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';
//import ConsoleSetup from './consoleSetup.js';
import { WritableStream } from './lib/stream/writableStream.js';

let filesystem;

async function main() {
  await PortableFileSystem((fs) => (filesystem = fs));
  //await ConsoleSetup();
  let err,
    st,
    outputFile = 'test.txt';

  try {
    // console.log(`filesystem :`, filesystem);
    console.log(`filesystem.writeFile('${outputFile}', ...):`, filesystem.writeFile(outputFile, 'BLAH\nthis is a test!\n\n'));
    console.log(`filesystem.readFile('test-filesystem.js'):`, Util.abbreviate(filesystem.readFile('test-filesystem.js')));
    console.log(`filesystem.realpath('/proc/self'):`, filesystem.realpath('/proc/self'));
    console.log(`filesystem.exists('${outputFile}'):`, filesystem.exists(outputFile));
    console.log(`filesystem.size('${outputFile}'):`, filesystem.size(outputFile));
    console.log(`filesystem.exists('blah.txt'):`, filesystem.exists('blah.txt'));
    st = filesystem.stat('test.txt');
    console.log(`filesystem.stat('test.txt'):`, Util.toString(Util.dumpMembers(st)));
    console.log(`st.isFile():`, st.isFile());
    console.log(`filesystem.stat('/proc/self').isSymbolicLink():`, filesystem.stat('/proc/self').isSymbolicLink());
    console.log(`filesystem.stat('/proc/self',true).isDirectory():`, filesystem.stat('/proc/self', true).isDirectory());

    let fd;

    fd = filesystem.open('test-filesystem.js');
    console.log('fd:', fd);
    console.log('close =', filesystem.close(fd));

    let ofd = filesystem.open('output.tmp', 'w+');

    console.log('ofd', ofd);

    console.log('ofd.write', ofd.write);
    console.log('write =', filesystem.write(ofd, `THIS IS A TEST:\n${new Date().toLocaleString('de-CH')}\n`));

    filesystem.close(ofd);

    fd = filesystem.open('output.tmp', 'r');
    if(fd == -1) console.log('fd:', fd);

    let r = filesystem.read(fd);

    console.log('r =', Util.className(r));
    console.log('r =', r);
    console.log('r.toString() =', r.toString());

    //  console.log('Buffer =', Util.getGlobalObject().Buffer);

    let w = new WritableStream('stream.tmp', { flags: 'w', encoding: 'utf-8' });

    //   w.on('open', arg => console.log("open:", arg));
    w.on('data', (arg) => console.log('data:', arg.byteLength, filesystem.bufferToString(arg).replace(/\n/g, '\\n')));
    w.on('close', () => console.log('close:', w.fd));

    w.write('TEST\nBLAH\n' + new Date());
    w.write('\n');

    w.close();
  } catch(error) {
    err = error;
  }
  if(err) {
    console.log(`error:`, err);
    console.log(`error.stack:\n` + err.stack);
  }
}

Util.callMain(main, true);
