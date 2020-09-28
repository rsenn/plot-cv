/*import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './consoleSetup.js';*/
import { execStream } from './childProcess.js';
import { AsyncWrite, AsyncRead, AcquireReader, AcquireWriter, PipeToRepeater, LineReader, WritableRepeater, WriteIterator, ReadFromIterator, TextTransformStream, PipeTo, CreateTransformStream, isStream, CreateWritableStream, LineBufferStream, RepeaterSink, RepeaterSource } from './lib/stream/utils.js';
import Util from './lib/util.js';

//prettier-ignore
let filesystem;

function alt_main(...args) {
  throw new Error('blah');
}

function pkgName(url) {
  url = url.replace(/.*\//g, "");
  url = url.replace(/\.pkg\..*/g, "");
  return url;
}
const URLS = ['https://repo.msys2.org/mingw/i686/mingw32.db', 'https://repo.msys2.org/mingw/x86_64/mingw64.db', 'https://repo.msys2.org/msys/i686/msys.db', 'https://repo.msys2.org/msys/x86_64/msys.db'];
async function main(...args) {
  console.log('main(', ...args, ')');
  // await ConsoleSetup({ breakLength: 400 });
  //await PortableFileSystem(fs => (filesystem = fs));

  let ret;

  if(args.length == 0) args.push(URLS[0]);

  for(let arg of args) ret = await processUrl(arg);
}


async function processUrl(url) {
  let dir = url.replace(/\/[^\/]*$/, '');
  let base = url.replace(/.*\//, '');
  let stream = execStream('sh', ['-c', `curl -s ${url}  | zcat | tee ${base}`]);
  let transform = await LineBufferStream();

  console.log('base:', base);
  console.log('transform.:', transform.lines);
  let [iterator, sink] = await RepeaterSink();
  stream.pipe(transform);
  transform.pipe(sink);
  let i = 0;
  let key;
  let pkg = '';
  let map = {};
  let getObj = Util.getOrCreate(map);
  let getProp;
  let obj;
  /* for await (let line of iterator) {
    console.log('line:', Util.unescape(line));
  }*/

  for await (let line of iterator) {
    //  if(line == '\n') continue;
    if(line.startsWith('%') && line.endsWith('%\n')) {
      key = line.slice(1, -2).toLowerCase();
      continue;
    } else if(line == '\n') {
      key = null;
      continue;
    } else if(!key && pkg !== null) {
      pkg = null;
      continue;
    } else if(!key && (pkg === null || (typeof pkg == 'string' && pkg.length))) {
      obj = null;
      pkg = line.trim();
      continue;
    }
    if(obj == null) {
      obj = getObj(pkgName(pkg));
      obj.url = pkg;
    }
    let data = line.trim();
    if(Util.isNumeric(data)) data = +data;
    if(obj[key] === undefined) obj[key] = data;
    else obj[key] += '\n' + data;

    /*    console.log(`key: ${key}`);
      console.log(`line #${++i}:`, line.replace(/\n/g, '\\n'));*/
  }
  console.log('map:', map);

  console.log(Object.keys(map).length);
  console.log(Object.keys(map).map(file => `${dir}/${file}`));
}
/*try {
  alt_main(...scriptArgs);
}catch(error) {
  console.log("ERROR:", error);
}*/

Util.callMain(main, true);
