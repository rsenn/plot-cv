import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './consoleSetup.js';
import { execStream } from './childProcess.js';
import { AsyncWrite, AsyncRead, AcquireReader, AcquireWriter, PipeToRepeater, LineReader, WritableRepeater, WriteIterator, ReadFromIterator, TextTransformStream, PipeTo, CreateTransformStream, isStream, CreateWritableStream, LineBufferStream, RepeaterSink, RepeaterSource } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import fs from 'fs';

//prettier-ignore
let filesystem;

function alt_main(...args) {
  throw new Error('blah');
}

function pkgName(url) {
  url = url.slice(url.lastIndexOf('/') + 1);
  url = url.replace(/\.pkg\..*/g, '');
  return url;
}

function concat(...args) {
  return args
    .map(arg =>
      arg
        .split(/\n/g)
        .filter(str => str !== '')
        .map(str => str.replace(/:.*/, ''))
    )
    .flat();
}
const URLS = ['https://repo.msys2.org/mingw/i686/mingw32.db', 'https://repo.msys2.org/mingw/x86_64/mingw64.db', 'https://repo.msys2.org/msys/i686/msys.db', 'https://repo.msys2.org/msys/x86_64/msys.db'];
async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 80 });
  await PortableFileSystem(fs => (filesystem = fs));

  let ret;
  let a,
    urls = URLS;

  if(args.length == 0) args.unshift('.*');
  console.log('args:', args);
  while(args.length > 0) {
    a = [...Util.filter(urls, new RegExp(args[0]))];
    if(a.length > 0) args.shift();
    else break;
    urls = a;
  }
  let data = {};
  console.log('urls:', urls);
  for(let url of urls) ret = await processUrl(url, data);
  if(args.length == 0) args.unshift('.*');
  /*let predicates = args.map(arg => new RegExp(arg));
  console.log("predicates:", predicates);*/
  let packages = Object.keys(data);
  let files = [];
  let i;
  console.log("packages.length:", packages.length);
  console.log("packages:", packages.slice(0,10));
  for(let i = 0; i < args.length; i++) {
    let arg = args[i].replace(/\+/, '\\+');
    let [name, ver] = arg.split('=');
    if(name.endsWith('>')) {
      name = name.slice(0, -1);
      ver = null;
    }

    let re = new RegExp(arg.startsWith('/') ? name + '-' + (ver || 'r?[0-9]') : arg, 'gi');
    console.log("re:", re); 
    let pkgs = [... Util.filter(packages, re)];

    console.log("pkgs:", pkgs);
    if(pkgs.length != 1) {
      pkgs = Util.filter(packages, (re = new RegExp(arg.startsWith('/') ? name + '-[a-z]+-' + (ver || 'r?[0-9]') : arg, 'gi')));

      if(pkgs.length != 1) {
        console.log('pkgs:', pkgs);
        throw new Error(`Number of packages ${pkgs.length} when matching ${re}`);
      }
    }
    for(let pkg of pkgs) {
      const obj = data[pkg];
      const { depends = '', optdepends = '' } = obj;

      const deps = concat(depends, optdepends);
      console.log('obj:', obj.name, deps);
      //console.log('deps:', concat(depends, optdepends));
      Util.pushUnique(args, ...deps.map(dep => '/' + dep));
      files.push(pkg);
    }
  }
  function matchAll(pkgs) {
    for(let arg of args) {
    }
  }
  let output = filesystem.open('msys-install.sh', 'w');

  for(let file of files) {
    let parts = file.split('/');
    let [system,arch] = parts.slice(-3, -1);
    console.log("parts:", parts, system);
    let os = system.startsWith('mingw') ? 'w64' : 'pc';
    let kernel = system.startsWith('mingw') ? 'mingw32' : 'msys';
    let rootDir = system.startsWith('mingw') ? '/mingw' : '/usr';
    let extractDest = `/usr/${arch}-${os}-${kernel}/sysroot${rootDir}`;
    let compressProgram = file.endsWith('xz') ? 'xz' : 'zstd';
    filesystem.write(output, `curl -s '${file}' | tar --use-compress-program=${compressProgram} -C ${extractDest} --strip-components=1 -xv\n`);
  }
  filesystem.close(output);
}

async function processUrl(url, map) {
  let dir = url.replace(/\/[^\/]*$/, '');
  let base = url.replace(/.*\//, '');

  let stat = filesystem.stat(base);
  let expired = stat.mtime + 5 * 60 * 1000 < new Date();

  console.log('expired:', expired);
  let stream = expired ? execStream('sh', ['-c', `curl -s ${url}  | zcat | tee ${base}`]) : fs.createReadStream(base);

  let transform = await LineBufferStream();

  console.log('base:', base);
  console.log('transform.:', transform.lines);
  let [iterator, sink] = await RepeaterSink();
  stream.pipe(transform);
  transform.pipe(sink);
  let i = 0;
  let key;
  let pkg = '';
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
      pkg = dir + '/' + pkg;
      obj = getObj(pkg);
    }
    let data = line.trim();
    if(Util.isNumeric(data)) data = +data;
    if(obj[key] === undefined) obj[key] = data;
    else obj[key] += '\n' + data;

    /*    console.log(`key: ${key}`);
      console.log(`line #${++i}:`, line.replace(/\n/g, '\\n'));*/
  }
}

Util.callMain(main, true);
