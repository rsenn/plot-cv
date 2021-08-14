import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { execStream } from './childProcess.js';
import {
  AsyncWrite,
  AsyncRead,
  AcquireReader,
  AcquireWriter,
  PipeToRepeater,
  LineReader,
  WritableRepeater,
  WriteIterator,
  ReadFromIterator,
  TextTransformStream,
  PipeTo,
  CreateTransformStream,
  isStream,
  CreateWritableStream,
  LineBufferStream,
  RepeaterSink,
  RepeaterSource
} from './lib/stream/utils.js';
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
const URLS = [
  'https://repo.msys2.org/mingw/i686/mingw32.db',
  'https://repo.msys2.org/mingw/x86_64/mingw64.db',
  'https://repo.msys2.org/msys/i686/msys.db',
  'https://repo.msys2.org/msys/x86_64/msys.db'
];
const BASE_URL = 'https://repo.msys2.org';
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
    a = [
      ...Util.filter(
        urls.map(url => url.replace(BASE_URL + '/', '')),
        new RegExp(args[0])
      )
    ];
    if(a.length > 0) args.shift();
    else break;
    urls = a;
  }
  let data = {};
  urls = urls.map(loc => BASE_URL + '/' + loc);
  console.log('urls:', urls);
  for(let url of urls) {
    ret = await processUrl(url, data);

    console.debug(`url: ${url} ret:`, ret);
  }
  if(args.length == 0) args.unshift('.*');
  /*let predicates = args.map(arg => new RegExp(arg));
  console.log("predicates:", predicates);*/
  let packages = Object.keys(data);
  let files = [];
  let i;

  WriteFile('packages.list', packages.join('\n'));

  let locations = packages.map(url => url.replace('https://repo.msys2.org/', ''));
  let names = locations.map(url =>
    url.replace(/(.*)(-[^-.]+)(\.pkg\..*)/g, '$1|$2|$3').split(/\|/g)
  );
  console.log('names.length:', names.length);

  console.log('names:', names.slice(-10, -1));

  for(let i = 0; i < args.length; i++) {
    let arg = args[i].replace(/\+/, '\\+');
    let [name, ver] = arg.split('=');
    if(name.endsWith('>')) {
      name = name.slice(0, -1);
      ver = null;
    }

    let re = new RegExp(arg.startsWith('/') ? name + '-' + (ver || 'r?[0-9]') : arg, 'gi');
    let matches = [...Util.filter(names, item => re.test(item[0]))];
    let pkgs = matches.map(loc => 'https://repo.msys2.org/' + loc.join(''));
    /*console.log("re:", re+'');
console.log("matches:", matches);*/
    if(pkgs.length == 0 || packages.length == pkgs.length) {
      console.log('re =', re, ' pkgs.length =', pkgs.length, ' pacakges.length =', packages.length);
      pkgs = Util.filter(
        packages,
        (re = new RegExp(arg.startsWith('/') ? name + '-[a-z]+-' + (ver || 'r?[0-9]') : arg, 'gi'))
      );
      if(pkgs.length == 0 || packages.length == pkgs.length) {
        console.log(
          're =',
          re,
          ' pkgs.length =',
          pkgs.length,
          ' pacakges.length =',
          packages.length
        );
        console.error(`Number of packages ${pkgs.length} when matching ${re}`);
        continue;
        throw new Error(`Number of packages ${pkgs.length} when matching ${re}`);
      }
    }
    for(let pkg of pkgs) {
      const obj = data[pkg];
      const { depends = '', optdepends = '' } = obj;

      const deps = concat(depends, optdepends);
      //console.log('obj:', obj.name, deps);
      //console.log('deps:', deps);
      Util.pushUnique(args, ...deps.map(dep => '/' + dep));
      Util.pushUnique(files, pkg);
    }
  }
  let dirs = Util.unique(files.map(file => path.dirname(file))).map(
    dir => Util.parseURL(dir).location
  );
  //  console.debug("dirs:", dirs);

  let host = dirs[0]
    .split(/\//g)
    .filter(p => p != '')
    .join('-');
  let installScript = `install-${host}.sh`;
  console.log(`Writing install script: '${installScript}' ...`);
  let output = filesystem.open(installScript, 'w');
  filesystem.write(output, `#!/bin/sh -x\n`);

  for(let file of files) {
    let parts = file.split('/');
    let [system, arch] = parts.slice(-3, -1);
    let [os, kernel, rootDir] = system.startsWith('mingw')
      ? ['w64', 'mingw32', '/sysroot/mingw']
      : ['pc', 'msys', ''];
    let extractDest = `/usr/${arch}-${os}-${kernel}${rootDir}`;
    let compressProgram = file.endsWith('xz') ? 'xz' : 'zstd';
    // let line =  `CMD="curl -s '${file}' | tar --use-compress-program=${compressProgram} -C ${extractDest} --strip-components=1 -xv 2>/dev/null"; eval "$CMD" || { R=$?; echo "ERROR: $CMD" ; exit $R; }\n`;
    let line = `curl -s '${file}' | tar --use-compress-program=${compressProgram} -C ${extractDest} --strip-components=1 -xv \n`;

    filesystem.write(output, line);
  }
  filesystem.close(output);
}

async function processUrl(url, map) {
  let dir = url.replace(/\/[^\/]*$/, '');
  let base = url.replace(/.*\//, '');

  let stat = Util.tryCatch(
    () => filesystem.stat(base),
    st => st,
    () => ({ mtime: 0 })
  );

  let expired = stat.mtime + 5 * 60 * 1000 < new Date();

  console.log('expired:', expired);
  let stream = expired
    ? execStream('sh', ['-c', `curl -s ${url}  | zcat | tee ${base}`])
    : fs.createReadStream(base);

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
  /* for await(let line of iterator) {
    console.log('line:', Util.escape(line));
  }*/

  for await(let line of iterator) {
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
      console.log(`line #${++i}:`, line.replaceAll('\n', '\\n'));*/
  }
}

Util.callMain(main, true);

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}
