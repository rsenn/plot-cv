import { readFileSync } from 'fs';
import Alea from './lib/alea.js';
import cparse from './lib/cparse.js';
import cpp from './lib/cpp.js';
import * as path from './lib/path.js';
let filesystem,
  childProcess,
  prng = new Alea(318);

const sources = [
  'quickjs/cutils.c',
  'quickjs/ffi/ffi.c',
  'quickjs/libbf.c',
  'quickjs/libregexp.c',
  'quickjs/libunicode.c',
  'quickjs/qjs-net/minnet.c',
  'quickjs/qjs.c',
  'quickjs/qjscalc.c',
  'quickjs/qjsc.c',
  'quickjs/quickjs.c',
  'quickjs/quickjs-debugger.c',
  'quickjs/quickjs-debugger-transport-unix.c',
  'quickjs/quickjs-find-module.c',
  'quickjs/quickjs-libc.c',
  'quickjs/repl.c',
];

const includeDirs = ['/opt/diet/include', '.'];

const FindIncludeFunc = source => {
  const dirs = [path.dirname(source), ...includeDirs];

  return name => {
    for(let dir of dirs) {
      let file = path.join(dir, name);
      //console.log('file:', file);
      if(filesystem.exists(file)) return file;
    }
  };
};

const getSource = () => sources[randInt(0, sources.length - 1, prng)];

function* Reader(input) {
  const buffer = new ArrayBuffer(1024);
  let ret;
  do {
    ret = filesystem.read(input, buffer, 0, 1024);
    console.log('ret:', ret);

    yield buffer.slice(0, ret);
  } while(ret == 1024);
}

function ReadAll(input) {
  let data = '';
  for(let chunk of Reader(input)) {
    console.log('chunk:', chunk);
    console.log('chunk.length:', filesystem.bufferSize(chunk));
    data += filesystem.bufferToString(chunk);
  }
  console.log('data:', data);
  return data;
}

function StripPP(code) {
  return code
    .split(/\n/g)
    .filter(line => !/^\s*#/.test(line))
    .join('\n');
}

function main(...args) {
  const file = 'quickjs/hello.c' || getSource();

  console.log('Source file:', file);
  //const output = filesystem.open('out.e', 'w');
  // console.log('out fd:', filesystem.fileno(output));
  let cmd = ['/usr/lib/gcc/x86_64-linux-gnu/10/cc1', '-E', ...includeDirs.map(dir => `-I${dir}`), file /*, '-o', 'out.e'*/];

  console.log('cmd:', cmd.join(' '));

  /*  let proc = childProcess(cmd[0], cmd.slice(1), {
    block: false,
    stdio: [null, 'pipe', 'pipe']
  });

  console.log('out:', proc.stdout);*/

  //const src =   ReadAll(proc.stdout);
  const src = readFileSync(file, 'utf-8');

  const findInclude = FindIncludeFunc(file);
  let code;
  const pp = cpp({
    includeFunc(file, system, resolve) {
      // console.log('completionFunc', { file, system, resolve });
      file = findInclude(file);
      // console.log('completionFunc', file);

      const code = filesystem.readFileSync(file);
      console.log('includeFunc', {
        file,
        code: abbreviate(escape(code + ''), 40),
      });

      resolve(code);
    },
    completionFunc(text, arr, state) {
      // console.log('completionFunc', { text: abbreviate(text), arr: arr.map(s => abbreviate(s)), state });
      code = text;
    },
    errorFunc(error) {
      console.log('errorFunc', { error });
      throw new Error(error);
    },
    warnFunc(warning) {
      console.log('warnFunc', { warning });
    },
  });

  pp.define('__WORDSIZE', 64);
  pp.define('__STDC_LIMIT_MACROS', 1);
  pp.define('__STDC_CONSTANT_MACROS', 1);
  pp.define('__SIZE_TYPE__', 'unsigned long');
  pp.define('__PTRDIFF_TYPE__', 'long');

  let e;

  e = pp.run(src);

  //const src = filesystem.readFileSync('out.e');

  console.log('Source code:', code);

  const ast = cparse(code, {
    file,
    types: [/*'int8_t','int16_t','int32_t','int64_t', 'uint8_t','uint16_t','uint32_t','uint64_t',*/ 'void', 'char', 'short', 'int', 'long', 'float', 'double'],
  });

  console.log(ast);
}

main(...scriptArgs.slice(1));
