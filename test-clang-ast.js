import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AsyncWrite, AsyncRead, AcquireReader, AcquireWriter, PipeToRepeater, LineReader, WritableRepeater, WriteIterator, ReadFromIterator, TextTransformStream, PipeTo, CreateTransformStream, isStream, CreateWritableStream, LineBufferStream, RepeaterSink, RepeaterSource } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import fs from 'fs';
import inspect from './lib/objectInspect.js';
import deep from './lib/deep.js';

//prettier-ignore
let filesystem, spawn;

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 80, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  console.log('Processing files:', args);
  await processFiles(...args);
}

async function processFiles(...files) {
  let args = [
    '-Xclang',
    '-ast-dump=json',
    '-fsyntax-only',
    '-D_WIN32=1',
    '-DWINAPI=',
    '-D__declspec(x)=',
    /*    '-DPDWORD=unsigned long*',
    '-DUCHAR=unsigned char',
    '-DBYTE=char',
    '-DTBYTE=uint16',
    '-DWORD=unsigned short',
    '-DDWORD=unsigned long',
    '-DULONG=unsigned long',
    '-DCONST=const',*/
    '-include',
    '/usr/x86_64-w64-mingw32/include/wtypesbase.h',
    '-I/usr/x86_64-w64-mingw32/include'
  ];

  let child = spawn(['clang', ...args, ...files], {
    stdin: 'inherit',
    stdio: 'pipe',
    stderr: 'pipe'
  });

  let json = '',
    errors = '';

  AcquireReader(child.stdout, async reader => {
    let r, str;
    while((r = await reader.read())) {
      if(!r.done) {
        str = r.value.toString();
        //  console.log('stdout:', );
        json += str;
      }
    }
  });

  AcquireReader(child.stderr, async reader => {
    let r;
    while((r = await reader.read())) {
      if(!r.done) errors += r.value.toString();
    }
  });

  console.log('child.wait():', await child.wait());
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  const numErrors = +errorLines[errorLines.length - 1].replace(/.*\s([0-9]+)\serrors\sgenerated.*/g,
    '$1'
  );
  errorLines = errorLines.filter(line => /error:/.test(line));

  let obj = JSON.parse(json);
  let flat = deep.flatten(obj, new Map(), (v, p) => Util.isObject(v) /*&& 'kind' in v*/);
  let re = /(^[_P]?IMAGE_)/;
  let file = [...Util.filter(flat.entries(), ([path, decl]) => 'file' in decl)];
  let typedefs = [...Util.filter(flat.entries(), ([path, decl]) => decl.kind == 'TypedefDecl')];
  const names = decls => [...decls].map(([path, decl]) => decl.name);
  const defs = decls => [...decls].map(([path, decl]) => [decl.name, decl.loc]);
  let typenames = names(typedefs.filter(([path, decl]) => re.test(decl.name)));

  //console.log('obj:', obj);
  console.log(`numErrors: ${numErrors}`);
  console.log('errorLines:', errorLines);
  console.log('typedefs:', [...typedefs]);
  console.log('typedefs:', defs(typedefs));
  console.log('file:', file);
  /*console.log("typenames:", typenames.join("\n"));
console.log("typenames:", names(typedefs));*/
}

Util.callMain(main, true);

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}
