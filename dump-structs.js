import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AcquireReader } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';
import { Type, Compile, AstDump } from './clang-ast.js';

//prettier-ignore
let filesystem, spawn;

Util.define(Array.prototype, {
  findLastIndex(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this[i];
      if(predicate(x, i, this)) {
        return i;
      }
    }
    return -1;
  },
  tail() {
    return this[this.length - 1];
  },
  startsWith(start) {
    for(let i = 0; i < start.length; i++) if(this[i] !== start[i]) return false;
    return true;
  }
});

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  let params = Util.getOpt({
      output: [true, null, 'o'],
      xml: [true, null, 'X'],
      json: [true, null, 'j'],
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      debug: [false, null, 'x'],
      'system-includes': [false, null, 's'],
      'no-remove-empty': [false, null, 'E'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );
  console.log('main', params);

  let defs = params.define || [];
  let includes = params.include || [];

  args = [];
  const win32 = false;

  if(win32) {
    defs = defs.concat(Object.entries({
        PDWORD: 'unsigned long*',
        UCHAR: 'unsigned char',
        BYTE: 'char',
        TBYTE: 'uint16',
        WORD: 'unsigned short',
        DWORD: 'unsigned long',
        ULONG: 'unsigned long',
        CONST: 'const'
      })
    );

    args = args.concat([
      '-D_WIN32=1',
      '-DWINAPI=',
      '-D__declspec(x)=',
      '-include',
      '/usr/x86_64-w64-mingw32/include/wtypesbase.h',
      '-I/usr/x86_64-w64-mingw32/include'
    ]);
  }
  console.log('args', { defs, includes });
  args = args.concat(defs.map(d => `-D${d}`));
  args = args.concat(includes.map(v => `-I${v}`));

  console.log('Processing files:', args);

  await processFiles(...params['@']);

  async function processFiles(...files) {
    for(let file of files) {
      let json, ast;
      let outfile = path.basename(file, /\.[^./]*$/) + '.ast.json';

      let st = [file, outfile].map(name => filesystem.stat(name));

      let times = st.map(stat => (stat && stat.mtime) || 0);

      if(times[1] >= times[0]) {
        console.log('Reading cached AST from:', outfile);
        json = filesystem.readFile(outfile);
        ast = JSON.parse(json);
      } else {
        json = await AstDump(file, args);
        ast = JSON.parse(json);

        dumpFile(outfile, JSON.stringify(ast, null, 2));
      }

      let tree = new Tree(ast);
      let flat = /*tree.flat();*/ deep.flatten(ast,
        new Map(),
        (v, p) =>
          ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 &&
          Util.isObject(v) /*&& 'kind' in v*/
      );
      let entries = [...flat];
      let locations = [];
      let l = Object.setPrototypeOf({}, { toString() {} });
      //let path = new WeakMap();
      let idmap = {};
      let id2path = {};
      for(let entry of entries) {
        const [p, n] = entry;
        let loc = GetLoc(n);
        if(loc)
          l = Object.setPrototypeOf({ ...(l || {}), ...loc },
            {
              toString() {
                let s;
                if(this.file) {
                  s = this.file;
                  if('line' in this) s += ':' + this.line;
                  if('col' in this) s += ':' + this.col;
                }
                return s;
              }
            }
          );
        if('id' in n) {
          idmap[n.id] = entry;
          id2path[n.id] = p;
        }
        //Util.removeKeys(entries[locations.length][1], ['loc','range']);
        entry[2] = l;
        locations.push(l);
      }

      for(let [n, p] of deep.iterate(ast, v => Util.isObject(v))) {
        if(/(loc|range)/.test(p[p.length - 1] + '')) {
          //console.log("remove", p);
          deep.unset(ast, p);
        } else {
          //let node = deep.get(ast, p);
          //  path.set(n, p);
        }
      }

      function Path2Loc(path) {
        if(Array.isArray(path)) path = path.join('.');

        let idx = entries.findLastIndex(([p]) => path.startsWith(p));

        if(idx != -1) return entries[idx][2];
      }
    }
  }
}

Util.callMain(main, true);

function dumpFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function writeOutput(name, data) {
  let n = data.length;
  let ret = dumpFile(name, data.join('\n'), false);

  /*if(ret > 0)*/ console.log(`Wrote ${n} records to '${name}'.`);
  return ret;
}

function GetLibraryFor(symbolName) {
  if(/BZ2/.test(symbolName)) return 'libbz2.so.1';
  if(/(flate|compress|zlib|gz)/i.test(symbolName)) return 'libz.so.1';
  if(/lzma/i.test(symbolName)) return 'liblzma.so.1';
  if(/brotli/i.test(symbolName)) return 'libbrotli.so.1';
}

function* GenerateInspectStruct(type, members, includes) {
  for(let include of ['stdio.h', ...includes]) yield `#include <${include}>`;
  yield `${type} svar;`;
  yield `int main() {`;
  yield `  printf("${type} - %u\\n", sizeof(svar));`;
  for(let member of members)
    yield `  printf(".${member} %u %u\\n", (char*)&svar.${member} - (char*)&svar, sizeof(svar.${member}));`;
  yield `  return 0;`;
  yield `}`;
}

async function InspectStruct(type, members, includes) {
  const code = [...GenerateInspectStruct(type, members, includes)].join('\n');
  const file = `inspect-${type}-struct.c`;
  dumpFile(file, code);

  let result = await Compile(file);

  console.log('InspectStruct', { file, result });
  return result;
}

function* GenerateStructClass(name, [size, map]) {
  yield `class ${name} extends ArrayBuffer {`;
  yield `  constructor(obj = {}) {\n    super(${size});\n    Object.assign(this, obj);\n  }`;
  yield `  get [Symbol.toStringTag]() { return \`[struct ${name} @ \${this} ]\`; }`;
  let fields = [];
  for(let [name, [type, offset, size]] of map) {
    if(/reserved/.test(name)) continue;
    yield '';
    yield `  /* ${offset}: ${type} ${name}@${size} */`;
    yield* GenerateGetSet(name, offset, size).map(line => `  ${line}`);
    fields.push(name);
  }
  yield '';
  yield `  toString() {\n    const { ${fields.join(', '
  )} } = this;\n    return \`struct ${name} {${fields
    .map(field => '\\n\\t.' + field + ' = ${' + field + '}')
    .join(',')}\\n}\`;\n  }`;
  yield '}';
}

function GenerateGetSet(name, offset, size) {
  return [
    `set ${name}(v) { new ${ByteLength2TypedArray(size)}(this, ${offset})[0] = ${ByteLength2Value(size
    )}; }`,
    `get ${name}() { return new ${ByteLength2TypedArray(size)}(this, ${offset})[0]; }`
  ];
}

function ByteLength2TypedArray(byteLength) {
  switch (byteLength) {
    case 1:
      return 'Uint8Array';
    case 2:
      return 'Uint16Array';
    case 4:
      return 'Uint32Array';
    case 8:
      return 'BigUint64Array';
    default: return 'Uint8Array';
  }
}
function ByteLength2Value(byteLength) {
  switch (byteLength) {
    case 8:
      return 'BigInt(v)';
    default: return 'v';
  }
}
