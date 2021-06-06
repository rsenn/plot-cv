import PortableSpawn from './lib/spawn.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import * as deep from 'deep';
import { Console } from 'console';
import REPL from './repl.js';
import { SIZEOF_POINTER, Node, Type, RecordDecl, EnumDecl, TypedefDecl, VarDecl, FunctionDecl, Location, TypeFactory, SpawnCompiler, AstDump, NodeType, NodeName, GetLoc, GetType, GetTypeStr, NodePrinter, isNode, SourceDependencies, GetTypeNode, GetFields, PathRemoveLoc, PrintAst } from './clang-ast.js';
import Tree from './lib/tree.js';
import { Pointer } from 'pointer';
import * as Terminal from './terminal.js';
import * as ECMAScript from './lib/ecmascript.js';
import { ECMAScriptParser } from './lib/ecmascript.js';
import { lazyInitializer } from './lib/lazyInitializer.js';
import fs from 'fs';
import { extendArray } from 'util';
import { Predicate } from 'predicate';

extendArray(Array.prototype);

let params;
let files;
let spawn, base, cmdhist;
let defs, includes, libs, sources;
let libdirs = [
  '/lib',
  '/lib/i386-linux-gnu',
  '/lib/x86_64-linux-gnu',
  '/lib32',
  '/libx32',
  '/usr/lib',
  '/usr/lib/i386-linux-gnu',
  '/usr/lib/i386-linux-gnu/i686/sse2',
  '/usr/lib/i386-linux-gnu/sse2',
  '/usr/lib/x86_64-linux-gnu',
  '/usr/lib/x86_64-linux-gnu/libfakeroot',
  '/usr/lib32',
  '/usr/libx32',
  '/usr/local/lib',
  '/usr/local/lib/i386-linux-gnu',
  '/usr/local/lib/x86_64-linux-gnu'
];
let libdirs32 = libdirs.filter(d => /(32$|i[0-9]86)/.test(d));
let libdirs64 = libdirs.filter(d => !/(32$|i[0-9]86)/.test(d));

Util.define(Array.prototype, {
  findLastIndex(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this[i];
      if(predicate(x, i, this)) return i;
    }
    return -1;
  },
  findLast(predicate) {
    let i;
    if((i = this.findLastIndex(predicate)) == -1) return null;
    return this[i];
  },
  rotateRight(n) {
    this.unshift(...this.splice(n, this.length - n));
    return this;
  },
  rotateLeft(n) {
    this.push(...this.splice(0, n));
    return this;
  },
  at(index) {
    return this[Util.mod(index, this.length)];
  },
  get first() {
    return this[this.length-1];
  },
  get head() {
    return this[this.length-1];
  },
  get tail() {
    return this[this.length-1];
  },
  get last() {
    return this[this.length-1];
  }
});

async function ImportModule(modulePath, ...args) {
  let done = false;
  let moduleName = path.basename(modulePath, /\.[^.]*$/);
  return await import(modulePath).then(module => {
    done = true;
    module = Object.create(null,
      Util.getMemberNames(module, Infinity, 0).reduce(
        (acc, item) => ({
          ...acc,
          [item]: { value: module[item], enumerable: true }
        }),
        Object.getOwnPropertyDescriptors(module)
      )
    );

    if(!globalThis.modules) globalThis.modules = {};
    globalThis.modules[moduleName] = module;

    Object.assign(globalThis, module);
    return { moduleName, modulePath, module };
  });
  /*.catch(e => {
      done = true;
      return { moduleName, modulePath, module: e.message };
    })*/
}

async function CommandLine() {
  let log = console.reallog;
  let outputLog = fs.fopen('output.log', 'w+');

  console.log('outputLog', outputLog);

  let repl;
  repl = globalThis.repl = new REPL('AST');
  //console.log('repl', repl);

  repl.importModule = ImportModule;
  repl.history_set(LoadHistory(cmdhist));
  repl.directives = {
    c(...args) {
      Compile(...args);
      return false;
    },
    l(...args) {
      ProcessFile(...args);
      return false;
    }
  };
  repl.show = value => {
    if((Util.isArray(value) || value instanceof Array) &&
      Util.isObject(value.first) &&
      value.first.kind
    )
      console.log(Table(value));
    else if(typeof value == 'string') console.log(value);
    else console.log(inspect(value, { ...console.options, hideKeys: ['loc', 'range'] }));
  };
  let debugLog = fs.fopen('debug.log', 'a');
  repl.debugLog = debugLog;

  globalThis.printNode = arg => {
    console.log(NodePrinter($.data).print(arg));
  };

  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'strping' || arg.indexOf('\x1b') == -1)
        s += inspect(arg, { depth: Infinity, depth: 6, compact: false });
      else s += arg;
    }
    debugLog.puts(s + '\n');
    debugLog.flush();
  };

  repl.cleanup = () => {
    Terminal.mousetrackingDisable();
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);
    fs.writeFileSync(cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\n') + '\n')
        .join('')
    );
    console.log(`EXIT (wrote ${hist.length} history entries)`);
    std.exit(0);
  };

  Util.atexit(() => repl.cleanup());

  repl = Util.traceProxy(repl);

  await repl.run();
  console.log('REPL done');
}

function* DirIterator(...args) {
  let pred = typeof args[0] != 'string' ? Util.predicate(args.shift()) : () => true;
  for(let dir of args) {
    let entries = os.readdir(dir)[0] ?? [];
    for(let entry of entries.sort()) {
      let file = path.join(dir, entry);
      let lst = os.lstat(file)[0];
      let st = os.stat(file)[0];
      let is_dir = (st?.mode & os.S_IFMT) == os.S_IFDIR;
      let is_symlink = (lst?.mode & os.S_IFMT) == os.S_IFLNK;

      if(is_dir) file += '/';
      if(!pred(entry, file, is_dir, is_symlink)) continue;
      yield file;
    }
  }
}

function* RecursiveDirIterator(dir, maxDepth = Infinity, pred = (entry, file, dir) => true) {
  if(!dir.endsWith('/')) dir += '/';
  for(let file of fs.readdir(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;
    let entry = `${dir}${file}`;
    let isDir = false;
    let st = fs.stat(entry);
    isDir = st && st.isDirectory();
    if(isDir) entry += '/';
    let show = pred(entry, file, dir);
    if(show) yield entry;
    if(maxDepth > 0 && isDir) yield* RecursiveDirIterator(entry, maxDepth - 1, pred);
  }
}

function* IncludeAll(dir, maxDepth = Infinity, pred = entry => /\.[ch]$/.test(entry)) {
  for(let entry of RecursiveDirIterator(dir, maxDepth, pred)) yield `#include "${entry}"`;
}

function SelectLocations(node) {
  let result = deep.select(node, n =>
    ['offset', 'line', 'file'].some(prop => n[prop] !== undefined)
  );
  //console.log('result:', console.config({ depth: 1 }), result);
  return result;
}

function LocationString(loc) {
  if(typeof loc == 'object' && loc != null) {
    let file = loc.includedFrom ? loc.includedFrom.file : loc.file;
    if(typeof loc.line == 'number')
      return `${file ? file + ':' : ''}${loc.line}${
        typeof loc.col == 'number' ? ':' + loc.col : ''
      }`;
    return `${file ? file : ''}@${loc.offset}`;
  }
}

const TypeMap = Util.weakMapper(node => new Type(node));

function Structs(nodes) {
  return nodes
    .filter(node => node.inner && node.inner.some(field => field.kind == 'FieldDecl'))
    .map(node => [
      //deep.find(node, n => typeof n.line == 'number'),
      new Location(GetLoc(node)),
      ((node.tagUsed ? node.tagUsed + ' ' : '') + (node.name ?? '')).trim(),
      new Map(node.inner.map((field, i) =>
          /Attr/.test(field.kind)
            ? [Symbol(field.kind), field.id]
            : [field.name || i, (field.type && TypeFactory(field.type)) || field.kind]
        )
      )
    ]);
  /*.map(node => types(node))*/
}

function Table(list, pred = (n, l) => true /*/\.c:/.test(l)*/) {
  let entries = [...list]
    .map((n, i) => (n ? [i, LocationString(GetLoc(n)), n] : undefined))
    .filter(e => e);
  const colSizes = [5, 15, 12, 30, 12, 15, 25];
  const colKeys = ['id', 'kind', 'name', 'tagUsed' /*, 'previousDecl', 'completeDefinition'*/];
  const colNames = ['#', ...colKeys, 'location'];
  const outputRow = (cols, pad, sep) =>
    cols
      .map((s, col) => (s + '')[`pad${col ? 'End' : 'Start'}`](colSizes[col] ?? 30, pad ?? ' '))
      .join(sep ?? '│ ')
      .trimEnd();
  return (outputRow(colNames) +
    '\n' +
    outputRow(colNames.map(n => ''),
      '─',
      '┼─'
    ) +
    '\n' +
    entries
      .filter(([i, l, n]) => pred(n, l))
      .reduce((acc, [i, l, n]) => {
        let row = colKeys.map(k => n[k] ?? '');
        row.unshift(i);
        row.push(l);
        acc.push(outputRow(row));
        return acc;
      }, [])
      .join('\n')
  );
}

function PrintRange(range) {
  if('range' in range) range = range.range;

  const { begin, end } = range;

  let data = fs.readFileSync(begin.file, 'utf-8');
  return data ? data.slice(begin.offset, end.offset + (end.tokLen | 0)) : null;
}

function OverlapRange(r1, r2) {
  const GetRange = r => [r.begin.offset, r.end.offset + (r.end.tokLen | 0)];
  const InRange = (i, r) => i >= r[0] && i < r[1];

  r1 = GetRange(r1);
  r2 = GetRange(r2);

  if(InRange(r1[0], r2) || InRange(r1[1], r2)) return true;
  if(InRange(r2[0], r1) || InRange(r2[1], r1)) return true;

  return false;
}

function ReadFile(name, binary) {
  let ret = fs.readFileSync(name, binary ? null : 'utf-8');

  console.log(`Read ${name}: ${ret.length} bytes`);
  return ret;
}

function Terminate(exitCode) {
  console.log('Terminate', exitCode);

  Util.exit(exitCode);
}

function LoadHistory(filename) {
  let contents = std.loadFile(filename);
  let data;

  const parse = () => {
    try {
      data = JSON.parse(contents);
    } catch(e) {}
    if(data) return data;
    try {
      data = contents.split(/\n/g);
    } catch(e) {}
    if(data) return data;
  };

  return (parse() ?? [])
    .filter(entry => (entry + '').trim() != '')
    .map(entry => entry.replace(/\\n/g, '\n'));
}

function ReadJSON(filename) {
  let data = std.loadFile(filename);

  if(data) console.log(`ReadJSON('${filename}') ${data.length} bytes read`);
  return data ? JSON.parse(data) : null;
}
function MapFile(filename) {
  let fd = os.open(filename, os.O_RDONLY);
  let { size } = os.stat(filename)[0];
  console.log(`MapFile`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  os.close(fd);
  return data;
}
function ReadBJSON(filename) {
  let fd = os.open(filename, os.O_RDONLY);
  let { size } = os.stat(filename)[0];
  console.log(`ReadBJSON`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  console.log(`ReadBJSON`, { data });
  let ret = bjson.read(data, 0, size);

  mmap.munmap(data);
  os.close(fd);
  return ret;
}

function WriteFile(name, data, verbose = true) {
  if(Util.isIterator(data)) data = [...data];
  if(Util.isArray(data)) data = data.join('\n');

  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  console.log(`fs`, fs);
  let ret = fs.writeFileSync(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function WriteJSON(name, data) {
  WriteFile(name, JSON.stringify(data, null, 2));
}

function WriteBJSON(name, data) {
  let buf = bjson.write(data);
  let size = buf.byteLength;
  let fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_TRUNC);

  let ret = os.write(fd, buf, 0, size);
  console.log('WriteBJSON', { name, fd, size, ret });
  os.close(fd);

  return ret;
}

function* GenerateInspectStruct(decl, includes) {
  let { name, members } = decl;

  includes ??= [decl.loc.file.replace(/^\/usr\/include\//, '')];

  //console.log('GenerateInspectStruct', { name, members, includes });

  yield '#include <stdio.h>';
  yield '#include <stddef.h>';

  if(MemberNames(members).some(name => /:/.test(name)))
    yield `
size_t
byte_firstnot(const void* p, size_t len, unsigned char v) {
  const unsigned char* x;
  for(x = p; len; len--, x++)
    if(*x != v) break;
  return x - (const unsigned char*)p;
}
size_t
byte_lastnot(const void* p, size_t len, unsigned char v) {
  const unsigned char* x;
  for(x = (const unsigned char*)p + len - 1; len; len--, x--)
    if(*x != v) break;
  return x - (const unsigned char*)p;
}
size_t
bit_firstnot(unsigned char v, unsigned char b) {
  int i;
  for(i = 0; i < 8; i++, v >>= 1) 
    if((v & 1) == !b) break;
  return i;
}
size_t
bit_lastnot(unsigned char v, unsigned char b) {
  int i;
  for(i = 7; i >= 0; i--) 
    if(!!(v & (1 << i)) == !b) break;
  return i >= 0 ? i : 8;
}
size_t
firstnot(const void* p, size_t len, unsigned char v) {
 const char* x = p;
 size_t i = byte_firstnot(p, len, v);
 return i * 8 + bit_firstnot(x[i], v);
}
size_t
lastnot(const void* p, size_t len, unsigned char v) {
 const unsigned char* x = p;
 size_t i = byte_lastnot(p, len, v);
 return i * 8 + bit_lastnot(x[i], v);
}
size_t
bitsize(const void* p, size_t len) {
 return lastnot(p, len, 0xff) + 1 - firstnot(p, len, 0xff);
}
`;

  for(let include of includes) yield `#include "${include}"`;
  yield `${name} svar;`;
  yield `int main() {`;
  yield `  printf("${name} %zu\\n", sizeof(svar) * 8);`;

  for(let member of MemberNames(members)) {
    if(true /*(type == null || typeof type.size == 'number') && member != undefined*/) {
      let field = member.replace(/:.*/, '');
      if(/:/.test(member)) {
        yield `
  memset(&svar, 0xff, sizeof(svar));
  svar.${field} = 0;
  printf(".${field} %zu %zi\\n", firstnot(&svar, sizeof(svar), 0xff), bitsize(&svar, sizeof(svar)));`;
      } else {
        yield `  printf(".${field} %zu %zu\\n", offsetof(${name}, ${field}) * 8, sizeof(svar.${field}) * 8);`;
      }
    }
  }

  yield `  return 0;`;
  yield `}`;
}

function InspectStruct(decl, includes, compiler = 'clang') {
  if(typeof decl == 'string') {
    let name = (/ /.test(decl) ? '' : 'struct ') + decl;
    decl = Type.get(name, $.data);
    decl ??= $.getType(name);
    decl.name = name;
  }
  //console.log('InspectStruct',decl);

  const code = [...GenerateInspectStruct(decl, includes)].join('\n');
  const program = `/tmp/inspect-${decl.name.replace(/\ /g, '_')}`;
  WriteFile(program + '.c', code);

  let command = [compiler, '-O2', '-g', '-w', '-o', program, program + '.c', ...flags];
  console.log('InspectStruct', { command: command.join(' ') });

  let result = os.exec(command);

  if(result == 0) {
    let [fd, stdout] = os.pipe();

    os.exec([(program.startsWith('/') ? '' : `./`) + program], { stdout });
    let output = fs.readAll(fd);

    let lines = output.trim().split('\n');
    let firstLine = lines.shift();

    let [name, size] = [...Util.splitAt(firstLine, [...firstLine].lastIndexOf(' '))];

    name = name.replace(/^(struct|union|enum)\ /, '');

    //console.log("lines:", lines);
    result = lines
      .map(line => (typeof line == 'string' ? line.split(' ') : line))
      .map(line => line.map((col, i) => (isNaN(+col) ? col : +col)))
      .map(([field, offset, size]) => [
        field.replace(/:.*/, '').replace(/^\./, name + '.'),
        offset,
        size
      ]);

    let end = 0;
    result = result.reduce((acc, line) => {
      if(acc.length) {
        if(end < acc.last[1] + acc.last[2]) end = acc.last[1] + acc.last[2];

        if(end < line[1]) acc.push([null, end, line[1] - end]);
      }
      acc.push(line);
      return acc;
    }, []);

    result.unshift([name, '-', +size]);

    Util.define(result, {
      toString(sep = ' ') {
        return this.map(line => line.join(sep).replace('.', ' ')).join('\n');
      }
    });
  }

  return result;
}

function RoundTo(value, align) {
  return Math.floor((value + (align - 1)) / align) * align;
}

function MakeStructClass(decl, filename) {
  let code = [...GenerateStructClass(decl)].join('\n');

  WriteFile((filename ?? decl.name.replace(/[^A-Za-z0-9_]/g, '-')) + '.js', code);

  if(!filename) return code;
}

function* GenerateStructClass(decl, ffiPrefix = '') {
  let name;
  if(decl instanceof TypedefDecl) {
    name = decl.name;
    decl = decl.type;
  }

  let { size, members } = decl;
  name ??= decl.name;

  let className = name.replace(/struct\s*/, '');
  yield `class ${className} extends ArrayBuffer {`;
  yield `  constructor(obj = {}) {\n    super(${size});\n    Object.assign(this, obj);\n  }`;
  yield `  get [Symbol.toStringTag]() { return \`[${name} @ \${this} ]\`; }`;
  let fields = [];
  let offset = 0;

  // console.log('GenerateStructClass', decl);
  for(let [name, type] of members) {
    if(/reserved/.test(name)) continue;

    if(type.size == 8) offset = RoundTo(offset, 8);
    let desugared = type.desugared && type.desugared != type ? ` (${type.desugared})` : '';
    let pointer = type.pointer;

    yield '';
    let subscript = type.subscript ?? '';
    yield `  /* ${offset}: ${type}${desugared} ${name}${subscript} */`;
    yield* GenerateGetSet(name, offset, type, ffiPrefix).map(line => `  ${line}`);
    fields.push(name);
    offset += RoundTo(type.size, 4);
  }
  yield '';
  yield `  static from(address) {\n    let ret = ${ffiPrefix}toArrayBuffer(address, ${offset});\n    return Object.setPrototypeOf(ret, ${className}.prototype);\n  }`;
  yield '';

  yield `  toString() {\n    const { ${fields.join(', ')} } = this;\n    return \`${name} {${[
    ...members
  ]
    .map(([field, member]) =>
        '\\n\\t.' +
        field +
        ' = ' +
        (member.isPointer() ? '0x' : '') +
        '${' +
        field +
        (member.isPointer() ? '.toString(16)' : '') +
        '}'
    )
    .join(',')}\\n}\`;\n  }`;
  yield '}';
}

function GenerateGetSet(name, offset, type, ffiPrefix) {
  const { size, signed } = type;
  const floating = type.isFloatingPoint();
  const pointer = type.getPointer($.data);
  let ctor = ByteLength2TypedArray(size, signed, floating);
  let toHex = v => v;
  if(type.isPointer()) toHex = v => `'0x'+${v}.toString(16)`;

  let a = [];

  if(pointer) {
    let { name, size, signed, desugared } = pointer;
    a.unshift(`/* ${name}${desugared ? ` (${desugared})` : ''} ${size} ${signed} */`);
    console.log('GenerateStructClass', { pointer });
  }
  return [
    ...a,
    `set ${name}(value) { if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = ${ffiPrefix}toPointer(value); new ${ctor}(this, ${offset})[0] = ${ByteLength2Value(size,
      signed,
      floating
    )}; }`,
    `get ${name}() { return ${toHex(`new ${ctor}(this, ${offset})[0]`)}; }`
  ];
}

function ByteLength2TypedArray(byteLength, signed, floating) {
  if(floating) {
    switch (byteLength) {
      case 4:
        return 'Float32Array';
      case 8:
        return 'Float64Array';
      default: throw new Error(`Floating point, but ${byteLength} size`);
    }
  }
  switch (byteLength) {
    case 1:
      return signed ? 'Int8Array' : 'Uint8Array';
    case 2:
      return signed ? 'Int16Array' : 'Uint16Array';
    case 4:
      return signed ? 'Int32Array' : 'Uint32Array';
    case 8:
      return signed ? 'BigInt64Array' : 'BigUint64Array';
    default: return signed ? 'Int8Array' : 'Uint8Array';
  }
}

function ByteLength2Value(byteLength, signed, floating) {
  if(byteLength == 8 && !floating) return 'BigInt(value)';
  return 'value';
}

export class FFI_Function {
  constructor(node, prefix = '') {
    const { name, returnType, parameters } = node;
    this.name = name;
    this.prefix = prefix;
    this.returnType = returnType.ffi;
    this.parameters = [...(parameters ?? [])].map(([name, type]) => [name, type.ffi]);
  }

  generateDefine(fp, lib) {
    const { prefix, name, returnType, parameters } = this;
    fp ??= (name, lib) => `${prefix}dlsym(${lib ?? 'RTLD_DEFAULT'}, '${name}')`;
    let code = `'${name}', ${fp(name, lib)}, null, '${returnType}'`;
    for(let [name, type] of parameters) {
      console.log('name:', name, 'type:', type);
      code += ', ';
      code += `'${type}'`;
    }
    return `${prefix}define(${code});`;
  }

  generateCall() {
    const { prefix, name, returnType, parameters } = this;
    const paramNames = parameters.map(([name, type]) => name);
    let code = `function ${name}(${paramNames.join(', ')}) {\n`;
    code += `  return ${prefix}call('${name}', ${paramNames.join(', ')});\n`;
    code += `}`;
    return code;
  }

  generate(fp, lib, exp) {
    return [
      this.generateDefine(fp, lib),
      '\n',
      exp ? 'export ' : '',
      this.generateCall(),
      '\n'
    ].join('');
  }

  generateFunction(fp, lib) {
    const { prefix, name, returnType, parameters } = this;
    const paramNames = parameters.map(([name, type]) => name);
    let code = `new Function(${paramNames.map(p => `'${p}'`).join(', ')}, `;

    code += `'return ${prefix}call("${name}"${paramNames.map(p => `, ${p}`).join('')})');`;
    return code;
  }

  compileFunction(fp, lib) {
    let code = this.generateCall(fp, lib);
    let fn = new Function(`return ${code}`);
    return fn();
  }
}
function FdReader(fd, bufferSize = 1024) {
  let buf = fs.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await fs.waitRead(fd);
      ret = fs.read(fd, buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(fs.bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    fs.close(fd);
  });
}

export async function CommandRead(args) {
  let child = spawn(args, {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });
  let output = '';
  let done = false;
  let buf = new ArrayBuffer(1024);
  if(Util.platform == 'quickjs') {
    let { fd } = child.stdout;
    //return await (async function() {
    for(;;) {
      1;
      let r;
      await fs.waitRead(fd);
      r = ReadOutput(fd);
      if(r > 0 && r < buf.byteLength) break;
    }
    let result = await child.wait();
    //console.log('child.wait():', result);
    return output.trimEnd();
    //})();
  } else {
    AcquireReader(child.stdout, async reader => {
      let r;
      while((r = await reader.read())) {
        if(!r.done) errors += r.value.toString();
      }
    });
  }
  function ReadOutput(fd) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    output += fs.bufferToString(buf.slice(0, r));
    //if(r > 0) console.log('r:', r, 'output:', output.slice(-100));
    return r;
  }
}

export async function LibraryExports(file) {
  console.log(`LibraryExports:`, file);

  let output = await CommandRead(['/opt/diet/bin/objdump', '-T', file]);
  output = output.replace(/.*DYNAMIC SYMBOL TABLE:\s/m, '');
  let lines = output.split(/\n/g).filter(line => /\sBase\s/.test(line));
  let columns = Util.colIndexes(lines[0]);

  let entries = lines.map(line => Util.colSplit(line, columns).map(column => column.trimEnd()));
  entries.sort((a, b) => a[0].localeCompare(b[0]));

  return entries.map(entry => entry[entry.length - 1].trimStart());
}

function ProcessFile(file, debug = true) {
  const ext = path.extname(file);
  let ret = null;
  switch (ext) {
    case '.js':
      ret = ParseECMAScript(file, debug);
      break;
    case '.c':
    case '.h':
      ret = Compile(file, debug);
      break;
  }
  return ret;
}

function ParseECMAScript(file, debug = false) {
  let data = fs.readFileSync(file, 'utf-8');
  let ast, error;
  let parser;
  globalThis.parser = parser = new ECMAScriptParser(data?.toString ? data.toString() : data,
    file,
    debug
  );

  globalThis.ast = ast = parser.parseProgram();
  parser.addCommentsToNodes(ast);

  globalThis.files[file] = ast;

  return ast;
}

function PrintECMAScript(ast, comments, printer = new ECMAScript.Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

function Namespaces(nodePath, ast = $.data) {
  let ptr = new Pointer(nodePath);
  let ptrs = ptr.chain(2);
  let get = deep.get(ast);

  let ns = ptrs.map(p => get(p)).filter(n => n.kind == 'NamespaceDecl');
  return ns;
}

MemberNames.UPPER = 1;
MemberNames.METHODS = 2;
MemberNames.PROPERTIES = 4;

function MemberNames(members, flags = 0) {
  let ret = [];
  if(members.members) members = members.members;

  if(!Array.isArray(members)) {
    for(let ptr of deep
      .select(members, n => n.kind.endsWith('Decl') && n.name, deep.RETURN_PATH)
      .map(path => new Pointer(path))) {
      let ptrs = ptr.chain(2);
      console.log('ptrs:', ptrs);
      let names = ptrs.map(p => deep.get(members, [...p, 'name'], deep.NO_THROW));
      let kinds = ptrs.map(p => deep.get(members, [...p, 'kind'], deep.NO_THROW));
      console.log('kinds:', kinds);
      console.log('names:', names);
      ret.push(names.filter(name => name).join('.'));
    }
  } else {
    for(let ptr of deep
      .select(members,
        n => Array.isArray(n) && n.length == 2 && typeof n[0] == 'string' && n[1] !== null,
        deep.RETURN_PATH
      )
      .map(path => new Pointer(path))) {
      let ptrs = ptr.chain(3);
      let names = ptrs.map(p => deep.get(members, [...p, 0]));
      ret.push(names.filter(name => name).join('.'));
    }
  }
  if(flags & MemberNames.UPPER) {
    ret = ret.map(name => Util.decamelize(name, '_').toUpperCase());
  }
  return ret;
}

function UnsetLoc(node, pred = (v, p) => true) {
  for(let [v, p] of deep.select(node,
    (v, k) => k == 'loc' || k == 'range',
    deep.RETURN_VALUE_PATH
  )) {
    console.log('UnsetLoc', { v, p });
    if(pred(deep.get(node, [...p].slice(0, -1)), [...p].last)) deep.unset(node, p);
  }
  return node;
}


function MakeFFI(node, fp, lib, exp) {
  if(Array.isArray(node)) {
    let out = '';
    let i = 0;
    for(let item of node) {
      console.log(`MakeFFI item #${i + 1}/${node.length}`);
      try {
        let ret = MakeFFI(item, fp, lib, exp);
        if(typeof ret == 'string' && ret.length > 0) {
          if(out) out += '\n';
          out += ret;
        }
      } catch(error) {
        console.log(`ERROR item [${i}]:`, error.message);
      }
      i++;
    }
    return out;
  }
  if(!(node instanceof Node)) node = TypeFactory(node, $.data);

  if(typeof node == 'object' && node && node.kind == 'FunctionDecl') node = new FunctionDecl(node);

  if(node instanceof FunctionDecl) {
    let ffi = new FFI_Function(node);

    let protoStr = PrintAst(node.ast, $.data).split(/\n/g)[0].replace(/\ {$/, ';');
    protoStr = protoStr.replace(/^\s*extern\s+/, '');
    return `/* ${protoStr} */\n` + ffi.generate(fp, lib, exp);
  } else if(node instanceof RecordDecl || node instanceof TypedefDecl) {
    let code = [...GenerateStructClass(node)].join('\n');
    return code;
  }
}

async function ASTShell(...args) {
  let inspectOptions = {
    /*breakLength: 240, */ customInspect: true,
    compact: false,
    depth: Infinity,
    maxArrayLength: Infinity,
    hideKeys: ['loc', 'range']
  };
  globalThis.console = new Console({ inspectOptions });
  await PortableSpawn(fn => (spawn = fn));

  await import('bjson').then(module => {
    const { read, write } = module;
    globalThis.bjson = { read, write };
  });
  console.log('bjson:', globalThis.bjson);
  await import('mmap').then(module => {
    const {
      mmap,
      munmap,
      toString,
      PROT_READ,
      PROT_WRITE,
      PROT_EXEC,
      PROT_SEM,
      PROT_NONE,
      PROT_GROWSDOWN,
      PROT_GROWSUP,
      MAP_SHARED,
      MAP_PRIVATE,
      MAP_TYPE,
      MAP_FIXED,
      MAP_ANONYMOUS,
      MAP_GROWSDOWN,
      MAP_DENYWRITE,
      MAP_EXECUTABLE,
      MAP_LOCKED,
      MAP_NORESERVE,
      MAP_POPULATE,
      MAP_NONBLOCK,
      MAP_STACK
    } = module;
    globalThis.mmap = {
      mmap,
      munmap,
      toString,
      PROT_READ,
      PROT_WRITE,
      PROT_EXEC,
      PROT_SEM,
      PROT_NONE,
      PROT_GROWSDOWN,
      PROT_GROWSUP,
      MAP_SHARED,
      MAP_PRIVATE,
      MAP_TYPE,
      MAP_FIXED,
      MAP_ANONYMOUS,
      MAP_GROWSDOWN,
      MAP_DENYWRITE,
      MAP_EXECUTABLE,
      MAP_LOCKED,
      MAP_NORESERVE,
      MAP_POPULATE,
      MAP_NONBLOCK,
      MAP_STACK
    };
  });
  console.log('mmap:', globalThis.mmap);

  /*  console.options = consoleOptions;
  console.options.compact = 1;
  console.options.hideKeys = ['loc', 'range'];*/
  console.options.depth = 2;
  console.options.compact = 1;

  globalThis.files = files = {};

  const platform = Util.getPlatform();
  console.log('platform:', platform);
  if(platform == 'quickjs') await import('std').then(module => (globalThis.std = module));

  if(platform == 'node')
    await import('util').then(module => (globalThis.inspect = module.inspect));

  (await Util.getPlatform()) == 'quickjs'
    ? import('deep.so').then(module => (globalThis.deep = module))
    : import('./lib/deep.js').then(module => (globalThis.deep = module['default']));

  base = path.basename(Util.getArgv()[1], /\.[^.]*$/);
  cmdhist = `.${base}-cmdhistory`;

  params = globalThis.params = Util.getOpt({
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      libs: [true, (a, p) => (p || []).concat([a]), 'l'],
      debug: [false, null, 'x'],
      force: [false, null, 'f'],
      'system-includes': [false, null, 's'],
      'no-remove-empty': [false, null, 'E'],
      'output-dir': [true, null, 'd'],
      compiler: ['clang', null, 'c'],
      '@': 'input'
    },
    args
  );

  defs = params.define || [];
  includes = params.include || [];
  libs = params.libs || [];
  sources = params['@'] || [];

  Util.define(globalThis, {
    defs,
    includes,
    libs,
    get flags() {
      return [...includes.map(v => `-I${v}`), ...defs.map(d => `-D${d}`), ...libs.map(l => `-l${l}`)];
    }
  });

  async function Compile(file, ...args) {
    let r = await AstDump(params.compiler, file, [...globalThis.flags, ...args], params.force);
    r.source = file;

    globalThis.files[file] = r;

    function nameOrIdPred(name_or_id, pred = n => true) {
      if(typeof name_or_id == 'number') name_or_id = '0x' + name_or_id.toString(16);
      return name_or_id instanceof RegExp
        ? node => name_or_id.test(node.name) && pred(node)
        : name_or_id.startsWith('0x')
        ? node => node.id == name_or_id && pred(node)
        : node => node.name == name_or_id && pred(node);
    }

    Object.assign(r, {
      select(name_or_id, pred = n => true) {
        return this.data.inner.filter(nameOrIdPred(name_or_id, pred));
      },
      getByIdOrName(name_or_id, pred = n => true) {
        let node = this.data.inner.findLast(nameOrIdPred(name_or_id, pred));

        node ??= this.classes.findLast(nameOrIdPred(name_or_id, pred));
        return node;
      },
      getType(name_or_id) {
        let result =
          this.getByIdOrName(name_or_id,
            n => !/(FunctionDecl)/.test(n.kind) && /Decl/.test(n.kind)
          ) ?? GetType(name_or_id, this.data);

        if(result) {
          let type = TypeFactory(result, this.data);
          if(type) result = type;
        }
        return result;
      },

      getFunction(name_or_id) {
        let result = isNode(name_or_id)
          ? name_or_id
          : this.getByIdOrName(name_or_id, n => /(FunctionDecl)/.test(n.kind));

        if(result) return new FunctionDecl(result, this.data);
      },
      getVariable(name_or_id) {
        let result = isNode(name_or_id)
          ? name_or_id
          : this.getByIdOrName(name_or_id, n => /(VarDecl)/.test(n.kind));

        if(result) return new VarDecl(result, this.data);
      }
    });
    return Util.define(r, {
      pathOf(needle, maxDepth = 10) {
        for(let [node, path] of deep.iterate(r.data,
          n => typeof n == 'object' && n != null,
          deep.RETURN_VALUE_PATH,
          maxDepth
        )) {
          //console.log("pathOf",console.config({depth:1}),{node,path});
          if(node === needle) return new Pointer(path);
        }
      }
      /*tree() {
        return new Tree(this.data);
      }*/
    });
  }

  Object.assign(globalThis, {
    SIZEOF_POINTER,
    Type,
    AstDump,
    SourceDependencies,
    NodePrinter,
    NodeType,
    NodeName,
    GetLoc,
    GetTypeStr,
    ReadFile,
    MapFile,
    ReadJSON,
    ReadBJSON,
    WriteFile,
    WriteJSON,
    WriteBJSON,
    PrintRange,
    OverlapRange,
    GenerateInspectStruct,
    GenerateStructClass,
    InspectStruct,
    MakeStructClass,
    DirIterator,
    RecursiveDirIterator,
    IncludeAll,
    Terminal,
    PrintAst,
    MakeFFI,
    ParseECMAScript,
    PrintECMAScript,
    ProcessFile
  });

  Pointer.prototype.chain = function(step, limit = Infinity) {
    let ptr = this;
    let ret = [];
    let len = ptr.length;
    for(;;) {
      if(ret.length >= limit) break;
      ret.unshift(ptr);
      len -= step;
      if(len <= 0) break;
      ptr = ptr.slice(0, -step);
    }
    return ret;
  };

  Object.assign(globalThis, {
    Pointer,
    Predicate,
    Tree,
    deep,
    Compile,
    SelectLocations,
    LocationString,
    Table,
    Structs,
    Node,
    Type,
    RecordDecl,
    EnumDecl,
    TypedefDecl,
    FunctionDecl,
    Location,
    TypeFactory,
    SpawnCompiler,
    AstDump,
    NodeType,
    NodeName,
    GetLoc,
    GetType,
    GetTypeStr,
    GetTypeNode,
    GetFields,
    PathRemoveLoc,
    FFI_Function,
    libdirs,
    libdirs32,
    libdirs64,
    LibraryExports,
    MemberNames,
    Namespaces,
    UnsetLoc
  });
  globalThis.util = Util;
  globalThis.F = arg => $.getFunction(arg);
  globalThis.T = arg => $.getType(arg);

  Util.lazyProperty(globalThis, 'P', () => {
    let printer = NodePrinter($.data);

    return node => {
      if('ast' in node) node = node.ast;
      printer.clear();
      return printer.print(node);
    };
  });

  const unithist = `.${base}-unithistory`;
  let items = [];
  let hist = ReadJSON(unithist) || [];

  const pushUnique = (arr, item) => {
    if(Util.findIndex(arr, elem => deep.equals(elem, item)) === -1) {
      arr.push(item);
      return true;
    }
  };

  for(let source of sources) {
    let item;
    if(/\.js$/.test(source)) item = ParseECMAScript(source);
    else item = await Compile(source);
    if(item) {
      pushUnique(hist, [...flags, source]);
      items.push(item);
    }
  }
  WriteFile(unithist, JSON.stringify(hist, null, 2));

  globalThis.$ = items.length == 1 ? items[0] : items;
  console.log('CommandLine');
  await CommandLine();
}

Util.callMain(ASTShell, true);
