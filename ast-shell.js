import { AstDump, CompleteLocation, CompleteRange, EnumDecl, FindType, FunctionDecl, GetFields, GetLoc, GetParams, GetType, GetTypeNode, GetTypeStr, Hier, isNode, List, Location, Node, NodeName, NodePrinter, NodeType, PathOf, PathRemoveLoc, PrintAst, Range, RawLocation, RawRange, RecordDecl, SIZEOF_POINTER, SourceDependencies, SpawnCompiler, Type, TypedefDecl, TypeFactory, VarDecl } from './clang-ast.js';
import { DirIterator, RecursiveDirIterator } from './dir-helpers.js';
import { LoadHistory, ReadFile, ReadJSON, WriteFile, WriteJSON } from './io-helpers.js';
import * as deep from './lib/deep.js';
import * as ECMAScript from './lib/ecmascript.js';
import * as fs from './lib/filesystem.js';
import { define, defineGetter, getOpt, isObject, lazyProperty, memoize, pushUnique, toArrayBuffer, toString, weakMapper } from './lib/misc.js';
import { extendArray } from 'extendArray';
import * as path from './lib/path.js';
import { Pointer } from './lib/pointer.js';
import Tree from './lib/tree.js';
import Util from './lib/util.js';
import { Shell, Spawn } from './os-helpers.js';
import * as Terminal from './terminal.js';
import { Console } from 'console';
import { REPL } from 'repl';
import { inspect } from 'inspect';
//import PortableSpawn from './lib/spawn.js';

extendArray(Array.prototype);

globalThis.fs = fs;

let params;
let files;
let spawn, base, cmdhist, config;
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

const ConcatIterator = iterator => {
  let result,
    s = '';

  if(!('next' in iterator)) {
    if(Symbol.iterator in iterator) iterator = iterator[Symbol.iterator]();
  }

  while((result = iterator.next()) && !result.done) {
    if(s != '') s += '\n';
    s += result.value;
  }

  return s;
};

const StringGenerator =
  gen =>
  (...args) => {
    const iterator = gen(...args);

    return Object.assign(iterator, {
      toString() {
        return ConcatIterator(this);
      }
    });
  };

async function ImportModule(modulePath, ...args) {
  let done = false,
    moduleName = path.basename(modulePath, '.js');

  return await import(modulePath).then(module => {
    done = true;
    module = Object.create(
      null,
      getMemberNames(module, Infinity, 0).reduce(
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

function CommandLine() {
  let log = console.reallog;
  // let outputLog = fs.openSync('output.log', 'w+');

  let repl;
  repl = globalThis.repl = new REPL('AST', false);
  //console.log('repl', repl);

  let cfg = ReadJSON(config);

  if(cfg) Object.assign(console.options, cfg.inspectOptions);

  repl.importModule = ImportModule;
  repl.history = LoadHistory(cmdhist);
  Object.assign(repl.directives, {
    c: [
      (...args) => {
        Compile(...args);
        return false;
      },
      'compile source'
    ],
    l: [
      (...args) => {
        ProcessFile(...args);
        return false;
      },
      'load source file'
    ],
    i: [
      (module, ...args) => {
        console.log('args', args);
        try {
          return require(module);
        } catch(e) {}
        import(module).then(m => (globalThis[module] = m));
      },
      'import module'
    ]
  });
  repl.inspectOptions = console.options;
  repl.show = value => {
    let first, str;
    if(isObject(value) && (first = value.first ?? value[0]) && isObject(first) && ('id' in first || 'kind' in first)) str = Table(value);
    else if(typeof value == 'string') str = value;
    else
      str = inspect(value, {
        ...(cfg?.inspectOptions ?? {}),
        ...repl.inspectOptions,
        hideKeys: ['loc', 'range']
      });
    std.out.puts(str + '\n');
  };

  let debugLog = fs.openSync('debug.log', 'a');
  repl.debugLog = debugLog;

  globalThis.printNode = arg => {
    console.log(NodePrinter($.data).print(arg));
  };

  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'strping' || arg.indexOf('\x1b') == -1) s += inspect(arg, { depth: Infinity, depth: 6, compact: false });
      else s += arg;
    }
    fs.writeSync(debugLog, s + '\n');
    if(debugLog.flush) debugLog.flush();
  };

  repl.addCleanupHandler(() => {
    Terminal.mousetrackingDisable();
    let hist = repl.history.filter((item, i, a) => a.lastIndexOf(item) == i);
    WriteFile(
      cmdhist,
      hist
        .filter(entry => (entry + '').trim() != '')
        .map(entry => entry.replace(/\n/g, '\\n') + '\n')
        .join('')
    );

    let cfg = { inspectOptions: console.options };
    WriteJSON(config, cfg);

    console.log(`EXIT (wrote ${hist.length} history entries)`);
    std.exit(0);
  });

  //atexit(() => repl.cleanup());

  repl = traceProxy(repl);

  if(params.exec) repl.evalAndPrint(params.exec);
  else repl.run(false);
}

function* IncludeAll(dir, maxDepth = Infinity, pred = entry => /\.[ch]$/.test(entry)) {
  for(let entry of RecursiveDirIterator(dir, maxDepth, pred)) yield `#include "${entry}"`;
}

function SelectLocations(node) {
  return deep.select(node, n => ['offset', 'line', 'file'].some(prop => n[prop] !== undefined));
}

function LocationString(loc) {
  if(typeof loc == 'object' && loc != null) {
    let file = loc.file ?? (loc.includedFrom && loc.includedFrom.file);

    if(typeof loc.line == 'number') return `${file ? file + ':' : ''}${loc.line}${typeof loc.col == 'number' ? ':' + loc.col : ''}`;
    return `${file ? file : ''}@${loc.offset}`;
  }
}

const TypeMap = weakMapper(node => new Type(node));

function Structs(nodes) {
  return nodes
    .filter(node => node.inner && node.inner.some(field => field.kind == 'FieldDecl'))
    .map(node => [
      //deep.find(node, n => typeof n.line == 'number'),
      new Location(GetLoc(node)),
      ((node.tagUsed ? node.tagUsed + ' ' : '') + (node.name ?? '')).trim(),
      new Map(node.inner.map((field, i) => (/Attr/.test(field.kind) ? [Symbol(field.kind), field.id] : [field.name || i, (field.type && TypeFactory(field.type)) || field.kind])))
    ]);
  /*.map(node => types(node))*/
}

function Table(list, pred = (n, l) => true) {
  let entries = [...list].map((n, i) => (n ? [i, LocationString(GetLoc(n)), n] : undefined)).filter(e => e);
  let typeKey = 'kind' in list[0] ? 'kind' : 'type';
  let keys = ['id', typeKey, 'name'].filter(k => !!k);
  let items = entries.filter(([i, l, n]) => pred(n, l));
  const first = items[0][2];

  if(/Function/.test(first[typeKey])) {
    keys = [
      ...keys,
      function returnType(n) {
        if(n.type && typeof n.type == 'object') {
          const { qualType } = n.type;
          return qualType.replace(/\s*\(.*$/g, '');
        }
        if(n[typeKey]) return n[typeKey];
      },
      function numArgs(n) {
        let params = GetParams(n);
        return params ? params.length : undefined;
      },
      function Params(n) {
        let params = GetParams(n);
        return params ? params.map(p => PrintAst(p)).join(',') : undefined;
      }
    ];
  }
  keys = ['n', ...keys, 'location'];
  const names = keys.map(k => (typeof k == 'function' ? k.name : k));
  let rows = items.map(([i, l, n]) => Object.fromEntries([['n', i], ...keys.slice(1, -1).map((k, j) => [names[j + 1], (typeof k == 'string' ? n[k] : k(n)) ?? '']), ['location', l]]));
  let sizes = {};

  for(let row of rows) {
    for(let [j, i] of names.entries()) {
      const col = row[i] + '';
      if((sizes[i] ?? 0) < col.length) sizes[i] = col.length;
    }
  }

  let width = names.reduce((acc, name) => (acc ? acc + 3 + sizes[name] : sizes[name]), 0);
  if(width > repl.termWidth) sizes['Params'] -= width - repl.termWidth;

  function padTrunc(len, pad = ' ') {
    let m = 'pad' + (len >= 0 ? 'End' : 'Start');
    len = Math.abs(len);

    return s => {
      s = s + '';
      return s.length > len ? s.slice(0, len) : s[m](len, pad);
    };
  }

  const trunc = names.map((name, i) => padTrunc((i == 0 ? -1 : 1) * sizes[name]));
  const pad = (cols, pad, sep) => {
    if(!Array.isArray(cols)) cols = names.map((key, i) => cols[key]);
    return cols
      .map((s, col) => trunc[col](s, pad))
      .join(sep ?? ' │ ')
      .trimEnd();
  };

  return (
    pad(names) +
    '\n' +
    pad(
      names.reduce((acc, n) => ({ ...acc, [n]: '' }), {}),
      '─',
      '─┼─'
    ) +
    '\n' +
    rows.reduce((acc, row) => {
      return acc + pad(row).slice(0, repl.columns) + '\n';
    }, '')
  );
}

function PrintRange(range, file) {
  if('range' in range) range = range.range;

  const { begin, end } = range;

  file ??= begin.file ?? $.source;

  let data = ReadFile(file, 'utf-8');
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

function ParentNode(node, ast = $.data) {
  let p = PathOf(node, ast);

  return p.up(2).deref(ast);
}

function NextSibling(node, ast = $.data) {
  let p = PathOf(node, ast);
  p[p.length - 1] += 1;

  return p.deref(ast);
}

function PreviousSibling(node, ast = $.data) {
  let p = PathOf(node, ast);
  p[p.length - 1] += 1;
  return p.deref(ast);
}

function FirstChild(node, ast = $.data) {
  return PathOf(node, ast).down('inner', 0).deref(ast);
}

function LastChild(node, ast = $.data) {
  let a = PathOf(node, ast).down('inner').deref(ast);
  return a[a.length - 1];
}

function Terminate(exitCode) {
  console.log('Terminate', exitCode);

  exit(exitCode);
}

function ParseStructs(text) {
  const re = /([^\n]*)\r?\n/gmy;
  let line,
    fields,
    structs = [];

  while((line = re.exec(text))) {
    if(/^\s*$/g.test(line[1])) continue;
    let columns = line[1].split(/\s+/g);

    if(isNaN(+columns[1])) continue;

    let [name] = columns;

    if(fields && name[0] == '.') {
      let [, offset, size] = columns;
      fields.push([name, +offset, +size]);
    } else {
      structs.push([name, { size: +columns[1], fields: (fields = []) }]);
    }
  }

  return new Map(structs);
}

const GenerateInspectStruct = StringGenerator(function* (decl, includes) {
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
});

function InspectStruct(decl, includes, compiler = 'clang') {
  if(typeof decl == 'string') {
    let name = (decl.indexOf(' ') != -1 ? '' : 'struct ') + decl;
    decl = Type.get(name, $.data);
    decl ??= $.getType(name);
    decl.name = name;
  }
  console.log('InspectStruct', decl);

  const code = [...GenerateInspectStruct(decl, includes)].join('\n');
  const program = `/tmp/inspect-${decl.name.replace(/ /g, '_')}`;
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
      .map(([field, offset, size]) => [field.replace(/:.*/, '').replace(/^\./, name + '.'), offset, size]);

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

    define(result, {
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

  let { size, members = [] } = decl;
  name ??= decl.name;

  let className = name.replace(/struct\s*/, '');

  yield `class ${className} extends ArrayBuffer {`;
  yield `  constructor(obj = {}) {\n    super(${size});\n    Object.assign(this, obj);\n  }`;
  yield `  get [Symbol.toStringTag]() { return \`[${name} @ \${this} ]\`; }`;

  let fields = [],
    offset = 0;

  console.log('GenerateStructClass', { decl, members });

  for(let [name, type] of members) {
    if(/reserved/.test(name)) continue;

    if(type.size == 8) offset = RoundTo(offset, 8);
    let desugared = type.desugared && type.desugared != type ? ` (${type.desugared})` : '';
    let pointer = type.pointer;

    yield '';

    let subscript = type.subscript ?? '';

    yield `  /* ${offset}: ${type}${desugared} ${name}${subscript} */`;

    try {
      yield* GenerateGetSet(name, offset, type, ffiPrefix).map(line => `  ${line}`);
    } catch(e) {}

    fields.push(name);

    offset += RoundTo(type.size, 4);
  }

  yield '';
  yield `  static from(address) {\n    let ret = ${ffiPrefix}toArrayBuffer(address, ${offset});\n    return Object.setPrototypeOf(ret, ${className}.prototype);\n  }`;
  yield '';

  yield `  toString() {\n    const { ${fields.join(', ')} } = this;\n    return \`${name} {${[...members]
    .map(([field, member]) => '\\n\\t.' + field + ' = ' + (member.isPointer() ? '0x' : '') + '${' + field + (member.isPointer() ? '.toString(16)' : '') + '}')
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

  const a = [];

  if(pointer) {
    const { name, size, signed, desugared } = pointer;
    a.unshift(`/* ${name}${desugared ? ` (${desugared})` : ''} ${size} ${signed} */`);
    console.log('GenerateStructClass', { pointer });
  }

  return [
    ...a,
    `set ${name}(value) { if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = ${ffiPrefix}toPointer(value); new ${ctor}(this, ${offset})[0] = ${ByteLength2Value(
      size,
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
      default:
        throw new Error(`Floating point, but ${byteLength} size`);
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
      return floating ? 'Float64Array' : signed ? 'BigInt64Array' : 'BigUint64Array';
    default:
      return signed ? 'Int8Array' : 'Uint8Array';
  }
}

function ByteLength2Value(byteLength, signed, floating) {
  if(byteLength == 8 && !floating) return 'BigInt(value)';
  return 'value';
}

export class FFI_Function {
  constructor(node, prefix = '') {
    const { name, returnType = 'void', parameters = [] } = node;

    //console.log('FFI_Function.constructor', node, { name, parameters });

    this.name = name;
    this.prefix = prefix;
    this.returnType = returnType.ffi;
    this.parameters = [...(parameters || [])].map(([name, type], idx) => [name ?? `arg${idx + 1}`, type.ffi]);
  }

  generateDefine(fp, lib) {
    const { prefix, name, returnType, parameters } = this;
    fp ??= (name, lib) => `${prefix}dlsym(${lib ?? 'RTLD_DEFAULT'}, '${name}')`;
    let code = `'${name}', ${fp(name, lib)}, null, '${returnType}'`;
    const colorText = x => x;

    //console.log('function', colorText(name, 1, 33), 'returnType:', colorText(returnType, 1, 31));

    let paramIndex = 0;

    for(let [paramName, type] of parameters) {
      ++paramIndex;

      //console.log(`param #${paramIndex}`, ...(paramName ? ['name:', paramName] : []), 'type:', type);

      code += ', ';
      code += `'${type}'`;
    }
    return `${prefix}define(${code});`;
  }

  generateDoc() {
    const { prefix, name, returnType, parameters } = this;
    const lines = [];
    const columns = [10, 16];

    const push = (...args) =>
      lines.push(
        [...args]
          .map(field => (typeof field != 'string' ? '' : field))
          .map((field, col) => field.padEnd(columns[col] ?? 0))
          .join('')
      );

    push('/**');
    push(`@function`, `${name}`);

    if(parameters.length) push();

    let js = type => {
      let typeObj = Type.declarations.get(type);
      return typeObj ? typeObj.toJS() : type;
    };

    for(let [paramName, type] of parameters) push(`@param`, `{${js(type)}}`, paramName);

    if(returnType != 'void') {
      push();
      push(`@return`, `{${js(returnType)}}`);
    }

    return lines.join('\n * ') + '\n */\n';
  }

  generateCall() {
    const { prefix, name, returnType, parameters } = this;
    const paramNames = parameters.map(([name, type]) => name);
    let code = `function ${name}(${paramNames.join(', ')}) {\n`;

    code += `  ${returnType != 'void' ? 'return ' : ''}${prefix}call('${name}', ${paramNames.join(', ')});\n`;
    code += `}`;

    return code;
  }

  generate(fp, lib, exp) {
    return [this.generateDefine(fp, lib), '\n', exp ? 'export ' : '', this.generateCall(), '\n'].join('');
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
  let child = Spawn(args, {
    block: false,
    stdio: ['inherit', 'pipe', 'inherit']
  });

  let output = '',
    done = false,
    buf = new ArrayBuffer(1024);

  if(platform == 'quickjs') {
    let { fd } = child.stdout;

    for(;;) {
      1;
      let r;
      await fs.waitRead(fd);
      r = ReadOutput(fd);
      if(r > 0 && r < buf.byteLength) break;
    }

    let result = await child.wait();

    return output.trimEnd();
  } else {
    AcquireReader(child.stdout, async reader => {
      let r;

      while((r = await reader.read())) if(!r.done) errors += r.value.toString();
    });
  }

  function ReadOutput(fd) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);

    output += fs.bufferToString(buf.slice(0, r));

    return r;
  }
}

export async function LibraryExports(file) {
  console.log(`LibraryExports:`, file);

  let output = await CommandRead(['/opt/diet/bin/objdump', '-T', file]);
  output = output.replace(/.*DYNAMIC SYMBOL TABLE:\s/m, '');

  let lines = output.split(/\n/g).filter(line => /\sBase\s/.test(line));
  let columns = colIndexes(lines[0]);
  let entries = lines.map(line => colSplit(line, columns).map(column => column.trimEnd()));

  entries.sort((a, b) => a[0].localeCompare(b[0]));

  return entries.map(entry => entry[entry.length - 1].trimStart());
}

function SaveLibraries() {
  const layers = Object.values([...project.schematic.layers, ...project.board.layers].reduce((acc, [n, e]) => ({ ...acc, [n]: e.raw }), {}));
}

function ProcessFile(file, debug = true) {
  const ext = path.extname(file);
  let ret = null;

  switch (ext) {
    case '.js':
      ret = ParseECMAScript(file, debug);
      break;
    case '.cpp':
    case '.cxx':
    case '.cc':
    case '.c':
    case '.h':
      ret = Compile(file /*, debug*/);
      break;
  }

  return ret;
}

function ParseECMAScript(file, params = {}) {
  let data, b, ret;
  const { debug } = params;

  if(file == '-') file = '/dev/stdin';

  if(file && fs.existsSync(file)) {
    data = ReadFile(file, 'utf8');
    console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }

  console.log('OK, data: ', abbreviate(escape(data)));

  if(debug) ECMAScriptParser.instrumentate();

  console.log('ECMAScriptParser:', ECMAScriptParser);

  let parser, ast, error;

  globalThis.parser = parser = null;
  globalThis.parser = parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  try {
    ast = parser.parseProgram();
  } catch(err) {
    const tokens = [...parser.processed, ...parser.tokens];
    const token = tokens[tokens.length - 1];

    console.log('parseProgram token', token);

    if(err !== null) {
      console.log('parseProgram ERROR message:', err?.message);
      console.log('parseProgram ERROR stack:\n  ' + new Stack(err?.stack, (fr, i) => fr.functionName != 'esfactory' && i < 5).toString().replace(/\n/g, '\n  '));

      throw err;
    } else {
      console.log('parseProgram ERROR:', err);
      throw new Error('parseProgram');
    }
  }

  parser.addCommentsToNodes(ast);
  return ast;
}

/*function ParseECMAScript(file, debug = false) {
  console.log(`Parsing '${file}'...`);
  let data = ReadFile(file, 'utf-8');
  let ast, error;
  let parser;
  console.log('data', data);
  globalThis.parser = parser = new ECMAScriptParser(data?.toString ? data.toString() : data, file, debug);

  globalThis.ast = ast = parser.parseProgram();
  parser.addCommentsToNodes(ast);

  globalThis.files[file] = ast;

  return {
    ast,
    get data() {
      return this.ast;
    }
  };
}*/

function PrintECMAScript(ast, comments, printer = new ECMAScript.Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

function PrintCArray(strings) {
  let o = '';

  for(let str of strings) {
    if(o != '') o += ', ';

    if(typeof str == 'string') o += '"' + str + '"';
    else o += '0';
  }

  /* strings.map(str => (typeof str == 'string' && str ? '"' + str + '"' : '""')).map(str => str != '' ? str : '""').join(',\n') +*/

  o += ` });`;
  return `((const char*const []){ ` + o;
}

function Namespaces(nodePath, ast = $.data) {
  let ptr = new Pointer(nodePath);
  let ptrs = ptr.chain(2);
  let get = deep.get(ast);

  let ns = ptrs.map(p => get(p)).filter(n => n.kind == 'NamespaceDecl');
  return ns;
}

function* Constants(node, t = (name, value) => [name, !isNaN(+value) ? +value : value]) {
  for(let inner of node.inner) {
    yield t(...PrintAst(inner).split(/ = /g));
  }
}

MemberNames.UPPER = 1;
MemberNames.METHODS = 2;
MemberNames.PROPERTIES = 4;

function GetImports(ast = $.data) {
  const r = [];

  for(let [n, p] of deep.select($.data, n => (n.type ?? n.kind).startsWith('Import'))) r.push(n);

  return r;
}

function GetIdentifiers(nodes, key = null) {
  const r = [];

  for(let node of nodes) for (let n of deep.select(node, (n, k) => (n.type ?? n.kind) == 'Identifier' && (key === null || k == key), deep.RETURN_VALUE)) r.push(n.name);

  return r;
}

function MemberNames(members, flags = 0) {
  let ret = [];

  if(members.members) members = members.members;

  if(!Array.isArray(members)) {
    for(let ptr of deep.select(members, n => n.kind.endsWith('Decl') && n.name, deep.RETURN_PATH).map(path => new Pointer(path))) {
      let ptrs = ptr.chain(2);

      console.log('ptrs:', ptrs);

      let names = ptrs.map(p => deep.get(members, [...p, 'name'], deep.NO_THROW));
      let kinds = ptrs.map(p => deep.get(members, [...p, 'kind'], deep.NO_THROW));

      console.log('kinds:', kinds);
      console.log('names:', names);

      ret.push(names.filter(name => name).join('.'));
    }
  } else {
    const memberNamePointers = deep.select(members, n => Array.isArray(n) && n.length == 2 && typeof n[0] == 'string' && n[1] !== null, deep.RETURN_VALUE_PATH).map(([node, ptr]) => ptr);
    //console.log('memberNamePointers', memberNamePointers);

    for(let ptr of memberNamePointers.map(path => new Pointer(path))) {
      let ptrs = ptr.chain(3),
        names = ptrs.map(p => deep.get(members, [...p, 0]));

      ret.push(names.filter(name => name).join('.'));
    }
  }

  if(flags & MemberNames.UPPER) ret = ret.map(name => decamelize(name, '_').toUpperCase());

  return ret;
}

function UnsetLoc(node, pred = (v, p) => true) {
  for(let [v, p] of deep.select(node, (v, k) => k == 'loc' || k == 'range', deep.RETURN_VALUE_PATH)) if(pred(deep.get(node, [...p].slice(0, -1)), [...p].last)) deep.unset(node, p);

  return node;
}

function MakeFFI(node, lib, exp, fp) {
  if(Array.isArray(node))
    return (function* () {
      let i = 0;

      if(!fp) yield `import { dlopen, define, dlerror, dlclose, dlsym, call, errno, RTLD_NOW } from 'ffi';\n`;

      if(lib) {
        let libvar = lib.replace(/\.so($|\..*)/g, '').replace(/[^A-Za-z0-9_]/g, '_');

        yield `const ${libvar} = dlopen('${lib}', RTLD_NOW);\n`;

        lib = libvar;
      }

      for(let item of node) {
        let out = '';
        //console.log(`MakeFFI item #${i + 1}/${node.length}`);

        try {
          let ret = MakeFFI(item, lib, exp, fp);

          if(typeof ret == 'string' && ret.length > 0) {
            if(out) out += '\n';
            out += ret;
          }
        } catch(error) {
          console.log(`ERROR item [${i}]:`, error.message + '\n' + error.stack);
        }

        i++;

        yield out;
      }
    })();

  try {
    if(!(node instanceof Node)) node = TypeFactory(node, $.data);
  } catch(e) {
    console.error('TypeFactory', e?.message);
  }

  if(typeof node == 'object' && node && node.kind == 'FunctionDecl') node = new FunctionDecl(node);

  if(node instanceof FunctionDecl) {
    console.log('node', (globalThis.node = node));

    let ffi = new FFI_Function(node);

    let protoStr = PrintAst(node.ast, $.data)
      .replace(/^([^\n\(]*)\n/, '$1 ')
      .split(/\n/g)[0]
      .replace(/\ {$/, ';');

    protoStr = protoStr.replace(/^\s*extern\s+/, '');

    return (ffi.generateDoc(fp, lib, exp) ?? `/* ${protoStr} */\n`) + ffi.generate(fp, lib, exp);
  } else if(node instanceof RecordDecl || node instanceof TypedefDecl) {
    return GenerateStructClass(node);
    //return [...GenerateStructClass(node)].join('\n');
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

  globalThis.console = new Console({ stdout: process.stdout, inspectOptions });

  globalThis.files = files = {};

  base = path.basename(args[0], '.js').replace(/\.[a-z]*$/, '');
  cmdhist = `.${base}-cmdhistory`;
  config = `.${base}-config`;

  params = globalThis.params = getOpt(
    {
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      libs: [true, (a, p) => (p || []).concat([a]), 'l'],
      debug: [false, null, 'x'],
      force: [false, null, 'f'],
      target: [true, null, 't'],
      exec: [true, null, 'e'],
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

  define(globalThis, {
    defs,
    includes,
    libs,
    /* prettier-ignore */ get flags() {
      return [ ...(params.target? [`--target=${params.target}`] : []),  ...includes.filter(v => typeof v == 'string').map(v => `-I${v}`), ...defs.map(d => `-D${d}`), ...libs.map(l => `-l${l}`)];
    }
  });

  async function Compile(file, ...args) {
    console.log('Compiling', { file, args });
    let r;

    /* if(params.target)
      args.unshift(`--target=${params.target}`);*/

    try {
      r = await AstDump(params.compiler, file, [...globalThis.flags, ...args], params.force);
    } catch(e) {
      console.log('Compile ERROR:', e.message + '\n' + e.stack);
      return e;
    }
    r.source = file;

    globalThis.files[file] = r;

    function nameOrIdPred(name_or_id, pred = n => true) {
      if(/^(struct|union)\s/.test(name_or_id)) {
        let idx = name_or_id.indexOf(' ');
        let tag = name_or_id.substring(0, idx);
        let name = name_or_id.substring(idx + 1);
        return node => node.name == name && node.tagUsed == tag;
      }

      if(typeof name_or_id == 'number') name_or_id = '0x' + name_or_id.toString(16);

      return name_or_id instanceof RegExp
        ? node => name_or_id.test(node.name) && pred(node)
        : name_or_id.startsWith('0x')
        ? node => node.id == name_or_id && pred(node)
        : node => node.name == name_or_id && pred(node);
    }

    r,
      {
        select(name_or_id, pred = n => true) {
          return this.data.inner.filter(nameOrIdPred(name_or_id, pred));
        },
        getByIdOrName(name_or_id, pred = n => true) {
          let node = this.data.inner.findLast(nameOrIdPred(name_or_id, pred));

          node ??= this.classes.findLast(nameOrIdPred(name_or_id, pred));
          node ??= deep.find(this.data, nameOrIdPred(name_or_id, pred), deep.RETURN_VALUE);
          return node;
        },
        getType(name_or_id) {
          let result = this.getByIdOrName(name_or_id, n => !/(FunctionDecl)/.test(n.kind) && /Decl/.test(n.kind)) ?? GetType(name_or_id, this.data);

          if(result) {
            let type = TypeFactory(result, this.data);
            if(type) result = type;
          }

          return result;
        },

        getFunction(name_or_id) {
          let result = isNode(name_or_id) ? name_or_id : this.getByIdOrName(name_or_id, n => /(FunctionDecl)/.test(n.kind));

          if(result) return new FunctionDecl(result, this.data);
        },
        getVariable(name_or_id) {
          let result = isNode(name_or_id) ? name_or_id : this.getByIdOrName(name_or_id, n => /(VarDecl)/.test(n.kind));

          if(result) return new VarDecl(result, this.data);
        },
        getLoc(node) {
          return CompleteLocation(node);
        }
      };
    defineGetter(
      r,
      'tree',
      memoize(() => new Tree(r.data))
    );
    return define(r, {
      pathOf(needle, maxDepth = 10) {
        if('ast' in needle) needle = needle.ast;

        for(let [node, path] of deep.iterate(r.data, n => typeof n == 'object' && n != null, deep.RETURN_VALUE_PATH, maxDepth)) if(node === needle) return new Pointer(path);
      }
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
    RawLocation,
    CompleteLocation,
    RawRange,
    CompleteRange,
    GetTypeStr,
    PrintRange,
    OverlapRange,
    ParentNode,
    NextSibling,
    PreviousSibling,
    FirstChild,
    LastChild,
    GenerateInspectStruct,
    GenerateStructClass,
    InspectStruct,
    MakeStructClass,
    DirIterator,
    RecursiveDirIterator,
    IncludeAll,
    Terminal,
    PrintAst,
    ConcatIterator,
    StringGenerator,
    ReadFile,
    WriteFile,
    MakeFFI,
    ParseECMAScript,
    PrintECMAScript,
    ProcessFile,
    toArrayBuffer,
    toString,
    Constants,
    PrintCArray,
    GetParams,
    List,
    Shell,
    ParseStructs
  });

  Pointer.prototype.chain = function(step, limit = Infinity) {
    let ptr = this,
      ret = [],
      len = ptr.length;

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
    Tree,
    deep,
    path,
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
    VarDecl,
    FindType,
    Hier,
    PathOf,
    FunctionDecl,
    Location,
    Range,
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
    GetImports,
    GetIdentifiers,
    Namespaces,
    UnsetLoc
  });
  //globalThis.Util = Util;
  globalThis.F = arg => $.getFunction(arg);
  globalThis.T = arg => $.getType(arg);

  lazyProperty(globalThis, 'P', () => {
    let printer = NodePrinter($.data);

    return node => {
      if('ast' in node) node = node.ast;
      printer.clear();
      return printer.print(node);
    };
  });

  console.log('Loading history');

  const unithist = `.${base}-unithistory`;
  let items = [],
    hist = ReadJSON(unithist) || [];

  console.log('Loading sources:' + sources.map(s => ' ' + s).join(','));

  globalThis['_'] = items;

  for(let source of sources) {
    let item = await ProcessFile(source);

    globalThis['$'] = item;

    /*if(/\.js$/.test(source)) item = ParseECMAScript(source);
    else item = await Compile(source);*/

    if(item) {
      pushUnique(hist, [...flags, source]);
      items.push(item);
    }
  }

  WriteFile(unithist, JSON.stringify(hist, null, 2));

  // globalThis.$ = items.length == 1 ? items[0] : items;
  await CommandLine();
}

let error;

try {
  const argv = [...(process?.argv ?? scriptArgs)].slice(2);
  ASTShell(...argv);
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log('FAIL: ' + error.message, '\n  ' + new Stack(error.stack, fr => fr.functionName != 'esfactory').toString().replace(/\n/g, '\n  '));
    console.log('FAIL');
    std.exit(1);
  }
}
