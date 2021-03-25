import PortableFileSystem from './lib/filesystem.js';
import PortableSpawn from './lib/spawn.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
//import * as deep from 'deep.so';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './repl.js';
//import * as std from 'std';
import { SIZEOF_POINTER, Node, Type, RecordDecl, EnumDecl, TypedefDecl, FunctionDecl, Location, TypeFactory, SpawnCompiler, AstDump, NodeType, NodeName, GetLoc, GetType, GetTypeStr, NodePrinter } from './clang-ast.js';
import Tree from './lib/tree.js';
import * as Terminal from './terminal.js';
import * as ECMAScript from './lib/ecmascript.js';
import { ECMAScriptParser } from './lib/ecmascript.js';

let files;
let filesystem, spawn, base, cmdhist;
let defs, includes, sources;
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
  get head() {
    return this[this.length-1];
  },
  get tail() {
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
        (acc, item) => ({ ...acc, [item]: { value: module[item], enumerable: true } }),
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
  let outputLog = filesystem.fopen('output.log', 'w+');
  /*console.reallog = function(...args) {
    log.call(this, ...args);
    outputLog.puts(args.join(' ')+'\n');
    outputLog.flush();
  };*/

  let repl = (globalThis.repl = new REPL('AST'));
  repl.exit = Util.exit;
  repl.importModule = ImportModule;
  repl.history_set(LoadJSON(cmdhist));
  repl.directives = {
    c(...args) {
      //      console.log('c', { args });
      Compile(...args);
      return false;
    },
    l(...args) {
      ProcessFile(...args);
      return false;
    }
  };
  repl.show = value => {
    if(Util.isArray(value) && value[0]?.kind) console.log(Table(value));
    else if(typeof value == 'string') console.log(value);
    /*if(typeof value == 'object' && value != null)*/ else
      console.log(inspect(value, { ...console.options, hideKeys: ['loc', 'range'] }));
    // else console.log(value);
  };
  let debugLog = filesystem.fopen('debug.log', 'a');
  repl.debugLog = debugLog;
  repl.debug = (...args) => {
    let s = '';
    for(let arg of args) {
      if(s) s += ' ';
      if(typeof arg != 'strping' || arg.indexOf('\x1b') == -1)
        s += inspect(arg, { depth: Infinity, depth: 6, compact: 4 });
      else s += arg;
    }
    debugLog.puts(s + '\n');
    debugLog.flush();
  };

  Util.atexit(() => {
    debugLog.close();
    Terminal.mousetrackingDisable();
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);
    filesystem.writeFile(cmdhist, JSON.stringify(hist, null, 2));
    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
  //Terminal.mousetrackingEnable();

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

function* RecursiveDirIterator(dir, maxDepth = Infinity) {
  for(let file of filesystem.readdir(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;

    let entry = `${dir}/${file}`;
    let isDir = false;
    let st = filesystem.stat(entry);

    isDir = st && st.isDirectory();

    yield isDir ? entry + '/' : entry;

    if(maxDepth > 0 && isDir) yield* RecursiveDirIterator(entry, maxDepth - 1);
  }
}

function SelectLocations(node) {
  let result = deep.select(node, n =>
    ['offset', 'line', 'file'].some(prop => n[prop] !== undefined)
  );
  console.log('result:', console.config({ depth: 1 }), result);
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
  let entries = [...list].map((n, i) => [i, LocationString(GetLoc(n)), n]);
  const colSizes = [5, 10, 12, 30, 12, 15, 25];
  const colKeys = ['id', 'kind', 'name', 'tagUsed', 'previousDecl', 'completeDefinition'];
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

function LoadJSON(filename) {
  let data = filesystem.readFile(filename, 'utf-8');
  return data ? JSON.parse(data) : null;
}

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function* GenerateInspectStruct(decl, includes) {
  let { name, members } = decl;

  includes ??= [decl.loc.file.replace(/^\/usr\/include\//, '')];
  yield '#include <stdio.h>';
  yield '#include <stddef.h>';

  console.log('GenerateInspectStruct', { name, members, includes });
  for(let include of includes) yield `#include "${include}"`;
  yield `${name} svar;`;
  yield `int main() {`;
  yield `  printf("${name} %zu\\n", sizeof(svar));`;
  for(let [member, type] of members)
    if((type == null || typeof type.size == 'number') && member != undefined) {
      member = member.replace(/:.*/, '');
      yield `  printf(".${member} %zu %zu\\n", offsetof(${name}, ${member}), sizeof(svar.${member}));`;
    }

  yield `  return 0;`;
  yield `}`;
}

function InspectStruct(decl, compiler = 'tcc') {
  if(typeof decl == 'string') decl = $.getType(decl);

  const code = [...GenerateInspectStruct(decl)].join('\n');
  //console.log('InspectStruct', { code });
  const program = `inspect-${decl.name.replace(/\ /g, '_')}`;
  WriteFile(program + '.c', code);

  let command = [compiler, '-Os', '-w', '-o', program, program + '.c', ...flags];
  console.log('InspectStruct', { command: command.join(' ') });

  let result = os.exec(command);

  if(result == 0) {
    let [fd, stdout] = os.pipe();

    os.exec([`./${program}`], { stdout });
    let output = filesystem.readAll(fd);

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

    result.unshift([name, '-', '-', +size]);

    result.toString = function(sep = ' ') {
      return this.map(line => line.join(sep).replace('.', ' ')).join('\n');
    };
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
    this.parameters = [...parameters].map(([name, type]) => [name, type.ffi]);
  }

  generateDefine(fp, lib) {
    const { prefix, name, returnType, parameters } = this;
    fp ??= (name, lib) => `${prefix}dlsym(${lib ?? 'RTLD_DEFAULT'}, '${name}')`;
    let code = `'${name}', ${fp(name, lib)}, null, '${returnType}'`;
    for(let [name, type] of parameters) {
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
  let buf = filesystem.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await filesystem.waitRead(fd);
      ret = filesystem.read(fd, buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(filesystem.bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    filesystem.close(fd);
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
      await filesystem.waitRead(fd);
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
    output += filesystem.bufferToString(buf.slice(0, r));
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
  let data = filesystem.readFile(file, 'utf-8');
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
function PrintAst(node, ast) {
  ast ??= $.data;

  let printer = NodePrinter(ast);
  globalThis.printer = printer;

  Object.defineProperties(printer, {
    path: {
      get() {
        return deep.pathOf(ast, this.node);
      }
    }
  });

  if(Array.isArray(node)) {
    for(let elem of node) {
      if(printer.output) printer.put('\n');

      printer(elem, ast);
    }
  } else {
    printer(node, ast);
  }
  return printer.output;
}

function MakeFFI(node) {
  if(Array.isArray(node)) {
    let out = '';
    for(let item of node) {
      try {
        let ret = MakeFFI(item);
        if(typeof ret == 'string' && ret.length > 0) {
          if(out) out += '\n';
          out += ret;
        }
      } catch(error) {
        out += `/* ERROR: ${error.message} */`;
      }
    }
    return out;
  }
  if(!(node instanceof Node)) node = TypeFactory(node, $.data);

  if(typeof node == 'object' && node && node.kind == 'FunctionDecl') node = new FunctionDecl(node);

  if(node instanceof FunctionDecl) {
    let ffi = new FFI_Function(node);

    let protoStr = PrintAst(node.ast, $.data).split(/\n/g)[0].replace(/\ {$/, ';');

    return `/* ${protoStr} */\n` + ffi.generate();
  } else if(node instanceof RecordDecl || node instanceof TypedefDecl) {
    let code = [...GenerateStructClass(node)].join('\n');
    return code;
  }
}

async function ASTShell(...args) {
  let consoleOptions = {
    /*breakLength: 240, */ customInspect: true,
    compact: 1,
    depth: 1,
    maxArrayLength: Infinity
  };
  console.options = consoleOptions;
  console.options.compact = 1;
  console.options.hideKeys = ['loc', 'range'];
  await ConsoleSetup(consoleOptions);
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

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

  let params = Util.getOpt({
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
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
  sources = params['@'] || [];

  Util.define(globalThis, {
    defs,
    includes,
    get flags() {
      return [...includes.map(v => `-I${v}`), ...defs.map(d => `-D${d}`)];
    }
  });

  async function Compile(file, ...args) {
    let r = await AstDump(params.compiler, file, [...globalThis.flags, ...args], params.force);
    r.source = file;

    globalThis.files[file] = r;

    Object.assign(r, {
      getByIdOrName(name_or_id, pred = n => true) {
        if(typeof name_or_id == 'number') name_or_id = '0x' + name_or_id.toString(16);

        return this.data.inner.find(name_or_id.startsWith('0x')
            ? node => node.id == name_or_id && pred(node)
            : node => node.name == name_or_id && pred(node)
        );
      },
      getType(name_or_id) {
        const types = this.data.inner.filter(n => /(RecordDecl|TypedefDecl|EnumDecl)/.test(n.kind));
        let result, idx;

        if(typeof name_or_id == 'object' && name_or_id) {
          result = name_or_id;
        } else if(typeof name_or_id == 'string') {
          let results = types.filter(name_or_id.startsWith('0x')
              ? node => node.id == name_or_id
              : node => node.name == name_or_id
          );
          if(results.length <= 1 || (idx = results.findIndex(r => r.completeDefinition)) == -1)
            idx = 0;
          result = results[idx];

          if(!result && Type.declarations.has(name_or_id))
            result = Type.declarations.get(name_or_id);
        } else {
          result = types[name_or_id];
        }

        if(result) return TypeFactory(result, this.data);
      },
      getFunction(name_or_id) {
        let result = this.getByIdOrName(name_or_id, n => /(FunctionDecl)/.test(n.kind));

        if(result) return new FunctionDecl(result, this.data);
      }
    });
    return Util.lazyProperties(r, {
      tree() {
        return new Tree(this.data);
      }
    });
  }

  Object.assign(globalThis, {
    SIZEOF_POINTER,
    Type,
    AstDump,
    NodePrinter,
    NodeType,
    NodeName,
    GetLoc,
    GetTypeStr,
    WriteFile,
    LoadJSON,
    GenerateInspectStruct,
    GenerateStructClass,
    InspectStruct,
    MakeStructClass,
    DirIterator,
    RecursiveDirIterator,
    Terminal,
    PrintAst,
    MakeFFI,
    ParseECMAScript,
    PrintECMAScript,
    ProcessFile
  });

  Object.assign(globalThis, {
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
    FFI_Function,
    libdirs,
    libdirs32,
    libdirs64,
    LibraryExports
  });
  globalThis.util = Util;

  const unithist = `.${base}-unithistory`;
  let items = [];
  let hist = LoadJSON(unithist) || [];

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

  await CommandLine();
}

Util.callMain(ASTShell, true);
