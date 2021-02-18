import PortableFileSystem from './lib/filesystem.js';
import PortableSpawn from './lib/spawn.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './repl.js';
import * as std from 'std';
import { SIZEOF_POINTER, Node, Type, RecordDecl, EnumDecl, TypedefDecl, FunctionDecl, Location, TypeFactory, SpawnCompiler, AstDump, NodeType, NodeName, GetLoc, GetType, GetTypeStr } from './clang-ast.js';
import Tree from './lib/tree.js';

let filesystem, spawn, base, histfile;
let defs, includes, sources;

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
  let repl = (globalThis.repl = new REPL('AST'));
  repl.exit = Util.exit;
  repl.importModule = ImportModule;
  repl.history_set(JSON.parse(std.loadFile(histfile) || '[]'));
  Util.atexit(() => {
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);
    filesystem.writeFile(histfile, JSON.stringify(hist, null, 2));
    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
  await repl.run();
  console.log('REPL done');
}

function SelectLocations(node) {
  let result = deep.select(node, n =>
    ['offset', 'line', 'file'].some(prop => n[prop] !== undefined)
  );
  console.log('result:', console.config({ depth: 1 }), result);
  return result;
}

function LocationString(loc) {
  let file = loc.includedFrom ? loc.includedFrom.file : loc.file;
  if(typeof loc.line == 'number')
    return `${file ? file + ':' : ''}${loc.line}${typeof loc.col == 'number' ? ':' + loc.col : ''}`;
  return `${file ? file : ''}@${loc.offset}`;
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
  const colKeys = ['id', 'kind', 'name', 'tagUsed', 'previousDecl'];
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

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function* GenerateInspectStruct(type, members, includes) {
  console.log('GenerateInspectStruct', { type, members, includes });
  for(let include of ['stdio.h', ...includes]) yield `#include "${include}"`;
  yield `${type} svar;`;
  yield `int main() {`;
  yield `  printf("${type} - %u\\n", sizeof(svar));`;
  for(let member of members)
    yield `  printf(".${member} %u %u\\n", (char*)&svar.${member} - (char*)&svar, sizeof(svar.${member}));`;
  yield `  return 0;`;
  yield `}`;
}

async function InspectStruct(decl) {
  const include = decl.loc.file.replace(/^\/usr\/include\//, '');
  const code = [...GenerateInspectStruct(decl.name, [...decl.members.keys()], [include])].join('\n'
  );
  const file = `inspect-${decl.name.replace(/\ /g, '_')}`;
  WriteFile(file + '.c', code);

  let result = await SpawnCompiler(file + '.c', ['-o', file, ...flags]);

  console.log('InspectStruct', { file, result });
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

function* GenerateStructClass(decl) {
  let { name, size, members } = decl;

  yield `class ${name.replace(/struct\s*/, '')} extends ArrayBuffer {`;
  yield `  constructor(obj = {}) {\n    super(${size});\n    Object.assign(this, obj);\n  }`;
  yield `  get [Symbol.toStringTag]() { return \`[${name} @ \${this} ]\`; }`;
  let fields = [];
  let offset = 0;
  for(let [name, member] of members) {
    if(/reserved/.test(name)) continue;

    if(member.size == 8) offset = RoundTo(offset, 8);

    yield '';
    yield `  /* ${offset}: ${member} ${name}@${member.size} */`;
    yield* GenerateGetSet(name,
      offset,
      member.size,
      member.signed && !member.isPointer(),
      member.isFloatingPoint()
    ).map(line => `  ${line}`);
    fields.push(name);
    offset += RoundTo(member.size, 4);
  }
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

function GenerateGetSet(name, offset, size, signed, floating) {
  let ctor = ByteLength2TypedArray(size, signed, floating);
  return [
    `set ${name}(value) { new ${ctor}(this, ${offset})[0] = ${ByteLength2Value(size,
      signed,
      floating
    )}; }`,
    `get ${name}() { return new ${ctor}(this, ${offset})[0]; }`
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

async function ASTShell(...args) {
  await ConsoleSetup({ /*breakLength: 240, */ customInspect: true, compact: 1, depth: 1 });

  console.options.compact = 1;
  console.options.hideKeys = ['loc', 'range'];

  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  base = path.basename(Util.getArgv()[1], /\.[^.]*$/);
  histfile = `.${base}-history`;

  let params = Util.getOpt({
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
    let r = await AstDump(file, [...globalThis.flags, ...args]);
    r.source = file;

    Object.assign(r, {
      getByIdOrName(name_or_id, pred = n => true) {
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
    NodeType,
    NodeName,
    GetLoc,
    GetTypeStr,
    WriteFile,
    GenerateInspectStruct,
    GenerateStructClass,
    InspectStruct,
    MakeStructClass
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
    GetTypeStr
  });
  globalThis.util = Util;

  let items = [];
  for(let source of sources) {
    items.push(await Compile(source));
  }
  globalThis.$ = items.length == 1 ? items[0] : items;

  await CommandLine();
}

Util.callMain(ASTShell, true);
