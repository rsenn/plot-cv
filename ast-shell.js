import PortableFileSystem from './lib/filesystem.js';
import PortableSpawn from './lib/spawn.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
import ConsoleSetup from './lib/consoleSetup.js';
import REPL from './repl.js';
import * as std from 'std';
import { SIZEOF_POINTER, Type, AstDump, NodeType, NodeName, GetLoc, GetType } from './clang-ast.js';
import Tree from './lib/tree.js';

let filesystem, spawn, base, histfile;

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

async function ImportModule(moduleName, ...args) {
  let done = false;
  return await import(moduleName)
    .then(module => {
      done = true;
      Object.assign(globalThis, { [moduleName]: module });
      return module;
    })
    .catch(e => {
      console.error(moduleName + ':', e);
      done = true;
    });
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
      node.kind,
      ((node.tagUsed ? node.tagUsed + ' ' : '') + (node.name ?? '')).trim(),
      new Map(
        node.inner.map((field, i) =>
          /Attr/.test(field.kind)
            ? [Symbol(field.kind), field.id]
            : [field.name || i, (field.type && new Type(field.type)) || field.kind]
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

async function ASTShell(...args) {
  await ConsoleSetup({ /*breakLength: 240, */ depth: Infinity });

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

  let defs = params.define || [];
  let includes = params.include || [];
  let sources = params['@'] || [];

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
    GetType
  });

  Object.assign(globalThis, {
    Tree,
    deep,
    Compile,
    SelectLocations,
    LocationString,
    Table,
    Structs,
    Type,
    TypeMap
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
