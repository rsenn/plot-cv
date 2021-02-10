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

async function importModule(moduleName, ...args) {
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

async function run() {
  let repl = (globalThis.repl = new REPL('AST'));
  repl.exit = Util.exit;
  repl.importModule = importModule;
  repl.history_set(JSON.parse(std.loadFile(histfile) || '[]'));
  Util.atexit(() => {
    let hist = repl.history_get().filter((item, i, a) => a.lastIndexOf(item) == i);
    filesystem.writeFile(histfile, JSON.stringify(hist, null, 2));
    console.log(`EXIT (wrote ${hist.length} history entries)`);
  });
  await repl.run();
  console.log('REPL done');
}

async function main(...args) {
  await ConsoleSetup({ /*breakLength: 240, */ depth: 10 });
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

    return r;
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
    Compile
  });

  for(let source of sources) {
    await Compile(source);
  }

  await run();
}

Util.callMain(main, true);
