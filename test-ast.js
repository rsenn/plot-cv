import REPL from './repl.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import * as Terminal from './terminal.js';
import { SIZEOF_POINTER, Node, Type, RecordDecl, EnumDecl, TypedefDecl, FunctionDecl, Location, TypeFactory, SpawnCompiler, AstDump, NodeType, NodeName, GetLoc, GetType, GetTypeStr, NodePrinter } from './clang-ast.js';

const consoleOpts = { depth: 5, compact: 2, hideKeys: ['range', 'loc'] };
async function main(...args) {
  await ConsoleSetup(consoleOpts);
  await PortableFileSystem();

  let name = args[0] ?? 'main';
  let json = JSON.parse(filesystem.readFile('example_gl3.ast.json', 'utf-8'));
  let fns = json.inner.filter(n => n.kind == 'FunctionDecl' && n.name == name);

  //fns = fns.filter(n => n.inner && n.inner.find(n => n.kind == 'CompoundStmt'));
  console.log('fns:', console.config(consoleOpts), fns);

  let fn;

  fn = fns.pop();

  console.log('fn.inner[0]:', console.config(consoleOpts), fn.inner[0]);
  console.log('fn:', console.config(consoleOpts), fn);
  let printer = NodePrinter();

  printer.print(fn);
  console.log('output:', printer.output);
}

Util.callMain(main, true);
