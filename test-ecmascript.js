import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, TemplateLiteral, CallExpression, ImportStatement, Identifier, ObjectBindingPattern, ArrowFunction } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';
import { ImmutablePath } from './lib/json.js';
import { ConsoleSetup } from './consoleSetup.js';

let filesystem;
let globalThis;

const testfn = () => true;
const testtmpl = `this is\na test`;

const code = `

for(let [value, path] of deep.iterate(x, (v, k) => /data-/.test(k[k.length - 1]))) deep.unset(x, path);
`;

Util.callMain(main, Util.putError);

function dumpFile(name, data) {
  console.log('dumpFile', { name, data: Util.abbreviate(Util.unescape(data)) });
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  data = data.trim();

  if(data != '') {
    filesystem.writeFile(name, data + '\n');
    console.log(`Wrote ${name}: ${data.length} bytes`);
  }
}

function printAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);
  console.log('printAst:', Util.abbreviate(output), Util.decodeAnsi(output));
  return output;
}

let files = {};

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));

  await ConsoleSetup({ depth: 3 });
  console.log('main args =', args);
  console.log('console.depth:', console.depth);
  console.log('console.colors:', console.colors);

  globalThis = Util.getGlobalObject();
  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: console.colors, indent: 4 }))
  });

  // console.log('globalThis', globalThis);
  console.log('globalThis.printer', globalThis.printer);

  //await import('tty');

  if(args.length == 0) args.push(null); //'./lib/ecmascript/parser.js');
  for(let file of args) {
    let error;

    await Util.safeCall(processFile, file);

    files[file] = finish(error);

    if(error) {
      Util.putError(error);
      process.exit(1);
    }

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}

async function processFile(file) {
  let data, b, ret;
  if(file == '-') file = '/dev/stdin';
  if(file && filesystem.exists(file)) data = filesystem.readFile(file);
  else {
    file = 'stdin';
    data = code;
  }
  console.log('opened:', file);
  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, false);

  console.log('prototypeChain:', Util.getPrototypeChain(parser));
  console.log('OK, data: ', Util.abbreviate(Util.unescape(data)));
  ast = parser.parseProgram();
  parser.addCommentsToNodes(ast);

  let flat = deep.flatten(ast,
    new Map(),
    node => node instanceof ESNode || Util.isArray(node),
    (path, value) => {
      path = /*path.join("."); //*/ new Path(path);
      return [path, value];
    }
  );

  let [path, fn] = Util.find(flat, (value, key) => value instanceof ArrowFunction);
  path = path.slice(0, path.lastIndexOf('arguments') + 1);
  console.log('path:', path);

  let node = Util.find(flat, (value, key) => path.equals(key));
  /*console.log('keys:', flat.keys());
  console.log('node:', node);*/

  let node2path = new WeakMap();
  let nodeKeys = [];

  for(let [path, node] of flat) {
    node2path.set(node, path);
    nodeKeys.push(path);
  }

  const isRequire = node => node instanceof CallExpression && node.callee.value == 'require';
  const isImport = node => node instanceof ImportStatement;

  let commentMap = new Map([...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      { comment, pos, len, node }
    ]),
    (a, b) => a - b
  );

  console.log('commentMap:', commentMap);
  let allNodes = nodeKeys.map((path, i) => [i, flat.get(path)]);

  // for(let [i, n] of allNodes) console.log(new ImmutablePath(node2path.get(n)), n);

  const output_file = file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

  console.log('saving to:', output_file);
  const output = printAst(ast, parser.comments, printer);
  console.log('output:', output);

  dumpFile(output_file, output);

  const imports = [...flat].filter(([path, node]) => isRequire(node) || isImport(node));
  const importStatements = imports
    .map(([path, node]) => (isRequire(node) || true ? path.slice(0, 2) : path))
    .map(path => [path, deep.get(ast, path)]);

  console.log('imports:', new Map(imports.map(([path, node]) => [ESNode.assoc(node).position, node])));
  console.log('importStatements:', importStatements);

  const importedFiles = imports.map(([pos, node]) => Identifier.string(node.source || node.arguments[0]));
  console.log('importedFiles:', importedFiles);

  let importIdentifiers = importStatements
    .map(([p, n]) => [p, n.identifiers ? n.identifiers : n])
    .map(([p, n]) => [p, n.declarations ? n.declarations : n]);
  console.log('importIdentifiers:', importIdentifiers);

  /*  importIdentifiers = importIdentifiers.map(([p, n]) =>
    n
      .map(decl => (decl.id instanceof ObjectBindingPattern ? decl.id.properties : [decl.id]))
      .flat()
      .map(n => [n.id ? n.id : n])
      .flat()
      .map(n => Identifier.string(n))
  );*/
  console.log('importIdentifiers:', Util.unique(importIdentifiers.flat()).join(', '));
  await ConsoleSetup({ depth: Infinity });
  const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral);

  console.log('templates:', templates);
}

function finish(err) {
  let fail = !!err;
  if(fail) {
    err.stack = PathReplacer()('' + err.stack)
      .split(/\n/g)
      .filter(s => !/esfactory/.test(s))
      .join('\n');
  }

  if(err) {
    console.log(parser.lexer.currentLine());
    console.log(Util.className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
  }

  let lexer = parser.lexer;
  let t = [];
  console.log(parser.trace());
  dumpFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}
