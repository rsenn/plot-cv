import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, Program, ModuleDeclaration, ModuleSpecifier, ImportDeclaration, ImportSpecifier, ImportDefaultSpecifier, ImportNamespaceSpecifier, Super, Expression, FunctionLiteral, Pattern, Identifier, Literal, RegExpLiteral, TemplateLiteral, BigIntLiteral, TaggedTemplateExpression, TemplateElement, ThisExpression, UnaryExpression, UpdateExpression, BinaryExpression, AssignmentExpression, LogicalExpression, MemberExpression, ConditionalExpression, CallExpression, DecoratorExpression, NewExpression, SequenceExpression, Statement, EmptyStatement, DebuggerStatement, LabeledStatement, BlockStatement, FunctionBody, StatementList, ExpressionStatement, Directive, ReturnStatement, ContinueStatement, BreakStatement, IfStatement, SwitchStatement, SwitchCase, WhileStatement, DoWhileStatement, ForStatement, ForInStatement, ForOfStatement, WithStatement, TryStatement, CatchClause, ThrowStatement, Declaration, ClassDeclaration, ClassBody, MethodDefinition, MetaProperty, YieldExpression, FunctionArgument, FunctionDeclaration, ArrowFunctionExpression, VariableDeclaration, VariableDeclarator, ObjectExpression, Property, ArrayExpression, JSXLiteral, AssignmentProperty, ObjectPattern, ArrayPattern, RestElement, AssignmentPattern, AwaitExpression, SpreadElement, ExportNamedDeclaration, ExportSpecifier, AnonymousDefaultExportedFunctionDeclaration, AnonymousDefaultExportedClassDeclaration, ExportDefaultDeclaration, ExportAllDeclaration } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';
import { ImmutablePath } from './lib/json.js';
import Tree from './lib/tree.js';
import { ConsoleSetup } from './lib/consoleSetup.js';

let filesystem;
let globalThis;

const testfn = () => true;
const testtmpl = `this is\na test`;

const code = ` (function() { for(let [value, path] of deep.iterate(x, (v, k) => /data-/.test(k[k.length - 1]))) deep.unset(x, path);
})();

`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

Util.callMain(main, Util.putError);

function WriteFile(name, data) {
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
  await ConsoleSetup({ colors: true, depth: 10, compact: 1 });
  await PortableFileSystem(fs => (filesystem = fs));

  let params = Util.getOpt({
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
      debug: [false, null, 'x'],
      '@': 'input'
    },
    args
  );

  console.log('MethodDefinition.prototype[inspectSymbol]:',
    MethodDefinition.prototype[inspectSymbol] + ''
  );

  globalThis = Util.getGlobalObject();
  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
  });

  // console.log('globalThis', globalThis);
  console.log('params', params);

  //await import('tty');

  if(params['@'].length == 0) params['@'].push(null); //'./lib/ecmascript/parser.js');
  for(let file of params['@']) {
    let error;

    await Util.safeCall(processFile, file, params);

    files[file] = finish(error);

    if(error) {
      Util.putError(error);
      Util.exit(1);
    }

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  Util.exit(Number(files.length == 0));
}

async function processFile(file, params) {
  let data, b, ret;
  const { debug } = params;
  if(file == '-') file = '/dev/stdin';
  if(file && filesystem.exists(file)) data = filesystem.readFile(file);
  else {
    file = 'stdin';
    data = code;
  }
  console.log('opened:', file);
  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  console.log('prototypeChain:', Util.getPrototypeChain(parser));
  console.log('OK, data: ', Util.abbreviate(Util.escape(data)));
  ast = parser.parseProgram();
  parser.addCommentsToNodes(ast);

  if(params['output-ast'])
    WriteFile(params['output-ast'], JSON.stringify(ast /*.toJSON()*/, null, 2));

  let node2path = new WeakMap();

  let flat = deep.flatten(ast,
    new Map(),
    node => node instanceof ESNode || Util.isArray(node),
    (path, value) => {
      //   path = /*path.join("."); //*/ new Path(path);
      node2path.set(value, path);
      return [path, value];
    }
  );
  let nodeKeys = [];

  //  await ConsoleSetup({ depth: 10, colors: true });

  //console.log('ast', ast);
  /*
console.log("find:",[...flat].find(([path,node]) => node instanceof CallExpression));
console.log("find:",[...flat].find(([path,node]) => node instanceof SequenceExpression));*/

  /*  let [path, fn] = Util.find(flat, (value, key) => value instanceof ArrowFunctionExpression);
  path = path.slice(0, path.lastIndexOf('arguments') + 1);
  console.log('path:', path);

  let node = Util.find(flat, (value, key) => path.equals(key));
 
 
  for(let [path, node] of flat) {
    node2path.set(node, path);
    nodeKeys.push(path);
  }
*/
  const isRequire = node => node instanceof CallExpression && node.callee.value == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map([...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      { comment, pos, len, node }
    ]),
    (a, b) => a - b
  );

  console.log('commentMap:', commentMap);
  // let allNodes = nodeKeys.map((path, i) => [i, flat.get(path)]);

  // for(let [i, n] of allNodes) console.log(new ImmutablePath(node2path.get(n)), n);

  const output_file =
    params['output-js'] ?? file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

  let tree = new Tree(ast);
  console.log('tree:',
    tree.flat(null, ([path, node]) => {
      return !Util.isPrimitive(node);
    })
  );

  /*let st =
    deep.get(ast, ['body', 0, 'callee', 'expressions', 0].join('.')) || ast;*/
  // console.log('flat', flat);

  /*  let [p, n] = [...flat].find(([path, node]) => node instanceof BlockStatement);
  console.log('find:', [...p]);
  //console.log("find:",n);

  let body = deep.get(ast, p.concat(['body'])) || ast.body;

  console.log('find:', body);
*/
  const code = printAst(ast, parser.comments, printer);
  console.log('code:', Util.abbreviate(Util.escape(code)));

  WriteFile(output_file, code);

  function getImports() {
    const imports = [...flat].filter(([path, node]) => isRequire(node) || isImport(node));
    const importStatements = imports
      .map(([path, node]) => (isRequire(node) || true ? path.slice(0, 2) : path))
      .map(path => [path, deep.get(ast, path)]);

    console.log('imports:',
      new Map(imports.map(([path, node]) => [ESNode.assoc(node).position, node]))
    );
    console.log('importStatements:', importStatements);

    const importedFiles = imports.map(([pos, node]) =>
      Identifier.string(node.source || node.arguments[0])
    );
    console.log('importedFiles:', importedFiles);

    let importIdentifiers = importStatements
      .map(([p, n]) => [p, n.identifiers ? n.identifiers : n])
      .map(([p, n]) => [p, n.declarations ? n.declarations : n]);
    console.log('importIdentifiers:', importIdentifiers);

    /*  importIdentifiers = importIdentifiers.map(([p, n]) =>
    n
      .map(decl => (decl.id instanceof ObjectPattern ? decl.id.properties : [decl.id]))
      .flat()
      .map(n => [n.id ? n.id : n])
      .flat()
      .map(n => Identifier.string(n))
  );*/
    console.log('importIdentifiers:', Util.unique(importIdentifiers.flat()).join(', '));
  }

  //  await ConsoleSetup({ depth: Infinity });
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
  WriteFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}
