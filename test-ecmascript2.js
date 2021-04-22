import * as deep from 'deep';
import { ECMAScriptParser, Lexer } from './lib/ecmascript/parser2.js';
import { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, Program, ModuleDeclaration, ModuleSpecifier, ImportDeclaration, ImportSpecifier, ImportDefaultSpecifier, ImportNamespaceSpecifier, Super, Expression, FunctionLiteral, Pattern, Identifier, Literal, RegExpLiteral, TemplateLiteral, BigIntLiteral, TaggedTemplateExpression, TemplateElement, ThisExpression, UnaryExpression, UpdateExpression, BinaryExpression, AssignmentExpression, LogicalExpression, MemberExpression, ConditionalExpression, CallExpression, DecoratorExpression, NewExpression, SequenceExpression, Statement, EmptyStatement, DebuggerStatement, LabeledStatement, BlockStatement, FunctionBody, StatementList, ExpressionStatement, Directive, ReturnStatement, ContinueStatement, BreakStatement, IfStatement, SwitchStatement, SwitchCase, WhileStatement, DoWhileStatement, ForStatement, ForInStatement, ForOfStatement, WithStatement, TryStatement, CatchClause, ThrowStatement, Declaration, ClassDeclaration, ClassBody, MethodDefinition, MetaProperty, YieldExpression, FunctionArgument, FunctionDeclaration, ArrowFunctionExpression, VariableDeclaration, VariableDeclarator, ObjectExpression, Property, ArrayExpression, JSXLiteral, AssignmentProperty, ObjectPattern, ArrayPattern, RestElement, AssignmentPattern, AwaitExpression, SpreadElement, ExportNamedDeclaration, ExportSpecifier, AnonymousDefaultExportedFunctionDeclaration, AnonymousDefaultExportedClassDeclaration, ExportDefaultDeclaration, ExportAllDeclaration } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';
import { ImmutablePath } from './lib/json.js';
import Tree from './lib/tree.js';
import { ConsoleSetup } from './lib/consoleSetup.js';

let filesystem;
let lexer, parser;

Error.stackTraceLimit = 100;

const testfn = () => true;
const testtmpl = `this is\na test`;

const source = `this.define('DecimalDigit', /[0-9]/);`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

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
  //console.log('printAst:', Util.abbreviate(output), Util.decodeAnsi(output));
  return output;
}

let files = {};

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: 10, compact: false });
  await PortableFileSystem(fs => (filesystem = fs));
  console.log('main', { args });
  console.log('console.options', Object.keys(console.options));
  console.options.compact = 3;

  let params = Util.getOpt({
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${Util.getArgs()[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n')
          );
          Util.exit(0);
        },
        'h'
      ],
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
      debug: [false, null, 'x'],
      '@': 'input'
    },
    args
  );

  //params.debug ??= true;

  if(Util.getPlatform() == 'quickjs') {
    await import('os').then(os => {
      os.signal(os.SIGINT, () => {
        console.log(`Got SIGINT. (${os.SIGINT})`);
        Util.putStack();
        Util.exit(1);
      });
      console.log(`SIGINT (${os.SIGINT}) handler installed`);
    });
  }

  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
  });

  const time = () => Date.now() / 1000;

  if(params['@'].length == 0) params['@'].push(null); //'./lib/ecmascript/parser.js');
  for(let file of params['@']) {
    let error;

    const processing = /*Util.instrument*/ () => processFile(file, params);

    let start = await time(),
      end;
    let times = [];
    // Util.safeCall(processFile, file, params);
    try {
      await processing(); //.catch(err => console.log('processFile ERROR:', err));
    } catch(error) {
      console.log('ERROR:', error.message);
      console.log('STACK:', error.stack);
    }

    end = await time();

    times.push(Date.now());

    //processFile(file, params);
    files[file] = finish(error);

    if(error) {
      Util.putError(error);
      //Util.exit(1);
      break;
    }

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  Util.exit(Number(files.length == 0));
}

function processFile(file, params) {
  let data, b, ret;
  const { debug } = params;
  if(file == '-') file = '/dev/stdin';
  if(file && filesystem.exists(file)) {
    data = filesystem.readFile(file);
    //console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log('OK, data: ', Util.abbreviate(Util.escape(data)));
  ECMAScriptParser.instrumentate();

  let ast, error;
  globalThis.parser = parser = null;
  globalThis.parser = parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  console.log('prototypeChain:', Util.getPrototypeChain(parser));
  console.log('parser:', parser);
  // console.log('parser.parseProgram:', parser.parseProgram);

  try {
    ast = parser.parseProgram();
    console.log('ast:', console.config({ depth: 1 }), ast);
  } catch(err) {
    console.log('parseProgram token', parser.token);
    console.log('parseProgram loc', parser.lexer.loc + '');
    console.log('parseProgram ERROR:', err);
    if(Util.isObject(err)) {
      console.log('parseProgram ERROR message:', err.message);
      console.log('parseProgram ERROR stack:', err.stack);
    }
    throw err;
  }

  console.log('Parsed: ', console.config({ depth: 1 }), ast);
  parser.assoc(ast);
  //console.log('parser.assocMap: ', console.config({depth: 0 }), [...parser.assoc.map.keys()]);

  deep.forEach(ast,
    (node, ptr) => {
      const { loc } = node;
      //console.log('node', console.config({depth: 1 }), node, loc);
    },
    null,
    deep.TYPE_OBJECT
  );
  parser.addCommentsToNodes(ast);

  WriteFile(params['output-ast'] ?? file.replace(/.*\//g, '') + '.ast.json',
    JSON.stringify(ast /*.toJSON()*/, null, 2)
  );

  let node2path = new WeakMap();

  /*let flat = deep.flatten(ast,
    new Map(),
    node => node instanceof ESNode || Util.isArray(node),
    (path, value) => {
       node2path.set(value, path);
      return [path, value];
    }
  );*/
  let nodeKeys = [];

  const isRequire = node => node instanceof CallExpression && node.callee.value == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map([...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      { comment, pos, len, node }
    ]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  const output_file =
    params['output-js'] ?? file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

  let tree = new Tree(ast);

  let flat = tree.flat(null, ([path, node]) => {
    return !Util.isPrimitive(node);
  });

  const code = printAst(ast, parser.comments, printer);
  //console.log('code:', Util.abbreviate(Util.escape(code)));

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

    console.log('importIdentifiers:', Util.unique(importIdentifiers.flat()).join(', '));
  }

  //  await ConsoleSetup({ depth: Infinity });
  const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral);

  //console.log('templates:', templates);
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

  lexer = parser.lexer;
  let t = [];
  console.log(parser.trace());
  WriteFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}

main(...scriptArgs.slice(1))
  .then(() => console.log('SUCCESS'))
  .catch(error => {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    std.exit(1);
  });
