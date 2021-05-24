//import 'module-alias/register.js';
import { ECMAScriptParser, Lexer } from './lib/ecmascript/parser.js';
import { PathReplacer } from './lib/ecmascript.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, Program, ModuleDeclaration, ModuleSpecifier, ImportDeclaration, ImportSpecifier, ImportDefaultSpecifier, ImportNamespaceSpecifier, Super, Expression, FunctionLiteral, Pattern, Identifier, Literal, RegExpLiteral, TemplateLiteral, BigIntLiteral, TaggedTemplateExpression, TemplateElement, ThisExpression, UnaryExpression, UpdateExpression, BinaryExpression, AssignmentExpression, LogicalExpression, MemberExpression, ConditionalExpression, CallExpression, DecoratorExpression, NewExpression, SequenceExpression, Statement, EmptyStatement, DebuggerStatement, LabeledStatement, BlockStatement, FunctionBody, StatementList, ExpressionStatement, Directive, ReturnStatement, ContinueStatement, BreakStatement, IfStatement, SwitchStatement, SwitchCase, WhileStatement, DoWhileStatement, ForStatement, ForInStatement, ForOfStatement, WithStatement, TryStatement, CatchClause, ThrowStatement, Declaration, ClassDeclaration, ClassBody, MethodDefinition, MetaProperty, YieldExpression, FunctionArgument, FunctionDeclaration, ArrowFunctionExpression, VariableDeclaration, VariableDeclarator, ObjectExpression, Property, ArrayExpression, JSXLiteral, AssignmentProperty, ObjectPattern, ArrayPattern, RestElement, AssignmentPattern, AwaitExpression, SpreadElement, ExportNamedDeclaration, ExportSpecifier, AnonymousDefaultExportedFunctionDeclaration, AnonymousDefaultExportedClassDeclaration, ExportDefaultDeclaration, ExportAllDeclaration } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import { ImmutablePath } from './lib/json.js';
import Tree from './lib/tree.js';
import fs from 'fs';
import * as deep from './lib/deep.js';
import { Console } from 'console';

let lexer, parser;

Error.stackTraceLimit = 1_0_0;

const testfn = () => true;
const testtmpl = `this is\na test`;

const source = `this.define('DecimalDigit', /[0-9]/);`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  data = data.trim();

  if(data != '') {
    fs.writeFileSync(name, data + '\n');
    console.log(`Wrote ${name}: ${data.length} bytes`);
  }
}

function printAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);
  //console.log('printAst:', Util.abbreviate(output), Util.decodeAnsi(output));
  return output;
}

let files = {};

function main(...args) {
  console.log('main', args);
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      colors: true,
      depth: 8,
      maxArrayLength: 100,
      compact: 3,
      customInspect: true
    }
  });

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
      debug: [
        false,
        function(v, r, o, result) {
          const thisObj = this;
          //          console.log('debug', { v, r, o, result,thisObj });
          return (result.debug | 0) + 1;
        },
        'x'
      ],
      '@': 'input'
    },
    args
  );

  console.log('params.debug', params.debug);
  // params.debug ??= true;
  if(params.debug) ECMAScriptParser.instrumentate();

  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
  });

  const time = () => Date.now() / 1000;

  if(params['@'].length == 0) params['@'].push(Util.getArgv()[1]);

  console.log(`params['@']`, params?.['@']);
  for(let file of params['@']) {
    let error;

    const processing = /*Util.instrument*/ () => processFile(file, params);

    // Util.safeCall(processFile, file, params);
    try {
      processing(); //.catch(err => console.log('processFile ERROR:', err));
    } catch(error) {
      if(error) {
        console.log?.('ERROR:', error?.message);
        console.log?.('STACK:', error?.stack);
      } else {
        console.log('ERROR:', error);
      }
      if(error !== null) throw error;
    }

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
  if(file && fs.existsSync(file)) {
    data = fs.readFileSync(file, 'utf8');
    console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log('OK, data: ', Util.abbreviate(Util.escape(data)));
  if(debug) ECMAScriptParser.instrumentate();
  console.log('ECMAScriptParser:', ECMAScriptParser);

  let ast, error;
  globalThis.parser = parser = null;
  globalThis.parser = parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  try {
    ast = parser.parseProgram();
  } catch(err) {
    console.log('parseProgram token', parser.token);
    console.log('parseProgram loc', parser.lexer.loc + '');

    if(err !== null) {
      console.log('parseProgram ERROR message:', err?.message);
      console.log('parseProgram ERROR stack:',
        (err.stack + '')
          .split(/\n/g)
          .filter(line => !/at (esfactory|call \(native\))/.test(line))
          .join('\n')
      );
      Util.exit(1);
      throw err;
    } else {
      console.log('parseProgram ERROR:', err);
      throw new Error(`parseProgram`);
    }
  }

  //console.log('Parsed: ', ast);
  /* parser.assoc(ast);
  console.log('parser.assocMap: ', console.config({depth: 0 }), [...parser.assoc.map.keys()]);*/

  /*  deep.forEach(ast,
    (node, ptr) => {
      const { loc } = node;
      //console.log('node', console.config({depth: 1 }), node, loc);
    },
    null,
    deep.TYPE_OBJECT
  );*/
  parser.addCommentsToNodes(ast);

  WriteFile(params['output-ast'] ?? file.replace(/.*\//g, '') + '.ast.json',
    JSON.stringify(ast /*.toJSON()*/, null, 2)
  );

  let node2path = new WeakMap();
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

  WriteFile(output_file, code);

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

let error;
try {
  main(...Util.getArgs().slice(1));
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    console.log('FAIL');
    Util.exit(1);
  } else {
    console.log('SUCCESS');
  }
}
