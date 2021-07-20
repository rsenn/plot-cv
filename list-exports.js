//import 'module-alias/register.js';
import { ECMAScriptParser } from './lib/ecmascript/parser2.js';
import { PathReplacer } from './lib/ecmascript.js';
import Printer from './lib/ecmascript/printer.js';
import {
  estree,
  ESNode,
  Program,
  ModuleDeclaration,
  ModuleSpecifier,
  ImportDeclaration,
  ImportSpecifier,
  ImportDefaultSpecifier,
  ImportNamespaceSpecifier,
  Super,
  Expression,
  FunctionLiteral,
  Pattern,
  Identifier,
  Literal,
  RegExpLiteral,
  TemplateLiteral,
  BigIntLiteral,
  TaggedTemplateExpression,
  TemplateElement,
  ThisExpression,
  UnaryExpression,
  UpdateExpression,
  BinaryExpression,
  AssignmentExpression,
  LogicalExpression,
  MemberExpression,
  ConditionalExpression,
  CallExpression,
  DecoratorExpression,
  NewExpression,
  SequenceExpression,
  Statement,
  EmptyStatement,
  DebuggerStatement,
  LabeledStatement,
  BlockStatement,
  FunctionBody,
  StatementList,
  ExpressionStatement,
  Directive,
  ReturnStatement,
  ContinueStatement,
  BreakStatement,
  IfStatement,
  SwitchStatement,
  SwitchCase,
  WhileStatement,
  DoWhileStatement,
  ForStatement,
  ForInStatement,
  ForOfStatement,
  WithStatement,
  TryStatement,
  CatchClause,
  ThrowStatement,
  Declaration,
  ClassDeclaration,
  ClassBody,
  MethodDefinition,
  MetaProperty,
  YieldExpression,
  FunctionArgument,
  FunctionDeclaration,
  ArrowFunctionExpression,
  VariableDeclaration,
  VariableDeclarator,
  ObjectExpression,
  Property,
  ArrayExpression,
  JSXLiteral,
  AssignmentProperty,
  ObjectPattern,
  ArrayPattern,
  RestElement,
  AssignmentPattern,
  AwaitExpression,
  SpreadElement,
  ExportNamedDeclaration,
  ExportSpecifier,
  AnonymousDefaultExportedFunctionDeclaration,
  AnonymousDefaultExportedClassDeclaration,
  ExportDefaultDeclaration,
  ExportAllDeclaration
} from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import Tree from './lib/tree.js';
import fs from 'fs';
import * as deep from './lib/deep.js';
import { Console } from 'console';
import { Stack } from './lib/stack.js';

let lexer, parser;

Error.stackTraceLimit = 100;

const testfn = () => true;
const testtmpl = `this is\na test`;

const source = `this.define('DecimalDigit', /[0-9]/);`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

const SliceOffEmpty = (arr, pred = item => item != '') => arr.slice(arr.findIndex(pred));
const FilterOutEmpty = (arr, pred = item => item != '') => arr.filter(pred);

globalThis.GetStack = (stack, cond = fr => fr.functionName != 'esfactory') =>
  new Stack(stack, cond);

globalThis.FormatStack = (stack, start, limit) => {
  start ??= fr => /^(expect|match|parse)/.test(fr);

  limit ??= Infinity;
  if(typeof start == 'function') start = stack.findIndex((frame, i) => start(frame, i, stack));
  if(Number.isFinite(start)) stack = stack.slice(start);
  if(Number.isFinite(limit)) stack = stack.slice(0, limit);

  return (
    /*stack.join('\n') || */ '\n' +
    stack
      .filter(fr => fr.functionName != 'esfactory')
      .map(fr => [fr.functionName, fr.fileName, fr.lineNumber])
      .map(([func, file, line]) => '  ' + func.padEnd(40) + file + ':' + line)
      .join('\n')
  );
};
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

function main(...argv) {
   globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      colors: true,
      depth: 2,
      breakLength: null,
      maxStringLength: Infinity,
      maxArrayLength: 100,
      compact: 2,
      customInspect: true
    }
  });
  let params = Util.getOpt(
    {
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${Util.getArgs()[0]} [OPTIONS]\n`);
          console.log(
            o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n')
          );
          Util.exit(0);
        },
        'h'
      ],
      'output-ast': [true, null, 'a'],
      output: [true, null, 'o'],
      debug: [
        false,
        function(v, r, o, result) {
          const thisObj = this;
          // console.log('debug', { v, r, o, result,thisObj });
          return (result.debug | 0) + 1;
        },
        'x'
      ],
      '@': 'input'
    },
    argv
  );
    if(params.debug) ECMAScriptParser.instrumentate();
  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
  });
  const time = () => Date.now() / 1000;
  if(params['@'].length == 0) params['@'].push(Util.getArgv()[1]);
   for(let file of params['@']) {
    let error;
    const processing =  () => processFile(file, params);
    try {
      processing() 
    } catch(error) {
      if(error) {
        console.log?.('ERROR:', error?.message);
        console.log?.(
          'STACK:\n  ' +
            new Stack(error?.stack, fr => fr.functionName != 'esfactory')
              .toString()
              .replace(/\n/g, '\n  ')
        );
      } else {
        console.log('ERROR:', error);
      }
      if(error !== null) throw error;
    }
     files[file] = finish(error);
    if(error) {
      Util.putError(error);
       break;
    }
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
   } else {
    file = 'stdin';
    data = source;
  }
   if(debug) ECMAScriptParser.instrumentate();
 
  let ast, error;
  globalThis.parser = parser = null;
  globalThis.parser = parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);
  try {
    ast = parser.parseProgram();
  } catch(err) {
    const tokens = [...parser.processed, ...parser.tokens];
    const token = tokens[tokens.length - 1];
    if(err !== null) {
      throw err;
    } else {
      throw new Error(`parseProgram`);
    }
  }
  parser.addCommentsToNodes(ast);

  WriteFile(
    params['output-ast'] ?? file.replace(/.*\//g, '') + '.ast.json',
    JSON.stringify(ast /*.toJSON()*/, null, 2)
  );

  let node2path = new WeakMap();
  let nodeKeys = [];

  const isRequire = node => node instanceof CallExpression && node.callee.value == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      { comment, pos, len, node }
    ]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  let tree = new Tree(ast);

  let flat = tree.flat(null, ([path, node]) => {
    return !Util.isPrimitive(node);
  });

  ShowOutput(ast, tree, flat, file, params);
  // const code = printAst(ast, parser.comments, printer);

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
  // WriteFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}

let error;
try {
  const argv = [...(process?.argv ?? scriptArgs)].slice(2);
  main(...argv);
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(
      `FAIL: ${error.message}`,
      `\n  ` +
        new Stack(error.stack, fr => fr.functionName != 'esfactory')
          .toString()
          .replace(/\n/g, '\n  ')
    );
    console.log('FAIL');
    Util.exit(1);
  } else {
    console.log('SUCCESS');
  }
}

function ShowOutput(ast, tree, flat, file, params) {
  const output_file =
    params['output-js'] ?? file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

  let nodes = deep.select(ast, node => /Export/.test(Util.className(node)), deep.RETURN_VALUE);
  let names = [
    ...deep.select(ast, (node, key) => key == 'exported', deep.RETURN_VALUE),
    ...deep
      .select(ast, (node, key) => node instanceof ExportNamedDeclaration, deep.RETURN_VALUE)
      .map(node =>
        node.declaration && node.declaration.id ? node.declaration.id : node.declaration
      )
  ];

  let defaultExport = deep.find(
    ast,
    node => node instanceof ExportDefaultDeclaration,
    deep.RETURN_VALUE
  );

  // let code = nodes.map(node => printAst(node, parser.comments, printer)).join('\n');

  // console.log('nodes:', nodes.map(node => node) );
  console.log(
    'names:',
    names.map(n => NodeToName(n))
  );

  let importNode = new ImportDeclaration(
    [...names.map(n =>  new ImportSpecifier( new Identifier(NodeToName(n))))],
    new Literal(`'${file}'`)
  );

  let code = printAst(importNode);
  console.log('code:', code);
  // console.log('defaultExport:', defaultExport);

  WriteFile(output_file, code);

  //  const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral);
}

function NodeToName(node) {
  let id;
  if(node.name) id = node;
  if(!id)
    id = deep.find(node, (path, key) => ['id', 'exported'].indexOf(key) != -1, deep.RETURN_VALUE);
  if(!id && node.id && node.id instanceof Identifier) id = node.id;
  if(id)
    if(id instanceof Identifier) id = Identifier.string(id);
    else if('name' in id) id = id.name;

  return id;
}
