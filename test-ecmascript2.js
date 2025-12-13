import fs from 'fs';
import { ReadFile, WriteFile } from './io-helpers.js';
import { PathReplacer } from './lib/ecmascript.js';
import { CallExpression, ImportDeclaration, TemplateLiteral } from './lib/ecmascript/estree.js';
import { ECMAScriptParser } from './lib/ecmascript/parser.js';
import Printer from './lib/ecmascript/printer.js';
import { Stack } from './lib/stack.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
import { getOpt,defineGettersSetters,once,abbreviate,escape,isObject } from 'util';

let lexer, parser;

Error.stackTraceLimit = 100;

const testfn = () => true;

const testtmpl = `this is\na test`;

const source = `this.define('DecimalDigit', /[0-9]/);`;

const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

const SliceOffEmpty = (arr, pred = item => item != '') => arr.slice(arr.findIndex(pred));
const FilterOutEmpty = (arr, pred = item => item != '') => arr.filter(pred);

globalThis.GetStack = (stack, cond = fr => fr.functionName != 'esfactory') => new Stack(stack, cond);

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
}

function Write(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  data = data.trim();

  if(data != '') {
    WriteFile(name, data + '\n');
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
  globalThis.console = new Console(std.out, {
    inspectOptions: {
      colors: true,
      depth: 8,
      breakLength: null,
      maxStringLength: Infinity,
      maxArrayLength: 100,
      compact: 2,
      customInspect: true
    }
  });

  let params = getOpt(
    {
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${scriptArgs[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => ('  --' + name + ', -' + ch).padEnd(20)).join('\n'));
          process.exit(0);
        },
        'h'
      ],
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
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

  // params.debug ??= true;
  if(params.debug) ECMAScriptParser.instrumentate();

  defineGettersSetters(globalThis, {
    printer: once(() => new Printer({ colors: false, indent: 2 }))
  });

  const time = () => Date.now() / 1000;

  if(params['@'].length == 0) params['@'].push(process.argv[1]);

  for(let file of params['@']) {
    let error;

    const processing = /*Util.instrument*/ () => processFile(file, params);

    // Util.safeCall(processFile, file, params);
    try {
      processing(); //.catch(err => console.log('processFile ERROR:', err));
    } catch(error) {
      if(error) {
        console.log('ERROR:', error?.message);
        console.log('STACK:\n  ' + new Stack(error?.stack, fr => fr.functionName != 'esfactory').toString().replace(/\n/g, '\n  '));
      } else {
        console.log('ERROR:', error);
      }
      if(error !== null) throw error;
    }

    //processFile(file, params);
    files[file] = finish(error);

    if(error) {
      Util.putError(error);
      //process.exit(1);
      break;
    }

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}

function ParseECMAScript(file, params) {
  let data, b, ret;
  const { debug } = params;
  if(file == '-') file = '/dev/stdin';
  if(file && fs.existsSync(file)) {
    data = ReadFile(file, 'utf8');
    console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log('OK, data: ', abbreviate(escape(data)));
  if(debug) ECMAScriptParser.instrumentate();
  console.log('ECMAScriptParser:', ECMAScriptParser);

  let ast, error;
  globalThis.parser = parser = null;
  globalThis.parser = parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  try {
    ast = parser.parseProgram();
  } catch(err) {
    const tokens = [...parser.processed, ...parser.tokens];
    const token = tokens[tokens.length - 1];
    /*console.log(
      'parseProgram tokens',
      tokens.slice(-3).map(tok => [tok, new Stack(tok.stack.slice(0, 3)) + ''])
    );*/
    console.log('parseProgram token', token);
    /*
    if(token)   console.log('parseProgram token.stack\n  ' + token.stack.toString().replace(/\n/g, '\n  '));
      console.log('parseProgram loc', token.loc + '');
    console.log('parseProgram stateStack', parser.lexer.stateStack);
     console.log('parseProgram parser.stack', parser.stack.map(({frame,...entry}) =>  [entry,frame?.loc]));*/

    if(err !== null) {
      console.log('parseProgram ERROR message:', err?.message);
      console.log('parseProgram ERROR stack:\n  ' + new Stack(err?.stack, (fr, i) => fr.functionName != 'esfactory' && i < 5).toString().replace(/\n/g, '\n  '));
      //console.log('parseProgram parser.stack\n', parser.stack .map(entry => [entry, parser.constructor.stackMap.get(entry)]) .map(([entry, frame]) => [entry.position + '', frame ? frame + '' : entry.methodName]));
      throw err;
    } else {
      console.log('parseProgram ERROR:', err);
      throw new Error('parseProgram');
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
  return ast;
}

function processFile(file, params) {
  let ast = ParseECMAScript(file, params);

  Write(params['output-ast'] ?? file.replace(/.*\//g, '') + '.ast.json', JSON.stringify(ast /*.toJSON()*/, null, 2));

  let node2path = new WeakMap();
  let nodeKeys = [];

  const isRequire = node => node instanceof CallExpression && node.callee.name == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node }]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  const output_file = params['output-js'] ?? file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

  let tree = new Tree(ast);

  let flat = tree.flat(null, ([path, node]) => isObject(node));

  const code = printAst(ast, parser.comments, printer);

  Write(output_file, code);

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
  Write('trace.log', parser.trace());
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
    console.log('FAIL: ' + error.message, '\n  ' + new Stack(error.stack, fr => fr.functionName != 'esfactory').toString().replace(/\n/g, '\n  '));
    console.log('FAIL');
    process.exit(1);
  } else {
    console.log('SUCCESS');
  }
}