import fs from 'fs';
import * as path from 'path';
import { ReadFile, WriteFile } from './io-helpers.js';
import { PathReplacer, Printer } from './lib/ecmascript.js';
import { CallExpression, ImportDeclaration } from './lib/ecmascript/estree.js';
import { ECMAScriptParser } from './lib/ecmascript/parser.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
function WriteFile(name, data) {
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
  //console.log('printAst:', abbreviate(output), decodeAnsi(output));
  return output;
}

let files = {};

function main(...args) {
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      colors: true,
      depth: 1,
      maxArrayLength: 10,
      compact: 3,
      customInspect: false
    }
  });

  let params = getOpt(
    {
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${getArgs()[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n'));
          exit(0);
        },
        'h'
      ],
      debug: [
        false,
        function(v, r, o, res) {
          const thisObj = this;
          console.log('debug', { v, r, o, res, thisObj });
        },
        'x'
      ],
      '@': 'input'
    },
    args
  );

  defineGettersSetters(globalThis, {
    printer: once(() => new Printer({ colors: false, indent: 2 }))
  });

  if(params['@'].length == 0) params['@'].push(null); //'./lib/ecmascript/parser.js');

  for(let file of params['@']) {
    let error;

    const processing = () => processFile(file, params);

    try {
      processing();
    } catch(err) {
      if(err) {
        console.log('ERROR:', err.message);
        console.log('ERROR:', err.stack);
      } else {
        console.log('ERROR:', err);
      }
      throw err;
    }

    if(error) {
      putError(error);
      throw error;
    }
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  exit(Number(files.length == 0));
}

function processFile(file, params) {
  let data, b, ret;
  const { debug } = params;
  if(file == '-') file = '/dev/stdin';
  if(file && fs.existsSync(file)) {
    data = ReadFile(file, 'utf8');
    //console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log(`'${file}' OK, data:`, abbreviate(escape(data)));

  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  try {
    ast = parser.parseProgram();
  } catch(err) {
    console.log('parseProgram token', parser.token);
    console.log('parseProgram loc', parser.lexer.loc + '');
    if(isObject(err)) {
      console.log('parseProgram ERROR message:', err.message);
      console.log('parseProgram ERROR stack:', err.stack);
    }
    throw err;
  }

  console.log('Parsed: ', ast);

  parser.addCommentsToNodes(ast);

  let tree = new Tree(ast);

  let node2path = new WeakMap();

  let flat = tree.flat(null, ([path, node]) => {
    return !isPrimitive(node);
  });
  console.log('flat:', [...flat.keys()]);

  let nodeKeys = [];

  const isRequire = node => node instanceof CallExpression && node.callee.name == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node }]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  const output_file = params['output-js'] ?? path.basename(file, path.extname(file)) + '.es';

  WriteFile(params['output-ast'] ?? path.basename(file, path.extname(file)) + '.ast.json', JSON.stringify(ast /*.toJSON()*/, null, 2));
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
    console.log(className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
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

let error;
try {
  main(...getArgs().slice(1));
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    console.log('FAIL');
    exit(1);
  } else {
    console.log('SUCCESS');
  }
}