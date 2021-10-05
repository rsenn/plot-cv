import { ECMAScriptParser } from './lib/ecmascript/parser.js';
import { Printer, PathReplacer } from './lib/ecmascript.js';
import { ESNode, ImportDeclaration, CallExpression } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
import fs from 'fs';
import * as path from 'path';
import { inspect } from 'util';

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

  let params = Util.getOpt(
    {
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
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

  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
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
      Util.putError(error);
      throw error;
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
    //console.log('opened:', file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log(`'${file}' OK, data:`, Util.abbreviate(Util.escape(data)));

  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  try {
    ast = parser.parseProgram();
  } catch(err) {
    console.log('parseProgram token', parser.token);
    console.log('parseProgram loc', parser.lexer.loc + '');
    if(Util.isObject(err)) {
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
    return !Util.isPrimitive(node);
  });
  console.log('flat:', [...flat.keys()]);

  let nodeKeys = [];

  const isRequire = node => node instanceof CallExpression && node.callee.name == 'require';
  const isImport = node => node instanceof ImportDeclaration;

  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      { comment, pos, len, node }
    ]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  const output_file = params['output-js'] ?? path.basename(file, path.extname(file)) + '.es';

  WriteFile(
    params['output-ast'] ?? path.basename(file, path.extname(file)) + '.ast.json',
    JSON.stringify(ast /*.toJSON()*/, null, 2)
  );
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
