import fs from 'fs';
import { ReadFile, WriteFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { ECMAScriptParser, PathReplacer } from './lib/ecmascript.js';
import { CallExpression, ESNode, Identifier, ImportDeclaration, TemplateLiteral } from './lib/ecmascript/estree.js';
import Printer from './lib/ecmascript/printer.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
import { getOpt,defineGettersSetters,once,abbreviate,isObject } from './lib/misc.js';

const testfn = () => true;
const testtmpl = `this is\na test`;

const source = `console.log(...cols.map((col, i) => (col + '').replaceAll('\n', '\\n').padEnd(colSizes[i])));`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

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
  //console.log('printAst:', abbreviate(output), decodeAnsi(output));
  return output;
}

let files = {};

async function main(...args) {
  console.log('main', args);
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      colors: true,
      depth: 1,
      maxArrayLength: 10,
      compact: 3,
      customInspect: true,
      hideKeys: ['range', 'loc']
    }
  });
  console.log('console.options', console.options);

  let params = getOpt(
    {
      'output-ast': [true, null, 'a'],
      'output-js': [true, null, 'o'],
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${scriptArgs[0]} [OPTIONS]\n`);
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

  //  params.debug ??= true;

  /*await signal('SIGINT', () => {
    console.log(`Got SIGINT. (${os.SIGINT})`);
    putStack();
    exit(1);
  }).then(() => console.log(`SIGINT (${os.SIGINT}) handler installed`));*/

  defineGettersSetters(globalThis, {
    printer: once(() => new Printer({ colors: false, indent: 2 }))
  });

  console.log('params', params);
  const time = () => Date.now() / 1000;

  if(params['@'].length == 0) params['@'].push(null); //'./lib/ecmascript/parser.js');
  for(let file of params['@']) {
    let error;

    const processing = () => processFile(file, params);

    // safeCall(processFile, file, params);
    try {
      await processing(); //.catch(err => console.log('processFile ERROR:', err));
    } catch(err) {
      console.log('ERROR:', err);
      if(err) {
        console.log('ERROR:', err.message);
        console.log('ERROR:', err.stack);
      }
    }

    //processFile(file, params);
    // files[file] = finish(error);

    if(error) {
      putError(error);
      //exit(1);
      break;
    }

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  std.exit(Number(files.length == 0));
}

function processFile(file, params) {
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
  console.log('OK, data: ', data);
  console.log('OK, data: ', abbreviate(escape(data)));

  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);

  // console.log('prototypeChain:', getPrototypeChain(parser));

  try {
    ast = parser.parseProgram();
  } catch(err) {
    console.log('parseProgram token', parser.token);
    console.log('parseProgram loc', parser.lexer.loc + '');
    console.log('parseProgram ERROR:', err);
    if(isObject(err)) {
      console.log('parseProgram ERROR message:', err.message);
      console.log('parseProgram ERROR stack:', err.stack);
    }
  }

  console.log('Parsed: ', ast);

  parser.addCommentsToNodes(ast);

  let node2path = new WeakMap();

  /*let flat = deep.flatten(ast,
    new Map(),
    node => node instanceof ESNode || Array.isArray(node),
    (path, value) => {
       node2path.set(value, path);
      return [path, value];
    }
  );*/
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

  let flat = tree.flat(null, ([path, node]) => {
    return isObject(node);
  });

  Write(params['output-ast'] ?? file + '.ast.json', JSON.stringify(ast /*.toJSON()*/, null, 2));

  const code = printAst(ast, parser.comments, printer);
  //console.log('code:', abbreviate(escape(code)));

  Write(output_file, code);

  function getImports() {
    const imports = [...flat].filter(([path, node]) => isRequire(node) || isImport(node));
    const importStatements = imports.map(([path, node]) => (isRequire(node) || true ? path.slice(0, 2) : path)).map(path => [path, deep.get(ast, path)]);

    console.log('imports:', new Map(imports.map(([path, node]) => [ESNode.assoc(node).position, node])));
    console.log('importStatements:', importStatements);

    const importedFiles = imports.map(([pos, node]) => Identifier.string(node.source || node.arguments[0]));
    console.log('importedFiles:', importedFiles);

    let importIdentifiers = importStatements.map(([p, n]) => [p, n.identifiers ? n.identifiers : n]).map(([p, n]) => [p, n.declarations ? n.declarations : n]);
    console.log('importIdentifiers:', importIdentifiers);

    console.log('importIdentifiers:', unique(importIdentifiers.flat()).join(', '));
  }

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

main(...scriptArgs.slice(1))
  .then(() => console.log('SUCCESS'))
  .catch(error => {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    exit(1);
  });