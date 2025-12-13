import { ReadFile, WriteFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { ECMAScriptParser, PathReplacer } from './lib/ecmascript.js';
import { CallExpression, ESNode, Identifier, ImportDeclaration, TemplateLiteral } from './lib/ecmascript/estree.js';
import Printer from './lib/ecmascript/printer.js';
import Tree from './lib/tree.js';
let fs;

const testfn = () => true;

const testtmpl = `this is\na test`;
const source = `console.log(...cols.map((col, i) => (col + '').replaceAll('\n', '\\n').padEnd(colSizes[i])));`;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  data = data.trim();
  if(data != '') {
    fs.writeFile(name, data + '\n');
    console.log(`Wrote${name}${data.length}bytes`);
  }
}

function printAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);
  return output;
}

let files = {};
async function main(...args) {
  let params = getOpt(
    {
      ['output-ast']: [true, null, 'a'],
      ['output-js']: [true, null, 'o'],
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage:${getArgs()[0]}[OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  -${(name + ', -' + ch).padEnd(20)}`).join('\n'));
          exit(0);
        },
        'h'
      ],
      debug: [false, null, 'x'],
      ['@']: 'input'
    },
    args
  );
  console.log(`Platform:${getPlatform()}`);
  if(getPlatform() == 'quickjs') {
    await import('os').then(os => {
      console.log('os:', os);
      os.signal(os.SIGINT, () => {
        console.log(`Got SIGINT. ${os.SIGINT}`);
        putStack();
        exit(1);
      });
      console.log(`SIGINT ${os.SIGINT} handler installed`);
    });
  }
  defineGettersSetters(globalThis, {
    printer: once(
      () =>
        new Printer({
          colors: false,
          indent: 2
        })
    )
  });
  console.log('params', params);
  const time = () => Date.now() / 1000;
  if(params['@'].length == 0) params['@'].push(null);
  for(let file of params['@']) {
    let error;
    const processing = instrument(() => processFile(file, params));
    let start = await time(),
      end;
    let times = [];
    try {
      await processing();
    } catch(err) {
      console.log('ERROR:', err);
      if(err) {
        console.log('ERROR:', err.message);
        console.log('ERROR:', err.stack);
      }
    }
    end = await time();
    times.push(await now());
    files[file] = finish(error);
    if(error) {
      putError(error);
      break;
    }
    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  exit(Number(files.length == 0));
}

function processFile(file, params) {
  let data, b, ret;
  const { debug } = params;
  if(file == '-') file = '/dev/stdin';
  if(file && fs.exists(file)) {
    data = ReadFile(file);
  } else {
    file = 'stdin';
    data = source;
  }
  console.log('OK, data: ', abbreviate(escape(data)));
  let ast, error;
  globalThis.parser = null;
  globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);
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
  let nodeKeys = [];
  const isRequire = node => node instanceof CallExpression && node.callee.name == 'require';
  const isImport = node => node instanceof ImportDeclaration;
  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
      pos * 10 - 1,
      {
        comment,
        pos,
        len,
        node
      }
    ]),
    (a, b) => a - b
  );
  const output_file = params['output-js'] ?? file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';
  let tree = new Tree(ast);
  let flat = tree.flat(null, ([path, node]) => {
    return !isPrimitive(node);
  });
  WriteFile(params['output-ast'] ?? file + '.ast.json', JSON.stringify(ast, null, 2));
  const code = printAst(ast, parser.comments, printer);
  WriteFile(output_file, code);
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

main(...process.argv.slice(1))
  .then(() => console.log('SUCCESS'))
  .catch(error => {
    console.log(`FAIL:${error.message}${error.stack}`);
    exit(1);
  });