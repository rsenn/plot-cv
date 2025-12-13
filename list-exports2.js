import * as fs from 'fs';
import * as path from 'path';
import { WriteFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { ECMAScriptParser, ExportDefaultDeclaration, Identifier, ImportDeclaration, ImportSpecifier, Literal, PathReplacer, Printer } from './lib/ecmascript.js';
import { inspect } from './lib/misc.js';
import { Stack } from './lib/stack.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
let lexer, parser, childProcess;

globalThis.fs = fs;

function PrintAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);

  return output;
}

let files = {};

function main(...argv) {
  globalThis.console = new Console({
    stdout: process.stdout,
    stderr: process.stderr,
    inspectOptions: {
      colors: true,
      depth: 2,
      breakLength: 1000,
      maxStringLength: 300,
      maxArrayLength: Infinity,
      compact: 1,
      customInspect: true
    }
  });

  let params = getOpt(
    {
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${getArgs()[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n'));
          exit(0);
        },
        'h'
      ],
      'output-ast': [true, null, 'a'],
      output: [true, null, 'o'],
      debug: [false, (v, r, o, result) => (result.debug | 0) + 1, 'x'],
      '@': 'input'
    },
    argv
  );

  if(params.debug >= 2) ECMAScriptParser.instrumentate();
  defineGettersSetters(globalThis, {
    printer: once(() => new Printer({ colors: false, indent: 2 }))
  });
  const time = () => Date.now() / 1000;
  if(params['@'].length == 0) params['@'].push(getArgv()[1]);
  for(let file of params['@']) {
    let error;
    const processing = () => ProcessFile(file, params);
    try {
      processing();
    } catch(error) {
      if(error !== null) throw error;
    }
    files[file] = Finish(error);
    if(error) {
      putError(error);
      break;
    }
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  exit(Number(files.length == 0));
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
      `FAIL: ${className(error)} ${error.message}`,
      `\n  ` + new Stack(error.stack, fr => fr.functionName != 'esfactory').toString().replace(/\n/g, '\n  ').split('\n').slice(0, 10).join('\n')
    );
    console.log('FAIL');
    exit(1);
  } else {
    console.log('SUCCESS');
  }
}

function ProcessFile(file, params) {
  let data, b, ret, parser;
  const { debug } = params;
  console.log(`Processing file '${file}'...`);
  if(file == '-') file = '/dev/stdin';
  if(file && fs.existsSync(file)) {
    data = ReadFile(file, 'utf8');
  } else {
    file = 'stdin';
    data = ReadFile('/dev/stdin', 'utf8');
  }
  if(debug >= 2) ECMAScriptParser.instrumentate();
  let ast, error;
  parser = globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, debug);
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
  params['output-ast'] ??= path.basename(file) + '.ast.json';
  console.log('output-ast', params['output-ast']);
  WriteFile(params['output-ast'], JSON.stringify(ast, null, 2));
  let node2path = new WeakMap();
  let nodeKeys = [];
  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node }]),
    (a, b) => a - b
  );
  let tree = new Tree(ast);
  ShowOutput(ast, tree, null, file, params);
}

function Finish(err) {
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
  let t = [];
  if(globalThis.parser) {
    lexer = globalThis.parser.lexer;
    console.log(globalThis.parser.trace());
  }
  if(fail) {
    console.log('\nERR:', err.msg, '\n', parser.lexer.currentLine());
  }
  return !fail;
}

function ShowOutput(ast, tree, flat, file, params) {
  const output_file = params['output-js'] ?? '/dev/stdout';
  const flags = deep.RETURN_VALUE_PATH;
  let nodes = [...deep.select(ast, (node, key) => ['exported', 'imported', 'local'].indexOf(key) != -1, flags), ...deep.select(ast, (node, key) => /Export/.test(node.type), flags)].map(
    ([node, path]) => [node, path.slice(0, -1), deep.get(ast, path.slice(0, -1))]
  );
  let names = nodes.filter(([n, p, parent]) => !/Import/.test(parent.type)).map(([node, path, parent]) => (node.declaration && node.declaration.id ? node.declaration.id : node));
  let defaultExport = deep.find(ast, node => node instanceof ExportDefaultDeclaration, deep.RETURN_VALUE);
  if(!file.startsWith('./') && !file.startsWith('/') && !file.startsWith('..')) file = './' + file;
  let importNode = new ImportDeclaration(
    [
      ...names
        .reduce((acc, n) => {
          let name = NodeToName(n);
          if(name && acc.indexOf(name) == -1) acc.push(name);
          return acc;
        }, [])
        .map(name => new ImportSpecifier(new Identifier(name)))
    ],
    new Literal(`'${file}'`)
  );
  let code = PrintAst(importNode).trim() + '\n';
  if(output_file == '/dev/stdout') fs.writeSync(process.stdout.fd ?? process.stdout, code);
  else WriteFile(output_file, code);
}

function NodeType(node) {
  if(typeof node == 'object') return typeof node.type == 'string' ? node.type : className(node);
}

function NodeToName(node) {
  let id;
  if(Array.isArray(node) && node.length == 2) node = node[0];
  if(node instanceof ExportDefaultDeclaration) return null;
  if(typeof node == 'object' && node != null) {
    if(node instanceof Identifier || node.type == 'Identifier' || 'name' in node) id = node.name;
    if(!id && node.id && node.id instanceof Identifier) id = node.id;
    if(!id && node.name) id = node.name;
    if(!id) id = deep.find(node, (path, key) => ['id', 'exported'].indexOf(key) != -1, deep.RETURN_VALUE);
    if(id instanceof Identifier) id = Identifier.string(id);
  } else if(typeof node == 'number' || typeof node == 'string') id = node;
  if(!id) {
    let entries = deep.select(node, (n, p) => p[p.length - 1] == 'id' && typeof n == 'object' && n != null && (n.name || (n.type ?? n.kind) == 'Identifier'), deep.RETURN_VALUE_PATH);
    let idList = deep.select(node, (n, p) => p[p.length - 1] == 'id' && typeof n == 'object' && n != null && (n.type ?? n.kind) == 'Identifier', deep.RETURN_VALUE_PATH);
    let firstId = idList[0];
    entries = entries.filter(([n, p]) => p.indexOf('init') == -1);
    console.log(
      'entries',
      entries.map(([n, p]) => [p.join('.'), NodeType(deep.get(node, [...p].slice(0, -1))), n.type, n.name, p.length])
    );
    entries = entries.map(([n, p]) => [n.name, p.length, NodeType(deep.get(node, p.slice(0, -1))), [...Ancestors(n, p, (n, k) => [k, NodeType(n) ?? `[${k}]`, n.name])]]);
    id = entries.map(e => e[0]);
    console.log('node.type', node.type);
  }
  if(!id) {
    let message =
      'NodeToName(' +
      node.kind +
      ' ' +
      abbreviate(
        inspect(node, {
          breakLength: 1000,
          multiline: false,
          compact: 100,
          depth: 2,
          colors: true,
          maxStringLength: 30,
          maxArrayLength: 2
        }),
        500
      );
    console.log(message);
    let e = new Error(message);
    throw e;
  }
  return id;
}

function* Ancestors(obj, path, t = a => a) {
  let i = 0;
  for(let k of path) {
    if(typeof obj == 'object') yield t(obj, k);
    try {
      obj = obj[k];
    } catch(e) {
      break;
    }
    ++i;
  }
}

function NodeParent(obj, path) {
  let r;
  for(let n of Ancestors(obj, path)) if(n && n.type) r = n;
  return r;
}