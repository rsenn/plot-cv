//import 'module-alias/register.js';
import { ECMAScriptParser } from './lib/ecmascript/parser.js';
import { PathReplacer } from './lib/ecmascript.js';
import Printer from './lib/ecmascript/printer.js';
import { ImportDeclaration, ImportSpecifier, Identifier, Literal, TemplateLiteral, CallExpression, ExportNamedDeclaration, ExportDefaultDeclaration } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import Tree from './lib/tree.js';
import fs from 'fs';
import deep from './lib/deep.js';
import { Stack } from './lib/stack.js';
import { IfDebug, ReadFile, LoadHistory, ReadJSON, MapFile, ReadBJSON, WriteFile, WriteJSON, WriteBJSON, DirIterator, RecursiveDirIterator } from './io-helpers.js';
import PortableFileSystem from './lib/filesystem.js';
import PortableChildProcess from './lib/childProcess.js';
import { Console } from 'console';
import { inspect } from 'util';
import process from 'process';

let lexer, parser, childProcess;

function PrintAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);
  //console.log('PrintAst:', Util.abbreviate(output), Util.decodeAnsi(output));
  return output;
}

let files = {};

async function main(...argv) {
  console.log('process:',process);
 await PortableFileSystem(fs => (globalThis.fs = filesystem = fs));
  //await PortableChildProcess(cp => (childProcess = cp));
  globalThis.console = new Console({
    stdout: process.stderr,
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

  let params = Util.getOpt(
    {
      help: [
        false,
        (v, r, o) => {
          console.log(`Usage: ${Util.getArgs()[0]} [OPTIONS]\n`);
          console.log(o.map(([name, [arg, fn, ch]]) => `  --${(name + ', -' + ch).padEnd(20)}`).join('\n'));
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
  if(params.debug >= 2) ECMAScriptParser.instrumentate();
  Util.defineGettersSetters(globalThis, {
    printer: Util.once(() => new Printer({ colors: false, indent: 2 }))
  });
  const time = () => Date.now() / 1000;
  if(params['@'].length == 0) params['@'].push(Util.getArgv()[1]);
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
      Util.putError(error);
      break;
    }
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  Util.exit(Number(files.length == 0));
}

let error;
try {
  const argv = [...(process?.argv ?? scriptArgs)].slice(2);
  main(...argv);
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}`, `\n  ` + new Stack(error.stack, fr => fr.functionName != 'esfactory').toString().replace(/\n/g, '\n  ').split('\n').slice(0, 10).join('\n'));
    console.log('FAIL');
    Util.exit(1);
  } else {
    console.log('SUCCESS');
  }
}

function ShowOutput(ast, tree, flat, file, params) {
  const output_file = params['output-js'] ?? '/dev/stdout';

  const flags = deep.RETURN_VALUE_PATH;
  let nodes = [...deep.select(ast, (node, key) => ['exported', 'imported', 'local'].indexOf(key) != -1, flags), ...deep.select(ast, (node, key) => /Export/.test(node.type), flags)].map(([node, path]) => [node, path.slice(0, -1), deep.get(ast, path.slice(0, -1))]);
  let names = nodes.filter(([n, p, parent]) => !/Import/.test(parent.type)).map(([node, path, parent]) => (node.declaration && node.declaration.id ? node.declaration.id : node));
  let defaultExport = deep.find(ast, node => node instanceof ExportDefaultDeclaration, deep.RETURN_VALUE);
  //console.log('names:', names);
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
  //console.log('importNode', importNode);

  let code = PrintAst(importNode);

  WriteFile(output_file, code);
}
function NodeType(node) {
  if(typeof node == 'object') return typeof node.type == 'string' ? node.type : Util.className(node);
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

  if(/*0 ||*/ !id) {
    let entries = deep.select(node, (n, p) => p[p.length - 1] == 'id' && typeof n == 'object' && n != null && (n.name || (n.type ?? n.kind) == 'Identifier'), deep.RETURN_VALUE_PATH);
    let idList = deep.select(node, (n, p) => p[p.length - 1] == 'id' && typeof n == 'object' && n != null && (n.type ?? n.kind) == 'Identifier', deep.RETURN_VALUE_PATH);
    let firstId = idList[0];
    entries = entries.filter(([n, p]) => p.indexOf('init') == -1);

    //console.log('entries', entries.map(([n, p]) => [p.join('.'), NodeType(deep.get(node, [...p].slice(0, -1))), n.type, n.name, p.length]));

    entries = entries.map(([n, p]) => [n.name, p.length, NodeType(deep.get(node, p.slice(0, -1))), [...Ancestors(n, p, (n, k) => [k, NodeType(n) ?? `[${k}]`, n.name])]]);

    //console.log('idList', idList.map(([n, p]) => [ n.name, p.length].concat(Util.range(-5,-1).map(x=> NodeType(deep.get(node, p.slice(0, x)))))));
    id = entries.map(e => e[0]);
    console.log('node.type', node.type);
  }
  if(!id) {
    let message =
      'NodeToName(' +
      node.kind +
      ' ' +
      Util.abbreviate(
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
  //console.log(`Ancestors`,`[${path.length}]`);
  for(let k of path) {
    //console.log(` `, ...[i, k, NodeType(obj)].map(col => col+'').map((c,i) =>c.padEnd(i ? 20 : 7)), obj);
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

function ProcessFile(file, params) {
  let data, b, ret, parser;
  const { debug } = params;
  console.log(`Processing file '${file}'...`);
  if(file == '-') file = '/dev/stdin';
  if(file && fs.existsSync(file)) {
    data = fs.readFileSync(file, 'utf8');
  } else {
    file = 'stdin';
    data = source;
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

  WriteFile(params['output-ast'] ?? file.replace(/.*\//g, '') + '.ast.json', JSON.stringify(ast /*.toJSON()*/, null, 2));

  let node2path = new WeakMap();
  let nodeKeys = [];

  let commentMap = new Map(
    [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node }]),
    (a, b) => a - b
  );

  //console.log('commentMap:', commentMap);

  let tree = new Tree(ast);

  let flat = tree.flat(null, ([path, node]) => {
    return !Util.isPrimitive(node);
  });

  ShowOutput(ast, tree, flat, file, params);

  delete globalThis.parser;

  //std.gc();
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
    console.log(Util.className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
  }
  let t = [];
  if(globalThis.parser) {
    lexer = parser.lexer;
    console.log(parser.trace());
  }
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  return !fail;
}
