//import 'module-alias/register.js';
import { ECMAScriptParser } from './lib/ecmascript/parser2.js';
import { PathReplacer } from './lib/ecmascript.js';
import Printer from './lib/ecmascript/printer.js';
import { ImportDeclaration, ImportSpecifier, Identifier, Literal, TemplateLiteral, CallExpression, ExportNamedDeclaration, ExportDefaultDeclaration } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import Tree from './lib/tree.js';
import fs from 'fs';
import * as deep from './lib/deep.js';
import { Console } from 'console';
import { Stack } from './lib/stack.js';

let lexer, parser;

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  data = data.trim();

  if(data != '') {
    fs.writeFileSync(name, data + '\n');
    let st = fs.statSync(name);
    if(st && st.isFile()) console.log(`Wrote ${name}: ${data.length} bytes`);
  }
}

function PrintAst(ast, comments, printer = globalThis.printer) {
  let output = printer.print(ast);
  //console.log('PrintAst:', Util.abbreviate(output), Util.decodeAnsi(output));
  return output;
}

let files = {};

function main(...argv) {
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      colors: true,
      depth: 5,
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

  /*let nodes = deep.select(ast, node => /Import|Export/.test(Util.className(node)), deep.RETURN_VALUE);
  console.log('nodes:', nodes);/
*/ const flags = deep.RETURN_VALUE_PATH;
  let nodes = [...deep.select(ast, (node, key) => ['exported', 'imported', 'local'].indexOf(key) != -1, flags), ...deep.select(ast, (node, key) => /Export/.test(node.type), flags)].map(([node, path]) => [node, path.slice(0, -1), deep.get(ast, path.slice(0, -1))]);

  let names = nodes.filter(([n, p, parent]) => !/Import/.test(parent.type)).map(([node, path, parent]) => (node.declaration && node.declaration.id ? node.declaration.id : node));

  let defaultExport = deep.find(ast, node => node instanceof ExportDefaultDeclaration, deep.RETURN_VALUE);
  //console.log('names:', names);

  if(!file.startsWith('./') && !file.startsWith('/') && !file.startsWith('..')) file = './' + file;

  let importNode = new ImportDeclaration(
    [
      ...names.reduce((acc, n) => {
        let name = NodeToName(n);
        if(name) acc.push(new ImportSpecifier(new Identifier(name)));
        return acc;
      }, [])
    ],
    new Literal(`'${file}'`)
  );
  //console.log('importNode', importNode);

  let code = PrintAst(importNode);

  WriteFile(output_file, code);
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

  if(!id) throw new Error(`NodeToName(${inspect(node, { breakLength: 120, multiline: true, compact: 0 })})`);

  return id;
}

function ProcessFile(file, params) {
  let data, b, ret;
  const { debug } = params;
  console.log('ProcessFile', { debug });
  if(file == '-') file = '/dev/stdin';
  if(file && fs.existsSync(file)) {
    data = fs.readFileSync(file, 'utf8');
  } else {
    file = 'stdin';
    data = source;
  }
  if(debug >= 2) ECMAScriptParser.instrumentate();
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
  lexer = parser.lexer;
  let t = [];
  console.log(parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  return !fail;
}
