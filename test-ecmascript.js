import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import {
  estree,
  ESNode,
  TemplateLiteral,
  CallExpression,
  ImportStatement,
  Identifier,
  ObjectBindingPattern
} from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';
import { ImmutablePath } from './lib/json.js';
import { ConsoleSetup } from './consoleSetup.js';

let filesystem;

const testfn = () => true;
const testtmpl = `this is\na test`;

Util.callMain(main, true);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg"> <defs /> */ function dumpFile(name,
  data
) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

globalThis.parser = null;
let files = {};

async function main(...args) {
  if(args.length == 0) args.push('-');
  //await import('tty');
  const stdout = (await import('process')).stdout;

  const breakLength = stdout.columns || process.env.COLUMNS || 80;
  await ConsoleSetup({ breakLength, maxStringLength: breakLength });
  console.log('breakLength:', breakLength);

  filesystem = await PortableFileSystem();

  if(args.length == 0) args.push('./lib/ecmascript/parser.js');
  for(let file of args) {
    let data, b, ret;
    if(file == '-') file = '/dev/stdin';

    if(filesystem.exists(file)) data = filesystem.readFile(file);

    console.log('file:', file);
    console.log('opened:', Util.abbreviate(data));
    let ast, error;

    globalThis.parser = new ECMAScriptParser(data ? data.toString() : data, file, true);
    globalThis.printer = new Printer({ indent: 4 });

    console.log('prototypeChain:', Util.getPrototypeChain(parser));

    console.log('methodNames:', Util.getMethodNames(parser, 2));

    //console.log(new parser.estree.Identifier());
    console.log('OK');

    try {
      ast = parser.parseProgram();

      parser.addCommentsToNodes(ast);

      /*let imports = [...deep.iterate(ast, (node) => node instanceof CallExpression && /console.log/.test(printer.print(node)))].map(([node, path]) => node);
      let importNodes = imports.map((node) => ({ str: printer.print(node), toks: ECMAScriptParser.printToks(parser.tokensForNode(node)) }));

      console.log('imports:', importNodes);*/

      //for(let imp of imports) console.log('tokens:', parser.tokensForNode(imp));
      let flat = deep.flatten(ast,
        new Map(),
        node => node instanceof ESNode,
        (path, value) => {
          path = new Path(path);
          //value = Util.map(value,(k,v) => [k,v instanceof ESNode ? path.down(k) : v ]);
          //value = Util.filter(value, (v,k) => !(v instanceof Path));
          return [path, value];
        }
      );

      let node2path = new WeakMap();
      let nodeKeys = [];

      for(let [path, node] of flat) {
        node2path.set(node, path);
        nodeKeys.push(path);
      }

      const isRequire = node => node instanceof CallExpression && node.callee.value == 'require';
      const isImport = node => node instanceof ImportStatement;

      //let posMap = new SortedMap([...flat].map(([key, value]) => [value.position ? value.position.pos : -1, value]), (a, b) => a - b);

      let commentMap = new Map([...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [
          pos * 10 - 1,
          { comment, pos, len, node: posMap.keyOf(node) }
        ]),
        (a, b) => a - b
      );

      console.log('commentMap:', commentMap);
      //      posMap = new SortedMap([...posMap, ...commentMap], (a, b) => a - b);
      //    console.log('posMap:', [...posMap.keys()]);
      //   console.log('ast:', [...posMap.keys()]);
      //   let allNodes = nodeKeys.map((path) => flat.get(path));
      let allNodes = nodeKeys.map((path, i) => [i, flat.get(path)]);

      //  for(let [i, n] of allNodes) console.log(`\n  ${i}:\n `, new ImmutablePath(node2path.get(n)), '\n ', n, '\n ', ESNode.assoc(n).position, '\n');
      for(let [i, n] of allNodes) console.log(new ImmutablePath(node2path.get(n)), n);

      const output_file = file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

      console.log('saving to:', output_file);
      const output = printAst(ast, parser.comments, printer);
      console.log('output:', output);

      dumpFile(output_file, output);

      const imports = [...flat].filter(([path, node]) => isRequire(node) || isImport(node));
      const importStatements = imports
        .map(([path, node]) => (isRequire(node) || true ? path.slice(0, 2) : path))
        .map(path => [path, deep.get(ast, path)]);

      console.log('imports:', new Map(imports.map(([path, node]) => [ESNode.assoc(node).position, node])));
      console.log('importStatements:', importStatements);

      const importedFiles = imports.map(([pos, node]) => Identifier.string(node.source || node.arguments[0]));
      console.log('importedFiles:', importedFiles);
      const importIdentifiers = importStatements
        .map(([p, n]) => [p, n.identifiers ? n.identifiers : n])
        .map(([p, n]) => [p, n.declarations ? n.declarations : n])
        .map(([p, n]) =>
          n
            .map(decl => (decl.id instanceof ObjectBindingPattern ? decl.id.properties : [decl.id]))
            .flat()
            .map(n => [n.id ? n.id : n])
            .flat()
            .map(n => Identifier.string(n))
        );
      //  console.log('importIdentifiers:',importIdentifiers.map(ids => ids.join(", ")).join(",\n"));
      console.log('importIdentifiers:', Util.unique(importIdentifiers.flat()).join(', '));
      await ConsoleSetup({ depth: Infinity });
      const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral);

      console.log('templates:', templates);
    } catch(err) {
      error = err;
      console.log('error:', err);
    }
    files[file] = finish(error);

    if(error) process.exit(1);

    console.log('files:', files);
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
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
  dumpFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}
