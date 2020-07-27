import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { ImportStatement, ExportStatement, VariableDeclaration } from './lib/ecmascript/estree.js';
import { estree, ESNode, CallExpression } from './lib/ecmascript/estree.js';

import Util from './lib/util.js';
import fs from 'fs';
import path from 'path';
import { Console } from 'console';
import deep from './lib/deep.js';
//import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';

const code = `export const Progress = ({ className, percent, ...props }) => html\`<\x24{Overlay} className=\x24{classNames('progress', 'center', className)} text=\x24{percent + '%'} style=\x24{{
  position: 'relative',
  width: '100%',
  height: '1.5em',
  border: '1px solid black',
  textAlign: 'center',
  zIndex: '99'
}}><div className=\x24{classNames('progress-bar', 'fill')} style=\x24{{
  width: percent + '%',
  position: 'absolute',
  left: '0px',
  top: '0px',
  zIndex: '98'
}}></div></\x24{Overlay}>\`"`;

import process from 'process';

Error.stackTraceLimit = 1000;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 3, colors: true }
});

let args = process.argv.slice(2);
//[if(args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});
/*
process.on('uncaughtException', (err, origin) => {
  fs.writeSync(process.stderr.fd, `Caught exception: ${err}\nException origin: ${origin}\nStack: ${err.stack}`);
  process.exit();
});

process.on('SIGINT', () => {
  fs.writeSync(process.stderr.fd, "\nSIGINT - Exit\n");
  console.log('\nSIGINT - Exit\n');

  finish();
  process.exit(3);
});

process.on('exit', () => {
  fs.writeSync(process.stderr.fd, "\nexited\n");
  console.log('\nexited\n');
  process.exit();
});*/

main(args);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  fs.writeFileSync(name, data + '\n');

  //console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

Error.stackTraceLimit = 100;

function main(args) {
  if(args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];
  while(args.length > 0) {
    processFile(args.shift());
  }
  // console.log("result:",r);
  fs.writeFileSync('new.js', r.join('\n'));

  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));

  function removeFile(file) {
    let idx = args.indexOf(file);
    if(idx != -1) args.splice(idx, idx + 1);

    processed.push(file);
  }

  function processFile(file) {
    let data, b, ret;
    /* if(file == '-') {
      file = '/dev/stdin';
    }*/
    console.log('processing:', file);

    removeFile(file);

    data = fs.readFileSync(file).toString();
    //  console.log('opened:', data);
    let ast, error;

    let parser = new ECMAScriptParser(data ? data.toString() : code, file);
    let printer = new Printer({ indent: 4 });

    //console.log('prototypeChain:', Util.getPrototypeChain(parser));

    //console.log("methodNames:",Util.getMethodNames(parser, 2));

    //console.log(new parser.estree.Identifier());
    try {
      ast = parser.parseProgram();

      parser.addCommentsToNodes(ast);

      /*   let imports = [...deep.iterate(ast, node => node instanceof CallExpression && /console.log/.test(printer.print(node)))].map(([node, path]) => node);

*/
      //for(let imp of imports) console.log('tokens:', parser.tokensForNode(imp));

      let flat = deep.flatten(
        ast,
        new Map(),
        node => node instanceof ESNode,
        (path, value) => [path, value]
      );
      //console.log('flat:', flat);

      const KillStatements = stmt => {
        stmt = stmt.map(([p, v]) => [[...p], v]);

        stmt.forEach(([path, node]) => {
          if(node instanceof ImportStatement || node.what == 'default') {
            deep.unset(ast, path);
          } else {
            console.log('i:', deep.get(ast, path.slice(0, -2)));
            if(!Util.isArray(node.declarations)) node = node.declarations;
            else Object.setPrototypeOf(node, VariableDeclaration.prototype);
            deep.set(ast, path, node);
          }
        });

        //console.log('i:', stmt.map(([path, node]) => deep.get(ast, [...path])) );
      };

      let imports = [...flat.entries()].filter(([key, node]) => node instanceof ImportStatement);

      let importFiles = Util.unique(imports.map(([key, node]) => node.source.value.replace(/'(.*)'/, '$1')));

      importFiles = importFiles.map(imp => path.join(path.dirname(file), imp));
      importFiles = importFiles.filter(file => processed.indexOf(file) == -1);

      console.log('imports:', importFiles + '');

      importFiles.forEach(processFile);

      console.log('file:', file);

      KillStatements(imports);
      let exports = [...flat.entries()].filter(([key, value]) => value instanceof ExportStatement);
      console.log('exports:', exports);
      KillStatements(exports);
    } catch(err) {
      console.error(err);
      console.error(err && err.pos && err.pos.toString());
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer);
    r.push(output + '\n');
  }
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
  //console.log(parser.trace() );
  dumpFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}
