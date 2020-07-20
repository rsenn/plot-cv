import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, CallExpression } from './lib/ecmascript/estree.js';

import Util from './lib/util.js';
import fs from 'fs';
import { Console } from 'console';
import deep from './lib/deep.js';
import { Path } from './lib/json.js';
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
  inspectOptions: { depth: 2, colors: true }
});

const testfn = () => true;
const testtmpl = `this is\na test`;
let args = process.argv.slice(2);
if(args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

process.on('uncaughtException', (err, origin) => {
  fs.writeSync(process.stderr.fd, `Caught exception: ${err}\nException origin: ${origin}\nStack: ${err.stack}`);
  process.exit();
});

process.on('SIGINT', () => {
  //fs.writeSync(process.stderr.fd, "\nSIGINT - Exit\n");
  //console.log('\nSIGINT - Exit\n');

  finish();
  process.exit(3);
});

process.on('exit', () => {
  //fs.writeSync(process.stderr.fd, "\nexited\n");
  //console.log('\nexited\n');
  process.exit();
});

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

global.parser = null;

function main(args) {
  if(args.length == 0) args.push('./lib/ecmascript/parser.js');
  for(let file of args) {
    let data, b, ret;
    if(file == '-') {
      file = '/dev/stdin';
      data = fs.readFileSync(file);
    }
    //console.log('file:', file);
    //console.log('opened:', data);
    let ast, error;

    global.parser = new ECMAScriptParser(data ? data.toString() : code, file);
    global.printer = new Printer({ indent: 4 });

    //console.log('prototypeChain:', Util.getPrototypeChain(parser));

    //console.log("methodNames:",Util.getMethodNames(parser, 2));

    //console.log(new parser.estree.Identifier());
    try {
      ast = parser.parseProgram();

      parser.addCommentsToNodes(ast);

      let imports = [...deep.iterate(ast, node => node instanceof CallExpression && /console.log/.test(printer.print(node)))].map(([node, path]) => node);

      //console.log('imports:', imports.map(node => ({ str: printer.print(node), toks: ECMAScriptParser.printToks( parser.tokensForNode(node))  })));

      //for(let imp of imports) console.log('tokens:', parser.tokensForNode(imp));
      /*
      let flat = deep.flatten(
        ast,
        new Map(),
        node => node instanceof ESNode,
        (path, value) => {
          path = new Path(path);
          //value = Util.map(value,(k,v) => [k,v instanceof ESNode ? path.down(k) : v ]);
           //value = Util.filter(value, (v,k) => !(v instanceof Path));
          return [path, value];
        }
      );
      //console.log('flat:', flat);

      let posMap = new SortedMap(
        [...flat].map(([key, value]) => [value.position ? value.position.pos : -1, value]),
        (a, b) => a - b
      );

      let commentMap = new SortedMap(
        [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node: posMap.keyOf(node) }]),
        (a, b) => a - b
      );

       //posMap = new SortedMap([...posMap, ...commentMap], (a, b) => a - b);

      //console.log("ast:", [...posMap.keys()]);$
      //console.log('commentMap:', commentMap);
      //console.log('posMap:', [...posMap.keys()]);

      //console.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
//
*/
    } catch(err) {
      error = err;
      //console.log('error:', err);
    }
    files[file] = finish(error);

    if(!error) {
      const output_file = file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';

      //console.log('saving to:', output_file);
      const output = printAst(ast, parser.comments, printer);
      //console.log('output:', output);

      dumpFile(output_file, output);
    } else {
      process.exit(1);
    }

    //console.log('files:', files);
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
    //console.log(parser.lexer.currentLine());
    //console.log(Util.className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
  }

  let lexer = parser.lexer;
  let t = [];
  //console.log(parser.trace() );
  dumpFile('trace.log', parser.trace());
  if(fail) {
    //console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  //console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}
