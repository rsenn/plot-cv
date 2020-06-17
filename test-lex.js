import { ECMAScriptParser } from './lib/ecmascript/parser.js';
import Lexer, { Stack, PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import Util from './lib/util.js';
import fs from 'fs';
import util from 'util';
import { Console } from 'console';
import { estree, Factory, ESNode } from './lib/ecmascript/estree.js';
import deep from './lib/deep.js';
import { SortedMap } from './lib/container/sortedMap.js';

//import process from 'process';
Error.stackTraceLimit = 1000;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 20, colors: true }
});

let args = process.argv.slice(2);
if (args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

main(args);

Error.stackTraceLimit = 100;

global.lexer = null;

function main(args) {
  if (args.length == 0) args.push('./components.js');

  for (let file of args) {
    let data, b, ret;
    if (file == '-') file = '/dev/stdin';
    console.log('file:', file);
    data = fs.readFileSync(file);
    console.log('opened:', data);
    let token, error;

    global.lexer = new Lexer(data.toString(), file);
    try {
      while ((token = lexer.lex())) {
        //      const { type, value, length, start, end } = token;
        //        const position = token.position.start.toString();

        //console.log("position:", position);
        console.log(`Token`, token.toString());
        //console.log(token.position.start.toString());
      }
      //  console.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
    }
    catch (err) {
      error = err;
      const { msg } = error;
      console.log('ERROR:', error);
      console.log('stack:\n' + err.stack);
    }
    files[file] = error;
  }

  //console.log("files:", files);
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}
