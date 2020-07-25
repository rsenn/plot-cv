import Lexer from './lib/ecmascript/lexer.js';
import Util from './lib/util.js';
import fs from 'fs';
import { Console } from 'console';

//import process from 'process';
Error.stackTraceLimit = 1000;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 20, colors: true }
});

let args = process.argv.slice(2);
if(args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

main(args);

Error.stackTraceLimit = 100;

global.lexer = null;

function main(args) {
  if(args.length == 0) args.push('./components.js');

  for(let file of args) {
    let data, b, ret;
    if(file == '-') file = '/dev/stdin';
    //Util.log('file:', file);
    data = fs.readFileSync(file);
    //Util.log('opened:', data);
    let token, error;

    global.lexer = new Lexer(data.toString(), file);
    try {
      while((token = lexer.lex())) {
        //const { type, value, length, start, end } = token;
        //const position = token.position.start.toString();
        //Util.log("position:", position);
        //Util.log(`Token`, token.toString());
        //Util.log(token.position.start.toString());
      }
      //Util.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
    } catch(err) {
      error = err;
      const { msg } = error;
      //Util.log('ERROR:', error);
      //Util.log('stack:\n' + err.stack);
    }
    files[file] = error;
  }

  //Util.log("files:", files);
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}
