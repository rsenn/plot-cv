import Lexer from './lib/ecmascript/lexer.js';
let args = scriptArgs;
if(args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

main(args);

globalThis.lexer = null;

function main(args) {
  if(args.length == 0) args.push('./components.js');

  for(let file of args) {
    let data, b, ret;
    if(file == '-') file = '/dev/stdin';
    //console.log('file:', file);
    data = filesystem.readFileSync(file);
    //console.log('opened:', data);
    let token, error;

    globalThis.lexer = new Lexer(data.toString(), file);
    try {
      while((token = lexer.lex())) {
        //const { type, value, length, start, end } = token;
        //const position = token.position.start.toString();
        //console.log("position:", position);
        //console.log(`Token`, token.toString());
        //console.log(token.position.start.toString());
      }
      //console.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
    } catch(err) {
      error = err;
      const { msg } = error;
      //console.log('ERROR:', error);
      //console.log('stack:\n' + err.stack);
    }
    files[file] = error;
  }

  //console.log("files:", files);
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}