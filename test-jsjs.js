import { ECMAScriptParser, ECMAScriptInterpreter } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript.js';
import Printer from './lib/ecmascript/printer.js';
import { CallExpression } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import PortableFileSystem from './lib/filesystem.js';
import { ConsoleSetup } from './lib/consoleSetup.js';

let filesystem;

const code =
  "Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;";

let args = Util.getArgs();
let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

Util.callMain(main, true);

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

globalThis.parser = null;

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: 10 });
  if(args.length == 0) args.push('-');
  for(let file of args) {
    let data, b, ret;
    data = file == '-' ? code : filesystem.readFile(file);
    console.log(`read ${file}:`, Util.abbreviate(data).replaceAll('\n', '\\n'));
    let ast, error;
    globalThis.parser = new ECMAScriptParser(data, file);
    globalThis.printer = new Printer({ indent: 4 });
    /*    globalThis.interpreter = new ECMAScriptInterpreter(util);
    interpreter.util = util;*/
    try {
      ast = parser.parseProgram();
      console.log('ast:', ast);

      //    ret = interpreter.run(ast);
      parser.addCommentsToNodes(ast);
      let imports = [
        ...deep.iterate(ast,
          node => node instanceof CallExpression && /console.log/.test(printer.print(node))
        )
      ].map(([node, path]) => node);
    } catch(err) {
      error = err;
    }
    /*     let output = printer.print(ast);
      console.log('output:', output);*/

    files[file] = finish(error);
    if(!error) {
      const output_file = file.replace(/.*\/?/, '').replace(/\.[^.]*$/, '') + '.es';
      const output = printAst(ast, parser.comments, printer);
      console.log('ret:', ret);
      WriteFile(output_file, output);
    } else {
      const pos = error.pos;
      console.log(pos && pos.toString ? error.pos.toString() : pos);
      Util.putError(error);

      Util.exit(1);
    }
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  Util.exit(Number(files.length == 0));
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
  }
  let lexer = parser.lexer;
  let t = [];
  WriteFile('trace.log', parser.trace());
  if(fail) {
  }
  return !fail;
}
