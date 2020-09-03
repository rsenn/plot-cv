import { ECMAScriptParser, ECMAScriptInterpreter } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { CallExpression } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';

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

let args = Util.getArgs();
let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

main(args);

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

globalThis.parser = null;

function main(args) {
  if(args.length == 0) args.push('./lib/ecmascript/parser.js');
  for(let file of args) {
    let data, b, ret;
    data = filesystem.readFile(file);
    console.log(`read ${file}:`, Util.abbreviate(data).replace(/\n/g, '\\n'));
    let ast, error;
    globalThis.parser = new ECMAScriptParser(data ? data.toString() : code, file);
    globalThis.printer = new Printer({ indent: 4 });
    globalThis.interpreter = new ECMAScriptInterpreter(util);
    interpreter.util = util;
    try {
      ast = parser.parseProgram();
      ret = interpreter.run(ast);
      parser.addCommentsToNodes(ast);
      let imports = [...deep.iterate(ast, (node) => node instanceof CallExpression && /console.log/.test(printer.print(node)))].map(([node, path]) => node);
    } catch(err) {
      error = err;
    }
    files[file] = finish(error);
    if(!error) {
      const output_file = file.replace(/.*\/?/, '').replace(/\.[^.]*$/, '') + '.es';
      const output = printAst(ast, parser.comments, printer);
      console.log('ret:', ret);
      dumpFile(output_file, output);
    } else {
      const pos = error.pos;
      console.log(pos && pos.toString ? error.pos.toString() : pos);
      Util.putError(error);

      process.exit(1);
    }
  }
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}
function finish(err) {
  let fail = !!err;
  if(fail) {
    err.stack = PathReplacer()('' + err.stack)
      .split(/\n/g)
      .filter((s) => !/esfactory/.test(s))
      .join('\n');
  }
  if(err) {
  }
  let lexer = parser.lexer;
  let t = [];
  dumpFile('trace.log', parser.trace());
  if(fail) {
  }
  return !fail;
}
