import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { estree, ESNode, TemplateLiteral, CallExpression, ImportStatement, Identifier, ObjectBindingPattern } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import deep from './lib/deep.js';
import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';
import { ImmutablePath } from './lib/json.js';
import { ConsoleSetup } from './consoleSetup.js';

let filesystem;

const code = `export const Progress = ({ className, percent, ...props }) =>  h(Overlay,
    {
      className: classNames('progress', 'center', className),
      text: percent + '%',
      style: {
        position: 'relative',
        width: '100%',
        height: '1.5em',
        border: '1px solid black',
        textAlign: 'center',
        zIndex: '99'
      }
    },
    h('div', {
      className: classNames('progress-bar', 'fill'),
      style: {
        width: percent + '%',
        position: 'absolute',
        left: '0px',
        top: '0px',
        zIndex: '98'
      }
    })
  );`;

const testfn = () => true;
const testtmpl = `this is\na test`;

Util.callMain(main, true);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
function dumpFile(name, data) {
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
  await ConsoleSetup({ breakLength, maxStringLength: breakLength, depth: Infinity });
  console.log('breakLength:', breakLength);

  filesystem = await PortableFileSystem();

  if(args.length == 0) args.push('./lib/ecmascript/parser.js');
  for(let file of args) {
    let data, b, ret;
    if(file == '-') file = '/dev/stdin';

    if(filesystem.exists(file)) data = filesystem.readFile(file);

    console.log(`opened '${file}':`, Util.abbreviate(data));
    let ast, error;

    globalThis.parser = new ECMAScriptParser(data ? data.toString() : code, file, false);
    globalThis.printer = new Printer({ indent: 4 });
    console.log('OK');

    try {
      ast = parser.parseProgram();

      parser.addCommentsToNodes(ast);

      let flat = deep.flatten(ast,
        new Map(),
        (node) => node instanceof ESNode,
        (path, value) => {
          path = new Path(path);
          return [path, value];
        }
      );

      let node2path = new WeakMap();
      let nodeKeys = [];

      for(let [path, node] of flat) {
        node2path.set(node, path);
        nodeKeys.push(path);
      }
      let commentMap = new Map([...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node: posMap.keyOf(node) }]),
        (a, b) => a - b
      );

      console.log('commentMap:', commentMap);

      const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral && path[path.length - 1] == 'arguments');

      console.log('templates:', templates);

      const output_file = file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';
      const output = printAst(ast, parser.comments, printer);
      //console.log('output:', output);

      dumpFile(output_file, output);
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
      .filter((s) => !/esfactory/.test(s))
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
