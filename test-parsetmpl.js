import deep from './lib/deep.js';
import { ECMAScriptParser, PathReplacer } from './lib/ecmascript.js';
import { ESNode, Literal, TemplateLiteral } from './lib/ecmascript/estree.js';
import Printer from './lib/ecmascript/printer.js';
import { Path } from './lib/json.js';

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

main(...scriptArgs.slice(1));

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

function transformTagged(node) {
  const {
    arguments: { parts }
  } = node;
  let j = 0;

  return transformParts(parts);

  function transformParts(parts) {
    let inTag = false;
    let inProps = false;
    let closing = false;
    let propName;
    let value;
    let inValue = false;
    let tagName;
    let a = [];
    let r = [];
    let children = [];
    const len = parts.length;

    function nextProp() {
      if(propName) {
        a.push([propName, value]);
        value = [];
        propName = '';
      }
    }

    function nextTag() {
      if(propName) nextProp();

      if(children) {
        children = children.filter(part => typeof part != 'string' || !isSpace(part));

        if(children.length) r.push(children);

        children = [];
      }

      if(a.length) {
        if(!Array.isArray(a[0])) {
          let tag = a.shift();
          let t = a.length == 0 ? { tag } : { tag, attrs: a };
          if(closing) t.closing = closing;
          r.push(t);
        } else if(a.length) {
          r.push(a);
        }
        a = [];
      }
    }
    function concat(a, str) {
      if(a.length == 0 || typeof a[a.length - 1] != typeof str) a.push(str);
      else a[a.length - 1] += str;
    }

    function isSpace(char) {
      return /^[\ \t\s\r\n]*$/.test(char);
    }

    for(; j < len; j++) {
      const arg = parts[j];
      const str = arg instanceof Literal ? Literal.string(arg) : null;

      if(arg instanceof Literal)
        for(let i = 0; i < str.length; i++) {
          if(!inTag && str[i] == '<') {
            inTag = true;
            tagName = '';
            if(str[i + 1] == '/') {
              closing = true;
              i++;
            } else {
              closing = false;
            }
            nextTag();
            continue;
          } else if(!inTag) {
            concat(children, str[i]);
            continue;
          } else if(inTag && str[i] == '>') {
            //  console.log("rest:", str.substring(i, str.length));
            if(tagName) {
              a.push(tagName);
              tagName = '';
            }
            nextTag();
            closing = inValue = inTag = inProps = false;
            continue;
          } else if(inTag && !inProps && !isSpace(str[i])) {
            tagName += str[i];
          } else if(inTag && !inProps && isSpace(str[i])) {
            inProps = !closing;
            inValue = false;
            propName = '';
            value = [];
            if(tagName) {
              a.push(tagName);
              tagName = '';
            }
            nextProp();
            continue;
          } else if(inProps && str[i] == '=') {
            if('\'`"'.indexOf(str[i + 1]) != -1) value = [(inValue = str[++i])];
            else inValue = ' \r\n\t';
            continue;
          } else if(inProps && propName == '' && str[i] == '/') {
            closing = true;
            continue;
          } else if(inProps && !inValue && !isSpace(str[i])) {
            propName += str[i];
            if(propName == '...') inValue = ' \r\n\t';

            continue;
          } else if(inProps && inValue && inValue.indexOf(str[i]) == -1) {
            concat(value, str[i]);
            continue;
          } else if(inValue && inValue.indexOf(str[i]) != -1) {
            //   console.log("",{ propName, inValue, value });
            if('\'`"'.indexOf(str[i]) != -1) {
              concat(value, str[i]);
            }

            inValue = false;
            nextProp();
            continue;
          }
        }
      else {
        if(!inTag) {
          concat(children, arg);
        }
        if(inTag && !inProps) {
          tagName = arg;
        } else if(inProps && inValue) {
          concat(value, arg);
        }
      }
      console.log('', arg);
    }
    nextTag();
    //console.log('', { r, a, j, len });

    return r;
  }
}

globalThis.parser = null;
let files = {};

async function main(...args) {
  if(args.length == 0) args.push('-');
  //await import('tty');
  const stdout = (await import('process')).stdout;

  const breakLength = stdout.columns || process.env.COLUMNS || 80;
  console.log('breakLength:', breakLength);

  if(args.length == 0) args.push('./lib/ecmascript/parser.js');
  for(let file of args) {
    let data, b, ret;
    if(file == '-') file = '/dev/stdin';

    if(filesystem.exists(file)) data = filesystem.readFileSync(file);

    console.log(`opened '${file}':`, Util.abbreviate(data));
    let ast, error;

    globalThis.parser = new ECMAScriptParser(data ? data.toString() : code, file, false);
    globalThis.printer = new Printer({ indent: 4 });
    console.log('OK');

    try {
      ast = parser.parseProgram();

      parser.addCommentsToNodes(ast);

      let flat = deep.flatten(
        ast,
        new Map(),
        node => node instanceof ESNode,
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
      let commentMap = new Map(
        [...parser.comments].map(({ comment, text, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node: posMap.keyOf(node) }]),
        (a, b) => a - b
      );

      console.log('commentMap:', commentMap);

      const templates = [...flat].filter(([path, node]) => node instanceof TemplateLiteral);
      const taggedTemplates = templates.filter(([path, node]) => path[path.length - 1] == 'arguments');
      const taggedCalls = taggedTemplates.map(([path, node]) => [path.up(), deep.get(ast, path.up())]);

      console.log('taggedCalls:', taggedCalls);
      console.log(
        'transformTagged:',
        taggedCalls.map(([path, node]) => transformTagged(node))
      );

      const output_file = file.replace(/.*\//, '').replace(/\.[^.]*$/, '') + '.es';
      const output = printAst(ast, parser.comments, printer);
      //console.log('output:', output);

      WriteFile(output_file, output);
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
  WriteFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}