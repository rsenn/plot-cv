import { ECMAScriptParser, Printer, PathReplacer } from './lib/ecmascript.js';
import { ObjectBindingPattern, ObjectLiteral, ImportStatement, ExportStatement, VariableDeclaration, estree, ESNode, Literal } from './lib/ecmascript.js';
import ConsoleSetup from './consoleSetup.js';
import Util from './lib/util.js';
import { ImmutablePath } from './lib/json.js';
import deep from './lib/deep.js';
import path from './lib/path.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';

let filesystem;

let cwd;

let exportMap = new Map();
let allExports = [];
const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);

console.log('main');
Util.callMain(main);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg"> <defs /> */ function PrefixRemover(reOrStr, replacement = '') {
  if(!(Util.isArray(reOrStr) || Util.isIterable(reOrStr))) reOrStr = [reOrStr];

  return arg => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), arg);
}

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: 5 });

  cwd = filesystem.realpath('.');

  // cwd = process.cwd() || fs.realpath('.');
  console.log('cwd=', cwd);

  if(args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];
  console.log('args=', args);
  console.log('test', path.dirname('/usr/bin/ls'));
  console.log('test', path.resolve('/proc/self/../ls'));
  const argDirs = [...args].map(arg => path.dirname(arg));
  // console.log('argDirs',Util.toString(argDirs));

  const dirs = [cwd].concat(argDirs);
  /*.map(p => path.resolve(p))*/ console.log('dirs=', dirs);

  console.log('join()', path.join(cwd, argDirs[0]));
  console.log('cwd', cwd);
  console.log('argDirs[0]', argDirs[0]);
  ///path.cwd = cwd;

  try {
    console.log('relative()', path.relative(cwd, argDirs[0], cwd));
    console.log('relative()', path.relative(argDirs[0], cwd, cwd));
  } catch(err) {
    console.log(err);
  }
  const name = args.join(', ');
  while(args.length > 0) processFile(args.shift());
  // console.log("result:",r);

  for(let ids of exportMap.values()) r.push(`Util.weakAssign(globalObj, { ${Util.unique(ids).join(', ')} });`);

  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;

  function removeFile(file) {
    let idx = args.indexOf(file);
    if(idx != -1) args.splice(idx, idx + 1);
    if(processed.indexOf(file) != -1) return false;
    processed.push(file);
    return true;
  }

  function processFile(file) {
    let data, b, ret;

    if(!removeFile(file)) return;
    const modulePath = removeModulesDir(file);
    console.log('processing:', modulePath);

    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    data = filesystem.readFile(file).toString();
    let ast, error;
    let parser = new ECMAScriptParser(data ? data.toString() : code, file);
    let printer = new Printer({ indent: 4 });
    try {
      ast = parser.parseProgram();
      parser.addCommentsToNodes(ast);
      let flat = deep.flatten(ast,
        new Map(),
        node => node instanceof ESNode,
        (path, value) => [path, value]
      );

      let exports = [...flat.entries()].filter(([key, value]) => value instanceof ExportStatement || value.exported === true);

      for(let [path, node] of exports) {
        log(`export ${path}`, node);
        deep.set(ast, path, node.declarations[0]);
      }

      allExports.splice(allExports.length, 0, ...exports.map(([key, value]) => [ESNode.assoc(value).position.toString(), value]));

      exports = exports.map(([p, stmt]) => (Util.isObject(stmt.declarations, 'id', 'value') == Util.isObject(stmt.what, 'value') ? stmt.declarations : stmt));
      exports = exports.map(decl => (decl instanceof ObjectBindingPattern ? decl.properties.map(prop => ('id' in prop ? prop.id : prop)) : decl instanceof ObjectLiteral ? decl.members.map(prop => ('id' in prop ? prop.id : prop)) : decl));
      exports = exports.map(decl => (Util.isObject(decl) && 'id' in decl ? decl.id : decl));
      exports = exports.flat().map(e => e.value);
      log(`exports =`, exports.join(', '));

      exportMap.set(modulePath, Util.unique(exports));
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer).trim();
    if(output != '') r = r.concat(`/* --- concatenanted '${file}' --- */\n${output}\n`.split(/\n/g));

    function log(...args) {
      const assoc = args.map(arg => arg instanceof ESNode && ESNode.assoc(arg)).filter(assoc => !!assoc);
      //if(assoc[0]) console.log('ASSOC:', assoc[0].position.clone());
      const prefix = (assoc.length == 1 && assoc[0].position.clone()) || modulePath;
      console.log(prefix.toString() + ':', ...args);
    }
  }

  console.log('processed:', ...processed);
  console.log('exports:', allExports);
  console.log('exportMap:', exportMap);
  let output = '';
  for(let [source, list] of exportMap) {
    output += `export { ${list.join(', ')} } from './${source}';\n`;
  }
  console.log(output);
  process.exit(Number(processed.length == 0));
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
