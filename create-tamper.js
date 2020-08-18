import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { ImportStatement, ExportStatement, VariableDeclaration } from './lib/ecmascript/estree.js';
import { estree, ESNode, CallExpression, Literal } from './lib/ecmascript/estree.js';

import Util from './lib/util.js';
import fs from 'fs';
import path from 'path';
import { Console } from 'console';
import deep from './lib/deep.js';
//import { Path } from './lib/json.js';
import { SortedMap } from './lib/container/sortedMap.js';
//prettier-ignore

const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filename) {return fs.realpathSync(filename); },
  stat(filename) {return fs.statSync(filename); }
};
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

let cwd = process.cwd();

let searchPath = [];
let packagesPath = [];
let moduleAliases;
let packageFiles;
let importFiles;
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

class ES6Import {
  constructor(ast) {
    this.node = ast;
    this.position = ESNode.assoc(this.node).position;
  }

  get from() {
    let value = this.node.source;
    while(Util.isObject(value, v => v.value)) value = value.value;
    return value;
  }
  set from(value) {
    console.log('from value: ', value);
    /*  while(Util.isObject(value, v => v.value))
      value = value.value;
*/
    this.node.source = value instanceof Literal ? value : new estree.Literal(`'${value}'`);
  }
  toSource() {
    return printAst(this.node);
  }
  get code() {
    return this.toSource();
  }
}

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
  const cwd = process.cwd() || fs.realpath('.');
  console.info('cwd=', cwd);

  if(args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];

  const dirs = [cwd, ...args.map(arg => path.dirname(arg))].map(p => path.resolve(p));

  searchPath = makeSearchPath(dirs);
  console.info('searchPath=', searchPath);
  packagesPath = makeSearchPath(dirs, 'package.json');
  console.info('packagesPath=', packagesPath);
  moduleAliases = packagesPath.reduce((acc, p) => {
    let json = JSON.parse(filesystem.readFile(p));
    let aliases = json._moduleAliases || {};
    for(let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      if(!filesystem.exists(module)) throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      let file = findModule(module);
      let st = filesystem.stat(file);
      acc.set(alias, file);
    }
    return acc;
  }, new Map());
  console.info('moduleAliases=', moduleAliases);

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
    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
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
      function removeStatements(statements, predicate = stmt => true) {
        if(Util.isArray(statements)) statements = new Map(statements);

        console.log(
          'removeStatements:',
          [...statements].map(([path, stmt]) => printAst(stmt))
        );
        console.log(
          'removeStatements:',
          [...statements].map(([path, stmt]) => stmt)
        );
        let removed = [];
        //       statements = [...statements].map(([p, v]) => [[...p], v]);
        for(let [path, node] of statements) {
          if(!predicate(node, path)) continue;
          if(node instanceof ImportStatement || node.what == 'default') {
            deep.unset(ast, path);
          } else {
            console.log('i:', deep.get(ast, path.slice(0, -2)));
            if(!Util.isArray(node.declarations)) node = node.declarations;
            else Object.setPrototypeOf(node, VariableDeclaration.prototype);
            deep.set(ast, path, node);
          }
          removed.push(node);
        }
        return removed;
        //console.log('i:', stmt.map(([path, node]) => deep.get(ast, [...path])) );
      }
      const getBase = filename => filename.replace(/\.[a-z0-9]*$/, '');
      const getRelative = filename => path.join(thisdir, filename);
      const getFile = Util.memoize(module => searchModuleInPath(module, file));

      let imports,
        importStatements = [...flat.entries()].filter(([key, node]) => node instanceof ImportStatement);
      //   console.log(`${file}: importStatements =`, importStatements);
      imports = importStatements.map(([path, node], i) => {
        const getFromValue = Util.memoize(() => node.source.value.replace(/['"](.*)['"]/, '$1'));
        const getFromBase = () => getBase(getFromValue());
        const getFromPath = () => getFile(getFromBase());
        const getAssoc = Util.memoize(() => ESNode.assoc(node));
        return Object.assign(Object.setPrototypeOf({}, ES6Import.prototype), {
          node,
          path: path.join('.'),
          file,
          position: getAssoc().position,
          fromPath: getFromPath(),
          fromBase: getFromBase(),
          fromValue: getFromValue()
          //    variables: Util.isObject(node.identifiers, () => node.identifiers.variables) ? node.identifiers : node
        });
      });

      let statement2module = new WeakMap(imports.map(({ node, ...module }) => [node, module]));
      // console.log(`${file}: statement2module =`, statement2module);
      //console.log(`${file}: imports =`, imports);
      let alter = imports.filter(({ fromPath, ...module }) => /^lib/.test(fromPath));
      alter = alter.map(node => {
        /*   const { from, file } = node;*/
        const to = node.fromPath;
        const from = node.fromValue;
        node.from = new estree.Literal(`'${to}'`);
        console.log(`node alter ${node.position.toString()}  => '${to}'   (was '${from}' )`, node);
        return node;
      });
      console.log(
        `${file}: imports =`,
        imports.map(imp => imp.toSource())
      );
      console.log(
        `${file}: alter =`,
        alter.map(({ node }) => printAst(node))
      );
      let remove = imports.filter(({ fromPath, ...module }) => !/^lib/.test(fromPath));
      console.log(`${file}: remove =\n`, ...remove.map(module => ['\n' + module.position, module.toSource()]).flat());
      removeStatements(remove.map(({ path, node }) => [path, node]));
      let recurseFiles = remove.map(module => module.file).filter(file => processed.indexOf(file) == -1);
      console.log(`${file}: recurseFiles =`, recurseFiles);
      removeFile(file);
      recurseFiles.forEach(file => processFile(file));
      let exports = [...flat.entries()].filter(([key, value]) => value instanceof ExportStatement);
      console.log(
        'exports:',
        exports.map(([p, stmt]) => (Util.isObject(stmt.declarations, 'id', id => id.value) == stmt.what.value ? stmt.declarations : stmt))
      );
      //  removeStatements(exports);
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer);
    r.push(`/** concatenanted '${file}' ***/\n${output}\n`);
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

function makeSearchPath(dirs, extra = 'node_modules') {
  let r = [];
  const addPath = p => {
    p = path.relative(cwd, p);
    if(r.indexOf(p) == -1) r.push(p);
  };
  let i = 0;

  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    console.info('parts=', parts);

    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      const extra_dir = path.join(dir, extra);
      if(extra == 'node_modules') if (i == 0) addPath(dir);
      if(filesystem.exists(extra_dir)) addPath(extra_dir);
      i++;
      parts.pop();
    }
    if(extra == 'node_modules') i = 0;
  }
  return r;
}

function checkExists(path) {
  let r = filesystem.exists(path);
  console.info(`checkExists('${path}') = ${r}`);
  return r;
}

function findModule(relpath) {
  let st = filesystem.stat(relpath);
  let module;
  if(st.isDirectory()) {
    const name = path.basename(relpath);
    let indexes = [...makeNames(relpath + '/dist/' + name), ...makeNames(relpath + '/dist/index'), ...makeNames(relpath + '/build/' + name), ...makeNames(relpath + '/' + name), ...makeNames(relpath + '/index')];

    module = indexes.find(i => checkExists(i));
  } else if(st.isFile()) {
    module = relpath;
  }
  if(!module) throw new Error(`Module '${relpath}' not found`);

  return module;
}

function searchModuleInPath(name, _from) {
  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
  console.log('thisdir:', thisdir);
  console.log('name:', name);
  console.log('_from:', _from);

  name = name.replace(/\..?js$/g, '');
  if(moduleAliases.has(name)) return moduleAliases.get(name);

  let names = makeNames(name);
  let indexes = [...makeNames(name + '/dist/' + name), ...makeNames(name + '/build/' + name), ...makeNames(name + '/' + name), ...makeNames(name + '/index')];

  for(let dir of [thisdir, ...searchPath]) {
    let searchFor = dir.endsWith('node_modules') ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      if(filesystem.exists(modPath)) {
        console.info('modPath', modPath);
        let path = findModule(modPath);
        console.info('path', path);
        if(path) return path;
      }
    }
  }
  throw new Error(`Module '${name}' imported from '${_from}' not found`);
}

function makeNames(prefix) {
  return [prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js'];
}
