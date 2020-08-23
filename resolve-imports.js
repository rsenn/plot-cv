import { ECMAScriptParser } from './lib/ecmascript.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { ImportStatement, ExportStatement, VariableDeclaration, MemberExpression, estree, ESNode, CallExpression, Literal } from './lib/ecmascript/estree.js';

import Util from './lib/util.js';
import fs from 'fs';
import path from 'path';
import { Console } from 'console';
//import { Path } from './lib/json.js';
import { ImmutablePath } from './lib/json.js';
import deep from './lib/deep.js';
import { SortedMap } from './lib/container/sortedMap.js';
//prettier-ignore

const filesystem = {
  readFile(filename) {let data = fs.readFileSync(filename).toString(); return data; },
  writeFile(filename, data, overwrite = true) {return fs.writeFileSync(filename, data, { flag: overwrite ? 'w' : 'wx' }); },
  exists(filename) {return fs.existsSync(filename); },
  realpath(filenamee) {return fs.realpathSync(filename); },
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
let moduleList = new SortedMap();

let args = process.argv.slice(2);
//[if(args.length == 0) args.push('-');

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

process.on('uncaughtException', (err, origin) => {
  fs.writeSync(process.stderr.fd, `Caught exception: ${err}\nException origin: ${origin}\nStack: ${err.stack}`);
  process.exit();
});

process.on('SIGINT', () => {
  fs.writeSync(process.stderr.fd, '\nSIGINT - Exit\n');
  console.log('\nSIGINT - Exit\n');

  finish();
  process.exit(3);
});

process.on('exit', () =>
  Util.once(() => {
    fs.writeSync(process.stderr.fd, '\nexited\n');
    console.log('\nexited\n');
    process.exit();
  })
);

class ES6Module {
  impExpList = [];

  static create(file) {
    let mod = new ES6Module();
    mod.file = file;
    return mod;
  }

  toString() {
    return this.file;
  }
  valueOf() {
    return this.toString();
  }
}

class ES6ImportExport {
  static create(obj) {
    let ret = new ES6ImportExport();
    let nodeClass = Util.className(obj.node);
    let type = Util.decamelize(nodeClass).split('-')[0];
    let position = ESNode.assoc(obj.node).position;
    ret = Util.define(ret, { position, nodeClass, type });
    /* this.node = node;*/
    console.log('ES6ImportExport; obj:', ret);
    // if(!new.target) return Object.setPrototypeOf(ret, ES6ImportExport.prototype);
    return Object.assign(ret, obj);
  }

  get from() {
    let value = this.node.source || c;
    while(Util.isObject(value, (v) => v.value)) value = value.value;
    return value;
  }
  set from(value) {
    console.log('from value: ', value);
    value = value instanceof Literal ? value : new estree.Literal(`'${value}'`);
    if(this.node.source) this.node.source = value;
    else this.node.arguments = [value];
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

function PrefixRemover(reOrStr, replacement = '') {
  if(!(Util.isArray(reOrStr) || Util.isIterable(reOrStr))) reOrStr = [reOrStr];

  return (arg) => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), arg);
}

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

  const dirs = [cwd, ...args.map((arg) => path.dirname(arg))].map((p) => path.resolve(p));

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
  while(args.length > 0) processFile(args.shift());
  // console.log("result:",r);
  fs.writeFileSync('new.js', r.join('\n'));
  let success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));

  function removeFile(file) {
    Util.remove(args, file);
    Util.pushUnique(processed, file);
  }

  function processFile(file) {
    let b, ret;
    console.log('processing:', file);
    removeFile(file);
    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);

    let { data, error, ast, parser, printer, flat, map } = parseFile(file);
    console.log('flat:', flat());

    try {
      const getBase = (filename) => filename.replace(/\.[a-z0-9]*$/, '');
      const getRelative = (filename) => path.join(thisdir, filename);
      const getFile = Util.memoize((module) => searchModuleInPath(module, file));
      let imports;
      const useStrict = map().filter(([key, node]) => node instanceof Literal && /use strict/.test(node.value));
      console.log('useStrict:', useStrict);

      useStrict.map(([path, node]) => deep.unset(ast, path));

      const requireStatements = map()
        .filter(([key, node]) => node instanceof CallExpression && node.callee.value == 'require')
        .map(([path, node]) => [path.slice(0, 2), deep.get(ast, path)]);
      console.log('requireStatements:', requireStatements);

      const importStatements = map().filter(([key, node]) => node instanceof ImportStatement);

      const getFromValue = Util.memoize((node) => (node.source || node.arguments[0]).value.replace(/['"](.*)['"]/, '$1'));
      const getFromBase = (node) => getBase(getFromValue(node)).replace(/^\.\//, '');
      const getFromPath = (node) => getFile(getFromBase(node));

      imports = [...requireStatements, ...importStatements].sort(([a], [b]) => a.join('.').localeCompare(b.join('.')));
      console.log('imports:', imports);

      imports = imports.map(([path, node], i) =>
        ES6ImportExport.create({
          node,
          path: path.join('.'),
          file,
          position: ESNode.assoc(node).position,
          fromPath: getFromPath(node),
          fromBase: getFromBase(node),
          fromValue: getFromValue(node)
          //    variables: Util.isObject(node.identifiers, () => node.identifiers.variables) ? node.identifiers : node
        })
      );
      let statement2module = imports.map((imp) => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ fromPath, ...module }) => /^lib/.test(fromPath));
      alter = alter.map((node) => {
        const to = node.fromPath;
        const from = node.fromValue;
        node.from = new estree.Literal(`'${to}'`);
        console.log(`node alter ${node.position.toString()}  => '${to}'   (was '${from}' )`, node);
        return node;
      });

      const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);
      const modulePath = removeModulesDir(file);

      /* prettier-ignore */ console.log(`${modulePath}: imports =`, imports.map(imp => [(imp.fromPath),imp.toSource()]));
      /* prettier-ignore */ console.log(`${modulePath}: alter =`, alter.map(imp => printAst(imp.node)));
      let remove = imports.map((imp, idx) => [idx, imp.node]).filter((imp, idx) => !/^lib/.test(imp.fromPath));
      /* prettier-ignore */ console.log(`${modulePath}: remove =`, remove .reduce((acc,imp) => [...acc,imp/*(imp.fromPath),imp.toSource()*/], []));

      removeStatements(
        ast,
        remove.map(([idx, node]) => [imports[idx].path, node])
      );

      let recurseFiles = remove.map(([idx, node]) => imports[idx]).filter((imp) => processed.indexOf(removeModulesDir(imp.fromPath)) == -1);
      //recurseFiles = recurseFiles.map(([path,module]) => { console.log("module:",module.fromPath); return module.fromPath; });
      removeFile(modulePath);

      console.info('processed files:', ...processed);
      console.log(`${modulePath}: recurseFiles =`, recurseFiles);
      recurseFiles.forEach((imp) => {
        console.info('imp', imp);
        processFile(imp.fromPath);
      });
      let exports = map().filter(([key, value]) => value instanceof ExportStatement);
      let moduleExports = map()
        .filter(([key, value]) => value instanceof MemberExpression && value.object.value == 'module' && value.property.value == 'exports')
        .map(([path, node]) => path.slice(0, 2))
        .map((p) => [p, deep.get(ast, p)]);

      console.log(`${modulePath}: moduleExports:`, moduleExports);

      console.log(`${modulePath}: exports:`, ...exports.map(([p, stmt]) => (Util.isObject(stmt.declarations, 'id', 'value') == Util.isObject(stmt.what, 'value') ? stmt.declarations : stmt)));
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer);
    r.push(`/* --- concatenanted '${file}' --- */\n${output}\n`);
  }

  console.info('processed files:', ...processed);
}

function parseFile(file) {
  let data, error, ast, parser, printer, flat;
  try {
    data = fs.readFileSync(file).toString();
    parser = new ECMAScriptParser(data ? data.toString() : code, file);
    printer = new Printer({ indent: 4 });
    ast = parser.parseProgram();
    parser.addCommentsToNodes(ast);
  } catch(err) {
    error = err;
  } finally {
    flat = Util.memoize(() =>
      deep.flatten(
        ast,
        new Map(),
        (node) => node instanceof ESNode,
        (path, value) => [new ImmutablePath(path), value]
      )
    );
  }
  return {
    data,
    error,
    ast,
    parser,
    printer,
    flat,
    map() {
      let map = flat();

      /*    return Util.mapWrapper(
        flat,
        (k) => k.join('.'),
        (k) => k.split('.')
      );
*/
      return Util.mapAdapter((key, value) =>
        key !== undefined
          ? value !== undefined
            ? value === null
              ? deep.unset(ast, key)
              : deep.set(ast, key, value)
            : deep.get(ast, key)
          : (function* () {
              for(let [key, value] of map) yield [[...key], key.apply(ast, true)];
            })()
      );
    }
  };
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
  const addPath = (p) => ((p = path.relative(cwd, p)), r.indexOf(p) == -1 && r.push(p));
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    //console.info('parts=', parts);
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
  //console.info(`checkExists('${path}') = ${r}`);
  return r;
}

function findModule(relpath) {
  let st = filesystem.stat(relpath);
  let module;
  if(st.isDirectory()) {
    const name = path.basename(relpath);
    let indexes = [...makeNames(relpath + '/dist/' + name), ...makeNames(relpath + '/dist/index'), ...makeNames(relpath + '/build/' + name), ...makeNames(relpath + '/' + name), ...makeNames(relpath + '/index')];
    module = indexes.find((i) => checkExists(i));
  } else if(st.isFile()) {
    module = relpath;
  }
  if(!module) throw new Error(`Module '${relpath}' not found`);
  return module;
}

function searchModuleInPath(name, _from) {
  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
  /* console.log('thisdir:', thisdir);
  console.log('name:', name);
  console.log('_from:', _from);*/

  name = name.replace(/\..?js$/g, '');
  if(moduleAliases.has(name)) return moduleAliases.get(name);

  let names = makeNames(name);
  let indexes = [...makeNames(name + '/dist/' + name), ...makeNames(name + '/build/' + name), ...makeNames(name + '/' + name), ...makeNames(name + '/index')];

  for(let dir of [thisdir, ...searchPath]) {
    let searchFor = dir.endsWith('node_modules') ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      if(filesystem.exists(modPath)) {
        //console.info('modPath', modPath);
        let path = findModule(modPath);
        //console.info('path', path);
        if(path) return path;
      }
    }
  }
  throw new Error(`Module '${name}' imported from '${_from}' not found`);
}

function removeStatements(ast, statements, predicate = (stmt) => true) {
  //  if(Util.isArray(statements)) statements = new Map(statements);
  // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(mod=> printAst(mod.stmt)));
  // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(([path, stmt]) => stmt));
  let removed = [];
  for(let [path, node] of statements) {
    console.log('removeStatements loop:', new ImmutablePath(path), printAst(node));
    if(!predicate(node, path)) continue;
    if(node instanceof ImportStatement || (Util.isObject(node) && node.what == 'default')) {
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
}
function makeNames(prefix) {
  return [prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js'];
}
