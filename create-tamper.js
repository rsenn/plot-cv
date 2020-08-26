import { ECMAScriptParser, Printer, Lexer, PathReplacer } from './lib/ecmascript.js';
import ConsoleSetup from './consoleSetup.js';
import { ImportStatement, ExportStatement, VariableDeclaration, estree, ESNode, CallExpression, Literal } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import { ImmutablePath } from './lib/json.js';
import deep from './lib/deep.js';
import path from './lib/path.js';
import { SortedMap } from './lib/container/sortedMap.js';
import PortableFileSystem from './lib/filesystem.js';

let filesystem;

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

let cwd;

let searchPath = [];
let packagesPath = [];
let moduleAliases;
let packageFiles;
let importFiles;
let moduleList = new SortedMap();

class ES6Module {
  impExpList = [];

  static create(file) {
    let mod = new ES6Module();
    mod.file = file;
  }
}

class ES6ImportExport {
  static create(obj) {
    let ret = new.target ? this : obj;
    let nodeClass = Util.className(obj.node);
    let type = Util.decamelize(nodeClass).split('-')[0];
    let position = ESNode.assoc(obj.node).position;
    ret = Util.define(ret, { position, nodeClass, type }, obj);

    /* this.node = node;*/
    Util.log('ES6ImportExport; obj:', ret);
    if (!new.target) return Object.setPrototypeOf(ret, ES6ImportExport.prototype);
    return ret;
  }

  get from() {
    let value = this.node.source;
    while (Util.isObject(value, (v) => v.value)) value = value.value;
    return value;
  }
  set from(value) {
    Util.log('from value: ', value);
    this.node.source = value instanceof Literal ? value : new estree.Literal(`'${value}'`);
  }
  toSource() {
    return printAst(this.node);
  }
  get code() {
    return this.toSource();
  }
}
Util.log('main');
Util.callMain(main);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/

function PrefixRemover(reOrStr, replacement = '') {
  if (!(Util.isArray(reOrStr) || Util.isIterable(reOrStr))) reOrStr = [reOrStr];

  return (arg) => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), arg);
}

function dumpFile(name, data) {
  if (Util.isArray(data)) data = data.join('\n');
  if (typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  //console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

async function main(...args) {
  filesystem = await PortableFileSystem();
  Util.log('filesystem:', filesystem);
  //await ConsoleSetup();

  cwd = filesystem.realpath('.');

  // cwd = process.cwd() || fs.realpath('.');
  Util.log('cwd=', cwd);

  if (args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];
  Util.log('args=', args);
  Util.log('test', path.dirname('/usr/bin/ls'));
  Util.log('test', path.resolve('/proc/self/../ls'));
  const argDirs = [...args].map((arg) => path.dirname(arg));
  // console.log('argDirs',Util.toString(argDirs));

  const dirs = [cwd].concat(argDirs); /*.map(p => path.resolve(p))*/
  Util.log('dirs=', dirs);

  Util.log('join()', path.join(cwd, argDirs[0]));
  Util.log('cwd', cwd);
  Util.log('argDirs[0]', argDirs[0]);
  ///path.cwd = cwd;

  try {
    Util.log('relative()', path.relative(cwd, argDirs[0], cwd));
    Util.log('relative()', path.relative(argDirs[0], cwd, cwd));
  }
  catch (err) {
    Util.log(err);
  }

  searchPath = makeSearchPath(dirs);
  Util.log('searchPath=', searchPath);
  packagesPath = makeSearchPath(dirs, 'package.json');
  Util.log('packagesPath=', packagesPath);
  moduleAliases = packagesPath.reduce((acc, p) => {
    let json = JSON.parse(filesystem.readFile(p));
    let aliases = json._moduleAliases || {};
    for (let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      if (!filesystem.exists(module)) throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      let file = findModule(module);
      // let st = filesystem.stat(file);
      acc.set(alias, file);
    }
    return acc;
  }, new Map());
  Util.log('moduleAliases=', moduleAliases);
  while (args.length > 0) processFile(args.shift());
  // console.log("result:",r);
  filesystem.writeFile('new.js', r.join('\n'));
  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(processed.length == 0));

  function removeFile(file) {
    let idx = args.indexOf(file);
    if (idx != -1) args.splice(idx, idx + 1);
    processed.push(file);
  }

  function processFile(file) {
    let data, b, ret;
    Util.log('processing:', file);
    removeFile(file);
    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    data = filesystem.readFile(file).toString();
    let ast, error;
    let parser = new ECMAScriptParser(data ? data.toString() : code, file);
    let printer = new Printer({ indent: 4 });
    try {
      ast = parser.parseProgram();
      parser.addCommentsToNodes(ast);
      let flat = deep.flatten(
        ast,
        new Map(),
        (node) => node instanceof ESNode,
        (path, value) => [path, value]
      );
      function removeStatements(statements, predicate = (stmt) => true) {
        //  if(Util.isArray(statements)) statements = new Map(statements);
        // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(mod=> printAst(mod.stmt)));
        // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(([path, stmt]) => stmt));
        let removed = [];
        for (let [path, node] of statements) {
          Util.log('removeStatements loop:', new ImmutablePath(path), printAst(node));
          if (!predicate(node, path)) continue;
          if (node instanceof ImportStatement || (Util.isObject(node) && node.what == 'default')) {
            deep.unset(ast, path);
          }
          else {
            Util.log('i:', deep.get(ast, path.slice(0, -2)));
            if (!Util.isArray(node.declarations)) node = node.declarations;
            else Object.setPrototypeOf(node, VariableDeclaration.prototype);
            deep.set(ast, path, node);
          }
          removed.push(node);
        }
        return removed;
      }
      const getBase = (filename) => filename.replace(/\.[a-z0-9]*$/, '');
      const getRelative = (filename) => path.join(thisdir, filename);
      const getFile = Util.memoize((module) => searchModuleInPath(module, file));
      let imports,
        importStatements = [...flat.entries()].filter(([key, node]) => node instanceof ImportStatement);
      imports = importStatements.map(([path, node], i) => {
        const getFromValue = Util.memoize(() => node.source.value.replace(/['"](.*)['"]/, '$1'));
        const getFromBase = () => getBase(getFromValue()).replace(/^\.\//, '');
        const getFromPath = () => getFile(getFromBase());
        const getAssoc = Util.memoize(() => ESNode.assoc(node));
        return ES6ImportExport.create({
          // Object.assign(Object.setPrototypeOf({}, ES6ImportExport.prototype), {
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
      let statement2module = imports.map((imp) => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ fromPath, ...module }) => /^lib/.test(fromPath));
      alter = alter.map((node) => {
        const to = node.fromPath;
        const from = node.fromValue;
        node.from = new estree.Literal(`'${to}'`);
        Util.log(`node alter ${node.position.toString()}  => '${to}'   (was '${from}' )`, node);
        return node;
      });

      const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);
      const modulePath = removeModulesDir(file);

      /* prettier-ignore */ console.log(`${modulePath}: imports =`, imports.map(imp => [(imp.fromPath),imp.toSource()]));
      /* prettier-ignore */ console.log(`${modulePath}: alter =`, alter.map(imp => printAst(imp.node)));
      let remove = imports.map((imp, idx) => [idx, imp.node]).filter((imp, idx) => !/^lib/.test(imp.fromPath));
      /* prettier-ignore */ console.log(`${modulePath}: remove =`, remove .reduce((acc,imp) => [...acc,imp/*(imp.fromPath),imp.toSource()*/], []));

      removeStatements(remove.map(([idx, node]) => [imports[idx].path, node]));

      let recurseFiles = remove.map(([idx, node]) => imports[idx]).filter((imp) => processed.indexOf(removeModulesDir(imp.fromPath)) == -1);
      //recurseFiles = recurseFiles.map(([path,module]) => { console.log("module:",module.fromPath); return module.fromPath; });
      removeFile(modulePath);

      Util.log('processed processed:', ...processed);
      Util.log(`${modulePath}: recurseFiles =`, recurseFiles);
      recurseFiles.forEach((imp) => {
        Util.log('imp', imp);
        processFile(imp.fromPath);
      });
      let exports = [...flat.entries()].filter(([key, value]) => value instanceof ExportStatement);
      Util.log('exports:', ...exports.map(([p, stmt]) => (Util.isObject(stmt.declarations, 'id', 'value') == Util.isObject(stmt.what, 'value') ? stmt.declarations : stmt)));
    }
    catch (err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer);
    r.push(`/* --- concatenanted '${file}' --- */\n${output}\n`);
  }

  Util.log('processed processed:', ...processed);
}

function finish(err) {
  let fail = !!err;
  if (fail) {
    err.stack = PathReplacer()('' + err.stack)
      .split(/\n/g)
      .filter((s) => !/esfactory/.test(s))
      .join('\n');
  }
  if (err) {
    Util.log(parser.lexer.currentLine());
    Util.log(Util.className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
  }
  let lexer = parser.lexer;
  let t = [];
  //console.log(parser.trace() );
  dumpFile('trace.log', parser.trace());
  if (fail) {
    Util.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  Util.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}

function makeSearchPath(dirs, extra = 'node_modules') {
  let r = [];
  const addPath = (p) => ((p = path.relative(cwd, p)), r.indexOf(p) == -1 && r.push(p));
  let i = 0;
  for (let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    //console.log('parts=', parts);
    while (parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      const extra_dir = path.join(dir, extra);
      if (extra == 'node_modules') if (i == 0) addPath(dir);
      if (filesystem.exists(extra_dir)) addPath(extra_dir);
      i++;
      parts.pop();
    }
    if (extra == 'node_modules') i = 0;
  }
  return r;
}

function checkExists(path) {
  let r = filesystem.exists(path);
  //console.log(`checkExists('${path}') = ${r}`);
  return r;
}

function findModule(relpath) {
  let st = filesystem.stat(relpath);
  let module;
  if (st.isDirectory()) {
    const name = path.basename(relpath);
    let indexes = [...makeNames(relpath + '/dist/' + name), ...makeNames(relpath + '/dist/index'), ...makeNames(relpath + '/build/' + name), ...makeNames(relpath + '/' + name), ...makeNames(relpath + '/index')];
    module = indexes.find((i) => checkExists(i));
  }
  else if (st.isFile()) {
    module = relpath;
  }
  if (!module) throw new Error(`Module '${relpath}' not found`);
  return module;
}

function searchModuleInPath(name, _from) {
  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);

  /* console.log('thisdir:', thisdir);
  Util.log('name:', name);
  Util.log('_from:', _from);*/

  name = name.replace(/\..?js$/g, '');
  if (moduleAliases.has(name)) return moduleAliases.get(name);

  let names = makeNames(name);
  let indexes = [...makeNames(name + '/dist/' + name), ...makeNames(name + '/build/' + name), ...makeNames(name + '/' + name), ...makeNames(name + '/index')];

  for (let dir of [thisdir, ...searchPath]) {
    let searchFor = dir.endsWith('node_modules') ? [name] : names;
    for (let module of searchFor) {
      let modPath = path.join(dir, module);
      if (filesystem.exists(modPath)) {
        //console.log('modPath', modPath);
        let path = findModule(modPath);
        //console.log('path', path);
        if (path) return path;
      }
    }
  }
  throw new Error(`Module '${name}' imported from '${_from}' not found`);
}

function makeNames(prefix) {
  return [prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js'];
}
