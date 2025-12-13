import filesystem from 'fs';
import { SortedMap } from './lib/container/sortedMap.js';
import deep from './lib/deep.js';
import { ECMAScriptParser, ESNode, ExportNamedDeclaration, ImportDeclaration, Literal, ObjectExpression, ObjectPattern, PathReplacer, Printer, VariableDeclaration } from './lib/ecmascript.js';
import { ImmutablePath } from './lib/json.js';
import { define, isObject, memoize, unique } from './lib/misc.js';
import * as path from './lib/path.js';
#!/usr/bin/env qjsm
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
let exportMap = new Map();

const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);

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
    let nodeClass = className(obj.node);
    let type = decamelize(nodeClass).split('-')[0];
    let position = ESNode.assoc(obj.node).position;
    ret = define(ret, { position, nodeClass, type }, obj);

    /* this.node = node;*/
    //console.log('ES6ImportExport; obj:', ret);
    if(!new.target) return Object.setPrototypeOf(ret, ES6ImportExport.prototype);
    return ret;
  }

  /* prettier-ignore */ get from() {
    let value = this.node.source;
    while(isObject(value) && value.value) value = value.value;
    return value;
  }
  /* prettier-ignore */ set from(value) {
    this.node.source =
      value instanceof Literal ? value : new Literal(`'${value}'`);
  }
  toSource() {
    return printAst(this.node);
  }
  /* prettier-ignore */ get code() {
    return this.toSource();
  }
}

console.log('main');
main(...scriptArgs.slice(1));

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/

function PrefixRemover(reOrStr, replacement = '') {
  if(!(Array.isArray(reOrStr) || isIterable(reOrStr))) reOrStr = [reOrStr];

  return arg => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), arg);
}

function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFileSync(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function printAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

async function main(...args) {
  cwd = process.cwd();

  // cwd = process.cwd() || fs.realpath('.');
  console.log('cwd=', cwd);

  if(args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];
  console.log('args=', args);
  console.log('test', path.dirname('/usr/bin/ls'));
  console.log('test', path.resolve('/proc/self/../ls'));
  const argDirs = [...args].map(arg => path.dirname(arg));
  // console.log('argDirs',inspect(argDirs));

  const dirs = [cwd].concat(argDirs); /*.map(p => path.resolve(p))*/
  console.log('dirs=', dirs);

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

  searchPath = makeSearchPath(dirs);
  console.log('searchPath=', searchPath);
  packagesPath = makeSearchPath(dirs, 'package.json');
  console.log('packagesPath=', packagesPath);
  moduleAliases = packagesPath.reduce((acc, p) => {
    let json = JSON.parse(filesystem.readFileSync(p));
    let aliases = json._moduleAliases || {};
    for(let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      // if(!filesystem.existsSync(module)) throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      if(filesystem.existsSync(module)) {
        let file = findModule(module);
        // let st = filesystem.statSync(file);
        acc.set(alias, file);
      }
    }
    return acc;
  }, new Map());
  console.log('moduleAliases=', moduleAliases);
  const name = args.join(', ');
  while(args.length > 0) processFile(args.shift());
  // console.log("result:",r);

  for(let ids of exportMap.values()) r.push(`weakDefine(globalObj, { ${unique(ids).join(', ')} });`);

  const script = `// ==UserScript==

// @name         ${name}
// @namespace    create-tamper
// @version      0.2
// @description  ${processed.map(p => path.basename(p)).join(', ')}
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:3000/tamper.js
// @grant        none
// @run-at       document-end
// ==/UserScript==

/* jshint esversion: 6 */
/* jshint ignore:start */

(function(globalObj) {
  ${r.join('\n  ')}
})(window);

/* jshint ignore:end */
`;

  WriteFile('tamper.js', script);

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
    data = filesystem.readFileSync(file).toString();
    let ast, error;
    let parser = new ECMAScriptParser(data ? data.toString() : code, file);
    let printer = new Printer({ indent: 4 });
    try {
      ast = parser.parseProgram();
      parser.addCommentsToNodes(ast);
      let flat = deep.flatten(
        ast,
        new Map(),
        node => node instanceof ESNode,
        (path, value) => [path, value]
      );
      function removeStatements(statements, predicate = stmt => true) {
        //  if(Array.isArray(statements)) statements = new Map(statements);
        // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(mod=> printAst(mod.stmt)));
        // /* prettier-ignore */ console.log('removeStatements:', [...statements].map(([path, stmt]) => stmt));
        let removed = [];
        for(let [path, node] of statements) {
          if(!predicate(node, path)) continue;
          console.log('removeStatements loop:', new ImmutablePath(path), printAst(node));

          if(node instanceof ImportDeclaration || (isObject(node) && node.what == 'default')) {
            deep.unset(ast, path);
          } else {
            console.log('i:', deep.get(ast, path.slice(0, -2)));
            if(!Array.isArray(node.declarations)) node = node.declarations;
            else Object.setPrototypeOf(node, VariableDeclaration.prototype);
            deep.set(ast, path, node);
          }
          removed.push(node);
        }
        return removed;
      }
      const getBase = filename => filename.replace(/\.[a-z0-9]*$/, '');
      const getRelative = filename => path.join(thisdir, filename);
      const getFile = memoize(module => searchModuleInPath(module, file));
      let imports,
        importStatements = [...flat.entries()].filter(([key, node]) => node instanceof ImportDeclaration);
      imports = importStatements.map(([path, node], i) => {
        //   console.debug("node:",node);
        const getFromValue = memoize(() => Literal.string(node.source));
        const getFromBase = () => getBase(getFromValue()).replace(/^\.\//, '');
        const getFromPath = () => getFile(getFromBase());
        const getAssoc = memoize(() => ESNode.assoc(node));
        return ES6ImportExport.create({
          // Object.assign(Object.setPrototypeOf({}, ES6ImportExport.prototype), {
          node,
          path: path.join('.'),
          file,
          position: getAssoc().position,
          fromPath: getFromPath(),
          fromBase: getFromBase(),
          fromValue: getFromValue()
        });
      });
      let statement2module = imports.map(imp => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ fromPath, ...module }) => /^lib/.test(fromPath));
      alter = alter.map(node => {
        const to = node.fromPath;
        const from = node.fromValue;
        node.from = new Literal(`'${to}'`);
        console.log(`node alter ${node.position /*.toString()*/}  => '${to}'   (was '${from}' )`);
        return node;
      });

      log(
        `imports =`,
        imports.map(imp => imp.toSource())
      );
      log(
        `alter =`,
        alter.map(imp => printAst(imp.node))
      );
      let remove = imports.map((imp, idx) => [idx, imp.node]).filter((imp, idx) => !/^lib/.test(imp.fromPath));
      log(
        `remove =`,
        remove.reduce((acc, [i, imp]) => [...acc, imp /*(imp.fromPath),imp.toSource()*/], []).map(imp => className(imp))
      );

      removeStatements(remove.map(([idx, node]) => [imports[idx].path, node]));

      let recurseFiles = remove.map(([idx, node]) => imports[idx]).filter(imp => processed.indexOf(imp.fromPath) == -1);
      //recurseFiles = recurseFiles.map(([path,module]) => { console.log("module:",module.fromPath); return module.fromPath; });
      // removeFile(modulePath);

      log(
        `recurseFiles =`,
        recurseFiles.map(imp => imp.fromPath)
      );
      recurseFiles.forEach(imp => processFile(imp.fromPath));
      let exports = [...flat.entries()].filter(([key, value]) => value instanceof ExportNamedDeclaration || value.exported === true);

      for(let [path, node] of exports) {
        log(`export ${path}`, node);
        deep.set(ast, path, node.declarations[0]);
      }

      exports = exports.map(([p, stmt]) =>
        (isObject(stmt.declarations) && isObject(stmt.declarations.id) && isObject(stmt.declarations.id.value)) == (isObject(stmt.what) && isObject(stmt.what.value)) ? stmt.declarations : stmt
      );
      exports = exports.map(decl =>
        decl instanceof ObjectPattern
          ? decl.properties.map(prop => ('id' in prop ? prop.id : prop))
          : decl instanceof ObjectExpression
          ? decl.members.map(prop => ('id' in prop ? prop.id : prop))
          : decl
      );
      exports = exports.map(decl => (isObject(decl) && 'id' in decl ? decl.id : decl));
      exports = exports.map(e => e.value);
      log(`exports =`, exports.join(', '));

      exportMap.set(modulePath, unique(exports.flat()));
    } catch(err) {
      console.error(err.message);
      putStack(err.stack);
      exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer).trim();
    if(output != '') r = r.concat(`/* --- concatenated '${file}' --- */\n${output}\n`.split(/\n/g));

    function log(...args) {
      const assoc = args.map(arg => arg instanceof ESNode && ESNode.assoc(arg)).filter(assoc => !!assoc);
      //if(assoc[0]) console.log('ASSOC:', assoc[0].position.clone());
      const prefix = (assoc.length == 1 && assoc[0].position.clone()) || modulePath;
      console.log(prefix.toString() + ':', ...args);
    }
  }

  console.log('processed:', ...processed);
  console.log('exportMap:', exportMap);
  exit(Number(processed.length == 0));
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
    console.log(className(err) + ': ' + (err.msg || err) + '\n' + err.stack);
  }
  let lexer = parser.lexer;
  let t = [];
  //console.log(parser.trace() );
  WriteFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}

function makeSearchPath(dirs, extra = 'node_modules') {
  let r = [];
  const addPath = p => ((p = path.relative(cwd, p)), r.indexOf(p) == -1 && r.push(p));
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    //console.log('parts=', parts);
    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      const extra_dir = path.join(dir, extra);
      if(extra == 'node_modules') if (i == 0) addPath(dir);
      if(filesystem.existsSync(extra_dir)) addPath(extra_dir);
      i++;
      parts.pop();
    }
    if(extra == 'node_modules') i = 0;
  }
  return r;
}

function checkExists(path) {
  let r = filesystem.existsSync(path);
  //console.log(`checkExists('${path}') = ${r}`);
  return r;
}

function findModule(relpath) {
  let st = filesystem.statSync(relpath);
  let module;
  if(st.isDirectory()) {
    const name = path.basename(relpath);
    let indexes = [
      ...makeNames(relpath + '/dist/' + name),
      ...makeNames(relpath + '/dist/index'),
      ...makeNames(relpath + '/build/' + name),
      ...makeNames(relpath + '/' + name),
      ...makeNames(relpath + '/index')
    ];
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
      if(filesystem.existsSync(modPath)) {
        //console.log('modPath', modPath);
        let path = findModule(modPath);
        //console.log('path', path);
        if(path) return path;
      }
    }
  }
  throw new Error(`Module '${name}' imported from '${_from}' not found`);
}

function makeNames(prefix) {
  return [prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js'];
}