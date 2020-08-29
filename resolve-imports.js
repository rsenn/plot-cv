import { ECMAScriptParser } from './lib/ecmascript.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './consoleSetup.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import estree, { ImportStatement, ExportStatement, VariableDeclaration, Identifier, MemberExpression, ESNode, CallExpression, ObjectBindingPattern, Literal } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import path from './lib/path.js';
//import { Path } from './lib/json.js';
import { ImmutablePath, Path } from './lib/json.js';
import deep from './lib/deep.js';
import { SortedMap } from './lib/container/sortedMap.js';

let filesystem, searchPath, packagesPath, moduleAliases, files;
let node2path, flat, value, list;
const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);
let name;
let parser, printer;
const g = Util.getGlobalObject();
class ES6Module {
  impExpList = [];
  importedFrom = null;

  static moduleList = new Set();

  static create(file, from) {
    let mod = new ES6Module();
    mod.file = file;
    if(from) mod.importedFrom = from;

    ES6Module.moduleList.add(mod);
    return mod;
  }

  static getOrCreate(file, from) {
    let mod = ES6Module.get(file);
    if(!mod) mod = ES6Module.create(file, from);
    return mod;
  }

  static get(file) {
    return [...ES6Module.moduleList].find((m) => m.file === file);
  }
  get chain() {
    let ret = [];
    let tmp,
      mod = this;
    while(mod) {
      ret.push([mod, mod.importedFrom]);
      tmp = mod.importedFrom;
      if(tmp === mod) break;
      mod = tmp;
    }
    return ret;
  }

  toString() {
    return this.file;
  }
  valueOf() {
    return this.toString();
  }
}
const IMPORT = 1;
const EXPORT = 2;
class ES6Env {
  static getCwd = Util.memoize(() => filesystem.realpath('.') || path.resolve('.'));
  static get cwd() {
    return ES6Env.getCwd();
  }

  static pathTransform(p) {
    const { cwd } = ES6Env;
    //console.tog('pathTransform', { p, cwd });
    if(/(\\|\/)/.test(p)) return path.relative(cwd, p);
    return p;
  }
}

class ES6ImportExport {
  bindings = null;

  static importExportList = new Set();

  static create(obj) {
    let ret = new ES6ImportExport();
    let nodeClass = Util.className(obj.node);
    let code = Util.memoize(() => '' + Util.decamelize(nodeClass) + ' ' + printAst(obj.node))();
    let p, n;
    p = obj.path.slice(0, 2);

    n = p.apply(obj.ast, true);
    //console.log('ES6ImportExport obj', Util.filterOutKeys(obj, ['ast']));
    code = printAst(n);

    if(obj.node instanceof ESNode) n = obj.node;
    else p = obj.node;
    //console.log('ES6ImportExport node', n, Util.ansi.text('path', 1, 31), p, Util.ansi.text('code', 1, 31), code);

    let type = [/(import|require)/i, /exports?[^a-z]/i].filter((re) => re.test(code));
    type = type.map((re) => [...re.exec(code)]).map(([m]) => m);
    type = type.map((m) => m + '');
    type = (Util.isArray(type) && type[0]) || type;
    console.log('create  n', Util.className(n));
    console.log('create  obj.node', Util.className(obj.node));
    let position = obj.position || ESNode.assoc(obj.node || n).position;
    console.log('position', position);
    if(position) position = [...position].map((p) => ES6Env.pathTransform(p)).join(':');
    let bindings = (obj.bindings instanceof Map && obj.bindings) || new Map();

    ret = Object.assign(ret, { type, bindings, position });

    //console.log('ES6ImportExport create', { code, type, position });
    let dir = path.relative(ES6Env.cwd, path.dirname(obj.file));
    ret = Util.define(ret, { ...obj, nodeClass, dir });
    const ColorIf = (cond, str, ...codes) => {
      let b;
      if(typeof cond == 'function') b = cond();
      else b = !!cond;
      if(!b) return str;
      return Util.ansi.text(str, ...codes) + Util.ansi.code(0);
    };
    bindings[Symbol.for('nodejs.util.inspect.custom')] = function () {
      let entries = [...Util.getMemberNames(ret.bindings)]
        .filter((n) => !Util.isObject(ret[n]))

        .map((n) => [n, ret[n]]);
      //console.log('nodejs.util.inspect.custom', entries);

      //Util.isObject(bindings)  && typeof bindings.entries == 'function'  ? bindings.entries() : [...bindings];

      return (Util.ansi.code(1, 36) +
        '{' +
        entries
          .reduce((acc, [id, ref]) => {
            let s = '';
            s += Util.ansi.text(id, 1, 33);
            if(ref !== id) {
              s += Util.ansi.code(1, 31) + ' as ';
              s += Util.ansi.text(ref, 1, 33);
            }
            return [...acc, ' ', s];
          }, [])
          .join(Util.ansi.code(1, 36) + ', ') +
        Util.ansi.text('}', 1, 36)
      );
    };
    let importNodes = obj.importNode
      .filter((p) => p.length < 3)
      .map((p) => deep.get(obj.ast, p))
      .map((n) => [ESNode.assoc(n).position, printAst(n)])
      .map(([p, n]) => [Util.isGenerator(position) && [...position].map((p) => ES6Env.pathTransform(p)).join(':'), n]);
    console.log('importNodes:', importNodes.map((a) => '\n  ' + a.join('  ')).join('') + '\n');
    if(Util.isObject(position) && position.toString) position = position.toString(true, (p, i) => (i == 0 ? path.relative(ES6Env.cwd, p) : p)).replace(/1;33/, '1;34');
    const InspectFn = ret.bindings[Symbol.for('nodejs.util.inspect.custom')];
    console.log(Util.ansi.text(Util.ucfirst((type + '').toLowerCase()), 1, 31) + Util.ansi.text(` @ `, 1, 36),
      InspectFn ? InspectFn() : '',
      '\n  importNode:',
      [...obj.importNode].map((n) => [node2path.get(n), printAst(n)]).filter(([p, c]) => c.trim() != ''),
      ...['dir', 'relpath'].reduce((acc, n) => [...acc, Util.ansi.text(n, 38, 5, 197) + Util.ansi.text(' ', 1, 36) + (typeof ret[n] == 'string' ? Util.ansi.text(ret[n], 1, 32) : typeof InspectFn == 'function' ? InspectFn.call(ret) : Util.toString(ret[n], { multiline: false, newline: '' }).replace(/.*\)\ {/g, '')) + ' '], [])
    );
    Object.assign(ret, obj);
    ret.module = ES6Module.getOrCreate(ret.file);
    ret.fromModule = ES6Module.getOrCreate(ret.fromPath, ret.file);

    ES6ImportExport.importExportList.add(ret);
    return ret;
  }

  /*  static get(path) {
    let i = 0;
    for(let impexp of ES6ImportExport.importExportList) {
      console.debug(`impexp #${i}:`,impexp.fromPath)
      if(impexp.fromPath == path)
        return impexp;
      i++;
    }
  }*/

  statement(path) {
    path = path || node2path.get(this.importNode);
    console.log('importNode:', this.importNode);
    console.log('path:', path);
  }

  get relpath() {
    const { file } = this;
    //console.log('relpath()', file, Object.keys(this));
    if(ES6Env.cwd && file) {
      let r = path.relative(ES6Env.cwd, file);
      if(!/\//.test(r)) r = './' + r;
      return r;
    }
  }
  get fromBase() {
    return path.basename(this.relpath).replace(/\.([0-9a-z]{2}|[0-9a-z]{3})$/, '');
  }
  get from() {
    let { node, path } = this;

    let value = (node && node.source) || node;
    path = (value && path.down('source')) || path;

    while(Util.isObject(value, (v) => v.value)) value = value.value;
    return [value, path];
  }
  set from(value) {
    let { ast, node, path } = this;
    if(node.source) {
      let key = path.last;
      let parent = path.up();
      let obj = parent.apply(ast, true);
      obj[key] = new Literal(value);
    }
  }
  toSource() {
    return printAst(this.node);
  }
  get code() {
    return this.toSource();
  }
  [Symbol.for('nodejs.util.inspect.custom')]() {
    let { type, position, bindings, path, node, file, from, fromPath, relpath } = this;
    const opts = { colors: true, colon: ': ', multiline: false, quote: '' };
    if(Util.isIterator(position))  position = [...position].map((p) => ES6Env.pathTransform(p)).join(':');
    return Util.toString(Object.assign(
        Object.setPrototypeOf({
            type,
            bindings: Util.toString(bindings, {
              ...opts,
              newline: '',
              separator: '',
              spacing: ' ,',
              quote: "'"
            })
              .replace(/=>/g, Util.ansi.text('as', 1, 31))
              .replace(/\s*.\[1;31[^ ,]*as[^,}]*(,*)/g, '\x1b[1;36m$1 ')
              .replace(/'/g, ''),
            path: path.join('.'),
            node: printAst(node) //Util.toString(node, { ...opts, separator: '', newline: '', depth: 0 })
          },
          ES6ImportExport.prototype
        ),
        { fromPath, file, position }
      ),
      {
        ...opts,
        multiline: true
      }
    )
      .replace(/(.*){(.*)/, '\n$1{\n $2\n')
      .replace(/\n\s*\n/g, '\n')
      .replace(/\s*'\s*/g, '');

    let proto = Object.getPrototypeOf(this);
    let obj = Util.filterOutMembers(this, (x) => [Util.isFunction /*, Util.isObject*/].some((f) => f(x)));
    return Util.toString({ ...obj, __proto__: proto }, { multiline: true });
  }
  toString() {
    console.log('toString',
      ...Util.getMemberNames(this)
        .map((p) => [p, this[p]])
        .map(([k, v]) => [k, v + ''])
        .flat()
    );
    return path.relative(ES6Env.cwd, this.file || '');
  }
  [Symbol.toStringTag]() {
    return path.relative(ES6Env.cwd, this.file || '');
  }
}

Util.callMain(main, true);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
function PrintCode(node) {
  return PrintObject(node, (node) => printAst(node));
}
function PrintObject(node, t = (n, p) => n) {
  return Object.entries(node)
    .map(([prop, value]) => [prop, ': ', t(value, prop)])
    .flat();
}

function PrefixRemover(reOrStr, replacement = '') {
  if(!(Util.isArray(reOrStr) || Util.isIterable(reOrStr))) reOrStr = [reOrStr];

  return (arg) => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), typeof arg == 'string' ? arg : '');
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

const hl = {
  id: (text) => Util.ansi.text(text, 1, 33),
  punct: (text) => Util.ansi.text(text, 1, 36),
  str: (text) => Util.ansi.text(text, 1, 32)
};

function dumpNode(node) {
  return [hl.id`path` + hl.punct`:`, node2path.get(node), Util.ansi.text(`node`, 1, 33) + `:`, Util.toString(node, { multiline: false, depth: 4 })];
}

function GenerateFlatMap(ast, root = [], pred = (n, p) => true, t = (n, p) => n) {
  console.log('ast', Util.className(ast));
  flat = deep.flatten(ast,
    new Map(),
    (n, p) => n instanceof ESNode && pred(n, p),
    (p, n) => {
      /*console.log('t(', { p, n }, ')');
      console.log('root =', root);*/

      return [root.concat(p).join('.'), t(n)];
    }
  );
  console.log('flat', Util.className(flat), flat.size);
  return flat;
}
async function main(...args) {
  await ConsoleSetup({ colors: true, depth: 10 });
  filesystem = await PortableFileSystem();
  // ES6Env.cwd = filesystem.realpath('.');
  //console.log('cwd=', ES6Env.cwd);
  const re = /(lib\/util.js$)/;
  let parameters = [];

  while(/^-/.test(args[0])) parameters.push(args.shift());

  if(args.length == 0) args = [/*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];
  console.log('args=', args);

  const dirs = [ES6Env.cwd, ...args.map((arg) => path.dirname(arg))]; /*.map((p) => path.resolve(p))*/

  searchPath = makeSearchPath(dirs);
  console.log('searchPath=', searchPath);
  packagesPath = makeSearchPath(dirs, 'package.json');
  console.log('packagesPath=', packagesPath);
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
  //console.log('moduleAliases=', moduleAliases);

  while(args.length > 0) {
    let arg = args.shift();
    while(/:[0-9]+:?$/.test(arg)) arg = arg.replace(/:[0-9]*$/g, '');
    processFile(arg);
  }

  console.log('processed:', ...processed.map((file) => `\n  ${file}`));

  dumpFile(`${name}.es`, r.join('\n'));
  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(processed.length == 0));

  function removeFile(file) {
    Util.remove(args, file);
    Util.pushUnique(processed, ES6Env.pathTransform(file));
  }

  function processFile(file, depth = 0) {
    let b, ret;
    console.log('file:', file);
    if(!name) {
      name = file;
      if(/\/(src|index)[^/]*$/.test(name)) name = name.replace(/(src\/|index)[^/]*$/g, '');

      name = path.basename(name).replace(/\.[ce]?[tj]s$/, '');
    }
    const moduleDir = removeModulesDir(file);
    console.log('moduleDir:', moduleDir);
    const modulePath = path.relative(ES6Env.cwd, moduleDir);
    console.log('modulePath:', modulePath);
    console.log('processing:', ...[file, moduleDir, modulePath].reduce((acc, it) => [...acc, '\n  ', it], []));
    //  qconsole.log('processing:', ...Util.unique([file, moduleDir, modulePath]) .filter((p) => !path.isAbsolute(p)) .reduce((acc, item) => [...acc, '\n  ', item], []) );

    removeFile(file);
    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    let imports;

    let { data, error, ast, parser, printer, flat, map } = parseFile(file);
    console.log('flat size:', flat().size);

    try {
      const getRelative = (filename) => path.join(thisdir, filename);

      const useStrict = map().filter(([key, node]) => node instanceof Literal && /use strict/.test(node.value));
      console.log('useStrict:', useStrict);

      //useStrict.map(([path, node]) => deep.unset(ast, path));
      useStrict.forEach(([path, node]) => deep.unset(ast, path)); //path.remove(ast));
      const isRequire = ([path, node]) => node instanceof CallExpression && node.callee.value == 'require';
      const isImport = ([path, node]) => node instanceof ImportStatement;
      const isES6Export = ([path, node]) => node instanceof ExportStatement;
      const isCJSExport = ([path, node]) => node instanceof MemberExpression && node.object.node == 'module' && node.property.node == 'exports';

      const getImport = (arg) => {
        let [p, n] = arg;
        //console.log('getImport:', { p,n });

        let r = [];
        if(!Util.isObject(n) || !(n instanceof ESNode)) throw new Error('No node:' + n);
        // if(!Util.isObject(p) || !('length' in p)) throw new Error('No path:' + p);

        if(!(n instanceof ImportStatement)) r.push(ImmutablePath.prototype.slice.call(p, 0, 2));
        if(n instanceof CallExpression && Util.isObject(n, 'callee').value == 'require') r.push(new ImmutablePath(p.concat(['arguments', 0])));
        //console.log('getImport:', r);
        return r;
      };

      node2path = new WeakMap([...flat()].map(([path, node]) => [node, path]));
      console.log('node2path:', node2path);

      const AddValue = (...args) => {
        let tmp, arg, subj;
        while(args.length > 0) {
          arg = args.shift();
          if(typeof arg == 'function') {
            subj = args.shift() || [];
            subj = Util.isArray(subj) ? subj : [subj];
            tmp = arg(value, ...subj);
          } else tmp = typeof arg == 'string' ? value[arg] : arg;
          console.log('AddValue', (arg + '').replace(/\n.*/gm, ''), { value, tmp, list });
          if(!tmp) return;
          if(typeof tmp == 'boolean') tmp = value;
          if(tmp !== value) list.push(tmp);
          value = tmp;
          return value;
        }
      };

      imports = [...flat()].filter((it) => isRequire(it) || isImport(it));

      imports = imports.map(([path, node], i) => {
        //console.log('getFromPath   create : ', node, ...[path, Util.className(node)]);
        /* let nodes = paths.map((p) => (!(p instanceof ESNode) ? p.apply(ast, true) : p));
        let idx = nodes.findIndex((n) => typeof n.value == 'string');
        let node = idx >= 0 && nodes[idx];*/

        return ES6ImportExport.create({
          ast,
          node,
          importNode: getImport([path, node]),
          path: path,
          file,
          identifiers: node.right instanceof Identifier ? [node.right.value] : [],
          //    position: ESNode.assoc(node).position,
          fromValue: getFromValue([node, path]),
          fromPath: getFromPath([path, node], file),
          bindings: new Map(getDeclarations(node))
        });
      });
    //  console.log('imports:', Util.toString(imports, { multiline: false, depth: 4 }));

      let statement2module = imports.map((imp) => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ file, ...module }) => re.test(file));
      console.log('alter:', alter);
      alter = alter.map((node) => {
        const to = path.relative(ES6Env.cwd, node.file);
        const from = node.fromValue;
        node.from = new Literal(`'${to}'`);
        console.log(`node alter ${node.position.toString(true, (p) => path.relative(ES6Env.cwd, p))}  => '${to}'`, `\n   (was '${from}' )`, '\n  ', printAst(node.node));
        return node;
      });

      //     /* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: imports =`, imports.map(imp => [(imp.file),imp.toSource()]));
      ///* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: `, alter.map(imp => printAst(imp.node)));
      let remove = imports.map((imp, idx) => [idx, imp]); /*.filter((imp, idx) => !re.test(imp.file))*/
      //  /* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: remove =`, remove .reduce((acc,imp) => [...acc,imp/*(imp.file),imp.toSource()*/], []));
      //  console.log(`remove [${depth}]:`,  remove);

      remove.forEach(([i, imp]) => {
        let { path, node } = imp;
        console.log(`remove.forEach arg`, i, path, Util.className(node));

        //console.log(' deep.set(', i, p, Util.className(n), ')');
        deep.set(ast, [...path], undefined);
      });

      let recurseImports = Util.unique(remove.map(([idx, imp]) => imp || imports[idx]));
      console.log(`recurseImports [${depth}]:`, recurseImports);

      let recurseFiles = recurseImports
        //.filter((imp) => processed.indexOf(imp.file) == -1)
        .map((imp) => getFromPath([imp.path, imp.node], file)); //imp.from || imp.relpath)
      // .filter((imp) => !re.test(imp.file))
      console.log(`recurseFiles [${depth}] `, recurseFiles.length, recurseFiles);
      imports = imports.filter(({ file, ...module }) => !re.test(file));
      console.log(`${Util.ansi.text(modulePath, 1, 36)}: recurseFiles x${depth}]:`,
        recurseFiles.map((f) => f)
      );
      recurseFiles.forEach((imp, idx) => {
        if(processed.indexOf(imp) == -1) {
          console.log(`recurseFiles [${depth}] forEach #${idx}:`, imp);
          processFile(imp, (depth || 0) + 1);
        }
      });
      //console.log('imports:', ...imports.map((imp, i) => `\n  #${i} ` + printAst(imp.node)));

      let moduleExports = map()
        .filter((entry) => isCJSExport(entry) || isES6Export(entry))
        .map(([path, node]) => [isCJSExport([path, node]) ? path.slice(0, 2) : path, node]);
      //  console.log(`${Util.ansi.text(modulePath, 1, 36)}: moduleExports:`, moduleExports);
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    let output = '';
    output = printAst(ast, parser.comments, printer);
    r.push(`/*\n * concatenanted ${file}\n */\n\n${output}\n`);
  }

  console.log('processed files:', ...processed);
}

function parseFile(file) {
  let data, error, ast, flat;
  try {
    data = filesystem.readFile(file);
    parser = new ECMAScriptParser( data.toString() , file);
    g.parser = parser;

      ast = parser.parseProgram();
    parser.addCommentsToNodes(ast);
  } catch(err) {
    error = err;
  } finally {
    if( ast)
    flat = Util.memoize(() =>
      deep.flatten(ast,
        new Map(),
        (node) => node instanceof ESNode,
        (path, value) => [new ImmutablePath(path), value]
      )
    );
  else if(!error)
          error = new Error(`No ast for file '${file}'`);

  }
  if(error)
      throw error;

  printer = new Printer({ indent: 4 });
    g.printer = printer;


  return {
    data,
    error,
    ast,
    parser,
    printer,
    flat,
    map() {
      console.log("flat =",Util.className(flat));
    let map = flat();

      /*    return Util.mapWrapper(flat,
        (k) => k.join('.'),
        (k) => k.split('.')
      );
*/
      return Util.mapAdapter((key, value) =>
        key !== undefined
          ? value !== undefined
            ? value === null
              ? deep.unset(ast, [...key])
              : deep.set(ast, [...key], value)
            : deep.get(ast, key)
          : (function* () {
              for(let [key, value] of map) yield [new ImmutablePath(key), key.apply(ast, true)];
            })()
      );
    }
  };
}

function getFile(module, file) {
  console.log(`getFile(  `, module, file, ` )`);
  // Util.memoize((module) => searchModuleInPath(module, file));
  //console.log(`getFile( module:`, Util.entries(module), `file:`, file, ` )`);
  return searchModuleInPath(module, file);
}

function getFromValue(...args) {
  if(args[0] == undefined) args.shift();
  //   args= args.shift();
  if(Util.isArray(args) && args.length == 1) args = args[0];
  //console.log('getFromValue arg', args);
  //let arg = args.shift() || [];
  let [p, n] = args[0] instanceof ESNode ? args.reverse() : args; //rgs[0] ;
  //console.log('getFromValue args', { n, p });
  if(!p || (!('length' in p) && typeof p != 'string')) throw new Error('No path:' + p + ' node:' + printAst(n));
  if(!n || !(n instanceof ESNode)) throw new Error('No node:' + n + ' path:' + p);
  if(!(n instanceof ESNode)) n = deep.get(ast, n);
  let pathStr = p.join('.');
  let flat = GenerateFlatMap(n,
    p,
    (n, p) => true || Util.isArray(n) || [ExportStatement, ImportStatement, ObjectBindingPattern, Literal].some((ctor) => n instanceof ctor),
    (n, p) => Object.setPrototypeOf({ ...Util.filterKeys(n, (k) => n instanceof CallExpression || (k != 'type' && !(Util.isObject(n[k]) || Util.isFunction(n[k])))) }, Object.getPrototypeOf(n))
  );
  //let calls = [...flat.entries()].map(([p, n]) => [new Path(p), n]).filter(([p, n]) => [...p.slice(-2)].join('.') == 'arguments.0');
  let literals = [...flat.entries()].filter(([p, n]) => n instanceof Literal);
  //console.log('getFromValue literals', literals);
  let paths = literals.map(([p, n]) => flat.get(p).value);
  //console.log('getFromValue paths', paths);
  if(paths) {
    let v = paths.map((n) => n.replace(/^[\u0027\u0022\u0060](.*)[\u0027\u0022\u0060]$/g, '$1'))[0];
    if(v) {
      //console.log('getFromValue v', v);
      return v;
    }
  }
}

function getFrom([node, path]) {
  if(node instanceof CallExpression) {
    node = node['arguments'][0];
    path = path.down('arguments', 0);
  }
  if(node instanceof Literal) {
    node = node['value'];
    path = path.down('value');
  }
  return [node, path];
}

function getFromBase(path, node) {
  //console.log('getFromBase  : ', { path, node });
  let str = getFromValue([node, path]);
  if(typeof str == 'string') str = str.replace(/^\.\//, '');
  return str;
}

function getFromPath([path, node], file) {
  //console.log('getFromPath:', { path, node });
  let fileName,
    fromStr = getFromValue([node, path]);
  //console.log('getFromPath fromStr: ', fromStr);
  if(typeof fromStr == 'string') fileName = getFile(fromStr, file);
  //console.log('getFromPath:', { fileName });
  return fileName;
}

function getBase(filename) {
  return path.basename(filename).replace(/\.[a-z0-9]*$/, '');
}

function readdirRecursive(dir) {
  let ret = [];
  for(let entry of filesystem.readdir(dir)) {
    let file = path.join(dir, entry);
    if(filesystem.stat(file).isDirectory()) {
      ret = ret.concat(readdirRecursive(file));
      continue;
    }
    ret.push(entry);
  }
  return ret;
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

function getBindings(properties) {
  return properties.filter((node) => node).map((node) => (node.id ? [node.id.value, node.value.value] : [node, node]));
}

function makeSearchPath(dirs, extra = 'node_modules') {
  let r = [];
  const { cwd } = ES6Env;
  const addPath = (p) => {
    //console.log('cwd=', cwd, typeof cwd);
    console.log('p=', p, typeof p);

    p = path.relative(cwd, p, cwd);
   if(r.indexOf(p)== -1) 
    r.unshift(p);
  };
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    console.log('parts=', parts);

    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      console.log('dir=', dir);
      const extra_dir = path.join(dir, extra);
      if(extra == 'node_modules')
        if(i == 0) {
          console.log('dir=', dir);
          addPath(dir);
        }
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
  //console.log(`checkExists('${path}') = ${r}`);
  return r;
}

function findModule(relpath) {
  let st = filesystem.stat(relpath);
  let module;
  const name = path.basename(relpath);
  let indexes = [relpath + '/package.json', ...makeNames(relpath + '/src/index'), ...makeNames(relpath + '/src/' + name), makeNames(relpath + '/dist/' + name), ...makeNames(relpath + '/dist/index'), ...makeNames(relpath + '/build/' + name), ...makeNames(relpath + '/' + name), ...makeNames(relpath + '/index'), ...makeNames(relpath + '/browser')];
  console.log('findModule(', relpath, ')');

  if(st.isDirectory()) {
    while(indexes.length) {
      module = indexes.shift();
      console.log('findModule(', relpath, ')', { module });

      if(!checkExists(module)) continue;
      if(module.endsWith('/package.json')) {
        let r;
        if((r = filesystem.readFile(module))) {
          let json = JSON.parse(r.toString());
          if(!json.module) continue;
          module = path.join(relpath, json.module);
        }
      }
      break;
    }
    if(!checkExists(module)) {
      module = null;
      let entries = readdirRecursive(relpath).filter((name) => /\.js$/.test(name));

      if(entries.length == 1) module = path.join(relpath, entries[0]);
      //            console.log('findModule()',{entries });
    }
  } else if(st.isFile()) {
    module = relpath;
  }
  if(!module) throw new Error(`Module '${relpath}' not found`);
  return module;
}

function searchModuleInPath(name, _from, position) {
  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
   const isRelative = /^\.\.?\//.test(name);
  /* console.log('thisdir:', thisdir);
  console.log('name:', name);
  console.log('_from:', _from);*/
  name = name.replace(/\..?js$/g, '');
  if(moduleAliases.has(name)) return moduleAliases.get(name);
  let names = [name, ...makeNames(name)];
//  let indexes = [name, ...makeNames(name + '/dist/' + name), ...makeNames(name + '/build/' + name), ...makeNames(name + '/' + name), ...makeNames(name + '/index')];
  let searchDirs = isRelative ? [thisdir] : [thisdir, ...searchPath];
  console.log('searchModuleInPath(', { name, _from,  searchDirs },')');
 for(let dir of searchDirs) {
    let searchFor = dir.endsWith('node_modules') && !/\//.test(name) ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      let p;
      console.log('modPath',   modPath);

      if(filesystem.exists(modPath)) p = findModule(modPath);

      if(p) {
        //console.log('path', p, modPath);
        return p;
      }
    }
  }
  let fromModule = ES6Module.get(_from);
  if(!fromModule) throw new Error(`Module "${_from}" not found (${name})`, name);
  let chain = fromModule.chain;
  console.log('_from:', _from);
  console.log('chain:', chain);

  throw new URIError(`Module '${name}' imported from '${_from}' not found ` + Util.toString({ name, chain }, { multiline: false }), name);
}

function removeStatements(ast, statements, predicate = (stmt) => true) {
  let removed = [];
  for(let [path, node] of statements) {
    console.log('removeStatements loop:', new ImmutablePath(path), printAst(node));
    if(!predicate(node, path)) continue;
    deep.unset(ast, path);
    removed.push(node);
  }
  return removed;
}
function getDeclarations(node) {
  let acc = [];
  if(node.identifiers) node = node.identifiers;
  if(node.declarations)
    acc = node.declarations.reduce((acc, decl) => {
      if(decl.id) decl = decl.id;
      if(decl.properties) acc = acc.concat(getBindings(decl.properties));
      if(decl.value) acc = acc.concat([[Symbol.for('default'), decl.value]]);
      return acc;
    }, acc);
  if(node.properties) acc = acc.concat(getBindings(node.properties));
  return acc;
}

function makeNames(prefix) {
  return [prefix + '.njs', prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js'];
}
