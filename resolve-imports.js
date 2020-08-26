import { ECMAScriptParser } from './lib/ecmascript.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './consoleSetup.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import {
  ImportStatement,
  ExportStatement,
  VariableDeclaration,
  Identifier,
  MemberExpression,
  estree,
  ESNode,
  CallExpression,
  ObjectBindingPattern,
  Literal
} from './lib/ecmascript/estree.js';
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
const IMPORT = 1;
const EXPORT = 2;
class ES6Env {
  static getCwd = Util.memoize(() => filesystem.realpath('.') || path.resolve('.'));
  static get cwd() {
    return ES6Env.getCwd();
  }

  static pathTransform(p) {
    const { cwd } = ES6Env;
    console.log('pathTransform', { p, cwd });
    if(/(\\|\/)/.test(p)) return path.relative(cwd, p);
    return p;
  }
}

class ES6ImportExport {
  bindings = null;

  static create(obj) {
    let ret = new ES6ImportExport();
    let nodeClass = Util.className(obj.node);
    let code = Util.memoize(() => '' + Util.decamelize(nodeClass) + ' ' + printAst(obj.node))();
    let p, n;
    p = obj.path.slice(0, 2);

    n = p.apply(obj.ast, true);
    console.log('ES6ImportExport obj', Util.filterOutKeys(obj, ['ast']));
    code = printAst(n);

    if(obj.node instanceof ESNode) n = obj.node;
    else p = obj.node;
    console.log(
      'ES6ImportExport  node',
      n,
      Util.ansi.text('path', 1, 31),
      p,
      Util.ansi.text('code', 1, 31),
      code
    );

    let type = [/(import|require)/i, /exports?[^a-z]/i].filter((re) => re.test(code));
    type = type.map((re) => [...re.exec(code)]).map(([m]) => m);
    type = type.map((m) => m + '');
    type = (Util.isArray(type) && type[0]) || type;
    let position = ESNode.assoc(n || obj.node).position;

    position = [...position].map((p) => ES6Env.pathTransform(p)).join(':');
    console.log('position', position);
    let bindings = (obj.bindings instanceof Map && obj.bindings) || new Map();

    ret = Object.assign(ret, { type, bindings, position });

    console.log('ES6ImportExport create', { code, type, position });
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
      console.log('nodejs.util.inspect.custom', entries);

      //Util.isObject(bindings)  && typeof bindings.entries == 'function'  ? bindings.entries() : [...bindings];

      return (
        Util.ansi.code(1, 36) +
        '{' +
        entries
          .reduce((acc, [id, ref]) => {
            let s = '';
            s += Util.ansi.text(id, 1, 33);
            if(ref !== id) {
              s += Util.ansi.code(1, 31) + ' as ';
              s += Util.ansi.text(ref, 1, 33);
            }
            return [...acc, s];
          }, [])
          .join(Util.ansi.code(1, 36) + ',') +
        Util.ansi.text('}', 1, 36)
      );
    };
    /* this.ret = ret;*/
    console.log('importNode:', obj.importNode);
    if(Util.isObject(position) && position.toString)
      position = position
        .toString(true, (p, i) => (i == 0 ? path.relative(ES6Env.cwd, p) : p))
        .replace(/1;33/, '1;34');
    const InspectFn = ret.bindings[Symbol.for('nodejs.util.inspect.custom')];
    console.log(
      Util.ansi.text(Util.ucfirst((type + '').toLowerCase()), 1, 31) + Util.ansi.text(` @ `, 1, 36),
      InspectFn ? InspectFn() : '',
      '\n  importNode:',
      [...obj.importNode]
        .map((n) => [node2path.get(n), printAst(n)])
        .filter(([p, c]) => c.trim() != ''),
      ...[/*'bindings',*/ 'dir', 'relpath' /*, 'abspath' */].reduce(
        (acc, n) => [
          ...acc,
          Util.ansi.text(n, 38, 5, 197) +
            Util.ansi.text(' ', 1, 36) +
            (typeof ret[n] == 'string'
              ? Util.ansi.text(ret[n], 1, 32)
              : typeof InspectFn == 'function'
              ? InspectFn.call(ret)
              : Util.toString(ret[n], { multiline: false, newline: '' }).replace(/.*\)\ {/g, '')) +
            ' '
        ],
        []
      )
      /*   , Util.getMemberNames(obj)*/
    );
    // if(!new.target) return Object.setPrototypeOf(ret, ES6ImportExport.prototype);
    Object.assign(ret, obj);
    console.log('ES6ImportExport ret', ret);
    return ret;
  }
  statement(path) {
    path = path || node2path.get(this.importNode);
    console.log('importNode:', this.importNode);
    console.log('path:', path);
  }

  get relpath() {
    const {   file } = this;
    console.log('relpath()',   file, Object.keys(this));
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

    let value = node && node.source || node;
    path = value && path.down('source') || path;

    while(Util.isObject(value, (v) => v.value)) value = value.value;
    return [value,path];
  }
  set from(value) {
      let { ast, node, path } = this;

if(node.source) {
  
  let key = path.last;
  let parent = path.up();
let obj = parent.apply(ast, true);

obj[key] = new Literal(value);
}
/*   node.source = value;
    let value = node && node.source || node;
  value = this.relpath || value;
    console.log('from value: ', value);
    value = value instanceof Literal ? value : new Literal(`'${value}'`);
    if(this.node.source) this.node.source = value;
    else this.node.arguments = [value];*/
  }
  toSource() {
    return printAst(this.node);
  }
  get code() {
    return this.toSource();
  }
  [Symbol.for('nodejs.util.inspect.custom')]() {
    const { type, position, bindings, path, node,file,from, fromPath,relpath } = this;
    const opts = { colors: true, colon: ': ', multiline: false, quote: '' };
    return (
      Util.toString(
        Util.define( 
        Object.setPrototypeOf(
          {
            type,
           position: [...position].map((p) => ES6Env.pathTransform(p)).join(':'),
            bindings: Util.toString(bindings, {...opts, newline: '', separator: '', spacing: '', quote: "'"}),
            path: path.join('.'),
            node: Util.toString(node, {...opts, separator: '', newline: ''})/*.replace(/\n\s+/g, '') */
          },
          ES6ImportExport.prototype
        ), { file, relpath }),
        { ...opts,/*separator: '  ', newline: '\n',*/multiline: true }
      )
        .replace(/(.*){(.*)/, '\n$1{\n $2\n')
        .replace(/\n\s*\n/g, '\n')
//        .replace(/'/g, '')
     //   .replace(/\s*}([^}]*)$/, '$1') + '}\n--\n'
    );

    let proto = Object.getPrototypeOf(this);
    let obj = Util.filterOutMembers(this, (x) =>
      [Util.isFunction /*, Util.isObject*/].some((f) => f(x))
    );
    return Util.toString({ ...obj, __proto__: proto }, { multiline: true });
  }
  toString() {
    console.log(
      'toString',
      ...Util.getMemberNames(this)
        .map((p) => [p, this[p]])
        .map(([k, v]) => [k, v + ''])
        .flat()
    );
    return path.relative(ES6Env.cwd, this.abspath || '');
  }
  [Symbol.toStringTag]() {
    return path.relative(ES6Env.cwd, this.abspath || '');
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

  return (arg) =>
    reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), typeof arg == 'string' ? arg : '');
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

const dumpNode = (node) => [
  hl.id`path` + hl.punct`:`,
  node2path.get(node),
  Util.ansi.text(`node`, 1, 33) + `:`,
  Util.toString(node, { multiline: false, depth: 4 })
];

function GenerateFlatMap(ast, root = [], pred = (n, p) => true, t = (n, p) => n) {
  console.log('ast', Util.className(ast));
  flat = deep.flatten(
    ast,
    new Map(),
    (n, p) => n instanceof ESNode && pred(n, p),
    (p, n) => {
      console.log('t(', { p, n }, ')');
      console.log('root =', root);

      return [root.concat(p).join('.'), t(n)];
    }
  );
  console.log('flat', flat);
  return flat;
}
async function main(...args) {
  await ConsoleSetup({ colors: true, depth: 10 });
  filesystem = await PortableFileSystem();
  // ES6Env.cwd = filesystem.realpath('.');
  console.log('cwd=', ES6Env.cwd);
  const re = /(lib\/util.js$)/;
  while(/^-/.test(args[0])) {
    let arg = args.shift();
  }

  if(args.length == 0)
    args = [
      /*'lib/geom/align.js', 'lib/geom/bbox.js','lib/geom/line.js'*/ 'lib/geom/point.js',
      'lib/geom/size.js',
      'lib/geom/trbl.js',
      'lib/geom/rect.js',
      'lib/dom/element.js'
    ];
  let r = [];
  let processed = [];
  console.log('args=', args);

  const dirs = [
    ES6Env.cwd,
    ...args.map((arg) => path.dirname(arg))
  ]; /*.map((p) => path.resolve(p))*/

  searchPath = makeSearchPath(dirs);
  console.log('searchPath=', searchPath);
  packagesPath = makeSearchPath(dirs, 'package.json');
  console.log('packagesPath=', packagesPath);
  moduleAliases = packagesPath.reduce((acc, p) => {
    let json = JSON.parse(filesystem.readFile(p));
    let aliases = json._moduleAliases || {};
    for(let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      if(!filesystem.exists(module))
        throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      let file = findModule(module);
      let st = filesystem.stat(file);
      acc.set(alias, file);
    }
    return acc;
  }, new Map());
  console.log('moduleAliases=', moduleAliases);

  while(args.length > 0) processFile(args.shift());

  console.log('processed:', ...processed.map((file) => `\n  ${file}`));

  dumpFile(`${name}.es`, r.join('\n'));
  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(processed.length == 0));

  function removeFile(file) {
    Util.remove(args, file);
    Util.pushUnique(processed, file);
  }

  function processFile(file, depth = 0) {
    let b, ret;
    if(!name) {
      name = file;
      if(/\/(src|index)[^/]*$/.test(name)) name = name.replace(/(src\/|index)[^/]*$/g, '');

      name = path.basename(name).replace(/\.[ce]?[tj]s$/, '');
    }
    const moduleDir = removeModulesDir(file);
    const modulePath = path.relative(ES6Env.cwd, moduleDir);
    console.log('processing:', { file, moduleDir, modulePath });

    removeFile(file);
    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    let imports;

    let { data, error, ast, parser, printer, flat, map } = parseFile(file);
    console.log('flat size:', flat().size);

    try {
      const getBase = (filename) => path.basename(filename).replace(/\.[a-z0-9]*$/, '');
      const getRelative = (filename) => path.join(thisdir, filename);
      const getFile = (module, file) => {
        console.log(`getFile(  `, args, file, ` )`);
        // Util.memoize((module) => searchModuleInPath(module, file));
        console.log(`getFile( module:`, Util.entries(module), `file:`, file, ` )`);
        return searchModuleInPath(module, file);
      };

      const useStrict = map().filter(
        ([key, node]) => node instanceof Literal && /use strict/.test(node.value)
      );
      console.log('useStrict:', useStrict);

      //useStrict.map(([path, node]) => deep.unset(ast, path));
      useStrict.forEach(([path, node]) => deep.unset(ast, path)); //path.remove(ast));
      const isRequire = ([path, node]) =>
        node instanceof CallExpression && node.callee.value == 'require';
      const isImport = ([path, node]) => node instanceof ImportStatement;
      const isES6Export = ([path, node]) => node instanceof ExportStatement;
      const isCJSExport = ([path, node]) =>
        node instanceof MemberExpression &&
        node.object.node == 'module' &&
        node.property.node == 'exports';

      const getImport = ([p, n]) => {
        let r = [p];
        if(!('length' in p)) throw new Error('No path:' + p);

        console.log('getImport:', { p, n });
        if(!(n instanceof ImportStatement)) r.push(ImmutablePath.prototype.slice.call(p, 0, 2));
        if(n instanceof CallExpression && Util.isObject(n, 'callee').value == 'require')
          r.push(new ImmutablePath(p.concat(['arguments', 0])));
        console.log('getImport:', r);
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
      const getFromValue = ([n, p]) => {
        console.log('getFromValue args', { n, p });
        if(!p || !('length' in p)) throw new Error('No path:' + p + ' node:' + printAst(n));
        if(!n || !(n instanceof ESNode)) throw new Error('No node:' + n + ' path:' + p);
        if(!(n instanceof ESNode)) n = deep.get(ast, n);
        let pathStr = p.join('.');
        let flat = GenerateFlatMap(
          n,
          p,
          (n, p) =>
            true ||
            Util.isArray(n) ||
            [ExportStatement, ImportStatement, ObjectBindingPattern, Literal].some(
              (ctor) => n instanceof ctor
            ),
          (n, p) =>
            Object.setPrototypeOf(
              {
                ...Util.filterKeys(
                  n,
                  (k) =>
                    n instanceof CallExpression ||
                    (k != 'type' && !(Util.isObject(n[k]) || Util.isFunction(n[k])))
                )
              },
              Object.getPrototypeOf(n)
            )
        );
        console.log('getFromValue flat', [...flat.entries()]);
        let calls = [...flat.entries()]
          .map(([p, n]) => [new Path(p), n])
          .filter(([p, n]) => [...p.slice(-2)].join('.') == 'arguments.0');
        let literals = calls.filter(([p, n]) => n.v);
        let paths = literals.map(([p, n]) => [p.down('v'), n['v']]);
        if(paths) {
          let v = paths.map(([p, n]) =>
            n.replace(/^[\u0027\u0022\u0060](.*)[\u0027\u0022\u0060]$/g, '$1')
          )[0];
          if(v) {
            console.log('getFromValue v', v);
            return v;
          }
        }
        /*  node = node || (flat && flat.get && flat.get(pathStr));
        let result = [...flat].filter(([path, node]) => /(arguments.0.value|source)$/.test(path)).map(([p, n]) => [new Path(p), n])[0];
        console.log('getFromValue result', result);
        if(result) [path, node] = result;
        let pt = new ImmutablePath(path);
        let stmtPath = pt.slice(0, 2);
        let stmtFlat = flat.get(stmtPath.join('.'));
        let stmtNode = stmtPath.apply(ast, true);
        list = [];
        value = deep.get(ast, path);
           AddValue(Util.isObject, 'arguments', 0);
        AddValue(Util.isObject, 'declarations');
        AddValue(Util.isObject, 'source');
        AddValue(Util.isObject, 'value');
        AddValue(Util.isObject, (v) => v.length === 1);
        Util.isObject(value, 'declarations') || Util.isObject(value, 'source');
        if(Util.isObject(value) && typeof value.value != 'undefined') AddValue(value.value);
        //console.log('value:',  (value));
        if(Util.isArray(value) && value.length == 1) AddValue(value[0]);
        if(typeof value == 'string') value = value.replace(/['"](.*)['"]/, '$1');
        console.log(
          'list:',
          list.map((n) => node2path.get(n))
        );* 
        console.log('flat:', flat);
        console.log('Util.className(value):', Util.className(value));
        console.log('Util.className(path):', Util.className(path));
        if(typeof value != 'string') if (value instanceof ESNode) value = printAst(value);
        if(value == undefined) throw new Error([`getFromValue path = ` + path, Util.ansi.code(1, 37), `  `, node, Util.toString(node, { multiline: false, depth: 4 })].join('\n'));
        console.log('getFromValue value:',  Util.className(node), ...PrintObject(node));
        return value;*/
      };
      const getFromBase = (path, node) => {
        let str = getFromValue(node, path);
        if(typeof str == 'string') str = str.replace(/^\.\//, '');
        return str;
      };
      const getFromPath = (path, node) => {
        let fileName,
          fromStr = getFromBase(path, node);
        console.log('getFromPath fromStr: ' + fromStr);

        if(typeof fromStr == 'string') fileName = getFile(fromStr);
        console.log('getFromPath fileName: ', fileName);
        return fileName;
      };
      const getBindings = (properties) =>
        properties
          .filter((node) => node)
          .map((node) => (node.id ? [node.id.value, node.value.value] : [node, node]));

      imports = [...flat()].filter((it) => isRequire(it) || isImport(it));
      console.log('getFromPath imports: ', imports);
      imports = imports.map(getImport);
      console.log('getFromPath imports: ', imports);

      imports = imports.map((paths, i) => {
        let nodes = paths.map((p) => (!(p instanceof ESNode) ? p.apply(ast, true) : p));
        let idx = nodes.findIndex((n) => typeof n.value == 'string');
        let node = idx >= 0 && nodes[idx];

        /*
            if(!(node instanceof ESNode)) 
              node = node.apply(ast, true);*/
        console.log('getFromPath imports create : ', node);

        return ES6ImportExport.create({
          ast,
          node,
          importNode: getImport([paths[idx], node]),
          path: paths[idx],
          file,
          identifiers: node.right instanceof Identifier ? [node.right.value] : [],
          position: ESNode.assoc(node).position,
          fromValue: getFromValue([node, paths[idx]]),
          bindings: new Map(getDeclarations(node))
        });
      });
      console.log('imports:', Util.toString(imports, { multiline: false, depth: 4 }));

      imports.forEach((imp, i) => {
        console.log(`import #${i}:`, PrintCode(imp));
        //imp.from = imp.abspath;
      });
      //  const importList = (a  = []) => imports.reduce((acc, imp, i) => [...acc, `\n\t#${i}: `, ... PrintObject(imp)  ], a);
      const importList = (a = []) => imports.map((imp, i) => PrintObject(imp) || imp).flat();
      console.log('importsList() = ', importList());

      let statement2module = imports.map((imp) => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ abspath, ...module }) => re.test(abspath));
      console.log('alter:', alter);
      alter = alter.map((node) => {
        const to = path.relative(ES6Env.cwd, node.abspath);
        const from = node.fromValue;
        node.from = new Literal(`'${to}'`);
        console.log(
          `node alter ${node.position.toString(true, (p) =>
            path.relative(ES6Env.cwd, p)
          )}  => '${to}'`,
          `\n   (was '${from}' )`,
          '\n  ',
          printAst(node.node)
        );
        return node;
      });

      //     /* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: imports =`, imports.map(imp => [(imp.abspath),imp.toSource()]));
      ///* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: `, alter.map(imp => printAst(imp.node)));
      let remove = imports.map((imp, idx) => [
        idx,
        imp.path,
        imp.node
      ]); /*.filter((imp, idx) => !re.test(imp.abspath))*/
      //  /* prettier-ignore */ console.log(`${Util.ansi.text(modulePath,1,36)}: remove =`, remove .reduce((acc,imp) => [...acc,imp/*(imp.abspath),imp.toSource()*/], []));

      remove.forEach(([i, p, n]) => {
        //  console.log(`remove.forEach arg`, i,p,n  );

        console.log(' deep.set(', i, p, n, ')');
        deep.set(ast, p, undefined);
      });

      let recurseFiles = remove
        .map(([idx, path, node]) => imports[idx])
        .filter((imp) => processed.indexOf(imp.abspath) == -1)
        .filter((imp) => !re.test(imp.abspath));
      console.log('recurseFiles:', ...recurseFiles);
      imports = imports.filter(({ abspath, ...module }) => !re.test(abspath));
      console.log(
        `${Util.ansi.text(modulePath, 1, 36)}: recurseFiles =`,
        recurseFiles.map((f) => f.abspath)
      );
      recurseFiles.forEach((imp, idx) => {
        console.log(`recurseFiles [${depth}] forEach #${idx}:`, printAst(imp.node));
        //  processFile(imp.file, (depth||0)+1);
      });
      //console.log('imports:', ...imports.map((imp, i) => `\n  #${i} ` + printAst(imp.node)));

      let moduleExports = map()
        .filter((entry) => isCJSExport(entry) || isES6Export(entry))
        .map(([path, node]) => [isCJSExport([path, node]) ? path.slice(0, 2) : path, node]);
      console.log(`${Util.ansi.text(modulePath, 1, 36)}: moduleExports:`, moduleExports);
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
  let data, error, ast, parser, printer, flat;
  try {
    data = filesystem.readFile(file).toString();
    parser = new ECMAScriptParser(data ? data.toString() : code, file);
    printer = new Printer({ indent: 4 });
    ast = parser.parseProgram();
    parser.addCommentsToNodes(ast);
  } catch(err) {
    error = err;
    throw err;
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
  const { cwd } = ES6Env;
  const addPath = (p) => {
    console.log('cwd=', cwd, typeof cwd);
    console.log('p=', p, typeof p);

    (p = path.relative(cwd, p, cwd)), r.indexOf(p) == -1 && r.push(p);
  };
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    //console.log('parts=', parts);
    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
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
  let indexes = [
    relpath + '/package.json',
    ...makeNames(relpath + '/src/index'),
    makeNames(relpath + '/dist/' + name),
    ...makeNames(relpath + '/dist/index'),
    ...makeNames(relpath + '/build/' + name),
    ...makeNames(relpath + '/' + name),
    ...makeNames(relpath + '/index')
  ];

  if(st.isDirectory()) {
    while(indexes.length) {
      module = indexes.shift();
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
    if(!checkExists(module)) module = null;
  } else if(st.isFile()) {
    module = relpath;
  }
  if(!module) throw new Error(`Module '${relpath}' not found`);
  return module;
}

function searchModuleInPath(name, _from, position) {
  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
  console.log('searchModuleInPath(', name, _from, ')');
  /* console.log('thisdir:', thisdir);
  console.log('name:', name);
  console.log('_from:', _from);*/
  name = name.replace(/\..?js$/g, '');
  if(moduleAliases.has(name)) return moduleAliases.get(name);
  let names = makeNames(name);
  let indexes = [
    ...makeNames(name + '/dist/' + name),
    ...makeNames(name + '/build/' + name),
    ...makeNames(name + '/' + name),
    ...makeNames(name + '/index')
  ];
  for(let dir of [thisdir, ...searchPath]) {
    let searchFor = dir.endsWith('node_modules') ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      if(filesystem.exists(modPath)) {
        //console.log('modPath', modPath);
        let path = findModule(modPath);
        //console.log('path', path);
        if(path) return path;
      }
    }
  }
  throw new URIError(`Module '${name}' imported from '${_from}' not found`, name);
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
  return [
    prefix + '.njs',
    prefix + '.es6.js',
    prefix + '.esm.js',
    prefix + '.module.js',
    prefix + '.module.ejs',
    prefix + '.js'
  ];
}
