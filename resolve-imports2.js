import fs from 'fs';
import { ReadFile, WriteFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { ECMAScriptParser, PathReplacer } from './lib/ecmascript.js';
import { ArrayPattern, AssignmentExpression, CallExpression, ESNode, ExportDefaultDeclaration, ExportNamedDeclaration, ExpressionStatement, Identifier, ImportDeclaration, Literal, MemberExpression, ModuleSpecifier, ObjectPattern, VariableDeclaration, VariableDeclarator } from './lib/ecmascript/estree.js';
import Printer from './lib/ecmascript/printer.js';
import { Token } from './lib/ecmascript/token.js';
import { Path } from './lib/json.js';
import { define, isObject, memoize, unique } from './lib/misc.js';
import * as path from './lib/path.js';
import { Repeater } from './lib/repeater/repeater.js';
import { RepeaterSink } from './lib/stream/utils.js';
import Tree from './lib/tree.js';
import { Console } from 'console';
import inspect from 'inspect';
let childProcess, search, files;
let node2path, flat, value, list;

const removeModulesDir = PrefixRemover([/node_modules[/]/g, /^\.\//g]);
let name;
let parser, printer;

const g = globalThis;
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');
const packages = new Map();

let searchExts = ['.js', '.mjs'];
let exportMap = {};
let importMap = {};

define(Array.prototype, {
  startsWith(other) {
    if(other.length > this.length) return false;
    for(let i = 0; i < other.length; i++) if(this[i] !== other[i]) return false;
    return true;
  },
  endsWith(other) {
    let offset = this.length - other.length;
    if(offset < 0) return false;
    for(let i = 0; i < other.length; i++) if(this[i + offset] !== other[i]) return false;
    return true;
  },
  removeStart(other) {
    return this.slice(this.startsWith(other) ? other.length : 0);
  },
  removeEnd(other) {
    return this.slice(0, this.endsWith(other) ? this.length - other.length : this.length);
  }
});

class ES6Module {
  /*importedFrom = null;*/
  root = null;

  static moduleList = new Set();

  static create(file, from, position) {
    let mod = new ES6Module();
    mod.file = file;
    if(position || from) define(mod, { importedFrom: position ? position : from });
    if(ES6Module.root == null) ES6Module.root = mod;
    ES6Module.moduleList.add(mod);
    return mod;
  }

  static get list() {
    return [...ES6Module.moduleList];
  }

  static getOrCreate(file, from, position) {
    let mod = ES6Module.get(file);
    if(!mod) mod = ES6Module.create(file, from, position);
    return mod;
  }

  static get(file) {
    return [...ES6Module.moduleList].find(m => m.file === file);
  }
  /* prettier-ignore */ get chain() {
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

  [inspectSymbol]() {
    const t = (...args) => ansi.text(...args);
    let s = t(className(this), 1, 31) + t(' {', 1, 36);
    for(let prop of ['file', 'importedPosition', 'bindings']) if(this[prop]) s += '\n' + t(prop.padStart(13, ' '), 1, 33) + t(': ', 1, 36) + formatProp(this[prop]);
    s += t('\n}', 1, 36);
    function formatProp(value) {
      if(Array.isArray(value)) return t('[ ', 1, 36) + value.map(mod => formatProp(mod.file)).join(t(', ', 1, 36)) + t(' ]', 1, 36);
      return typeof value == 'string' ? t(value, 1, 32) : inspect(value).replaceAll('\n', '\n  ');
    }
    return s;
  }

  /* prettier-ignore */ get deps() {
    return [...ES6Module.getDeps(this)];
  }

  /* prettier-ignore */ get imports() {
    return [...ES6Module.getImportExport(this, (i) => i == -1)];
  }

  /* prettier-ignore */ get exports() {
    return [...ES6Module.getImportExport(this, (i) => i != -1)];
  }

  /* prettier-ignore */ get bindings() {
    let imports = [...ES6ImportExport.importExportList].filter(impExp => impExp.fromPath == this.file);

    return new Map(imports.map(imp => [...imp.bindings]).flat().sort((a,b) => a[0].localeCompare(b[0])));
  }

  static *getDeps(module) {
    for(let m of ES6Module.moduleList) {
      if(m === module) continue;
      if(m.importedFrom.file == module.file) yield m;
      else if((m.importedFrom + '').startsWith(module.file)) yield m;
    }
  }

  static *getImportExport(module, skipFn = idx => idx == -1) {
    for(let impExp of ES6ImportExport.importExportList) {
      if(skipFn(['require', 'import'].indexOf(impExp.type))) continue;
      if(impExp.file == module.file) yield impExp;
    }
  }

  static tree(colors = true) {
    const t = colors ? (...args) => ansi.text(...args) : t => t;
    let arr = [...ES6Module.moduleList];
    let modules = new Set(arr);
    let lines = [];
    function printModule(module, sym = '') {
      // if(!modules.has(module)) return;
      modules.delete(module);
      if(typeof module.file != 'string') {
        //console.log('module:', module);
        throw new Error();
      }
      lines.push(t(sym, 1, 36) + t(module.file, 1, 33));
      let deps = [...ES6Module.getDeps(module)];
      for(let i = 0; i < deps.length; i++) {
        //console.log('', { i, sym });
        printModule(deps[i], sym.replace(/\u251c\u2500/g, '\u2502 ').replace(/\u2514\u2500/g, '  ') + ((i + 1 == deps.length ? '\u2514' : '\u251c') + '\u2500') + ' ');
      }
    }
    printModule(arr[0], '');
    return lines.join('\n');
  }
}

const IMPORT = 1;
const EXPORT = 2;

class ES6Env {
  static getCwd = memoize(() => fs.realpathSync('.') || path.resolve('.'));
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

  static get list() {
    return [...ES6ImportExport.importExportList];
  }

  static create(obj) {
    let ret = new ES6ImportExport();
    let nodeClass = className(obj.node);
    let code = memoize(() => '' + decamelize(nodeClass) + ' ' + PrintAst(obj.node))();
    let p, n;
    p = obj.path.slice(0, 2);
    n = p.apply(obj.ast, true);
    code = PrintAst(n);
    if(obj.node instanceof ESNode) n = obj.node;
    else p = obj.node;
    let type = [/(import|require)/i, /exports?[^a-z]/i].filter(re => re.test(code));
    type = type.map(re => [...re.exec(code)]).map(([m]) => m);
    type = type.map(m => m + '');
    type = (Array.isArray(type) && type[0]) || type;
    let assoc = ESNode.assoc(obj.node) || ESNode.assoc(n);
    let position = ESNode.assoc(obj.node).position || ESNode.assoc(n).position;
    if(position) {
      position = position.clone();
      position.file = ES6Env.pathTransform(position.file);
    }
    let bindings = (obj.bindings instanceof Map && obj.bindings) || new Map();
    ret = Object.assign(ret, { type, bindings, position });
    let dir = path.relative(ES6Env.cwd, path.dirname(obj.file));
    ret = define(ret, { ...obj, nodeClass, dir });

    /*    bindings[inspectSymbol] = function() {
      return inspect(this);
    };*/
    obj.importNode = obj.importNode || [];

    let importNode = obj.importNode.filter(p => p instanceof Path);
    //console.log('importNode:', importNode);

    let importNodes = importNode
      //   .filter(p => p.length < 3)
      .map(p => deep.get(obj.ast, p))
      .map(n => [ESNode.assoc(n).position, PrintAst(n)])
      .map(([p, n]) => [isGenerator(position) && [...position].map(p => ES6Env.pathTransform(p)).join(':'), n]);

    if(isObject(position) && position.toString) position = position.toString(true, (p, i) => (i == 0 ? path.relative(ES6Env.cwd, p) : p)).replace(/1;33/, '1;34');
    /* const InspectFn = ret.bindings[inspectSymbol];
  console.log(ansi.text(ucfirst((type + '').toLowerCase()), 1, 31) + ansi.text(` @ `, 1, 36),
      InspectFn ? InspectFn.call(ret.bindings) : '',
      'importNode:',
      [...obj.importNode].map((n) => [node2path.get(n), PrintAst(n)]).filter(([p, c]) => c.trim() != ''),
      ...['dir', 'relpath'].reduce((acc, n) => [...acc, ansi.text(n, 38, 5, 197) + ansi.text(' ', 1, 36) + (typeof ret[n] == 'string' ? ansi.text(ret[n], 1, 32) : typeof InspectFn == 'function' ? InspectFn.call(ret) : inspect(ret[n], { multiline: false, newline: '' }).replace(/.*\)\ {/g, '')) + ' '], [])
    );*/
    Object.assign(ret, obj);
    if(ret.file) ret.module = ES6Module.getOrCreate(ret.file, null);
    if(ret.fromPath) ret.fromModule = ES6Module.getOrCreate(ret.fromPath, position || ret.file);

    ES6ImportExport.importExportList.add(ret);

    //if(ret.bindings instanceof Map) console.log("bindings:", [...ret.bindings]);

    return ret;
  }

  /* prettier-ignore */ get relpath() {
    const { file } = this;
    if(ES6Env.cwd && file) {
      let r = path.relative(ES6Env.cwd, file);
      if(!/\//.test(r)) r = './' + r;
      return r;
    }
  }

  /* prettier-ignore */ get fromBase() {
    return path
      .basename(this.relpath)
      .replace(/\.([0-9a-z]{2}|[0-9a-z]{3})$/, '');
  }

  /* prettier-ignore */ get from() {
    let { node, path } = this;
    let value = (node && node.source) || node;
    path = (value && path.down('source')) || path;
    while(isObject(value) && value.value) value = value.value;
    return [value, path];
  }

  /* prettier-ignore */ set from(value) {
    let { ast, node, path } = this;
    if(node.source) {
      let key = path.last;
      let parent = path.up();
      let obj = parent.apply(ast, true);
      obj[key] = new Literal(value);
    }
  }
  toSource() {
    return PrintAst(this.node);
  }
  /* prettier-ignore */ get code() {
    return this.toSource();
  }
  static get exports() {
    let ret = [];
    for(let impExp of ES6ImportExport.importExportList) {
      if(impExp.exportNode) {
        //console.log('impExp.bindings:', impExp.bindings);

        if(isIterable(impExp.bindings)) for(const entry of impExp.bindings) if (entry && entry.length) ret.push(entry);
      }
    }
    return accumulate(ret);
  }
  [inspectSymbol]() {
    let { type, position, bindings, path, node, file, from, fromPath, module, relpath } = this;
    const opts = { colors: true, colon: ': ', multiline: false, quote: '' };
    if(isIterator(position)) position = [...position].map(p => ES6Env.pathTransform(p)).join(':');

    return inspect(
      Object.assign(
        Object.setPrototypeOf(
          {
            type,
            module: module[inspectSymbol]().replace(/\n\s*/g, '\n  '),
            bindings: inspect(bindings, {
              colors: true,
              toString: 'toString'
            }).replaceAll('\n', '\n  ')
            /* path: path.join('.'),*/
            /*node: PrintAst(node) */ //inspect(node, { ...opts, separator: '', newline: '', depth: 0 })
          },
          ES6ImportExport.prototype
        ),
        { file /*, position */ }
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
    let obj = filterOutMembers(this, x => [isFunction /*, isObject*/].some(f => f(x)));
    return inspect({ ...obj, __proto__: proto }, { multiline: true });
  }
  toString() {
    //console.log('toString', ...Util.getMemberNames(this).map((p) => [p, this[p]]).map(([k, v]) => [k, v + '']).flat());
    return path.relative(ES6Env.cwd, this.file || '');
  }
  [Symbol.toStringTag]() {
    return path.relative(ES6Env.cwd, this.file || '');
  }
}

const isRequire = ([path, node]) => {
  //console.log("isRequire node:", node);

  return (node instanceof CallExpression || node.type == 'CallExpression') && (node.callee.name == 'require' || PrintAst(node.callee) == 'require');
};

const isImport = ([path, node]) => node instanceof ImportDeclaration;
const isES6Export = ([path, node]) => node?.type?.startsWith('Export');
/*const isCJSExport = ([path, node]) =>
  node instanceof MemberExpression &&   PrintAst(node).startsWith('module.exports');*/
const isCJSExport = ([path, node]) => node.left && PrintAst(node.left).startsWith('module.exports'); // node instanceof MemberExpression &&   PrintAst(node).startsWith('module.exports');

const getImport = ([p, n]) => {
  let r = [];
  if(n instanceof CallExpression && isObject(n) && n.callee && n.callee.name == 'require') {
    let idx = p.lastIndexOf('init');
    let start = idx != -1 ? idx + 1 : p.length;
    let end = p.length;
    let imp = [];
    imp.push(p.down('arguments', 0));
    imp.push(p.slice(0, idx));
    while(start < end) {
      imp.push(p.slice(0, start));
      start++;
    }
    r.push(imp);
  } /* if(!(n instanceof ImportDeclaration))*/ else {
    let source = p.concat(['source']);
    for(let [node, path] of deep.iterate(n, v => v instanceof ModuleSpecifier)) {
      const name = p.concat([...path, 'name']);
      const alias = p.concat([...path, 'as']);
      r.push([source, alias, name]);
    }
  }
  //console.log('getImport', { p, n, r });
  return r;
};
const getExport = ([p, n]) => [n instanceof ExportNamedDeclaration ? p : p.slice(0, 2), p]; /*.map(p => [...p])*/

function PrintCode(node) {
  return PrintObject(node, node => PrintAst(node));
}

function PrintObject(node, t = (n, p) => n) {
  return Object.entries(node)
    .map(([prop, value]) => [prop, ': ', t(value, prop)])
    .flat();
}

function PrefixRemover(reOrStr, replacement = '') {
  if(!(Array.isArray(reOrStr) || isIterable(reOrStr))) reOrStr = [reOrStr];

  return arg => reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), typeof arg == 'string' ? arg : '');
}

function DumpFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  WriteFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function PrintAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

const hl = {
  id: text => ansi.text(text, 1, 33),
  punct: text => ansi.text(text, 1, 36),
  str: text => ansi.text(text, 1, 32)
};

function DumpNode(node) {
  return [hl.id`path` + hl.punct`:`, node2path.get(node), ansi.text(`node`, 1, 33) + `:`, inspect(node, { multiline: false, depth: 4 })];
}

function GenerateFlatMap(ast, root = [], pred = (n, p) => true, t = (p, n) => [root.concat(p).join('.'), n]) {
  flat = deep.flatten(
    ast,
    new Map(),
    (n, p) => n instanceof ESNode && pred(n, p),
    (p, n) => t(p, n)
  );
  return flat;
}

function GenerateDistinctVariableDeclarations(variableDeclaration) {
  const { kind, declarations } = variableDeclaration;

  let ret = [];

  for(let decl of declarations) {
    ret.push(new VariableDeclaration([decl], kind));
  }
  return ret;
}

async function main(...args) {
  //console.log('process:',process);
  let consoleOpts;
  globalThis.console = new Console(
    (consoleOpts = {
      stdout: process.stdout,
      inspectOptions: { colors: true, depth: 1, compact: false, breakLength: 80 }
    })
  );
  console.options = consoleOpts;
  let params = getOpt(
    {
      output: [true, null, 'o'],
      debug: [false, null, 'x'],
      '@': 'input'
    },
    args
  );
  console.log('params', params);
  const re = {
    name: /^(process|readline)$/,
    path: /(lib\/util.js$|^lib\/)/
  };

  args = params['@'];
  if(args.length == 0) args = ['lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  console.log('args:', args);
  let r = [];
  let processed = [];
  let allImports = [];
  const dirs = [ES6Env.cwd, ...args.map(arg => path.dirname(arg))];
  search = MakeSearch(dirs);
  //console.log('search:', search);
  let optind = 0;

  while(args.length > 0) {
    let arg = args.shift();
    while(/:[0-9]+:?$/.test(arg)) arg = arg.replace(/:[0-9]*$/g, '');
    let ret;
    console.log('arg:', arg);
    if((ret = await processFile(/*ES6Env.pathTransform*/ arg)) < 0) return -ret;
    optind++;
  }

  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  console.log('processed files:', processed);
  DumpFile(`${name}.es`, r.join('\n'));

  function FriendlyPrintNode(n) {
    if(n.type)
      return Object.defineProperties([GetPosition(n), n.type || className(n), /*st ? st.pathOf(n) : */ undefined, abbreviate(escape(PrintAst(n))), GetName(n) /*|| Symbol.for('default')*/], {
        inspectSymbol: {
          value() {
            let arr = this.reduce((acc, item) => [...acc, (item[inspectSymbol] ?? item.toString).call(item)], []);
            let [pos, type, path, node, id] = arr;
            return (
              '{' +
              Object.entries({ pos, type, path, node, id })
                .map(([k, v]) => `\n    ${k}: ${v}`)
                .join('') +
              '\n}'
            );
            return arr.join('\n  ');
          }
        },
        id: {
          get() {
            if(n.id) return Identifier.string(n.id);
          }
        }
      });
    return n;
  }
  function FriendlyPrintNodes(nodes) {
    return Array.isArray(nodes) ? nodes.map(n => (Array.isArray(n) ? FriendlyPrintNodes(n) : FriendlyPrintNode(n))) : nodes;
  }

  console.log(`\nModules:\n\n  ` + ES6Module.tree().replaceAll('\n', '\n  '));
  console.log('exportMap:', exportMap);
  console.log('importMap(1):', importMap);

  function removeFile(file) {
    remove(args, file);
    pushUnique(processed, ES6Env.pathTransform(file));
  }

  async function processFile(file, depth = 0) {
    Verbose(`processFile[${depth}] '${file}'`);

    function Verbose(...args) {
      console.log(...[`${file}:`, ...args]);
    }

    let b, ret;
    if(!name) {
      name = file;
      if(/\/(src|index)[^/]*$/.test(name)) name = name.replace(/(src\/|index)[^/]*$/g, '');

      name = path.basename(name).replace(/\.[ce]?[tj]s$/, '');
    }
    const module = ES6Module.getOrCreate(file, `<command-line:${optind}>`);

    const moduleDir = removeModulesDir(file);
    const modulePath = path.relative(ES6Env.cwd, moduleDir);
    removeFile(file);

    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    let moduleImports, imports;
    Verbose('processing:', { file, thisdir });

    const result = await ParseFile(file);
    /// Verbose('result:', result);
    let { data, error, ast, parser, printer } = result;
    let flat, map;
    let st = new Tree(ast);

    let astStr = JSON.stringify(ast, null, 2);
    WriteFile(path.basename(file, /\.[^.]+$/) + '.ast.json', astStr);
    Verbose(`${file} parsed:`, { data, error });
    function generateFlatAndMap() {
      console.log('ast:', ast);
      flat = deep.flatten(ast, new Map(), node => typeof node == 'object' && node != null);
      console.log(
        'flat:',
        [...flat].map(([p, n]) => [p, n])
      );
      map = mapAdapter((key, value) =>
        key !== undefined
          ? value !== undefined
            ? value === null
              ? deep.unset(ast, [...key])
              : deep.set(ast, [...key], value)
            : deep.get(ast, [...key])
          : (function* () {
              for(let [key, value] of flat.entries()) {
                yield [key, deep.get(ast, key) /*?? key.apply(ast, true)*/];
              }
            })()
      );
      node2path = new WeakMap([...flat].map(([path, node]) => [node, path]));
    }

    try {
      const getRelative = filename => path.join(thisdir, filename);

      //Verbose('node2path:', node2path);

      const AddValue = (...args) => {
        let tmp, arg, subj;
        while(args.length > 0) {
          arg = args.shift();
          if(typeof arg == 'function') {
            subj = args.shift() || [];
            subj = Array.isArray(subj) ? subj : [subj];
            tmp = arg(value, ...subj);
          } else tmp = typeof arg == 'string' ? value[arg] : arg;
          Verbose('AddValue', (arg + '').replace(/\n.*/gm, ''), {
            value,
            tmp,
            list
          });
          if(!tmp) return;
          if(typeof tmp == 'boolean') tmp = value;
          if(tmp !== value) list.push(tmp);
          value = tmp;
          return value;
        }
      };
      let importNodes;
      let importValues;

      again: while(true) {
        generateFlatAndMap();

        //        moduleImports = [...flat];
        moduleImports = [...flat].filter(([p, n]) => {
          if(typeof n == 'object' && n != null) {
            let cl = className(n);
            if(cl == 'CallExpression') {
              let o = PrintAst(n);
              if(o.startsWith('require')) return true;
            }
            //          let p = st.pathOf(n);
            console.log('n:', p + '', cl);

            //          console.log("it:",{p,n}, className(n));
            if(isRequire([p, n])) return true;
            if(isImport([p, n])) return true;
          }

          return false;
        });
        console.log(
          'moduleImports:',
          moduleImports.map(([p, n]) => [p, PrintAst(n)])
        );

        function Source(source, file, node) {
          const obj = new String(source);
          return Object.defineProperties(obj, {
            path: {
              get: once(function () {
                let ret = SearchModuleInPath(source, file, ESNode.assoc(node).position) || source;
                delete this.path;
                this.path = ret;
                return ret;
              }),
              enumerable: true,
              configurable: true
            },
            valueOf: {
              value() {
                return this.path;
              }
            },
            toString: {
              value() {
                return this + '';
              }
            },
            [inspectSymbol]: {
              value() {
                let path = this.path;
                return this;
              }
            }
          });
        }
        //       Verbose(`flat:`, flat);
        Verbose(`moduleImports:`, moduleImports);
        let importEntries = moduleImports.map(([p, n]) => [n, GetImportBindings([n, p] /*, a=> a*/)]);
        //Verbose(`importEntries:`, importEntries);
        importEntries = new Map(
          importEntries.map(([node, [parent, bindings]]) => {
            //Verbose('bindings:', {n,bindings});
            bindings ??= [];
            bindings = [...bindings].map(([name, binding]) => [name, binding /*.concat([n])*/]);
            bindings = bindings.map(([name, [source, imp /*, node*/]]) => [name, [source, typeof imp == 'symbol' ? Symbol.keyFor(imp) : imp, node, parent]]);
            let assoc = ESNode.assoc(node) || ESNode.assoc(parent);

            let { position } = assoc;
            bindings = bindings.map(([name, [source, imp, node, parent]]) => [
              name,
              Object.defineProperties([source, imp == 'default' ? null : imp, node, st.pathOf(node), position && position.start], {
                from: {
                  get() {
                    return GetFrom(node)[0];
                  }
                },
                imported: {
                  get() {
                    const [, id] = this;
                    return typeof id == 'symbol' ? Symbol.keyFor(id) : id;
                  }
                },
                source: {
                  get() {
                    const [source] = this;
                    return new Source(source, file, node);
                  }
                },
                node: {
                  get() {
                    return this[2];
                  }
                },
                position: {
                  get() {
                    return this[3];
                  }
                },
                [inspectSymbol]: {
                  value() {
                    const [source, imported, node, path, position] = this;
                    //console.log("position", {file,line,column});
                    return [source, imported, PrintNode(node, path, position)].join(', ');
                  }
                }

                //  return PrintNodes(this);
                /* const { source, imported, node, position } = this;
                      return {
                      imported: imported ?? Symbol.for('default'), //  ...(imported ? { imported } : {}),
                        position,
                        source: source[inspectSymbol](),
                        node: abbreviate(escape(PrintAst(node)))
                      };*/
              })
            ]);

            return [node, new Map(bindings)];
          })
        );
        Verbose(`importEntries[${size(importEntries)}]:`, importEntries);

        importMap[file] = Object.fromEntries([...importEntries].reduce((acc, [n, bindings]) => /*(Array.isArray(bindings) ?*/ [...acc, ...bindings] /*: acc)*/, []));

        function PrintNode(node, path, position) {
          let type = node.type;
          let ctxt = colorText;
          if(type) {
            path ??= st.pathOf(node);
            position ??= ESNode.assoc(node).position;
            if(position?.start) position = position.start;
            let props = [ctxt(type, 38, 5, 214), path, position].map(a => (a && a[inspectSymbol] ? a[inspectSymbol]() : a));
            /* console.log("path:", [...path]);*/
            props[1] = `[ ${props[1]} ]`;
            //console.log('props:', props);
            const p = [38, 5, 124] || [1, 31];
            const b = [0, 37];
            props = props.reduce((acc, item) => (acc && acc.length ? [...acc, ctxt('\u066d', ...p), item] : [item]), []);
            return [ctxt('Node', 38, 5, 198), ctxt(`{`, ...b), ...props, ctxt(`}`, ...b)].reduce((acc, item) => (acc ? acc + ' ' + item : item), '');
          }
          return node;
        }
        function PrintNodes(nodes) {
          // console.log("PrintNodes",{nodes});
          return Array.isArray(nodes) ? nodes.map(n => (Array.isArray(n) ? PrintNodes(n) : PrintNode(n))) : PrintNode(nodes);
        }

        /*   for(let imp of moduleImports) {
          let expr = ParentExpr(imp);
          if(!(expr instanceof AssignmentExpression || expr instanceof VariableDeclarator)) {
            if(imp[0].length == 2) continue;
            let stmt = Path2Ptr(imp[0].slice(0, 2));
            let root = Path2Ptr(imp[0].slice(0, 1));
            let pos = imp[0][1];
            let source = GetFrom(imp[1], imp[0])[0];
            let name = source && camelize(path.basename(source));
            if(name) deep.set(ast, imp[0], new Identifier(name));
            else throw new Error(`From: ${imp[1].type} ${PrintAst(imp[1])}`);
            let importStatement = new ImportDeclaration([new ImportSpecifier(new Identifier('default'), new Identifier(name))],
              new Literal(source)
            );
            let assoc = filterKeys(ESNode.assoc(stmt[1]), ['range', 'comments', 'position']);
            ESNode.assocMap.set(importStatement, assoc);
            root[1].splice(pos, 0, importStatement);
            continue again;
          }
        }*/

        if(params.debug) {
          Verbose(`importMap[${size(importMap[file])}]:`, importMap[file]);
          // Verbose('importEntries:', importEntries);
        }

        function GetName(node) {
          // console.log('GetName', { node });
          /* if(node.left) return GetName(node.left);
          if(node.declaration) return GetName(node.declaration);
 
          */ if(node.property) {
            let str = PrintAst(node);
            if(/^module.exports/.test(str)) return str.replace(/^module\.exports\.?/, '');
          }

          if(node.id) {
            if(node.id.type == 'Identifier') return Identifier.string(node.id);
            //console.log("GetName",{type: node.type});
          }
          let parent = node.left || node.declaration; /*|| st.parentNode(node)*/

          if(parent) {
            return GetName(parent);
          } else if(node.type == 'Program') {
            return;
          }
          if(node.type == 'ExpressionStatement') {
            node = node.expression;

            if(node && node.type == 'AssignmentExpression') {
              if(node.right && node.right.type == 'FunctionDeclaration') {
                const { right } = node;
                console.log('GetName', { right });
                return Identifier.string(right.id);
              }
            }
          }

          //console.log('GetName', { type: node.type, node/*, oncode: PrintAst(node)*/ }); throw new Error(`GetName ${node.type}`);
        }

        function ExportName(exp) {
          if(exp.type) exp = PrintAst(exp).replace(/^module.exports\.?/, '') || Symbol.for('default');
          if(typeof exp == 'symbol') return Symbol.keyFor(exp);
          if(typeof exp == 'string') return exp;
          console.log('exp:', exp);
        }
        function ExportSymbol(exp) {
          if(exp.type) exp = PrintAst(exp).replace(/^module.exports\.?/, '') || Symbol.for('default');
          if(exp == 'default') exp = Symbol.for('default');
          return exp;
        }
        /* Verbose(`CJS exports:`,
          new Map(
            [...flat]
              .filter(e => isCJSExport(e))
              .map(([p, n]) => [p, GetPosition(n), n])
              .map(([p, pos, n]) => [p, pos, n.left, n.right])
              .map(([p, pos, exp, val]) => [
                p,
                pos,
                PrintAst(exp).replace(/^module.exports\.?/, '') || Symbol.for('default'),
                exp,
                val
              ])
              .map(([p, pos, e, ...nodes]) => [e, ...nodes.slice(1, 2)])
              .map(([p, ...nodes]) => [p, ...FriendlyPrintNodes(nodes)])
          )
        );*/
        let exportNodes = [...flat].filter(e => isCJSExport(e)).map(([p, n]) => [n.left, n.right, n, ...st.anchestors(n)]);

        exportNodes = exportNodes
          //  .map(e => [...e, GetExportBindings(ast, e)])
          .map(([exp, ...nodes]) => [
            ExportSymbol(exp),
            nodes
              .reduce((acc, n) => (n.id && [...acc, n.id]) || acc, [])
              .map(id => Identifier.string(id))
              .find(n => typeof n == 'string'),
            ...nodes
          ]);

        if(params.debug) Verbose(`CJS exports 1:`, exportNodes);
        exportNodes = exportNodes.map(([exp, name, ...nodes]) => [ExportSymbol(exp), name, ...nodes]);
        exportMap[file] = Object.fromEntries(
          exportNodes.map(([exp, name, ...nodes]) => [
            exp,
            /*FriendlyPrintNodes*/ nodes /*.slice(0,2)*/

              /*.map(n => [st.pathOf(n), n])
                .reduce((acc, [p, n]) => [
                    ...acc,
                    n,
                    ...(p[p.length - 1] == 'right'
                      ? [ st.parentNode(n).left]
                      : [])
                  ],
                  []
                )*/
              .map(n => [n, GetName(n) || [n.type]])
              .map(([n, name]) => [FriendlyPrintNode(n), name /* || Symbol.for('default')*/])
              .find(([n, name]) => name !== null && name !== undefined)
          ])
        );

        Verbose(`CJS exports 2:`, exportMap[file]);

        //Verbose('imports:', imports);
        importNodes = moduleImports.map(it => getImport(it));
        //Verbose('importNodes:', importNodes);

        importValues = moduleImports.map(it => {
          let importIds = getImport(it);
          //Verbose('importIds:', importIds);
          let acc = [];
          for(let importId of importIds) {
            let ret = [];

            for(let [p, n] of importId.map(p => [p, deep.get(ast, p)])) {
              if(n instanceof Token) n = n.value;
              if(n instanceof MemberExpression) n = n.property;
              if(n instanceof VariableDeclarator) n = n.id;
              /* if(n instanceof Literal) n = Literal.string(n);
              if(n instanceof Identifier) n = n.value;*/
              ret.push([p, n]);
            }
            if(ret.length == 2) ret.push([null, new Identifier('default')]);

            acc.push(ret);
          }
          return acc;
        }, []);
        if(params.debug) Verbose('moduleImports:', moduleImports);

        if(moduleImports.length) {
          //Verbose('ast.body:', ast.body);
          let importStatements = moduleImports.map(([, imp]) => imp);
          //Verbose('importStatements:', importStatements);
          let declPaths = unique(importStatements.map(stmt => stmt.toString()));

          let declStatements = declPaths
            .map(p => p.split(/\./g))
            //  .map(p => new ImmutablePath(p))
            .map(p => [p, deep.get(ast, p)])
            .filter(([path, stmt]) => stmt instanceof VariableDeclaration);

          let declCounts = Math.max(...declStatements.map(([p, n]) => n.declarations.length));

          if(declCounts > 1 && declStatements.length > 0) {
            for(let [path, stmt] of declStatements.reverse()) {
              let index = path.last;
              let arr = deep.get(ast, path.up());
              let decls = GenerateDistinctVariableDeclarations(stmt);

              arr.splice(index, 1, ...decls);
            }
            continue;
          }
        }
        break;
      }

      const useStrict = map.filter(([key, node]) => node instanceof Literal && /use strict/.test(node.value));
      useStrict.forEach(([path, node]) => deep.unset(ast, path));
      // Verbose('importNodes:', importNodes);
      let importDeclarations = importValues.map(ids =>
        ids.map(a =>
          a /*.slice(1)*/
            .map(([p, n]) => n)
        )
      );

      //Verbose('moduleImports:', moduleImports);

      imports = moduleImports.map(([path, node]) => [path, node, Literal.string(GetLiteral(node))]).filter(([path, node, module]) => !re.name.test(module));

      //Verbose('imports:', imports);

      imports = imports.map(([path, node], i) => {
        const assoc = ESNode.assoc(node);
        //Verbose(`imports[${i}]`, { path, node, assoc });

        const { position } = assoc;
        const fromPath = GetFromPath([path, node], position || file);

        //   const decls = importDeclarations[i].map(a => a.slice(1).map(n => new Identifier(Literal.string(n))));
        const importNode = imports[i];

        //Verbose('ES6ImportExport:', { path, node, fromPath, decls, importNode });

        return ES6ImportExport.create({
          ast,
          node,
          importNode,
          path,
          file,
          identifiers: node.right instanceof Identifier ? [node.right.value] : [],
          position,
          fromValue: GetFromValue([node, path]),
          fromPath,
          bindings: GetImportBindings([node, path])
        });
      });

      let statement2module = imports.map(imp => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);

      //Verbose(`imports YYY:`, imports.map(imp => [imp.path, imp.position || { file: imp.file }]));

      const fromMap = new WeakMap(
        imports.map((imp, idx) => [
          imp.node,
          tryCatch(
            () => imp.fromPath,
            v => v,
            () => Literal.string(GetLiteral(imp.node))
          )
        ])
      );

      //Verbose(`imports XXX:`, imports.map(imp => fromMap.get(imp.node)));

      let remove = imports.filter(imp => fromMap.get(imp.node)).map((imp, idx) => [idx, imp]);

      remove = remove.filter(([idx, imp]) => !IsBuiltinModule(imp.fromPath));
      //Verbose(`remove:`, remove.map(([idx, imp]) => imp.fromPath));
      /*  remove.forEach(([i, imp]) => {
        let nodes = importNodes[i];
        let { path, node } = imp;
         deep.set(ast, [...path], new ExpressionStatement(new Literal(`"removed ${PrintAst(node).replace(/;$/, '')}"`)));
      });*/

      let recurseImports = unique(remove.map(([idx, imp]) => imp || imports[idx]));
      let recursePaths = recurseImports.map(imp => [ES6Env.cwd, GetFromPath([imp.path, imp.node], file), imp.node]);
      //Verbose(`recursePaths [${depth}]:`, recursePaths.map(p => p[1]));
      let recurseFiles = recursePaths.map(paths => path.relative(...paths.slice(0, 2)));

      //Verbose(`recurseFiles [${depth}]:`, recurseFiles);
      recurseFiles = recurseFiles.filter(f => processed.indexOf(f) == -1);
      recurseFiles = recurseFiles.filter(f => !re.path.test(f));

      imports = imports.filter(({ file, ...module }) => !re.path.test(file));

      if(recurseFiles.length > 0) {
        //Verbose(`recurseFiles [${depth}] `, recurseFiles.length, recurseFiles);
        //Verbose(`${ansi.text(modulePath, 1, 36)}: recurseFiles [${depth}]:`, recurseFiles.map((f) => f));
        let idx = 0;
        for(let imp of recurseFiles) {
          if(processed.indexOf(imp) == -1) {
            //Verbose(`recurseFiles [${depth}] forEach #${idx}:`, imp);
            await processFile(imp, (depth || 0) + 1);
          }
          idx++;
        }
      }

      let exportNodes = [...flat].filter(entry => isCJSExport(entry) || isES6Export(entry));
      exportNodes = exportNodes.map(([p, n]) => (n instanceof MemberExpression ? p.slice(0, -1) : p)).map(p => [p, deep.get(ast, [...p])]);

      /*      let exportStatements = exportNodes.map(([p, n]) => [
        p,
        typeOf(n),
        ...st.filter(n, n => n instanceof ESNode && PrintAst(n) == 'module.exports')
      ]);

      if(params.debug) Verbose('exportStatements:', exportStatements);
*/
      // Verbose('exportNodes:', exportNodes);
      let exportPaths = exportNodes.map(getExport);
      //Verbose('exportPaths:', exportPaths);
      let exportEntries = exportPaths
        .map(([path, path2]) => [path, path2, deep.get(ast, [...path])])
        .map(([p, p2, n]) => [p, p2, n, n instanceof ExpressionStatement ? n.expression : n])
        .map(([p, p2, n]) => [p, p2, n, n instanceof AssignmentExpression ? n.right : n]);

      let moduleExports = exportEntries.map(([path, path2, node, node2]) => [node != node2 ? 'default' : null, node2]);
      let exportInstances = moduleExports.map(([name, node], i) =>
        ES6ImportExport.create({
          ast,
          node,
          exportNode: exportEntries[i][2],
          path: exportEntries[i][0],
          file,
          identifiers: [name],
          bindings: GetExportBindings(node)
        })
      );

      let output = '';

      //  Verbose('exports:',[...ES6ImportExport.exports].map(([name,list]) => [name,list.length]));
      //      Verbose('exports > 1:', [...ES6ImportExport.exports].filter(([name, list]) => list.length > 1));
      let e = ES6ImportExport.exports;
      [...e].filter(([name, list]) => list.length > 1 && name != 'default').map(([name, list]) => list.slice(1).map(node => st.remove(node)));
      let defaultExports = [...flat].filter(([p, n]) => n instanceof ExportDefaultDeclaration);
      //[1].map(node => st.replace(node, node.declaration));

      defaultExports.forEach(([p, n]) => st.replace(n, n.declaration));
      //      Verbose('defaultExports:', defaultExports);

      //moduleImports.forEach(([p, n]) => st.remove(n));
      moduleImports.forEach(([p, n]) => st.removeAt(p));

      output = PrintAst(ast, parser.comments, printer);
      r.push(`/*\n * concatenated ${file}\n */\n\n${output}\n`);

      function GetImportBindings([node, path], retMap = /*(arg => arg) ||*/ arg => new Map(arg)) {
        if(node instanceof ImportDeclaration) {
          console.log('specifiers:', node.specifiers);
          return [
            node,
            new Map(
              node.specifiers.map(({ local, imported }) => [
                Identifier.string(local),
                [
                  Literal.string(node.source),

                  ifThenElse(
                    str => str == 'default',
                    () => Symbol.for('default'),
                    s => s
                  )(imported)
                  //, src => src == 'default' ? Symbol.for('default') : src , Identifier.string(imported)]])
                ]
              ])
            )
          ];
        }

        //console.log("parent:", path.split('.').slice(0, -1).join('.'), path);
        let code = PrintAst(node);

        console.log('code:', code);
        if(node instanceof CallExpression || node.type == 'CallExpression' || code.startsWith('require(')) {
          let source = (node.arguments[0] && Literal.string(node.arguments[0])) || null;
          let parentPath = path.split('.').slice(0, -1).join('.');
          console.log('parentPath:', parentPath);
          let parent = /*deep.get(ast, parentPath); // ??*/ flat.get(parentPath);

          let name = Symbol.for('default');

          if(parent instanceof MemberExpression) {
            name = PrintAst(parent.property);
          }

          while(!parent.id) {
            parentPath = parentPath.split('.').slice(0, -1).join('.');

            parent = deep.get(ast, parentPath); // st.parentNode(parent);
          }
          console.log('parent:', parent);

          if('id' in parent) {
            //console.log("parent.id:", parent.id);
            //console.log("parent.id.value:", Identifier.string(parent.id));
            if(typeof Identifier.string(parent.id) == 'string') return [parent, retMap([[Identifier.string(parent.id) || Symbol.for('default'), [source, name]]])];

            if(parent.id.properties) {
              let { properties } = parent.id;
              parent.id.properties[0].key = new Identifier('XXX');

              console.log('GetImportBindings:', PrintAst(parent.id));

              return [parent, retMap(properties.map(({ key, value }) => [Identifier.string(key), [source, Identifier.string(value)]]))];
            }
          }
        }
        console.log('GetImportBindings:', node, node.callee, node.arguments);
        throw new Error('Unhandled import bindings');
      }

      function ParentNode([path, node]) {
        //console.log("ParentNode", { path,node });
        if(path.length > 0) {
          path = path.slice(0, -1);
          node = deep.get(ast, path);
          return [path, node];
        }
        return [null, null];
      }

      function Path2Ptr(path) {
        return [[...path], deep.get(ast, [...path])];
      }

      function IsPtr(arg) {
        return Array.isArray(arg) && arg.length == 2 && Array.isArray(arg[0]);
      }

      function ParentExpr(ptr) {
        //console.log("ParentExpr", ptr);
        if(ptr && ptr[0]?.last == 'callee') ptr[0] = ptr[0].up();

        if(isRequire(ptr)) {
          ptr = ParentNode(ptr);
          if(ptr[1] instanceof MemberExpression) ptr = ParentNode(ptr);
          return ptr;
        }
      }
      /*function GetExportBindings(node) {
        let ret = GetExportBindingsX(node);
        Verbose("exportBindings:", ret, node);
        return ret;
      }*/
      function GetExportBindings(node) {
        if(node instanceof ExportNamedDeclaration) {
          if(node.declaration) {
            const { declaration } = node;
            if(declaration.id) {
              const { id } = declaration;
              return new Map([[Identifier.string(id), node]]);
            }
            if(declaration.declarations) {
              //   Verbose("declarations:", declaration.declarations);
              return new Map(
                declaration.declarations.map(({ id, init }) => {
                  if(id instanceof ArrayPattern) {
                    const ids = id.elements.map(prop => Identifier.string(prop.key));
                    const values = init.elements;

                    let ret = zip([ids, values]);
                    Verbose('ret:', ret);
                    return ret;
                  }
                  return [Identifier.string(id), init];
                })
              );
            }
          }

          if(node.specifiers) {
            //Verbose("node.specifiers:", node.specifiers);
            return new Map(node.specifiers.map(spec => [Identifier.string(spec.exported), spec]));
          }
          //  return new Map(node.specifiers.map(({ local, imported }) => [local, imported].map(Identifier.string)));1
        } else if(node instanceof ExportDefaultDeclaration) {
          return new Map([['default', node]]);
        }

        if(node instanceof ExpressionStatement) node = node.expression;
        let p = st.pathOf(node).join('.');
        let type = typeOf(node);
        //       Verbose('GetExportBindings:', {p,type});
        if(node instanceof AssignmentExpression) {
          const { left, right } = node;

          if(left instanceof MemberExpression && PrintAst(left).startsWith('module.exports')) return new Map([['default', right]]);
        }
        let children = [...deep.iterate(node, n => n && n instanceof ESNode)];
        children = children.filter(([n, p]) => n instanceof MemberExpression && /^module\.exports/.test(PrintAst(n)));
        children = children.map(([n, p]) => [st.parentNode(n), p.slice(0, -1)]).filter(([n, p]) => n instanceof AssignmentExpression);
        let bindings = children
          .filter(([n, p]) => p[p.length - 1] == 'left')
          .map(([n, p]) => [p, GetPosition(n), escape(/*abbreviate*/ PrintAst(n)), n, new Map([...st.anchestors(n, n => n.type && n.type != 'Program' && [st.pathOf(n), PrintAst(n)])])]);
        if(size(bindings))
          Verbose(
            `GetExportBindings:`,
            new Map(
              bindings.map(([p, pos, code, n, ...anchestors], i) => {
                return [i, { path: p, pos, code, node: n, anchestors }];
              })
            )
          );

        return bindings;

        //    Verbose('GetExportBindings:', { children});
        //q;

        throw new Error(`Unhandled export bindings: ${p} ${typeOf(node)} ${PrintAst(node)}`);
      }
    } catch(err) {
      putError(err);

      throw err;
    }
  }
}

function FdReader(fd, bufferSize = 1024) {
  let buf = fs.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await fs.waitRead(fd);
      ret = fs.read(fd, buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(fs.bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    fs.close(fd);
  });
}

async function Prettier(file) {
  let input = fs.open(file, 'r');
  let proc = childProcess('sh', ['-c', `node_modules/.bin/prettier --config .prettierrc --parser babel <'${file}' | tee '${file}.prettier'`], {
    block: false,
    stdio: [input, 'pipe', 'pipe']
  });
  let sink = RepeaterSink(wr => proc.stdio[1].pipe(wr));
  let data = '';
  for await(let r of await sink) data += r;
  return data;
}

async function ParseFile(file) {
  let data, error, ast, flat;
  try {
    data = ReadFile(file);

    //data = await Prettier(file);
    // console.log('data:', abbreviate(escape(data + ''), 40));
    //   console.log('data:', data.length, data || abbreviate(escape(data + ''), 40));

    ECMAScriptParser.instrumentate();
    const debug = await getEnv('DEBUG');
    console.log('FILE:', file);
    console.log('DEBUG:', debug);
    console.log('DATA:', abbreviate(data));
    parser = new ECMAScriptParser(data.toString(), file, debug ?? false);
    g.parser = parser;
    ast = parser.parseProgram();
    parser.addCommentsToNodes(ast);
  } catch(err) {
    console.log('err:', err);
    error = err;
  } finally {
    if(!ast) if (error === undefined) error = new Error(`No ast for file '${file}'`);
  }
  if(error) throw error;
  printer = new Printer({ indent: 4 });
  g.printer = printer;
  return { data, error, ast, parser, printer };
}

function GetPosition(node) {
  let assoc = ESNode.assoc(node);

  let position = (assoc ?? {}).position?.start;
  if(!position) {
    const nodesWithPosition = [...deep.iterate(node, (v, p) => v && v instanceof ESNode)].map(([n, p]) => [typeOf(n), ESNode.assoc(n).position, n]).filter(([type, pos, n]) => pos);
    let [posNode] = nodesWithPosition;
    let a = PrintAst(node),
      b = PrintAst(posNode[2]);
    if(a.startsWith(b)) position = posNode[1];
    //console.log('GetPosition', {a,b})
  }
  if(position.start) position = position.start;

  //   if(typeOf(position) == 'Location')  return [...position].slice(0,2).join(':');
  return position;
}

function GetFile(module, position) {
  let r;
  let file = isObject(position) && typeof position.file == 'string' ? position.file : position;
  //if(position instanceof Range) position = position.start;
  // console.log('GetFile', { module, position, file }, className(position));
  module = module.replace(/\?.*/g, '');
  if(module.startsWith('.') && typeof file == 'string' && !path.isAbsolute(module)) module = path.join(path.dirname(file), module);
  try {
    if(!fs.exists(module)) {
      for(let name of MakeNames(module)) {
        if(fs.exists(name)) {
          module = name;
          break;
        }
      }
    }
    if(fs.exists(module)) r = module;
    else r = SearchModuleInPath(module, file, position);
  } catch(err) {
    console.log(`GetFile(`, module, ', ', position, `) =`, err);
    throw err;
  }
  return r;
}

function GetFromValue(...args) {
  if(args[0] == undefined) args.shift();
  if(Array.isArray(args) && args.length == 1) args = args[0];
  let [p, n] = args[0] instanceof ESNode ? args.reverse() : args;
  if(!p || (!('length' in p) && typeof p != 'string')) throw new Error('No path:' + p + ' node:' + PrintAst(n));
  if(!n || !(n instanceof ESNode)) throw new Error('No node:' + n + ' path:' + p);
  if(!(n instanceof ESNode)) n = deep.get(ast, n);
  let pathStr = p.join('.');
  let flat = GenerateFlatMap(
    n,
    p,
    (n, p) => true || Array.isArray(n) || [ExportNamedDeclaration, ImportDeclaration, ObjectPattern, Literal].some(ctor => n instanceof ctor),
    (p, n) => [
      p,
      Object.setPrototypeOf(
        {
          ...filterKeys(n, k => n instanceof CallExpression || (k != 'type' && !(isObject(n[k]) || isFunction(n[k]))))
        },
        Object.getPrototypeOf(n)
      )
    ]
  );
  let literals = [...flat.entries()].filter(([p, n]) => n instanceof Literal);
  let paths = literals.map(([p, n]) => flat.get(p).value);
  if(paths) {
    let v = paths.map(n => n.replace(/^[\u0027\u0022\u0060](.*)[\u0027\u0022\u0060]$/g, '$1'))[0];
    if(v) {
      return v;
    }
  }
}

function GetFrom(node, path) {
  if(node instanceof CallExpression) {
    node = node.arguments[0];
    if(path) path = path.down('arguments', 0);
  }
  if(node instanceof ImportDeclaration) {
    node = node.source;
    if(path) path = path.down('source');
  }
  if(node instanceof Literal) {
    node = Literal.string(node);
    if(path) path = path.down('value');
  }
  return [node, path];
}

function GetFromBase(path, node) {
  //console.log('GetFromBase  : ', { path, node });
  let str = GetFromValue([node, path]);
  if(typeof str == 'string') str = str.replace(/^\.\//, '');
  return str;
}

function GetLiteral(node) {
  return (deep.find(node, n => n instanceof Literal) || {}).value;
}

function IsBuiltinModule(name) {
  return /^(std|os|ffi|net|_http_agent|_http_client|_http_common|_http_incoming|_http_outgoing|_http_server|_stream_duplex|_stream_passthrough|_stream_readable|_stream_transform|_stream_wrap|_tls_common|_tls_wrap|assert|async_hooks|buffer|child_process|cluster|console|constants|crypto|dgram|dns|domain|events|fs|http|http2|https|inspector|module|net|os|path|perf_hooks|process|punycode|querystring|readline|repl|stream|string_decoder|timers|tls|trace_events|tty|url|util|v8|vm|worker_threads|zlib)$/.test(
    name
  );
}

function GetFromPath([path, node], position) {
  //console.log('GetFromPath', { path, node, position });
  let fileName,
    fromStr = GetFromValue([node, path]);

  if(IsBuiltinModule(fromStr)) return fromStr;

  if(typeof fromStr == 'string') fileName = GetFile(fromStr, position);
  return fileName;
}

function GetBase(filename) {
  return path.basename(filename, /\.[^.]+$/);
}

function ReaddirRecursive(dir) {
  let ret = [];
  for(let entry of fs.readdir(dir)) {
    if(entry == '.' || entry == '..') continue;
    let file = path.join(dir, entry);
    if(fs.stat(file).isDirectory()) {
      ret = ret.concat(ReaddirRecursive(file));
      continue;
    }
    ret.push(entry);
  }
  return ret;
}

function Finish(err) {
  let fail = !!err;
  if(fail) {
    err.stack = PathReplacer()('' + err.stack)
      .split(/\n/g)
      .filter(s => !/esfactory/.test(s))
      .join('\n');
  }
  if(err) {
    console.log(parser.lexer.currentLine());
    console.log(className(err) + ': ' + (err.msg || err) + '\n', err.stack);
  }
  let lexer = parser.lexer;
  let t = [];
  //console.log(parser.trace() );
  DumpFile('trace.log', parser.trace());
  if(fail) {
    console.log('\nerror:', err.msg, '\n', parser.lexer.currentLine());
  }
  console.log('Finish: ' + (fail ? 'error' : 'success'));
  return !fail;
}

function MakeSearchPath(dirs, extra = 'node_modules') {
  let r = [];
  const { cwd } = ES6Env;
  const addPath = p => {
    p = path.relative(cwd, p, cwd);
    if(r.indexOf(p) == -1) r.unshift(p);
  };
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      const extra_dir = path.join(dir, extra);
      if(extra == 'node_modules') if (i == 0) addPath(dir);
      if(fs.existsSync(extra_dir)) addPath(extra_dir);
      i++;
      parts.pop();
    }
    if(extra == 'node_modules') i = 0;
  }
  return r;
}

function CheckExists(path) {
  let r = fs.exists(path);
  //console.log(`CheckExists('${path}') = ${r}`);
  return r;
}

function HasExtension(path) {
  return !/^[^.]*$/.test(path.basename(path));
}

function GetMain(dir) {
  let pkg;
  let name = path.basename(dir);
  let names = ['src/index', 'src/' + name, 'dist/' + name, 'dist/index', 'build/' + name, name, 'index', 'browser'];
  if((pkg = GetPackage(dir))) {
    let main = pkg.source || pkg.module || pkg.main;
    names.unshift(main);
  }
  // console.log("GetMain",{dir,name,names});
  let indexes = names.reduce((arr, name) => (name ? [...arr, ...MakeNames(name)] : arr), []);
  //console.log("GetMain",{indexes});
  for(let index of indexes) {
    let file = path.join(dir, index);
    if(fs.exists(file)) return file;
  }
}

function FileOrDirectory(relpath, callback = name => {}) {
  let names = MakeNames(relpath);
  let index = names.findIndex(name => fs.exists(name));

  if(index != -1) callback(names[index]);
  return index;
}

function FindModule(relpath) {
  //console.log('FindModule', { relpath });

  FileOrDirectory(relpath, name => (relpath = name));

  let st = fs.stat(relpath);
  let module;
  const name = path.basename(relpath);
  let pkg = GetPackage(relpath);

  if(st.isDirectory()) {
    module = GetMain(relpath);
    //console.log('FindModule:', { relpath, module });

    if(!CheckExists(module)) {
      module = null;
      let entries = ReaddirRecursive(relpath).filter(name => /\.js$/.test(name));
      if(entries.length == 1) module = path.join(relpath, entries[0]);
    }
  } else if(st.isFile()) {
    module = relpath;
  }
  if(!module) throw new Error(`Module '${relpath}' not found`);
  return module;
}

const SearchPathGenerator = (pathList = search.path) =>
  function* (module) {
    for(let dir of pathList) yield path.join(dir, module);
  };

const SearchExtsGenerator = (pathGen, extList = searchExts) =>
  function* (module) {
    for(let file of pathGen(module)) for (let ext of extList) yield file + ext;
  };

function SearchModuleInPath(name, _from, position) {
  // console.log('SearchModuleInPath', { name, _from, position });

  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
  const isRelative = /^\.\.?\//.test(name);
  name = name.replace(/\..?js$/g, '');
  if(search.aliases.has(name)) return search.aliases.get(name);
  let names = [name, ...MakeNames(name)];
  let searchDirs = isRelative ? [thisdir] : [thisdir, ...search.path];
  for(let dir of searchDirs) {
    let searchFor = dir.endsWith('node_modules') && !/\//.test(name) ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      let p;
      if(fs.exists(modPath)) {
        let st = fs.stat(modPath);
        if(st.isDirectory()) if (IsPackage(modPath)) AddPackage(modPath);
        if((p = FindModule(modPath))) return p;
      }
      if(!/\.js$/.test(modPath)) {
        modPath += '.js';
        if(fs.exists(modPath)) if ((p = FindModule(modPath))) return p;
      }
    }
  }
  let fromModule = ES6Module.get(_from);
  if(!fromModule) throw new Error(`Module "${_from}" not found (${name})`, name);
  let chain = fromModule.chain;
  throw new URIError(`Module '${name}' imported from '${_from}' not found ` + inspect({ name, chain }, { multiline: false }), name);
}

function RemoveStatements(ast, statements, predicate = stmt => true) {
  let removed = [];
  for(let [path, node] of statements) {
    if(!predicate(node, path)) continue;
    deep.unset(ast, path);
    removed.push(node);
  }
  return removed;
}

function GetBindings(properties) {
  return properties.filter(node => node).map(node => (node.id ? [node.id.value, node.value.value] : [node, node]));
}

function ReduceNode([n, p]) {
  if(n instanceof Token) {
    n = n.value;
    p = p.down('value');
  }
  if(n instanceof MemberExpression) {
    n = n.property;
    p = p.down('property');
  }
  if(n instanceof VariableDeclarator) {
    n = n.id;
    p = p.down('id');
  }
  if(n instanceof Literal) {
    n = Literal.string(n);
    p = p.down('value');
  }
  if(n instanceof Identifier) {
    n = n.value;
    p = p.down('value');
  }
  return [n, p];
}

function GetDeclarations(ast, paths) {
  return paths.map(plist => plist.map(p => ReduceNode([p, deep.get(ast, p)])));

  /* let acc = [];
  let [path, path2] = paths.flat();
  // console.log('GetDeclarations', {path, path2});
  path = path.slice(0, path.indexOf('init'));
  let node = path.apply(ast) || deep.get(ast, [...path]);
  //console.log('GetDeclarations:', {node});
  // console.log('node:', node);
  if(node.id && node.init) return [[node.id, node.init]];
  if(node.identifiers) node = node.identifiers;
  if(node.declarations)
    acc = node.declarations.reduce((acc, decl) => {
      if(decl.id) decl = decl.id;
      if(decl.properties) acc = acc.concat(GetBindings(decl.properties));
      if(decl.value) acc = acc.concat([[Symbol.for('default'), decl.value]]);
      return acc;
    }, acc);
  if(node.properties) acc = acc.concat(GetBindings(node.properties));
  return acc;*/
}

function MakeNames(prefix) {
  if(/\.(njs|es6.js|esm.js|module.js|module.ejs|js|mjs)$/.test(prefix)) return [prefix];
  return [prefix + '.njs', prefix + '.es6.js', prefix + '.esm.js', prefix + '.module.js', prefix + '.module.ejs', prefix + '.js', prefix + '.mjs', prefix];
}

function IsPackage(dir) {
  if(packages.has(dir)) return true;
  const packageFile = path.join(dir, 'package.json');
  return fs.exists(packageFile);
}

function AddPackage(dir) {
  const packageFile = path.join(dir, 'package.json');
  const obj = JSON.parse(ReadFile(packageFile));
  packages.set(dir, obj);
  return obj;
}

function GetPackage(dir) {
  if(packages.has(dir)) return packages.get(dir);
  if(IsPackage(dir)) return AddPackage(dir);
}

function ParentPackage(path) {
  let ret;
  for(let dir of [...ParentDirs(path)].reverse()) {
    //console.log("ParentPackage", {dir}, packages.keys());
    let pkg = GetPackage(dir);
    if(pkg) ret = dir;
  }
  return ret;
}

function* ParentDirs(dir) {
  let prev;
  while(dir && dir != prev) {
    yield dir;
    prev = dir;
    dir = path.dirname(dir);
  }
}

function MakeSearch(dirs) {
  let o = {
    path: MakeSearchPath(dirs),
    packages: [] //MakeSearchPath(dirs, 'package.json')
  };
  o.aliases = o.packages.reduce((acc, p) => {
    let json = JSON.parse(ReadFile(p));
    let aliases = json._moduleAliases || {};
    for(let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      if(!fs.exists(module)) throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      let file = FindModule(module);
      let st = fs.stat(file);
      acc.set(alias, file);
    }
    return acc;
  }, new Map());
  return o;
}

main(...scriptArgs.slice(1));