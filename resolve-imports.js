import { ECMAScriptParser } from './lib/ecmascript.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './consoleSetup.js';
import Lexer, { PathReplacer } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import estree, { VariableDeclaration, ImportStatement, ExportStatement, Identifier, MemberExpression, ESNode, CallExpression, ObjectBindingPattern, Literal, AssignmentExpression } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { ImmutablePath, Path } from './lib/json.js';
import deep from './lib/deep.js';

let filesystem, searchPath, packagesPath, moduleAliases, files;
let node2path, flat, value, list;
const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);
let name;
let parser, printer;
const g = Util.getGlobalObject();
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

class ES6Module {
  impExpList = [];
  importedFrom = null;
  root = null;

  static moduleList = new Set();

  static create(file, from, position) {
    let mod = new ES6Module();
    mod.file = file;
    if(from) mod.importedFrom = position ? position : from;
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

  [inspectSymbol]() {
    const t = (...args) => Util.ansi.text(...args);
    let s = t(Util.className(this), 1, 31) + t(' {', 1, 36);
    for(let prop of ['file', 'importedFrom', 'importedPosition', 'deps']) if(this[prop]) s += '\n' + t(prop.padStart(13, ' '), 1, 33) + t(': ', 1, 36) + formatProp(this[prop]);
    s += t('\n}', 1, 36);
    function formatProp(value) {
      if(Util.isArray(value)) return t('[ ', 1, 36) + value.map((mod) => formatProp(mod.file)).join(t(', ', 1, 36)) + t(' ]', 1, 36);
      return typeof value == 'string' ? t(value, 1, 32) : value;
    }
    return s;
  }

  get deps() {
    return [...ES6Module.getDeps(this)];
  }

  get imports() {
    return [...ES6Module.getImportExport(this, (i) => i == -1)];
  }

  get exports() {
    return [...ES6Module.getImportExport(this, (i) => i != -1)];
  }

  static *getDeps(module) {
    for(let m of ES6Module.moduleList) {
      if(m === module) continue;
      if(m.importedFrom.file == module.file) yield m;
      else if((m.importedFrom + '').startsWith(module.file)) yield m;
    }
  }

  static *getImportExport(module, skipFn = (idx) => idx == -1) {
    for(let impExp of ES6ImportExport.importExportList) {
      if(skipFn(['require', 'import'].indexOf(impExp.type))) continue;
      if(impExp.file == module.file) yield impExp;
    }
  }

  static tree(colors = true) {
    const t = colors ? (...args) => Util.ansi.text(...args) : (t) => t;
    let arr = [...ES6Module.moduleList];
    let modules = new Set(arr);
    let lines = [];
    function printModule(module, sym = '') {
      // if(!modules.has(module)) return;
      modules.delete(module);
      if(typeof module.file != 'string') {
        //console.debug('module:', module);
        throw new Error();
      }
      lines.push(t(sym, 1, 36) + t(path.basename(module.file), 1, 33));
      let deps = [...ES6Module.getDeps(module)];
      for(let i = 0; i < deps.length; i++) printModule(deps[i], sym.replace(/\u251c\u2500/g, '\u2502 ') + ((i + 1 == deps.length ? '\u2514' : '\u251c') + '\u2500') + ' ');
    }
    printModule(arr[0], '');
    return lines.join('\n');
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

  static get list() {
    return [...ES6ImportExport.importExportList];
  }

  static create(obj) {
    let ret = new ES6ImportExport();
    let nodeClass = Util.className(obj.node);
    let code = Util.memoize(() => '' + Util.decamelize(nodeClass) + ' ' + PrintAst(obj.node))();
    let p, n;
    p = obj.path.slice(0, 2);
    n = p.apply(obj.ast, true);
    code = PrintAst(n);
    if(obj.node instanceof ESNode) n = obj.node;
    else p = obj.node;
    let type = [/(import|require)/i, /exports?[^a-z]/i].filter((re) => re.test(code));
    type = type.map((re) => [...re.exec(code)]).map(([m]) => m);
    type = type.map((m) => m + '');
    type = (Util.isArray(type) && type[0]) || type;
    let assoc = ESNode.assoc(obj.node) || ESNode.assoc(n);
    let position = ESNode.assoc(obj.node).position || ESNode.assoc(n).position;
    if(position) {
      position = position.clone();
      position.file = ES6Env.pathTransform(position.file);
    }
    let bindings = (obj.bindings instanceof Map && obj.bindings) || new Map();
    ret = Object.assign(ret, { type, bindings, position });
    let dir = path.relative(ES6Env.cwd, path.dirname(obj.file));
    ret = Util.define(ret, { ...obj, nodeClass, dir });

    bindings[inspectSymbol] = function() {
      return Util.inspect(this);
    };
    obj.importNode = obj.importNode || [];

    let importNodes = obj.importNode
      .filter((p) => p.length < 3)
      .map((p) => deep.get(obj.ast, p))
      .map((n) => [ESNode.assoc(n).position, PrintAst(n)])
      .map(([p, n]) => [Util.isGenerator(position) && [...position].map((p) => ES6Env.pathTransform(p)).join(':'), n]);

    if(Util.isObject(position) && position.toString) position = position.toString(true, (p, i) => (i == 0 ? path.relative(ES6Env.cwd, p) : p)).replace(/1;33/, '1;34');
    const InspectFn = ret.bindings[inspectSymbol];
    /*  console.log(Util.ansi.text(Util.ucfirst((type + '').toLowerCase()), 1, 31) + Util.ansi.text(` @ `, 1, 36),
      InspectFn ? InspectFn.call(ret.bindings) : '',
      'importNode:',
      [...obj.importNode].map((n) => [node2path.get(n), PrintAst(n)]).filter(([p, c]) => c.trim() != ''),
      ...['dir', 'relpath'].reduce((acc, n) => [...acc, Util.ansi.text(n, 38, 5, 197) + Util.ansi.text(' ', 1, 36) + (typeof ret[n] == 'string' ? Util.ansi.text(ret[n], 1, 32) : typeof InspectFn == 'function' ? InspectFn.call(ret) : Util.toString(ret[n], { multiline: false, newline: '' }).replace(/.*\)\ {/g, '')) + ' '], [])
    );*/
    Object.assign(ret, obj);
    if(ret.file) ret.module = ES6Module.getOrCreate(ret.file, null);
    if(ret.fromPath) ret.fromModule = ES6Module.getOrCreate(ret.fromPath, position || ret.file);

    ES6ImportExport.importExportList.add(ret);
    return ret;
  }

  get relpath() {
    const { file } = this;
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
    return PrintAst(this.node);
  }
  get code() {
    return this.toSource();
  }
  [inspectSymbol]() {
    let { type, position, bindings, path, node, file, from, fromPath, relpath } = this;
    const opts = { colors: true, colon: ': ', multiline: false, quote: '' };
    if(Util.isIterator(position)) position = [...position].map((p) => ES6Env.pathTransform(p)).join(':');
    return Util.toString(Object.assign(
        Object.setPrototypeOf({
            type,
            bindings: Util.inspect(bindings, { colors: true, toString: 'toString' }),
            path: path.join('.'),
            node: PrintAst(node) //Util.toString(node, { ...opts, separator: '', newline: '', depth: 0 })
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
    //console.log('toString', ...Util.getMemberNames(this).map((p) => [p, this[p]]).map(([k, v]) => [k, v + '']).flat());
    return path.relative(ES6Env.cwd, this.file || '');
  }
  [Symbol.toStringTag]() {
    return path.relative(ES6Env.cwd, this.file || '');
  }
}

const isRequire = ([path, node]) => node instanceof CallExpression && node.callee.value == 'require';
const isImport = ([path, node]) => node instanceof ImportStatement;
const isES6Export = ([path, node]) => node instanceof ExportStatement;
const isCJSExport = ([path, node]) => node instanceof MemberExpression && node.object instanceof Identifier && node.property instanceof Identifier && node.object.value == 'module' && node.property.value == 'exports';

const getImport = ([p, n]) => {
  let r = [];
  if(n instanceof CallExpression && Util.isObject(n, 'callee').value == 'require') r.push(p.concat(['arguments', 0]));
  else if(!(n instanceof ImportStatement)) r.push(p.slice(0, 2));
  r.push(p);
  return r;
};
const getExport = ([p, n]) => [n instanceof ExportStatement ? p : p.slice(0, 2), p];

function PrintCode(node) {
  return PrintObject(node, (node) => PrintAst(node));
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

function DumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;
  filesystem.writeFile(name, data + '\n');
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

function PrintAst(ast, comments, printer = new Printer({ indent: 4 }, comments)) {
  return printer.print(ast);
}

const hl = {
  id: (text) => Util.ansi.text(text, 1, 33),
  punct: (text) => Util.ansi.text(text, 1, 36),
  str: (text) => Util.ansi.text(text, 1, 32)
};

function DumpNode(node) {
  return [hl.id`path` + hl.punct`:`, node2path.get(node), Util.ansi.text(`node`, 1, 33) + `:`, Util.toString(node, { multiline: false, depth: 4 })];
}

function GenerateFlatMap(ast, root = [], pred = (n, p) => true, t = (p, n) => [root.concat(p).join('.'), n]) {
  flat = deep.flatten(ast,
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
  await ConsoleSetup({ colors: true, depth: 4 });
  filesystem = await PortableFileSystem();

  const re = /(lib\/util.js$)/;
  let parameters = [];

  while(/^-/.test(args[0])) parameters.push(args.shift());

  if(args.length == 0) args = ['lib/geom/point.js', 'lib/geom/size.js', 'lib/geom/trbl.js', 'lib/geom/rect.js', 'lib/dom/element.js'];
  let r = [];
  let processed = [];

  const dirs = [ES6Env.cwd, ...args.map((arg) => path.dirname(arg))];

  searchPath = makeSearchPath(dirs);
  packagesPath = makeSearchPath(dirs, 'package.json');
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

  let optind = 0;

  while(args.length > 0) {
    let arg = args.shift();
    while(/:[0-9]+:?$/.test(arg)) arg = arg.replace(/:[0-9]*$/g, '');
    processFile(ES6Env.pathTransform(arg));
    optind++;
  }

  console.log('processed:', ...processed.map((file) => `\n  ${file}`));

  DumpFile(`${name}.es`, r.join('\n'));
  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  // process.exit(Number(processed.length == 0));

  function removeFile(file) {
    Util.remove(args, file);
    Util.pushUnique(processed, ES6Env.pathTransform(file));
  }

  function processFile(file, depth = 0) {
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
    console.log('processing:', file);

    let thisdir = path.dirname(file);
    let absthisdir = path.resolve(thisdir);
    let imports;

    let { data, error, ast, parser, printer } = parseFile(file);
    let flat, map;

    function generateFlatAndMap() {
      flat = GenerateFlatMap(ast,
        [],
        (node) => node instanceof ESNode,
        (p, n) => [new ImmutablePath(p), n]
      );
      map = Util.mapAdapter((key, value) =>
        key !== undefined
          ? value !== undefined
            ? value === null
              ? deep.unset(ast, [...key])
              : deep.set(ast, [...key], value)
            : deep.get(ast, [...key])
          : (function* () {
              for(let [key, value] of flat.entries()) yield [key, key.apply(ast, true)];
            })()
      );
      node2path = new WeakMap([...flat].map(([path, node]) => [node, path]));
    }

    try {
      const getRelative = (filename) => path.join(thisdir, filename);

      //console.log('node2path:', node2path);

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
      let importNodes;

      while(true) {
        generateFlatAndMap();

        imports = [...flat].filter((it) => isRequire(it) || isImport(it));
        importNodes = imports.map((it) => getImport(it));

        if(imports.length) {
          let importStatements = importNodes.map(([stmt, imp]) => deep.get(ast, stmt));
          let declPaths = Util.unique(importNodes.map(([stmt]) => stmt.join('.')));

          let declStatements = declPaths
            .map((p) => new ImmutablePath(p.split(/\./g)))
            .map((p) => [p, deep.get(ast, p)])
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
      let importDeclarations = imports.map(([path, node], i) => getDeclarations(ast, importNodes[i]));
      //console.debug('importDeclarations:', importDeclarations);
      imports = imports.map(([path, node], i) =>
        ES6ImportExport.create({
          ast,
          node,
          importNode: importNodes[i],
          path,
          file,
          identifiers: node.right instanceof Identifier ? [node.right.value] : [],
          position: ESNode.assoc(node).position,
          fromValue: getFromValue([node, path]),
          fromPath: getFromPath([path, node], file),
          bindings: new Map(importDeclarations[i])
        })
      );
      let statement2module = imports.map((imp) => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);
      let alter = imports.filter(({ file, ...module }) => re.test(file));
      alter = alter.map((node) => {
        const to = path.relative(ES6Env.cwd, node.file);
        const from = node.fromValue;
        node.from = new Literal(`'${to}'`);
        return node;
      });
      let remove = imports.map((imp, idx) => [idx, imp]);
      remove.forEach(([i, imp]) => {
        let nodes = importNodes[i];
        let { path, node } = imp;
        console.log(`remove.forEach arg`, i, importDeclarations[i].entries(), Util.className(node));
        deep.set(ast, [...path], undefined);
      });

      let recurseImports = Util.unique(remove.map(([idx, imp]) => imp || imports[idx]));
      //   console.log(`recurseImports [${depth}]:`, recurseImports);
      let recurseFiles = recurseImports.map((imp) => path.relative(ES6Env.cwd, getFromPath([imp.path, imp.node], file))).filter((imp) => processed.indexOf(imp.file) == -1);

      imports = imports.filter(({ file, ...module }) => !re.test(file));

      if(recurseFiles.length > 0) {
        console.log(`recurseFiles [${depth}] `, recurseFiles.length, recurseFiles);
        console.log(`${Util.ansi.text(modulePath, 1, 36)}: recurseFiles x${depth}]:`,
          recurseFiles.map((f) => f)
        );
        recurseFiles.forEach((imp, idx) => {
          if(processed.indexOf(imp) == -1) {
            console.log(`recurseFiles [${depth}] forEach #${idx}:`, imp);
            processFile(imp, (depth || 0) + 1);
          }
        });
      }

      let exportNodes = [...flat].filter((entry) => isCJSExport(entry) || isES6Export(entry));
      let exportPaths = exportNodes.map(getExport);
      let exportEntries = exportPaths.map(([path, path2]) => [path, path2, deep.get(ast, path)]).map(([path, path2, node]) => [path, path2, node, node instanceof AssignmentExpression ? node.right : node]);
      let moduleExports = exportEntries.map(([path, path2, node, node2]) => [node != node2 ? 'default' : null, node2]);
      let exportInstances = moduleExports.map(([name, node], i) =>
        ES6ImportExport.create({
          ast,
          node,
          exportNode: exportEntries[i][2],
          path: exportEntries[i][0],
          file,
          identifiers: [name],
          bindings: Object.fromEntries([[name, node]])
        })
      );

      //  console.log(`moduleExports:`, moduleExports);

      let output = '';
      output = PrintAst(ast, parser.comments, printer);
      r.push(`/*\n * concatenanted ${file}\n */\n\n${output}\n`);
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
    console.log('processed files:', ...processed);
  }

  console.debug(`Modules:\n` + ES6Module.tree());
  /*  console.debug(`ES6Imports:`, new Map(ES6Module.list.map((module) => [module.file, module.imports])));
  console.debug(`ES6Exports:`, Util.toString(new Map(ES6Module.list.map((module) => [module.file, new Map(module.exports.map((exp) => [exp.position && exp.position.clone(true, false), exp.bindings]))])), { colors: true, multiline: true, toString: Symbol.toStringTag }));
  console.debug(`ES6Module.root.imports[0].bindings`, Util.inspect(ES6Module.list.map((module) => module.exports.map((i) => new Map(Object.entries(i.bindings)))).flat()[0]));
  console.debug(`new Identifier("test")`, Util.toString(new Identifier('test'), { toString: 'toString' }));
  console.debug(`new Literal("'test'")`, Util.toString(new Literal("'test'"), { toString: 'toString' }));*/
}

function parseFile(file) {
  let data, error, ast, flat;
  try {
    data = filesystem.readFile(file);
    parser = new ECMAScriptParser(data.toString(), file);
    g.parser = parser;
    ast = parser.parseProgram();
    parser.addCommentsToNodes(ast);
  } catch(err) {
    error = err;
  } finally {
    if(!ast) if (!error) error = new Error(`No ast for file '${file}'`);
  }
  if(error) throw error;
  printer = new Printer({ indent: 4 });
  g.printer = printer;
  return { data, error, ast, parser, printer };
}

function getFile(module, file) {
  // console.log(`getFile(  `, module, file, ` )`);
  return searchModuleInPath(module, file);
}

function getFromValue(...args) {
  if(args[0] == undefined) args.shift();
  if(Util.isArray(args) && args.length == 1) args = args[0];
  let [p, n] = args[0] instanceof ESNode ? args.reverse() : args;
  if(!p || (!('length' in p) && typeof p != 'string')) throw new Error('No path:' + p + ' node:' + PrintAst(n));
  if(!n || !(n instanceof ESNode)) throw new Error('No node:' + n + ' path:' + p);
  if(!(n instanceof ESNode)) n = deep.get(ast, n);
  let pathStr = p.join('.');
  let flat = GenerateFlatMap(n,
    p,
    (n, p) => true || Util.isArray(n) || [ExportStatement, ImportStatement, ObjectBindingPattern, Literal].some((ctor) => n instanceof ctor),
    (p, n) => [
      p,
      Object.setPrototypeOf({
          ...Util.filterKeys(n, (k) => n instanceof CallExpression || (k != 'type' && !(Util.isObject(n[k]) || Util.isFunction(n[k]))))
        },
        Object.getPrototypeOf(n)
      )
    ]
  );
  let literals = [...flat.entries()].filter(([p, n]) => n instanceof Literal);
  let paths = literals.map(([p, n]) => flat.get(p).value);
  if(paths) {
    let v = paths.map((n) => n.replace(/^[\u0027\u0022\u0060](.*)[\u0027\u0022\u0060]$/g, '$1'))[0];
    if(v) {
      return v;
    }
  }
}

function getFrom([node, path]) {
  if(node instanceof CallExpression) {
    node = node.arguments[0];
    path = path.down('arguments', 0);
  }
  if(node instanceof Literal) {
    node = node.value;
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
  let fileName,
    fromStr = getFromValue([node, path]);
  if(typeof fromStr == 'string') fileName = getFile(fromStr, file);
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
  DumpFile('trace.log', parser.trace());
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
    p = path.relative(cwd, p, cwd);
    if(r.indexOf(p) == -1) r.unshift(p);
  };
  let i = 0;
  for(let cwd of dirs) {
    let parts = (cwd + '').split(/[\\\/]/g);
    while(parts.length && parts[parts.length - 1] != '') {
      const dir = parts.join('/');
      const extra_dir = path.join(dir, extra);
      if(extra == 'node_modules')
        if(i == 0) {
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
    if(!checkExists(module)) {
      module = null;
      let entries = readdirRecursive(relpath).filter((name) => /\.js$/.test(name));
      if(entries.length == 1) module = path.join(relpath, entries[0]);
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
  name = name.replace(/\..?js$/g, '');
  if(moduleAliases.has(name)) return moduleAliases.get(name);
  let names = [name, ...makeNames(name)];
  let searchDirs = isRelative ? [thisdir] : [thisdir, ...searchPath];
  for(let dir of searchDirs) {
    let searchFor = dir.endsWith('node_modules') && !/\//.test(name) ? [name] : names;
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      let p;
      if(filesystem.exists(modPath)) p = findModule(modPath);
      if(p) {
        return p;
      }
    }
  }
  let fromModule = ES6Module.get(_from);
  if(!fromModule) throw new Error(`Module "${_from}" not found (${name})`, name);
  let chain = fromModule.chain;
  throw new URIError(`Module '${name}' imported from '${_from}' not found ` + Util.toString({ name, chain }, { multiline: false }), name);
}

function removeStatements(ast, statements, predicate = (stmt) => true) {
  let removed = [];
  for(let [path, node] of statements) {
    if(!predicate(node, path)) continue;
    deep.unset(ast, path);
    removed.push(node);
  }
  return removed;
}

function getBindings(properties) {
  return properties.filter((node) => node).map((node) => (node.id ? [node.id.value, node.value.value] : [node, node]));
}

function getDeclarations(ast, paths) {
  let acc = [];
  let [path, path2] = paths;

  path = path.slice(0, path.indexOf('init'));
  let node = deep.get(ast, path);

  if(node.id && node.init) return [[node.id, node.init]];

  console.log('getDeclarations:', node);

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

Util.callMain(main, true);
