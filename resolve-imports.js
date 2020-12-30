import { ECMAScriptParser } from './lib/ecmascript.js';
import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import Lexer, { PathReplacer, Position, Range } from './lib/ecmascript/lexer.js';
import Printer from './lib/ecmascript/printer.js';
import { Token } from './lib/ecmascript/token.js';
import estree, { AliasName, VariableDeclaration, VariableDeclarator, ImportStatement, ExportStatement, Identifier, MemberExpression, ESNode, CallExpression, ObjectBindingPattern, Literal, AssignmentExpression, ExpressionStatement } from './lib/ecmascript/estree.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import { ImmutablePath, Path } from './lib/json.js';
import deep from './lib/deep.js';
import PortableChildProcess, { SIGTERM, SIGKILL, SIGSTOP, SIGCONT } from './lib/childProcess.js';
import { Repeater } from './lib/repeater/repeater.js';
import { isStream, AcquireReader, AcquireWriter, ArrayWriter, readStream, PipeTo, WritableRepeater, WriteIterator, AsyncWrite, AsyncRead, ReadFromIterator, WriteToRepeater, LogSink, StringReader, LineReader, DebugTransformStream, CreateWritableStream, CreateTransformStream, RepeaterSource, RepeaterSink, LineBufferStream, TextTransformStream, ChunkReader, ByteReader, PipeToRepeater, Reader, ReadAll } from './lib/stream/utils.js';

let filesystem, childProcess, search, files;
let node2path, flat, value, list;
const removeModulesDir = PrefixRemover([/node_modules\//g, /^\.\//g]);
let name;
let parser, printer;
const g = Util.getGlobalObject();
const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');
const packages = new Map();

let searchExts = ['.js', '.mjs'];

Util.define(Array.prototype, {
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
    return [...ES6Module.moduleList].find(m => m.file === file);
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
    for(let prop of ['file', 'importedFrom', 'importedPosition', 'deps'])
      if(this[prop])
        s += '\n' + t(prop.padStart(13, ' '), 1, 33) + t(': ', 1, 36) + formatProp(this[prop]);
    s += t('\n}', 1, 36);
    function formatProp(value) {
      if(Util.isArray(value))
        return (t('[ ', 1, 36) +
          value.map(mod => formatProp(mod.file)).join(t(', ', 1, 36)) +
          t(' ]', 1, 36)
        );
      return typeof value == 'string' ? t(value, 1, 32) : value;
    }
    return s;
  }

  get deps() {
    return [...ES6Module.getDeps(this)];
  }

  get imports() {
    return [...ES6Module.getImportExport(this, i => i == -1)];
  }

  get exports() {
    return [...ES6Module.getImportExport(this, i => i != -1)];
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
    const t = colors ? (...args) => Util.ansi.text(...args) : t => t;
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
        printModule(deps[i],
          sym.replace(/\u251c\u2500/g, '\u2502 ').replace(/\u2514\u2500/g, '  ') +
            ((i + 1 == deps.length ? '\u2514' : '\u251c') + '\u2500') +
            ' '
        );
      }
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
    let type = [/(import|require)/i, /exports?[^a-z]/i].filter(re => re.test(code));
    type = type.map(re => [...re.exec(code)]).map(([m]) => m);
    type = type.map(m => m + '');
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

    let importNode = obj.importNode.filter(p => p instanceof Path);
    //console.log('importNode:', importNode);

    let importNodes = importNode
      //   .filter(p => p.length < 3)
      .map(p => deep.get(obj.ast, p))
      .map(n => [ESNode.assoc(n).position, PrintAst(n)])
      .map(([p, n]) => [
        Util.isGenerator(position) && [...position].map(p => ES6Env.pathTransform(p)).join(':'),
        n
      ]);

    if(Util.isObject(position) && position.toString)
      position = position
        .toString(true, (p, i) => (i == 0 ? path.relative(ES6Env.cwd, p) : p))
        .replace(/1;33/, '1;34');
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
    while(Util.isObject(value) && value.value) value = value.value;
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
    if(Util.isIterator(position))
      position = [...position].map(p => ES6Env.pathTransform(p)).join(':');
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
    let obj = Util.filterOutMembers(this, x =>
      [Util.isFunction /*, Util.isObject*/].some(f => f(x))
    );
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

const isRequire = ([path, node]) =>
  node instanceof CallExpression && node.callee.value == 'require';
const isImport = ([path, node]) => node instanceof ImportStatement;
const isES6Export = ([path, node]) => node instanceof ExportStatement;
const isCJSExport = ([path, node]) =>
  node instanceof MemberExpression &&
  node.object instanceof Identifier &&
  node.property instanceof Identifier &&
  node.object.value == 'module' &&
  node.property.value == 'exports';

const getImport = ([p, n]) => {
  let r = [];
  //console.log('getImport', { p, n });
  if(n instanceof CallExpression && Util.isObject(n) && n.callee && n.callee.value == 'require') {
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
  } /* if(!(n instanceof ImportStatement))*/ else {
    let source = p.concat(['source']);
    for(let [node, path] of deep.iterate(n, v => v instanceof AliasName)) {
      const name = p.concat([...path, 'name']);
      const alias = p.concat([...path, 'as']);
      r.push([source, alias, name]);
    }
  }
  return r;
};
const getExport = ([p, n]) => [
  n instanceof ExportStatement ? p : p.slice(0, 2),
  p
]; /*.map(p => [...p])*/

function PrintCode(node) {
  return PrintObject(node, node => PrintAst(node));
}
function PrintObject(node, t = (n, p) => n) {
  return Object.entries(node)
    .map(([prop, value]) => [prop, ': ', t(value, prop)])
    .flat();
}

function PrefixRemover(reOrStr, replacement = '') {
  if(!(Util.isArray(reOrStr) || Util.isIterable(reOrStr))) reOrStr = [reOrStr];

  return arg =>
    reOrStr.reduce((acc, re, i) => acc.replace(re, replacement), typeof arg == 'string' ? arg : '');
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
  id: text => Util.ansi.text(text, 1, 33),
  punct: text => Util.ansi.text(text, 1, 36),
  str: text => Util.ansi.text(text, 1, 32)
};

function DumpNode(node) {
  return [
    hl.id`path` + hl.punct`:`,
    node2path.get(node),
    Util.ansi.text(`node`, 1, 33) + `:`,
    Util.toString(node, { multiline: false, depth: 4 })
  ];
}

function GenerateFlatMap(ast,
  root = [],
  pred = (n, p) => true,
  t = (p, n) => [root.concat(p).join('.'), n]
) {
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
  await ConsoleSetup({ colors: true, depth: 6 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableChildProcess(cp => (childProcess = cp));

  const re = {
    name: /^(process|readline)$/,
    path: /(lib\/util.js$|^lib\/)/
  };
  let parameters = [];

  while(/^-/.test(args[0])) parameters.push(args.shift());

  if(args.length == 0)
    args = [
      'lib/geom/point.js',
      'lib/geom/size.js',
      'lib/geom/trbl.js',
      'lib/geom/rect.js',
      'lib/dom/element.js'
    ];
  let r = [];
  let processed = [];

  const dirs = [ES6Env.cwd, ...args.map(arg => path.dirname(arg))];

  search = MakeSearch(dirs);

  console.log('search:', search);
  let optind = 0;

  while(args.length > 0) {
    let arg = args.shift();
    while(/:[0-9]+:?$/.test(arg)) arg = arg.replace(/:[0-9]*$/g, '');
    await processFile(ES6Env.pathTransform(arg));
    optind++;
  }

  //console.log('processed:', ...processed.map(file => `\n  ${file}`));

  let success = Object.entries(processed).filter(([k, v]) => !!v).length != 0;
  // process.exit(Number(processed.length == 0));

  console.log('processed files:', processed);

  DumpFile(`${name}.es`, r.join('\n'));

  console.log(`\nModules:\n\n  ` + ES6Module.tree().replace(/\n/g, '\n  '));

  /*  console.log(`ES6Imports:`, new Map(ES6Module.list.map((module) => [module.file, module.imports])));
  console.log(`ES6Exports:`, Util.toString(new Map(ES6Module.list.map((module) => [module.file, new Map(module.exports.map((exp) => [exp.position && exp.position.clone(true, false), exp.bindings]))])), { colors: true, multiline: true, toString: Symbol.toStringTag }));
  console.log(`ES6Module.root.imports[0].bindings`, Util.inspect(ES6Module.list.map((module) => module.exports.map((i) => new Map(Object.entries(i.bindings)))).flat()[0]));
  console.log(`new Identifier("test")`, Util.toString(new Identifier('test'), { toString: 'toString' }));
  console.log(`new Literal("'test'")`, Util.toString(new Literal("'test'"), { toString: 'toString' }));*/

  function removeFile(file) {
    Util.remove(args, file);
    Util.pushUnique(processed, ES6Env.pathTransform(file));
  }

  async function processFile(file, depth = 0) {
    console.log('processFile:', { file, depth });
    console.log(`ParentPackage(${file}) = ${ParentPackage(file)}`);

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
    let imports;
    // console.log('processing:', { file, thisdir });

    let { data, error, ast, parser, printer } = await ParseFile(file);
    let flat, map;
    //console.log(`${file} parsed:`, { data, error });

    function generateFlatAndMap() {
      flat = GenerateFlatMap(ast,
        [],
        node => node instanceof ESNode,
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
      const getRelative = filename => path.join(thisdir, filename);

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
      let importValues;
      again: while(true) {
        generateFlatAndMap();

        imports = [...flat].filter(it => isRequire(it) || isImport(it));
        // console.log("imports:",imports.map(([p,n]) => [p,n]));
        //console.log("flat:",flat);

        for(let imp of imports) {
          let expr = ParentExpr(ast, imp);

          if(!(expr instanceof AssignmentExpression || expr instanceof VariableDeclarator)) {
            if(imp[0].length == 2) continue;

            let stmt = Path2Ptr(ast, imp[0].slice(0, 2));
            let root = Path2Ptr(ast, imp[0].slice(0, 1));
            let pos = imp[0][1];

            let source = GetFrom([imp[1], imp[0]])[0];

            let name = Util.camelize(path.basename(source));

            deep.set(ast, imp[0], new Identifier(name));

            let importStatement = new ImportStatement([new AliasName(new Identifier('default'), new Identifier(name))],
              new Literal(source)
            );

            // console.log('stmt:', stmt[1]);
            let assoc = Util.filterKeys(ESNode.assoc(stmt[1]), ['range', 'comments', 'position']);

            ///console.log("imp[2]",assoc);
            ESNode.assocMap.set(importStatement, assoc);

            root[1].splice(pos, 0, importStatement);

            continue again;
          }
        }

        //console.log('imports:', imports);
        importNodes = imports.map(it => getImport(it));
        //console.log('importNodes:', importNodes);

        importValues = imports.map(it => {
          let importIds = getImport(it);
          //console.log('importIds:', importIds);
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
        //console.log('importValues:', importValues);

        if(imports.length) {
          //console.log('ast.body:', ast.body);
          let importStatements = imports.map(([stmt, imp]) => imp);
          let declPaths = Util.unique(importNodes.map(([stmt]) => stmt.toString()));

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

      const useStrict = map.filter(([key, node]) => node instanceof Literal && /use strict/.test(node.value)
      );
      useStrict.forEach(([path, node]) => deep.unset(ast, path));
      // console.log('importNodes:', importNodes);
      let importDeclarations = importValues.map(ids =>
        ids.map(a =>
          a /*.slice(1)*/
            .map(([p, n]) => n)
        )
      );

      //console.log('importDeclarations:', importDeclarations);

      imports = imports
        .map(([path, node]) => [path, node, Literal.string(GetLiteral(node))])
        .filter(([path, node, module]) => !re.name.test(module));

      //console.log('imports:', imports);

      imports = imports.map(([path, node], i) => {
        const assoc = ESNode.assoc(node);
        //console.log(`imports[${i}]`, { path, node, assoc });

        const { position } = assoc;
        const fromPath = GetFromPath([path, node], position || file);

        const decls = importDeclarations[i].map(a =>
          a.slice(1).map(n => new Identifier(Literal.string(n)))
        );
        const importNode = imports[i];

        //console.log('ES6ImportExport:', { path, node, fromPath, decls, importNode });

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
          bindings: new Map(decls)
        });
      });
      let statement2module = imports.map(imp => [imp.node, imp]);
      statement2module = new WeakMap(statement2module);

      //console.log(`imports YYY:`, imports.map(imp => [imp.path, imp.position || { file: imp.file }]));

      const fromMap = new WeakMap(imports.map((imp, idx) => [
          imp.node,
          Util.tryCatch(() => imp.fromPath,
            v => v,
            () => Literal.string(GetLiteral(imp.node))
          )
        ])
      );

      //console.log(`imports XXX:`, imports.map(imp => fromMap.get(imp.node)));

      let remove = imports.filter(imp => fromMap.get(imp.node)).map((imp, idx) => [idx, imp]);

      remove = remove.filter(([idx, imp]) => !IsBuiltinModule(imp.fromPath));
      //console.log(`remove:`, remove.map(([idx, imp]) => imp.fromPath));
      remove.forEach(([i, imp]) => {
        let nodes = importNodes[i];
        let { path, node } = imp;
        // console.log(`remove.forEach arg`, i, GetFromPath([path,node], file), importDeclarations[i].entries(), Util.className(node));
        deep.set(ast, [...path], new ExpressionStatement( new Literal('"removed import"')));
      });

      let recurseImports = Util.unique(remove.map(([idx, imp]) => imp || imports[idx]));
      let recursePaths = recurseImports.map(imp => [
        ES6Env.cwd,
        GetFromPath([imp.path, imp.node], file),
        imp.node
      ]);
      //console.log(`recursePaths [${depth}]:`, recursePaths.map(p => p[1]));
      let recurseFiles = recursePaths.map(paths => path.relative(...paths.slice(0, 2)));

      //console.log(`recurseFiles [${depth}]:`, recurseFiles);
      recurseFiles = recurseFiles.filter(f => processed.indexOf(f) == -1 && !re.path.test(f));
      recurseFiles = recurseFiles.filter(f => !re.path.test(f));

      imports = imports.filter(({ file, ...module }) => !re.path.test(file));

      if(recurseFiles.length > 0) {
        //console.log(`recurseFiles [${depth}] `, recurseFiles.length, recurseFiles);
        console.log(`${Util.ansi.text(modulePath, 1, 36)}: recurseFiles [${depth}]:`,
          recurseFiles.map(f => f)
        );
        let idx = 0;
        for(let imp of recurseFiles) {
          if(processed.indexOf(imp) == -1) {
            //console.log(`recurseFiles [${depth}] forEach #${idx}:`, imp);
            await processFile(imp, (depth || 0) + 1);
          }
          idx++;
        }
      }

      let exportNodes = [...flat].filter(entry => isCJSExport(entry) || isES6Export(entry));
      let exportPaths = exportNodes.map(getExport);
      //console.log('exportPaths:', exportPaths);
      let exportEntries = exportPaths
        .map(([path, path2]) => [path, path2, deep.get(ast, [...path])])
        .map(([path, path2, node]) => [
          path,
          path2,
          node,
          node instanceof AssignmentExpression ? node.right : node
        ]);
      let moduleExports = exportEntries.map(([path, path2, node, node2]) => [
        node != node2 ? 'default' : null,
        node2
      ]);
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

      //console.log(`moduleExports:`, moduleExports);

      let output = '';
      output = PrintAst(ast, parser.comments, printer);
      r.push(`/*\n * concatenanted ${file}\n */\n\n${output}\n`);
    } catch(err) {
      console.error(err.message);
      Util.putStack(err.stack);
      process.exit(1);
    }
  }
}

function FdReader(fd, bufferSize = 1024) {
  let buf = filesystem.buffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await filesystem.waitRead(fd);
      ret = filesystem.read(fd, buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(filesystem.bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    filesystem.close(fd);
  });
}

async function Prettier(file) {
  let input = filesystem.open(file, 'r');
  let proc = childProcess('sh',
    [
      '-c',
      `node_modules/.bin/prettier --config .prettierrc --parser babel <'${file}' | tee '${file}.prettier'`
    ],
    {
      block: false,
      stdio: [input, 'pipe', 'pipe']
    }
  );
  let sink = RepeaterSink(wr => proc.stdio[1].pipe(wr));
  let data = '';
  for await (let r of await sink) data += r;
  return data;
}

async function ParseFile(file) {
  let data, error, ast, flat;
  try {
    //    data = filesystem.readFile(file);

    data = await Prettier(file);
    // console.log('data:', Util.abbreviate(Util.escape(data + ''), 40));
    //   console.log('data:', data.length, data || Util.abbreviate(Util.escape(data + ''), 40));

    ECMAScriptParser.instrumentate();
    parser = new ECMAScriptParser(data.toString(), file, false);
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

function GetFile(module, position) {
  let r;
  let file = typeof position.file == 'string' ? position.file : position;
  if(position instanceof Range) position = position.start;

  console.log('GetFile', { module, position, file }, Util.className(position));

  module = module.replace(/\?.*/g, '');

  if(module.startsWith('.') && typeof file == 'string' && !path.isAbsolute(module))
    module = path.join(path.dirname(file), module);

  try {
    if(filesystem.exists(module)) r = module;
    else r = SearchModuleInPath(module, file, position);
  } catch(err) {
    console.log(`GetFile(`, module, ', ', position, `) =`, err);
    throw err;
  }
  return r;
}

function GetFromValue(...args) {
  if(args[0] == undefined) args.shift();
  if(Util.isArray(args) && args.length == 1) args = args[0];
  let [p, n] = args[0] instanceof ESNode ? args.reverse() : args;
  if(!p || (!('length' in p) && typeof p != 'string'))
    throw new Error('No path:' + p + ' node:' + PrintAst(n));
  if(!n || !(n instanceof ESNode)) throw new Error('No node:' + n + ' path:' + p);
  if(!(n instanceof ESNode)) n = deep.get(ast, n);
  let pathStr = p.join('.');
  let flat = GenerateFlatMap(n,
    p,
    (n, p) =>
      true ||
      Util.isArray(n) ||
      [ExportStatement, ImportStatement, ObjectBindingPattern, Literal].some(ctor => n instanceof ctor
      ),
    (p, n) => [
      p,
      Object.setPrototypeOf({
          ...Util.filterKeys(n,
            k =>
              n instanceof CallExpression ||
              (k != 'type' && !(Util.isObject(n[k]) || Util.isFunction(n[k])))
          )
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

function ParentNode(ast, [path, node]) {
  if(path.length > 0) {
    path = path.slice(0, -1);
    node = deep.get(ast, path);
    return [path, node];
  }
  return [null, null];
}

function Path2Ptr(ast, path) {
  /*if(!(path instanceof Path))
    path = new Path(path);*/

  return [path, deep.get(ast, path)];
}

function ParentExpr(ast, ptr) {
  if(ptr[0].last == 'callee') ptr[0] = ptr[0].up();

  if(isRequire(ptr)) {
    ptr = ParentNode(ast, ptr);
    if(ptr[1] instanceof MemberExpression) ptr = ParentNode(ast, ptr);
    return ptr;
  }
}

function GetFrom([node, path]) {
  if(node instanceof CallExpression) {
    node = node.arguments[0];
    path = path.down('arguments', 0);
  }
  if(node instanceof ImportStatement) {
    node = node.source;
    path = path.down('source');
  }
  if(node instanceof Literal) {
    node = Literal.string(node);
    path = path.down('value');
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
  return /^(std|os|ffi|net|_http_agent|_http_client|_http_common|_http_incoming|_http_outgoing|_http_server|_stream_duplex|_stream_passthrough|_stream_readable|_stream_transform|_stream_wrap|_tls_common|_tls_wrap|assert|async_hooks|buffer|child_process|cluster|console|constants|crypto|dgram|dns|domain|events|fs|http|http2|https|inspector|module|net|os|path|perf_hooks|process|punycode|querystring|readline|repl|stream|string_decoder|timers|tls|trace_events|tty|url|util|v8|vm|worker_threads|zlib)$/.test(name
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
  return path.basename(filename).replace(/\.[a-z0-9]*$/, '');
}

function ReaddirRecursive(dir) {
  let ret = [];
  for(let entry of filesystem.readdir(dir)) {
    if(entry == '.' || entry == '..') continue;
    let file = path.join(dir, entry);
    if(filesystem.stat(file).isDirectory()) {
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
    console.log(Util.className(err) + ': ' + (err.msg || err) + '\n', err.stack);
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
      if(filesystem.exists(extra_dir)) addPath(extra_dir);
      i++;
      parts.pop();
    }
    if(extra == 'node_modules') i = 0;
  }
  return r;
}

function CheckExists(path) {
  let r = filesystem.exists(path);
  //console.log(`CheckExists('${path}') = ${r}`);
  return r;
}

function HasExtension(path) {
  return !/^[^.]*$/.test(path.basename(path));
}

function GetMain(dir) {
  let pkg;
  let name = path.basename(dir);
  let names = [
    'src/index',
    'src/' + name,
    'dist/' + name,
    'dist/index',
    'build/' + name,
    name,
    'index',
    'browser'
  ];
  if((pkg = GetPackage(dir))) {
    let main = pkg.source || pkg.module || pkg.main;
    names.unshift(main);
  }
  //console.log("GetMain",{dir,name,names});
  let indexes = names.reduce((arr, name) => [...arr, ...MakeNames(name)], []);
  for(let index of indexes) {
    let file = path.join(dir, index);
    //console.log("GetMain",{file});
    if(filesystem.exists(file)) return file;
  }
}

function FileOrDirectory(relpath, callback = name => {}) {
  let names = MakeNames(relpath);
  let index = names.findIndex(name => filesystem.exists(name));

  if(index != -1) callback(names[index]);
  return index;
}

function FindModule(relpath) {
  // console.log('FindModule', { relpath });

  //if(!filesystem.exists(relpath))
  FileOrDirectory(relpath, name => (relpath = name));

  let st = filesystem.stat(relpath);
  let module;
  const name = path.basename(relpath);
  let pkg = GetPackage(relpath);

  /*.filter(p => filesystem.exists(p))*/ if(st.isDirectory()) {
    module = GetMain(relpath);
    console.log('FindModule:', { relpath, module });
    /*    
 let indexes = main ?  MakeNames(path.join(relpath, main)) : [
    ...MakeNames(relpath + '/src/index'),
    ...MakeNames(relpath + '/src/' + name),
    ...MakeNames(relpath + '/dist/' + name),
    ...MakeNames(relpath + '/dist/index'),
    ...MakeNames(relpath + '/build/' + name),
    ...MakeNames(relpath + '/' + name),
    ...MakeNames(relpath + '/index'),
    ...MakeNames(relpath + '/browser'),
    ...MakeNames(relpath + '/package.json')
  ];
    while(indexes.length) {
      module = indexes.shift();
      if(!CheckExists(module)) continue;
      if(module.endsWith('/package.json')) {
        let r;
        if((r = filesystem.readFile(module))) {
          let json = JSON.parse(r.toString());
          if(!json.module) continue;
          module = path.join(relpath, json.module);
        }
      }
      break;
    }*/
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
  console.log('SearchModuleInPath', { name, _from, position });

  const thisdir = _from ? path.dirname(_from) : '.';
  const absthisdir = path.resolve(thisdir);
  const isRelative = /^\.\.?\//.test(name);
  name = name.replace(/\..?js$/g, '');
  //console.log('name:', name);
  if(search.aliases.has(name)) return search.aliases.get(name);
  let names = [name, ...MakeNames(name)];
  let searchDirs = isRelative ? [thisdir] : [thisdir, ...search.path];
  for(let dir of searchDirs) {
    let searchFor = dir.endsWith('node_modules') && !/\//.test(name) ? [name] : names;
    //console.log("searchFor:", searchFor);
    for(let module of searchFor) {
      let modPath = path.join(dir, module);
      let p;

      if(filesystem.exists(modPath)) {
        let st = filesystem.stat(modPath);

        if(st.isDirectory()) if (IsPackage(modPath)) AddPackage(modPath);

        if((p = FindModule(modPath))) return p;
      }

      if(!/\.js$/.test(modPath)) {
        modPath += '.js';
        if(filesystem.exists(modPath)) if ((p = FindModule(modPath))) return p;
      }
    }
  }
  let fromModule = ES6Module.get(_from);
  if(!fromModule) throw new Error(`Module "${_from}" not found (${name})`, name);
  let chain = fromModule.chain;
  throw new URIError(`Module '${name}' imported from '${_from}' not found ` +
      Util.toString({ name, chain }, { multiline: false }),
    name
  );
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
  return properties
    .filter(node => node)
    .map(node => (node.id ? [node.id.value, node.value.value] : [node, node]));
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
  return [
    prefix + '.njs',
    prefix + '.es6.js',
    prefix + '.esm.js',
    prefix + '.module.js',
    prefix + '.module.ejs',
    prefix + '.js',
    prefix + '.mjs',
    prefix
  ];
}

function IsPackage(dir) {
  if(packages.has(dir)) return true;
  const packageFile = path.join(dir, 'package.json');
  return filesystem.exists(packageFile);
}

function AddPackage(dir) {
  const packageFile = path.join(dir, 'package.json');
  const obj = JSON.parse(filesystem.readFile(packageFile));
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
    let json = JSON.parse(filesystem.readFile(p));
    let aliases = json._moduleAliases || {};
    for(let alias in aliases) {
      let module = path.join(path.dirname(p), aliases[alias]);
      if(!filesystem.exists(module))
        throw new Error(`No such module alias from '${alias}' to '${aliases[alias]}'`);
      let file = FindModule(module);
      let st = filesystem.stat(file);
      acc.set(alias, file);
    }
    return acc;
  }, new Map());
  return o;
}

Util.callMain(main, true);
