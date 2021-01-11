import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AcquireReader } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import fs from 'fs';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';

//prettier-ignore
let filesystem, spawn;

Array.prototype.findLastIndex = function findLastIndex(predicate) {
  for(let i = this.length - 1; i >= 0; --i) {
    const x = this[i];
    if(predicate(x, i, this)) {
      return i;
    }
  }
  return -1;
};

Array.prototype.tail = function tail() {
  return this[this.length - 1];
};

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  console.log('Processing files:', args);
  await processFiles(...args);
}
const win32 = false;
async function processFiles(...files) {
  let defs = {
    PDWORD: 'unsigned long*',
    UCHAR: 'unsigned char',
    BYTE: 'char',
    TBYTE: 'uint16',
    WORD: 'unsigned short',
    DWORD: 'unsigned long',
    ULONG: 'unsigned long',
    CONST: 'const'
  };
  let args = ['-Xclang', '-ast-dump=json', '-fsyntax-only'];

  if(win32)
    args = args.concat([
      ...Object.entries(defs).map(([n, v]) => `-D${n}=${v}`),
      '-D_WIN32=1',
      '-DWINAPI=',
      '-D__declspec(x)=',
      '-include',
      '/usr/x86_64-w64-mingw32/include/wtypesbase.h',
      '-I/usr/x86_64-w64-mingw32/include'
    ]);

  for(let file of files) {
    let json = await AstDump(file, args);

    let ast = JSON.parse(json);
    let tree = new Tree(ast);
    let flat = /*tree.flat();*/ deep.flatten(ast,
      new Map(),
      (v, p) => ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 && Util.isObject(v) /*&& 'kind' in v*/
    );
    let entries = [...flat];
    let locations = [];
    let l = Object.setPrototypeOf({}, { toString() {} });
    let path = new WeakMap();
    let idmap = {};
    let id2path = {};
    for(let entry of entries) {
      const [p, n] = entry;
      let loc = GetLoc(n);
      if(loc)
        l = Object.setPrototypeOf({ ...(l || {}), ...loc },
          {
            toString() {
              let s;
              if(this.file) {
                s = this.file;
                if('line' in this) s += ':' + this.line;
                if('col' in this) s += ':' + this.col;
              }
              return s;
            }
          }
        );
      if('id' in n) {
        idmap[n.id] = entry;
        id2path[n.id] = p;
      }
      //Util.removeKeys(entries[locations.length][1], ['loc','range']);
      entry[2] = l;
      locations.push(l);
    }

    for(let [n, p] of deep.iterate(ast, v => Util.isObject(v))) {
      if(/(loc|range)/.test(p[p.length - 1] + '')) {
        //console.log("remove", p);
        deep.unset(ast, p);
      } else {
        //let node = deep.get(ast, p);
        path.set(n, p);
      }
    }
    dumpFile(file.replace(/.*\/(.*)/, '$1.ast.json'), JSON.stringify(ast, null, 2));

    function Path2Loc(path) {
      if(Array.isArray(path)) path = path.join('.');

      let idx = entries.findLastIndex(([p]) => path.startsWith(p));

      if(idx != -1) return entries[idx][2];
    }

    let indexes = new WeakMap(entries.map(([p, n], i) => [n, i]));
    let re = /(^[_P]?IMAGE_)/;
    const NoSystemIncludes = ([p, n, l]) => !/^\/usr/.test(l.file + '');
    let mainNodes = entries.filter(NoSystemIncludes);

    let typedefs = [...Util.filter(mainNodes, ([path, decl]) => decl.kind == 'TypedefDecl')];

    //typedefs = NoSystemIncludes(typedefs);

    const names = decls => [...decls].map(([path, decl]) => decl.name);
    const declarations = decls => [...decls].map(([path, decl, loc]) => [decl.name, loc.toString()]);
    let typenames = names(typedefs.filter(([path, decl]) => re.test(decl.name)));

    //console.log('obj:', obj);
    console.log('typedefs:',
      typedefs.map(([p, n, l]) => [
        indexes.get(n),
        //   p,
        n.name,
        // Util.filterOutKeys(n, (v) => typeof v == 'object'),
        l.toString()
      ])
    );
    console.log('typedefs:', declarations(typedefs));

    let nodes = new Map(mainNodes.filter(([p, n]) => 'kind' in n));
    let nodeTypes = [...nodes].map(([p, n]) => n.kind);
    let hist = Util.histogram(nodeTypes, new Map());
    console.log('histogram:', new Map([...hist].sort((a, b) => a[1] - b[1])));

    let offsetNodes = mainNodes.filter(([p, n]) => 'offset' in n);

    let namedNodes = mainNodes.filter(([p, n]) => 'name' in n);

    let loc_name =
      namedNodes.filter(([p, n]) => /Decl/.test(n.kind + '') && Util.isNumeric(p[p.length - 1])).map(([p]) => p) ||
      Util.intersect(typedefs.map(([p]) => p),
        namedNodes.map(([p]) => p)
      );
    let decls = loc_name
      .map(p => [p.split('.'), flat.get(p)])
      .map(([p, n]) => [p, n, locations[indexes.get(n)]])
      .map(([p, n, l]) => [
        n.id || indexes.get(n),
        //   p.filter((p) => p != 'inner').join('.'),
        n.name || n.referencedMemberDecl || Object.keys(n).filter(k => typeof n[k] == 'string'),
        GetType(n),
        n.kind /*.replace(/Decl$/, '')*/,
        p.join('.').replace(/\.?inner\./g, '/'),
        l.toString()
      ]);
    console.log('loc ∩ name:', loc_name.length);
    console.log('names:',
      decls
        .map(decl =>
          decl
            .map((field, i) =>
              (Util.abbreviate(field, [Infinity, Infinity, 20, Infinity, Infinity, Infinity][i]) + '').padEnd([6, 25, 20, 20, 40, 0][i]
              )
            )
            .join(' ')
        )
        .join('\n')
    );
    console.log('number of nodes:', nodes.size);
    console.log('nodes with offset:', offsetNodes.length);
    console.log('nodes with offset and no line:', offsetNodes.filter(([p, n]) => !('line' in n)).length);
    console.log('nodes with offset and no file:', offsetNodes.filter(([p, n]) => !('file' in n)).length);
    console.log('nodes with name:', namedNodes.length);
    function BasePathIndex(path) {
      return path.findIndex(k => !(k == 'inner' || Util.isNumeric(k)));
    }
    const ids = deep
      .select(ast, (v, p) => /^0x/.test(v + '') && p[p.length - 1] != 'id')
      .map(({ path, value }) => [
        path.slice(BasePathIndex(path)).join('.'),
        ...(n => [n.id, n.kind])(deep.get(ast, path.slice(0, BasePathIndex(path)))),
        value,
        idmap[value].map((f, i) => (i == 2 ? f.toString() : f)).slice(1),
        Path2Loc(path).toString()
      ])
      .map(([path, declId, refKind, refId, [declNode, declLoc], refLoc]) => [
        path,
        refId,
        refKind,
        refLoc,
        declId,
        declNode.kind,
        declLoc
      ]);
    console.log('ids:', ids);
    const refs = ids.map(([path, refId, refKind, refLoc, declId, declKind, declLoc]) => [
      path,
      [refId, refKind, refLoc],
      [declId, declKind, declLoc]
    ]);
    console.log('refs:', refs);

    const assoc = {};
    0x12cef10;
    const usages = {};
    const getDecl = Util.getOrCreate(assoc, () => new Set());

    refs.forEach(([path, ref, decl]) => {
      if(decl[1] != 'BuiltinType') getDecl(ref[0]).add(decl[0]);
      // getDecl(decl.join(' ')).push(ref.join(' '));
    });
    /* let unused = new Set();
    for(let id in assoc) {
      let decls = assoc[id];
      let remove = decls.filter((decl) => !(decl in assoc));
      //console.log(`unused decls in ${id}:`,remove);
      remove.forEach((decl) => unused.add(decl));
      decls = decls.filter((decl) => decl in assoc);
      //if (!decls.length) delete assoc[id]; else
      assoc[id] = decls;
    }
    console.log(`all unused decls:`, unused);*/
    console.log('assoc:', assoc);

    function FindBackwards(node, pred = ([p, n]) => false) {
      for(let i = indexes.get(node); i >= 0; i--) {
        if(pred(entries[i])) return entries[i][1];
      }
    }

    function GetLoc(node) {
      let loc;
      if('loc' in node) loc = node.loc;
      else if('range' in node) loc = node.range;
      else return null; //throw new Error(`no loc in ${tree.pathOf(node)}`);
      if('expansionLoc' in loc) loc = loc.expansionLoc;
      if('begin' in loc) loc = loc.begin;

      if(!('offset' in loc)) return null; // throw new Error(`no offset in loc of ${node.kind} ${Util.isEmpty(loc)}`);
      return loc;
    }
    function GetType(node) {
      let type;
      if(node.type) type = node.type;
      else if('inner' in node && node.inner.some(inner => 'name' in inner || 'type' in inner)) {
        type = node.inner.map(inner => [inner.name, GetType(inner)]);
        return '{ ' + type.map(([n, t]) => `${t} ${n};`).join(' ') + ' }';
      }
      if(typeof type != 'object') return type;

      if(type.qualType) type = type.qualType;
      return type;
    }
  }
}

Util.callMain(main, true);

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function AstDump(file, extraArgs) {
  let child = spawn(['clang', ...extraArgs, file], {
    stdin: 'inherit',
    stdio: 'pipe',
    stderr: 'pipe'
  });

  let json = '',
    errors = '';

  AcquireReader(child.stdout, async reader => {
    let r, str;
    while((r = await reader.read())) {
      if(!r.done) {
        str = r.value.toString();
        //  console.log('stdout:', );
        json += str;
      }
    }
  });

  AcquireReader(child.stderr, async reader => {
    let r;
    while((r = await reader.read())) {
      if(!r.done) errors += r.value.toString();
    }
  });

  console.log('child.wait():', await child.wait());
  console.log('errors:', errors);
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  const numErrors =
    errorLines.length && +errorLines[errorLines.length - 1].replace(/.*\s([0-9]+)\serrors\sgenerated.*/g, '$1');
  errorLines = errorLines.filter(line => /error:/.test(line));
  console.log(`numErrors: ${numErrors}`);
  console.log('errorLines:', errorLines);
  return json;
}
