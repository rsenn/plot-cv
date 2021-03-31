  import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AcquireReader } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';
import { Type, Compile, AstDump, NodeType, NodeName, GetLoc, GetTypeStr } from './clang-ast.js';

//prettier-ignore
let filesystem, spawn;

Util.define(Array.prototype, {
  findLastIndex(predicate) {
    for(let i = this.length - 1; i >= 0; --i) {
      const x = this[i];
      if(predicate(x, i, this)) {
        return i;
      }
    }
    return -1;
  },
  tail() {
    return this[this.lengtGh - 1];
  },
  startsWith(start) {
    for(let i = 0; i < start.length; i++) if(this[i] !== start[i]) return false;
    return true;
  }
});

const WriteBJSON = async (filename, obj) =>
  await import('bjson.so').then(({ write }) => {
    let data = write(obj);
    WriteFile(filename, data);
    return data.byteLength;
  });

const ReadBJSON = async filename =>
  await import('bjson.so').then(({ read }) => {
    let data = filesystem.readFile(filename, null);
    return Util.instrument(read)(data, 0, data.byteLength);
  });

async function main(...args) {
  console.log('main(', ...args, ')');
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(fs => (filesystem = fs));
  await PortableSpawn(fn => (spawn = fn));

  let params = Util.getOpt({
      output: [true, null, 'o'],
      xml: [true, null, 'X'],
      json: [true, null, 'j'],
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      debug: [false, null, 'x'],
      'system-includes': [false, null, 's'],
      'no-remove-empty': [false, null, 'E'],
      'output-dir': [true, null, 'd'],
      '@': 'input'
    },
    args
  );
  console.log('main', params);

  let defs = params.define || [];
  let includes = params.include || [];

  args = [];
  const win32 = false;

  if(win32) {
    defs = defs.concat(Object.entries({
        PDWORD: 'unsigned long*',
        UCHAR: 'unsigned char',
        BYTE: 'char',
        TBYTE: 'uint16',
        WORD: 'unsigned short',
        DWORD: 'unsigned long',
        ULONG: 'unsigned long',
        CONST: 'const'
      })
    );

    args = args.concat([
      '-D_WIN32=1',
      '-DWINAPI=',
      '-D__declspec(x)=',
      '-include',
      '/usr/x86_64-w64-mingw32/include/wtypesbase.h',
      '-I/usr/x86_64-w64-mingw32/include'
    ]);
  }
  console.log('args', { defs, includes });
  args = args.concat(defs.map(d => `-D${d}`));
  args = args.concat(includes.map(v => `-I${v}`));

  console.log('Processing files:', args);

  await processFiles(...params['@']);

  async function processFiles(...files) {
    for(let file of files) {
      const start = /*await Util.now(); //*/ await Util.hrtime();
      console.log('start:', start);
      let json, ast;
      let base = path.basename(file, /\.[^./]*$/);
      let outfile = base + '.ast.json';
      let boutfile = base + '.ast.bjson';

      async function ReadAST(outfile,
        load = f => filesystem.readFile(f),
        save = WriteFile,
        parse = JSON.parse
      ) {
        let st = [file, outfile].map(name => filesystem.stat(name));
        let times = st.map(stat => (stat && stat.mtime) || 0);
        let cached = times[1] >= times[0];
        if(cached) {
          console.log('Reading cached AST from:', outfile);
          json = /*filesystem.readFile*/ await load(outfile);
          ast = await parse(json);
          return ast;
        } /*else {
        json = await AstDump(file, args);
         save(outfile, ret);
      }*/
      }

      const loadFunctions = [
        async () => await ReadAST(boutfile, ReadBJSON, WriteBJSON, a => a).catch(() => 0),
        async () => await ReadAST(outfile).catch(() => 0),
        async () => {
          if((json = await AstDump(file, args))) {
            ast = await Util.instrument(JSON.parse)(json);
            await WriteBJSON(boutfile, ast).catch(err => {
              console.error(err);
              WriteFile(outfile, json);
            });
            return ast;
          }
        }
      ];

      for(let fn of loadFunctions) {
        //console.log('fn:', fn + '');
        if((ast = await fn())) break;
      }
      //console.log("ast:", ast);

      let tree = new Tree(ast);
      let flat = /*tree.flat();*/ deep.flatten(ast,
        new Map(),
        (v, p) =>
          ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 &&
          Util.isObject(v) /*&& 'kind' in v*/
      );
      let locations = [];
      let l = Object.setPrototypeOf({}, { toString() {} });
      //let path = new WeakMap();
      let idmap = {};
      let id2path = {};
      for(let entry of flat) {
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
        if(/(loc|range)/.test(p[p.length - 1] + '')) deep.unset(ast, p);
      }
      ListNodes(params['system-includes']);

      function ListNodes(sysinc = false) {
        let re = /(^[_P]?IMAGE_)/;
        const NoSystemIncludes = ([p, n, l]) => !/^\/usr/.test(l.file + '');
        let entries = [...flat];
        let mainNodes = sysinc ? entries : entries.filter(NoSystemIncludes);

        let typedefs = [...Util.filter(mainNodes, ([path, decl]) => decl.kind == 'TypedefDecl')];

        const names = decls => [...decls].map(([path, decl]) => decl.name);
        const declarations = decls =>
          [...decls].map(([path, decl, loc]) => [decl.name, loc.toString()]);

        if(params.debug) {
          let nodeTypes = [...nodes].map(([p, n]) => n.kind);
          let hist = Util.histogram(nodeTypes, new Map());
          console.log('histogram:', new Map([...hist].sort((a, b) => a[1] - b[1])));
        }

        let namedNodes = mainNodes.filter(([p, n]) => 'name' in n);

        let loc_name = Util.intersect(/*namedNodes
            .filter(([p, n]) => /Decl/.test(n.kind + '') && Util.isNumeric(p[p.length - 1]))
            .map(([p]) => p) ||*/
          typedefs.map(([p]) => p),
          namedNodes.map(([p]) => p)
        ).map(p => [p.split('.'), flat.get(p)]);

        if(params.debug) console.log('loc_name:', loc_name);

        let namedDecls = new Map(loc_name
            .filter(([p, n]) => !/(ParmVar|FieldDecl)/.test(n.kind))
            .map(([p, n]) => [n.name, n])
            .sort((a, b) => a[0].localeCompare(b[0]))
        );

        if(params.debug) console.log('namedDecls:', namedDecls);

        let decls = loc_name
          .map(([p, n]) => [p, n, locations[tree.pathOf(n)]])
          .map(([p, n, l]) => [
            p,
            n,
            n.id || tree.pathOf(n),
            n.name || n.referencedMemberDecl || Object.keys(n).filter(k => typeof n[k] == 'string'),
            GetTypeStr(n),
            n.kind,
            p.join('.').replace(/\.?inner\./g, '/'),
            l + ''
          ]);

        if(params.debug) console.log('loc ∩ name:', loc_name.length);

        for(let decl of decls.filter(([path, node, id, name, type, kind]) => !/ParmVar/.test(kind)
        )) {
          const line = decl
            .slice(2)
            .map((field, i) =>
              (Util.abbreviate(field, [Infinity, Infinity, 20, Infinity, Infinity, Infinity][i]) +
                ''
              ).padEnd([6, 25, 20, 20, 40, 0][i])
            )
            .join(' ');

          console.log(line);
        }
      }
    }
  }
}

Util.callMain(main, true);

function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}

function writeOutput(name, data) {
  let n = data.length;
  let ret = WriteFile(name, data.join('\n'), false);

  /*if(ret > 0)*/ console.log(`Wrote ${n} records to '${name}'.`);
  return ret;
}

function GetLibraryFor(symbolName) {
  if(/BZ2/.test(symbolName)) return 'libbz2.so.1';
  if(/(flate|compress|zlib|gz)/i.test(symbolName)) return 'libz.so.1';
  if(/lzma/i.test(symbolName)) return 'liblzma.so.1';
  if(/brotli/i.test(symbolName)) return 'libbrotli.so.1';
}
