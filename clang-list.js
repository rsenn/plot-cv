import { AstDump, GetLoc, GetTypeStr } from './clang-ast.js';
import { ReadBJSON, ReadFile, WriteBJSON, WriteFile } from './io-helpers.js';
import { define } from 'util';
import deep from './lib/deep.js';
import * as path from './lib/path.js';
import Tree from './lib/tree.js';

//prettier-ignore
let fs, spawn;

define(Array.prototype, {
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
  },
});

async function main(...args) {
  console.log('main(', ...args, ')');

  let params = getOpt(
    {
      output: [true, null, 'o'],
      xml: [true, null, 'X'],
      json: [true, null, 'j'],
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      debug: [false, null, 'x'],
      'system-includes': [false, null, 's'],
      'no-remove-empty': [false, null, 'E'],
      'output-dir': [true, null, 'd'],
      '@': 'input',
    },
    args,
  );
  console.log('main', params);

  let defs = params.define || [];
  let includes = params.include || [];

  args = [];
  const win32 = false;

  if(win32) {
    defs = defs.concat(
      Object.entries({
        PDWORD: 'unsigned long*',
        UCHAR: 'unsigned char',
        BYTE: 'char',
        TBYTE: 'uint16',
        WORD: 'unsigned short',
        DWORD: 'unsigned long',
        ULONG: 'unsigned long',
        CONST: 'const',
      }),
    );

    args = args.concat([
      '-D_WIN32=1',
      '-DWINAPI=',
      '-D__declspec(x)=',
      '-include',
      '/usr/x86_64-w64-mingw32/include/wtypesbase.h',
      '-I/usr/x86_64-w64-mingw32/include',
    ]);
  }
  console.log('args', { defs, includes });
  args = args.concat(defs.map(d => `-D${d}`));
  args = args.concat(includes.map(v => `-I${v}`));

  console.log('Processing files:', args);

  await processFiles(...params['@']);

  async function processFiles(...files) {
    for(let file of files) {
      const start = /*await now(); //*/ await hrtime();
      console.log('start:', start);
      let json, ast;
      let base = path.basename(file, /\.[^./]*$/);
      let outfile = base + '.ast.json';
      let boutfile = base + '.ast.bjson';

      async function ReadAST(outfile, load = f => ReadFile(f), save = WriteFile, parse = JSON.parse) {
        let st = [file, outfile].map(name => fs.stat(name));
        let times = st.map(stat => (stat && stat.mtime) || 0);
        let cached = times[1] >= times[0];
        if(cached) {
          console.log('Reading cached AST from:', outfile);
          json = /*fs.readFile*/ await load(outfile);
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
            ast = await instrument(JSON.parse)(json);
            await WriteBJSON(boutfile, ast).catch(err => {
              console.error(err);
              WriteFile(outfile, json);
            });
            return ast;
          }
        },
      ];

      for(let fn of loadFunctions) {
        //console.log('fn:', fn + '');
        if((ast = await fn())) break;
      }
      //console.log("ast:", ast);

      let tree = new Tree(ast);
      let flat = /*tree.flat();*/ deep.flatten(
        ast,
        new Map(),
        (v, p) => ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 && isObject(v) /*&& 'kind' in v*/,
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
          l = Object.setPrototypeOf(
            { ...(l || {}), ...loc },
            {
              toString() {
                let s;
                if(this.file) {
                  s = this.file;
                  if('line' in this) s += ':' + this.line;
                  if('col' in this) s += ':' + this.col;
                }
                return s;
              },
            },
          );
        if('id' in n) {
          idmap[n.id] = entry;
          id2path[n.id] = p;
        }
        //removeKeys(entries[locations.length][1], ['loc','range']);
        entry[2] = l;
        locations.push(l);
      }
      for(let [n, p] of deep.iterate(ast, v => isObject(v))) {
        if(/(loc|range)/.test(p[p.length - 1] + '')) deep.unset(ast, p);
      }
      ListNodes(params['system-includes']);

      function ListNodes(sysinc = false) {
        let re = /(^[_P]?IMAGE_)/;
        const NoSystemIncludes = ([p, n, l]) => !/^\/usr/.test(l.file + '');
        let entries = [...flat];
        let mainNodes = sysinc ? entries : entries.filter(NoSystemIncludes);

        let typedefs = [...filter(mainNodes, ([path, decl]) => decl.kind == 'TypedefDecl')];

        const names = decls => [...decls].map(([path, decl]) => decl.name);
        const declarations = decls => [...decls].map(([path, decl, loc]) => [decl.name, loc.toString()]);

        if(params.debug) {
          let nodeTypes = [...nodes].map(([p, n]) => n.kind);
          let hist = histogram(nodeTypes, new Map());
          console.log('histogram:', new Map([...hist].sort((a, b) => a[1] - b[1])));
        }

        let namedNodes = mainNodes.filter(([p, n]) => 'name' in n);

        let loc_name = intersect(
          /*namedNodes
            .filter(([p, n]) => /Decl/.test(n.kind + '') && isNumeric(p[p.length - 1]))
            .map(([p]) => p) ||*/
          typedefs.map(([p]) => p),
          namedNodes.map(([p]) => p),
        ).map(p => [p.split('.'), flat.get(p)]);

        if(params.debug) console.log('loc_name:', loc_name);

        let namedDecls = new Map(
          loc_name
            .filter(([p, n]) => !/(ParmVar|FieldDecl)/.test(n.kind))
            .map(([p, n]) => [n.name, n])
            .sort((a, b) => a[0].localeCompare(b[0])),
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
            l + '',
          ]);

        if(params.debug) console.log('loc ∩ name:', loc_name.length);

        for(let decl of decls.filter(([path, node, id, name, type, kind]) => !/ParmVar/.test(kind))) {
          const line = decl
            .slice(2)
            .map((field, i) =>
              (abbreviate(field, [Infinity, Infinity, 20, Infinity, Infinity, Infinity][i]) + '').padEnd(
                [6, 25, 20, 20, 40, 0][i],
              ),
            )
            .join(' ');

          console.log(line);
        }
      }
    }
  }
}

main(...scriptArgs.slice(1));

/*function WriteFile(name, data, verbose = true) {
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = fs.writeFile(name, data);

  if(verbose) console.log(`Wrote ${name}: ${ret} bytes`);
}*/

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
