import filesystem from 'fs';
import { AstDump, GetLoc, Type } from './clang-ast.js';
import deep from './lib/deep.js';
import * as path from './lib/path.js';
import PortableSpawn from './lib/spawn.js';
import Tree from './lib/tree.js';
//prettier-ignore
let filesystem, spawn;

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
    return this[this.length - 1];
  },
  startsWith(start) {
    for(let i = 0; i < start.length; i++) if(this[i] !== start[i]) return false;
    return true;
  }
});

async function main(...args) {
  console.log('dump-structs', ...args);
  await PortableSpawn(fn => console.log('PortableSpawn', (globalThis.spawn = spawn = fn)));

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
    defs = defs.concat(
      Object.entries({
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

    args = args.concat(['-D_WIN32=1', '-DWINAPI=', '-D__declspec(x)=', '-include', '/usr/x86_64-w64-mingw32/include/wtypesbase.h', '-I/usr/x86_64-w64-mingw32/include']);
  }
  console.log('args', { defs, includes });
  args = args.concat(defs.map(d => `-D${d}`));
  args = args.concat(includes.map(v => `-I${v}`));

  await processFiles(...params['@']);

  async function processFiles(...files) {
    for(let file of files) {
      console.log('Processing file:', file);
      let json, ast;
      let outfile = path.basename(file, /\.[^./]*$/) + '.ast.json';

      let st = [file, outfile].map(name => filesystem.stat(name));

      let times = st.map(stat => (stat && stat.mtime) || 0);

      if(times[1] >= times[0]) {
        console.log('Reading cached AST from:', outfile);
        json = filesystem.readFileSync(outfile);
        ast = JSON.parse(json);
      } else {
        json = await AstDump(file, args);
        ast = JSON.parse(json);

        WriteFile(outfile, JSON.stringify(ast, null, 2));
      }

      let tree = new Tree(ast);
      let flat = /*tree.flat();*/ deep.flatten(ast, new Map(), (v, p) => ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 && isObject(v) /*&& 'kind' in v*/);
      let entries = [...flat];
      let locations = [];
      let l = Object.setPrototypeOf({}, { toString() {} });
      //let path = new WeakMap();
      let idmap = {};
      let id2path = {};
      for(let entry of entries) {
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
              }
            }
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
        if(/(loc|range)/.test(p[p.length - 1] + '')) {
          //console.log("remove", p);
          deep.unset(ast, p);
        } else {
          let indexes = new WeakMap(entries.map(([p, n], i) => [n, i]));
          let re = /(^[_P]?IMAGE_)/;
          const NoSystemIncludes = ([p, n, l]) => !/^\/usr/.test(l.file + '');
          let mainNodes = params['system-includes'] ? entries : entries.filter(NoSystemIncludes);

          let typedefs = [...filter(mainNodes, ([path, decl]) => decl.kind == 'TypedefDecl')];

          Type.declarations = new Map([...entries].filter(([p, n]) => isObject(n) && /Decl/.test(n.kind) && n.name).map(([p, n]) => [n.name, n]));

          //console.log('Type.declarations:', [...Type.declarations.keys()]);
          //typedefs = NoSystemIncludes(typedefs);

          const names = decls => [...decls].map(([path, decl]) => decl.name);
          const declarations = decls => [...decls].map(([path, decl, loc]) => [decl.name, loc.toString()]);
          let typenames = names(typedefs.filter(([path, decl]) => re.test(decl.name)));

          //console.log('obj:', obj);
          //console.log('typedefs:', typedefs.map(([p, n, l]) => [indexes.get(n), n.name, l.toString()]));
          //console.log('typedefs:', declarations(typedefs));

          let nodes = new Map(mainNodes.filter(([p, n]) => 'kind' in n));
          let nodeTypes = [...nodes].map(([p, n]) => n.kind);
          let hist = histogram(nodeTypes, new Map());
          console.log('histogram:', new Map([...hist].sort((a, b) => a[1] - b[1])));

          let offsetNodes = mainNodes.filter(([p, n]) => 'offset' in n);

          let namedNodes = mainNodes.filter(([p, n]) => 'name' in n);

          let loc_name = (
            namedNodes.filter(([p, n]) => /Decl/.test(n.kind + '') && isNumeric(p[p.length - 1])).map(([p]) => p) ||
            intersect(
              typedefs.map(([p]) => p),
              namedNodes.map(([p]) => p)
            )
          ).map(p => [p.split('.'), flat.get(p)]);
          //           console.log('loc_name:', loc_name);
          //
          let namedDecls = new Map(
            loc_name
              .filter(([p, n]) => !/(ParmVar|FieldDecl)/.test(n.kind))
              .map(([p, n]) => [n.name, n])
              .sort((a, b) => a[0].localeCompare(b[0]))
          );

          let decls = loc_name
            .map(([p, n]) => [p, n, locations[indexes.get(n)]])
            .map(([p, n, l]) => [
              p,
              n,
              n.id || indexes.get(n),
              n.name || n.referencedMemberDecl || Object.keys(n).filter(k => typeof n[k] == 'string'),
              GetTypeStr(n),
              n.kind,
              p.join('.').replace(/\.?inner\./g, '/'),
              l.toString()
            ]);
          console.log('loc ∩ name:', loc_name.length);
          let idNodes = new Map(
            [...tree.filter(node => typeof node.id == 'string' && node.id.startsWith('0x'))]
              .map(([p, n]) => p)
              .filter(p => p.indexOf('decl') == -1 && p.indexOf('ownedTagDecl') == -1)
              .map(p => [p, tree.at(p)])
              .map(([p, n]) => [n.id, n])
          );

          console.log(
            'names:',
            decls
              .filter(([path, node, id, name, type, kind]) => !/ParmVar/.test(kind))
              .map(decl =>
                decl
                  .slice(2)
                  .map((field, i) => (abbreviate(field, [Infinity, Infinity, 20, Infinity, Infinity, Infinity][i]) + '').padEnd([6, 25, 20, 20, 40, 0][i]))
                  .join(' ')
              )
              .join('\n')
          );
          let findDecl = node =>
            (
              deep.find(
                node,
                (n, p) => {
                  //console.log('findDecl', p);
                  return ['ownedTagDecl', 'decl'].indexOf(p[p.length - 1]) != -1;
                },
                []
              ) || {}
            ).value;

          let findType = name => deep.find(ast, (n, p) => isObject(n) && n.kind && n.name == name, []).value;

          let nodeType = n =>
            n.type
              ? (t => {
                  let { typeAliasDeclId, ...type } = t;
                  if(typeof typeAliasDeclId == 'string') type.typeAliasDecl = idNodes.get(typeAliasDeclId);

                  if(Type.declarations && Type.declarations.has(t.desugaredQualType)) {
                    type = Type.declarations.get(t.desugaredQualType);
                  }

                  if(isObject(type) && isObject(type.type)) type = type.type;
                  return new Type(type);
                })(n.type)
              : nodeType(deep.find(ast, n => isObject(n) && n.type).value);

          let nodeName = (n, name) => {
            if(typeof name != 'string') name = '';
            if(name == '' && n.name) name = n.name;
            if(n.tagUsed) name = n.tagUsed + ' ' + name;
            return name;
          };
          let findId = id => [...tree.filter((node, path) => isObject(node) && node.id == id)][0];
          let findIdKind = (id, kind) => [...tree.filter((node, path) => isObject(node) && node.id == id && node.kind == kind)][0];
          let findNode = (id, kind, exclude) =>
            deep
              .select(
                ast,
                (n, p) => {
                  if(n == exclude) return false;
                  if(isObject(n)) {
                    if(kind && kind != n.kind) return false;
                    if(id && id != n.id) return false;
                    return true;
                  }
                },
                []
              )
              .map(({ path, value }) => [path, value.inner && value.inner.filter(n => /Field/.test(n.kind)).length, value])
              .filter(([p, len, n]) => len > 0)
              .map(([p, , n]) => n)[0];
          let fieldDecls = node => {
            if(node && node.inner) {
              let decls = new Map(
                node.inner
                  .filter(n => /Field/.test(n.kind) || n.name)
                  .map(n => [n.name, nodeType(n)])
                  .reduce(([offset, arr], [name, type]) => [offset + type.size, arr.concat([[name, type, offset, type.size]])], [0, []])[1]
                  .map(([name, type, offset, size]) => [name, [type, offset, size]])
              );
              return decls;
            }
          };
          let structSize = map => [...map].reduce((acc, [name, [type, offset, size]]) => acc + size, 0);
          /*
      let paramDeclarations = unique([...entries]
          .filter(([p, n]) => isObject(n) && n.kind && n.kind.startsWith('Parm'))
          .map(([p, n]) => {
            let name = nodeName(n);
            let type = nodeType(n);
            return [name, type];
          })
          .filter(([n, t]) => !/unused/i.test(n))
          .filter(([n, t]) => t.pointer && ['void', 'char'].indexOf(t.pointer + '') == -1)
          .map(([n, t]) => [n, [t + '', t.pointer + '']])
          .filter(([n, [t, p]]) => p.endsWith('int'))
       );

      console.log('paramDeclarations:', paramDeclarations);*/

          /*let scalars = new Map(decls
          .filter(([path, node, id, name, type, kind]) => kind != 'FunctionDecl' && /(\*)/.test(type) && !/\(.*\)$/.test(type))
          .map(([path, node]) => nodeType(node))
          .map(t => [t + '', t.pointer])
      );
      console.log('scalars:', scalars);*/

          let structs = new Map(
            decls
              .filter(([path, node, id, name, type, kind]) => /Typedef/.test(kind) && /(struct|union)/.test(type))
              .map(([path, node, id, name, type, kind]) => [findDecl(node), node])
              .map(([decl, node]) => [node.name, fieldDecls(findNode(decl.id, decl.kind, decl))])
              .filter(([name, value]) => !!value)
              .map(([name, fields]) => [name, [structSize(fields), fields]])
            /* .filter(([name, [size, value]]) => !isNaN(size))*/
          );
          if(params.debug) console.log('structs:', structs);
          let prototypes = new Map(
            [...entries]
              .filter(([path, node]) => /FunctionDecl/.test(node.kind))
              .map(([path, node]) => {
                let { name, type, inner } = node;
                inner = inner && inner.map(sub => [nodeName(sub), nodeType(sub)]);
                type = nodeType(node);
                type = type + '';
                //console.log('type:', type);
                let match;
                if(inner && inner[0]) {
                  //console.log("inner[0]", inner[0]);
                  if((match = inner[0][1].regExp.exec(type))) {
                    type = type.slice(0, type[match.index - 1] == '(' ? match.index - 1 : match.index).trimEnd();
                  }
                }
                type = new Type(type);
                return [name, [type, inner && new Map(inner.reduce(([offset, entries], [key, type]) => [offset + type.size, entries.concat([[key, [type, offset]]])], [0, []])[1])]];
              })
          );
          let prototypeOutput = [],
            structOutput = [];
          let libraries = new Map();

          function DefinePrototype(name, retType, params) {
            let ret = tryCatch(
              () => retType.ffi,
              t => t,
              null
            );
            let lib = GetLibraryFor(name);
            let libname = lib && lib.replace(/\.so.*/g, '');
            let varname = n => `dlsym(RTLD_DEFAULT, '${n}')`;
            if(lib) {
              if(!libraries.has(lib)) {
                libraries.set(lib, libname);
                prototypeOutput.push(`const ${libname} = dlopen('${lib}', RTLD_NOW);`);
              }
              varname = name => `dlsym(${libraries.get(lib)}, '${name}')`;
            }
            if(ret) {
              prototypeOutput.push(`\ndefine('${name}', ${varname(name)}, null, '${ret}'${[...(params || [])].map(([name, [param, offset]]) => ", '" + param.ffi + "'").join('')});`);
              let paramNames = [...(params || [])].map(([name], i) => name || `arg${i}`);
              paramNames = paramNames.map(name => (/^__/.test(name) ? name.replace(/^__/, '') : name));
              prototypeOutput.push(`export function ${name}(${paramNames.join(', ')}) {
  ${ret == 'void' ? '' : 'return '}call('${name}'${paramNames.map(n => `, ${n}`).join('')});
}

`);
            }
          }

          for(let [name, proto] of prototypes) {
            //  console.log("proto:", proto);
            const [retType, params] = proto;
            prototypeOutput.push(DefinePrototype(name, retType, params));
          }

          //console.log('prototypes:', prototypes);

          if(params.debug) console.log('structs:', structs);
          for(let [name, struct] of structs) {
            let code = [...GenerateStructClass(name, struct)];

            if(structOutput.length) structOutput.push('');
            structOutput.push(code.join('\n'));
          }

          prototypeOutput = prototypeOutput.filter(record => typeof record == 'string' && record.trim() != '').map(r => r.trimStart());

          if(prototypeOutput.length) writeOutput(params.prototypeOutput ?? MakeFilename('functions'), prototypeOutput);
          if(structOutput.length) writeOutput(params.structOutput ?? MakeFilename('structs'), structOutput);

          //console.log('prototypeOutput: ' + prototypeOutput.filter(p => typeof p != 'string'));

          let records = [...tree.filter(node => node.kind == 'RecordDecl')];

          let getIds = (id, exclude) => [...tree.filter((node, path) => node == id && !path.startsWith(exclude))].map(([p, n]) => p /*.join('.')*/);

          let recordNodes = records
            .map(([p, n]) => [
              p.join('.'),
              nodeName(n) || n.id,
              if(n, p => [className(p), p.kind, nodeName(p)]),
              if(tree.parentNode(tree.parentNode(tree.parentNode(n))), p => [className(p), p.kind, nodeName(p)]),
              [n, ...tree.anchestors(n)]
                .filter(n => !(n instanceof Array) && n.kind != 'TranslationUnitDecl')
                .reduce((a, n) => [...a, ...(typeof n.tagUsed == 'string' && n.tagUsed != '' ? [n.tagUsed] : []), ...(typeof n.name == 'string' ? [n.name] : [])], []),

              [...tree.anchestors(n, [...p])].filter(([p, n]) => typeof n.kind == 'string' && (/[^t]Decl/.test(n.kind) || n.name)).map(([p, n]) => [p.join('.'), n.kind, n.name, n.tagUsed]),
              // getIds(n.id, p.slice(0, 2)),
              nodeType(n),
              n.inner &&
                new Map(
                  n.inner
                    .reduce((a, field) => (/Comment/.test(field.kind) ? a : [...a, [nodeName(field), nodeType(field)]]), [])
                    .map(([name, type]) => [
                      name,
                      [
                        type,
                        tryCatch(
                          () => type && type.size,
                          s => s
                        )
                      ]
                    ])
                )
            ]) /*.map(([p,i,x,y,a1,a2,t,c]) => [p,i,x,y,a1,a2,t && t.size,c])*/
            .filter(a => a[a.length - 1]);
          recordNodes.forEach(([p, n, x, y, a1, a2, t, inner]) => {
            inner = [...inner].reduce(
              ([offset, arr], [name, [type, byteLength]], i) => [
                offset + byteLength,
                [
                  ...arr,
                  [
                    name,
                    [
                      type,
                      byteLength,
                      offset,
                      `set ${name}(v) { new ${ByteLength2TypedArray(byteLength)}(this, ${offset})[0] = ${ByteLength2Value(byteLength)}; }`,
                      `get ${name}() { return new ${ByteLength2TypedArray(byteLength)}(this, ${offset})[0]; }`
                    ]
                  ]
                ]
              ],
              [0, []]
            )[1];
            const structSize = [...inner.values()].reduce((a, [type, byteLength]) => (byteLength | 0) + a, 0);

            // console.log('inner:', inner);
            //console.log('structSize:', structSize);
          });
          //   console.log('recordNodes:', recordNodes);

          let recordIds = recordNodes.map(([p, id]) => id);

          let getId = (id, exclude) => [...tree.filter((node, path) => node.id == id && node != exclude)];

          let recordTypes = [...tree.filter((node, path) => typeof node.id == 'string' && recordIds.indexOf(node.id) != -1)].map(([p, n]) => [p, getId(n.id, n).map(([p, n]) => p /*.join('.')*/), n]);

          console.log('number of nodes:', nodes.size);
          console.log('nodes with offset:', offsetNodes.length);
          console.log('nodes with offset and no line:', offsetNodes.filter(([p, n]) => !('line' in n)).length);
          console.log('nodes with offset and no file:', offsetNodes.filter(([p, n]) => !('file' in n)).length);
          console.log('nodes with name:', namedNodes.length);
          function BasePathIndex(path) {
            return path.findIndex(k => !(k == 'inner' || isNumeric(k)));
          }

          function FindBackwards(node, pred = ([p, n]) => false) {
            for(let i = indexes.get(node); i >= 0; i--) {
              if(pred(entries[i])) return entries[i][1];
            }
          }

          function GetTypeStr(node) {
            let type;
            if(node.type) type = node.type;
            else if('inner' in node && node.inner.some(inner => 'name' in inner || 'type' in inner)) {
              type = node.inner.map(inner => [inner.name, GetTypeStr(inner)]);
              return '{ ' + type.map(([n, t]) => `${t} ${n};`).join(' ') + ' }';
            }
            if(typeof type != 'object') return type;

            if(type.qualType) type = type.qualType;
            return type;
          }
          function MakeFilename(className) {
            return path.join(params['output-dir'] ?? 'tmp', path.basename(file || scriptName(), /\.[^.\/]*$/) + `.${className}.js`);
          }
          //let node = deep.get(ast, p);
          //  path.set(n, p);
        }
      }

      function Path2Loc(path) {
        if(Array.isArray(path)) path = path.join('.');

        let idx = entries.findLastIndex(([p]) => path.startsWith(p));

        if(idx != -1) return entries[idx][2];
      }
    }
  }
}

main(...scriptArgs.slice(1));

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

function* GenerateInspectStruct(type, members, includes) {
  for(let include of ['stdio.h', ...includes]) yield `#include <${include}>`;
  yield `${type} svar;`;
  yield `int main() {`;
  yield `  printf("${type} - %u\\n", sizeof(svar));`;
  for(let member of members) yield `  printf(".${member} %u %u\\n", (char*)&svar.${member} - (char*)&svar, sizeof(svar.${member}));`;
  yield `  return 0;`;
  yield `}`;
}

async function InspectStruct(type, members, includes) {
  const code = [...GenerateInspectStruct(type, members, includes)].join('\n');
  const file = `inspect-${type}-struct.c`;
  WriteFile(file, code);

  let result = await Compile(file);

  console.log('InspectStruct', { file, result });
  return result;
}

function* GenerateStructClass(name, [size, map]) {
  yield `class ${name} extends ArrayBuffer {`;
  yield `  constructor(obj = {}) {\n    super(${size});\n    Object.assign(this, obj);\n  }`;
  yield `  get [Symbol.toStringTag]() { return \`[struct ${name} @ \${this} ]\`; }`;
  let fields = [];
  for(let [name, [type, offset, size]] of map) {
    if(/reserved/.test(name)) continue;
    yield '';
    yield `  /* ${offset}: ${type} ${name}@${size} */`;
    yield* GenerateGetSet(name, offset, size).map(line => `  ${line}`);
    fields.push(name);
  }
  yield '';
  yield `  toString() {\n    const { ${fields.join(', ')} } = this;\n    return \`struct ${name} {${fields.map(field => '\\n\\t.' + field + ' = ${' + field + '}').join(',')}\\n}\`;\n  }`;
  yield '}';
}

function GenerateGetSet(name, offset, size) {
  return [`set ${name}(v) { new ${ByteLength2TypedArray(size)}(this, ${offset})[0] = ${ByteLength2Value(size)}; }`, `get ${name}() { return new ${ByteLength2TypedArray(size)}(this, ${offset})[0]; }`];
}

function ByteLength2TypedArray(byteLength) {
  switch (byteLength) {
    case 1:
      return 'Uint8Array';
    case 2:
      return 'Uint16Array';
    case 4:
      return 'Uint32Array';
    case 8:
      return 'BigUint64Array';
    default:
      return 'Uint8Array';
  }
}

function ByteLength2Value(byteLength) {
  switch (byteLength) {
    case 8:
      return 'BigInt(v)';
    default:
      return 'v';
  }
}