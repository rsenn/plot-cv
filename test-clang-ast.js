import PortableFileSystem from './lib/filesystem.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableSpawn from './lib/spawn.js';
import { AcquireReader } from './lib/stream/utils.js';
import Util from './lib/util.js';
import path from './lib/path.js';
import fs from 'fs';
import deep from './lib/deep.js';
import Tree from './lib/tree.js';
import { Type, AstDump } from './clang-ast.js';

//prettier-ignore
let filesystem, spawn;

Array.prototype.findLastIndex = function findLastIndex(predicate) {
  for (let i = this.length - 1; i >= 0; --i) {
    const x = this[i];
    if (predicate(x, i, this)) {
      return i;
    }
  }
  return -1;
};

Array.prototype.tail = function tail() {
  return this[this.length - 1];
};
Array.prototype.startsWith = function startsWith(start) {
  for (let i = 0; i < start.length; i++) if (this[i] !== start[i]) return false;
  return true;
};

async function main(...args) {
  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem((fs) => (filesystem = fs));
  await PortableSpawn((fn) => (spawn = fn));

  let params = Util.getOpt(
    {
      output: [true, null, 'o'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      include: [true, (a, p) => (p || []).concat([a]), 'I'],
      define: [true, (a, p) => (p || []).concat([a]), 'D'],
      'no-remove-empty': [false, null, 'E'],
      '@': 'input'
    },
    args
  );
  console.log('main', params);

  let defs = params.define || [];
  let includes = params.include || [];

  args = ['-Xclang', '-ast-dump=json', '-fsyntax-only', '-I.'];
  console.log('main(', ...args, ')');
  const win32 = false;

  if (win32) {
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
  args = args.concat(defs.map((d) => `-D${d}`));
  args = args.concat(includes.map((v) => `-I${v}`));

  console.log('Processing files:', args);

  await processFiles(...params['@']);

  async function processFiles(...files) {
    for (let file of files) {
      let json, ast;
      let outfile = file.replace(/(?:.*\/|)(.*)(?:\.[^.]*|)$/, '$1.ast.json');
      let st = [file, outfile].map((name) => filesystem.stat(name));

      let times = st.map((stat) => stat.mtime);

      if (times[1] >= times[0]) {
        console.log('Reading cached AST from:', outfile);
        json = filesystem.readFile(outfile);
        ast = JSON.parse(json);
      } else {
        json = await AstDump(file, args);
        ast = JSON.parse(json);

        dumpFile(outfile, JSON.stringify(ast, null, 2));
      } /* for(let [node,path] of deep.iterate(ast)) {
   if(Util.isObject(node) && /Comment/.test(node.kind)) 
     deep.unset(ast, path);
 }*/

      let tree = new Tree(ast);
      let flat = /*tree.flat();*/ deep.flatten(
        ast,
        new Map(),
        (v, p) =>
          ['inner', 'loc', 'range'].indexOf(p[p.length - 1]) == -1 &&
          Util.isObject(v) /*&& 'kind' in v*/
      );
      let entries = [...flat];
      let locations = [];
      let l = Object.setPrototypeOf({}, { toString() {} });
      let path = new WeakMap();
      let idmap = {};
      let id2path = {};
      for (let entry of entries) {
        const [p, n] = entry;
        let loc = GetLoc(n);
        if (loc)
          l = Object.setPrototypeOf(
            { ...(l || {}), ...loc },
            {
              toString() {
                let s;
                if (this.file) {
                  s = this.file;
                  if ('line' in this) s += ':' + this.line;
                  if ('col' in this) s += ':' + this.col;
                }
                return s;
              }
            }
          );
        if ('id' in n) {
          idmap[n.id] = entry;
          id2path[n.id] = p;
        }
        //Util.removeKeys(entries[locations.length][1], ['loc','range']);
        entry[2] = l;
        locations.push(l);
      }

      for (let [n, p] of deep.iterate(ast, (v) => Util.isObject(v))) {
        if (/(loc|range)/.test(p[p.length - 1] + '')) {
          //console.log("remove", p);
          deep.unset(ast, p);
        } else {
          //let node = deep.get(ast, p);
          path.set(n, p);
        }
      }

      function Path2Loc(path) {
        if (Array.isArray(path)) path = path.join('.');

        let idx = entries.findLastIndex(([p]) => path.startsWith(p));

        if (idx != -1) return entries[idx][2];
      }

      let indexes = new WeakMap(entries.map(([p, n], i) => [n, i]));
      let re = /(^[_P]?IMAGE_)/;
      const NoSystemIncludes = ([p, n, l]) => !/^\/usr/.test(l.file + '');
      let mainNodes = entries.filter(NoSystemIncludes);

      let typedefs = [...Util.filter(mainNodes, ([path, decl]) => decl.kind == 'TypedefDecl')];

      Type.declarations = new Map(
        [...entries]
          .filter(([p, n]) => Util.isObject(n) && /Decl/.test(n.kind) && n.name)
          .map(([p, n]) => [n.name, n])
      );

      console.log('Type.declarations:', [...Type.declarations.keys()]);
      console.log(
        'Type.declarations. "BrotliDecoderErrorCode" :',
        Type.declarations.get('BrotliDecoderErrorCode')
      );
      console.log(
        'Type.declarations. "BrotliDecoderParameter" :',
        Type.declarations.get('BrotliDecoderParameter')
      );
      //typedefs = NoSystemIncludes(typedefs);

      const names = (decls) => [...decls].map(([path, decl]) => decl.name);
      const declarations = (decls) =>
        [...decls].map(([path, decl, loc]) => [decl.name, loc.toString()]);
      let typenames = names(typedefs.filter(([path, decl]) => re.test(decl.name)));

      //console.log('obj:', obj);
      console.log(
        'typedefs:',
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

      let loc_name = (
        namedNodes
          .filter(([p, n]) => /Decl/.test(n.kind + '') && Util.isNumeric(p[p.length - 1]))
          .map(([p]) => p) ||
        Util.intersect(
          typedefs.map(([p]) => p),
          namedNodes.map(([p]) => p)
        )
      ).map((p) => [p.split('.'), flat.get(p)]);
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
          n.name || n.referencedMemberDecl || Object.keys(n).filter((k) => typeof n[k] == 'string'),
          GetType(n),
          n.kind,
          p.join('.').replace(/\.?inner\./g, '/'),
          l.toString()
        ]);
      console.log('loc ∩ name:', loc_name.length);
      let idNodes = new Map(
        [...tree.filter((node) => typeof node.id == 'string' && node.id.startsWith('0x'))]
          .map(([p, n]) => p)
          .filter((p) => p.indexOf('decl') == -1 && p.indexOf('ownedTagDecl') == -1)
          .map((p) => [p, tree.at(p)])
          .map(([p, n]) => [n.id, n])
      );

      console.log(
        'names:',
        decls
          .filter(([path, node, id, name, type, kind]) => !/ParmVar/.test(kind))
          .map((decl) =>
            decl
              .slice(2)
              .map((field, i) =>
                (
                  Util.abbreviate(
                    field,
                    [Infinity, Infinity, 20, Infinity, Infinity, Infinity][i]
                  ) + ''
                ).padEnd([6, 25, 20, 20, 40, 0][i])
              )
              .join(' ')
          )
          .join('\n')
      );
      let findDecl = (node) =>
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

      let findType = (name) =>
        deep.find(ast, (n, p) => Util.isObject(n) && n.kind && n.name == name, []).value;

      let nodeType = (n) =>
        n.type
          ? ((t) => {
              let { typeAliasDeclId, ...type } = t;
              if (typeof typeAliasDeclId == 'string')
                type.typeAliasDecl = idNodes.get(typeAliasDeclId);

              if (Type.declarations && Type.declarations.has(t.desugaredQualType)) {
                type = Type.declarations.get(t.desugaredQualType);
              }

              if (Util.isObject(type) && Util.isObject(type.type)) type = type.type;
              return new Type(type);
            })(n.type)
          : nodeType(deep.find(ast, (n) => Util.isObject(n) && n.type).value);

      let nodeName = (n, name) => {
        if (typeof name != 'string') name = '';
        if (name == '' && n.name) name = n.name;
        if (n.tagUsed) name = n.tagUsed + ' ' + name;
        return name;
      };
      let findId = (id) =>
        [...tree.filter((node, path) => Util.isObject(node) && node.id == id)][0];
      let findIdKind = (id, kind) =>
        [
          ...tree.filter((node, path) => Util.isObject(node) && node.id == id && node.kind == kind)
        ][0];
      let findNode = (id, kind, exclude) =>
        deep
          .select(
            ast,
            (n, p) => {
              if (n == exclude) return false;
              if (Util.isObject(n)) {
                if (kind && kind != n.kind) return false;
                if (id && id != n.id) return false;
                return true;
              }
            },
            []
          )
          .map(({ path, value }) => [
            path,
            value.inner && value.inner.filter((n) => /Field/.test(n.kind)).length,
            value
          ])
          .filter(([p, len, n]) => len > 0)
          .map(([p, , n]) => n)[0];
      let fieldDecls = (node) => {
        if (node && node.inner) {
          let decls = new Map(
            node.inner
              .filter((n) => /Field/.test(n.kind) || n.name)
              .map((n) => [n.name, nodeType(n)])
              .reduce(
                ([offset, arr], [name, type]) => [
                  offset + type.size,
                  arr.concat([[name, type, offset, type.size]])
                ],
                [0, []]
              )[1]
              .map(([name, type, offset, size]) => [name, [type, offset, size]])
          );
          return decls;
        }
      };
      let structSize = (map) =>
        [...map].reduce((acc, [name, [type, offset, size]]) => acc + size, 0);

      let paramDeclarations = Util.unique(
        [...entries]
          .filter(([p, n]) => Util.isObject(n) && n.kind && n.kind.startsWith('Parm'))
          .map(([p, n]) => {
            let name = nodeName(n);
            let type = nodeType(n);
            return [name, type];
          })
          .filter(([n, t]) => !/unused/i.test(n))
          .filter(([n, t]) => t.pointer && ['void', 'char'].indexOf(t.pointer + '') == -1)
          .map(([n, t]) => [n, [t + '', t.pointer + '']])
          .filter(([n, [t, p]]) => p.endsWith('int'))
        /*.map(([n,[t,p]]) => p)*/
      );

      console.log('paramDeclarations:', paramDeclarations);

      let scalars = new Map(
        decls
          .filter(
            ([path, node, id, name, type, kind]) =>
              kind != 'FunctionDecl' && /(\*)/.test(type) && !/\(.*\)$/.test(type)
          )
          .map(([path, node]) => nodeType(node))
          .map((t) => [t + '', t.pointer])
      );
      console.log('scalars:', scalars);

      let structs = new Map(
        decls
          .filter(
            ([path, node, id, name, type, kind]) =>
              /Typedef/.test(kind) && /(struct|union)/.test(type)
          )
          .map(([path, node, id, name, type, kind]) => [findDecl(node), node])
          .map(([decl, node]) => [node.name, fieldDecls(findNode(decl.id, decl.kind, decl))])
          .filter(([name, value]) => !!value)
          .map(([name, fields]) => [name, [structSize(fields), fields]])
        /* .filter(([name, [size, value]]) => !isNaN(size))*/
      );
      console.log('structs:', structs);
      let prototypes = new Map(
        [...entries]
          .filter(([path, node]) => /FunctionDecl/.test(node.kind))
          .map(([path, node]) => {
            let { name, type, inner } = node;
            inner = inner && inner.map((sub) => [nodeName(sub), nodeType(sub)]);
            type = nodeType(node);
            type = type + '';
            //console.log('type:', type);
            let match;
            if (inner && inner[0]) {
              //console.log("inner[0]", inner[0]);
              if ((match = inner[0][1].regExp.exec(type))) {
                type = type
                  .slice(0, type[match.index - 1] == '(' ? match.index - 1 : match.index)
                  .trimEnd();
              }
            }
            type = new Type(type);
            return [
              name,
              [
                type,
                inner &&
                  new Map(
                    inner.reduce(
                      ([offset, entries], [key, type]) => [
                        offset + type.size,
                        entries.concat([[key, [type, offset]]])
                      ],
                      [0, []]
                    )[1]
                  )
              ]
            ];
          })
      );
      let output = [];
      let libraries = new Map();

      function DefinePrototype(name, retType, params) {
        let ret = Util.tryCatch(
          () => retType.ffi,
          (t) => t,
          null
        );
        let lib = GetLibraryFor(name);
        let libname = lib && lib.replace(/\.so.*/g, '');
        let varname = (n) => `dlsym(RTLD_DEFAULT, '${n}')`;
        if (lib) {
          if (!libraries.has(lib)) {
            libraries.set(lib, libname);
            output.push(`const ${libname} = dlopen('${lib}', RTLD_NOW);`);
          }
          varname = (name) => `dlsym(${libraries.get(lib)}, '${name}')`;
        }
        if (ret) {
          output.push(
            `\ndefine('${name}', ${varname(name)}, null, '${ret}'${[...(params || [])]
              .map(([name, [param, offset]]) => ", '" + param.ffi + "'")
              .join('')});`
          );
          let paramNames = [...(params || [])].map(([name], i) => name || `arg${i}`);
          output.push(
            `export function ${name}(${paramNames.join(', ')}) {
  ${ret == 'void' ? '' : 'return '}call('${name}'${paramNames.map((n) => `, ${n}`).join('')});
}
`
          );
        }
      }
      console.log('output:', output);

      for (let [name, proto] of prototypes) {
        //  console.log("proto:", proto);
        const [retType, params] = proto;
        output.push(DefinePrototype(name, retType, params));
      }

      console.log('prototypes:', prototypes);

      let generateGetSet = (name, offset, size) => [
        `set ${name}(v) { new ${ByteLength2TypedArray(
          size
        )}(this, ${offset})[0] = ${ByteLength2Value(size)}; }`,
        `get ${name}() { return new ${ByteLength2TypedArray(size)}(this, ${offset})[0]; }`
      ];

      // console.log('structs:', structs);
      for (let [name, [size, map]] of structs) {
        output.push('');
        output.push(`class ${name} extends ArrayBuffer {`);
        output.push(`  constructor(obj = {}) {
    super(${size});
    Object.assign(this, obj);
  }`);
        output.push(`  get [Symbol.toStringTag]() { return \`[struct ${name} @ \${this} ]\`; }`);
        let fields = [];
        for (let [name, [type, offset, size]] of map) {
          if (/reserved/.test(name)) continue;
          output = output.concat([
            '',
            `  /* ${name}@${offset} ${type} ${size} */`,
            ...generateGetSet(name, offset, size).map((line) => `  ${line}`)
          ]);
          fields.push(name);
        }
        output = output.concat([
          '',
          `  toString() {
    const { ${fields.join(', ')} } = this;
    return \`struct ${name} {${fields
            .map((field) => '\\n\\t.' + field + ' = ${' + field + '}')
            .join(',')}\\n}\`;
  }`
        ]);
        output.push('}');
      }

      console.log(
        'output: ' +
          output
            .filter((p) => typeof p == 'string')
            .map((p) => (p || '').trimRight())
            .join('\n')
      );

      /*
          switch (decl.kind) {
            case 'TypedefDecl': {
              return [name, `typedef ${type} ${name};`];
            }
            case 'FunctionDecl': {
              // console.log('FunctionDecl', decl);
              break;
              return [name, `function ${name}`];
            }
            case 'EnumConstantDecl': {
              if (!decl.inner) console.log('EnumConstantDecl', { decl });

              const expr = (decl.inner && decl.inner[0]) || decl || {};
              const value = ((expr && expr.inner && expr.inner[0]) || expr || {}).value;
              console.log('EnumConstantDecl', { name, type, value });
              return [name, `enum ${name} ${type} ${value}`];
            }
            case 'RecordDecl': {
              break;
            }
            default: {
              throw new Error(`Node '${decl.kind}' type=${type} name=${name}`);
            }
          }
          return null;
        })
      );*/

      let records = [...tree.filter((node) => node.kind == 'RecordDecl')];

      let getIds = (id, exclude) =>
        [...tree.filter((node, path) => node == id && !path.startsWith(exclude))].map(
          ([p, n]) => p /*.join('.')*/
        );

      let recordNodes = records
        .map(([p, n]) => [
          p.join('.'),
          nodeName(n) || n.id,
          Util.if(n, (p) => [Util.className(p), p.kind, nodeName(p)]),
          Util.if(tree.parentNode(tree.parentNode(tree.parentNode(n))), (p) => [
            Util.className(p),
            p.kind,
            nodeName(p)
          ]),
          [n, ...tree.anchestors(n)]
            .filter((n) => !(n instanceof Array) && n.kind != 'TranslationUnitDecl')
            .reduce(
              (a, n) => [
                ...a,
                ...(typeof n.tagUsed == 'string' && n.tagUsed != '' ? [n.tagUsed] : []),
                ...(typeof n.name == 'string' ? [n.name] : [])
              ],
              []
            ),

          [...tree.anchestors(n, [...p])]
            .filter(([p, n]) => typeof n.kind == 'string' && (/[^t]Decl/.test(n.kind) || n.name))
            .map(([p, n]) => [p.join('.'), n.kind, n.name, n.tagUsed]),
          // getIds(n.id, p.slice(0, 2)),
          nodeType(n),
          n.inner &&
            new Map(
              n.inner
                .reduce(
                  (a, field) =>
                    /Comment/.test(field.kind) ? a : [...a, [nodeName(field), nodeType(field)]],
                  []
                )
                .map(([name, type]) => [
                  name,
                  [
                    type,
                    Util.tryCatch(
                      () => type && type.size,
                      (s) => s
                    )
                  ]
                ])
            )
        ]) /*.map(([p,i,x,y,a1,a2,t,c]) => [p,i,x,y,a1,a2,t && t.size,c])*/
        .filter((a) => a[a.length - 1]);
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
                  `set ${name}(v) { new ${ByteLength2TypedArray(
                    byteLength
                  )}(this, ${offset})[0] = ${ByteLength2Value(byteLength)}; }`,
                  `get ${name}() { return new ${ByteLength2TypedArray(
                    byteLength
                  )}(this, ${offset})[0]; }`
                ]
              ]
            ]
          ],
          [0, []]
        )[1];
        const structSize = [...inner.values()].reduce(
          (a, [type, byteLength]) => (byteLength | 0) + a,
          0
        );

        // console.log('inner:', inner);
        console.log('structSize:', structSize);
      });
      //   console.log('recordNodes:', recordNodes);

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
      let recordIds = recordNodes.map(([p, id]) => id);
      // let idNodes = [...tree.filter(node => typeof node == 'string' && node.startsWith('0x'))].map(([p,n]) => p.slice(0,-1)).map(p => [p, tree.at(p)]);

      /*      console.log(
        'recordNodes:',
        recordNodes.map((n) => n )
      );
*/
      let getId = (id, exclude) => [
        ...tree.filter((node, path) => node.id == id && node != exclude)
      ];

      let recordTypes = [
        ...tree.filter(
          (node, path) => typeof node.id == 'string' && recordIds.indexOf(node.id) != -1
        )
      ].map(([p, n]) => [p, getId(n.id, n).map(([p, n]) => p /*.join('.')*/), n]);

      //  let structs = [...records.map(([path,node]) => [path.slice(0,-2),tree.at(path.slice(0,-2))])];
      //   let fields = [...tree.filter(node => node.kind == 'FieldDecl')];

      /* console.log('records:', records);
    console.log('recordTypes:', recordTypes);*/
      //console.log("fields:",fields);
      //
      console.log('number of nodes:', nodes.size);
      console.log('nodes with offset:', offsetNodes.length);
      console.log(
        'nodes with offset and no line:',
        offsetNodes.filter(([p, n]) => !('line' in n)).length
      );
      console.log(
        'nodes with offset and no file:',
        offsetNodes.filter(([p, n]) => !('file' in n)).length
      );
      console.log('nodes with name:', namedNodes.length);
      function BasePathIndex(path) {
        return path.findIndex((k) => !(k == 'inner' || Util.isNumeric(k)));
      }
      /*  const ids = deep
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
     const refs = ids.map(([path, refId, refKind, refLoc, declId, declKind, declLoc]) => [
      path,
      [refId, refKind, refLoc],
      [declId, declKind, declLoc]
    ]);
 
    const assoc = {};
    0x12cef10;
    const usages = {};
    const getDecl = Util.getOrCreate(assoc, () => new Set());

    refs.forEach(([path, ref, decl]) => {
      if(decl[1] != 'BuiltinType') getDecl(ref[0]).add(decl[0]);
     });*/

      //console.log('assoc:', assoc);

      function FindBackwards(node, pred = ([p, n]) => false) {
        for (let i = indexes.get(node); i >= 0; i--) {
          if (pred(entries[i])) return entries[i][1];
        }
      }

      function GetLoc(node) {
        let loc;
        if ('loc' in node) loc = node.loc;
        else if ('range' in node) loc = node.range;
        else return null; //throw new Error(`no loc in ${tree.pathOf(node)}`);
        if ('expansionLoc' in loc) loc = loc.expansionLoc;
        if ('begin' in loc) loc = loc.begin;

        if (!('offset' in loc)) return null; // throw new Error(`no offset in loc of ${node.kind} ${Util.isEmpty(loc)}`);
        return loc;
      }
      function GetType(node) {
        let type;
        if (node.type) type = node.type;
        else if (
          'inner' in node &&
          node.inner.some((inner) => 'name' in inner || 'type' in inner)
        ) {
          type = node.inner.map((inner) => [inner.name, GetType(inner)]);
          return '{ ' + type.map(([n, t]) => `${t} ${n};`).join(' ') + ' }';
        }
        if (typeof type != 'object') return type;

        if (type.qualType) type = type.qualType;
        return type;
      }
    }
  }
}
Util.callMain(main, true);

function dumpFile(name, data) {
  if (Util.isArray(data)) data = data.join('\n');
  if (typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = filesystem.writeFile(name, data);

  console.log(`Wrote ${name}: ${ret} bytes`);
}

function GetLibraryFor(symbolName) {
  if (/BZ2/.test(symbolName)) return 'libbz2.so.1';
  if (/(flate|compress|zlib|gz)/i.test(symbolName)) return 'libz.so.1';
  if (/lzma/i.test(symbolName)) return 'liblzma.so.1';
  if (/brotli/i.test(symbolName)) return 'libbrotli.so.1';
}
