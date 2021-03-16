import Util from './lib/util.js';
import path from './lib/path.js';
import { AcquireReader } from './lib/stream/utils.js';
export let SIZEOF_POINTER = 8;

function FileTime(filename) {
  let st = filesystem.stat(filename);
  return st.mtime ?? st.time;
}

function Newer(file, other) {
  return FileTime(file) > FileTime(other);
}
function Older(file, other) {
  return FileTime(file) < FileTime(other);
}
export class Node {
  static ast2node = new WeakMap();
  static node2ast = new WeakMap();

  constructor(ast) {
    if(typeof ast == 'object') {
      Node.ast2node.set(ast, this);
      Node.node2ast.set(this, ast);
    }
  }

  static get(ast) {
    return Node.ast2node.get(ast);
  }

  get ast() {
    return Node.node2ast.get(this);
  }

  get id() {
    return this.ast.id;
  }
  get loc() {
    return new Location(GetLoc(this.ast));
  }

  inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    const type = Util.className(this);

    return text(type, 1, 31) + ' ' + inspect(Util.getMembers(this), depth, opts);
  }

  toJSON(obj) {
    const { kind } = this;
    obj ??= { kind };
    if(!obj.kind)
      obj.kind = Util.className(this);
    return obj;
  }
}

export class Type extends Node {
  static declarations = new Map();

  constructor(node, ast) {
    let name, desugared, typeAlias, qualType;
    //  console.log('Type', node);

    ast ??= globalThis['$']?.data;

    if(typeof node == 'string') {
      let tmp;
      name = node;
      qualType = node;
      node = null;
      if(Type.declarations.has(name)) {
        node = Type.declarations.get(name).ast;
      }
      // ast ??= globalThis['$']?.data;
      if(ast &&
        typeof (tmp = deep.find(ast, n => typeof n == 'object' && n && n.name == name)) ==
          'object' &&
        tmp != null
      ) {
        //console.log('Type', tmp, name;
        tmp = 'kind' in tmp ? TypeFactory(tmp, ast) : new Type(tmp, ast);

        if(tmp) node = 'ast' in tmp ? tmp.ast : tmp;
      }

      const isPointer = name.endsWith('*');

      if(isPointer) {
        node = { qualType: name, desugaredQualType: 'void *' };
      }
      if(!isPointer && !node) {
        //console.log('Type', node, ast);
        throw new Error(`No such type '${name}'`);
        node = {};
      }
    }

    if(node instanceof Node) {
      console.log('Type', node);
      Util.putStack();
      throw new Error();
    }

    super(node);
    //console.log('node:', node);
    if(node.name) name = node.name;

    if(node.tagUsed && name) name = (node.tagUsed ? node.tagUsed + ' ' : '') + name;
    if(node.kind && node.kind.startsWith('Enum')) name = 'enum ' + name;

    /*if(node.qualType) */ qualType ??= node.qualType;
    if(node.desugaredQualType) desugared = node.desugaredQualType;
    if(node.typeAliasDeclId) typeAlias = node.typeAliasDeclId;

    //console.log("Type", { name, qualType, node });

    if(desugared === qualType) desugared = undefined;
    if(name == '') name = undefined;

    if(name) {
      if(!Type.declarations.has(name)) Type.declarations.set(name, this);
    } else if(qualType) {
      if(!Type.declarations.has(qualType)) Type.declarations.set(qualType, this);
    }

    Util.weakAssign(this, { name, desugared, typeAlias, qualType });

    if(this.isPointer()) {
      let ptr = (name ?? this + '').replace(/\*$/, '').trimEnd();
      //console.log('ptr:', ptr);
      if(ast) {
        let node = deep.find(ast, (n, p) =>
          p.length > 2 ? -1 : typeof n == 'object' && n && n.name == ptr
        )?.value;

        if(node) new Type(node, ast);
      }
    }
  }

  get regExp() {
    return new RegExp(`(?:${this.qualType}${this.typeAlias ? '|' + this.typeAlias : ''})`.replace(/\*/g, '\\*'),
      'g'
      );
  }
  isPointer() {
    let str = this + '';
    return /(\(\*\)\(|\*$)/.test(str);
  }

  isFunction() {
    let str = this + '';
    return /\(.*\)$/.test(str) && !/\(\*\)\(/.test(str);
  }

  isArray() {
    let str = this + '';
    return /\[[0-9]*\]$/.test(str);
  }

  get subscripts() {
    if(this.isArray()) {
      let matches = [...Util.matchAll(/\[([0-9]*)\]/g, this+'')];
      //console.log("matches[0].index", matches[0].index);
      return matches.map(m => [m.index, +m[1]]);
    }
  }

  trimSubscripts() {
    let str = this + '';
    let [[index]] = this.subscripts ?? [[str.length]];
    return str.slice(0, index).trimEnd();
  }

  get pointer() {
    let str = this + '';
    let name;

    name = str.replace(/(\*$|\(\*\))/, '');
    if(name == str) return undefined;

    return name;
    /*
    let match = Util.if(/^([^\(\)]*)(\(?\*\)?\s*)(\(.*\)$|)/g.exec(str),
      (m) => [...m].slice(1),
      () => []
      );
   // console.log("Type.get pointer",{str,match});
    if(match[1]) {
     let name = [match[0].trimEnd(), match[2]].join('');
     if(/^const/.test(name) && !Type.declarations.has(name))
      name = name.replace('const ', '');
    return name;
  }*/
}
  getPointer(ast) {
    const target = this.pointer;

    if(target) {
      let node = deep.find(ast, n => typeof n == 'object' && n && n.name == target);
      //   console.log("getPointer",node);
      if(node) return TypeFactory(node, ast);

      if(Type.declarations.has(target)) return Type.declarations.get(target);
    }
  }

  get unsigned() {
  let str = this + '';
  return /(unsigned|ushort|uint|ulong)/.test(str);
}

  get signed() {
  return /(^|[^n])signed/.test(this+'') || !this.unsigned;
}

  isFloatingPoint() {
    return /(\ |^)(float|double)$/.test(this + '');
  }

  get ffi() {
  const { pointer, size, unsigned,desugared } = this;

  let str = this+'';

  for(const type of [str, desugared])
    if(["void", "sint8", "sint16", "sint32", "sint64", "uint8", "uint16", "uint32", "uint64", "float", "double", "schar", "uchar", "sshort", "ushort", "sint", "uint", "slong", "ulong", "longdouble", "pointer", "int", "long", "short", "char", "size_t", "unsigned char", "unsigned int", "unsigned long", "void *", "char *", "string"].indexOf(type) != -1)
      return type;

    if(pointer == 'char') return 'char *';
    if(size == SIZEOF_POINTER && !this.isPointer())
      return ['', 'unsigned '][unsigned | 0] + 'long';
    // if(size == SIZEOF_POINTER && unsigned) return 'size_t';
    if(size == SIZEOF_POINTER / 2 && !unsigned) return 'int';
    if(size == 4) return ['s', 'u'][unsigned | 0] + 'int32';
    if(size == 2) return ['s', 'u'][unsigned | 0] + 'int16';
    if(size == 1) return ['s', 'u'][unsigned | 0] + 'int8';

    if(this.isPointer()) return 'void *';
    if(size > SIZEOF_POINTER) return 'void *';
    if(size === 0) return 'void';

    if(Type.declarations.has(this + '')) {
      let decl = Type.declarations.get(this + '');
      if(decl.kind == 'EnumDecl') return 'int';
    } else {
      throw new Error(`No ffi type '${this}' ${size}`);
    }
  }

  get size() {
    if(this.isPointer()) return SIZEOF_POINTER;

    const  desugared = this.desugared || this+'' || this.name;
    if(desugared == 'char') return 1;

    const re = /(?:^|\s)_*u?int([0-9]+)(_t|)$/;
    let size, match;
    if((match = re.exec(desugared))) {
      const [, bits] = match;
      if(!isNaN(+bits)) return +bits / 8;
    }
    if((match = /^[^\(]*\(([^\)]*)\).*/.exec(desugared))) {
      if(match[1] == '*') return SIZEOF_POINTER;
    }
    match = /^(unsigned\s+|signed\s+|const\s+|volatile\s+|long\s+|short\s+)*([^\[]*[^ \[\]]) *\[?([^\]]*).*$/g.exec(desugared
      );
    //console.log("match:", match);
    if(match) {
      switch (match[2]) {
        case 'char': 
        size = 1;
        break;
        case 'int8_t':
        case 'uint8_t':
        case 'bool': 
        size = 1;
        break;
        case 'size_t':
        case 'ptrdiff_t':
        case 'void *':
        case 'long': 
        size = SIZEOF_POINTER;
        break;
        case 'int16_t':
        case 'uint16_t':
        case 'short': 
        size = 2;
        break;   
        case 'int32_t': 
        case 'uint32_t':        
        case 'unsigned int':
        case 'int': 
        size = 4;
        break;
        case 'long double': 
        size = 16;
        break;
        case 'int64_t': 
        case 'uint64_t': 
        case 'long long': 
        size = 8;
        break;
        case 'float': 
        size = 4;
        break;
        case 'double': 
        size = 8;
        break;
        case 'void': 
        size = 0;
        break;

      }
      if(size === undefined && match[2].endsWith('*')) size = SIZEOF_POINTER;
      if(size === undefined && (this.qualType || '').startsWith('enum ')) size = 4;
      if(this.isArray()) {
        for(let [index,subscript] of this.subscripts)
          size *= subscript;
      }
    }
    if(size === undefined) {
      // throw new Error(`Type sizeof(${desugared}) ${this.desugaredQualType}`);
      size = NaN;
    }
    return size;
  }

  inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    const { name, size } = this;
    //console.log('Type.inspect:', { name, size, depth, opts });
    let props = Util.getMembers(this);

    if(size) props.size = size;

    return text(Util.className(this), 1, 31) + ' ' + inspect(props, depth, opts);
  }

  /* get [Symbol.toStringTag]() {
    return `${((this.typeAlias && this.typeAlias + '/') || '') + this.name}, ${this.size}${
      (this.pointer && ', ' + this.pointer) || ''
    }`;
  }*/

  [Symbol.toPrimitive](hint) {
    if(hint == 'default' || hint == 'string')
      return (this.qualType ?? this.desugaredQualType ?? '').replace(/\s+(\*+)$/, '$1'); //this+'';
    return this;
  }

  toJSON(obj) {
    const { qualType, size } = this;
    return super.toJSON({ ...obj, qualType, size });
  }
}

Type.declarations.set('void', new Type({ name: 'void' }));
Type.declarations.set('char', new Type({ name: 'char' }));
Type.declarations.set('int', new Type({ name: 'int' }));
Type.declarations.set('short', new Type({ name: 'short' }));
Type.declarations.set('long', new Type({ name: 'long' }));
Type.declarations.set('long long', new Type({ name: 'long long' }));
Type.declarations.set('__int128', new Type({ name: '__int128' }));
Type.declarations.set('unsigned char', new Type({ name: 'unsigned char' }));
Type.declarations.set('unsigned int', new Type({ name: 'unsigned int' }));
Type.declarations.set('unsigned short', new Type({ name: 'unsigned short' }));
Type.declarations.set('unsigned long', new Type({ name: 'unsigned long' }));
Type.declarations.set('unsigned long long', new Type({ name: 'unsigned long long' }));
Type.declarations.set('unsigned __int128', new Type({ name: 'unsigned __int128' }));
Type.declarations.set('float', new Type({ name: 'float' }));
Type.declarations.set('double', new Type({ name: 'double' }));

function RoundTo(value, align) {
  return Math.floor((value + (align - 1)) / align) * align;
}

export class RecordDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    const { tagUsed, name, inner } = node;

    if(tagUsed) this.name = tagUsed + (name ? ' ' + name : '');
    else if(name) this.name = name;

    if(inner?.find(child => child.kind == 'PackedAttr')) this.packed = true;

    let fields = inner?.filter(child => child.kind.endsWith('Decl'));
    //console.log('RecordDecl', fields);

    if(fields)
      this.members = /*new Map*/ fields
        .filter(node => /*!node.isImplicit &&*/ !('parentDeclContextId' in node))
        .map(node => {
          let name = node.name;
          if(node.isBitfield) name += ':1';
          if(node.kind == 'FieldDecl') {
            let type =  new Type(node.type, ast);

            if( type.desugared && type.desugared.startsWith('struct ')) {

              let tmp = ast.inner.find(n => n.kind == 'RecordDecl' && n.name == /^struct./.test(n.name));
              

if(tmp) type = TypeFactory(tmp.value, ast);
            }

            return [name, /*node.type?.kind ? TypeFactory(node.type, ast) :*/type];
          }

          return [name, node.kind.startsWith('Indirect') ? null : TypeFactory(node, ast)];
        });
  }

  get size() {
    let { members = [] } = this;
    return RoundTo([...members].reduce((acc,[name,type]) => {
      if(Number.isFinite(type?.size)) {
            if(type.size == 8)
        acc = RoundTo(acc, 8);
      return acc + RoundTo(type.size,4);
    }
    return acc;
    }, 0), SIZEOF_POINTER);
  }

  toJSON() {
    const { name, size, members } = this;
    return super.toJSON({ name, size, members: members.map(([name,member]) => [name, member != null &&  member.toJSON ? member.toJSON() : member]) });
  }
}

export class EnumDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    if(node.name) this.name = `enum ${node.name}`;

    let constants = node.inner.filter(child => child.kind == 'EnumConstantDecl');
    let number = 1;

    this.members = new Map(constants.map(({ name, type, inner }) => {
        let value = inner ? PrintNode(inner[0]) : number;
        if(!isNaN(+value)) value = +value;
        number = typeof value == 'string' ? `${value} + 1` : value + 1;
        return [name, [new Type(type, ast), value]];
      })
    );
  }
  toJSON() {
    const { name, size, members } = this;
    return super.toJSON({ name, size, members });
  }
}

export class TypedefDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    let inner = (node.inner ?? []).filter(n => !/Comment/.test(n.kind));
    let type = GetType(node, ast);

    //console.log('type:', type);

    Util.assertEqual(inner.length, 1);

    if(type.decl) type = type.decl;
    if(type.kind && type.kind.endsWith('Type')) type = type.type;

    this.name = node.name;
    this.type = type.kind ? TypeFactory(type, ast) : new Type(type, ast);
  }

  get size() {
    return this.type.size;
  }
  toJSON() {
    const { name, size } = this;
    return super.toJSON({ name, size });
  }
}

export class FieldDecl extends Node {
  constructor(node, ast) {
    super(node, ast);
    console.log('FieldDecl', node);

    let inner = (node.inner ?? []).filter(n => !/Comment/.test(n.kind));
    let type = GetType(node, ast);

    Util.assertEqual(inner.length, 1);

    if(type.decl) type = type.decl;
    if(type.kind && type.kind.endsWith('Type')) type = type.type;

    this.name = node.name;
    this.type = type.kind ? TypeFactory(type, ast) : new Type(type, ast);
  }
}

export class FunctionDecl extends Node {
  constructor(node, ast) {
    super(node, ast);

    this.name = node.name;

    if(node.mangledName && node.mangledName != node.name) this.mangledName = node.mangledName;

    let parameters = node.inner?.filter(child => child.kind == 'ParmVarDecl');
    let type = node.type?.qualType;
    let returnType = type.replace(/\s?\(.*/, '');

    let tmp = deep.find(ast, n => typeof n == 'object' && n && n.name == returnType);

    if(tmp) returnType = tmp;

    // console.log('FunctionDecl', { type, returnType,tmp });

    this.returnType = returnType.kind ? TypeFactory(returnType, ast) : new Type(returnType, ast);
    this.parameters =
      parameters && /*new Map*/ parameters.map(({ name, type }) => [name, new Type(type, ast)]);
  }
}

export class BuiltinType extends Type {
  constructor(node, ast) {
    super(node.type, ast);
  }
}

export class PointerType extends Node {
  constructor(node, ast) {
    super(node, ast);
    Util.assertEqual(node.inner.length, 1);

    this.type = new Type(node.type, ast);
    this.pointee = TypeFactory(node.inner[0], ast);
  }
}

export class ConstantArrayType extends Node {
  constructor(node, ast) {
    super(node, ast);
    let elementType = node.inner[0];
    Util.assertEqual(node.inner.length, 1);
    if(elementType.decl) elementType = elementType.decl;
    this.type = new Type(node.type, ast);
    this.elementType = TypeFactory(elementType, ast);
  }
}

export class Location {
  constructor(loc) {
    const { line, col, file } = loc;
    Object.assign(this, { line, col, file });
  }

  inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    return text('Location', 38, 5, 111) + ' [ ' + this.toString() + ' ]';
  }

  toString() {
    const { file, line, col } = this;
    if(file == undefined) return '<builtin>';
    return [file, line, col].join(':');
  }
}

export function TypeFactory(node, ast) {
  let obj;

  // console.log('TypeFactory:', { node });

  Util.assert(node.kind,
    `Not an AST node: ${inspect(node, { colors: false, compact: 0, depth: Infinity })}`
  );

  if((obj = Type.ast2node.get(node))) return obj;

  switch (node.kind) {
    case 'EnumDecl':
      obj = new EnumDecl(node, ast);
      break;
    case 'RecordDecl':
      obj = new RecordDecl(node, ast);
      break;
    case 'FieldDecl':
      obj = new FieldDecl(node, ast);
      break;
    case 'TypedefDecl':
      obj = new TypedefDecl(node, ast);
      break;
    case 'FunctionDecl':
      obj = new FunctionDecl(node, ast);
      break;
    case 'BuiltinType':
      obj = new BuiltinType(node, ast);
      break;
    case 'PointerType':
      obj = new PointerType(node, ast);
      break;
    case 'ConstantArrayType':
      obj = new ConstantArrayType(node, ast);
      break;

    case undefined:
      throw new Error(`Not an AST node: ${inspect(node, { colors: false, compact: 0 })}`);
      break;

    default: throw new Error(`No such kind of AST node: ${node.kind}`);
      //obj = new Type(node, ast);
      break;
  }
  return obj;
}

export async function SpawnCompiler(compiler, input, output, args = []) {
  let base = path.basename(input, /\.[^.]*$/);
  let outputFile = output ?? base + '.ast.json';

  args.push(input);

  if(args.indexOf('-ast-dump=json') != -1) {
    args.unshift(compiler ?? 'clang');
    args = [
      'sh',
      '-c',
      `exec ${args.map(p => (/\ /.test(p) ? `'${p}'` : p)).join(' ')} 1>${outputFile}`
    ];
  } else {
    args.unshift(outputFile);
    args.unshift('-o');
    args.unshift(compiler ?? 'clang');
  }

  console.log(`SpawnCompiler: ${args.map(p => (/\ /.test(p) ? `"${p}"` : p)).join(' ')}`);

  let child = spawn(args, {
    block: false,
    stdio: ['inherit', 'inherit', 'pipe']
  });

  let json = '',
    errors = '';
  let done = false;

  if(Util.platform == 'quickjs') {
    let { fd } = child.stderr;
    console.log('child.stderr:', child.stderr);

    filesystem.setReadHandler(child.stderr, () => {
      ReadErrors(fd);
    });
  } else {
    AcquireReader(child.stderr, async reader => {
      let r;
      while((r = await reader.read())) {
        if(!r.done) errors += r.value.toString();
      }
    });
  }
  let result = /*await*/ child.wait();
  console.log('child.wait():', result);

  filesystem.setReadHandler(child.stderr, null);

  if(result[1] != 0) ReadErrors(child.stderr);
  done = true;
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  errorLines = errorLines.filter(line => /error:/.test(line));
  const numErrors =
    [
      ...(/^([0-9]+)\s/g.exec(errorLines.find(line => /errors\sgenerated/.test(line)) || '0') || [])
    ][0] || errorLines.length;
  console.log('errors:', errors);
  //console.log(`numErrors: ${numErrors}`);
  if(numErrors) throw new Error(errorLines.join('\n'));
  //console.log('errorLines:', errorLines);

  function ReadErrors(fd) {
    let buf = new ArrayBuffer(1024);
    let r = filesystem.read(fd, buf, 0, buf.byteLength);
    errors += filesystem.bufferToString(buf.slice(0, r));
    //console.log('r:', r, 'errors:', errors.length);
  }

  //let fd = filesystem.open(outputFile, filesystem.O_RDONLY);
  return { input: outputFile, result };
}

export async function AstDump(compiler, source, args, force) {
  console.log('AstDump', { compiler, source, args, force });

  let output = path.basename(source, /\.[^.]*$/) + '.ast.json';
  let r;
  if(!force && filesystem.exists(output) && Newer(output, source)) {
    console.log(`Loading cached '${output}'...`);
    r = { file: output };
  } else {
    console.log(`Compiling '${source}'...`);
    r = await SpawnCompiler(compiler, source, output, [
      '-Xclang',
      '-ast-dump=json',
      '-fsyntax-only',
      '-I.',
      ...args
    ]);
  }

  console.log('AstDump', { r });

  //r.size = (await filesystem.stat(r.file)).size;
  r = Util.lazyProperties(r, {
    size() {
      return filesystem.stat(output)?.size;
    },
    json() {
      let json = filesystem.readFile(output);
      return json;
    },
    data() {
      let data = JSON.parse(this.json);
      let file;
      let maxDepth = 0;
      for(let node of data.inner) {
        let loc;
        if((loc = node.loc)) {
          if(loc.file) file = loc.file;
          else loc.file = file;
        }
      }
      return data;
    },
    files() {
      return Util.unique(this.data.inner.map(n => n.loc.file).filter(file => file != undefined));
    }
  });

  return Util.define(r, {
    matchFiles: null,
    nomatchFiles: /^\/usr/,
    filter(pred) {
      return this.data.inner.filter(node =>
          ((node.loc.file !== undefined &&
            ((this.matchFiles && this.matchFiles.test(node.loc.file ?? '')) ||
              !this.nomatchFiles.test(node.loc.file ?? ''))) ||
            node.isUsed) &&
          !node.isImplicit &&
          pred(node)
      );
    },
    get types() {
      return this.filter(n => /(Record|Typedef|Enum)Decl/.test(n.kind));
    },
    get functions() {
      return this.filter(n => /(Function)Decl/.test(n.kind));
    },
    get variables() {
      return this.filter(n => /(Var)Decl/.test(n.kind));
    }
  });
}

export function NodeType(n) {
  return n.type
    ? (t => {
        let { typeAliasDeclId, ...type } = t;
        if(typeof typeAliasDeclId == 'string') type.typeAliasDecl = idNodes.get(typeAliasDeclId);

        if(Type.declarations && Type.declarations.has(t.desugaredQualType)) {
          type = Type.declarations.get(t.desugaredQualType);
        }

        if(Util.isObject(type) && Util.isObject(type.type)) type = type.type;
        return new Type(type);
      })(n.type)
    : NodeType(deep.find(ast, n => Util.isObject(n) && n.type).value);
}

export function NodeName(n, name) {
  if(typeof name != 'string') name = '';
  if(name == '' && n.name) name = n.name;
  if(n.tagUsed) name = n.tagUsed + ' ' + name;
  return name;
}

export function GetLoc(node) {
  let loc;
  if('loc' in node) loc = node.loc;
  else if('range' in node) loc = node.range;
  else return null;
  if('begin' in loc) loc = loc.begin;
  if('expansionLoc' in loc) loc = loc.expansionLoc;

  // if(!('offset' in loc)) return null;

  return loc;
}

export function GetType(node, ast) {
  let type, elaborated;

  //  console.log('GetType', node);

  if((elaborated = node.inner?.find(n => n.kind.endsWith('Type')))) {
    if((type = (elaborated.inner ?? []).find(n => n.decl))) type = type.decl;
    else type = elaborated.ownedTagDecl;
    if(type) {
      //console.log('GetType', { type, id: type.id });
      let declType = ast.inner.find(n => n.id == type.id);
      if(!declType) throw new Error(`Type ${type.id} not found`);
      type = declType;
    }
  } else if(node.ownedTagDecl) {
    type = node.ownedTagDecl;
  } else if(node.inner && node.inner.length == 1 && node.inner[0].kind.endsWith('Type'))
    type = node.inner[0];

  type ??= node.type;
  return type;
}

export function GetTypeStr(node) {
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

export function NodePrinter(ast) {
  let out = '';
  let depth = 0;
  let printer;

  function put(str) {
    out += (str ?? '').replace(/\n/g, '\n' + '  '.repeat(depth));
  }

  function trim() {
    out = out.trimEnd();
  }

  printer = function(node) {
    printer.nodePrinter.ast ??= printer.ast;

    //  console.log('printer()', inspect({ node }, { depth: 1 }));
    printer.print(node);

    return printer;
  };
  printer.ast = ast;

  Object.defineProperties(printer, {
    output: {
      get() {
        return out;
      }
    },
    clear: {
      value() {
        out = '';
      }
    },
    put: { value: put },

    print: {
      value(node) {
        let fn = this.nodePrinter[node.kind];
        let oldlen = out.length;
        if(!fn) throw new Error(`No such printer for ${node.kind}`);
        this.node = node;

        let success = fn.call(this.nodePrinter, node, this.ast);

        let loc = GetLoc(node);
        let location = new Location(node);
        if(out.length == oldlen)
          throw new Error(`Node printer for ${node.kind} failed: ${inspect(
              { ...node, loc },
              {
                depth: 6,
                compact: 2,
                breakLength: 80,
                hideKeys: ['range', 'loc']
              }
            )}`
          );
        // else console.log('out:', out);
        return out;
      }
    },

    nodePrinter: {
      value: new (class NodePrinter {
        AbiTagAttr(abi_tag_attr) {
          put('__attribute__((__abi_tag__))');
        }
        AccessSpecDecl() {}
        AddrLabelExpr() {}
        AliasAttr() {}
        AlignedAttr(aligned_attr) {
          put('__attribute__((aligned))');
        }
        AllocSizeAttr() {}
        AlwaysInlineAttr(always_inline_attr) {
          put('__attribute__((always_inline))');
        }
        ArrayInitIndexExpr() {}
        ArrayInitLoopExpr() {}
        ArraySubscriptExpr(array_subscript_expr) {
          const { valueCategory } = array_subscript_expr;
          const [array, subscript] = array_subscript_expr.inner;
          printer.print(array);
          put('[');
          printer.print(subscript);
          put(']');
        }
        AsmLabelAttr(asm_label_attr) {
          put('__asm__');
        }
        AtomicExpr() {}
        AtomicType() {}
        AutoType() {}
        BinaryOperator(binary_operator) {
          const { valueCategory, opcode } = binary_operator;
          let [left, right] = binary_operator.inner;
          printer.print(left);
          put(` ${opcode} `);
          printer.print(right);
        }
        BlockCommandComment(block_command_comment) {
          const { name } = block_command_comment;

          put(`\\${name}`);
          for(let inner of block_command_comment.inner) printer.print(inner);

          //    trim();
        }
        BreakStmt(break_stmt) {
          put('break');
        }
        BuiltinTemplateDecl() {}
        BuiltinType() {}
        CallbackAttr() {}
        CallExpr(call_expr) {
          let [func, ...args] = call_expr.inner ?? [];
          printer.print(func);
          put('(');
          let i = 0;
          for(let inner of args) {
            if(i++ > 0) put(', ');
            printer.print(inner);
          }
          put(')');
        }
        CaseStmt(case_stmt) {
          put(`case `);
          const [value, ...rest] = case_stmt.inner;

          printer.print(value);
          put(': ');
          for(let node of rest) printer.print(node);
        }
        CharacterLiteral(character_literal) {
          const { value } = character_literal;
          put(`'${String.fromCharCode(value)}'`);
        }
        ClassTemplateDecl() {}
        ClassTemplatePartialSpecializationDecl() {}
        ClassTemplateSpecializationDecl() {}
        ComplexType() {}
        CompoundAssignOperator(compound_assign_operator) {
          const { valueCategory, opcode } = compound_assign_operator;
          let [left, right] = compound_assign_operator.inner;
          printer.print(left);
          put(` ${opcode} `);
          printer.print(right);
        }
        CompoundLiteralExpr() {}
        CompoundStmt(compound_stmt) {
          depth++;
          put('{\n');
          let i = 0;
          for(let inner of compound_stmt.inner ?? []) {
            if(i++ > 0) put('}; \t\n'.indexOf(out[out.length - 1]) != -1 ? '\n' : ';\n');
            printer.print(inner);
          }
          if(out[out.length - 1] != ';') put(';');
          depth--;
          put('\n}');
        }
        ConditionalOperator(conditional_operator) {
          const [cond, if_true, if_false] = conditional_operator.inner;

          printer.print(cond);
          put(' ? ');
          printer.print(if_true);
          put(' : ');
          printer.print(if_false);
        }
        ConstantArrayType() {}
        ConstantExpr(constant_expr) {
          const { valueCategory } = constant_expr;

          for(let inner of constant_expr.inner) printer.print(inner);
        }
        ConstAttr(const_attr) {
          put('__attribute__((const)');
        }
        ConstructorUsingShadowDecl() {}
        ContinueStmt(continue_stmt) {
          put('break');
        }
        ConvertVectorExpr() {}
        CStyleCastExpr(cstyle_cast_expr) {
          let type = new Type(cstyle_cast_expr.type, this.ast);
          const { valueCategory, castKind } = cstyle_cast_expr;
          put(`(${type})`);
          for(let inner of cstyle_cast_expr.inner) printer.print(inner);
        }
        DecayedType() {}
        DeclRefExpr(decl_ref_expr) {
          const { type, valueCategory, referencedDecl } = decl_ref_expr;
          put(referencedDecl.name);
          // printer.print(referencedDecl);
        }
        DeclStmt(decl_stmt) {
          let i = 0;
          let type, baseType;
          for(let inner of decl_stmt.inner) {
            if(!type) {
              type = new Type(inner.type, this.ast);
              baseType = type.trimSubscripts();
              put(`${baseType} `);
            }
            if(i++ > 0) put(', ');
            printer.print(inner, baseType);
          }
        }
        DecltypeType() {}
        DefaultStmt(default_stmt) {
          put('default:');
        }
        DependentNameType() {}
        DependentScopeDeclRefExpr() {}
        DependentSizedArrayType() {}
        DependentTemplateSpecializationType() {}
        DeprecatedAttr(deprecated_attr) {
          put('__attribute__((deprecated))');
        }
        DoStmt(do_stmt) {
          let [body, cond] = do_stmt.inner;
          put(`do `);
          printer.print(body);
          put(` while(`);
          printer.print(cond);
          put(`)`);
        }
        ElaboratedType() {}
        EmptyDecl(empty_decl) {
          put('\n');
          // if(';}'.indexOf(out[out.length - 1] ?? '\n') == -1) put(';');
        }
        EnumConstantDecl(enum_constant_decl) {
          const { name, value } = enum_constant_decl;

          put(name);

          if(value) {
            put(' = ');
            put(value);
          }
        }
        EnumDecl(enum_decl) {
          const { name } = enum_decl;
          //console.log('RecordDecl', record_decl);
          put('enum');
          if(name) {
            put(' ');
            put(name);
          }
          put(' ');
          put('{\n');
          let i = 0;
          for(let field of enum_decl.inner ?? []) {
            printer.print(field, this.ast);
          }
          put('};');
        }
        EnumType() {}
        ExprWithCleanups() {}
        FieldDecl(field_decl) {
          let { isReferenced, name } = field_decl;
          let type = new Type(field_decl.type, this.ast);
          put(type + '');
          put(' ');
          put(name);
          put(';');
        }
        FinalAttr() {}
        FloatingLiteral(floating_literal) {
          put(floating_literal.value);
        }
        FormatArgAttr() {}
        FormatAttr(format_attr) {
          put('__attribute__((format))');
        }
        ForStmt(for_stmt) {
          let inner = [...for_stmt.inner];
          let body = inner.pop();
          let numInit = inner.findIndex(n => n.kind == undefined);
          let init = inner.splice(0, numInit);
          if(inner.length && inner[inner.length - 1].kind == undefined) inner.pop();
          let incr = inner.pop();
          let cond = inner.pop();
          //console.log('ForStmt', console.config({depth: 4, compact: false, maxArrayLength: 5, hideKeys: ['range', 'loc'] }), { body, init, cond, incr });
          put('for(');
          let i = 0;
          for(let n of init) {
            if(i++ > 0) put(', ');
            printer.print(n);
          }
          put(';');
          if(cond.kind) {
            put(' ');
            printer.print(cond);
          }
          put(';');
          if(incr.kind) {
            put(' ');
            printer.print(incr);
          }
          put(') ');
          printer.print(body);
        }
        FriendDecl() {}
        FullComment(full_comment) {
          put('/*');

          for(let inner of full_comment.inner) {
            printer.print(inner);
            //  put('\n');
          }
          put('*/');
          put('\n');
        }
        FunctionDecl(function_decl) {
          const { storageClass, mangledName, isImplicit, isUsed } = function_decl;

          let i = 0;
          if(storageClass) put(storageClass + ' ');

          let node = new FunctionDecl(function_decl, this.ast);
          //console.log("FunctionDecl", node.returnType);
          let returnType = node.returnType; //new Type(node?.ast?.returnType ?? node?.returnType, this.ast);

          put(returnType.name + ' ' + function_decl.name + '(');
          i = 0;
          for(let inner of (function_decl.inner ?? []).filter(n => n.kind == 'ParmVarDecl')) {
            if(i++ > 0) put(', ');
            printer.print(inner);
          }
          put(') ');

          i = 0;
          for(let inner of (function_decl.inner ?? []).filter(n => n.kind != 'ParmVarDecl' && !/Comment/.test(n.kind)
          )) {
            if(i++ > 0) put(' ');
            printer.print(inner);
          }
          // put('');
          return true;
        }
        FunctionNoProtoType() {}
        FunctionProtoType() {}
        FunctionTemplateDecl() {}
        GCCAsmStmt() {}
        GNUInlineAttr() {}
        GNUNullExpr() {}
        GotoStmt(goto_stmt) {
          const { targetLabelDeclId } = goto_stmt;

          let target = deep.find(this.ast,
            n => typeof n == 'object' && n && n.declId == targetLabelDeclId
          )?.value;
          const { name } = target;

          put(`goto ${name}`);
        }
        HTMLEndTagComment(html_end_tag_comment) {
          const { name } = html_end_tag_comment;
          put(`</${name}>`);
        }
        HTMLStartTagComment(html_start_tag_comment) {
          const { name, attrs } = html_start_tag_comment;

          put(`<${name}`);

          for(let [k, v] of attrs) {
            put(` ${k}="${v}"`);
          }
          put(`>`);
        }
        IfStmt(if_stmt) {
          let [cond, body] = if_stmt.inner;
          put(`if(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
        }
        ImplicitCastExpr(implicit_cast_expr) {
          for(let inner of implicit_cast_expr.inner) printer.print(inner);
        }
        ImplicitValueInitExpr() {}
        IncompleteArrayType() {}
        IndirectFieldDecl() {}
        IndirectGotoStmt() {}
        InitListExpr(init_list_expr) {
          const { valueCategory } = init_list_expr;

          put('{ ');
          let i = 0;
          for(let inner of init_list_expr.inner) {
            if(i++ > 0) put(', ');

            printer.print(inner);
          }
          put(' }');
        }
        InjectedClassNameType() {}
        InlineCommandComment(inline_command_comment) {
          put(' InlineCommandComment ');
        }
        IntegerLiteral(integer_literal) {
          put(integer_literal.value);
        }
        LabelStmt(label_stmt) {
          put(`${label_stmt.name}:`);
        }
        LambdaExpr() {}
        LinkageSpecDecl() {}
        LValueReferenceType() {}
        MaterializeTemporaryExpr() {}
        MaxFieldAlignmentAttr() {}
        MayAliasAttr() {}
        MemberExpr(member_expr) {
          const { valueCategory, name, isArray, referencedMemberDecl } = member_expr;
          /*const { referencedDecl } = member_expr.inner[0];
        put(referencedDecl.name);*/
          for(let inner of member_expr.inner) {
            let type = new Type(inner.type, this.ast);

            printer.print(inner);
            put(type.isPointer() ? '->' : '.');
          }
          put(name);
        }
        MemberPointerType() {}
        MinVectorWidthAttr() {}
        ModeAttr() {}
        NamespaceDecl() {}
        NoDebugAttr() {}
        NoInlineAttr() {}
        NonNullAttr(non_null_attr) {
          put('__attribute__((__nonnull__))');
        }
        NonTypeTemplateParmDecl() {}
        NoThrowAttr(no_throw_attr) {
          put('__attribute__((nothrow))');
        }
        NullStmt(null_stmt) {
          put(';');
        }
        OffsetOfExpr() {}
        OpaqueValueExpr() {}
        OverrideAttr() {}
        OwnerAttr() {}
        PackedAttr() {}
        PackExpansionExpr() {}
        PackExpansionType() {}
        ParagraphComment(paragraph_comment) {
          for(let inner of paragraph_comment.inner) {
            printer.print(inner);
            //       put('\n');
          }
          put('\n');
        }
        ParamCommandComment(param_command_comment) {
          const { param } = param_command_comment;
          put(`\\param ${param}`);
          for(let inner of param_command_comment.inner) printer.print(inner);
        }
        ParenExpr(paren_expr) {
          const { valueCategory } = paren_expr;
          put('(');
          for(let inner of paren_expr.inner) printer.print(inner);
          put(')');
        }
        ParenListExpr() {}
        ParenType() {}
        ParmVarDecl(parm_var_decl) {
          let type = Node.get(parm_var_decl.type);
          put((type + '').replace(/\s+\*/g, '*'));

          if(parm_var_decl.name) {
            if(out[out.length - 1] != ' ') put(' ');
            put(parm_var_decl.name);
          }
        }
        PointerAttr() {}
        PointerType() {}
        PredefinedExpr() {}
        PureAttr(pure_attr) {
          put('__attribute__((pure))');
        }
        QualType() {}
        RecordDecl(record_decl) {
          const { tagUsed, name, completeDefinition, parentDeclContextId } = record_decl;
          //console.log('RecordDecl', record_decl);
          put(tagUsed);
          if(name) {
            put(' ');
            put(name);
          }
          put(' ');

          if(record_decl.inner) {
            put('{\n');
            for(let field of record_decl.inner ?? []) {
              put('  ');
              printer.print(field);
              put('\n');
            }
            put('};');
          }
        }
        RecordType() {}
        RestrictAttr(restrict_attr) {
          put('__restrict');
        }
        ReturnsNonNullAttr() {}
        ReturnStmt(return_stmt) {
          put('return ');
          if(return_stmt.inner) for(let inner of return_stmt.inner) printer.print(inner);
          put(';');
        }
        ReturnsTwiceAttr(returns_twice_attr) {
          put('__attribute__((returns_twice))');
        }
        RValueReferenceType() {}
        SentinelAttr() {}
        ShuffleVectorExpr() {}
        SizeOfPackExpr() {}
        StaticAssertDecl() {}
        StmtExpr() {}
        StringLiteral(string_literal) {
          put(string_literal.value);
        }
        SubstNonTypeTemplateParmExpr() {}
        SubstTemplateTypeParmType() {}
        SwitchStmt(switch_stmt) {
          let [cond, body] = switch_stmt.inner;
          put(`switch(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
        }
        TargetAttr() {}
        TemplateArgument() {}
        TemplateSpecializationType() {}
        TemplateTemplateParmDecl() {}
        TemplateTypeParmDecl() {}
        TemplateTypeParmType() {}
        TextComment(text_comment) {
          const { text } = text_comment;

          put(text);
        }
        TParamCommandComment() {}
        TranslationUnitDecl(translation_unit_decl) {
          for(let inner of translation_unit_decl.inner) printer.print(inner);
        }
        TypeAliasDecl() {}
        TypeAliasTemplateDecl() {}
        TypedefDecl(typedef_decl) {
          const { name } = typedef_decl;

          //console.log(`TypedefDecl`, typedef_decl);

          let type = new Type(typedef_decl.inner[0].type, this.ast);

          put('typedef ');
          put(type + '');
          put(' ');
          put(name);
          put(';\n');
        }
        TypedefType() {}
        TypeOfExprType() {}
        TypeTraitExpr() {}
        UnaryExprOrTypeTraitExpr(unary_expr_or_type_trait_expr) {
          const { valueCategory, name } = unary_expr_or_type_trait_expr;

          put(name);
          if(unary_expr_or_type_trait_expr.inner)
            for(let inner of unary_expr_or_type_trait_expr.inner) printer.print(inner);
        }
        UnaryOperator(unary_operator) {
          const { valueCategory, isPostfix, opcode, canOverflow } = unary_operator;
          if(!isPostfix) put(opcode);
          for(let inner of unary_operator.inner) printer.print(inner);
          if(isPostfix) put(opcode);
        }
        UnaryTransformType() {}
        UnresolvedLookupExpr() {}
        UnresolvedMemberExpr() {}
        UnresolvedUsingValueDecl() {}
        UnusedAttr() {}
        UsingDecl() {}
        UsingDirectiveDecl() {}
        UsingShadowDecl() {}
        VAArgExpr() {}
        VarDecl(var_decl, base_type) {
          let type = new Type(var_decl.type, this.ast);
          put(var_decl.name);
          let subscripts = (type.subscripts ?? [])
            .map(([offset, subscript]) => `[${subscript}]`)
            .join('');
          if(subscripts) put(subscripts);
          if(var_decl.inner && var_decl.inner.length) {
            put(' = ');
            for(let inner of var_decl.inner) printer.print(inner);
          }
        }
        VarTemplateDecl() {}
        VectorType() {}
        VerbatimBlockComment(verbatim_block_comment) {
          const { name, closeName } = verbatim_block_comment;
          //console.log('VerbatimBlockComment', verbatim_block_comment);
          put(`\\${name}`);
          for(let inner of verbatim_block_comment.inner) {
            printer.print(inner);
          }
          put(`\\${closeName}`);
        }
        VerbatimBlockLineComment(verbatim_block_line_comment) {
          const { text } = verbatim_block_line_comment;
          put(text);
        }

        VerbatimLineComment() {}
        VisibilityAttr() {}
        WarnUnusedResultAttr(warn_unused_result_attr) {
          put('__attribute__((warn_unused_result))');
        }
        WeakAttr(weak_attr) {
          put('__attribute__((weak))');
        }
        WeakRefAttr() {}
        WhileStmt(while_stmt) {
          let [cond, body] = while_stmt.inner;
          put(`while(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
        }
      })()
    }
  });

  return printer;
}

export function PrintNode(node) {
  let out;
  let sep = ' ';
  switch (node.kind) {
    case 'ConstantExpr':
      out = node.inner.map(PrintNode);
      break;
    case 'IntegerLiteral':
      out = node.value;
      break;
    case 'BinaryOperator':
      out = node.inner.map(PrintNode);
      sep = ` ${node.opcode} `;
      break;
  }
  if(Array.isArray(out)) out = out.join(sep);
  return out;
}

//export default AstDump;
