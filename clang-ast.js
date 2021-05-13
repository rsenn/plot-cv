import Util from './lib/util.js';
import path from 'path';
import { AcquireReader } from './lib/stream/utils.js';
import { Predicate } from 'predicate';
export let SIZEOF_POINTER = 8;
export let SIZEOF_INT = 4;

function FileTime(filename) {
  let st = filesystem.stat(filename);
  return st ? st.mtime ?? st.time : -1;
}

function Newer(file, ...other) {
  //console.log('Newer', { file, other });
  return other.every(other => FileTime(file) > FileTime(other));
}
function Older(file, other) {
  return FileTime(file) < FileTime(other);
}

function GetSubscripts(str) {
  let matches = [...Util.matchAll(/\[([0-9]*)\]/g, str)];
  return matches.map(m => [m.index, +m[1]]);
}

function TrimSubscripts(str, sub) {
  let [subscript = [str.length]] = sub ?? GetSubscripts(str) ?? [[str.length]];
  //console.log("subscript:",subscript);
  return str.slice(0, subscript[0]).trimEnd();
}

export class List extends Array {
  constructor(...args) {
    super(...args);
  }

  static get [Symbol.species]() {
    return List;
  }
  get [Symbol.species]() {
    return List;
  }

  filter(callback, thisArg = null) {
    let ret = new List();
    let i = 0;

    if(typeof callback == 'object' &&
      callback != null &&
      callback instanceof RegExp
    ) {
      var re = callback;
      callback = elem => re.test(elem);
    }

    for(let elem of this) {
      if(callback.call(thisArg, elem, i, this)) ret[i] = elem;
      i++;
    }
    return ret;
  }

  get first() {
    return this.find(elem => elem !== undefined);
  }

  entries() {
    let ret = [];
    for(let [i, elem] of super.entries()) {
      if(elem) ret.push([i, elem]);
    }
    return ret;
  }

  keys() {
    let ret = [];
    for(let [i, elem] of super.entries()) {
      if(elem) ret.push(i);
    }
    return ret;
  }
  values() {
    let ret = [];
    for(let [i, elem] of super.entries()) {
      if(elem) ret.push(elem);
    }
    return ret;
  }
}

export class Node {
  static ast2node = new WeakMap();
  static node2ast = new WeakMap();

  constructor(ast) {
    if(typeof ast == 'object') {
      if('path' in ast && 'value' in ast) ast = ast.value;

      //   throw new Error(`Node constructor ${inspect(ast)}`);
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
  get file() {
    const loc = this.ast.loc ?? deep.find(this.ast, (v,k) => k=='loc') ?? deep.find(t.ast, (v,k) => typeof v == 'object' && v != null && 'file' in v);
    if(loc) return loc.file;
  }

  get range() {
    const { range } = this.ast;
 let loc,file;
    if(range) {
      let { begin,end} = range;
      if(!('line' in begin))
        begin.line = (loc ??= this.loc).line;
      return [begin,end].map(r => {
     if(!('file' in r)) 
      r.file = file ??= this.file;
        return new Location(r);
      });
    }
      return [
        new Location(range.begin), 
        new Location(range.end)
      ];
  }

  /*  inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    const type = this.constructor?.name ?? Util.className(this);

    return (text(type, 1, 31) +
      ' ' +
      inspect(this ?? Object.getOwnPropertyNames(this).reduce((acc,name) => ({...acc, [name]: this[name] }), {}), depth, { ...opts, customInspect: true })
    );
  }*/

  toJSON(obj) {
    const { kind } = this;
    obj ??= { kind };
    if(!obj.kind) obj.kind = Util.className(this);
    return obj;
  }

  get [Symbol.toStringTag]() {
    return this.constructor.name;
  }
}

const getTypeFromNode = Util.memoize((node, ast) => new Type(node.type, ast),
  new WeakMap()
);

export class Type extends Node {
  static declarations = new Map();
  static node2type = getTypeFromNode.cache;

  constructor(node, ast) {
    let name, desugared, typeAlias, qualType;

    //console.log('Type', typeof node, typeof node == 'string' ? node : node.kind ?? node.name ?? node.qualType);

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
        typeof (tmp = deep.find(ast,
          n => typeof n == 'object' && n && n.name == name
        )) == 'object' &&
        tmp != null
      ) {
        //console.log('Type', tmp, name;
        tmp = 'kind' in tmp ? TypeFactory(tmp, ast) : new Type(tmp, ast);

        if(tmp) node = 'ast' in tmp ? tmp.ast : tmp;
      }

      const isPointer = name.endsWith('*');

      if(isPointer) {
        node = {
          kind: 'CustomType',
          qualType: name,
          desugaredQualType: 'void *'
        };
      }
      if(!isPointer && !node) {
        let subscripts = GetSubscripts(name);
        name = TrimSubscripts(name, subscripts);
        console.log('Type', { name, subscripts });

        node = GetType(name, ast);

        if(!node) {
          throw new Error(`No such type '${name}'`);
          node = {};
        } else {
          console.log(`Found type ${name}`, node);
        }
      }
    } else {
      if('path' in node && 'value' in node) node = node.value;
    }

    if(node instanceof Node) {
      //console.log('node', Util.className(node), node);
      Util.putStack();
      throw new Error();
    }

    super(node);
    //console.log('node:', node);

    if(node.tagUsed && name)
      name = (node.tagUsed ? node.tagUsed + ' ' : '') + name;
    if(node.kind && node.kind.startsWith('Enum')) name = 'enum ' + name;

    let type = 'type' in node ? node.type : node;

    name = type.name ?? node.name;
    qualType = type.qualType ?? node.qualType;

    desugared = type.desugaredQualType ?? node.desugaredQualType;
    typeAlias = type.typeAliasDeclId ?? node.typeAliasDeclId;

    if(node.inner && node.inner[0] && node.inner[0].kind == 'ElaboratedType') {
      typeAlias ??= node.inner[0]?.ownedTagDecl?.id;

      if(typeAlias) {
        let alias = new Type(ast.inner.find(n => n.id == typeAlias),
          ast
        );
        if(name) alias.name = name;
        if(qualType) alias.qualType = qualType;
        if(desugared) alias.desugared = desugared;
      }
    }

    if(desugared === qualType) desugared = undefined;
    if(name == '') name = undefined;

    if(name) {
      if(!Type.declarations.has(name)) Type.declarations.set(name, this);
    } else if(qualType) {
      if(!Type.declarations.has(qualType))
        Type.declarations.set(qualType, this);
    }

    if(typeAlias) typeAlias = +typeAlias;

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
    } else if(this.isEnum()) {
      this.desugared = 'int';
    }
  }

  get regExp() {
    return new RegExp(`(?:${this.qualType}${this.typeAlias ? '|' + this.typeAlias : ''})`.replace(/\*/g, '\\*'),
      'g'
      );
  }
  isEnum() {
    let str = this.qualType || this + '';
    return /^enum\s/.test(str);
  }
  isPointer() {
    let str = this + '';
    return /(?:\(\*\)\(|\*$)/.test(str);
  }

  isFunction() {
    let str = this + '';
    return /\(.*\)$/.test(str) && !/\(\*\)\(/.test(str);
  }

  isArray() {
    let str = this + '';
    return /\[[0-9]*\]$/.test(str);
  }

  arrayOf() {
    let typeName = this.trimSubscripts();
    return Type.declarations.get(typeName);
  }

  get subscripts() {
    if(this.isArray())
      return GetSubscripts(this+'');
  }

  trimSubscripts() {
    let str = this + '';
    return TrimSubscripts(str);
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
      let node = deep.find(ast,
        n => typeof n == 'object' && n && n.name == target
      );
      //   console.log("getPointer",node);
      if(node) return TypeFactory(node, ast);

      if(Type.declarations.has(target)) return Type.declarations.get(target);
    }
  }

  get unsigned() {
  let str = this + '';
  return /(?:unsigned|ushort|uint|ulong)/.test(str);
}

  get signed() {
  return /(?:^|[^n])signed/.test(this+'') || !this.unsigned;
}

  isCompound() {
    return /(?:struct|union)\s/.test(this + '');
  }

  isFloatingPoint() {
    return /(?:\ |^)(float|double)$/.test(this + '');
  }

  get alias() {
    if(this.typeAlias)
      return Type.get(this.typeAlias);
  }

  get aliases() {
    let type = this;
    let list = [];

    while(type) {
      list.push(type);
      type = type.alias;
    }
    return list;
  }

  isEnum() {
    for(let alias of this.aliases) if(/^enum\s/.test(alias + '')) return true;
    return false;
  }

  get ffi() {
    const {  desugared, qualType } = this;

    let str = this+'';
    if(str == 'char *' || qualType == 'char *'||this.pointer == 'char') return 'char *';

    for(const type of [str, desugared])
      if(["void", "sint8", "sint16", "sint32", "sint64", "uint8", "uint16", "uint32", "uint64", "float", "double", "schar", "uchar", "sshort", "ushort", "sint", "uint", "slong", "ulong", "longdouble", "pointer", "int", "long", "short", "char", "size_t", "unsigned char", "unsigned int", "unsigned long", "void *", "char *", "string"].indexOf(type) != -1)
        return type;
const { size,unsigned } = this;

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
      throw new Error(`No ffi type '${str}' ${size} ${Util.className(this)} ${this.ast.kind}`);
    }
  }

  get size() {
    if(this.isPointer()) return SIZEOF_POINTER;
    if(this.isEnum()) return SIZEOF_INT;
    if(this.isCompound()) {
      let node;
     if((node= Type.get(this+'')))
       return node.size;
    }
          const  desugared = this.desugared || this+'' || this.name;
    if(desugared == 'char') return 1;
    const re = /(?:^|\s)_*u?int([0-9]+)(_t|)$/;
    let size, match;
      if(this.isArray()) {
 size  ??= this.arrayOf()?.size;
        for(let [index,subscript] of this.subscripts)
          size *= subscript;
      }
      
    if((match = re.exec(desugared))) {
      const [, bits] = match;
      if(!isNaN(+bits)) return +bits / 8;
    }
    if((match = /^[^\(]*\(([^\)]*)\).*/.exec(desugared))) {
      if(match[1] == '*') return SIZEOF_POINTER;
    }
    match = /^(unsigned\s+|signed\s+|const\s+|volatile\s+|long\s+|short\s+)*([^\[]*[^ \[\]]) *\[?([^\]]*).*$/g.exec(desugared
      );
  /*  console.log("ast:", this.ast);
    console.log("str:", this+'');
    console.log("qualType:", this.qualType);
    console.log("match:", match);*/
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

    }
    if(size === undefined) {
      // throw new Error(`Type sizeof(${desugared}) ${this.desugaredQualType}`);
      size = NaN;
    }
    return size;
  }

  /* inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    const { name, size } = this;
    let props = {...this} ?? Object.getOwnPropertyNames(this).reduce((acc,name) => ({...acc, [name]: this[name] }), {});
    if(size) props.size = size;
    return (text(this.constructor?.name ?? Util.className(this), 1, 31) +
      ' ' +
      inspect(props, depth, { ...opts, compact: false, customInspect: true, numberBase: 16 })
    );
  }*/

  [Symbol.toPrimitive](hint) {
    if(hint == 'default' || hint == 'string')
      return (this.qualType ??
        this.desugaredQualType ??
        this?.ast?.name ??
        ''
      ).replace(/\s+(\*+)$/, '$1'); //this+'';
    return this;
  }

  toJSON(obj) {
    const { qualType, size } = this;
    return super.toJSON({ ...obj, qualType, size });
  }

  static get(name_or_id, ast = $.data) {
    let type;
    // if(typeof name_or_id == 'string' && (type = Type.declarations.get(name_or_id))) return type;
    let node =
      ast.inner.find(typeof name_or_id == 'number'
          ? node => /(?:Decl|Type)/.test(node.kind) && +node.id == name_or_id
          : node => /(?:Decl|Type)/.test(node.kind) && node.name == name_or_id
      ) ?? GetType(name_or_id, ast);
    if(node) {
      if(node.type) type = getTypeFromNode(node, ast);
      else type = TypeFactory(node, ast);
      if(node.name && typeof name_or_id != 'string') name_or_id = node.name;
    }
    if(typeof name_or_id == 'string' && !Type.declarations.has(name_or_id))
      Type.declarations.set(name_or_id, type);
    return type;
  }
}

Type.declarations.set('void', new Type({ qualType: 'void' }));
Type.declarations.set('char', new Type({ qualType: 'char' }));
Type.declarations.set('int', new Type({ qualType: 'int' }));
Type.declarations.set('short', new Type({ qualType: 'short' }));
Type.declarations.set('long', new Type({ qualType: 'long' }));
Type.declarations.set('long long', new Type({ qualType: 'long long' }));
Type.declarations.set('__int128', new Type({ qualType: '__int128' }));
Type.declarations.set('unsigned char', new Type({ qualType: 'unsigned char' }));
Type.declarations.set('unsigned int', new Type({ qualType: 'unsigned int' }));
Type.declarations.set('unsigned short',
  new Type({ qualType: 'unsigned short' })
);
Type.declarations.set('unsigned long', new Type({ qualType: 'unsigned long' }));
Type.declarations.set('unsigned long long',
  new Type({ qualType: 'unsigned long long' })
);
Type.declarations.set('unsigned __int128',
  new Type({ qualType: 'unsigned __int128' })
);
Type.declarations.set('float', new Type({ qualType: 'float' }));
Type.declarations.set('double', new Type({ qualType: 'double' }));
Type.declarations.set('void *', new Type({ qualType: 'void *' }));
Type.declarations.set('char *', new Type({ qualType: 'char *' }));
Type.declarations.set('const char *', new Type({ qualType: 'const char *' }));

function RoundTo(value, align) {
  return Math.floor((value + (align - 1)) / align) * align;
}

export class RecordDecl extends Type {
  constructor(node, ast) {
    super(node, ast);
    const { id, tagUsed, name, inner } = node;
    if(name && name != 'struct') this.name = name;
    this.name ??= NameFor(node, ast);
    if(inner?.find(child => child.kind == 'PackedAttr')) this.packed = true;
    let fields = inner?.filter(child => child.kind.endsWith('FieldDecl'));

    if(tagUsed) this.tag = tagUsed;

    if(fields)
      this.members = fields
        .filter(node => !('parentDeclContextId' in node))
        .reduce((acc, node) => {
          let { name, kind } = node;
          let type;
          if(node.isBitfield) {
            name += ':' + node.inner[0].inner[0].value;
          }
          if(kind.endsWith('FieldDecl')) {
            if(node?.type?.qualType && / at /.test(node.type.qualType)) {
              let loc = node.type.qualType.split(/(?:\s*[()]| at )/g)[2];
              let [file, line, column] = loc
                .split(/:/g)
                .map(i => (!isNaN(+i) ? +i : i));

              let typePath = PathRemoveLoc(deep.find(inner, n => n.line == line, deep.RETURN_PATH)
              );
              let typeNode = deep.get(inner, typePath);
              type = TypeFactory(typeNode, ast);
              //  console.log('loc:', { kind, file, line, column, typeNode});
            } else if(node.type) {
              type = new Type(node.type, ast);
              if(type.desugared && type.desugared.startsWith('struct ')) {
                let tmp = ast.inner.find(n =>
                    n.kind == 'RecordDecl' && n.name == /^struct./.test(n.name)
                );
                if(tmp) type = TypeFactory(tmp.value, ast);
              }
            }
          }
          if(type) acc.push([name, /*node.kind,*/ type]);
          else if(name)
            acc.push([
              name,
              node.kind.startsWith('Indirect') ? null : TypeFactory(node, ast)
            ]);
          return acc;
          /*  return [
            name,
            node.kind,
            node.kind.startsWith('Indirect') ? null : TypeFactory(node, ast)
          ];*/
        }, []);
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
    return super.toJSON({
      name,
      size,
      members: members.map(([name, member]) => [
        name,
        member != null && member.toJSON ? member.toJSON() : member
      ])
    });
  }
}

export class EnumDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    if(node.name) this.name = `enum ${node.name}`;

    let constants = node.inner.filter(child => child.kind == 'EnumConstantDecl'
    );
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
    let type;

    let { typeAlias } = node;

    let typeId = deep.find(inner, (v, k) => k == 'decl')?.id;

    if(typeAlias) type = ast.inner.find(n => n.id == typeAlias);
    else if(typeId) type = ast.inner.find(n => n.id == typeId);
    else type = node.inner.find(n => /Type/.test(n.kind));

    //type ??= GetType(node, ast);
    Util.assertEqual(inner.length, 1);
    console.log('TypedefDecl.constructor', { node, typeId, type });

    if(type?.decl) type = type.decl;
    if(type?.kind && type.kind.endsWith('Type')) type = type.type;

    this.name = node.name;
    this.type = type.kind ? TypeFactory(type, ast, false) : new Type(type, ast);
    //console.log('TypedefDecl.constructor',     this.type);
  }

  get size() {
    return this.type?.size;
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

    if(node.isBitfield) {
      throw new Error(`isBitfield ${node.kind}`);
    }

    this.name = node.name;
    this.type = type.kind ? TypeFactory(type, ast) : new Type(type, ast);
  }
}

export class FunctionDecl extends Node {
  constructor(node, ast) {
    super(node, ast);

    this.name = node.name;

    if(node.mangledName && node.mangledName != node.name)
      this.mangledName = node.mangledName;

    let parameters = node.inner?.filter(child => child.kind == 'ParmVarDecl');
    let body = node.inner?.find(child => child.kind != 'ParmVarDecl');
    let type = node.type?.qualType;
    let returnType = type.replace(/\s?\(.*/, '');

    // console.log('ast:', ast);

    let tmp = deep.find(ast ?? $.data,
      n => typeof n == 'object' && n && n.name == returnType
    );

    if(tmp) returnType = tmp;

    // console.log('FunctionDecl', { type, returnType,tmp });

    this.returnType = returnType.kind
      ? TypeFactory(returnType, ast)
      : new Type(returnType, ast);
    this.parameters =
      parameters &&
      /*new Map*/ parameters.map(({ name, type }) => [
        name,
        new Type(type, ast)
      ]);

    this.body = body;
  }
}

export class VarDecl extends Node {
  constructor(node, ast) {
    super(node, ast);

    this.name = node.name;

    if(node.mangledName && node.mangledName != node.name)
      this.mangledName = node.mangledName;

    let type = node.type?.qualType;

    this.type = type.kind ? TypeFactory(type, ast) : new Type(type, ast);

    console.log('VarDecl', this);
  }
}
export class ClassDecl extends RecordDecl {}

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
    const { line, col, file, offset } = loc;
    Object.assign(this, { line, col, file, offset });
  }

  inspect(depth, opts = {}) {
    const text = opts.colors
      ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m'
      : t => t;
    return text('Location', 38, 5, 111) + ' [ ' + this.toString() + ' ]';
  }

  toString() {
    const { file, line, col } = this;
    if(file == undefined) return '<builtin>';
    return [file, line, col].join(':');
  }

  [Symbol.toPrimitive](hint) {
    switch (hint) {
      case 'string':
        return this.toString();
      case 'number':
      default: return this.offset;
    }
  }
}

export function TypeFactory(node, ast, cache = true) {
  let obj;

  // console.log('TypeFactory:', { node });

  Util.assert(node.kind,
    `Not an AST node: ${inspect(node, {
      colors: false,
      compact: 0,
      depth: Infinity
    })}`
  );

  if(cache && (obj = Type.ast2node.get(node))) return obj;

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
    case 'CXXRecordDecl':
      obj = new ClassDecl(node, ast);
      break;

    case undefined:
      throw new Error(`Not an AST node: ${inspect(node, { colors: false, compact: 0 })}`
      );
      break;

    default: throw new Error(`No such kind of AST node: ${node.kind}`);
      //obj = new Type(node, ast);
      break;
  }
  return obj;
}

export async function SpawnCompiler(compiler, input, output, args = []) {
  // console.log(`SpawnCompiler`, { compiler, input, output, args });
  let base = path.basename(input, path.extname(input));

  args.push(input);

  if(args.indexOf('-ast-dump=json') != -1) {
    args.unshift(compiler ?? 'clang');
    args = [
      'sh',
      '-c',
      `exec ${args.map(p => (/\ /.test(p) ? `'${p}'` : p)).join(' ')}${
        output ? ` 1>${output}` : ''
      }`
    ];
  } else {
    if(output) {
      args.unshift(output);
      args.unshift('-o');
    }
    args.unshift(compiler ?? 'clang');
  }

  console.log(`SpawnCompiler: ${args.map(p => (/\ /.test(p) ? `"${p}"` : p)).join(' ')}`
  );

  let child = spawn(args, {
    block: false,
    stdio: ['inherit', output ? 'inherit' : 'pipe', 'pipe']
  });

  let json = '',
    errors = '';
  let done = false;

  if(Util.platform == 'quickjs') {
    let { fd } = child.stderr;
    //console.log('SpawnCompiler child.stderr:', child.stderr);
    //console.log('SpawnCompiler child.stdout:', child.stdout);
    await PipeReader(child.stderr.fd, data => (errors += data ?? ''));

    if(child.stdout) {
      output = '';
      await PipeReader(child.stdout.fd, data => (output += data ?? ''));
    }
  } else {
    AcquireReader(child.stderr, async reader => {
      let r;
      while((r = await reader.read())) {
        if(!r.done) errors += r.value.toString();
      }
    });
  }
  let result = await child.wait();
  //console.log('SpawnCompiler child.wait():', result);

  done = true;
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  errorLines = errorLines.filter(line => /error:/.test(line));
  const numErrors =
    [
      ...(/^([0-9]+)\s/g.exec(errorLines.find(line => /errors\sgenerated/.test(line)) || '0'
      ) || [])
    ][0] || errorLines.length;
  //console.log('errors:', errors);
  if(numErrors) throw new Error(errorLines.join('\n'));

  function PipeReader(fd, callback) {
    let ret;
    return new Promise((resolve, reject) => {
      os.setReadHandler(fd, () =>
        ReadPipe(fd, data => {
          if(data) ret = callback(data);
          else resolve(ret);
        })
      );
    });
  }

  function ReadPipe(fd, callback) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    let data;
    if(r > 0) {
      data = filesystem.bufferToString(buf.slice(0, r));
    } else {
      os.setReadHandler(fd, null);
      data = null;
    }
    //console.log('ReadPipe', { fd, r, data });
    callback(data);
  }
  function ReadOutput(fd) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    if(r > 0) {
      output += filesystem.bufferToString(buf.slice(0, r));
      //console.log('r:', r, 'output:', output.length);
    } else {
      os.setReadHandler(fd, null);
    }
  }
  let ret = { output, result, errors: errorLines };
  // console.log('SpawnCompiler return', ret);

  return ret;
}

export async function SourceDependencies(...args) {
  if(args.length < 3) args.unshift(params.compiler);

  let [compiler, source, flags = []] = args;

  //console.log('SourceDependencies', { compiler, source, flags });

  let r = await SpawnCompiler(compiler, source, null, ['-MM', '-I.', ...flags]);
  let { output, result, errors } = (globalThis.response = r);
  output = output.replace(/\s*\\\n\s*/g, ' ');
  let [object, sources] = output.split(/:\s+/);
  sources = sources.trim().split(/ /g);
  let [compilation_unit, ...includes] = sources;

  //console.log('output[1]:', { compilation_unit, includes });

  return sources;
}

export async function AstDump(compiler, source, args, force) {
  compiler ??= 'clang';
  console.log('AstDump', { compiler, source, args, force });
  let output = path.basename(source, /\.[^.]*$/) + '.ast.json';
  let r;
  let sources = await SourceDependencies(compiler, source, args);
  let newer;
  let existsAndNotEmpty =
    filesystem.exists(output) && filesystem.size(output) > 0;

  if(existsAndNotEmpty) newer = Newer(output, ...sources);

  console.log('AstDump', console.config({ maxArrayLength: 4, compact: 10 }), {
    output,
    source,
    sources,
    existsAndNotEmpty,
    newer
  });
  if(!force && existsAndNotEmpty && newer) {
    console.log(`Loading cached '${output}'...`);
    r = { file: output };
  } else {
    if(filesystem.exists(output)) filesystem.unlink(output);

    console.log(`Compiling '${source}'...`);
    r = await SpawnCompiler(compiler, source, output, [
      '-Xclang',
      '-ast-dump=json',
      '-fsyntax-only',
      '-I.',
      ...args
    ]);
  }

  console.log('AstDump', r);

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
        let range;

        const SetFile = loc => {
          try {
            if(loc) {
              if(loc.file) file = loc.file;
              else loc.file = file;
            }
          } catch(e) {}
        };
        SetFile(node.loc);
        SetFile(node.range?.begin);
        SetFile(node.range?.end);
      }
      return data;
    },
    files() {
      return Util.unique(this.data.inner.map(n => n.loc.file).filter(file => file != undefined)
      );
    }
  });

  r = Util.define(r, {
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
    }
  });
  r = Util.lazyProperties(r, {
    types() {
      return Object.setPrototypeOf(this.filter(n => /(?:Record|Typedef|Enum)Decl/.test(n.kind)),
        List.prototype
      );
    },
    functions() {
      return Object.setPrototypeOf(this.filter(n => /(?:Function)Decl/.test(n.kind)),
        List.prototype
      );
    },
    namespaces() {
      return Object.setPrototypeOf(deep.select(
          this.data,
          n => 'NamespaceDecl' == n.kind,
          deep.RETURN_VALUE,
          10
        ),
        List.prototype
      );
    },
    classes() {
      let predicate =
        (Predicate.property('kind', Predicate.equal('CXXRecordDecl')),
        Predicate.not(Predicate.property('isImplicit', Predicate.equal(true))));
      //predicate = n => 'CXXRecordDecl' == n.kind && !n.isImplicit;
      return Object.setPrototypeOf(deep.select(this.data, predicate, deep.RETURN_VALUE, 10),
        List.prototype
      );
    },
    variables() {
      return Object.setPrototypeOf(this.filter(n => /(?:Var)Decl/.test(n.kind)),
        List.prototype
      );
    }
  });
  return r;
}

export function NameFor(decl, ast = this.data) {
  const { id } = decl;
  let p;
  if((p = deep.find(
      ast,
      (value, key) => key == 'ownedTagDecl' && value.id == id,
      deep.RETURN_PATH
    ))
  ) {
    p = p.slice(0, -1);

    let node = deep.get(ast, p);
    let parent = deep.get(ast, p.slice(0, -2));

    if(parent.kind == 'TypedefDecl' && parent.name) return parent.name;

    console.log('p:', p, 'node:', node, 'parent:', parent);

    return node?.type?.desugaredQualType;
  }
}

export function NodeType(n) {
  return n.type
    ? (t => {
        let { typeAliasDeclId, ...type } = t;
        if(typeof typeAliasDeclId == 'string')
          type.typeAliasDecl = idNodes.get(typeAliasDeclId);

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

/*export function GetType(node, ast) {
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
}*/

export function GetTypeNode(node, ast = $.data) {
  for(let n = [node]; n[0]; n = n[0].inner) {
    let i;

    if((i = n.find(node => /Type/.test(node.kind))))
      if(i?.decl?.id) return ast.inner.find(node => node.id == i.decl.id);
  }
}

export function GetTypeStr(node) {
  let type;
  if(node.type) type = node.type;
  else if('inner' in node &&
    node.inner.some(inner => 'name' in inner || 'type' in inner)
  ) {
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
    out += (str ?? '').replaceAll('\n', '\n' + '  '.repeat(depth));
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

        if(loc?.line !== undefined) {
          const { line, column } = loc;
          this.loc = {
            line,
            column,
            toString() {
              return [line, column].filter(i => i).join(':');
            }
          };
        }
        if(out.length == oldlen) {
          console.log('printer error', { loc, location }, this.loc);
          throw new Error(`Node printer for ${node.kind} (${this.loc}) failed: ${inspect(
              { ...node, loc },
              {
                depth: 10,
                compact: false,
                breakLength: 80,
                hideKeys: ['range', 'loc']
              }
            )}`
          );
        }
        // else console.log('out:', out);
        return out;
      }
    },

    nodePrinter: {
      value: new (class NodePrinter {
        AbiTagAttr(abi_tag_attr) {
          put('__attribute__((__abi_tag__))');
        }
        AccessSpecDecl(access_spec_decl) {
          const { access } = access_spec_decl;
          //console.log("AccessSpecDecl",access_spec_decl)
          put(access + ':');
        }
        AlignedAttr(aligned_attr) {
          put('__attribute__((aligned))');
        }
        AlwaysInlineAttr(always_inline_attr) {
          put('__attribute__((always_inline))');
        }
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
        CallExpr(call_expr) {
          console.log('CallExpr', call_expr);
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
        CompoundAssignOperator(compound_assign_operator) {
          const { valueCategory, opcode } = compound_assign_operator;
          let [left, right] = compound_assign_operator.inner;
          printer.print(left);
          put(` ${opcode} `);
          printer.print(right);
        }
        CompoundStmt(compound_stmt) {
          depth++;
          put('{');
          let i = 0;
          let offset = out.length;
          if(compound_stmt?.inner?.length) put('\n');

          for(let inner of compound_stmt.inner ?? []) {
            if(i++ > 0)
              put('}; \t\n'.indexOf(out[out.length - 1]) != -1 ? '\n' : ';\n');
            printer.print(inner);
          }
          //console.log('CompoundStmt', { out });
          if(out.length > offset && out[out.length - 1] != ';') put(';');
          depth--;
          //if(out.length > offset)
          if(compound_stmt?.inner?.length) put('\n');
          put('}\n');
        }
        ConditionalOperator(conditional_operator) {
          const [cond, if_true, if_false] = conditional_operator.inner;

          printer.print(cond);
          put(' ? ');
          printer.print(if_true);
          put(' : ');
          printer.print(if_false);
        }
        ConstantExpr(constant_expr) {
          const { valueCategory } = constant_expr;

          for(let inner of constant_expr.inner) printer.print(inner);
        }
        ConstAttr(const_attr) {
          put('__attribute__((const)');
        }
        ContinueStmt(continue_stmt) {
          put('break');
        }
        CStyleCastExpr(cstyle_cast_expr) {
          let type = new Type(cstyle_cast_expr.type, this.ast);
          const { valueCategory, castKind } = cstyle_cast_expr;
          put(`(${type})`);
          for(let inner of cstyle_cast_expr.inner) printer.print(inner);
        }
        DeclRefExpr(decl_ref_expr) {
          const { type, valueCategory, referencedDecl } = decl_ref_expr;
          put(referencedDecl.name ?? '<DeclRefExpr>');

          // printer.print(referencedDecl);
        }
        DeclStmt(decl_stmt) {
          let i = 0;
          let type, baseType;
          for(let inner of decl_stmt.inner) {
            try {
              if(!type && inner.type) {
                type = new Type(inner.type, this.ast);
                //console.log('type:', type);
                //console.log('type.typeAlias:', type.typeAlias);
                //console.log('type.trimSubscripts():', type.trimSubscripts());
                baseType = type.trimSubscripts() ?? type.qualType;
                put(`${baseType} `);
              }
            } catch(err) {}
            if(i++ > 0) put(', ');
            printer.print(inner, baseType);
          }
        }
        DefaultStmt(default_stmt) {
          put('default:');
        }
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
        FieldDecl(field_decl) {
          let { isReferenced, name } = field_decl;
          let type = new Type(field_decl.type, this.ast);
          put(type + '');
          put(' ');
          put(name);
          put(';');
        }
        FloatingLiteral(floating_literal) {
          put(floating_literal.value);
        }
        FormatAttr(format_attr) {
          put('__attribute__((format))');
        }
        ForStmt(for_stmt) {
          let inner = [...for_stmt.inner];
          let body = inner.pop();
          let numInit = inner.findIndex(n => n.kind == undefined);
          let init = inner.splice(0, numInit);
          if(inner.length && inner[inner.length - 1].kind == undefined)
            inner.pop();
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
          const {
            storageClass,
            mangledName,
            isImplicit,
            isUsed
          } = function_decl;
          let i = 0;
          if(storageClass) put(storageClass + ' ');
          let node = new FunctionDecl(function_decl, this.ast);
          //console.log('FunctionDecl', node.returnType);
          let returnType = node.returnType;
          put(returnType + '\n' + function_decl.name + '(');
          i = 0;
          for(let inner of (function_decl.inner ?? []).filter(n => n.kind == 'ParmVarDecl'
          )) {
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
        InlineCommandComment(inline_command_comment) {
          put(' InlineCommandComment ');
        }
        IntegerLiteral(integer_literal) {
          put(integer_literal.value);
        }
        LabelStmt(label_stmt) {
          put(`${label_stmt.name}:`);
        }
        MemberExpr(member_expr) {
          const {
            valueCategory,
            name,
            isArray,
            referencedMemberDecl
          } = member_expr;
          /*const { referencedDecl } = member_expr.inner[0];
        put(referencedDecl.name);*/
          for(let inner of member_expr.inner) {
            let type = new Type(inner.type, this.ast);

            printer.print(inner);
            put(type.isPointer() ? '->' : '.');
          }
          put(name);
        }
        NonNullAttr(non_null_attr) {
          put('__attribute__((__nonnull__))');
        }
        NoThrowAttr(no_throw_attr) {
          put('__attribute__((nothrow))');
        }
        NullStmt(null_stmt) {
          put(';');
        }
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
        ParenListExpr(paren_list_expr) {
          const { type, valueCategory } = paren_list_expr;

          put(`(`);

          if(paren_list_expr.inner)
            for(let inner of paren_list_expr.inner) printer.print(inner);
          put(`)`);
        }
        ParmVarDecl(parm_var_decl) {
          let {
            name,
            type: { qualType: type }
          } = parm_var_decl;

          put((type + '').replace(/\s+\*/g, '*'));

          if(name) {
            if(out[out.length - 1] != ' ') put(' ');
            put(name);
          }

          if(parm_var_decl.inner) {
            put(` = `);
            for(let inner of parm_var_decl.inner) printer.print(inner);
          }
        }
        PureAttr(pure_attr) {
          put('__attribute__((pure))');
        }
        RecordDecl(record_decl) {
          const {
            tagUsed,
            name,
            completeDefinition,
            parentDeclContextId
          } = record_decl;
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
        RestrictAttr(restrict_attr) {
          put('__restrict');
        }
        ReturnStmt(return_stmt) {
          put('return ');
          if(return_stmt.inner)
            for(let inner of return_stmt.inner) printer.print(inner);
          put(';');
        }
        ReturnsTwiceAttr(returns_twice_attr) {
          put('__attribute__((returns_twice))');
        }
        StringLiteral(string_literal) {
          put(string_literal.value);
        }
        SwitchStmt(switch_stmt) {
          let [cond, body] = switch_stmt.inner;
          put(`switch(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
        }
        TextComment(text_comment) {
          const { text } = text_comment;

          put(text);
        }
        TranslationUnitDecl(translation_unit_decl) {
          for(let inner of translation_unit_decl.inner) printer.print(inner);
        }
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
        UnaryExprOrTypeTraitExpr(unary_expr_or_type_trait_expr) {
          const { valueCategory, name } = unary_expr_or_type_trait_expr;

          put(name);
          if(unary_expr_or_type_trait_expr.inner)
            for(let inner of unary_expr_or_type_trait_expr.inner)
              printer.print(inner);
        }
        UnaryOperator(unary_operator) {
          const {
            valueCategory,
            isPostfix,
            opcode,
            canOverflow
          } = unary_operator;
          if(!isPostfix) put(opcode);
          for(let inner of unary_operator.inner) printer.print(inner);
          if(isPostfix) put(opcode);
        }
        UnresolvedLookupExpr(unresolved_lookup_expr) {
          const {
            type,
            valueCategory,
            usesADL,
            name,
            lookups
          } = unresolved_lookup_expr;
          put(name);
        }
        UnresolvedMemberExpr(unresolved_member_expr) {
          const { type, valueCategory } = unresolved_member_expr;
        }
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

        WarnUnusedResultAttr(warn_unused_result_attr) {
          put('__attribute__((warn_unused_result))');
        }
        WeakAttr(weak_attr) {
          put('__attribute__((weak))');
        }
        WhileStmt(while_stmt) {
          let [cond, body] = while_stmt.inner;
          put(`while(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
        }

        OverrideAttr(override_attr) {
          put('override');
        }

        AddrLabelExpr(addr_label_expr) {}
        AliasAttr(alias_attr) {}
        AllocSizeAttr(alloc_size_attr) {}
        ArrayInitIndexExpr(array_init_index_expr) {}
        ArrayInitLoopExpr(array_init_loop_expr) {}
        AtomicExpr(atomic_expr) {}
        AtomicType(atomic_type) {}
        AutoType(auto_type) {}
        BuiltinTemplateDecl(builtin_template_decl) {}
        BuiltinType(builtin_type) {}
        CallbackAttr(callback_attr) {}
        ClassTemplateDecl(class_template_decl) {}
        ClassTemplatePartialSpecializationDecl(class_template_partial_specialization_decl
        ) {}
        ClassTemplateSpecializationDecl(class_template_specialization_decl) {}
        ComplexType(complex_type) {}
        CompoundLiteralExpr(compound_literal_expr) {}
        ConstantArrayType(constant_array_type) {}
        ConstructorUsingShadowDecl(constructor_using_shadow_decl) {}
        ConvertVectorExpr(convert_vector_expr) {}
        DecayedType(decayed_type) {}
        DecltypeType(decltype_type) {}
        DependentNameType(dependent_name_type) {}
        DependentScopeDeclRefExpr(dependent_scope_decl_ref_expr) {
          const { type, valueCategory } = dependent_scope_decl_ref_expr;
        }
        DependentSizedArrayType(dependent_sized_array_type) {}
        DependentTemplateSpecializationType(dependent_template_specialization_type
        ) {}
        ElaboratedType(elaborated_type) {}
        EnumType(enum_type) {}
        ExprWithCleanups(expr_with_cleanups) {}
        FinalAttr(final_attr) {}
        FormatArgAttr(format_arg_attr) {}
        FriendDecl(friend_decl) {}
        FunctionNoProtoType(function_no_proto_type) {}
        FunctionProtoType(function_proto_type) {}
        FunctionTemplateDecl(function_template_decl) {}
        GCCAsmStmt(gcc_asm_stmt) {}
        GNUInlineAttr(gnu_inline_attr) {}
        GNUNullExpr(gnu_null_expr) {}
        ImplicitValueInitExpr(implicit_value_init_expr) {}
        IncompleteArrayType(incomplete_array_type) {}
        IndirectFieldDecl(indirect_field_decl) {}
        IndirectGotoStmt(indirect_goto_stmt) {}
        InjectedClassNameType(injected_class_name_type) {}
        LambdaExpr(lambda_expr) {}
        LinkageSpecDecl(linkage_spec_decl) {}
        LValueReferenceType(l_value_reference_type) {}
        MaterializeTemporaryExpr(materialize_temporary_expr) {}
        MaxFieldAlignmentAttr(max_field_alignment_attr) {}
        MayAliasAttr(may_alias_attr) {}
        MemberPointerType(member_pointer_type) {}
        MinVectorWidthAttr(min_vector_width_attr) {}
        ModeAttr(mode_attr) {}
        NamespaceDecl(namespace_decl) {}
        NoDebugAttr(no_debug_attr) {}
        NoInlineAttr(no_inline_attr) {}
        NonTypeTemplateParmDecl(non_type_template_parm_decl) {}
        OffsetOfExpr(offset_of_expr) {}
        OpaqueValueExpr(opaque_value_expr) {}
        OwnerAttr(owner_attr) {}
        PackedAttr(packed_attr) {}
        PackExpansionExpr(pack_expansion_expr) {}
        PackExpansionType(pack_expansion_type) {}
        ParenType(paren_type) {}
        PointerAttr(pointer_attr) {}
        PointerType(pointer_type) {}
        PredefinedExpr(predefined_expr) {}
        QualType(qual_type) {}
        RecordType(record_type) {}
        ReturnsNonNullAttr(returns_non_null_attr) {}
        RValueReferenceType(r_value_reference_type) {}
        SentinelAttr(sentinel_attr) {}
        ShuffleVectorExpr(shuffle_vector_expr) {}
        SizeOfPackExpr(size_of_pack_expr) {}
        StaticAssertDecl(static_assert_decl) {}
        StmtExpr(stmt_expr) {}
        SubstNonTypeTemplateParmExpr(subst_non_type_template_parm_expr) {}
        SubstTemplateTypeParmType(subst_template_type_parm_type) {}
        TargetAttr(target_attr) {}
        TemplateArgument(template_argument) {}
        TemplateSpecializationType(template_specialization_type) {}
        TemplateTemplateParmDecl(template_template_parm_decl) {}
        TemplateTypeParmDecl(template_type_parm_decl) {}
        TemplateTypeParmType(template_type_parm_type) {}
        TParamCommandComment(t_param_command_comment) {}
        TypeAliasDecl(type_alias_decl) {}
        TypeAliasTemplateDecl(type_alias_template_decl) {}
        TypedefType(typedef_type) {}
        TypeOfExprType(type_of_expr_type) {}
        TypeTraitExpr(type_trait_expr) {}
        UnaryTransformType(unary_transform_type) {}
        UnresolvedUsingValueDecl(unresolved_using_value_decl) {}
        UnusedAttr(unused_attr) {}
        UsingDecl(using_decl) {}
        UsingDirectiveDecl(using_directive_decl) {}
        UsingShadowDecl(using_shadow_decl) {}
        VAArgExpr(va_arg_expr) {}
        VarTemplateDecl(var_template_decl) {}
        VectorType(vector_type) {}
        VerbatimLineComment(verbatim_line_comment) {}
        VisibilityAttr(visibility_attr) {}
        WeakRefAttr(weak_ref_attr) {}

        CXXRecordDecl(cxx_record_decl) {
          const { name, tagUsed } = cxx_record_decl;
          depth++;

          put(`${tagUsed} ${name} {\n`);

          if(cxx_record_decl.inner) {
            let i = -1;
            for(let inner of cxx_record_decl.inner) {
              i++;
              if(inner.kind &&
                (inner.kind.endsWith('Comment') ||
                  inner.kind == 'CXXRecordDecl')
              )
                continue;
              //  console.log(`CXXRecordDecl inner[${i}]`, inner);
              printer.print(inner);
              put(`\n`);
            }
          }
          depth--;
          put(`\n}`);
        }
        CXXConstructorDecl(cxx_constructor_decl) {
          const { name, type } = cxx_constructor_decl;
          let i = 0,
            param,
            initializer;
          let l = cxx_constructor_decl?.inner
            ? [...cxx_constructor_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')
              )
            : [];
          put(`${name}(`);
          while(l.length &&
            l[0].kind == 'ParmVarDecl' &&
            (param = l.shift())
          ) {
            if(param.name) {
              if(i > 0) put(', ');
              printer.print(param);
              i++;
            }
          }
          put(`)`);
          depth++;
          if(l.length && l[0].kind == 'CXXCtorInitializer') {
            put(`\n: `);
            i = 0;
            while(l.length &&
              l[0].kind == 'CXXCtorInitializer' &&
              (initializer = l.shift())
            ) {
              if(i++ > 0) put('\n, ');
              let implicit_cast = initializer.inner[0];
              if(implicit_cast?.inner) {
                let member_expr = implicit_cast.inner[0];

                put(member_expr.name + `(`);
                if(member_expr?.inner) printer.print(member_expr.inner[0]);

                put(`)`);
              }
            }
          }
          depth--;
          if(l.length) {
            put(`\n`);
            for(let node of l) printer.print(node);
          } else {
            put(`;`);
          }
          put(`\n`);
        }

        CXXCtorInitializer(cxx_ctor_initializer) {
          const {
            anyInit: { name }
          } = cxx_ctor_initializer;

          put(name);
          put(`(`);
          if(cxx_ctor_initializer.inner)
            for(let inner of cxx_ctor_initializer.inner) {
              if(inner.kind.endsWith('Comment')) continue;
              printer.print(inner ?? '');
            }
          put(`)`);
        }
        CXXDependentScopeMemberExpr(cxx_dependent_scope_member_expr) {
          const {
            type,
            valueCategory,
            isArrow,
            member
          } = cxx_dependent_scope_member_expr;

          // if(cxx_dependent_scope_member_expr.inner)
          for(let inner of cxx_dependent_scope_member_expr.inner) {
            if(inner.kind.endsWith('Comment')) continue;
            // console.log('CXXDependentScopeMemberExpr', inner);
            printer.print(inner);
          }

          put(`.${member}`);
        }
        CXXThisExpr(cxx_this_expr) {
          const { valueCategory, isImplicit } = cxx_this_expr;

          put('this');
        }
        CXXDestructorDecl(cxx_destructor_decl) {
          const {
            isImplicit,
            name,
            mangledName,
            type,
            inline,
            explicitlyDefaulted
          } = cxx_destructor_decl;

          let l = cxx_destructor_decl.inner
            ? [...cxx_destructor_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')
              )
            : [];
          put(`${name}(`);

          put(`)`);

          if(l.length) {
            put(`\n`);
            for(let node of l) printer.print(node);
          } else {
            put(`;`);
          }
          put(`\n`);
        }
        CXXMethodDecl(cxx_method_decl) {
          const {
            name,
            type: { qualType: functionType },
            storageClass
          } = cxx_method_decl;
          const returnType = functionType.slice(0, functionType.indexOf('('));

          //  console.log('CXXMethodDecl', console.config({ depth: 10 }), cxx_method_decl);
          let i = 0,
            param,
            initializer;
          let inner = cxx_method_decl.inner
            ? [...cxx_method_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')
              )
            : [];
          if(storageClass) put(`${storageClass} `);
          put(`${returnType}\n`);
          put(`${name}(`);
          while(inner.length &&
            inner[0].kind == 'ParmVarDecl' &&
            (param = inner.shift())
          ) {
            if(i++ > 0) put(', ');
            printer.print(param);
          }
          put(`)`);
          //console.log('CXXMethodDecl', { inner });
          if(inner.length > 0) {
            put(` `);
            depth++;
            for(let node of inner) printer.print(node);
            depth--;
          } else {
            put(`;`);
          }
          put(`\n`);
        }

        CXXStaticCastExpr(cxx_static_cast_expr) {
          const {
            type: { qualType: typeName },
            valueCategory,
            castKind
          } = cxx_static_cast_expr;

          if(castKind == 'NoOp') {
            put(`0`);
            return;
          }
          //console.log('CXXStaticCastExpr', cxx_static_cast_expr);
          put(`static_cast<`);
          put(typeName);
          put(`>(`);

          if(cxx_static_cast_expr.inner)
            for(let inner of cxx_static_cast_expr.inner) {
              if(inner.kind == 'DeclRefExpr' &&
                inner.referencedDecl.name == ''
              )
                continue;
              printer.print(inner);
            }
          put(`)`);
        }

        CXXUnresolvedConstructExpr(cxx_unresolved_construct_expr) {
          const { type, valueCategory } = cxx_unresolved_construct_expr;
          if(cxx_unresolved_construct_expr.inner)
            for(let inner of cxx_unresolved_construct_expr.inner) {
              if(inner.kind.endsWith('Comment')) continue;
              printer.print(inner);
            }
        }
        CXXNewExpr(cxx_new_expr) {
          const {
            type: { qualType: type },
            valueCategory,
            isArray
          } = cxx_new_expr;
          console.log('CXXNewExpr', cxx_new_expr);

          if(isArray) {
            put(`new ${type.replace(/[\s*]*$/g, '')}[`);
          } else {
            put(`new ${type} (`);
          }
          printer.print(cxx_new_expr.inner[0]);

          if(isArray) put(`]`);
          else put(`)`);
        }
        CXXDeleteExpr(cxx_delete_expr) {
          const {
            type: { qualType: type },
            valueCategory,
            isArray,
            isArrayAsWritten
          } = cxx_delete_expr;
          put(`delete `);

          printer.print(cxx_delete_expr.inner[0]);
        }
        CXXBoolLiteralExpr(cxx_bool_literal_expr) {
          const { type, valueCategory, value } = cxx_bool_literal_expr;

          put(value ? 'true' : 'false');
        }
        CXX11NoReturnAttr(cxx11_no_return_attr) {}
        CXXBindTemporaryExpr(cxx_bind_temporary_expr) {}
        CXXCatchStmt(cxx_catch_stmt) {}
        CXXConstructExpr(cxx_construct_expr) {}
        CXXConversionDecl(cxx_conversion_decl) {}
        CXXDefaultArgExpr(cxx_default_arg_expr) {}
        CXXDefaultInitExpr(cxx_default_init_expr) {}
        CXXForRangeStmt(cxx_for_range_stmt) {}
        CXXFunctionalCastExpr(cxx_functional_cast_expr) {}
        CXXMemberCallExpr(cxx_member_call_expr) {}

        CXXNoexceptExpr(cxx_noexcept_expr) {}
        CXXNullPtrLiteralExpr(cxx_null_ptr_literal_expr) {}
        CXXOperatorCallExpr(cxx_operator_call_expr) {}
        CXXReinterpretCastExpr(cxx_reinterpret_cast_expr) {}
        CXXScalarValueInitExpr(cxx_scalar_value_init_expr) {}
        CXXTemporaryObjectExpr(cxx_temporary_object_expr) {}
        CXXTryStmt(cxx_try_stmt) {}
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

export function isNode(obj) {
  return Util.isObject(obj) && typeof obj.kind == 'string';
}

export function GetType(name_or_id, ast = $.data) {
  //console.log('GetType:', name_or_id);
  let result, idx;

  if(typeof name_or_id == 'object' && name_or_id) {
    result = name_or_id;
  } else {
    const types = ast.inner.filter(n =>
      /(?:RecordDecl|TypedefDecl|EnumDecl)/.test(n.kind)
    );
    if(typeof name_or_id == 'string') {
      let tagUsed,
        wantKind = /^.*/;
      if(/^(?:struct|union|enum)\s/.test(name_or_id)) {
        tagUsed = name_or_id.replace(/\s.*/g, '');
        name_or_id = name_or_id.substring(tagUsed.length + 1);
      }
      switch (tagUsed) {
        case 'struct':
        case 'union':
          wantKind = /^RecordDecl/;
          break;
        case 'enum':
          wantKind = /^EnumDecl/;
          break;
      }
      let results = types.filter(name_or_id.startsWith('0x')
          ? node => node.id == name_or_id && wantKind.test(node.kind)
          : node => node.name == name_or_id && wantKind.test(node.kind)
      );
      if(results.length <= 1 ||
        (idx = results.findIndex(r => r.completeDefinition)) == -1
      )
        idx = 0;
      result = results[idx];

      if(!result && Type.declarations.has(name_or_id))
        result = Type.declarations.get(name_or_id);
    } else {
      result = types[name_or_id];
    }
  }
  return result;
}

export function GetFields(node) {
  let fields = deep
    .select(node,
      (v, k) => / at /.test(v) && k == 'qualType',
      deep.RETURN_VALUE_PATH
    )
    .map(([v, p]) => [v.split(/(?:\s*[()]| at )/g)[2], p.slice(0, -2)]);

  return fields.map(([loc, ptr]) =>
    loc
      .split(/:/g)
      .map(i => (!isNaN(+i) ? +i : i))
      .concat([deep.get(node, ptr).name])
  );
}

export function PathRemoveLoc(path) {
  let idx = path.findIndex(p => p == 'loc' || p == 'range');
  if(idx != -1) path = path.slice(0, idx);
  return path;
}
//export default AstDump;
