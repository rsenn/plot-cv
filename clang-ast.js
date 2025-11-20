import * as fs from 'fs';
import * as deep from 'deep';
import * as path from 'path';
import * as BJSON from 'bjson'
import { Pointer } from 'pointer';
import { Spawn } from './os-helpers.js';
//import { countSubstring } from './string-helpers.js';
import { inspect } from 'inspect';
import { assert, bits, className, define, nonenumerable, properties, entries, errors, filter, isArray, isFunction, isObject, isString, keys, lazyProperties, matchAll, memoize, predicate, range, repeat, split, toString, types, unique, values, weakDefine, readObject, mapFunction, defineProperty, defineProperties, setPrototypeOf, isInstanceOf, } from 'util';
import { string, property, shift, and, regexp, notnot } from 'predicate';
import { Location } from 'location';
export { Location } from 'location';
import {MapExtensions} from 'extendMap';

export const SIZEOF_POINTER = 8;
export const SIZEOF_INT = 4;


const ast2np = (
  (wm = new WeakMap()) =>
  (ast, r) =>
    (r = wm.get(ast)) ? r : (wm.set(ast, (r = new WeakMap())), r)
)();

export function DeepFind(ast, pred, flags = deep.RETURN_VALUE) {
  if(isString(pred)) pred = property('name', string(pred));

  const result = deep.find(ast, pred, deep.RETURN_VALUE_PATH, deep.TYPE_OBJECT, ['inner']);

  if(result) {
    const [value, path] = result;

    DeepCachePath(ast, path);

    switch (flags) {
      case deep.RETURN_VALUE_PATH:
        return [value, path];
      case deep.RETURN_PATH:
        return path;
      case deep.RETURN_VALUE:
        return value;
    }
  }
}

export function* DeepSelect(ast, pred, flags = deep.RETURN_VALUE) {
  const m = ast2np(ast);

  if(isString(pred)) pred = property('name', string(pred));

  for(const [value, path] of deep.iterate(ast, pred, deep.RETURN_VALUE_PATH | (flags & ~deep.RETURN_PATH_VALUE), deep.TYPE_OBJECT, ['inner'])) {
    DeepCachePath(ast, path, m);

    switch (flags & deep.RETURN_PATH_VALUE) {
      case deep.RETURN_VALUE_PATH:
        yield [value, path];
        break;
      case deep.RETURN_PATH:
        yield path;
        break;
      case deep.RETURN_VALUE:
        yield value;
        break;
    }
  }
}

export function DeepCachePath(ast, path, m) {
  m ??= ast2np(ast);
  const l = path.length;
  let n = ast;

  for(let i = 0; i < l; ++i) {
    n = n[path[i]];

    if(typeof n == 'object') {
      if(!m.get(n)) m.set(n, path.slice(0, i + 1));
    }
  }
}

export function DeepPathOf(ast, value) {
  let p;

  if((p = ast2np(ast).get(value))) return p;

  return deep.pathOf(ast, value);
}

export function DeepGet(ast, path, ...args) {
  const value = deep.get(ast, path, ...args);

  if(value) DeepCachePath(ast, path);

  return value;
}

const C = console.config({ compact: true });

function FileTime(filename) {
  try {
    let st = fs.statSync(filename);
    return st ? (st.mtime ?? st.time) : -1;
  } catch(e) {}
  return -1;
}

function Newer(file, ...other) {
  //console.log('Newer', { file, other });
  return other.every(other => FileTime(file) > FileTime(other));
}

function Older(file, other) {
  return FileTime(file) < FileTime(other);
}

function GetSubscripts(str) {
  let matches = [...matchAll(/\[([0-9]*)\]/g, str)];
  return matches.map(m => [m.index, +m[1]]);
}

function TrimSubscripts(str, sub) {
  let [subscript = [str.length]] = sub ?? GetSubscripts(str) ?? [[str.length]];
  //console.log("subscript:",subscript);
  return str.slice(0, subscript[0]).trimEnd();
}

export function nameOrIdPred(name_or_id, ...args) {
  if(/^(struct|union)\s/.test(name_or_id)) {
    const idx = name_or_id.indexOf(' ');
    const tag = name_or_id.substring(0, idx);
    const name = name_or_id.substring(idx + 1);

    return and(property('name', string(name)), property('tagUsed', string(tag)), ...args);
    return node => node.name == name && node.tagUsed == tag;
  }

  if(typeof name_or_id == 'number') name_or_id = '0x' + name_or_id.toString(16);

  return name_or_id instanceof RegExp
    ? and(property('name', regexp(name_or_id)), ...args)
    : name_or_id.startsWith('0x')
      ? and(property('id', string(name_or_id)), ...args)
      : and(property('name', string(name_or_id)), ...args);
}

export class List extends Array {
  constructor(...args) {
    super(...args);
  }

  static get [Symbol.species]() {
    return List;
  }

  /* prettier-ignore */ get [Symbol.species]() { return List; }

  filter(callback, thisArg = null) {
    let ret = new List(),
      i = 0;

    if(isInstanceOf(RegExp, callback)) {
      var re = callback;
      callback = elem => isObject(elem) && (re.test(elem.name) || (GetLoc(elem) && re.test(GetLoc(elem).file)));
    }

    for(let elem of this) {
      if(callback.call(thisArg, elem, i, this)) ret[i] = elem;
      i++;
    }

    return ret;
  }

  slice(start, end) {
    let ret = new List(),
      i = 0;

    if(start < 0) start = (start % this.length) + this.length;
    start ??= 0;
    if(end < 0) end = (end % this.length) + this.length;
    end ??= this.length;

    for(let i = start; i < end; i++) ret[i] = this[i];

    return ret;
  }

  /* prettier-ignore */ get first() { return this.find(elem => elem !== undefined); }

  entries() {
    const ret = [];

    for(let [i, elem] of super.entries()) if(elem) ret.push([i, elem]);

    return ret;
  }

  keys() {
    const ret = [];

    for(let [i, elem] of super.entries()) if(elem) ret.push(i);

    return ret;
  }

  values() {
    const ret = [];

    for(let [i, elem] of super.entries()) if(elem) ret.push(elem);

    return ret;
  }

  toArray() {
    return this.reduce((a, n) => [...a, n], []);
  }
}

define(List.prototype, { [Symbol.toStringTag]: 'List' });

export class Node {
  static ast2node = new WeakMap();
  static node2ast = new WeakMap();

  constructor(ast) {
    if(isObject(ast)) {
      if('path' in ast && 'value' in ast) ast = ast.value;

      //throw new Error(`Node constructor ${inspect(ast)}`);
      Node.ast2node.set(ast, this);
      Node.node2ast.set(this, ast);
    }
  }

  static get(ast) {
    return Node.ast2node.get(ast);
  }

  /* prettier-ignore */ get ast() { return Node.node2ast.get(this); }
  /* prettier-ignore */ get id() { return this.ast.id; }
  /* prettier-ignore */ get loc() { return new Location(GetLoc(this.ast)); }

  get file() {
    const loc = this.ast.loc ?? DeepFind(this.ast, (v, k) => k == 'loc') ?? DeepFind(t.ast, (v, k) => isObject(v) && 'file' in v);
    if(loc) return loc.file;
  }

  get range() {
    const { range } = this.ast;
    let loc, file;

    if(range) {
      const { begin, end } = range;

      if(!('line' in begin)) begin.line = (loc ??= this.loc).line;

      return [begin, end].map(r => {
        if(!('file' in r)) r.file = file ??= this.file;

        return new Location(r);
      });
    }

    return [new Location(range.begin), new Location(range.end)];
  }

  toJSON(obj) {
    const { kind } = this;
    obj ??= { kind };

    if(!obj.kind) obj.kind = className(this);

    return obj;
  }

  /* prettier-ignore */ get [Symbol.toStringTag]() { return this.constructor.name; }
}

setPrototypeOf(Node.prototype, null);

const getTypeFromNode = memoize((node, ast) => new Type(node.type, ast), new WeakMap());

export function PathOf(node, ast = globalThis['$'].data) {
  return new Pointer(DeepPathOf(ast, node?.ast ?? node));
}

export function* Hier(node_or_path, t = (p, ast, abort) => p.deref(ast), ast = globalThis['$'].data) {
  let p;

  if(node_or_path && node_or_path.kind) p = PathOf(node_or_path, ast);
  else p = new Pointer(node_or_path);

  const hier = p
    .hier()
    .reverse()
    .filter(p => p.at(-1) != 'inner');

  for(let pp of hier) {
    const doAbort = false,
      abortFn = () => (doAbort = true);

    yield t(pp, ast, abortFn);

    if(doAbort) break;
  }
}

export function FindType(typeName, ast = globalThis['$'].data) {
  const tokens = [...typeName.matchAll(/[A-Za-z_][A-Za-z0-9_]+/g)].map(([tok]) => tok);

  while(['const'].indexOf(tokens[0]) != -1) {
    tokens.shift();
    typeName = typeName.replace(new RegExp('^' + tokens[0] + '\\s*'), '');
  }

  const nodes = DeepSelect(ast, tokens[0]).filter(node => node.inner && node.inner.length);
  console.log('nodes', nodes);

  return new Type(typeName, ast);
}

export class PointerType extends Node {
  pointee = null;

  constructor(pointee, ast, ns) {
    super(ast, ns);
    define(this, { pointee });
  }

  static fromString(str, ast, ns) {
    const t = str.replace(/^(.*)\s*\*\s*$/g, '$1');

    if(t != str) {
      const type = Type.declarations.get(t) ?? new Type(t, ast, ns);
      return new PointerType(type, ast, ns);
    }
  }

  toString() {
    return this.pointee + ` *`;
  }
}

export class ReferenceType extends Node {
  pointee = null;

  constructor(pointee, ast, ns) {
    super(ast, ns);
    define(this, { pointee });
  }

  static fromString(str, ast, ns) {
    const t = str.replace(/^(.*)\s*\&\s*$/g, '$1');

    if(t != str) {
      const type = Type.declarations.get(t) ?? new Type(t, ast, ns);
      return new ReferenceType(type, ast, ns);
    }
  }

  toString() {
    return this.pointee + ` &`;
  }
}

export class Type extends Node {
  static declarations = new Map();
  static node2type = getTypeFromNode.cache;

  constructor(node, ast, ns) {
    let name, desugared, typeAlias, qualType;

    // console.log('Type.constructor', C, { node, ast });

    ast ??= globalThis['$']?.data;

    if(Array.isArray(node)) {
      throw new Error(`Node is array!`);
    }

    if(node.kind) {
      name = NameFor(node, ast);
      if(Type.declarations.has(name)) return Type.declarations.get(name);
    }

    if(node.qualType && Type.declarations.has(node.qualType)) return Type.declarations.get(node.qualType);

    if(typeof node == 'string') {
      let tmp;

      name = node.trim().replaceAll(/^(const\s+|volatile\s+|)*/g, '');
      qualType = node;
      node = null;

      if(Type.declarations.has(name)) return Type.declarations.get(name);

      if(ast && typeof (tmp = DeepFind(ast, name)) == 'object' && tmp != null) {
        tmp = 'kind' in tmp ? TypeFactory(tmp, ast) : new Type(tmp, ast);

        if(tmp) node = 'ast' in tmp ? tmp.ast : tmp;
      }

      const isPointer = name.endsWith('*');
      const isReference = name.endsWith('&');

      if(isPointer) {
        node = {
          kind: 'CustomType',
          qualType: name,
          desugaredQualType: 'void *',
        };
      }

      if(!isPointer && !isReference && !node) {
        const subscripts = GetSubscripts(name);
        name ??= TrimSubscripts(name, subscripts);
        //console.log('Type', { name, subscripts });

        if(ast) {
          node = GetType(name, ast) ?? GetClass(name, ast);

          //if(node) console.log(`Found type ${name}`, node);
        }
      }
    } else {
      const t = node?.qualType;

      if(/\*\s*$/.test(t)) return PointerType.fromString(t, ast, ns);
      if(/\&\s*$/.test(t)) return ReferenceType.fromString(t, ast, ns);

      if('path' in node && 'value' in node) node = node.value;
    }

    if(node instanceof Node) {
      return node;

      console.log('node', className(node), node);

      throw new Error();
    }

    super(node);

    if(node?.kind && node.kind.startsWith('Enum')) name = 'enum ' + name;

    let type = /*'type' in node ?*/ node?.type ?? node;

    name = /^Typedef|Record/.test(node?.kind) ? node.name : (node?.ast ?? node)?.kind ? NamespaceOf(node?.ast ?? node, ast) + '' : node?.qualType;
    if(node?.tagUsed && name) name = (node.tagUsed ? node.tagUsed + ' ' : '') + name;
    qualType = type?.qualType ?? node?.qualType;

    if(Type.declarations.has(name)) return Type.declarations.get(name);

    //console.log('Type.constructor', console.config({ depth: 2 }), { name, node });

    desugared = type?.desugaredQualType ?? node?.desugaredQualType;
    typeAlias = type?.typeAliasDeclId ?? node?.typeAliasDeclId;

    if(node?.inner && node.inner[0] && node.inner[0].kind == 'ElaboratedType') {
      typeAlias ??= node.inner[0]?.ownedTagDecl?.id;

      if(typeAlias) {
        const alias = new Type(
          ast.inner.find(n => n.id == typeAlias),
          ast,
        );

        if(name) alias.name = name;
        if(qualType) alias.qualType = qualType;
        if(desugared) alias.desugared = desugared;
      }
    }

    if(desugared === qualType) desugared = undefined;
    if(name == '') name = undefined;

    //console.log('Type.constructor',C, {name,qualType});

    if(ns) {
      if(!Type.declarations.has(name)) Type.declarations.set(ns, this);
    } else if(name) {
      if(!Type.declarations.has(name)) Type.declarations.set(name, this);
    } else if(qualType) {
      if(!Type.declarations.has(qualType)) Type.declarations.set(qualType, this);
    }

    if(typeAlias) typeAlias = +typeAlias;

    if(name) weakDefine(this, { name });
    if(qualType) weakDefine(this, { qualType });
    if(desugared) weakDefine(this, { desugared });
    if(typeAlias) define(this, nonenumerable({ typeAlias }));

    if(this.isPointer()) {
    } else if(this.isEnum()) {
      this.desugared = 'int';
    }
  }

  /* prettier-ignore */ get regExp() { return new RegExp(`(?:${this.qualType}${this.typeAlias ? '|' + this.typeAlias : ''})`.replace(/\*/g, '\\*'), 'g'); }

  isEnum() {
    const str = this.qualType || this + '';
    return /^enum\s/.test(str);
  }

  isPointer() {
    const { desugared, qualType } = this;
    return /(?:\(\*\)\(|\*$)/.test(desugared) || /\*$/.test(qualType);
  }

  isReference() {
    const { desugared, qualType } = this;
    return /(?:\(\&\)\(|\&$)/.test(desugared) || /\&$/.test(qualType);
  }

  isFunction() {
    const str = this + '';
    return /\(.*\)$/.test(str) && !/\(\*\)\(/.test(str);
  }

  isArray() {
    return /\[[0-9]*\]$/.test(this + '');
  }

  isInteger() {
    return !this.isPointer() && !this.isCompound() && !this.isFloatingPoint();
  }

  isStruct() {
    return this.tag == 'struct';
  }

  isClass() {
    return this.tag == 'class';
  }

  arrayOf() {
    const typeName = this.trimSubscripts();
    return Type.declarations.get(typeName);
  }

  /* prettier-ignore */ get subscripts() { if(this.isArray()) return GetSubscripts(this+''); }

  trimSubscripts() {
    return TrimSubscripts(this + '');
  }

  get pointer() {
    const str = this + '';
    const name = str.replace(/\s*(\*$|\(\*\))/, '');

    if(name == str) return undefined;

    return name;
  }

  get reference() {
    const str = this + '';
    const name = str.replace(/\s*(\&$|\(\&\))/, '');

    if(name == str) return undefined;

    return name;
  }

  getPointer(ast) {
    const target = this.pointer;

    if(target) {
      const node = DeepFind(ast, target);

      if(node) return TypeFactory(node, ast);

      if(Type.declarations.has(target)) return Type.declarations.get(target);
    }
  }

  /* prettier-ignore */ get unsigned() { return /(?:unsigned|ushort|uint|ulong)/.test(this + ''); }
  /* prettier-ignore */ get signed() { return /(?:^|[^n])signed/.test(this+'') || !this.unsigned; }

  isCompound() {
    return /^(class|struct|enum)$/.test(this.tag + '') || /(?:struct|union)\s/.test(this + '');
  }

  isFloatingPoint() {
    return /(?:\ |^)(float|double)$/.test(this + '');
  }

  /* prettier-ignore */ get alias() { if(this.typeAlias) return Type.get(this.typeAlias); }

  get aliases() {
    const list = [];
    let type = this;

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

  isString() {
    const { desugared, qualType } = this;

    return /^(const |)char \*$/.test(desugared) || /^(const |)char \*$/.test(qualType) || /^(const |)char$/.test(this.pointer);
  }

  get ffi() {
    const { desugared, qualType } = this;

    let str = this + '';
    if(str.startsWith('const ')) str = str.substring(6);
    if(/^char\s*\*$/.test(str) || this.isString()) return 'char *';

    for(const type of [str, desugared])
      if(
        [
          'void',
          'sint8',
          'sint16',
          'sint32',
          'sint64',
          'uint8',
          'uint16',
          'uint32',
          'uint64',
          'float',
          'double',
          'schar',
          'uchar',
          'sshort',
          'ushort',
          'sint',
          'uint',
          'slong',
          'ulong',
          'longdouble',
          'pointer',
          'int',
          'long',
          'short',
          'char',
          'size_t',
          'unsigned char',
          'unsigned int',
          'unsigned long',
          'void *',
          'char *',
          'string',
        ].indexOf(type) != -1
      )
        return type;

    const { size, unsigned } = this;

    if(size == SIZEOF_POINTER && !this.isPointer()) return ['', 'unsigned '][unsigned | 0] + 'long';

    //if(size == SIZEOF_POINTER && unsigned) return 'size_t';
    if(size == SIZEOF_POINTER / 2 && !unsigned) return 'int';
    if(size == 4) return ['s', 'u'][unsigned | 0] + 'int32';
    if(size == 2) return ['s', 'u'][unsigned | 0] + 'int16';
    if(size == 1) return ['s', 'u'][unsigned | 0] + 'int8';

    if(this.isPointer()) return 'void *';
    if(size > SIZEOF_POINTER) return 'void *';
    if(size === 0) return 'void';

    str ??= this + '';

    if(Type.declarations.has(str)) {
      const decl = Type.declarations.get(str);

      if(decl.kind == 'EnumDecl') return 'int';
    } else {
      throw new Error(`No ffi type '${str}' ${size} ${className(this)} ${this.ast.kind}`);
    }
  }

  get size() {
    if(this.isPointer()) return SIZEOF_POINTER;
    if(this.isEnum()) return SIZEOF_INT;
    if(this.isCompound()) {
      const node = Type.get(this + '');

      if(node && node !== this) return node.size;
    }
    const desugared = this.desugared || this + '' || this.name;

    if(desugared == 'char') return 1;
    const re = /(?:^|\s)_*u?int([0-9]+)(_t|)$/;

    let size, match;

    if(this.isArray()) {
      size ??= this.arrayOf()?.size;
      for(let [index, subscript] of this.subscripts) size *= subscript;
    }

    if((match = re.exec(desugared))) {
      const [, bits] = match;
      if(!isNaN(+bits)) return +bits / 8;
    }

    if((match = /^[^\(]*\(([^\)]*)\).*/.exec(desugared))) if (match[1] == '*') return SIZEOF_POINTER;

    match = /^(unsigned\s+|signed\s+|const\s+|volatile\s+|long\s+|short\s+)*([^\[]*[^ \[\]]) *\[?([^\]]*).*$/g.exec(desugared);

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

    if(size === undefined) size = NaN;

    return size;
  }

  toJS() {
    if(this.isString()) return 'String';

    return 'Number';
  }

  [Symbol.toPrimitive](hint) {
    if(hint == 'default' || hint == 'string') return (this.qualType ?? this.desugaredQualType ?? this?.ast?.name ?? '').replace(/\s+(\*+)$/, '$1'); //this+'';

    return this;
  }

  toJSON(obj) {
    const { qualType, size } = this;

    return super.toJSON({ ...obj, qualType, size });
  }

  static get(name_or_id, ast = globalThis['$'].data) {
    let type;
    const node =
      ast.inner.find(typeof name_or_id == 'number' ? node => /(?:Decl|Type)/.test(node.kind) && +node.id == name_or_id : node => /(?:Decl|Type)/.test(node.kind) && node.name == name_or_id) ??
      GetType(name_or_id, ast);

    if(node) {
      if(node.type) type = getTypeFromNode(node, ast);
      else type = TypeFactory(node, ast);
      if(node.name && typeof name_or_id != 'string') name_or_id = node.name;
    }

    if(typeof name_or_id == 'string' && !Type.declarations.has(name_or_id)) Type.declarations.set(name_or_id, type);

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
Type.declarations.set('unsigned short', new Type({ qualType: 'unsigned short' }));
Type.declarations.set('unsigned long', new Type({ qualType: 'unsigned long' }));
Type.declarations.set('unsigned long long', new Type({ qualType: 'unsigned long long' }));
Type.declarations.set('unsigned __int128', new Type({ qualType: 'unsigned __int128' }));
Type.declarations.set('float', new Type({ qualType: 'float' }));
Type.declarations.set('double', new Type({ qualType: 'double' }));
Type.declarations.set('void *', new Type({ qualType: 'void *' }));
Type.declarations.set('char *', new Type({ qualType: 'char *' }));
Type.declarations.set('const char *', new Type({ qualType: 'const char *' }));
Type.declarations.set('bool', new Type({ qualType: 'bool' }));

function RoundTo(value, align) {
  return Math.floor((value + (align - 1)) / align) * align;
}

export class RecordDecl extends Type {
  constructor(node, ast, ns) {
    super(node, ast);
    const { id, tagUsed, inner } = node;

    this.name ??= NameFor(node, ast, ns);

    if(inner?.find(child => child.kind == 'PackedAttr')) this.packed = true;
    const fields = inner?.filter(child => child.kind.endsWith('Decl'));

    if(tagUsed) this.tag = tagUsed;

    if(fields) {
      let tag, access;

      defineProperty(this, 'members', {
        enumerable: true,
        get: memoize(() =>
          fields
            .filter(node => !('parentDeclContextId' in node) && node.kind != 'FriendDecl')
            .reduce((acc, node) => {
              let { name, kind } = node;
              let type;

              //console.log('members', console.config({ compact: true }), { name, kind });

              if(node.isBitfield) name += ':' + node.inner[0].inner[0].value;

              if(kind.endsWith('Decl')) {
                if(kind.startsWith('Indirect')) {
                  type = null;
                } else if(node?.type?.qualType && /( at )/.test(node.type.qualType)) {
                  let loc = node.type.qualType.split(/(?:\s*[()]| at )/g)[2];
                  let [file, line, column] = loc.split(/:/g).map(i => (!isNaN(+i) ? +i : i));

                  let typePaths = [...DeepSelect(inner, n => n.line == line, deep.RETURN_PATH)];
                  let typePath = PathRemoveLoc(typePaths[0]);
                  let typeNode = DeepGet(inner, typePath);
                  type = TypeFactory(typeNode, ast, ns);
                } else if(kind == 'EnumDecl') {
                  /*let entries = node.inner.map(({ id, kind, name, type }) => [name, type]);

              console.log('EnumDecl', console.config({ compact: true }), { name, kind, entries });*/
                  /*for(let [name, value] of entries) acc.push([name, value]);*/
                } else if(/(Method|Friend)/.test(kind)) {
                } else if(kind.startsWith('Access')) {
                  access = node.access;

                  return acc;
                } else if(kind.startsWith('CXXRecord')) {
                  tag = node.tagUsed;
                  if(tag == 'class') access = 'private';

                  return acc;
                } else if(kind.startsWith('CXX')) {
                  type = TypeFactory(node, ast, ns);

                  if(/structor/.test(kind)) define(type, nonenumerable({ ctordtor: /Constructor/.test(kind) ? 'constructor' : 'destructor' }));
                } else if(kind.startsWith('Field') && !['protected', 'private'].includes(node.access)) {
                  type = TypeFactory(node, ast, ns);
                } else if(node.type) {
                  type = new Type(node.type, ast, ns);
                  if(type.desugared && type.desugared.startsWith('struct ')) {
                    let tmp = ast.inner.find(n => n.kind == 'RecordDecl' && n.name == /^struct./.test(n.name));
                    if(tmp) type = TypeFactory(tmp.value, ast, ns);
                  }
                } else if(node.kind.startsWith('Record')) {
                  type = new RecordDecl(node, ast, ns);
                } else {
                  throw new Error(`node.kind=${node.kind} `);
                }
              }

              if(type?.ast?.isImplicit) return acc;

              if(!type) type = node.kind.startsWith('Indirect') ? null : TypeFactory(node, ast, ns);

              if(type && access) define(type, nonenumerable({ access }));

              /*          if(type instanceof EnumDecl) {
            for(let [name,[,value]] of type.members) {
          acc.push([name, type]);
            }
          } else */
              acc.push([name, type]);

              return acc;
            }, [])
            .map(([name, t]) => Object.assign(t, { name })),
        ),
      });
    }
  }

  get size() {
    const { members = [] } = this;

    return RoundTo(
      [...members].reduce((acc, [name, type]) => {
        if(Number.isFinite(type?.size)) {
          if(type.size == 8) acc = RoundTo(acc, 8);
          return acc + RoundTo(type.size, 4);
        }
        return acc;
      }, 0),
      SIZEOF_POINTER,
    );
  }

  toJSON() {
    const { name, size, members } = this;
    return super.toJSON({
      name,
      size,
      members: members.map(([name, member]) => [name, member != null && member.toJSON ? member.toJSON() : member]),
    });
  }
}

export class EnumDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    ///if(node.name) this.name = `enum ${node.name}`;

    const constants = node.inner.filter(child => child.kind == 'EnumConstantDecl');
    let number = 1;

    this.members = new Map(
      constants.map(({ name, type, inner }) => {
        let value = inner ? PrintNode(inner[0]) : undefined;

        if(value !== '' && typeof value == 'string' && !isNaN(+value)) {
          value = +value;
        }

        if(typeof value != 'number') value = number;

        number = value + 1;
        return [name, this.name + name]; //[new Type(type, ast), value]];
      }),
    );
  }

  toJSON() {
    const { name, size, members } = this;
    return super.toJSON({ name, size, members });
  }
}

export class TypedefDecl extends Type {
  constructor(node, ast, ns) {
    super(node, ast, ns);

    let inner = (node.inner ?? []).filter(n => !/Comment/.test(n.kind));
    let type;

    let { typeAlias } = node;

    let typeId = DeepFind(inner, (v, k) => k == 'decl')?.id;

    if(typeAlias) type = ast.inner.find(n => n.id == typeAlias);
    else if(typeId) type = ast.inner.find(n => n.id == typeId);
    else type = node.inner.find(n => /Type/.test(n.kind));

    //type ??= GetType(node, ast);
    assert(inner.length, 1);

    if(type?.decl) type = type.decl;
    if(type?.kind && type.kind.endsWith('Type')) type = type.type;

    //console.log('TypedefDecl.constructor', { name: node.name, type  });

    this.name = node.name;
    this.type = type.kind ? TypeFactory(type, ast, false) : new Type(type, ast, ns);
  }

  /* prettier-ignore */ get size() { return this.type?.size; }

  toJSON() {
    const { name, size } = this;
    return super.toJSON({ name, size });
  }
}

export class FieldDecl extends Node {
  constructor(node, ast) {
    super(node, ast);

    let type = node.type ?? GetType(node, ast);

    this.name = node.name;
    this.type = Type.declarations.get(type?.qualType) ?? new Type(type?.qualType ?? type, ast);
  }
}

export class FunctionDecl extends Node {
  constructor(node, ast, ns) {
    super(node, ast, ns);

    this.name = node.name;

    if(node.mangledName && node.mangledName != node.name) {
      define(this, nonenumerable({ mangledName: node.mangledName }));
    }

    let parameters = node.inner?.filter(child => child.kind == 'ParmVarDecl');
    let body = node.inner?.find(child => child.kind != 'ParmVarDecl');
    let type = node.type?.qualType;
    let returnType = type.replace(/\s*\(.*/, '');

    let storageClass = node.storageClass;

    //console.log('FunctionDecl.constructor', { name: this.name, type, returnType });

    if(Type.declarations.has(returnType)) returnType = Type.declarations.get(returnType);

    let t;

    if(typeof returnType == 'string' && returnType.endsWith('&')) {
      this.returnType = ReferenceType.fromString(returnType, ast, ns);
    } else if(typeof returnType == 'string' && returnType.endsWith('*')) {
      this.returnType = PointerType.fromString(returnType, ast, ns);
    } else if(typeof returnType == 'string' && (t = GetNamespace(returnType, ast))) {
      //console.log('FunctionDecl.constructor', C, { returnType, t });
      this.returnType = new Type(t, ast, NamespaceOf(t, ast));
    } else this.returnType = returnType instanceof Node ? returnType : typeof returnType != 'string' ? TypeFactory(returnType, ast) : new Type(returnType, ast);

    this.parameters = parameters && parameters.map(({ name, type }) => [name, new Type(type, ast)]);
    this.body = body;

    if(storageClass) this.storageClass = storageClass;
  }

  isMethod() {
    return this.ast.kind == 'CXXMethodDecl';
  }

  toJSON() {
    const { name, parameters } = this;
    return super.toJSON({ name, parameters });
  }
}

export class VarDecl extends Node {
  constructor(node, ast) {
    super(node, ast);

    this.name = node.name;

    if(node.mangledName && node.mangledName != node.name) this.mangledName = node.mangledName;

    let type = node.type?.qualType;
    //console.log('VarDecl', { type });

    this.type = type.kind ? TypeFactory(type, ast) : new Type(type, ast);

    //console.log('VarDecl', this);
  }

  toJSON() {
    const { name, type } = this;
    return super.toJSON({ name, type });
  }
}

export class ClassDecl extends RecordDecl {
  constructor(node, ast) {
    super(node, ast);

    //console.log('ClassDecl.constructor', { node, ast });
  }
}

export class BuiltinType extends Type {
  constructor(node, ast) {
    super(node.type, ast);
  }
}

/*export class PointerType extends Node {
  constructor(node, ast) {
    super(node, ast);
    assert(node.inner.length, 1);

    this.type = new Type(node.type, ast);
    this.pointee = TypeFactory(node.inner[0], ast);
  }

  toJSON() {
    const { pointee, type } = this;
    return super.toJSON({ pointee, type });
  }
}*/

export class ConstantArrayType extends Node {
  constructor(node, ast) {
    super(node, ast);
    let elementType = node.inner[0];
    assert(node.inner.length, 1);
    if(elementType.decl) elementType = elementType.decl;
    this.type = new Type(node.type, ast);
    this.elementType = TypeFactory(elementType, ast);
  }

  toJSON() {
    const { type, elementType } = this;
    return super.toJSON({ type, elementType });
  }
}

export class Range {
  #begin = undefined;
  #end = undefined;

  constructor(begin, end) {
    if(begin !== undefined) this.begin = begin;
    if(end !== undefined) this.end = end;
  }

  /* prettier-ignore */ get begin() { return this.#begin; }
  /* prettier-ignore */ set begin(v) { this.#begin = Location.from(v); }

  /* prettier-ignore */ get end() { return this.#end; }
  /* prettier-ignore */ set end(v) { this.#end = Location.from(v); }

  toString(opts = { printFile: true }) {
    opts.onlyOffset = true;
    return this.#begin.toString(opts) + '-' + this.#end.toString({ ...opts, printFile: false });
  }

  [Symbol.inspect](depth, opts) {
    const { begin, end } = this;
    return inspect(
      {
        file: begin.file ?? end.file,
        begin: new Location({
          ...begin.toObject(),
          file: undefined,
          line: undefined,
          col: undefined,
        }),
        end: new Location({ ...end.toObject(), file: undefined, line: undefined, col: undefined }),
        [Symbol.toStringTag]: 'Range',
      },
      { ...opts, compact: false, customInspect: true, onlyOffset: true },
    );
  }

  get length() {
    return this.#end - this.#begin;
  }

  toArray() {
    return [this.#begin, this.#end];
  }

  toObject() {
    const { begin, end } = this;
    return { begin: begin.toObject(), end: end.toObject() };
  }

  *[Symbol.iterator]() {
    yield +this.#begin;
    yield +this.#end;
  }
}

Range.prototype[Symbol.toStringTag] = 'Range';

/*export class Location {
  #line = undefined;
  #column = undefined;
  #offset = undefined;
  file = undefined;

  static at(file, offset) {
    let data = ReadFile(file, 'utf-8').slice(0, offset);
    let lastLine = data.slice(data.lastIndexOf('\n') + 1);

    return new this({
      line: countSubstring(data, '\n') + 1,
      col: lastLine.length + 1,
      file,
      offset,
    });
  }

  static from(loc) {
    if(isObject(loc) && loc instanceof Location) return loc;
    try {
      return new Location(loc);
    } catch(e) {
      console.log('ERROR', e.message, loc);
      throw e;
    }
  }

  constructor(loc) {
    const { line, col, file, offset } = loc;

    if('line' in loc) this.#line = loc.line;
    if('col' in loc) this.#column = loc.col;
    if('offset' in loc) this.#offset = loc.offset;
    if('file' in loc) {
      const haveFilename = typeof loc.file == 'string';

      defineProperty(this, 'file', { value: loc.file, enumerable: true, writable: !haveFilename, configurable: true });
    }
  }

  update(other) {
    for(let prop of ['file', 'offset', 'line', 'column']) {
      if(other[prop]) this[prop] = other[prop];
    }
  }

  get line() { return this.#line; }
  set line(v) { if(v != this.#line) this.#column = undefined; this.#line = v; }

  get column() { return this.#column; }
  set column(v) { this.#column = v; }

  get offset() { return this.#offset; }
  set offset(v) { if(this.#offset - v < this.#column) this.#column -= this.#offset - v; else { this.#column = undefined; this.#line = undefined; } this.#offset = v; }

  [Symbol.for('nodejs.util.inspect.custom')](depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    return text('Location', 38, 5, 111) + ' ' + this.toString(opts);
  }

  toString(opts = { printFile: true, onlyOffset: false }) {
    let file = this.file,
      col = this.#column,
      line = this.#line;
    const { printFile = true, onlyOffset = false } = opts;

    if(line !== undefined && !onlyOffset) return [file ?? '<builtin>', line, ...(col !== undefined ? [col] : [])].slice(printFile ? 0 : 1).join(':');
    return `${printFile && file ? file + '@' : ''}${this.#offset}`;
  }

  [Symbol.toPrimitive](hint) {
    switch (hint) {
      case 'number':
        return this.offset;
      case 'string':
      default:
        return this.toString();
    }
  }

  toObject() {
    let ret = {};
    if(this.file) ret.file = this.file;
    if(this.#column) ret.col = this.#column;
    if(this.#line) ret.line = this.#line;
    ret.offset = this.#offset;

    return ret;
  }

  localeCompare(other) {
    let str = this + '';
    return str.localeCompare(other + '');
  }
}*/

export function TypeFactory(node, ast, cache = true) {
  let obj;

  let ns = NamespaceOf(node, ast) + '';
  //console.log('TypeFactory:', { name,node });

  assert(
    node?.kind,
    `Not an AST node: ${inspect(node, {
      colors: false,
      compact: 0,
      depth: Infinity,
    })}`,
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
      obj = new TypedefDecl(node, ast, ns);
      break;
    case 'CXXConstructorDecl':
    case 'CXXMethodDecl':
    case 'CXXDestructorDecl':
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
    case 'VarDecl':
      obj = new VarDecl(node, ast);
      break;
    case undefined:
      throw new Error(`Not an AST node: ${inspect(node, { colors: false, compact: 0 })}`);
      break;

    default:
      console.log('node:', node);
      throw new Error(`No such kind of AST node: ${node.kind}`);
      //obj = new Type(node, ast);
      break;
  }

  if(obj) {
    if(obj[Symbol.toStringTag] != node.kind) defineProperties(obj, { [Symbol.toStringTag]: { value: node.kind } });
  }

  return obj;
}

export async function SpawnCompiler(compiler, input, outfile, args = []) {
  //console.log(`SpawnCompiler`, new Error().stack.split(/\n/g).slice(0, 2));

  let base = path.basename(input, path.extname(input));

  args.push(input);

  if(args.indexOf('-ast-dump=json') != -1) {
    args.unshift(compiler ?? 'clang');
    args = [process?.env?.SHELL ?? 'sh', '-c', 'exec ' + args.map(p => (p.indexOf(' ') != -1 ? `'${p}'` : p)).join(' ') + (outfile ? ` 1>${outfile}` : '')];
  } else {
    if(outfile) {
      args.unshift(outfile);
      args.unshift('-o');
    }
    args.unshift(compiler ?? 'clang');
  }

  console.log('SpawnCompiler', args.map(p => (p.indexOf(' ') != -1 ? `'${p}'` : p)).join(' ') + (outfile ? ` 1>${outfile}` : ''));

  let child = Spawn(args.shift(), args, {
    block: false,
    stdio: ['inherit', outfile ? 'inherit' : 'pipe', 'pipe'],
  });

  let json = '',
    errors = '',
    output = '';
  let done = false;

  if(child.stdout) for(let chunk of fs.readerSync(child.stdout)) output += toString(chunk);

  for(let chunk of fs.readerSync(child.stderr)) errors += toString(chunk);

  let pid = await child.wait();

  let { exitcode, termsig, exited, signaled, stopped, continued } = child;

  //console.log('SpawnCompiler', console.config({ compact: true }), { pid, exited, exitcode });

  done = true;
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  errorLines = errorLines.filter(line => /error:/.test(line));
  const numErrors = [...(/^([0-9]+)\s/g.exec(errorLines.find(line => /errors\sgenerated/.test(line)) || '0') || [])][0] || errorLines.length;
  if(numErrors) {
    console.log('errors:', errors);
    throw new Error(errorLines.join('\n'));
  }

  function PipeReader(fd, callback) {
    let ret;
    return new Promise((resolve, reject) => {
      os.setReadHandler(fd, () =>
        ReadPipe(fd, data => {
          if(data) ret = callback(data);
          else resolve(ret);
        }),
      );
    });
  }

  function ReadPipe(fd, callback) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    let data;
    if(r > 0) {
      data = fs.bufferToString(buf.slice(0, r));
    } else {
      os.setReadHandler(fd, null);
      data = null;
    }
    callback(data);
  }
  function ReadOutput(fd) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    if(r > 0) {
      output += fs.bufferToString(buf.slice(0, r));
    } else {
      os.setReadHandler(fd, null);
    }
  }
  let ret = { output, exitcode, errors: errorLines };
  // console.log('SpawnCompiler return', ret);

  return ret;
}

export async function SourceDependencies(...args) {
  if(args.length < 3) args.unshift(params.compiler);

  let [compiler, source, flags = []] = args;

  let r = await SpawnCompiler(compiler, source, null, ['-MM', '-I.', ...flags]);
  let { output, result, errors } = (globalThis.response = r);
  output = output.replace(/\s*\\\n\s*/g, ' ');
  let [object, sources] = output.split(/:\s+/);

  sources = (sources ?? '').trim().split(/ /g);
  let [compilation_unit, ...includes] = sources;

  //console.log('output[1]:', { compilation_unit, includes });

  return sources;
}

export async function AstDump(compiler, source, args, force) {
  compiler ??= 'clang';
  // console.log('AstDump', { compiler, source, args, force });
  let output = path.basename(source, path.extname(source)) + '.ast.json';
  let bjson = path.basename(output, '.json') + '.bjson';
  const paths = [output, bjson];
  let r;
  let sources = await SourceDependencies(compiler, source, args);
  let newer;

  for(let p of paths) {
    if(!fs.existsSync(p)) continue;

    const existsAndNotEmpty = fs.sizeSync(p) > 0;

    if(existsAndNotEmpty) newer = Newer(p, ...sources);

    if(newer) output = p;
  }

  //console.log('AstDump', console.config({compact: true }), { bjson, output, newer, force, paths });

  if(!force && newer) {
    console.log(`Loading cached '${output}'...`);
  } else {
    console.log(`Compiling '${source}' to '${output}'...`);

    try {
      if(fs.existsSync(output)) fs.unlinkSync(output);
    } catch(e) {}

    console.log(`Compiling...`, console.config({ compact: Infinity, depth: 0 }), { source, compiler });

    const child = await SpawnCompiler(compiler, source, paths[0], ['-Xclang', '-ast-dump=json', '-fsyntax-only', '-I.', ...args]);

    let { exitcode, errors, ...result } = child;

    //console.log(`Compiling '${source}'...`, console.config({compact: true }), { output, exitcode, ...result });
  }
  r = { file: output };

  //console.log('AstDump', console.config({compact: true }), { ...r });

  r = lazyProperties(r, {
    size() {
      return fs.stat(output)?.size;
    },
    json() {
      const binary = /\.bjson$/i.test(this.file);

      if(binary) {
        let data=fs.readFileSync(this.file, null);
      
        if(data) 
        return BJSON.read(data);

      else throw new Error(`ERROR reading ${this.file}`)
      }

      return  JSON.parse(fs.readFileSync(this.file, 'utf-8'));
    },
    data() {
      let data = this.json;
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
      if(!/\.bjson$/i.test(this.file)) {
        console.log(`Writing '${bjson}'...`);
        fs.writeFileSync(bjson,  BJSON.write(data));
        console.log(`Deleting '${this.file}'...`);
        fs.unlinkSync(this.file);
        this.file = bjson;
      }
      return data;
    },
    files() {
      return unique(this.data.inner.map(n => n.loc.file).filter(file => file != undefined));
    },
  });

  r = define(r, {
    matchFiles: null,
    nomatchFiles: /^\/usr/,
    filter(pred, pred2 = (used, implicit) => used && !implicit) {
      return this.data.inner.filter(
        node =>
          ((node.loc.file !== undefined && ((this.matchFiles && this.matchFiles.test(node.loc.file ?? '')) || !this.nomatchFiles.test(node.loc.file ?? ''))) ||
            (pred2 ? pred2(node.isUsed, node.isImplicit) : false)) &&
          pred(node),
      );
    },
  });
  r = lazyProperties(r, {
    types() {
      return setPrototypeOf(
        this.filter(
          n => /(?:Record|Typedef|Enum)Decl/.test(n.kind),
          () => true,
        ).map(node => TypeFactory(node, this.data)),
        List.prototype,
      );
    },
    functions() {
      let list = this.filter(
        n => /(?:Function)Decl/.test(n.kind),
        () => true,
      );

      if(list.length == 0) list = [...DeepSelect(this.data, n => n.kind.startsWith('FunctionDecl'))];

      return setPrototypeOf(
        list.map(node => new FunctionDecl(node, this.data)),
        List.prototype,
      );
    },
    namespaces() {
      return setPrototypeOf([...DeepSelect(this.data, n => 'NamespaceDecl' == n.kind)], List.prototype);
    },
    classes() {
      let predicate = n => n.kind == 'CXXRecordDecl' && !n.isImplicit;
      return setPrototypeOf([...DeepSelect(this.data, predicate)], List.prototype);
    },
    structs() {
      return setPrototypeOf(
        this.filter(n => /RecordDecl/.test(n.kind)).map(n => TypeFactory(n, this.data)),
        List.prototype,
      );
    },
    variables() {
      return setPrototypeOf(
        this.filter(n => /(?:Var)Decl/.test(n.kind)).map(node => new VarDecl(node, this.data)),
        List.prototype,
      );
    },
    names(depth = 1) {
      return this.data.inner.filter(n => 'name' in n).map(n => n.name);
    },
  });
  return r;
}

export function NameFor(decl, ast = globalThis['$']?.data) {
  const { id } = decl;
  let p;

  if(isObject(decl) && decl.ast) decl = decl.ast;

  if(decl.kind && /CXX/.test(decl.kind)) return NamespaceOf(decl, ast) + '';

  if((p = DeepFind(ast, (value, key) => key == 'ownedTagDecl' && value.id == id, deep.RETURN_PATH))) {
    p = p.slice(0, -1);

    let node = DeepGet(ast, p);
    let parent = DeepGet(ast, p.slice(0, -2));

    if(parent.kind == 'TypedefDecl' && parent.name) return parent.name;

    //console.log('p:', p, 'node:', node, 'parent:', parent);

    return node?.type?.desugaredQualType;
  }
}

export function NodeType(n) {
  return n.type
    ? (t => {
        let { typeAliasDeclId, ...type } = t;
        if(typeof typeAliasDeclId == 'string') type.typeAliasDecl = idNodes.get(typeAliasDeclId);

        if(Type.declarations && Type.declarations.has(t.desugaredQualType)) {
          type = Type.declarations.get(t.desugaredQualType);
        }

        if(isObject(type) && isObject(type.type)) type = type.type;
        return new Type(type);
      })(n.type)
    : NodeType(DeepFind(ast, n => isObject(n) && n.type));
}

export function NodeName(n, name) {
  if(typeof name != 'string') name = '';
  if(name == '' && n.name) name = n.name;
  if(isObject(n) && n.tagUsed) name = n.tagUsed + ' ' + name;
  return name;
}

export function* RawLocation(path) {
  for(let node of Hier(path)) {
    if(node.loc) {
      yield node.loc;
    } else if(node.range) {
      yield node.range.begin;
      yield node.range.end;
    }
  }
}

export function* RawRange(path) {
  for(let node of Hier(path)) if(node.range) yield node.range;
}

export function CompleteLocation(path) {
  let a = [];
  for(let raw of RawLocation(path)) {
    a.push(raw);
    if(raw.file) break;
  }
  if(a[0].offset) a = a.filter(l => !(l.offset > a[0].offset));
  let loc = {};
  for(let raw of a.reverse()) Object.assign(loc, raw);
  return new Location(loc);
}

export function CompleteRange(path) {
  let a = [];
  for(let raw of RawRange(path)) {
    a.push(raw);
  }
  if(a[0].begin.offset) a = a.filter(l => !(l.begin.offset > a[0].begin.offset));
  let range = { begin: {}, end: {} };
  for(let { begin, end } of a.reverse()) {
    Object.assign(range.begin, begin);
    Object.assign(range.end, end);
    //console.log('CompleteRange',range)
  }
  return new Range(range.begin, range.end);
}

export function GetLoc(node) {
  let loc, ret;
  if('loc' in node) loc = node.loc;
  else if('range' in node) loc = node.range;
  else return null;
  if('begin' in loc) loc = loc.begin;
  if('expansionLoc' in loc) loc = loc.expansionLoc;

  // if(!('offset' in loc)) return null;

  ret = new Location(loc);

  return ret;
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

export function GetTypeNode(node, ast = globalThis['$'].data) {
  for(let n = [node]; n[0]; n = n[0].inner) {
    let i;

    if((i = n.find(node => /Type/.test(node.kind)))) if (i?.decl?.id) return ast.inner.find(node => node.id == i.decl.id);
  }
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

export class NodeError extends Error {
  constructor(message, node) {
    super(message);

    this.node = node;
  }
}

NodeError.prototype[Symbol.toStringTag] = 'NodeError';

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
    console.log(printer.print(node));

    return printer;
  };
  printer.ast = ast;

  defineProperties(printer, {
    output: {
      get() {
        return out;
      },
    },
    clear: {
      value() {
        out = '';
      },
    },
    put: { value: put },

    print: {
      value(node) {
        let fn = this.nodePrinter[node.kind];
        let oldlen = out.length;
        if(!fn) throw (this.error = new NodeError(`No such printer for ${node.kind}`, node));
        this.node = node;

        let success = fn.call(this.nodePrinter, node, this.ast);

        if(out.length == oldlen) {
          console.log(`printer error at ${loc}`, node);
          throw (this.error = new NodeError(`Node printer for ${node.kind} (${this.loc}) failed\n`, node));
          this.nodePrinter.errorNode = node;

          let { loc } = node;

          if(loc?.line !== undefined) {
            const { line, column } = loc;
            this.loc = {
              line,
              column,
              toString() {
                return [line, column].filter(i => i).join(':');
              },
            };
          }
        }
        // else console.log('out:', out);
        return out;
      },
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
        BuiltinAttr(builtin_attr) {
          const { implicit } = builtin_attr;
          //console.log('BuiltinAttr', builtin_attr);
          put(`/***BuiltinAttr***/`);
        }
        BreakStmt(break_stmt) {
          put('break');
        }
        CallExpr(call_expr) {
          //console.log('CallExpr', call_expr);
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
        RecoveryExpr(recovery_expr) {
          return this.CallExpr(recovery_expr);
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
            if(i++ > 0) put('}; \t\n'.indexOf(out[out.length - 1]) != -1 ? '\n' : ';\n');
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
          const { name, inner } = enum_constant_decl;
          let value = inner[0];
          put(name);
          if(value) {
            put(' = ');
            printer.print(value);
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
          //console.log('FunctionDecl', node.returnType);
          let returnType = node.returnType;
          put(returnType + '\n' + function_decl.name + '(');
          i = 0;
          for(let inner of (function_decl.inner ?? []).filter(n => n.kind == 'ParmVarDecl')) {
            if(i++ > 0) put(', ');
            printer.print(inner);
          }
          put(') ');
          i = 0;
          for(let inner of (function_decl.inner ?? []).filter(n => n.kind != 'ParmVarDecl' && !/Comment/.test(n.kind))) {
            if(i++ > 0) put(' ');
            printer.print(inner);
          }
          // put('');
          return true;
        }
        GotoStmt(goto_stmt) {
          const { targetLabelDeclId } = goto_stmt;
          try {
            let target = DeepFind(this.ast ?? this, n => typeof n == 'object' && n && n.declId == targetLabelDeclId);
            const { name } = target;

            put(`goto ${name}`);
          } catch(e) {
            console.log('GotoStmt', { goto_stmt });
            globalThis.goto_stmt = goto_stmt;

            startInteractive();
            throw e;
          }
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
          let [cond, body, alt] = if_stmt.inner;
          put(`if(`);
          printer.print(cond);
          put(`) `);
          printer.print(body);
          if(alt) {
            put(` else `);
            printer.print(alt);
          }
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
          const { valueCategory, name, isArray, referencedMemberDecl } = member_expr;

          for(let inner of member_expr.inner) {
            let { qualType } = inner.type;

            const isPointer = qualType.endsWith('*');

            //console.log('MemberExpr', { name, qualType, isPointer });

            printer.print(inner);
            put(isPointer ? '->' : '.');
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

          if(paren_list_expr.inner) for(let inner of paren_list_expr.inner) printer.print(inner);
          put(`)`);
        }
        ParmVarDecl(parm_var_decl) {
          let {
            name,
            type: { qualType: type },
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
        RestrictAttr(restrict_attr) {
          put('__restrict');
        }
        ReturnStmt(return_stmt) {
          put('return ');
          if(return_stmt.inner) for(let inner of return_stmt.inner) printer.print(inner);
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
          if(unary_expr_or_type_trait_expr.inner) for(let inner of unary_expr_or_type_trait_expr.inner) printer.print(inner);
        }
        UnaryOperator(unary_operator) {
          const { valueCategory, isPostfix, opcode, canOverflow } = unary_operator;
          if(!isPostfix) put(opcode);
          for(let inner of unary_operator.inner) printer.print(inner);
          if(isPostfix) put(opcode);
        }
        UnresolvedLookupExpr(unresolved_lookup_expr) {
          const { type, valueCategory, usesADL, name, lookups } = unresolved_lookup_expr;
          put(name);
        }
        UnresolvedMemberExpr(unresolved_member_expr) {
          const { type, valueCategory } = unresolved_member_expr;
        }
        VarDecl(var_decl, base_type) {
          let type = new Type(var_decl.type, this.ast);
          put(var_decl.name);
          let subscripts = (type.subscripts ?? []).map(([offset, subscript]) => `[${subscript}]`).join('');
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
        AlignValueAttr(align_value_attr) {
          put(`__attribute__((align_value(`);
          printer.print(align_value_attr.inner[0]);

          put(`)))`);
        }
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
        ClassTemplatePartialSpecializationDecl(class_template_partial_specialization_decl) {}
        ClassTemplateSpecializationDecl(class_template_specialization_decl) {}
        ComplexType(complex_type) {}
        CompoundLiteralExpr(compound_literal_expr) {
          const { type } = compound_literal_expr;

          if(type.qualType) put(`(${type.qualType})`);

          printer.print(compound_literal_expr.inner[0]);
        }
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
        DependentTemplateSpecializationType(dependent_template_specialization_type) {}
        ElaboratedType(elaborated_type) {}
        EnumType(enum_type) {}
        ExprWithCleanups(expr_with_cleanups) {
          for(let inner of expr_with_cleanups.inner) printer.print(inner);
        }
        FinalAttr(final_attr) {}
        FormatArgAttr(format_arg_attr) {}
        FriendDecl(friend_decl) {}
        FunctionNoProtoType(function_no_proto_type) {}
        FunctionProtoType(function_proto_type) {}
        FunctionTemplateDecl(function_template_decl) {}
        GCCAsmStmt(gcc_asm_stmt) {
          put('__asm__ ');
          for(let inner of gcc_asm_stmt.inner) printer.print(inner);
        }
        GNUInlineAttr(gnu_inline_attr) {}
        GNUNullExpr(gnu_null_expr) {
          put(`NULL`);
        }
        ImplicitValueInitExpr(implicit_value_init_expr) {}
        IncompleteArrayType(incomplete_array_type) {}
        IndirectFieldDecl(indirect_field_decl) {}
        IndirectGotoStmt(indirect_goto_stmt) {}
        InjectedClassNameType(injected_class_name_type) {}
        LambdaExpr(lambda_expr) {}
        LinkageSpecDecl(linkage_spec_decl) {
          const { language } = linkage_spec_decl;
          put(language ? `extern "${language}" ` : `extern `);
          for(let inner of linkage_spec_decl.inner) printer.print(inner);
        }
        LValueReferenceType(l_value_reference_type) {}
        MaterializeTemporaryExpr(materialize_temporary_expr) {
          for(let inner of materialize_temporary_expr.inner) printer.print(inner);
        }
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
        OpaqueValueExpr(opaque_value_expr) {
          //console.log('OpaqueValueExpr',opaque_value_expr);
          put(`/***FIXME: OpaqueValueExpr***/`);
        }
        OwnerAttr(owner_attr) {}
        PackedAttr(packed_attr) {}
        PackExpansionExpr(pack_expansion_expr) {}
        PackExpansionType(pack_expansion_type) {}
        ParenType(paren_type) {}
        PointerAttr(pointer_attr) {}
        PointerType(pointer_type) {}
        PredefinedExpr(predefined_expr) {
          const { name } = predefined_expr;

          put(name);
        }
        QualType(qual_type) {}
        RecordType(record_type) {}
        ReturnsNonNullAttr(returns_non_null_attr) {}
        RValueReferenceType(r_value_reference_type) {}
        SentinelAttr(sentinel_attr) {}
        ShuffleVectorExpr(shuffle_vector_expr) {}
        SizeOfPackExpr(size_of_pack_expr) {}
        StaticAssertDecl(static_assert_decl) {}
        StmtExpr(stmt_expr) {
          printer.print(stmt_expr.inner[0]);
        }
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
              if(inner.kind && (inner.kind.endsWith('Comment') || inner.kind == 'CXXRecordDecl')) continue;
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
          let l = cxx_constructor_decl?.inner ? [...cxx_constructor_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')) : [];
          put(`${name}(`);
          while(l.length && l[0].kind == 'ParmVarDecl' && (param = l.shift())) {
            /*if(param.name)*/ {
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
            while(l.length && l[0].kind == 'CXXCtorInitializer' && (initializer = l.shift())) {
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
            anyInit: { name },
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
          const { type, valueCategory, isArrow, member } = cxx_dependent_scope_member_expr;

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
          const { isImplicit, name, mangledName, type, inline, explicitlyDefaulted } = cxx_destructor_decl;

          let l = cxx_destructor_decl.inner ? [...cxx_destructor_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')) : [];
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
            storageClass,
          } = cxx_method_decl;
          const returnType = functionType.slice(0, functionType.indexOf('('));

          //  console.log('CXXMethodDecl', console.config({ depth: 10 }), cxx_method_decl);
          let i = 0,
            param,
            initializer;
          let inner = cxx_method_decl.inner ? [...cxx_method_decl.inner].filter(n => n.kind && !n.kind.endsWith('Comment')) : [];
          if(storageClass) put(`${storageClass} `);
          put(`${returnType}\n`);
          put(`${name}(`);
          while(inner.length && inner[0].kind == 'ParmVarDecl' && (param = inner.shift())) {
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
            castKind,
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
              if(inner.kind == 'DeclRefExpr' && inner.referencedDecl.name == '') continue;
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
            isArray,
          } = cxx_new_expr;
          //console.log('CXXNewExpr', cxx_new_expr);
          type = type.trim();
          if(isArray) {
            put('new ' + type + '[');
          } else {
            put('new ' + type + '(');
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
            isArrayAsWritten,
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
        CXXConstructExpr(cxx_construct_expr) {
          const { type } = cxx_construct_expr;

          for(let inner of cxx_construct_expr.inner) printer.print(inner);
        }
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
        CXXTemporaryObjectExpr(cxx_temporary_object_expr) {
          const { type } = cxx_temporary_object_expr;

          put(type.qualType);
          put(`(`);
          for(let inner of cxx_temporary_object_expr.inner) {
            printer.print(inner);
          }
          put(`)`);
        }
        CXXTryStmt(cxx_try_stmt) {}
      })(),
    },
  });

  return printer;
}

export function PrintNode(node) {
  let out,
    sep = ' ';

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

export function PrintAst(node, ast) {
  if('ast' in node) node = node.ast;
  ast ??= globalThis['$'].data;
  let printer = NodePrinter(ast);
  globalThis.printer = printer;
  defineProperties(printer, {
    path: {
      get() {
        return DeepPathOf(ast, this.node);
      },
    },
  });

  if(Array.isArray(node)) {
    for(let elem of node) {
      if(printer.output) printer.put('\n');
      printer(elem, ast);
    }
  } else {
    printer(node, ast);
  }

  return printer.output;
}

export function isNode(obj) {
  return isObject(obj) && typeof obj.kind == 'string';
}

export function GetType(name_or_id, ast = globalThis['$']?.data) {
  let result, idx;

  if(typeof name_or_id == 'object' && name_or_id) {
    result = name_or_id;
  } else {
    const types = ast.inner.filter(n => /(?:RecordDecl|TypedefDecl|EnumDecl)/.test(n.kind));

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
      let results = types.filter(name_or_id.startsWith('0x') ? node => node.id == name_or_id && wantKind.test(node.kind) : node => node.name == name_or_id && wantKind.test(node.kind));
      if(results.length <= 1 || (idx = results.findIndex(r => r.completeDefinition)) == -1) idx = 0;
      result = results[idx];

      if(!result && Type.declarations.has(name_or_id)) result = Type.declarations.get(name_or_id);
    } else {
      result = types[name_or_id];
    }
  }

  return result;
}

export function GetClass(name_or_id, ast = globalThis['$'].data) {
  let result =
    isString(name_or_id) && /::/.test(name_or_id)
      ? GetByName(name_or_id, ast, n => /RecordDecl/.test(n.kind) && n.completeDefinition)
      : DeepFind(ast, nameOrIdPred(name_or_id, property('kind', regexp(/RecordDecl/)), property('completeDefinition')), deep.RETURN_VALUE);

  if(result) {
    let type = TypeFactory(result, ast);
    if(type) result = type;

    return define(
      result,
      properties(
        {
          bases() {
            const a = [...GetBases(this, ast)];
            if(a.length > 0) return a;
          },
        },
        { enumerable: false, memoize: true },
      ),
    );
  }
}

export function* GetBases(node, ast = globalThis['$'].data) {
  if(node?.ast) node = node.ast;
  if(node?.kind != 'CXXRecordDecl') throw new TypeError(`argument 1 must be ClassDecl / CXXRecordDecl`);
  if(node?.bases) for(let base of node.bases) yield GetClass(base.type.qualType, ast);
}

export function GetByName(arg, ast = globalThis['$'].data, ...args) {
  const ns = isString(arg) ? arg.split('::') : arg;
  let node = ast;

  while(ns.length >= 1) {
    const arg = ns.shift();

    for(let v of DeepSelect(node, nameOrIdPred(arg, ...args))) {
      if(ns.length == 0) return v;

      let r;
      if((r = GetByName([...ns], v))) return r;
    }
  }
}

export function GetNamespace(arg, root = globalThis['$'].data, predicate = () => true) {
  const a = Array.isArray(arg) ? arg : arg.split('::');

  let [name] = a;

  for(let [node, path] of DeepSelect(root, n => (typeof n == 'object' ? (n.name == name ? deep.YIELD_NO_RECURSE : n.name ? deep.NO_RECURSE : deep.RECURSE) : 0), deep.RETURN_VALUE_PATH)) {
    if(a.length <= 1) {
      if(!predicate(node, path)) continue;
      return node;
    }

    let result = GetNamespace(a.slice(1), node, predicate);

    if(result) return result;
  }
}

export function NamespaceOf(node, path, ast = globalThis['$'].data) {
  if(isObject(node) && node.ast) node = node.ast;

  let p = path ?? DeepPathOf(ast, node),
    r = [],
    i = 0;

  while(p?.length > 0) {
    let n = DeepGet(ast, p);

    if(i == 0 || n.kind == 'NamespaceDecl' || n.name) r.push(n.name);
    p = p.slice(0, -2);
    ++i;
  }

  return define(
    r.reverse(),
    nonenumerable({
      toString() {
        return this.join('::');
      },
    }),
  );
}

export function GetFields(node) {
  let fields = [...DeepSelect(node, (v, k) => / at /.test(v) && k == 'qualType', deep.RETURN_VALUE_PATH)].map(([v, p]) => [v.split(/(?:\s*[()]| at )/g)[2], p.slice(0, -2)]);

  return fields.map(([loc, ptr]) =>
    loc
      .split(/:/g)
      .map(i => (!isNaN(+i) ? +i : i))
      .concat([DeepGet(node, ptr).name]),
  );
}

export function GetParams(node) {
  return (node?.inner ?? []).filter(child => child.kind.startsWith('Parm'));
}

export function PathRemoveLoc(path) {
  let idx = path.findIndex(p => p == 'loc' || p == 'range');
  if(idx != -1) path = path.slice(0, idx);
  return path;
}

//export default AstDump;
