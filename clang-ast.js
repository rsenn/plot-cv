import Util from './lib/util.js';
import path from './lib/path.js';
import { AcquireReader } from './lib/stream/utils.js';

export let SIZEOF_POINTER = 8;

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
}

export class Type extends Node {
  static declarations = new Map();

  constructor(node) {
    let name, desugared, typeAlias, qualType;

    if(typeof node == 'string') {
      qualType = node;
      if(Type.declarations.has(node)) node = Type.declarations.get(node).ast;
      else node = {};
    }

    if(node instanceof Node) {
      Util.putStack();
      throw new Error();
    }

    super(node);

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
  }

  /*  get name() {
    return this.qualType || this.desugaredQualType || this.typeAliasDeclId;
  }*/

  /* get desugared() {
    return this.desugaredQualType || this.qualType;
  }*/

  /*  get typeAlias() {
    return this.typeAliasDeclId;
  }*/
  get regExp() {
    return new RegExp(`(?:${this.qualType}${this.typeAlias ? '|' + this.typeAlias : ''})`.replace(/\*/g, '\\*'),
      'g'
    );
  }
  get subscripts() {
    let match;
    let str = this + '';
    let ret=[];
    while((match = /\[([0-9]+)\]/g.exec(str))) {
ret.push(match[1]);
      //console.log('match:', match);
    }
    return ret;
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
    if(this.isArray())
      return [...Util.matchAll(/\[([0-9]*)\]/g, this+'')].map(m => +m[1]);
  }

  get pointer() {
    let str = this + '';
    let match = Util.if(/^([^\(\)]*)(\(?\*\)?\s*)(\(.*\)$|)/g.exec(str),
      (m) => [...m].slice(1),
      () => []
    );
     if(match[1]) return new Type([match[0].trimEnd(), match[2]].join(''));
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
    if(this.pointer == 'char') return 'char *';
    if(this.size == SIZEOF_POINTER && !this.isPointer())
      return ['', 'unsigned '][this.unsigned | 0] + 'long';
    // if(this.size == SIZEOF_POINTER && this.unsigned) return 'size_t';
    if(this.size == SIZEOF_POINTER / 2 && !this.unsigned) return 'int';
    if(this.size == 4) return ['', 'u'][this.unsigned | 0] + 'int32';
    if(this.size == 2) return ['', 'u'][this.unsigned | 0] + 'int16';
    if(this.size == 1) return ['', 'u'][this.unsigned | 0] + 'int8';

    if(this.isPointer()) return 'void *';
    if(this.size > SIZEOF_POINTER) return 'void *';
    if(this.size === 0) return 'void';

    if(Type.declarations.has(this + '')) {
      let decl = Type.declarations.get(this + '');
      if(decl.kind == 'EnumDecl') return 'int';
    } else {
      throw new Error(`No ffi type '${this}' ${this.size}`);
    }
  }

  get size() {
    if(this.isPointer()) return SIZEOF_POINTER;

    const { desugared = this.qualType } = this;
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
        for(let subscript of this.subscripts)
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
    if(hint == 'default' || hint == 'string') return this.qualType; //this+'';
    return this;
  }
}

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

    let fields = inner?.filter(child => !child.isImplicit && child.kind.endsWith('Decl'));
    //console.log('RecordDecl', fields);

    if(fields)
      this.members = /*new Map*/ fields
        .filter(node => !node.isImplicit)
        .map(node => {
          let name = node.name;
          if(node.isBitfield) name += ':1';
          if(node.kind == 'FieldDecl')
            return [name, node.type?.kind ? TypeFactory(node.type, ast) : new Type(node.type, ast)];

          return [name, TypeFactory(node, ast)];
        });
  }

  get size() {
    return RoundTo([...this.members.values()].reduce((acc,member) => {
      if(member.size == 8)
        acc = RoundTo(acc, 8);
      return acc + RoundTo(member.size,4);
    }, 0), SIZEOF_POINTER);
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
}

export class TypedefDecl extends Type {
  constructor(node, ast) {
    super(node, ast);

    let inner = node.inner.filter(n => !/Comment/.test(n.kind));
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

    let parameters = node.inner.filter(child => child.kind == 'ParmVarDecl');
    let type = node.type?.qualType;
    let returnType = type.replace(/\ \(.*/, '');

    // console.log('parameters:', parameters);

    this.returnType = new Type(returnType, ast);
    this.parameters = new Map(parameters.map(({ name, type }) => [name, new Type(type, ast)]));
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

export async function SpawnCompiler(file, args = []) {
  let base = path.basename(file, /\.[^.]*$/);
  let outputFile = base + '.ast.json';

  args.push(file);
  args.unshift('clang');

  if(args.indexOf('-ast-dump=json') != -1)
    args = [
      'sh',
      '-c',
      `exec ${args.map(p => (/\ /.test(p) ? `'${p}'` : p)).join(' ')} 1>${outputFile}`
    ];

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

    os.setReadHandler(fd, () => {
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
  let result = await child.wait();
  console.log('child.wait():', result);
  os.setReadHandler(child.stderr.fd, null);

  if(result[1] != 0) ReadErrors(child.stderr.fd);
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
    let r = os.read(fd, buf, 0, buf.byteLength);
    errors += filesystem.bufferToString(buf.slice(0, r));
    //console.log('r:', r, 'errors:', errors.length);
  }

  //let fd = filesystem.open(outputFile, filesystem.O_RDONLY);
  return { file: outputFile };
}

export async function AstDump(source, args) {
  console.log('AstDump', { source, args });
  let r = await SpawnCompiler(source, [
    '-Xclang',
    '-ast-dump=json',
    '-fsyntax-only',
    '-I.',
    ...args
  ]);
  console.log('AstDump', { r });

  //r.size = (await filesystem.stat(r.file)).size;
  r = Util.lazyProperties(r, {
    size() {
      return filesystem.stat(this.file)?.size;
    },
    json() {
      return filesystem.readFile(this.file);
    },
    data() {
      let data = JSON.parse(this.json);
      let file;

      //data.inner.forEach
      /*deep.forEach(data, loc => {
        if(loc && loc.offset !== undefined) {
          if(loc.file) file = loc.file;
          else loc.file = file;
        }
      });*/

      //Util.instrument(async function() {
      let maxDepth = 0;
      //for(let [loc, path] of deep.iterate(data, (n, p) =>  n && n.offset != undefined)) {
      for(let node of data.inner) {
        let loc;

        //maxDepth = Math.max(maxDepth, path.length);
        if((loc = node.loc)) {
          if(loc.file) file = loc.file;
          else loc.file = file;
        }
      }

      // filesystem.writeFile(this.file, JSON.stringify(data, null, 2));
      //})();

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
      /*pred = Util.predicate(pred, (node,fn) => fn(node.loc.file));
      console.log("pred:",pred+'')*/
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
  if('expansionLoc' in loc) loc = loc.expansionLoc;
  if('begin' in loc) loc = loc.begin;

  // if(!('offset' in loc)) return null;

  return loc;
}

export function GetType(node, ast) {
  let type, elaborated;

  if((elaborated = node.inner.find(n => n.kind == 'ElaboratedType'))) {
    if((type = elaborated.inner.find(n => n.decl))) type = type.decl;
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
