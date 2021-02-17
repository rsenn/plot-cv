import Util from './lib/util.js';
import path from './lib/path.js';
import { AcquireReader } from './lib/stream/utils.js';

export let SIZEOF_POINTER = 8;

export class Node {
  static ast2node = new WeakMap();
  static node2ast = new WeakMap();

  constructor(ast) {
    Node.ast2node.set(ast, this);
    Node.node2ast.set(this, ast);
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
    const { name, size } = this;

    return text(Util.className(this), 1, 31) + ' ' + inspect(Util.getMembers(this), depth, opts);
  }
}

export class Type extends Node {
  static declarations = new Map();

  constructor(s) {
    if(typeof s == 'string' && Type.declarations.has(s)) {
      s = Type.declarations.get(s).ast;
    }

    super(s);

    let str = ((Util.isObject(s) && (s.desugaredQualType || s.qualType)) || s) + '';
    str = str.replace(/\s*restrict\s*/g, '');
    str = str.replace(/\s*const\s*/g, '');

    let name, desugared, typeAlias, qualType;

    name = s.name;

    if(s.tagUsed && name) name = (s.tagUsed ? s.tagUsed + ' ' : '') + name;

    if(!Type.declarations.has(str)) Type.declarations.set(str, this);

    if(Util.isObject(s)) {
      if(s.kind == 'EnumDecl' && s.name) qualType = `enum ${s.name}`;
      else {
        if(s.qualType) qualType = s.qualType;
        if(s.desugaredQualType !== undefined) desugared = s.desugaredQualType;
        if(s.typeAliasDeclId !== undefined) typeAlias = s.typeAliasDeclId;
      }
    } else {
      qualType = str + '';
    }
    if(s.desugaredQualType || s.qualType) desugared = s.desugaredQualType || s.qualType;
    if(s.typeAliasDeclId) typeAlias = s.typeAliasDeclId;

    if(desugared === qualType) desugared = undefined;

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
    let match = Util.if(/^([^\(\)]*)(\(?\*\)?\s*)(\(.*\)$|)/g.exec(str),
      m => [...m].slice(1),
      () => []
    );
    if(match[1]) return true;
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
    const re = /^[^0-9]*([0-9]+)(_t|)$/;
    let size, match;
   /* if((match = re.exec(desugared))) {
      const [, bits] = match;
      if(!isNaN(+bits)) return +bits / 8;
    }*/
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
      if(match[3]) {
        const num = parseInt(match[3]);
        //console.log('num:', { match, size, num });
        size *= num;
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
    // console.log('Type.inspect:', { name, size, opts });
    let props = Util.getMembers(this);
    props.size = size;

    return text('Type', 1, 31) + ' ' + inspect(props, depth, opts);
  }

  get [Symbol.toStringTag]() {
    return `${((this.typeAlias && this.typeAlias + '/') || '') + this.name}, ${this.size}${
      (this.pointer && ', ' + this.pointer) || ''
    }`;
  }

  [Symbol.toPrimitive](hint) {
    if(hint == 'default' || hint == 'string') return this.name; //this+'';
    return this;
  }
}

export class RecordDecl extends Node {
  constructor(node, ast) {
    super(node);

    const { tagUsed, name, inner } = node;

    if(tagUsed) this.name = tagUsed + ' ' + name;
    else this.name = name;

    let fields = inner.filter(child => child.kind == 'FieldDecl');
    console.log('RecordDecl', node, fields);

    this.members = new Map(fields.map(({ name, type }) => [name, TypeFactory(type, ast)]));
  }
}

export class EnumDecl extends Node {
  constructor(node, ast) {
    super(node);

    if(node.name) this.name = node.name;

    let constants = node.inner.filter(child => child.kind == 'EnumConstantDecl');
    this.members = new Map(constants.map(({ name, type }) => [name, TypeFactory(type, ast)]));
  }
}

export class TypedefDecl extends Node {
  name = null;
  type = null;

  constructor(node, ast) {
    super(node);

    this.name = node.name;
    this.type = TypeFactory(GetType(node, ast), ast);
  }
}

export class Location {
  constructor(loc) {
    const { line, col, file } = loc;

    Object.assign(this, { line, col, file });
  }

  inspect(depth, opts = {}) {
    const text = opts.colors ? (t, ...c) => '\x1b[' + c.join(';') + 'm' + t + '\x1b[m' : t => t;
    const { file, line, col } = this;

    return text('Location', 38, 5, 111) + ' [ ' + [file, line, col].join(':') + ' ]';
  }
}

export function TypeFactory(node, ast) {
  let obj;

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
    default: obj = new Type(node, ast);
      break;
  }
  return obj;
}

export async function SpawnCompiler(file, args = []) {
  let base = path.basename(file, /\.[^.]*$/);
  let outputFile = base + '.ast.json';

  /* console.log('globalThis.spawn:', globalThis.spawn);*/
  //  let output = filesystem.open(, filesystem.O_CREAT|filesystem.O_TRUNC|filesystem.O_WRONLY, 420);

  //  args.push(`-o${outputFile}`);
  args.push(file);
  args.unshift('clang');

  let argv = [
    'sh',
    '-c',
    `exec ${args.map(p => (/\ /.test(p) ? `'${p}'` : p)).join(' ')} 1>${outputFile}`
  ];

  console.log(`SpawnCompiler: ${argv.map(p => (/\ /.test(p) ? `"${p}"` : p)).join(' ')}`);

  let child = spawn(argv, {
    block: false,
    stdio: ['inherit', 'inherit', 'pipe']
  });
  console.log('child:', child);

  let json = '',
    errors = '';
  let done = false;

  if(Util.platform == 'quickjs') {
    let { fd } = child.stderr;
    /* (async function() {
      let r;
      let buf = new ArrayBuffer(1024);
      while(!done) {
     // await filesystem.waitRead(fd);
       // r = await child.stderr.read(buf);
      r = filesystem.read(fd, buf);
        console.log('r:',r, Util.typeOf(r), 'errors:',errors);
  errors += filesystem.bufferToString(buf, buf.slice(0, r));
      }
    })();*/

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
  console.log('errors:', errors);
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  errorLines = errorLines.filter(line => /error:/.test(line));
  const numErrors =
    [
      ...(/^([0-9]+)\s/g.exec(errorLines.find(line => /errors\sgenerated/.test(line)) || '0') || [])
    ][0] || errorLines.length;

  console.log(`numErrors: ${numErrors}`);
  if(numErrors) throw new Error(errorLines.join('\n'));
  console.log('errorLines:', errorLines);

  function ReadErrors(fd) {
    let buf = new ArrayBuffer(1024);
    let r = os.read(fd, buf, 0, buf.byteLength);
    errors += filesystem.bufferToString(buf.slice(0, r));
    console.log('r:', r, 'errors:', errors.length);
  }

  //let fd = filesystem.open(outputFile, filesystem.O_RDONLY);
  return { file: outputFile };
}

export async function AstDump(file, args) {
  console.log('AstDump', { file, args });
  let r = await SpawnCompiler(file, ['-Xclang', '-ast-dump=json', '-fsyntax-only', '-I.', ...args]);
  console.log('AstDump', { r });

  //r.size = (await filesystem.stat(r.file)).size;
  return Util.lazyProperties(r, {
    size() {
      return filesystem.stat(this.file)?.size;
    },
    json() {
      return filesystem.readFile(this.file);
    },
    data() {
      return JSON.parse(this.json);
    },
    types() {
      return this.data.inner.filter(n => /(Record|Typedef|Enum)Decl/.test(n.kind));
    },
    functions() {
      return this.data.inner.filter(n => /(Function)Decl/.test(n.kind));
    },
    variables() {
      return this.data.inner.filter(n => /(Var)Decl/.test(n.kind));
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
      console.log('GetType', { type, id: type.id });
      let declType = ast.inner.find(n => n.id == type.id);
      if(!declType) throw new Error(`Type ${type.id} not found`);
      type = declType;
    }
  }
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

//export default AstDump;
