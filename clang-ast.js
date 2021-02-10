import Util from './lib/util.js';
import path from './lib/path.js';
import { AcquireReader } from './lib/stream/utils.js';

export let SIZEOF_POINTER = 8;

export class Type {
  static declarations = new Map();

  constructor(s) {
    if(typeof s == 'string' && Type.declarations.has(s)) {
      s = Type.declarations.get(s);
    }

    let str = ((Util.isObject(s) && (s.desugaredQualType || s.qualType)) || s) + '';
    str = str.replace(/\s*restrict\s*/g, '');
    str = str.replace(/\s*const\s*/g, '');

    //    super(str);

    if(Util.isObject(s)) {
      if(s.kind == 'EnumDecl' && s.name) Util.define(this, { qualType: `enum ${s.name}` });
      else {
        if(s.qualType) Util.define(this, { qualType: s.qualType });
        if(s.desugaredQualType !== undefined)
          Util.define(this, { desugaredQualType: s.desugaredQualType });
        if(s.typeAliasDeclId !== undefined)
          Util.define(this, { typeAliasDeclId: s.typeAliasDeclId });
      }
    } else {
      Util.define(this, { qualType: str + '' });
    }
  }

  get name() {
    return this.qualType || this.desugaredQualType || this.typeAliasDeclId;
  }

  get desugared() {
    return this.desugaredQualType || this.qualType;
  }

  get typeAlias() {
    return this.typeAliasDeclId;
  }
  get regExp() {
    return new RegExp(`(?:${this.qualType}${this.typeAlias ? '|' + this.typeAlias : ''})`.replace(/\*/g, '\\*'),
      'g'
    );
  }
  get subscripts() {
    let match;
    let str = this + '';
    while((match = /\[([0-9]+)\]/g.exec(str))) {
      console.log('match:', match);
    }
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

    const { desugared: name } = this;
    const re = /^[^0-9]*([0-9]+)(_t|)$/;
    let size, match;
    if((match = re.exec(name))) {
      const [, bits] = match;
      if(!isNaN(+bits)) return +bits / 8;
    }
    if((match = /^[^\(]*\(([^\)]*)\).*/.exec(name))) {
      if(match[1] == '*') return SIZEOF_POINTER;
    }
    match = /^(unsigned\s+|signed\s+|const\s+|volatile\s+|long\s+|short\s+)*([^\[]*[^ \[\]]) *\[?([^\]]*).*$/g.exec(name
    );
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
        case 'float':
        case 'unsigned int':
        case 'int': 
          size = 4;
          break;
        case 'long double': 
          size = 16;
          break;
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
      // throw new Error(`Type sizeof(${name}) ${this.desugaredQualType}`);
      size = NaN;
    }
    return size;
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
  valueOf() {
    return this.name;
  }

  toString() {
    return this.name;
  }
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
    `exec ${args.map(p => (/ /.test(p) ? `'${p}'` : p)).join(' ')} 1>${outputFile}`
  ];

  console.log(`SpawnCompiler: ${argv.map(p => (/ /.test(p) ? `"${p}"` : p)).join(' ')}`);

  let child = spawn(argv, {
    block: false,
    stdio: ['inherit', 'inherit', 'pipe']
  });
  console.log('child:', child);

  let json = '',
    errors = '';

  /* if(Util.platform == 'quickjs') {
    (function () {
      let r;
      let buf = new ArrayBuffer(1024);
      r = filesystem.readAll(child.stderr.fd);
      errors += r;
    })();
  } else {
    AcquireReader(child.stderr, async reader => {
      let r;
      while((r = await reader.read())) {
        if(!r.done) errors += r.value.toString();
      }
    });
  }*/
  console.log('child.wait():', await child.wait());
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

export function GetType(node) {
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

//export default AstDump;
