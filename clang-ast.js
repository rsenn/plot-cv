import PortableSpawn from './lib/spawn.js';
import Util from './lib/util.js';
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
        if(s.desugaredQualType !== undefined) Util.define(this, { desugaredQualType: s.desugaredQualType });
        if(s.typeAliasDeclId !== undefined) Util.define(this, { typeAliasDeclId: s.typeAliasDeclId });
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
        case 'char': {
          size = 1;
          break;
        }
        case 'int8_t':
        case 'uint8_t':
        case 'bool': {
          size = 1;
          break;
        }
        case 'size_t':
        case 'ptrdiff_t':
        case 'void *':
        case 'long': {
          size = SIZEOF_POINTER;
          break;
        }
        case 'float':
        case 'unsigned int':
        case 'int': {
          size = 4;
          break;
        }
        case 'long double': {
          size = 16;
          break;
        }
        case 'long long': {
          size = 8;
          break;
        }
        case 'float': {
          size = 4;
          break;
        }
        case 'double': {
          size = 8;
          break;
        }
        case 'void': {
          size = 0;
          break;
        }
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

export async function AstDump(file, args) {
  args = ['-Xclang', '-ast-dump=json', '-fsyntax-only', '-I.', ...args];
  let child = spawn(['clang', ...args, file], {
    //    stdin: 'inherit',
    block: false,
    stdio: ['inherit', 'pipe', 'pipe']
    //  stderr: 'pipe'
  });

  let json = '',
    errors = '';
  console.log('child:', child);

  if(Util.platform == 'quickjs') {
    (function () {
      let r;
      let buf = new ArrayBuffer(1024);
      console.log('stdout:', child.stdout);
      r = filesystem.readAll(child.stdout.fd);
      //const str = filesystem.bufferToString(buf.slice(0, r));
      json += r;
      console.log('read:', r);
    })();
  } else {
    AcquireReader(child.stdout, async reader => {
      let r, str;
      while((r = await reader.read())) {
        if(!r.done) {
          str = r.value.toString();
          console.log('stdout:');
          json += str;
        }
      }
    });

    AcquireReader(child.stderr, async reader => {
      let r;
      while((r = await reader.read())) {
        if(!r.done) errors += r.value.toString();
      }
    });
  }
  console.log('child.wait():', await child.wait());
  console.log('errors:', errors);
  let errorLines = errors.split(/\n/g).filter(line => line.trim() != '');
  errorLines = errorLines.filter(line => /error:/.test(line));
  const numErrors =
    [...(/^([0-9]+)\s/g.exec(errorLines.find(line => /errors\sgenerated/.test(line)) || '0') || [])][0] ||
    errorLines.length;

  console.log(`numErrors: ${numErrors}`);
  if(numErrors) throw new Error(errorLines.join('\n'));
  console.log('errorLines:', errorLines);
  return json;
}

export default AstDump;
