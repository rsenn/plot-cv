import * as fs from 'fs';
import * as os from 'os';
import { define, predicate } from './lib/misc.js';
import * as path from './lib/path.js';

export function* DirIterator(...args) {
  let pred = typeof args[0] != 'string' ? predicate(args.shift()) : () => true;
  for(let dir of args) {
    dir = dir.replace(/~/g, process.env.HOME);
    let entries = os.readdir(dir)[0] ?? [];
    for(let entry of entries.sort()) {
      let file = path.join(dir, entry);
      let lst = os.lstat(file)[0];
      let st = os.stat(file)[0];
      let is_dir = (st?.mode & os.S_IFMT) == os.S_IFDIR;
      let is_symlink = (lst?.mode & os.S_IFMT) == os.S_IFLNK;

      if(is_dir) file += '/';
      if(!pred(entry, file, is_dir, is_symlink)) continue;
      yield file;
    }
  }
}

export function* RecursiveDirIterator(dir, pred = (entry, file, dir, depth) => true, depth = 0) {
  let re;
  if(typeof pred != 'function') {
    if(!pred) pred = '.*';
    if(typeof pred == 'string') pred = new RegExp(pred, 'gi');
    re = pred;
    pred = (entry, file, dir, depth) => re.test(entry) || re.test(file);
  }
  if(!dir.endsWith('/')) dir += '/';
  dir = dir.replace(/~/g, process.env.HOME);
  for(let file of fs.readdirSync(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;
    let entry = `${dir}${file}`;
    let isDir = false;
    let st = fs.statSync(entry);
    isDir = st && st.isDirectory();
    if(isDir) entry += '/';
    let show = pred(entry, file, dir, depth);
    if(show) {
      yield entry;
    }
    if(isDir) yield* RecursiveDirIterator(entry, pred, depth + 1);
  }
}

export function* ReadDirRecursive(dir, maxDepth = Infinity) {
  dir = dir.replace(/~/g, globalThis.process.env['HOME'] ?? getenv('HOME'));
  for(let file of fs.readdirSync(dir)) {
    if(['.', '..'].indexOf(file) != -1) continue;
    let entry = `${dir}/${file}`;
    let isDir = false;
    let st = fs.statSync(entry);
    isDir = st && st.isDirectory();
    yield isDir ? entry + '/' : entry;
    if(maxDepth > 0 && isDir) yield* ReadDirRecursive(entry, maxDepth - 1);
  }
}

export class Path {
  #string = null;

  constructor(str) {
    if(typeof str == 'string') str = path.normalize(str);
    this.#string = str;
  }

  assign(str) {
    this.#string = str;
    return this;
  }

  append(str) {
    this.#string = path.join(this.#string, str);
    return this;
  }

  concat(str) {
    this.#string += str;
    return this;
  }

  clear() {
    this.#string = undefined;
    return this;
  }

  removeFilename() {
    this.#string = path.dirname(this.#string);
    return this;
  }

  replaceFilename(filename) {
    this.remove_filename();
    this.append(filename);
    return this;
  }

  replaceExtension(ext) {
    this.#string = this.#string.replace(/(\.[^\/.]+|)$/, ext);
    return this;
  }

  get filename() {
    return path.basename(this.#string);
  }

  get stem() {
    const ext = path.extname(this.#string);
    return this.#string.slice(0, this.#string.length - ext.length);
  }

  get extension() {
    return path.extname(this.#string);
  }

  get empty() {
    return this.#string === '' || this.#string === undefined;
  }

  get parent() {
    return new (this[Symbol.species] ?? Path)(path.dirname(this.#string));
  }

  get isAbsolute() {
    return path.isAbsolute(this.#string);
  }
  get isRelative() {
    return path.isRelative(this.#string);
  }

  relativeTo(to) {
    return new Path(path.relative(to.#string, this.#string));
  }

  compare(other) {
    return this.#string.localeCompare(other.#string, 'en-US');
  }

  [Symbol.toPrimitive](hint) {
    console.log('Path.toPrimitive', hint);
    switch (hint) {
      case 'number':
        return valuePointer(this);
      case 'string':
      default:
        return this.#string;
      //throw new Error(`Path.toPrimitive (hint=${hint})`);
    }
  }

  [Symbol.inspect](depth, opts = {}) {
    return `\x1b[1;31m${this[Symbol.toStringTag]}\x1b[0;32m ` + this.#string + `\x1b[0m`;
  }

  static testPaths = [
    ['/foo/bar.jpg', '.png'],
    ['/foo/bar.jpg', 'png'],
    ['/foo/bar.jpg', '.'],
    ['/foo/bar.jpg', ''],
    ['/foo/bar.', 'png'],
    ['/foo/bar', '.png'],
    ['/foo/bar', 'png'],
    ['/foo/bar', '.'],
    ['/foo/bar', ''],
    ['/foo/.', '.png'],
    ['/foo/.', 'png'],
    ['/foo/.', '.'],
    ['/foo/.', ''],
    ['/foo/', '.png'],
    ['/foo/', 'png']
  ];
}

define(Path.prototype, { [Symbol.toStringTag]: 'Path' });

/* same as proto[Symbol.operatorSet] = Operators.create(..op_list)
       but allow shortcuts: left: [], right: [] or both
    */
function operators_set(proto, ...op_list) {
  var new_op_list, i, a, j, b, k, obj, tab;
  var fields = ['left', 'right'];
  new_op_list = [];
  for(i = 0; i < op_list.length; i++) {
    a = op_list[i];
    if(a.left || a.right) {
      tab = [a.left, a.right];
      delete a.left;
      delete a.right;
      for(k = 0; k < 2; k++) {
        obj = tab[k];
        if(obj) {
          if(!Array.isArray(obj)) {
            obj = [obj];
          }
          for(j = 0; j < obj.length; j++) {
            b = {};
            Object.assign(b, a);
            b[fields[k]] = obj[j];
            new_op_list.push(b);
          }
        }
      }
    } else {
      new_op_list.push(a);
    }
  }
  //proto[Symbol.operatorSet] = Operators.create.call(null, ...new_op_list);
}

//Path.prototype[Symbol.operatorSet] = Operators.create

operators_set(
  Path.prototype,
  {
    '/': (a, b) => new Path(path.join(a + '', b + '')),
    '/=': (a, b) => a.append(b),
    '==': (a, b) => a.compare(b) == 0,
    '<': (a, b) => a.compare(b) < 0,
    '+=': (a, b) => a.concat(b),
    pos(a) {
      return valuePointer(a);
    }
  },
  {
    right: [String, Number],
    '/'(a, b) {
      // console.log('/', { a, b });
      return new Path(path.join(a + '', stringPointer(b)));
    },
    '/='(a, b) {
      // console.log('/=', { a, b  });
      a.append(stringPointer(b));
    }
  }
);