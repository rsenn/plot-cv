import * as fs from 'fs';
import * as path from './lib/path.js';
import { types, toString, quote, escape, predicate } from './lib/misc.js';
import child_process from 'child_process';

let bjson;

import('bjson') .then(m => (bjson = m)) .catch(() => {});

let mmap;

import('mmap') .then(m => (mmap = m)) .catch(() => {});

let xml;

import('xml') .then(m => (xml = m)) .catch(() => {});

export function IfDebug(token) {
  const { DEBUG = '' } = globalThis.process ? globalThis.process.env : {}; //std.getenviron();

  const tokList = DEBUG.split(/[^A-Za-z0-9_]+/g);

  return tokList.indexOf(token) != -1;
}

export function LogIfDebug(token, loggerFn) {
  if(!IfDebug(token)) return () => {};

  return loggerFn;
}

const debug = LogIfDebug('io-helpers', (...args) => console.log(...args));

export function ReadFd(fd, binary) {
  let ab = new ArrayBuffer(1024);
  let out = '';
  for(;;) {
    let ret = fs.readSync(fd, ab, 0, 1024);

    if(ret <= 0) break;

    out += toString(ab.slice(0, ret));

    debug(`Read #${fd}: ${ret} bytes`);
  }
  return out;
}

export function ReadFile(name, binary) {
  let ret = fs.readFileSync(name, binary ? null : 'utf-8');

  debug(`Read ${name}: ${ret?.byteLength} bytes`);
  return ret;
}

export function LoadHistory(filename) {
  let contents = fs.readFileSync(filename, 'utf-8');
  let data;

  const parse = () => {
    try {
      data = JSON.parse(contents);
    } catch(e) {}
    if(data) return data;
    try {
      data = contents.split(/\n/g);
    } catch(e) {}
    if(data) return data;
  };

  return (parse() ?? []).filter(entry => (entry + '').trim() != '');
  //.map(entry => entry.replace(/\\n/g, '\n'))
}

export function ReadJSON(filename) {
  let data = fs.readFileSync(filename, 'utf-8');

  if(data) debug(`ReadJSON: ${data.length} bytes read from '${filename}'`);
  return data ? JSON.parse(data) : null;
}

export function ReadXML(filename, ...args) {
  let data = fs.readFileSync(filename, null);

  if(data) debug(`ReadXML: ${data.length} bytes read from '${filename}'`);
  return data ? xml.read(data, filename, ...args) : null;
}

export function MapFile(filename) {
  let fd = os.open(filename, os.O_RDONLY);
  let { size } = os.stat(filename)[0];
  debug(`MapFile`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  os.close(fd);
  return data;
}

export function WriteFile(name, data, verbose = true) {
  if(typeof data == 'object' && data !== null && Symbol.iterator in data) {
    let fd = fs.openSync(name, os.O_WRONLY | os.O_TRUNC | os.O_CREAT, 0o644);
    let r = 0;
    for(let item of data) {
      r += fs.writeSync(fd, toArrayBuffer(item + ''));
    }
    fs.closeSync(fd);
    let stat = fs.statSync(name);
    return stat?.size;
  }
  if(fs.existsSync(name)) fs.unlinkSync(name);

  if(Array.isArray(data)) data = data.join('\n');

  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = fs.writeFileSync(name, data);

  if(verbose) debug(`Wrote ${name}: ${ret} bytes`);
  return ret;
}

export function WriteJSON(name, data, ...args) {
  const [compact] = args;
  if(typeof compact == 'boolean') args = compact ? [] : [null, 2];

  return WriteFile(name, JSON.stringify(data, ...args));
}

export function WriteXML(name, data, ...args) {
  return WriteFile(name, xml.write(data, ...args));
}

export function ReadBJSON(filename) {
  let fd = os.open(filename, os.O_RDONLY);
  let { size } = os.stat(filename)[0];
  debug(`ReadBJSON`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  debug(`ReadBJSON`, { data });
  let ret = bjson.read(data, 0, size);

  mmap.munmap(data);
  os.close(fd);
  return ret;
}

export function WriteBJSON(name, data) {
  let buf = bjson.write(data);
  let size = buf.byteLength;
  let fd = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_TRUNC);

  let ret = os.write(fd, buf, 0, size);
  debug('WriteBJSON', { name, fd, size, ret });
  os.close(fd);

  return ret;
}

export function* DirIterator(...args) {
  let pred = typeof args[0] != 'string' ? predicate(args.shift()) : () => true;
  for(let dir of args) {
    dir = dir.replace(/~/g, std.getenv('HOME'));
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
  dir = dir.replace(/~/g, std.getenv('HOME'));
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
  dir = dir.replace(/~/g, globalThis.process.env['HOME'] ?? std.getenv('HOME'));
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

export function* Filter(gen, regEx = /.*/) {
  for(let item of gen) if(regEx.test(item)) yield item;
}

export function FilterImages(gen) {
  return Filter(gen, /\.(png|jpe?g)$/i);
}

export function SortFiles(arr, field = 'ctime') {
  return [...arr].sort((a, b) => a.stat[field] - b.stat[field]);
}

export function* StatFiles(gen) {
  for(let file of gen) {
    let stat = fs.statSync(file);
    let obj = define(
      { file, stat },
      {
        toString() {
          return this.file;
        }
      }
    );
    Object.defineProperty(obj, 'size', {
      get: memoize(() => {
        let { filename, ...info } = ImageInfo(obj.file);
        return define(info, {
          toString() {
            return this.width + 'x' + this.height;
          },
          get landscape() {
            return this.width > this.height;
          },
          get portrait() {
            return this.height > this.width;
          }
        });
      })
    });
    yield obj;
  }
}

/*export function ReadFd(fd, bufferSize) {
  function* FdRead() {
    let ret,
      buf = new ArrayBuffer(bufferSize);
    do {
      if((ret = fs.readSync(fd, buf, 0, buf.byteLength)) > 0) yield ret == buf.byteLength ? buf : buf.slice(0, ret);
    } while(ret > 0);
  }
  return [...FdRead()].reduce((acc, buf) => (acc += toString(buf)), '');
}*/
/*
export function FdReader(fd, bufferSize = 1024) {
  let buf = new ArrayBuffer(bufferSize);
  return new Repeater(async (push, stop) => {
    let ret;
    do {
      let r = await waitRead(fd);
      ret = typeof fd == 'number' ? fs.readSync(fd, buf) : fd.read(buf);
      if(ret > 0) {
        let data = buf.slice(0, ret);
        await push(fs.bufferToString(data));
      }
    } while(ret == bufferSize);
    stop();
    typeof fd == 'number' ? fs.closeSync(fd) : fd.destroy();
  });
}*/
export async function* FdReader(fd, bufferSize = 1024) {
  let buf = new ArrayBuffer(bufferSize);
  let ret;
  do {
    let r = await waitRead(fd);
    console.log('r', r);
    ret = typeof fd == 'number' ? await fs.read(fd, buf) : await fd.read(buf);
    if(ret > 0) {
      let data = buf.slice(0, ret);
      yield fs.bufferToString(data);
    }
  } while(ret == bufferSize);
  typeof fd == 'number' ? await fs.close(fd) : fd.destroy();
  return;
}

export function CopyToClipboard(text) {
  const { env } = process;
  let child = child_process.spawn('xclip', ['-in', '-verbose'], { env, stdio: ['pipe', 'inherit', 'inherit'] });
  let [pipe] = child.stdio;

  let written = fs.writeSync(pipe, text, 0, text.length);
  fs.closeSync(pipe);
  let status = child.wait();
  console.log('child', child);
  return { written, status };
}

export function ReadCallback(fd, fn = data => {}) {
  let buf = new ArrayBuffer(1024);
  os.setReadHandler(fd, () => {
    let r = fs.readSync(fd, buf, 0, 1024);
    if(r <= 0) {
      fs.closeSync(fd);
      os.setReadHandler(fd, null);
      return;
    }
    let data = buf.slice(0, r);
    data = toString(data);
    fn(data);
  });
}

export function LogCall(fn, thisObj) {
  let { name } = fn;
  return function(...args) {
    let result;
    result = fn.apply(thisObj ?? this, args);
    console.log('Function ' + name + '(', ...args.map(arg => inspect(arg, { colors: false, maxStringLength: 20 })), ') =', result);
    return result;
  };
}

export function Spawn(file, args, options = {}) {
  let { block = true, usePath = true, cwd, stdio = ['inherit', 'inherit', 'inherit'], env, uid, gid } = options;
  let parent = [...stdio];

  for(let i = 0; i < 3; i++) {
    if(stdio[i] == 'pipe') {
      let [r, w] = os.pipe();
      stdio[i] = i == 0 ? r : w;
      parent[i] = i == 0 ? w : r;
    } else if(stdio[i] == 'inherit') {
      stdio[i] = i;
    }
  }

  const [stdin, stdout, stderr] = stdio;
  let pid = os.exec([file, ...args], { block, usePath, cwd, stdin, stdout, stderr, env, uid, gid });
  for(let i = 0; i < 3; i++) {
    if(typeof stdio[i] == 'number' && stdio[i] != i) os.close(stdio[i]);
  }

  return {
    pid,
    stdio: parent,
    wait() {
      let [ret, status] = os.waitpid(this.pid, os.WNOHANG);
      return ret;
    }
  };
}

// 'https://www.discogs.com/sell/order/8369022-364'

export function FetchURL(url, options = {}) {
  let { headers, proxy, cookies = 'cookies.txt', range, body, version = '1.1', tlsv, 'user-agent': userAgent } = options;

  let args = Object.entries(headers ?? {})
    .reduce((acc, [k, v]) => acc.concat(['-H', `${k}: ${v}`]), [])
    .concat(Array.isArray(url) ? url : [url]);

  args.push('--compressed');
  args.unshift('-L', '-k');

  if(body) args.unshift('-d', body);
  if(version) args.unshift('--http' + version);
  if(tlsv) args.unshift('--tlsv' + tlsv);
  if(userAgent) args.unshift('-A', userAgent);
  if(range) args.unshift('-r', range);
  if(cookies) args.unshift('-c', cookies);
  if(proxy) args.unshift('-x', proxy);

  //args.unshift('-v');
  //args.unshift('-sS');
  args.unshift('--tcp-fastopen', '--tcp-nodelay');

  console.log('FetchURL', console.config({ maxArrayLength: Infinity, compact: false }), { args });

  let child = /* child_process.spawn*/ Spawn('curl', args, { block: false, stdio: ['inherit', 'pipe', 'pipe'] });

  let [, out, err] = child.stdio;

  console.log('child', { out, err });

  let output = '',
    errors = '';

  ReadCallback(out, data => {
    output += data;
    // console.log('data',data.length);
  });
  ReadCallback(err, data => {
    errors += data;
    std.err.puts(data);
    std.err.flush();
  });
  let flags = child_process.WNOHANG;
  console.log('flags', flags);
  child.wait(flags);

  let status;

  status = child.wait();

  console.log('FetchURL', { /* output: escape(output), errors,*/ status });

  return output;
}
