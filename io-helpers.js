import { strerror } from 'std';
import { define, toString, escape, error, assert, properties } from './lib/misc.js';
import { O_CREAT, O_RDONLY, O_TRUNC, O_WRONLY, close, open, read, setReadHandler, stat, write } from 'os';
import { SEEK_END, loadFile, fdopen, open as fopen, out as stdout, popen } from 'std';
import { spawn } from 'child_process';

let bjson;

//import('bjson') .then(m => (bjson = m)) .catch(() => {});

let mmap;

//import('mmap') .then(m => (mmap = m)) .catch(() => {});

let xml;

//import('xml') .then(m => (xml = m)) .catch(() => {});

export function IfDebug(token) {
  const { DEBUG = '' } = globalThis.process ? globalThis.process.env : {}; //getenviron();

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
    let ret = read(fd, ab, 0, ab.byteLength);

    if(ret <= 0) break;

    out += toString(ab.slice(0, ret));

    debug(`Read #${fd}: ${ret} bytes`);
  }
  return out;
}

export function IsStdio(obj) {
  return Object.getPrototypeOf(obj) === Object.getPrototypeOf(stdout);
}

export function ReadClose(file, binary) {
  if(IsStdio(file)) {
    if(!binary) return file.readAsString();

    return (function* () {
      while(!file.eof()) {
        let ab = new ArrayBuffer(typeof binary == 'number' ? binary : 1024);
        let r = file.read(ab, 0, ab.byteLength);

        if(r == 0) break;
        if(r < 0 || file.error()) throw new Error(`Error reading file`);

        yield ab.slice(0, r);
      }
    })();
  }
  throw new Error(`Unkown type of file: ${obj}`);
}

export function ReadFile(name, binary) {
  if(!binary || binary == 'utf-8') return loadFile(name);

  let f;

  if((f = fopen(name, 'rb'))) {
    f.seek(0, SEEK_END);
    let size = f.tell();
    let buf = new ArrayBuffer(size);
    let ret = f.read(buf, 0, size);

    console.debug(`Read ${name}: ${ret?.byteLength} bytes`);
    return buf;
  }
}

export function ReadAny(obj, binary) {
  return { number: ReadFd, string: ReadFile, object: ReadClose }[typeof obj](obj, binary);
}

export function LoadHistory(filename) {
  let contents = ReadFile(filename, false);
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
  let data = ReadAny(filename, false);

  if(data) debug(`ReadJSON: ${data.length} bytes read from '${filename}'`);
  return data ? JSON.parse(data) : null;
}

export function ReadXML(filename) {
  let data = ReadAny(filename, false);

  if(data) debug(`ReadXML: ${data.length} bytes read from '${filename}'`);
  return data ? xml.read(data, filename, ...args) : null;
}

export function MapFile(filename) {
  let fd = open(filename, O_RDONLY);
  let { size } = stat(filename)[0];
  debug(`MapFile`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  close(fd);
  return data;
}

export function WriteFile(file, data) {
  let f = fopen(file, 'w+');
  let r = typeof data == 'string' ? f.puts(data) : f.write(data, 0, data.byteLength);

  if(f.error()) throw new Error(`Error writing file '${file}': ${strerror(error().errno)}`);
  f.close();

  console.log('Wrote "' + file + '": ' + data.length + ' bytes' + ` (${r})`);
}

export function WriteFd(fd, data, offset, length) {
  if(typeof data == 'string') data = toArrayBuffer(data);

  return write(fd, data, offset ?? 0, length ?? data.byteLength);
}

export function WriteClose(file, data, offset, length) {
  if(IsStdio(file)) {
    let r;
    r = typeof data == 'string' ? file.puts(data) : file.write(data, offset ?? 0, length ?? data.byteLength);

    if(r <= 0 || file.error()) throw new Error(`Error writing file`);

    file.close();
    return r;
  }
  throw new Error(`Unkown type of file: ${obj}`);
}

export function WriteAny(obj, ...args) {
  return { number: WriteFd, string: WriteFile, object: WriteClose }[typeof obj](obj, ...args);
}

export function WriteJSON(name, data, ...args) {
  const [compact] = args;
  if(typeof compact == 'boolean') args = compact ? [] : [null, 2];

  return WriteAny(name, JSON.stringify(data, ...args));
}

export function WriteXML(name, data, ...args) {
  return WriteAny(name, xml.write(data, ...args));
}

export function ReadBJSON(filename) {
  let fd = open(filename, O_RDONLY);
  let { size } = stat(filename)[0];
  debug(`ReadBJSON`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  debug(`ReadBJSON`, { data });
  let ret = bjson.read(data, 0, size);

  mmap.munmap(data);
  close(fd);
  return ret;
}

export function WriteBJSON(name, data) {
  let buf = bjson.write(data);
  let size = buf.byteLength;
  let fd = open(name, O_WRONLY | O_CREAT | O_TRUNC);

  let ret = write(fd, buf, 0, size);
  debug('WriteBJSON', { name, fd, size, ret });
  close(fd);

  return ret;
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
    let [stat, err] = stat(file);
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

export async function* FdReader(fd, bufferSize = 1024) {
  let buf = new ArrayBuffer(bufferSize);
  let ret;
  do {
    let r = await waitRead(fd);
    console.log('r', r);
    ret = typeof fd == 'number' ? await read(fd, buf, 0, bufferSize) : await fd.read(buf, 0, bufferSize);
    if(ret > 0) {
      let data = buf.slice(0, ret);
      yield toString(data);
    }
  } while(ret == bufferSize);
  typeof fd == 'number' ? await close(fd) : fd.close();
  return;
}

export function CopyToClipboard(text) {
  return import('child_process').then(child_process => {
    const { env } = process;

    let child = spawn('xclip', ['-in', '-verbose'], {
      env,
      stdio: ['pipe', 'inherit', 'inherit']
    });
    let [pipe] = child.stdio;

    let written = write(pipe, text, 0, text.length);
    close(pipe);
    let status = child.wait();
    console.log('child', child);
    return { written, status };
  });
}

export function ReadCallback(fd, fn = data => {}) {
  let buf = new ArrayBuffer(1024);
  setReadHandler(fd, () => {
    let r = read(fd, buf, 0, 1024);
    if(r <= 0) {
      close(fd);
      setReadHandler(fd, null);
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

export function Spawn(...args) {
  const child = spawn(...args);

  console.log('child.stdio', child.stdio);

  //define(child, { get stdin() { return this.stdio[0]; },get stdout() { return this.stdio[1]; },get stderr() { return this.stdio[2]; } });
  define(
    child,
    properties(
      {
        stdin() {
          return this.stdio[0] >= 0 ? fdopen(this.stdio[0], 'w') : null;
        },
        stdout() {
          return this.stdio[1] >= 0 ? fdopen(this.stdio[1], 'r') : null;
        },
        stderr() {
          return this.stdio[2] >= 0 ? fdopen(this.stdio[2], 'r') : null;
        }
      },
      { memoize: true }
    )
  );

  return child;
}
/*export function Spawn(file, args, options = {}) {
  let {
    block = true,
    usePath = true,
    cwd,
    stdio = ['inherit', 'inherit', 'inherit'],
    env,
    uid,
    gid
  } = options;
  let child,
    parent = [...stdio];

  for(let i = 0; i < 3; i++) {
    if(stdio[i] == 'pipe') {
      let [r, w] = pipe();
      stdio[i] = i == 0 ? r : w;
      parent[i] = i == 0 ? w : r;
    } else if(stdio[i] == 'inherit') {
      stdio[i] = i;
    }
  }

  const [stdin, stdout, stderr] = stdio;
  const opts = { block, usePath, cwd, stdin, stdout, stderr, env, uid, gid };
  let pid = exec([file, ...args], opts);
  console.log('exec(' + inspect([file, ...args]) + ', ' + inspect(opts) + ') =', pid);

  for(let i = 0; i < 3; i++) {
    if(typeof stdio[i] == 'number' && stdio[i] != i) close(stdio[i]);
  }

  child = {
    pid,
    stdio: parent,
    get stdin() {
      return this.stdio[0];
    },
    get stdout() {
      return this.stdio[1];
    },
    get stderr() {
      return this.stdio[2];
    },
    get waiting() {
      return this.exited === undefined && this.termsig === undefined && this.stopped === undefined;
    },
    wait() {
      const { waiting, exited, termsig } = child;

      if(!waiting) return [pid, (exited << 8) | termsig];

      assert(exited, undefined, 'exited');
      assert(termsig, undefined, 'signalled');

      const [ret, status] = waitpid(pid, WNOHANG);
      const signal = status & 0x7f;

      if(ret == pid) {
        console.log('waitpid(' + pid + ', WNOHANG) =', [ret, status]);

        if(signal == 0) child.exited = (status >>> 8) & 0xff;
        else if(status & (0xff == 0x7f)) child.stopped = true;
        else child.termsig = signal;
      }

      if(ret < 0) throw new Error(`waitpid: ${strerror(-ret)}`);

      return [ret, status];
    }
  };

  return child;
}*/

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

  let child = spawn('curl', args, {
    block: false,
    stdio: ['inherit', 'pipe', 'pipe']
  });

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
    err.puts(data);
    err.flush();
  });
  let flags = child_process.WNOHANG;
  console.log('flags', flags);
  child.wait(flags);

  let status;

  status = child.wait();

  console.log('FetchURL', { /* output: escape(output), errors,*/ status });

  return output;
}

export function Shell(cmd) {
  let f = popen(cmd, 'r');
  let s = '';
  while(!f.eof() && !f.error()) {
    s += f.readAsString();
  }
  f.close();
  return s;
}

export function ExecTool(cmd, ...args) {
  let f = popen([cmd, ...args].join(' '), 'r');
  let s = '';
  for(;;) {
    let line = f.getline();

    if(line === null) break;
    s += line + '\n';
  }
  f.close();
  return s;
}
