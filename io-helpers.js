import { spawn } from 'child_process';
import { read as readBJSON, write as writeBJSON } from 'bjson';
import { closeSync, readFileSync, statSync, writeFileSync, readSync } from 'fs';
import { define, error, toString } from './lib/misc.js';

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
  let s = '';

  for(;;) {
    let ret = readSync(fd, ab, 0, ab.byteLength);

    if(ret <= 0) break;

    s += toString(ab.slice(0, ret));

    debug(`Read #${fd}: ${ret} bytes`);
  }

  return s;
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
  return readFileSync(name, binary ? null : 'utf-8');
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

export async function ReadXML(filename, ...args) {
  const { read } = await import('xml');
  let data = ReadAny(filename, false);

  if(data) debug(`ReadXML: ${data.length} bytes read from '${filename}'`);
  return data ? read(data, filename, ...args) : null;
}

export async function WriteXML(name, data, ...args) {
  const { write } = await import('xml');
  return WriteAny(name, write(data, ...args));
}

export function WriteFile(file, data) {
  return writeFileSync(file, data);
  /*let f = fopen(file, 'w+');
  let r = typeof data == 'string' ? f.puts(data) : f.write(data, 0, data.byteLength);

  if(f.error()) throw new Error(`Error writing file '${file}': ${strerror(error().errno)}`);
  f.close();*/

  //console.log('Wrote "' + file + '": ' + data.length + ' bytes' + ` (${r})`);
  return r;
}

export function WriteFd(fd, data, offset, length) {
  if(typeof data == 'string') data = toArrayBuffer(data);

  return writeSync(fd, data, offset ?? 0, length ?? data.byteLength);
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

export function ReadBJSON(filename) {
  let data = readFileSync(filename);
  const { byteLength: size } = data;
  return readBJSON(data, 0, size);
}

export function WriteBJSON(name, value) {
  return writeFileSync(name, writeBJSON(value));
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
    let [stat, err] = statSync(file);
    let obj = define(
      { file, stat },
      {
        toString() {
          return this.file;
        },
      },
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
          },
        });
      }),
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
    ret = typeof fd == 'number' ? readSync(fd, buf, 0, bufferSize) : fd.read(buf, 0, bufferSize);
    if(ret > 0) {
      let data = buf.slice(0, ret);
      yield toString(data);
    }
  } while(ret == bufferSize);
  typeof fd == 'number' ? closeSync(fd) : fd.close();
  return;
}

export function CopyToClipboard(text) {
  return import('child_process').then(child_process => {
    const { env } = process;

    let child = spawn('xclip', ['-in', '-verbose'], {
      env,
      stdio: ['pipe', 'inherit', 'inherit'],
    });
    let [pipe] = child.stdio;

    let written = writeSync(pipe, text, 0, text.length);
    closeSync(pipe);
    let status = child.wait();
    console.log('child', child);
    return { written, status };
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
