import * as os from 'os';
import Util from './lib/util.js';
import * as path from './lib/path.js';
import { types } from 'util';

export function IfDebug(token, loggerFn) {
  const { DEBUG = '' } = process.env;
  const tokList = DEBUG.split(/[^A-Za-z0-9_]+/g);

  if(tokList.indexOf(token) == -1) return () => {};

  return loggerFn;
}

const debug = IfDebug('io-helpers', (...args) => console.log(...args));

export function ReadFile(name, binary) {
  let ret = fs.readFileSync(name, binary ? null : 'utf-8');

  debug(`Read ${name}: ${ret.length} bytes`);
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

export function MapFile(filename) {
  let fd = os.open(filename, os.O_RDONLY);
  let { size } = os.stat(filename)[0];
  debug(`MapFile`, { filename, fd, size });
  let data = mmap.mmap(0, size + 10, mmap.PROT_READ, mmap.MAP_PRIVATE, fd, 0);
  os.close(fd);
  return data;
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

export function WriteFile(name, data, verbose = true) {
  if(Util.isGenerator(data)) {
    let fd = fs.openSync(name, os.O_WRONLY | os.O_TRUNC | os.O_CREAT, 0x1a4);
    let r = 0;
    for(let item of data) {
      r += fs.writeSync(fd, toArrayBuffer(item + ''));
    }
    fs.closeSync(fd);
    let stat = fs.statSync(name);
    return stat?.size;
  }
  if(Util.isIterator(data)) data = [...data];
  if(Util.isArray(data)) data = data.join('\n');

  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';
  let ret = fs.writeFileSync(name, data);

  if(verbose) debug(`Wrote ${name}: ${ret} bytes`);
}

export function WriteJSON(name, data) {
  WriteFile(name, JSON.stringify(data, null, 2));
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
  let pred = typeof args[0] != 'string' ? Util.predicate(args.shift()) : () => true;
  for(let dir of args) {
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
