import { getenv } from 'std';
import * as fs from 'fs';
import * as os from 'os';
import path from './lib/path.js';
import { predicate } from './lib/misc.js';

export function* DirIterator(...args) {
  let pred = typeof args[0] != 'string' ? predicate(args.shift()) : () => true;
  for(let dir of args) {
    dir = dir.replace(/~/g, getenv('HOME'));
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
  dir = dir.replace(/~/g, getenv('HOME'));
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
