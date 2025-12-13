import * as fs from 'fs';
import { ReadFile } from './io-helpers.js';
import * as path from './lib/path.js';

const mapVFSForProxy = new WeakMap();
const mapDirsForFS = new WeakMap();
//const getDirsForFS = memoize(fs => [], mapDirsForFS);
const mapFilenameForHandle = new WeakMap();

export class VirtFS {
  constructor(dirs = []) {
    let obj,
      vfs = this;

    obj = new Proxy(vfs, {
      get(target, prop, receiver) {
        //if(prop == 'dirs')  return   dirs;
        if(prop == 'vfs') return vfs;

        let obj = vfs;

        if(prop in VirtFS.prototype && typeof VirtFS.prototype[prop] == 'function') return VirtFS.prototype[prop].bind(vfs);
        if(!(prop in VirtFS.prototype) && typeof fs[prop] == 'function') return fs[prop];

        return Reflect.get(target, prop, receiver);
      }
    });
    mapDirsForFS.set(vfs, dirs);
    console.log('VirtFS.constructor', { dirs, obj, vfs });
    return obj;
  }

  find(filename, fn = fname => fs.existsSync(fname)) {
    let r,
      f,
      a = mapDirsForFS.get(this.vfs ?? this);
    for(let dir of a) {
      f = path.concat(dir, filename);
      if((r = fn(f))) break;
    }
    return r;
  }

  search(filename, fn = path => path) {
    let found;

    if((found = this.find(filename))) {
      return fn(found);
    }

    throw new Error(`File not found '${filename}'`);
  }

  reduce(fn, acc) {
    let i = 0;
    fn ??= (a, dir, i) => {
      a.push(dir);
      return a;
    };

    for(let dir of mapDirsForFS.get(this.vfs ?? this)) acc = fn(acc, dir, i++);
    return acc;
  }

  filename(handle) {
    return mapFilenameForHandle.get(handle);
  }

  openSync(filename, ...args) {
    return this.find(filename, fname => {
      let handle = fs.openSync(fname, ...args);

      if(handle) mapFilenameForHandle.set(handle, filename);
      return handle;
    });

    console.log('openSync', { a });

    let handle;
    for(let dir of a) {
      let f = path.concat(dir, filename);
      if((handle = fs.openSync(filename, ...args))) {
        mapFilenameForHandle.set(handle, filename);
        break;
      }
    }
    return handle;
  }

  readdirSync(pathname, fn = (fname, fpath) => fname) {
    return [
      ...this.reduce((acc, dir) => {
        let f = path.concat(dir, pathname);
        let x = fs.readdirSync(f) ?? [];
        for(let y of x) acc.add(fn(y, path.normalize(path.concat(f, y))));
        return acc;
      }, new Set())
    ];
  }

  chdir(pathname) {
    return this.search(pathname, found => fs.chdir(found));
  }
  readFileSync(pathname, ...args) {
    return this.search(pathname, found => ReadFile(found, ...args));
  }
  fopen(pathname, ...args) {
    return this.search(pathname, found => fs.fopen(found, ...args));
  }
  existsSync(pathname, ...args) {
    return this.search(pathname, found => fs.existsSync(found, ...args));
  }
  statSync(pathname, ...args) {
    return this.search(pathname, found => fs.statSync(found, ...args));
  }
  lstatSync(pathname, ...args) {
    return this.search(pathname, found => fs.lstatSync(found, ...args));
  }
  readlinkSync(pathname) {
    return this.search(pathname, found => fs.readlinkSync(found));
  }
  realpathSync(pathname) {
    return this.search(pathname, found => fs.realpathSync(found));
  }
  unlinkSync(pathname) {
    return this.search(pathname, found => fs.unlinkSync(found));
  }
}