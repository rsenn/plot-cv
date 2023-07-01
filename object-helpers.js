import { define } from './lib/misc.js';

const FWD = 0,
  REV = 1;

export class BiDirMap extends Function {
  set(o1, o2) {
    const { maps } = this;
    maps[FWD].set(o1, o2);
    maps[REV].set(o2, o1);
  }

  delete(obj, dir) {
    const { maps } = this;
    let o2;
    if(dir === FWD || (dir === undefined && (o2 = maps[FWD].get(obj)))) {
      o2 ??= maps[FWD].get(obj);
      maps[FWD].delete(obj);
      maps[REV].delete(o2);
    } else if(dir === REV || (dir === undefined && (o2 = maps[REV].get(obj)))) {
      o2 ??= maps[REV].get(obj);
      maps[REV].delete(obj);
      maps[FWD].delete(o2);
    }
  }

  has(obj, dir) {
    const { maps } = this;
    return dir !== undefined && maps[dir] ? !!maps[dir].get(obj) : maps.some(m => !!m.get(obj));
  }

  index(obj) {
    const { maps } = this;
    let i = 0;
    for(let map of maps) {
      if(map.get(obj)) return i;
      i++;
    }
    return -1;
  }

  /* get(obj, dir) {
    return this(obj, dir);
  }*/

  tuple(obj) {
    const { maps } = this;
    let ret = [null, null];
    let i = this.index(obj);
    if(i != -1) {
      ret[i] = obj;
      ret[i ^ 1] = maps[i].get(obj);
      return ret;
    }
  }

  constructor(a, b) {
    let obj,
      maps = [a ?? new WeakMap(), b ?? new WeakMap()];

    return define(Object.setPrototypeOf(get, BiDirMap.prototype), { maps, get });

    function get(arg, dir) {
      let r;
      if(dir !== undefined && maps[dir]) r = maps[dir].get(arg);
      else for(let map of maps) if((r = map.get(arg))) break;
      return r;
    }
  }
}

define(BiDirMap, { FWD, REV });
define(BiDirMap.prototype, { FWD, REV });

export function ObjectWrapper(factory = () => ({}), isInstance) {
  let proto,
    map = new BiDirMap();
  if(typeof factory != 'function') {
    proto = factory;
    factory = () => Object.create(proto);
  }

  if(proto && isInstance === undefined) isInstance = obj => isObject(obj) && isInstanceOf(obj, proto);

  if(isInstance && typeof isInstance != 'function') {
    proto = isInstance;
    isInstance = obj => isObject(obj) && Object.getPrototypeOf(obj) === proto;
  }

  let fn = wrap;

  if(isInstance) {
    let fn2 = fn;
    fn = (obj, ...args) => (isInstance(obj) ? obj : fn2(obj, ...args));
  }

  return Object.setPrototypeOf(define(fn, { map, proto, factory, wrap }), ObjectWrapper.prototype);

  function wrap(obj, ...args) {
    let target;
    // if(isInstance(obj) || map.has(obj, REV)) return obj;
    if(!(target = map.get(obj, FWD))) {
      target = factory(obj, ...args);
      map.set(obj, target);
    }
    return target;
  }
}

Object.setPrototypeOf(
  ObjectWrapper.prototype,
  Object.getPrototypeOf(function () {})
);

define(ObjectWrapper.prototype, {
  unwrap(obj) {
    const { map } = this;
    return map.get(obj, REV);
  }
});
