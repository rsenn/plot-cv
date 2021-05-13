export const Modulo = (a, b) => ((a % b) + b) % b;

export const WeakMapper = (createFn, map = new WeakMap(), hitFn) => {
  let self = function(obj, ...args) {
    let ret;
    if(map.has(obj)) {
      ret = map.get(obj);
      if(typeof hitFn == 'function') hitFn(obj, ret);
    } else {
      ret = createFn(obj, ...args);
      //if(ret !== undefined)
      map.set(obj, ret);
    }
    return ret;
  };
  self.set = (k, v) => map.set(k, v);
  self.get = k => map.get(k);
  self.map = map;
  return self;
};

export function WeakAssign(...args) {
  let obj = args.shift();
  args.forEach(other => {
    for(let key in other) {
      if(obj[key] === undefined && other[key] !== undefined) obj[key] = other[key];
    }
  });
  return obj;
}

export const GetMethodNames = (obj, depth = 1, start = 0) =>
  Object.getOwnPropertyNames(obj).filter(name => typeof obj[name] == 'function');

export const BindMethods = (obj, methods) => BindMethodsTo({}, obj, methods || obj);

export function BindMethodsTo(dest, obj, methods) {
  if(Array.isArray(methods)) {
    for(let name of methods) if(typeof obj[name] == 'function') dest[name] = obj[name].bind(obj);
    return dest;
  }
  let names = GetMethodNames(methods);

  for(let name of names)
    if(typeof methods[name] == 'function') dest[name] = methods[name].bind(obj);
  return dest;
}

export function FindKey(obj, pred, thisVal) {
  let fn = typeof pred == 'function' ? value : v => v === pred;
  for(let k in obj) if(fn.call(thisVal, obj[k], k)) return k;
}
