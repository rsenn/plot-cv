/* --------------------------- start of 'util.js' --------------------------- */
// ==UserScript==

// @name         libs.es
// @namespace    libs
// @version      0.2
// @description  libs.es
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:3000/element.es
// @grant        none
// @run-at       document-end
// ==/UserScript==

/* jshint esversion: 6 */
/* jshint ignore:start */

/**
 * Class for utility.
 *
 * @class      Util (name)
 */

const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

function Util(g) {
  const globalObject = g || Util.getGlobalObject();
  globalObject.Util = Util;
  return Util;
}

Util.toString = undefined;
//export const Util = {};

const lineSplit = new RegExp('\\n', 'g');

Util.inspectSymbol = inspectSymbol;

Util.formatAnnotatedObject = function(subject, o) {
  const { indent = '  ', spacing = ' ', separator = ',', newline = '\n', maxlen = 30, depth = 1, level = 0 } = o;
  const i = indent.repeat(o.level || 0);
  let nl = newline != '' ? newline + i : spacing;
  const opts = {
    ...o,
    newline: depth >= 0 ? newline : '',
    depth: depth - 1,
    level: level + 1
  };
  if(subject && subject.toSource !== undefined) return subject.toSource();
  if(subject instanceof Date) return 'new Date(' + new Date().toISOString() + ')';
  if(typeof subject == 'string') return "'" + subject + "'";
  if(typeof subject == 'number') return subject;
  if(subject != null && subject.y2 !== undefined) return `rect[${spacing}${subject.x}${separator}${subject.y} | ${subject.x2}${separator}${subject.y2} (${subject.w}x${subject.h}) ]`;
  if(Util.isObject(subject) && 'map' in subject && typeof subject.map == 'function') return `[${nl}${subject.map(i => Util.formatAnnotatedObject(i, opts)).join(separator + nl)}]`;
  if(typeof subject === 'string' || subject instanceof String) return `'${subject}'`;
  let longest = '';
  let r = [];
  for(let k in subject) {
    const v = subject[k];
    if(k.length > longest.length) longest = k;
    let s = '';
    if(typeof v === 'symbol') {
      s = 'Symbol';
    } else if(typeof v === 'string' || v instanceof String) {
      s = `'${v}'`;
    } else if(typeof v === 'function') {
      s = (v + '').replace(lineSplit, '\n' + i);
      s = (Util.fnName(s) || 'function') + '()';
    } else if(typeof v === 'number' || typeof v === 'boolean') {
      s = `${v}`;
    } else if(v === null) {
      s = 'null';
    } else if(v && v.length !== undefined) {
      try {
        s = depth <= 0 ? `Array(${v.length})` : `[ ${v.map(item => Util.formatAnnotatedObject(item, opts)).join(', ')} ]`;
      } catch(err) {
        s = `[${v}]`;
      }
    } else if(v && v.toSource !== undefined) {
      s = v.toSource();
    } else if(opts.depth >= 0) {
      s = s.length > maxlen ? `[Object ${Util.objName(v)}]` : Util.formatAnnotatedObject(v, opts);
    } else {
      let c = Util.className(v);
      let t = Util.ucfirst(typeof v);

      s = `[${t}${c !== t ? ' ' : ''}${c !== t ? c : ''}]`;
    }
    if(s == '') s = typeof v;
    r.push([k, s]);
  }
  let padding = x => indent + (opts.newline != '' ? Util.pad(x, longest.length, spacing) : spacing);
  let j = separator + spacing;
  if(r.length > 6) {
    nl = opts.newline + i;
    j = separator + (opts.newline != '' ? nl : spacing);
  }
  let ret = '{' + opts.newline + r.map(arr => padding(arr[0]) + arr[0] + ':' + spacing + arr[1]).join(j) + opts.newline + i + '}';
  return ret;
};
Util.curry = (fn, arity) => {
  if(arity == null) arity = fn.length;
  let ret = function curried(...args) {
    let thisObj = this;
    if(args.length >= arity) return fn.apply(this, args);

    let n = arity - args.length;
    let a = Array.from({ length: n }, (v, i) => String.fromCharCode(65 + i));
    let Curried = function(...a) {
      return curried.apply(thisObj, a);
    }; //;
    return [
      function() {
        return Curried(...args);
      },
      function(a) {
        return Curried(...args, a);
      },
      function(a, b) {
        return Curried(...args, a, b);
      },
      function(a, b, c) {
        return r(...args, a, b, c);
      },
      function(a, b, c, d) {
        return Curried(...args, a, b, c, d);
      }
    ][n];
    return new Function(...a, `const { curried,thisObj,args} = this; return curried.apply(thisObj, args.concat([${a.join(',')}]))`).bind({ args, thisObj, curried });
  };
  Object.defineProperties(ret, {
    length: {
      value: arity,
      configurable: true,
      writable: true,
      enumerable: false
    },
    orig: {
      get() {
        return fn;
      }
    }
  });
  return ret;
};
Util.arityN = (fn, n) => {
  const arityFn = [
    function(fn) {
      return function() {
        return fn();
      };
    },
    function(fn) {
      return function(a) {
        return fn(a);
      };
    },
    function(fn) {
      return function(a, b) {
        return fn(a, b);
      };
    },
    function(fn) {
      return function(a, b, c) {
        return fn(a, b, c);
      };
    },
    function(fn) {
      return function(a, b, c, d) {
        return fn(a, b, c, d);
      };
    },
    function(fn) {
      return function(a, b, c, d, e) {
        return fn(a, b, c, d, e);
        H;
      };
    }
  ];
  if(n && n <= 5) return arityFn[n](fn);
  return fn;
};

Util.getter = target => {
  let self;
  if(typeof target.get == 'function') self = target.get;
  else
    self = function(key) {
      if(!target) {
        if(this !== self && this) target = this;
        self.target = target;
      }
      let obj = target;
      if(!self.fn) {
        if(typeof obj == 'object' && obj !== null) {
          if(typeof obj.get == 'function') self.fn = key => obj.get(key);
        }
        if(!self.fn) self.fn = key => obj[key];
      }
      return self.fn(key);
    };
  if(target !== undefined) self.target = target;
  return self;
};
Util.setter = target => {
  let self;
  if(typeof target.set == 'function') self = target.set;
  else
    self = function(key, value) {
      if(!target) {
        if(this !== self && this) target = this;
        self.target = target;
      }
      let obj = target;
      if(!self.fn) {
        if(typeof obj == 'object' && obj !== null) {
          if(typeof obj.set == 'function') self.fn = (key, value) => obj.set(key, value);
        }
      }
      if(!self.fn) self.fn = (key, value) => ((obj[key] = value), obj);
      return self.fn(key, value);
    };
  if(target !== undefined) self.target = target;
  return self;
};
Util.remover = target => (typeof target == 'object' && target !== null ? (typeof target.delete == 'function' ? key => target.delete(key) : key => delete target[key]) : null);
Util.hasFn = target => (typeof target == 'object' && target !== null ? (typeof target.has == 'function' ? key => target.has(key) : key => key in target) : null);
Util.adder = target => {
  let self;

  if(target instanceof Set) return arg => target.add(arg);
  if(target instanceof Array) return arg => (target.push(arg), target);

  self = function(obj, arg = 1) {
    if(!target) if (obj) target = obj;

    if(!self.fn) ChooseFn(arg, obj);
    //console.debug('adder', self.fn + '');

    // if(!self.fn) console.log('adder', { target, thisObj: this, fn: self.fn + '', arg });
    return self.fn(obj, arg);
  };
  if(target && !self.fn) {
    ChooseFn(',', target);
    target = null;
  }

  return self;

  function ChooseFn(a, o) {
    if(!self.fn) {
      if(typeof target == 'object' && target !== null) {
        if(typeof target.add == 'function') self.fn = (obj, arg) => (obj.add(arg), undefined);
        else if(typeof target.push == 'function') self.fn = (obj, arg) => (obj.push(arg), undefined);
      }
    }
    let isNum = Util.isNumeric(a);
    //console.debug('ChooseFn', { a, o, f: self.fn });
    if(!self.fn) {
      if(typeof o == 'string') self.fn = (obj, arg) => (obj == '' ? '' : obj + ', ') + arg;
      else if(typeof o == 'number') self.fn = (num, arg) => (typeof num == 'number' ? num : 0) + +arg;
      else if(a) self.fn = (obj, arg) => ((obj || (isNum || typeof arg == 'number' ? 0 : '')) + isNum ? +arg : ',' + arg);
    }
  }
};
Util.updater = (target, get, set, fn) => {
  let value;

  /* prettier-ignore */ get = get || Util.getter(target);
  /* prettier-ignore */ set = set || Util.setter(target);

  return (k, f, i) => doUpdate(k, f || fn, i);
  function doUpdate(key, func, i) {
    value = get.call(target, key);
    let tmp = func(value, i, key);

    if(tmp !== undefined && tmp != value) {
      set.call(target, key, tmp);

      value = get.call(target, key);
    }
    return value;
  }
};
Util.getOrCreate = (target, create = () => ({}), set) => {
  const get = Util.getter(target),
    has = Util.hasFn(target);
  /* prettier-ignore */ set = set || Util.setter(target);
  let value;
  return key => (value = has.call(target, key) ? get.call(target, key) : ((value = create(key, target)), set.call(target, key, value), value));
};
Util.accumulate = (entries, dest = new Map()) => {
  let get = Util.getOrCreate(dest, () => []);
  for(let [key, value] of entries) Util.adder(get(key))(value);
  return dest;
};
Util.memoize = (fn, storage = new Map()) => {
  let self;
  const getter = typeof storage.get == 'function' ? storage.get : typeof storage == 'function' ? storage : Util.getter(storage);
  const setter = typeof storage.set == 'function' ? storage.set : typeof storage == 'function' ? storage : Util.setter(storage);
  self = function(...args) {
    // let n = args[0]; // just taking one argument here
    let cached;
    let key = args[0];

    if((cached = getter.call(storage, key))) {
      //console.log('Fetching from cache');
      return cached;
    }
    let result = fn.call(this, ...args);
    setter.call(storage, key, result);
    return result;
  };
  self.cache = storage;
  return Object.freeze(self);
};
Util.once = (fn, thisArg, memoFn) => {
  let ran = false;
  let ret;

  return function(...args) {
    if(!ran) {
      ran = true;
      ret = fn.call(thisArg || this, ...args);
    } else if(typeof memoFn == 'function') {
      ret = memoFn(ret);
    }
    return ret;
  };
};
Util.delay = (func, wait, thisObj) => {
  if(typeof func != 'function') throw new TypeError(FUNC_ERROR_TEXT);
  return function(...args) {
    setTimeout(function () {
      func.apply(thisObj || this, args);
    }, wait);
  };
};
Util.throttle = (f, t, thisObj) => {
  let lastCall;
  return function(...args) {
    let previousCall = lastCall;
    lastCall = Date.now();
    if(
      previousCall === undefined || // function is being called for the first time
      lastCall - previousCall > t
    )
      // throttle time has elapsed
      f.apply(thisObj || this, args);
  };
};
Util.debounce = (func, wait, options = {}) => {
  if(!Number.isFinite(wait)) throw new TypeError('Expected `wait` to be a finite number');
  let id, args, ctx, timestamp, r;
  const { leading, thisObj } = options;
  if(null == wait) wait = 100;
  function later() {
    let last = Date.now() - timestamp;
    if(last < wait && last >= 0) {
      id = setTimeout(later, wait - last);
    } else {
      id = null;
      if(!leading) {
        r = func.apply(ctx, args);
        ctx = args = null;
      }
    }
  }
  function debounced(...a) {
    ctx = thisObj || this;
    args = a;
    timestamp = Date.now();
    let callNow = leading && !id;
    if(!id) id = setTimeout(later, wait);
    if(callNow) {
      r = func.apply(ctx, args);
      ctx = args = null;
    }
    return r;
  }
  debounced.clear = function() {
    if(id) {
      clearTimeout(id);
      id = null;
    }
  };
  debounced.flush = function() {
    if(id) {
      r = func.apply(ctx, args);
      ctx = args = null;
      clearTimeout(id);
      id = null;
    }
  };
  return debounced;
};

Util.debounceAsync = (fn, wait, options = {}) => {
  if(!Number.isFinite(wait)) throw new TypeError('Expected `wait` to be a finite number');
  let r,
    id,
    resolveList = [];
  const { thisObj, leading } = options;
  return function(...a) {
    return new Promise(resolve => {
      const callNow = leading && !id;
      clearTimeout(id);
      id = setTimeout(() => {
        id = null;
        const result = leading ? r : fn.apply(thisObj || this, a);
        for(resolve of resolveList) resolve(result);
        resolveList = [];
      }, wait);
      if(callNow) {
        r = fn.apply(thisObj || this, a);
        resolve(r);
      } else {
        resolveList.push(resolve);
      }
    });
  };
};

/*Util.debounce = (f, t, thisObj) => {
  let lastCall, lastCallTimer;
  return function(...args) {
    let previousCall = lastCall;
    lastCall = Date.now();
    if(previousCall && lastCall - previousCall <= t) clearTimeout(lastCallTimer);

    return new Promise((resolve, reject) => {
      lastCallTimer = setTimeout(() => resolve(f.apply(thisObj || this, args)), t);
    });
  };
};*/
Util.getGlobalObject = Util.memoize(arg => {
  const retfn = typeof arg == 'function' ? arg : typeof arg == 'string' ? g => g[arg] : g => g;

  return Util.tryCatch(
    () => globalThis,
    retfn,
    err =>
      Util.tryCatch(
        () => globalThis,
        retfn,
        err =>
          Util.tryCatch(
            () => window,
            retfn,
            err => console.log('Util.getGlobalObject:', err)
          )
      )
  );
});

Util.isDebug = Util.memoize(() => {
  if(process !== undefined && process.env.NODE_ENV === 'production') return false;
  return true;
});

/*Util.log = Util.curry(function(n, base) {
  return Math.log(n) / (base ? Math.log(base) : 1);
});*/
Util.log = (...args) => {
  let location;
  if(args[0] instanceof Util.location) location = args.shift();
  else {
    let stack = Util.getStackFrames(2);
    if(/\/util\.js$/.test(stack[0].fileName)) stack = stack.slice(1);
    location = stack[0].getLocation();
  }
  let locationStr = location.toString(true);
  let c = [(locationStr[inspectSymbol] || locationStr.toString).call(locationStr)];
  c.push(' ');
  let filters = Util.log.filters;
  let results = filters.map(f => f.test(locationStr));
  if(filters.every(f => !f.test(locationStr))) return;
  console.log('log', { args, c });
  Util.putStack();
  args = args.reduce((a, p, i) => {
    if(Util.isObject(p) && p[Util.log.methodName]) p = p[Util.log.methodName]();
    else if(Util.isObject(p) && p[inspectSymbol]) p = p[inspectSymbol]();
    else if(typeof p != 'string') {
      if(Util.isObject(p) && typeof p.toString == 'function' && !Util.isNativeFunction(p.toString)) p = p.toString();
      else p = Util.inspect(p, { multiline: false });
    }

    //  if(i > 0) a.push(',');
    a.push(p);
    //    a.append([p]);
    return a;
  }, c);
  if(args.toConsole) args.toConsole();
  else console.log(...args);
};

Object.defineProperty(Util.log, 'methodName', {
  get: () => (Util.isBrowser() ? 'toConsole' : 'toAnsi256')
});

Util.log.filters = [/.*/];
Util.log.setFilters = function(args) {
  this.filters = [...args].map(arg => (arg instanceof RegExp ? arg : new RegExp(arg)));
};
Util.log.getFilters = function() {
  return this.filters;
};

Util.msg = (strings, ...substitutions) => {
  let i,
    o = [];
  for(i = 0; i < Math.max(strings.length, substitutions.length); i++) {
    if(strings[i] !== undefined) o.push(strings[i].trim());
    if(substitutions[i] !== undefined) o.push(substitutions[i]);
  }
  console.log(...o);
};

Util.logBase = Util.curry((base, n) => Math.log(n) / Math.log(base));

Util.generalLog = function(n, x) {
  return Math.log(x) / Math.log(n);
};
Util.toSource = function(arg, opts = {}) {
  const { quote = "'", colors = false, multiline = false, json = false } = opts;
  const { c = Util.coloring(colors) } = opts;
  let o = [];
  const { print = (...args) => (o = c.concat(o, c.text(...args))) } = opts;
  if(Util.isArray(arg)) {
    print('[', 1, 36);
    for(let item of arg) {
      if(o.length > 0) print(', ');
      Util.toSource(item, { ...opts, c, print });
    }
    print(']', 1, 36);
  } else if(typeof arg == 'number' || arg === undefined || arg === null) print(arg, 1, 35);
  else if(typeof arg == 'string') print(`${quote}${arg}${quote}`, 1, 36);
  else if(arg && arg.x !== undefined && arg.y !== undefined) {
    print('[', 1, 36);
    print(arg.x, 1, 32);
    print(',', 1, 36);
    print(arg.y, 1, 32);
    print(']', 1, 36);
  } else if(typeof arg == 'object') {
    let i = 0;
    let m = arg instanceof Map;
    if(m) {
      print('new ', 1, 31);
      print('Map', 1, 33);
    }
    print((m ? '([[' : '{') + (multiline ? '\n  ' : ' '), 1, 36);
    for(const [prop, value] of Util.entries(arg)) {
      if(i > 0) {
        let s = multiline ? ',\n  ' : ', ';
        if(m) s = ' ]' + s + '[ ';
        print(s, 1, 36);
      }
      if(!m) print(json ? `"${prop}"` : prop, 1, 33);
      else Util.toSource(prop, { ...opts, c, print });
      print(m ? ', ' : ': ', 1, 36);
      Util.toSource(value, { ...opts, c, print });
      i++;
    }
    print(multiline ? '\n' : ' ' + (m ? ']])' : '}'), 1, 36);
  }
  return o;
};
Util.debug = function(message) {
  const args = [...arguments];
  let cache = [];
  const removeCircular = function(key, value) {
    if(typeof value === 'object' && value !== null) {
      if(cache.indexOf(value) !== -1) return;
      cache.push(value);
    }
    return value;
  };
  const str = args
    .map(arg => (typeof arg === 'object' ? JSON.toString(arg, removeCircular) : arg))
    .join(' ')
    .replace(lineSplit, '');
  //console.log("STR: "+str);
  //console.log.call(console, str);
  //Util.log.apply(Util, args)
};
Util.type = function({ type }) {
  return (type && String(type).split(new RegExp('[ ()]', 'g'))[1]) || '';
};
Util.functionName = function(fn) {
  if(typeof fn == 'function' && typeof fn.name == 'string') return fn.name;
  try {
    const matches = /function\s*([^(]*)\(.*/g.exec(fn + '');
    if(matches && matches[1]) return matches[1];
  } catch {}
  return null;
};
Util.className = function(obj) {
  let proto;
  //console.log("class:", obj);
  try {
    proto = Object.getPrototypeOf(obj);
  } catch(err) {
    try {
      proto = obj.prototype;
    } catch(err) {}
  }
  if(Util.isObject(proto) && 'constructor' in proto) return Util.functionName(proto.constructor);
};
Util.unwrapComponent = function(c) {
  for(;;) {
    if(c.wrappedComponent) c = c.wrappedComponent;
    else if(c.WrappedComponent) c = c.WrappedComponent;
    else break;
  }
  return c;
};
Util.componentName = function(c) {
  for(;;) {
    if(c.displayName || c.name) {
      return (c.displayName || c.name).replace(/.*\(([A-Za-z0-9_]+).*/, '$1');
    } else if(c.wrappedComponent) c = c.wrappedComponent;
    else if(c.WrappedComponent) c = c.WrappedComponent;
    else break;
  }
  return Util.fnName(c);
};
Util.count = function(s, ch) {
  return (String(s).match(new RegExp(ch, 'g')) || Util.array()).length;
};
Util.parseNum = function(str) {
  let num = parseFloat(str);
  if(isNaN(num)) num = 0;
  return num;
};
Util.minmax = function(num, min, max) {
  return Math.min(Math.max(num, min), max);
};
Util.getExponential = function(num) {
  let str = typeof num == 'string' ? num : num.toExponential();
  const matches = /e\+?(.*)$/.exec(str);
  //console.log("matches: ", matches);
  return parseInt(matches[1]);
};
Util.getNumberParts = function(num) {
  let str = typeof num == 'string' ? num : num.toExponential();
  const matches = /^(-?)(.*)e\+?(.*)$/.exec(str);
  //console.log("matches: ", matches);
  const negative = matches[1] == '-';
  return {
    negative,
    mantissa: parseFloat(matches[2]),
    exponent: parseInt(matches[3])
  };
};
Util.pow2 = function(n) {
  return Math.pow(2, n);
};
Util.pow10 = function(n) {
  return n >= 0 ? Math.pow(10, n) : 1 / Math.pow(10, -n);
};
Util.bitValue = function(n) {
  return Util.pow2(n - 1);
};
Util.bitMask = function(bits, start = 0) {
  let r = 0;
  let b = 1 << start;

  for(let i = 0; i < bits; i++) {
    r |= b;
    b <<= 1;
  }
  return r;
};

Util.bitGroups = function(num, bpp, minLen) {
  let m = Util.bitMask(bpp, 0);
  let n = Math.floor(64 / bpp);
  let r = [];
  for(let i = 0; i < n; i++) {
    r.push(num & m);
    num /= m + 1;
  }
  while(r.length > 0 && r[r.length - 1] == 0 /* && Util.mod(r.length *bpp, 8) > 0*/) r.pop();
  while(r.length < minLen) r.push(0);
  return r;
};

Util.bitStuff = (arr, bpp) => {
  const m = Util.bitMask(bpp, 0);
  return arr.reduce(([b, f], n) => [b + (n & m) * f, f * (m + 1)], [0, 1])[0];
};

Util.toBinary = function(num) {
  return parseInt(num).toString(2);
};
Util.toBits = function(num) {
  let a = Util.toBinary(num).split('').reverse();
  return Array.from(Object.assign({}, a, { length: 50 }), bit => (bit ? 1 : 0));
};
Util.getBit = function(v, n) {
  let s = v.toString(2);
  return n < s.length ? parseInt(s[s.length - n - 1]) : 0;
};
Util.isSet = function(v, n) {
  return Util.getBit(v, n) == 1;
};
Util.bitCount = function(n) {
  return Util.count(Util.toBinary(n), '1');
};
Util.bitNo = function(n) {
  for(let i = 0; n; i++) {
    if(n & 1) return i;
    n >>= 1;
  }
};

Util.toggleBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n - Util.pow2(bit) : n + Util.pow2(bit);
};
Util.setBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n : n + Util.pow2(bit);
};
Util.clearBit = function(num, bit) {
  const n = Number(num);
  return Util.isSet(n, bit) ? n - Util.pow2(bit) : n;
};
Util.range = function(...args) {
  let [start, end, step = 1] = args;
  let ret;
  start /= step;
  end /= step;
  if(start > end) {
    ret = [];
    while(start >= end) ret.push(start--);
  } else {
    ret = Array.from({ length: end - start + 1 }, (v, k) => k + start);
  }
  if(step != 1) {
    ret = ret.map(n => n * step);
  }
  //console.log("Util.range ", r);
  return ret;
};
Util.set = function(obj, prop, value) {
  const set = obj instanceof Map ? (prop, value) => obj.set(prop, value) : (prop, value) => (obj[prop] = value);
  if(arguments.length == 1)
    return (prop, value) => {
      set(prop, value);
      return set;
    };
  if(arguments.length == 2) return value => set(prop, value);
  return set(prop, value);
};

Util.get = Util.curry((obj, prop) => (obj instanceof Map ? obj.get(prop) : obj[prop]));
Util.symbols = (() => {
  const { asyncIterator, hasInstance, isConcatSpreadable, iterator, match, matchAll, replace, search, species, split, toPrimitive, toStringTag, unscopables } = Symbol;
  return {
    inspect: inspectSymbol,
    asyncIterator,
    hasInstance,
    isConcatSpreadable,
    iterator,
    match,
    matchAll,
    replace,
    search,
    species,
    split,
    toPrimitive,
    toStringTag,
    unscopables
  };
})();

/*
  const { indent = '  ', newline = '\n', depth = 2, spacing = ' ' } = typeof opts == 'object' ? opts : { indent: '', newline: '', depth: typeof opts == 'number' ? opts : 10, spacing: ' ' };

  return Util.formatAnnotatedObject(obj, { indent, newline, depth, spacing });
};*/
Util.bitArrayToNumbers = function(arr) {
  let numbers = [];
  for(let i = 0; i < arr.length; i++) {
    const number = i + 1;
    if(arr[i]) numbers.push(number);
  }
  return numbers;
};
Util.bitsToNumbers = function(bits) {
  let a = Util.toBinary(bits).split('');
  let r = [];
  //return a;
  a.forEach((val, key, arr) => val == '1' && r.unshift(a.length - key));
  return r;
};
Util.shuffle = function(arr, rnd = Util.rng) {
  arr.sort((a, b) => 0.5 - rnd());
  return arr;
};
Util.sortNum = function(arr) {
  arr.sort((a, b) => a - b);
  //console.log("Util.sortNum ", { arr });
  return arr;
};
Util.draw = (arr, n = 1, rnd = Util.rng) => {
  let pos = Util.randInt(0, arr.length - n - 1, rnd);
  const r = arr.splice(pos, n);
  return n == 1 ? r[0] : r;
};
Util.is = function(what, ...pred) {
  let fnlist = pred.map(type => (Util.isConstructor(type) ? what instanceof type : this.is[type]));
  //console.debug('fnlist:', fnlist);
  return fnlist.every(fn => fn(what));
};

Util.instanceOf = (value, ctor) => Util.isObject(value) && Util.isConstructor(ctor) && value instanceof ctor;

Util.onoff = function(val) {
  if(Util.is.on(val)) return true;
  if(Util.is.off(val)) return false;
  return undefined;
};
Util.numbersToBits = function(arr) {
  return arr.reduce((bits, num) => bits + Util.bitValue(num), 0);
};
Util.randomNumbers = function([start, end], draws) {
  const r = Util.draw(Util.range(start, end), draws);
  //console.log("Util.randomNumbers ", { start, end, draws, r });
  return r;
};
Util.randomBits = function(r = [1, 50], n = 5) {
  return Util.numbersToBits(Util.randomNumbers(r, n));
};
Util.padFn = function(len, char = ' ', fn = (str, pad) => pad) {
  return (s, n = len) => {
    let m = Util.stripAnsi(s).length;
    s = s ? s.toString() : '' + s;
    return fn(s, m < n ? char.repeat(n - m) : '');
  };
};
Util.pad = function(s, n, char = ' ') {
  return Util.padFn(n, char)(s);
};
Util.abbreviate = function(str, max = 40, suffix = '...') {
  max = +max;
  if(isNaN(max)) max = Infinity;
  if(Util.isArray(str)) {
    return Array.prototype.slice.call(str, 0, Math.min(str.length, max)).concat([suffix]);
  }
  if(typeof str != 'string' || !Number.isFinite(max) || max < 0) return str;
  str = '' + str;
  if(str.length > max) {
    return str.substring(0, max - suffix.length) + suffix;
  }
  return str;
};
Util.trim = function(str, charset) {
  const r1 = RegExp('^[' + charset + ']*');
  const r2 = RegExp('[' + charset + ']*$');
  return str.replace(r1, '').replace(r2, '');
};
Util.trimRight = function(str, charset) {
  const r2 = RegExp('[' + charset + ']*$');
  return str.replace(r2, '');
};
Util.indent = (text, space = '  ') => {
  text = text.trim();
  if(!/\n/.test(text)) return text;
  return text.replace(/(\n)/g, '\n' + space) + '\n';
};
Util.define = (obj, ...args) => {
  if(typeof args[0] == 'object') {
    const [arg, overwrite = true] = args;
    let adecl = Object.getOwnPropertyDescriptors(arg);
    let odecl = {};
    for(let prop in adecl) {
      if(prop in obj) {
        if(!overwrite) continue;
        else delete obj[prop];
      }
      if(Object.getOwnPropertyDescriptor(obj, prop)) delete odecl[prop];
      else
        odecl[prop] = {
          ...adecl[prop],
          enumerable: false,
          configurable: true,
          writeable: true
        };
    }
    Object.defineProperties(obj, odecl);
    return obj;
  }
  const [key, value, enumerable = false] = args;
  Object.defineProperty(obj, key, {
    enumerable,
    configurable: true,
    writable: true,
    value
  });
  return obj;
};
Util.memoizedProperties = (obj, methods) => {
  let decls = {};
  for(let method in methods) {
    const memoize = Util.memoize(methods[method]);
    decls[method] = {
      get() {
        return memoize.call(this);
      },
      enumerable: true,
      configurable: true
    };
  }
  return Object.defineProperties(obj, decls);
};
Util.copyWhole = (dst, ...args) => {
  let chain = [];
  for(let src of args) chain = chain.concat(Util.getPrototypeChain(src).reverse());
  //console.debug('chain:', ...chain);
  for(let obj of chain) Util.define(dst, obj);
  return dst;
};
Util.copyEntries = (obj, entries) => {
  for(let [k, v] of entries) obj[k] = v;
  return obj;
};

Util.extend = (...args) => {
  let deep = false;
  if(typeof args[0] == 'boolean') deep = args.shift();

  let result = args[0];
  if(Util.isUnextendable(result)) throw new Error('extendee must be an object');
  let extenders = args.slice(1);
  let len = extenders.length;
  for(let i = 0; i < len; i++) {
    let extender = extenders[i];
    for(let key in extender) {
      if(true || extender.hasOwnProperty(key)) {
        let value = extender[key];
        if(deep && Util.isCloneable(value)) {
          let base = Array.isArray(value) ? [] : {};
          result[key] = Util.extend(true, result.hasOwnProperty(key) && !Util.isUnextendable(result[key]) ? result[key] : base, value);
        } else {
          result[key] = value;
        }
      }
    }
  }
  return result;
};

Util.isCloneable = obj => Array.isArray(obj) || {}.toString.call(obj) == '[object Object]';

Util.isUnextendable = val => !val || (typeof val != 'object' && typeof val != 'function');

/*
Util.extend = (obj, ...args) => {
  for(let other of args) {
    for(let key of Util.iterateMembers(other, (k, value) => obj[k] === undefined && [k, value])) {
      const value = other[key];
      try {
        Object.defineProperty(obj, key, {
          value,
          enumerable: false,
          configurable: false,
          writable: false
        });
      } catch(err) {
        console.log('extend:' + err + '\n', { obj, key, value });
      }
    }
  }
  return obj;
};*/

Util.static = (obj, functions, thisObj, pred = (k, v, f) => true) => {
  for(let [name, fn] of Util.iterateMembers(
    functions,

    Util.tryPredicate((key, depth) => obj[key] === undefined && typeof functions[key] == 'function' && pred(key, depth, functions) && [key, value])
  )) {
    const value = function(...args) {
      return fn.call(thisObj || obj, this, ...args);
    };
    try {
      obj[name] = value;

      /*        Object.defineProperty(obj, name, { value, enumerable: false, configurable: false, writable: false });*/
    } catch(err) {
      console.log('static:', err);
    }
  }
  return obj;
};
Util.defineGetter = (obj, key, fn, enumerable = false) =>
  obj[key] === undefined &&
  Object.defineProperty(obj, key, {
    enumerable,
    configurable: true,
    get: fn
  });
Util.defineGetterSetter = (obj, key, g, s, enumerable = false) =>
  obj[key] === undefined &&
  Object.defineProperty(obj, key, {
    get: g,
    set: s,
    enumerable
  });
Util.defineGettersSetters = (obj, gettersSetters) => {
  for(let name in gettersSetters) Util.defineGetterSetter(obj, name, gettersSetters[name], gettersSetters[name]);
};

Util.extendArray = function(arr = Array.prototype) {
  /*  Util.define(arr, 'tail', function() {
    return this[this.length - 1];
  });*/
  Util.define(arr, 'match', function(pred) {
    return Util.match(this, pred);
  });
  Util.define(arr, 'clear', function() {
    this.splice(0, this, length);
    return this;
  });
  Util.define(arr, 'unique', function() {
    return this.filter((item, i, a) => a.indexOf(item) == i);
  });
  Util.defineGetterSetter(
    arr,
    'tail',
    function() {
      return Util.tail(this);
    },
    function(value) {
      if(this.length == 0) this.push(value);
      else this[this.length - 1] = value;
    }
  );

  /*Util.define(arr, 'inspect', function(opts = {}) {
    return Util.inspect(this, { depth: 100, ...opts });
  });*/
};
Util.adapter = function(obj, getLength = obj => obj.length, getKey = (obj, index) => obj.key(index), getItem = (obj, key) => obj[key], setItem = (obj, index, value) => (obj[index] = value)) {
  const adapter = obj && {
    /* prettier-ignore */ get length() {
      return getLength(obj);
    },
    /* prettier-ignore */ get instance() {
      return obj;
    },
    key(i) {
      return getKey(obj, i);
    },
    get(key) {
      return getItem(obj, key);
    },
    has(key) {
      return this.get(key) !== undefined;
    },
    set(key, value) {
      return setItem(obj, key, value);
    },
    *keys() {
      const length = getLength(obj);
      for(let i = 0; i < length; i++) yield getKey(obj, i);
    },
    *entries() {
      for(let key of this.keys()) yield [key, getItem(obj, key)];
    },
    [Symbol.iterator]() {
      return this.entries();
    },
    toObject() {
      return Object.fromEntries(this.entries());
    },
    toMap() {
      return new Map(this.entries());
    }
  };
  return adapter;
};
Util.adapter.localStorage = function(s) {
  s = Util.tryCatch(
    () => !s && globalThis.window,
    w => w.localStorage,
    () => s
  );

  return Util.adapter(
    s,
    l => l.length,
    (l, i) => l.key(i),
    (l, key) => JSON.parse(l.getItem(key)),
    (l, key, v) => l.setItem(key, JSON.toString(v))
  );
};
let doExtendArray = Util.extendArray;
Util.array = function(a) {
  if(!(a instanceof Array)) {
    if(Util.isObject(a) && 'length' in a) a = Array.from(a);
  }
  if(doExtendArray)
    try {
      /*  if(Array.prototype.match === undefined) doExtendArray(Array.prototype);*/
      if(a.match === undefined) {
        doExtendArray(Array.prototype);
        if(a.match) doExtendArray = null;
      }
      if(a.match === undefined) doExtendArray(a);
    } catch(err) {}
  return a;
};
Util.arrayFromEntries = entries =>
  Array.from(
    entries.map(([k, v]) => k),
    key => entries.find(([k, v]) => k === key)[1]
  );

Util.toMap = function(hash = {}, fn) {
  let m, gen;
  if(hash instanceof Array && typeof fn == 'function') hash = hash.map(fn);

  if(hash[Symbol.iterator] !== undefined) gen = hash[Symbol.iterator]();
  else if(Util.isGenerator(hash)) gen = hash;
  else gen = Object.entries(hash);

  m = new Map(gen);

  /*
  if(m instanceof Array) m[Symbol.iterator] = m.entries;*/
  try {
    //if(m.toObject === undefined) Util.extendMap();
    if(Map.prototype.toObject === undefined) Util.extendMap(Map.prototype);
  } catch(err) {}
  return m;
};
Util.extendMap = function(map) {
  if(map.entries === undefined) {
    map.entries = function* iterator() {
      for(let entry of map) {
        yield entry.name !== undefined && entry.value !== undefined ? [entry.name, entry.value] : entry[0] !== undefined && entry[1] !== undefined ? entry : [entry, map[entry]];
      }
    };
  }
  map.toObject = function() {
    return Object.fromEntries(this.entries());
  };
  map.match = function(...args) {
    return Util.match.apply(this, args);
  };
};
Util.fromEntries = Object.fromEntries
  ? Object.fromEntries
  : entries => {
      let ret = {};
      for(let [k, v] of entries) {
        ret[k] = v;
      }
      return ret;
    };

Util.objectFrom = function(any) {
  if('toJS' in any) any = any.toJS();
  else if(Util.isArray(any)) return Util.fromEntries(any);
  else if('entries' in any) return Util.fromEntries(any.entries());
  return Object.assign({}, any);
};
Util.tail = function(arr) {
  return arr && arr.length > 0 ? arr[arr.length - 1] : null;
};
Util.splice = function(str, index, delcount, insert) {
  const chars = str.split('');
  Array.prototype.splice.apply(chars, arguments);
  return chars.join('');
};
Util.identity = arg => arg;
Util.reverse = arr => arr.reverse();

Util.keyOf = function(obj, prop) {
  const keys = Object.keys(obj);
  for(let k in keys) {
    if(obj[k] === prop) return k;
  }
  return undefined;
};
Util.rotateRight = function(arr, n) {
  arr.unshift(...arr.splice(n, arr.length));
  return arr;
};
Util.repeater = function(n, what) {
  if(typeof what == 'function')
    return (function* () {
      for(let i = 0; i < n; i++) yield what();
    })();
  return (function* () {
    for(let i = 0; i < n; i++) yield what;
  })();
};
Util.repeat = function(n, what) {
  return [...Util.repeater(n, what)];
};
Util.arrayDim = function(dimensions, init) {
  let args = [...dimensions];
  args.reverse();
  let ret = init;
  while(args.length > 0) {
    const n = args.shift();
    ret = Util.repeat(n, ret);
  }
  return ret;
};
Util.flatten = function(arr) {
  let ret = [];
  for(let i = 0; i < arr.length; i++) {
    ret = [...ret, ...arr[i]];
  }
  return ret;
};
Util.chunkArray = (a, size) =>
  a.reduce((acc, item, i) => {
    const idx = i % size;
    if(idx == 0) acc.push([]);

    acc[acc.length - 1].push(item);
    return acc;
  }, []);

Util.partition = function* (a, size) {
  for(let i = 0; i < a.length; i += size) yield a.slice(i, i + size);
};

Util.intersect = (a, b) => a.filter(Set.prototype.has, new Set(b));
Util.difference = (a, b, incicludes) => {
  //console.log('Util.difference', { a, b, includes });
  if(typeof includes != 'function') return [a.filter(x => !b.includes(x)), b.filter(x => !a.includes(x))];

  return [a.filter(x => !includes(b, x)), b.filter(x => !includes(a, x))];
};
Util.symmetricDifference = (a, b) => [].concat(...Util.difference(a, b));
Util.union = (a, b, equality) => {
  if(equality === undefined) return [...new Set([...a, ...b])];

  return Util.unique([...a, ...b], equality);
};

Util.chances = function(numbers, matches) {
  const f = Util.factorial;
  return f(numbers) / (f(matches) * f(numbers - matches));
};
Util.sum = function(arr) {
  return arr.reduce((acc, n) => acc + n, 0);
};

Util.expr = fn => {
  let nargs = fn.length;
  let ret = Util.curry(fn);

  return ret;
  return expr;
  function expr(...args) {
    let nums = [];

    function addArgs(args) {
      while(args.length > 0) {
        const arg = args.shift();

        if(typeof arg == 'function') args.unshift(arg(...args.splice(0, arg.length)));
        else if(typeof arg == 'number') nums.push(arg);
      }
    }
    addArgs(args);
    //console.debug('nargs:', nargs);
    //console.debug('nums.length:', nums.length);
    if(nums.length >= nargs) return fn(...nums);

    //let args = ['a','b','c','d'].slice(0,nargs - nums.length);
    let ret = function returnFn(...args) {
      addArgs(args.slice(0, nargs - nums.length));

      //console.log('nums.length:', nums.length);
      if(nums.length >= nargs) return fn(...nums);
      return returnFn;
    };
    ret.nums = nums;

    return ret;
  }
};

Util.add = Util.curry((a, b) => a + b);
Util.sub = Util.curry((a, b) => a - b);
Util.mul = Util.curry((a, b) => a * b);
Util.div = Util.curry((a, b) => a / b);
Util.xor = Util.curry((a, b) => a ^ b);
Util.or = Util.curry((a, b) => a | b);
Util.and = Util.curry((a, b) => a & b);
Util.mod = (a, b) => (typeof b == 'number' ? ((a % b) + b) % b : n => ((n % a) + a) % a);
Util.pow = Util.curry((a, b) => Math.pow(a, b));

/*Util.define(String.prototype,
  'splice',
  function(index, delcount, insert) {
    return Util.splice.apply(this, [this, ...arguments]);
  }
);*/
Util.fnName = function(f, parent) {
  if(typeof f == 'function') {
    if(f.name !== undefined) return f.name;
    const s = typeof f.toSource == 'function' ? f.toSource() : f + '';
    const matches = /([A-Za-z_][0-9A-Za-z_]*)\w*[(\]]/.exec(s);
    if(matches) return matches[1];
    if(parent !== undefined) {
      for(let key in parent) {
        if(parent[key] === f) return key;
      }
    }
  }
};

Util.objName = function(o) {
  if(o === undefined || o == null) return `${o}`;
  if(typeof o === 'function' || o instanceof Function) return Util.fnName(o);
  if(o.constructor) return Util.fnName(o.constructor);
  const s = `${o.type}`;
  return s;
};
Util.findKey = function(obj, pred, thisVal) {
  let fn = typeof pred == 'function' ? value : v => v === pred;
  for(let k in obj) if(fn.call(thisVal, obj[k], k)) return k;
};
Util.find = function(arr, value, prop = 'id') {
  let pred;
  if(typeof value == 'function') {
    pred = value;
  } else if(prop && prop.length !== undefined) {
    pred = function(obj) {
      if(obj[prop] == value) return true;
      return false;
    };
  } else {
    pred = typeof prop == 'function' ? obj => prop(value, obj) : obj => obj[prop] == value;
  }
  if(typeof arr.find == 'function') return arr.find(pred);
  if(!arr[Symbol.iterator] && typeof arr.entries == 'function') {
    let entryPred = pred;
    pred = ([key, value], arr) => entryPred(value, key, arr);
    arr = arr.entries();
  }
  for(let v of arr) {
    if(pred(v)) return v;
  }
  return null;
};

Util.findIndex = function(obj, pred, thisArg) {
  if(typeof obj.findIndex == 'function') return obj.findIndex(pred, thisArg);
  return Util.findKey(obj, pred, thisArg);
};

Util.match = function(arg, pred) {
  let match = pred;
  if(pred instanceof RegExp) {
    const re = pred;
    match = (val, key) => (val && val.tagName !== undefined && re.test(val.tagName)) || (typeof key === 'string' && re.test(key)) || (typeof val === 'string' && re.test(val));
  }
  if(Util.isArray(arg)) {
    if(!(arg instanceof Array)) arg = [...arg];
    return arg.reduce((acc, val, key) => {
      if(match(val, key, arg)) acc.push(val);
      return acc;
    }, []);
  } else if(Util.isMap(arg)) {
    //console.log('Util.match ', { arg });
    return [...arg.keys()].reduce((acc, key) => (match(arg.get(key), key, arg) ? acc.set(key, arg.get(key)) : acc), new Map());
  }
  return Util.filter(arg, match);
};
Util.toHash = function(map, keyTransform = k => Util.camelize('' + k)) {
  let ret = {};
  Util.foreach(map, (v, k) => (ret[keyTransform(k)] = v));
  return ret;
};
Util.indexOf = function(obj, prop) {
  for(let key in obj) {
    if(obj[key] === prop) return key;
  }
  return undefined;
};

/*
Util.injectProps = function(options) {
  return function(InitialComponent) {
    return function DndStateInjector() {
      return <InitialComponent {...options} />;
    }
  }
}*/

Util.greatestCommonDenominator = (a, b) => (b ? Util.greatestCommonDenominator(b, a % b) : a);

Util.leastCommonMultiple = (n1, n2) => {
  //Find the gcd first
  let gcd = Util.greatestCommonDenominator(n1, n2);

  //then calculate the lcm
  return (n1 * n2) / gcd;
};
Util.matchAll = Util.curry(function* (re, str) {
  let match;
  re = re instanceof RegExp ? re : new RegExp(Util.isArray(re) ? '(' + re.join('|') + ')' : re, 'g');
  do {
    if((match = re.exec(str))) yield match;
  } while(match != null);
});

Util.inspect = function(obj, opts = {}) {
  const {
    quote = '"',
    multiline = true,
    toString = Symbol.toStringTag || 'toString' /*Util.symbols.toStringTag*/,
    stringFn = str => str,
    indent = '',
    colors = false,
    stringColor = [1, 36],
    spacing = '',
    newline = '\n',
    padding = ' ',
    separator = ',',
    colon = ': ',
    depth = 10,
    json = false
  } = {
    ...Util.inspect.defaultOpts,
    toString: Util.symbols.inspect,
    colors: true,
    multiline: true,
    newline: '\n',
    ...opts
  };

  try {
    if(Util == obj) return Util;
  } catch(e) {}
  //console.log("Util.inspect", {quote,colors,multiline,json})

  let out;
  const { c = Util.coloring(colors) } = opts;
  const { print = (...args) => (out = c.concat(out, c.text(...args))) } = opts;
  const sep = multiline && depth > 0 ? (space = false) => newline + indent + (space ? '  ' : '') : (space = false) => (space ? spacing : '');
  if(typeof obj == 'number') {
    print(obj + '', 1, 36);
  } else if(typeof obj == 'undefined' || obj === null) {
    print(obj + '', 1, 35);
  } else if(typeof obj == 'function' /*|| obj instanceof Function || Util.className(obj) == 'Function'*/) {
    obj = '' + obj;
    //  if(!multiline)
    obj = obj.split(lineSplit)[0].replace(/{\s*$/, '{}');
    print(obj);
  } else if(typeof obj == 'string') {
    print(`${quote}${stringFn(obj)}${quote}`, 1, 36);
  } else if(obj instanceof Date) {
    print(`new `, 1, 31);

    print(`Date`, 1, 33);
    print(`(`, 1, 36);
    print(obj.getTime() + obj.getMilliseconds() / 1000, 1, 36);
    print(`)`, 1, 36);
  } else if(Object.getPrototypeOf(obj) == Array.prototype) {
    let i;
    print(`[`, 1, 36);
    for(i = 0; i < obj.length; i++) {
      if(i > 0) print(separator, 1, 36);
      else print(padding);
      print(sep(i > 0));
      Util.inspect(obj[i], {
        ...opts,
        c,
        print,
        newline: newline + '  ',
        depth: depth - 1
      });
    }
    print((padding || '') + `]`, 1, 36);
  } else if(Util.isObject(obj)) {
    const inspect = toString ? obj[toString] : null;
    if(typeof inspect == 'function' && !Util.isNativeFunction(inspect) && !/Util.inspect/.test(inspect + '')) {
      let s = inspect.call(obj, depth, { ...opts });
      //console.debug('s:', s);
      //console.debug('inspect:', inspect + '');

      out += s;
    } else {
      let isMap = obj instanceof Map;
      let keys = isMap ? obj.keys() : Object.getOwnPropertyNames(obj);
      //console.debug("keys:", keys);

      if(Object.getPrototypeOf(obj) !== Object.prototype) print(Util.className(obj) + ' ', 1, 31);
      isMap ? print(`(${obj.size}) {${sep(true)}`, 1, 36) : print('{' + (sep(true) || padding), 1, 36);
      let i = 0;
      let getFn = isMap ? key => obj.get(key) : key => obj[key];
      let propSep = isMap ? [' => ', 0] : [': ', 1, 36];
      for(let key of keys) {
        const value = getFn(key);
        if(i > 0) print(separator + sep(true), 36);
        if(typeof key == 'symbol') print(key.toString(), 1, 32);
        else if(Util.isObject(key) && typeof key[toString] == 'function') print(isMap ? `'${key[toString]()}'` : json ? `"${key.toString()}"` : key[toString](), 1, isMap ? 36 : 33);
        else if(typeof key == 'string' || (!isMap && Util.isObject(key) && typeof key.toString == 'function')) print(json ? `"${key.toString()}"` : isMap || /(-)/.test(key) ? `'${key}'` : key, 1, isMap ? 36 : 33);
        else
          Util.inspect(key, {
            ...opts,
            c,
            print,
            newline: newline + '  ',
            newline: '',
            multiline: false,
            toString: 'toString',
            depth: depth - 1
          });
        print(...propSep);
        if(typeof value == 'number') print(`${value}`, 1, 36);
        else if(typeof value == 'string' || value instanceof String) print(`${quote}${value}${quote}`, 1, 36);
        else if(typeof value == 'object')
          Util.inspect(value, {
            ...opts,
            print,
            multiline: isMap && !(value instanceof Map) ? false : multiline,
            newline: newline + '  ',
            depth: depth - 1
          });
        else print((value + '').replace(lineSplit, sep(true)));
        i++;
      }
      print(`${multiline ? newline : padding}}`, 1, 36);
    }
  }
  return out;
};

Util.inspect.defaultOpts = {
  spacing: ' ',
  padding: ' '
};

Util.dump = function(name, props) {
  const args = [name];
  for(let key in props) {
    f;
    args.push(`\n\t${key}: `);
    args.push(props[key]);
  }
  const w = Util.tryCatch(
    () => globalThis.window,
    w => w,
    () => null
  );

  if(w) {
    //if(window.alert !== undefined)
    //alert(args);
    if(w.console !== undefined) w.console.log(...args);
  }
};
Util.ucfirst = function(str) {
  if(typeof str != 'string') str = str + '';
  return str.substring(0, 1).toUpperCase() + str.substring(1);
};
Util.lcfirst = function(str) {
  if(typeof str != 'string') str = str + '';
  return str.substring(0, 1).toLowerCase() + str.substring(1);
};
Util.typeOf = v => {
  let type = typeof v;
  if(type == 'object' && v != null && Object.getPrototypeOf(v) != Object.prototype) type = Util.className(v);
  else type = Util.ucfirst(type);
  return type;
};

/**
 * Camelize a string, cutting the string by multiple separators like
 * hyphens, underscores and spaces.
 *
 * @param {text} string Text to camelize
 * @return string Camelized text
 */
Util.camelize = (text, sep = '') =>
  text.replace(/^([A-Z])|[\s-_]+(\w)/g, (match, p1, p2, offset) => {
    if(p2) return sep + p2.toUpperCase();
    return p1.toLowerCase();
  });

Util.decamelize = function(str, separator = '-') {
  return /.[A-Z]/.test(str)
    ? str
        .replace(/([a-z\d])([A-Z])/g, '$1' + separator + '$2')
        .replace(/([A-Z]+)([A-Z][a-z\d]+)/g, '$1' + separator + '$2')
        .toLowerCase()
    : str;
};
Util.ifThenElse = function(pred = value => !!value, _then = () => {}, _else = () => {}) {
  return function(value) {
    let result = pred(value);
    let ret = result ? _then(value) : _else(value);
    return ret;
  };
};
Util.if = (value, _then, _else, pred) => Util.ifThenElse(pred || (v => !!v), _then || (() => value), _else || (() => value))(value);

Util.ifElse = (value, _else, pred) => Util.ifThenElse(pred || (v => !!v), () => value, _else ? () => _else : () => value)(value);
Util.ifThen = (value, _then, pred) => Util.ifThenElse(pred || (v => !!v), _then ? () => _then : () => value, () => value)(value);

Util.switch = ({ default: defaultCase, ...cases }) =>
  function(value) {
    if(value in cases) return cases[value];
    return defaultCase;
  };

Util.transform = Util.curry(function* (fn, arr) {
  for(let item of arr) yield fn(item);
});

Util.colorDump = (iterable, textFn) => {
  textFn = textFn || ((color, n) => ('   ' + (n + 1)).slice(-3) + ` ${color}`);

  let j = 0;
  const filters = 'font-weight: bold; text-shadow: 0px 0px 1px rgba(0,0,0,0.8); filter: drop-shadow(30px 10px 4px #4444dd)';

  if(!Util.isArray(iterable)) iterable = [...iterable];
  for(let j = 0; j < iterable.length; j++) {
    const [i, color] = iterable[j].length == 2 ? iterable[j] : [j, iterable[j]];
    console.log(`  %c    %c ${color} %c ${textFn(color, i)}`, `background: ${color}; font-size: 18px; ${filters};`, `background: none; color: ${color}; min-width: 120px; ${filters}; `, `color: black; font-size: 12px;`);
  }
};

Util.bucketInserter = (map, ...extraArgs) => {
  let inserter;
  inserter =
    typeof map.has == 'function'
      ? function(...args) {
          //console.log("bucketInsert:",map,args);
          for(let [k, v] of args) {
            let a;
            map.has(k) ? (a = map.get(k)) : map.set(k, (a = []));
            a.push(v);
          }
          return inserter;
        }
      : function(...args) {
          for(let arg of args) {
            for(let k in arg) {
              const v = arg[k];
              let a = map[k] || [];
              if(typeof a.push == 'function') a.push(v);

              map[k] = a;
            }
          }
        };
  inserter(...extraArgs);
  inserter.map = map;
  return inserter;
};
Util.fifo = function fifo() {
  let resolve = () => {};
  const queue = [];

  //(there's no arrow function syntax for this)
  async function* generator() {
    for(;;) {
      if(!queue.length) {
        //there's nothing in the queue, wait until push()
        await new Promise(r => (resolve = r));
      }
      yield queue.shift();
    }
  }

  return {
    push(...args) {
      for(let event of args) {
        queue.push(event);
        if(queue.length === 1) resolve(); //allow the generator to resume
      }
      return this;
    },
    loop: generator(),

    process: async function run() {
      for await(const event of this.loop) {
        console.info('event:', event);
      }
    }
  };
};
Util.isEmail = function(v) {
  return /^[\-\w]+(\.[\-\w]+)*@[\-\w]+(\.[\-\w]+)+$/.test(v);
};
Util.isString = function(v) {
  return Object.prototype.toString.call(v) == '[object String]';
};

/**
 * Determines whether the specified v is numeric.
 *
 * @param      {<type>}   v       { parameter_description }
 * @return     {boolean}  True if the specified v is numeric, False otherwise.
 */
Util.isNumeric = v => /^[-+]?(0x|0b|0o|)[0-9]*\.?[0-9]+(|[Ee][-+]?[0-9]+)$/.test(v + '');

Util.isUndefined = arg => arg === undefined;
Util.isObject = obj => !(obj === null) && { object: obj, function: obj }[typeof obj];
Util.isPrimitive = obj => !(obj === null) && obj !== false && obj !== true && { number: obj, string: obj, boolean: obj, undefined: obj }[typeof obj];
Util.isFunction = arg => {
  if(arg !== undefined) return typeof arg == 'function' || !!(arg && arg.constructor && arg.call && arg.apply);

  /*
  let fn = arg => Util.isFunction(arg);
  fn.inverse = arg => !Util.isFunction(arg);
  return fn;*/
};
Util.not = fn =>
  function(...args) {
    return !fn(...args);
  };
Util.isAsync = fn => typeof fn == 'function' && /^[\n]*async/.test(fn + '') /*|| fn() instanceof Promise*/;

Util.isArrowFunction = fn => (Util.isFunction(fn) && !('prototype' in fn)) || /\ =>\ /.test(('' + fn).replace(/\n.*/g, ''));

Util.isEmptyString = v => Util.isString(v) && (v == '' || v.length == 0);

Util.isEmpty = (...args) => {
  function empty(v) {
    if(typeof v == 'object' && !!v && v.constructor == Object && Object.keys(v).length == 0) return true;
    if(!v || v === null) return true;
    if(typeof v == 'object' && v.length !== undefined && v.length === 0) return true;
    return false;
  }
  return args.length ? empty(args[0]) : empty;
};
Util.isNonEmpty = (...args) => {
  const empty = Util.isEmpty();
  const nonEmpty = v => !empty(v);
  return args.length ? nonEmpty(args[0]) : nonEmpty;
};
Util.isIpAddress = v => {
  const n = (v + '').split('.').map(i => +i);
  return n.length == 4 && n.every(i => !isNaN(i) && i >= 0 && i <= 255);
};
Util.isPortNumber = v => {
  const n = +v;
  return !isNaN(n) && n >= 0 && n <= 65535;
};

Util.hasProps = function(obj, props) {
  const keys = Object.keys(obj);
  return props ? props.every(prop => 'prop' in obj) : keys.length > 0;
};
Util.validatePassword = function(value) {
  return value.length > 7 && new RegExp('^(?![d]+$)(?![a-zA-Z]+$)(?![!#$%^&*]+$)[da-zA-Z!#$ %^&*]').test(value) && !/\s/.test(value);
};
Util.clone = function(obj, proto) {
  if(Util.isArray(obj)) return obj.slice();
  try {
    let ret = new obj.constructor(obj);
    return ret;
  } catch(err) {}
  if(typeof obj == 'object') return Object.create(proto || obj.constructor.prototype || Object.getPrototypeOf(obj), Object.getOwnPropertyDescriptors(obj));
};
//deep copy
Util.deepClone = function(data) {
  return JSON.parse(JSON.toString(data));
};
//Function
Util.findVal = function(object, propName, maxDepth = 10) {
  if(maxDepth <= 0) return null;
  for(let key in object) {
    if(key === propName) {
      //console.log(propName);
      //console.log(object[key]);
      return object[key];
    }
    let value = Util.findVal(object[key], propName, maxDepth - 1);
    if(value !== undefined) return value;
  }
};
//Deep copy for ObservableArray/Object == There is a problem
Util.deepCloneObservable = function(data) {
  let o;
  const t = typeof data;
  if(t === 'object') return data;

  if(t === 'object') {
    if(data.length) {
      for(const value of data) {
        o.push(this.deepCloneObservable(value));
      }
      return o;
    }
    for(const i in data) {
      o[i] = this.deepCloneObservable(data[i]);
    }
    return o;
  }
};
//Convert ObservableArray to Array
Util.toArray = function(observableArray) {
  return observableArray.slice();
};

/**
 * Convert the original array to tree
 * @param data original array
 * @param id id field
 * @param pId parent id field
 * @param appId the parent id value of the level one array
 */
Util.arryToTree = function(data, id, pId, appId) {
  const arr = [];
  data.map((e, i) => {
    e[pId] === appId && arr.push(e);
  });
  const res = this.to3wei(arr, data, id, pId);
  return res;
};

/**
 * Convert a first-level branch array to a tree
 * @param a level one branch array
 * @param old original array
 * @param id id field
 * @param pId parent id field
 */
Util.to3wei = function(a, old, id, pId) {
  a.map((e, i) => {
    a[i].children = [];
    old.map((se, si) => {
      if(se[pId] === a[i][id]) {
        a[i].children = [...a[i].children, se];
        this.to3wei(a[i].children, old, id, pId);
      }
    });
    if(!a[i].children.length) {
      delete a[i].children;
    }
  });
  return a;
};

/**
 * Exchange 2 element positions in the array
 * @param arr original array
 * @param i First element Starting from 0
 * @param j The second element starts at 0
 */
Util.arrExchangePos = function(arr, i, j) {
  arr[i] = arr.splice(j, 1, arr[i])[0];
};
Util.arrRemove = function(arr, i) {
  const index = arr.indexOf(i);
  if(index > -1) arr.splice(index, 1);
};
Util.move = function(src, dst = []) {
  let items = src.splice(0, src.length);
  dst.splice(dst.length, 0, ...items);
  return dst;
};
Util.moveIf = function(src, pred, dst = []) {
  let items = src.splice(0, src.length);
  let i = 0;
  for(let item of items) (pred(item, i++) ? src : dst).push(item);

  return dst;
};
//Remove the storage when logging out
Util.logOutClearStorage = function() {
  localStorage.removeItem('userToken');
  localStorage.removeItem('userLoginPermission');
  localStorage.removeItem('ssoToken');
  localStorage.removeItem('userId');
  localStorage.removeItem('userInfo');
  localStorage.removeItem('userGroupList');
  localStorage.removeItem('gameAuthList');
};
//Take the cookies
Util.getCookie = function(cookie, name) {
  let arr = cookie.match(new RegExp('(^| )' + name + '=([^;]*)(;|$)'));
  if(arr != null) return unescape(arr[2]);
  return null;
};
Util.parseCookie = function(c = document.cookie) {
  if(!(typeof c == 'string' && c && c.length > 0)) return {};
  let key = '';
  let value = '';
  const ws = ' \r\n\t';
  let i = 0;
  let ret = {};
  const skip = (pred = char => ws.indexOf(char) != -1) => {
    let start = i;
    while(i < c.length && pred(c[i])) i++;
    let r = c.substring(start, i);
    return r;
  };
  do {
    let str = skip(char => char != '=' && char != ';');
    if(c[i] == '=' && str != 'path') {
      i++;
      key = str;
      value = skip(char => char != ';');
    } else {
      i++;
      skip();
    }
    if(key != '') ret[key] = value;
    skip();
  } while(i < c.length);
  return ret;
};

/*
    matches.shift();
    return matches.reduce((acc, part) => {
      const a = part.trim().split('=');
      return { ...acc, [a[0]]: decodeURIComponent(a[1]) };
    }, {});
  };*/
Util.encodeCookie = c =>
  Object.entries(c)
    .map(([key, value]) => `${key}=${encodeURIComponent(value)}`)
    .join('; ');
Util.setCookies = c =>
  Object.entries(c).forEach(([key, value]) => {
    document.cookie = `${key}=${value}`;
    //console.log(`Setting cookie[${key}] = ${value}`);
  });
Util.clearCookies = function(c) {
  return Util.setCookies(
    Object.keys(Util.parseCookie(c)).reduce(
      (acc, name) =>
        Object.assign(acc, {
          [name]: `; max-age=0; expires=${new Date().toUTCString()}`
        }),
      {}
    )
  );
};
Util.deleteCookie = function(name) {
  const w = Util.tryCatch(
    () => globalThis.window,
    w => w,
    () => null
  );

  if(w) document.cookie = `${name}=; expires=Thu, 01 Jan 1970 00:00:01 GMT;`;
};
Util.accAdd = function(arg1, arg2) {
  let r1, r2, m;
  try {
    r1 = arg1.toString().split('.')[1].length;
  } catch(e) {
    r1 = 0;
  }
  try {
    r2 = arg2.toString().split('.')[1].length;
  } catch(e) {
    r2 = 0;
  }
  m = Math.pow(10, Math.max(r1, r2));
  return (arg1 * m + arg2 * m) / m;
};
//js subtraction calculation
//
Util.Subtr = function(arg1, arg2) {
  let r1, r2, m, n;
  try {
    r1 = arg1.toString().split('.')[1].length;
  } catch(e) {
    r1 = 0;
  }
  try {
    r2 = arg2.toString().split('.')[1].length;
  } catch(e) {
    r2 = 0;
  }
  m = Math.pow(10, Math.max(r1, r2));
  //last modify by deeka
  //动态控制精度长度
  n = r1 >= r2 ? r1 : r2;
  return (arg1 * m - arg2 * m) / m;
};
//js division function
//
Util.accDiv = function(arg1, arg2) {
  let t1 = 0;
  let t2 = 0;
  let r1;
  let r2;
  try {
    t1 = arg1.toString().split('.')[1].length;
  } catch(e) {}
  try {
    t2 = arg2.toString().split('.')[1].length;
  } catch(e) {}
  r1 = Number(arg1.toString().replace('.', ''));
  r2 = Number(arg2.toString().replace('.', ''));
  return (r1 / r2) * Math.pow(10, t2 - t1);
};
//js multiplication function
//
Util.accMul = function(arg1, arg2) {
  let m = 0;
  const s1 = arg1.toString();
  const s2 = arg2.toString();
  try {
    m += s1.split('.')[1].length;
  } catch(e) {}
  try {
    m += s2.split('.')[1].length;
  } catch(e) {}
  return (Number(s1.replace('.', '')) * Number(s2.replace('.', ''))) / Math.pow(10, m);
};
Util.dateFormatter = function(date, formate) {
  const year = date.getFullYear();
  let month = date.getMonth() + 1;
  month = month > 9 ? month : `0${month}`;
  let day = date.getDate();
  day = day > 9 ? day : `0${day}`;
  let hour = date.getHours();
  hour = hour > 9 ? hour : `0${hour}`;
  let minute = date.getMinutes();
  minute = minute > 9 ? minute : `0${minute}`;
  let second = date.getSeconds();
  second = second > 9 ? second : `0${second}`;
  return formate
    .replace(/Y+/, `${year}`.slice(-formate.match(/Y/g).length))
    .replace(/M+/, month)
    .replace(/D+/, day)
    .replace(/h+/, hour)
    .replace(/m+/, minute)
    .replace(/s+/, second);
};
Util.numberFormatter = function(numStr) {
  let numSplit = numStr.split('.');
  return numSplit[0].replace(/\B(?=(\d{3})+(?!\d))/g, ',').concat(`.${numSplit[1]}`);
};
Util.searchObject = function(object, matchCallback, currentPath, result, searched) {
  currentPath = currentPath || '';
  result = result || [];
  searched = searched || [];
  if(searched.indexOf(object) !== -1 && object === Object(object)) {
    return;
  }
  searched.push(object);
  if(matchCallback(object)) {
    result.push({ path: currentPath, value: object });
  }
  try {
    if(object === Object(object)) {
      for(const property in object) {
        const desc = Object.getOwnPropertyDescriptor(object, property);
        //console.log('x ', {property, desc})
        if(property.indexOf('$') !== 0 && typeof object[property] !== 'function' && !desc.get && !desc.set) {
          if(typeof object[property] === 'object') {
            try {
              JSON.toString(object[property]);
            } catch(err) {
              continue;
            }
          }
          //if (Object.prototype.hasOwnProperty.call(object, property)) {
          Util.searchObject(object[property], matchCallback, `${currentPath}.${property}`, result, searched);
          //}
        }
      }
    }
  } catch(e) {
    //console.log(object);
    //throw e;
  }
  return result;
};
Util.getURL = Util.memoize((req = {}) =>
  Util.tryCatch(
    () => process.argv[1],
    () => 'file://' + Util.scriptDir(),

    Util.tryCatch(
      () => window.location.href,

      url => url,

      () => {
        let proto = Util.tryCatch(() => (process.env.NODE_ENV === 'production' ? 'https' : null)) || 'http';
        let port = Util.tryCatch(() => (process.env.PORT ? parseInt(process.env.PORT) : process.env.NODE_ENV === 'production' ? 443 : null)) || 3000;
        let host = Util.tryCatch(() => globalThis.ip) || Util.tryCatch(() => globalThis.host) || Util.tryCatch(() => window.location.host.replace(/:.*/g, '')) || 'localhost';
        if(req && req.headers && req.headers.host !== undefined) host = req.headers.host.replace(/:.*/, '');
        else Util.tryCatch(() => process.env.HOST !== undefined && (host = process.env.HOST));
        if(req.url !== undefined) return req.url;
        const url = `${proto}://${host}:${port}`;
        return url;
      }
    )
  )
);
Util.parseQuery = function(url = Util.getURL()) {
  let startIndex;
  let query = {};
  try {
    if((startIndex = url.indexOf('?')) != -1) url = url.substring(startIndex);
    const args = [...url.matchAll(/[?&]([^=&#]+)=?([^&#]*)/g)];
    if(args) {
      for(let i = 0; i < args.length; i++) {
        const k = args[i][1];
        query[k] = decodeURIComponent(args[i][2]);
      }
    }
    return query;
  } catch(err) {
    return undefined;
  }
};
Util.encodeQuery = function(data) {
  const ret = [];
  for(let d in data) if(data[d] !== undefined) ret.push(`${encodeURIComponent(d)}=${encodeURIComponent(data[d])}`);
  return ret.join('&');
};
Util.parseURL = function(href = this.getURL()) {
  //console.debug('href:', href);
  const matches = new RegExp('^([^:]+://)?([^/:]*)(:[0-9]*)?(/?[^#]*)?(#.*)?', 'g').exec(href);
  const [all, proto, host, port, location = '', fragment] = matches;
  //console.debug('matches:', matches);
  if(!matches) return null;
  const argstr = location.indexOf('?') != -1 ? location.replace(/^[^?]*\?/, '') : ''; /* + "&test=1"*/
  const pmatches =
    typeof argstr === 'string'
      ? argstr
          .split(/&/g)
          .map(part => {
            let a = part.split(/=/);
            let b = a.shift();
            return [b, a.join('=')];
          })
          .filter(([k, v]) => !(k.length == 0 && v.length == 0))
      : [];
  const params = [...pmatches].reduce((acc, m) => {
    acc[m[0]] = m[1];
    return acc;
  }, {});
  //console.log("PARAMS: ", { argstr, pmatches, params });
  const ret = {
    protocol: proto ? proto.replace('://', '') : 'http',
    host,
    location: location.replace(/\?.*/, ''),
    query: params
  };
  Object.assign(ret, {
    href(override) {
      if(typeof override === 'object') Object.assign(this, override);
      const qstr = Util.encodeQuery(this.query);
      return (this.protocol ? `${this.protocol}://` : '') + (this.host ? this.host : '') + (this.port ? `:${this.port}` : '') + `${this.location}` + (qstr != '' ? `?${qstr}` : '');
    }
  });
  if(typeof port === 'string') ret.port = parseInt(port.substring(1));
  else if(ret.protocol == 'https') ret.port = 443;
  else if(ret.protocol == 'http') ret.port = 80;
  if(fragment) ret.fragment = fragment;
  return ret;
};
Util.makeURL = function(...args) {
  let href = typeof args[0] == 'string' ? args.shift() : Util.getURL();
  let url = Util.parseURL(href);
  let obj = typeof args[0] == 'object' ? args.shift() : {};
  if('host' in obj /*|| 'protocol' in obj*/) url = Util.filterOutKeys(url, [/*'protocol',*/ 'host', 'port']);
  Object.assign(url, obj);
  return url.href();

  /*
  let href = typeof args[0] === "string" ? args.shift() : this.getURL();
  let urlObj = null;
  urlObj = this.parseURL(href);
  return urlObj ? urlObj.href(args[0]) : null;*/
};
Util.numberFromURL = function(url, fn) {
  const obj = typeof url === 'object' ? url : this.parseURL(url);
  const nr_match = RegExp('.*[^0-9]([0-9]+)$').exec(url.location);
  const nr_arg = nr_match ? nr_match[1] : undefined;
  const nr = nr_arg && parseInt(nr_arg);
  if(!isNaN(nr) && typeof fn === 'function') fn(nr);
  return nr;
};
Util.tryPromise = fn => new Promise((resolve, reject) => Util.tryCatch(fn, resolve, reject));

Util.tryFunction = (fn, resolve = a => a, reject = () => null) => {
  if(typeof resolve != 'function') {
    let rval = resolve;
    resolve = () => rval;
  }
  if(typeof reject != 'function') {
    let cval = reject;
    reject = () => cval;
  }
  return Util.isAsync(fn)
    ? async function(...args) {
        let ret;
        try {
          ret = await fn(...args);
        } catch(err) {
          return reject(err, ...args);
        }
        return resolve(ret, ...args);
      }
    : function(...args) {
        let ret;
        try {
          ret = fn(...args);
        } catch(err) {
          return reject(err, ...args);
        }
        return resolve(ret, ...args);
      };
};
Util.tryCatch = (fn, resolve = a => a, reject = () => null, ...args) => {
  if(Util.isAsync(fn))
    return fn(...args)
      .then(resolve)
      .catch(reject);

  return Util.tryFunction(fn, resolve, reject)(...args);
};
Util.putError = err => {
  let e = Util.isObject(err) && err instanceof Error ? err : Util.exception(err);
  (console.info || console.log)('Util.putError ', e);
  let s = err.stack ? Util.stack(err.stack) : null;

  (console.error || console.log)('ERROR:\n' + err.message + (s ? '\nstack:\n' + s.toString() : s));
};
Util.putStack = (stack = new Util.stack().slice(3)) => {
  stack = stack instanceof Util.stack ? stack : Util.stack(stack);
  console.log('Util.putStack', Util.className(stack));

  console.log('STACK TRACE:\n' + stack.toString());
};

Util.trap = (() => {
  Error.stackTraceLimit = 100;
  return fn => /* prettier-ignore */ Util.tryFunction(fn, ret => ret, Util.putError);
})();

Util.tryPredicate = (fn, defaultRet) =>
  Util.tryFunction(
    fn,
    ret => ret,
    () => defaultRet
  );

Util.isBrowser = function() {
  let ret = false;

  Util.tryCatch(
    () => window,
    w => (Util.isObject(w) ? (ret = true) : undefined),
    () => {}
  );
  Util.tryCatch(
    () => document,
    d => (d == window.document && Util.isObject(d) ? (ret = true) : undefined),
    () => {}
  );
  return ret;
  //return !!(globalThis.window && globalThis.window.document);
};

Util.waitFor = async function waitFor(msecs) {
  if(!globalThis.setTimeout) {
    await import('os').then(({ setTimeout, clearTimeout, setInterval, clearInterval }) => {
      //console.log('', { setTimeout, clearTimeout, setInterval, clearInterval });
      Object.assign(globalThis, {
        setTimeout,
        clearTimeout,
        setInterval,
        clearInterval
      });
    });
  }
  if(msecs <= 0) return;

  let promise, clear, timerId;
  promise = new Promise(async (resolve, reject) => {
    timerId = setTimeout(() => resolve(), msecs);
    clear = () => {
      clearTimeout(timerId);
      reject();
    };
  });
  promise.clear = clear;
  return promise;
};

Util.timeout = async (msecs, promises, promiseClass = Promise) => await promiseClass.race([Util.waitFor(msecs)].concat(Util.isArray(promises) ? promises : [promises]));
Util.isServer = function() {
  return !Util.isBrowser();
};
Util.isMobile = function() {
  return true;
};
Util.uniquePred = (cmp = null) => (typeof cmp == 'function' ? (el, i, arr) => arr.findIndex(item => cmp(el, item)) == i : (el, i, arr) => arr.indexOf(el) == i);

Util.unique = (arr, cmp) => arr.filter(Util.uniquePred(cmp));
Util.allEqual = (arr, cmp = (a, b) => a == b) => arr.every((e, i, a) => cmp(e, a[0]));

Util.zip = a => a.reduce((a, b) => (a.length > b.length ? a : b), []).map((_, i) => a.map(arr => arr[i]));

Util.histogram = (...args) => {
  let arr = args.shift();
  const t = typeof args[0] == 'function' ? args.shift() : (k, v) => k;
  let [out = false ? {} : new Map(), initVal = () => 0 /* new Set()*/, setVal = v => v] = args;

  const set = /*Util.isObject(out) && typeof out.set == 'function' ? (k, v) => out.set(k, v) :*/ Util.setter(out);
  const get = Util.getOrCreate(out, initVal, set);
  let ctor = Object.getPrototypeOf(out) !== Object.prototype ? out.constructor : null;
  let tmp;

  if(Util.isObject(arr) && !Array.isArray(arr) && typeof arr.entries == 'function') arr = arr.entries();
  arr = [...arr];
  let entries = arr.map((it, i) => [i, it]);
  let x = {};
  let iv = initVal();
  const add = Util.adder(iv);
  const upd = Util.updater(out, get, set);

  let r = arr.map((item, i) => {
    let arg;
    let key;
    tmp = t(item, i);
    if(tmp) {
      key = tmp;
      if(Util.isArray(tmp) && tmp.length >= 2) [key, arg] = tmp.slice(-2);
      else arg = tmp;
    }
    [key, arg] = [key].concat(setVal(arg, i)).slice(-2);
    return [
      key,
      upd(key, (entry, idx, key) => {
        return add(entry, typeof entry == 'number' ? 1 : item);
      })
    ];
  });
  return out;
  //console.debug('r:', r);
  if(ctor) {
    let entries = r;
    let keys = r.map(([k, v]) => k);
    entries = [...entries].sort((a, b) => b[1] - a[1]);
    let tmp = new ctor(entries);
    r = tmp;
  }
  return r;
};
Util.concat = function* (...args) {
  for(let arg of args) {
    if(Util.isGenerator(arg)) {
      console.error('isGenerator:', arg);
      yield* arg;
    } else {
      /* if(Util.isArray(arg))*/
      for(let item of arg) yield item;
    }

    /*   else  else {
      throw new Error("No such arg type:"+typeof(arg));
    }*/
  }
};
Util.distinct = function(arr) {
  return Array.prototype.filter.call(arr, (value, index, me) => me.indexOf(value) === index);
};
Util.rangeMinMax = function(arr, field) {
  const numbers = [...arr].map(obj => obj[field]);
  return [Math.min(...numbers), Math.max(...numbers)];
};

Util.remap = (...args) => {
  const getR = () => (Util.isArray(args[0]) ? args.shift() : args.splice(0, 2));
  const _from = getR(),
    to = getR();

  const f = [to[1] - to[0], _from[1] - _from[0]];
  const factor = f[0] / f[1];

  const r = val => (val - _from[0]) * factor + to[0];

  return r;
};
Util.mergeLists = function(arr1, arr2, key = 'id') {
  let hash = {};

  for(let obj of arr1) hash[obj[key]] = obj;
  for(let obj of arr2) hash[obj[key]] = obj;
  return Object.values(hash);

  /* let hash = arr1.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
  hash = arr2.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
  let ret = [];
  for(let k in hash) {
    if(hash[k][key]) ret.push(hash[k]);
  }
  return ret;*/
};

Util.foreach = function(o, fn) {
  for(let [k, v] of Util.entries(o)) {
    if(fn(v, k, o) === false) break;
  }
};
Util.all = function(obj, pred) {
  for(let k in obj) if(!pred(obj[k])) return false;
  return true;
};
Util.isGenerator = function(fn) {
  return (typeof fn == 'function' && /^[^(]*\*/.test(fn.toString())) || (['function', 'object'].indexOf(typeof fn) != -1 && fn.next !== undefined);
};
Util.isIterator = obj => Util.isObject(obj) && typeof obj.next == 'function';

Util.isIterable = obj => {
  try {
    for(let item of obj) return true;
  } catch(err) {}
  return false;
};
Util.isNativeFunction = Util.tryFunction(x => typeof x == 'function' && /^[^\n]*\[(native\ code|[a-z\ ]*)\]/.test(x + ''));

Util.isConstructor = x => {
  if(x !== undefined) {
    let ret,
      members = [];
    const handler = {
      construct(target, args) {
        return Object.create(target.prototype);
      }
    };
    try {
      ret = new new Proxy(x, handler)();
    } catch(e) {
      ret = false;
    }
    let proto = (x && x.prototype) || Object.getPrototypeOf(ret);
    members = Util.getMemberNames(proto).filter(m => m !== 'constructor');
    //console.log('members:', !!ret, members, Util.fnName(x));
    return !!ret && members.length > 0;
  }
};

Util.filter = function(a, pred) {
  if(typeof pred != 'function') pred = Util.predicate(pred);
  if(Util.isArray(a)) return a.filter(pred);
  /*return (function* () {
      for(let [k, v] of a.entries()) if(pred(v, k, a)) yield v;
    })();*/

  if(Util.isGenerator(a))
    return (function* () {
      for(let item of a) if(pred(item)) yield item;
    })();
  let isa = Util.isArray(a);
  let ret = {};
  let fn = (k, v) => (ret[k] = v);
  for(let [k, v] of Util.entries(a)) if(pred(v, k, a)) fn(k, v);
  return Object.setPrototypeOf(ret, Object.getPrototypeOf(a));
};
Util.reduce = (obj, fn, accu) => {
  if(Util.isGenerator(obj)) {
    let i = 0;
    for(let item of obj) accu = fn(accu, item, i++, obj);
    return accu;
  }
  for(let key in obj) accu = fn(accu, obj[key], key, obj);
  return accu;
};
Util.mapFunctional = fn =>
  function* (arg) {
    for(let item of arg) yield fn(item);
  };
Util.map = (...args) => {
  let [obj, fn] = args;
  let ret = a => a;

  if(Util.isIterator(obj)) {
    return ret(function* () {
      let i = 0;
      for(let item of obj) yield fn(item, i++, obj);
    })();
  }
  if(typeof obj == 'function') return Util.mapFunctional(...args);

  if(typeof obj.map == 'function') return obj.map(fn);

  if(typeof obj.entries == 'function') {
    const ctor = obj.constructor;
    obj = obj.entries();
    ret = a => new ctor([...a]);
    //    ret = a => new ctor(a);
  }

  /*console.log("obj",(obj));
console.log("isGenerator",Util.isGenerator(obj));*/

  if(Util.isGenerator(obj))
    return ret(
      (function* () {
        let i = 0;
        for(let item of obj) yield fn(item, i++, obj);
      })()
    );
  //  if(typeof fn != 'function') return Util.toMap(...arguments);

  ret = {};
  for(let key in obj) {
    if(obj.hasOwnProperty(key)) {
      let item = fn(key, obj[key], obj);
      if(item) ret[item[0]] = item[1];
    }
  }
  return ret; //Object.setPrototypeOf(ret,Object.getPrototypeOf(obj));
};

/*Util.indexedMap = (arr, fn = arg => arg.name) => {
  return new Proxy(arr, {
    get(target, prop, receiver) {
      let idx = arr.findIndex(item => fn(item) == 'prop');
      if(idx != -1)
        prop = idx;

      return Reflect.get(arr, idx, receiver);
    }
  });
};*/

Util.entriesToObj = function(arr) {
  return [...arr].reduce((acc, item) => {
    const k = item[0];
    const v = item[1];
    acc[k] = v;
    return acc;
  }, {});
};
Util.isDate = function(d) {
  return d instanceof Date || (typeof d == 'string' && /[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]T[0-9][0-9]:[0-9][0-9]:[0-9][0-9]/.test(d));
};
Util.parseDate = function(d) {
  if(Util.isDate(d)) {
    d = new Date(d);
  }
  return d;
  //return /^[0-9]+$/.test(d) ? Util.fromUnixTime(d) : new Date(d);
};
Util.isoDate = function(date) {
  try {
    if(typeof date == 'number') date = new Date(date);
    const minOffset = date.getTimezoneOffset();
    const milliseconds = date.valueOf() - minOffset * 60 * 1000;
    date = new Date(milliseconds);
    return date.toISOString().replace(/T.*/, '');
  } catch(err) {}
  return null;
};
Util.toUnixTime = function(dateObj, utc = false) {
  if(!(dateObj instanceof Date)) dateObj = new Date(dateObj);
  let epoch = Math.floor(dateObj.getTime() / 1000);
  if(utc) epoch += dateObj.getTimezoneOffset() * 60;
  return epoch;
};
Util.unixTime = function(utc = false) {
  return Util.toUnixTime(new Date(), utc);
};
Util.fromUnixTime = function(epoch, utc = false) {
  let t = parseInt(epoch);
  let d = new Date(0);
  utc ? d.setUTCSeconds(t) : d.setSeconds(t);
  return d;
};
Util.formatTime = function(date = new Date(), format = 'HH:MM:SS') {
  let n;
  let out = '';
  if(typeof date == 'number') date = new Date(date);
  for(let i = 0; i < format.length; i += n) {
    n = 1;
    while(format[i] == format[i + n]) n++;
    const fmt = format.substring(i, i + n);
    let num = fmt;
    if(fmt.startsWith('H')) num = `0${date.getHours()}`.substring(0, n);
    else if(fmt.startsWith('M')) num = `0${date.getMinutes()}`.substring(0, n);
    else if(fmt.startsWith('S')) num = `0${date.getSeconds()}`.substring(0, n);
    out += num;
  }
  return out;
};
Util.leapYear = function(year) {
  if(year % 400 == 0) return true;
  if(year % 100 == 0) return false;
  if(year % 4 == 0) return true;
  return false;
};
Util.timeSpan = function(s) {
  const seconds = s % 60;
  s = Math.floor(s / 60);
  const minutes = s % 60;
  s = Math.floor(s / 60);
  const hours = s % 24;
  s = Math.floor(s / 24);
  const days = s % 7;
  s = Math.floor(s / 7);
  const weeks = s;
  let ret = '';
  ret = `${('0' + hours).substring(0, 2)}:${('0' + minutes).substring(0, 2)}:${('0' + seconds).substring(0, 2)}`;
  if(days) ret = `${days} days ${ret}`;
  if(weeks) ret = `${weeks} weeks ${ret}`;
  return ret;
};
Util.rng = Math.random;
Util.randFloat = function(min, max, rnd = Util.rng) {
  return rnd() * (max - min) + min;
};
Util.randInt = (...args) => {
  let range = args.splice(0, 2);
  let rnd = args.shift() || Util.rng;
  if(range.length < 2) range.unshift(0);
  return Math.round(Util.randFloat(...range, rnd));
};
Util.randStr = (len, charset, rnd = Util.rng) => {
  let o = '';
  if(!charset) charset = '_0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz';

  while(--len >= 0) {
    o += charset[Math.round(rnd() * (charset.length - 1))];
  }
  return o;
};

Util.hex = function(num, numDigits) {
  let v = typeof num == 'number' ? num : parseInt(num);
  let s = v.toString(16);
  numDigits = numDigits || Math.floor((s.length + 1) / 2) * 2;
  return ('0'.repeat(numDigits) + s).slice(-numDigits);
};
Util.numberParts = (num, base) => {
  let exp = 0;
  let sgn = 0;
  if(num === 0) return { sign: 0, mantissa: 0, exponent: 0 };
  if(num < 0) (sgn = 1), (num = -num);
  while(num > base) (num /= base), exp++;
  while(num < 1) (num *= base), exp--;
  return { sign: sgn, mantissa: num, exponent: exp };
};
Util.roundDigits = precision => {
  precision = precision + '';
  let index = precision.indexOf('.');
  let frac = index == -1 ? '' : precision.slice(index + 1);
  return frac.length;

  return -Util.clamp(-Infinity, 0, Math.floor(Math.log10(precision - Number.EPSILON)));
};

Util.roundFunction = (prec, digits, type) => {
  digits = digits || Util.roundDigits(prec);
  type = type || 'round';

  const fn = Math[type];
  if(prec == 1) return fn;

  return function(value) {
    let ret = fn(value / prec) * prec;
    if(typeof digits == 'number' && digits >= 1 && digits <= 100) ret = +ret.toFixed(digits);
    return ret;
  };
};
Util.roundTo = function(value, prec, digits, type) {
  if(!isFinite(value)) return value;
  digits = digits || Util.roundDigits(prec);
  type = type || 'round';
  const fn = Math[type];
  if(prec == 1) return fn(value);
  let ret = prec > Number.EPSILON ? fn(value / prec) * prec : value;

  if(typeof digits == 'number' && digits >= 1 && digits <= 100) ret = +ret.toFixed(digits);
  else ret = Math[type](ret);
  return ret;
};
Util.base64 = (() => {
  const g = Util.getGlobalObject();

  return {
    encode: Util.tryFunction(
      utf8 => g.btoa(g.unescape(g.encodeURIComponent(utf8))),
      v => v,
      utf8 => Buffer.from(utf8).toString('base64')
    ),
    decode: Util.tryFunction(
      base64 => g.decodeURIComponent(g.escape(g.atob(base64))),
      v => v,
      string => Buffer.from(string, 'base64').toString('utf-8')
    )
  };
})();

Util.formatRecord = function(obj) {
  let ret = {};
  for(let key in obj) {
    let val = obj[key];
    if(val instanceof Array) val = val.map(item => Util.formatRecord(item));
    else if(/^-?[0-9]+$/.test(val)) val = parseInt(val);
    else if(/^-?[.0-9]+$/.test(val)) val = parseFloat(val);
    else if(val == 'true' || val == 'false') val = Boolean(val);
    ret[key] = val;
  }
  return ret;
};
Util.isArray =
  Array.isArray ||
  function(obj) {
    if(obj.constructor === Array) return true;
    return (obj && !Util.isGetter(obj, 'length') && Util.isObject(obj) && 'length' in obj && !(obj instanceof String) && !(obj instanceof Function) && typeof obj == 'function') || obj instanceof Array;
  };
Util.isArrayLike = obj => typeof obj == 'object' && obj !== null && 'length' in obj;

Util.equals = function(a, b) {
  if(Util.isArray(a) && Util.isArray(b)) {
    return a.length == b.length && a.every((e, i) => b[i] === e);
  } else if(Util.isObject(a) && Util.isObject(b)) {
    const size_a = Util.size(a);

    if(size_a != Util.size(b)) return false;

    for(let k in a) if(!Util.equals(a[k], b[k])) return false;

    return true;
  }
  return a == b;
};
/*#define _GNU_SOURCE
#include <ctype.h>
#include <string.h>

int
strverscmp(const char* a0, const char* b0) {
  const unsigned char* a = (const void*)a0;
  const unsigned char* b = (const void*)b0;
  size_t i, dp, j;
  int z = 1;
  for(dp = i = 0; a[i] == b[i]; i++) {
    int c = a[i];
    if(!c)
      return 0;
    if(!isdigit(c))
      dp = i + 1, z = 1;
    else if(c != '0')
      z = 0;
  }
  if(a[dp] != '0' && b[dp] != '0') {
    for(j = i; isdigit(a[j]); j++)
      if(!isdigit(b[j]))
        return 1;
    if(isdigit(b[j]))
      return -1;
  } else if(z && dp < i && (isdigit(a[i]) || isdigit(b[i]))) {
    return (unsigned char)(a[i] - '0') - (unsigned char)(b[i] - '0');
  }
  return a[i] - b[i];
}*/
Util.versionCompare = (a, b) => {
  // console.log("Util.versionCompare",{a,b});
  if(typeof a != 'string') a = a + '';
  if(typeof b != 'string') b = b + '';

  let i,
    dp,
    j,
    z = 1;
  const isdigit = c => /^[0-9]$/.test(c);

  for(dp = i = 0; a[i] == b[i]; i++) {
    let c;
    if(!(c = a[i])) return 0;
    if(!isdigit(c)) (dp = i + 1), (z = 1);
    else if(c != '0') z = 0;
  }
  if(a[dp] != '0' && b[dp] != '0') {
    for(j = i; isdigit(a[j]); j++) if(!isdigit(b[j])) return 1;
    if(isdigit(b[j])) return -1;
  } else if(z && dp < i && (isdigit(a[i]) || isdigit(b[i]))) {
    return a.codePointAt(i) - 0x30 - (b.codePointAt(i) - 0x30);
  }

  return a.codePointAt(i) - b.codePointAt(i);
};

/*
Util.isObject = function(obj) {
  const type = typeof obj;
  return type === 'function' || (type === 'object' && !!obj);
};*/

Util.isGetter = (obj, propName) => {
  while(obj) {
    let desc = Object.getOwnPropertyDescriptor(obj, propName);
    if(desc && 'get' in desc) return true;
    obj = Object.getPrototypeOf(obj);
  }
  return false;
};
Util.isBool = value => value === true || value === false;
Util.size = (...args) => {
  function size(obj) {
    if(Util.isObject(obj)) {
      if(obj instanceof Map) return obj.size;
      else if('length' in obj) return obj.length;
      else return Object.keys(obj).length;
    }
  }
  if(args.length == 0) return size;
  return size(args[0]);
};
Util.isMap = function(obj) {
  return (obj && obj.get !== undefined && obj.keys !== undefined) || obj instanceof Map;
};
Util.effectiveDeviceWidth = function() {
  let deviceWidth = window.orientation == 0 ? window.screen.width : window.screen.height;
  //iOS returns available pixels, Android returns pixels / pixel ratio
  //http://www.quirksmode.org/blog/archives/2012/07/more_about_devi.html
  if(navigator.userAgent.indexOf('Android') >= 0 && window.devicePixelRatio) {
    deviceWidth = deviceWidth / window.devicePixelRatio;
  }
  return deviceWidth;
};
Util.getFormFields = function(initialState) {
  return Util.mergeObjects([initialState, [...document.forms].reduce((acc, { elements }) => [...elements].reduce((acc2, { name, value }) => (name == '' || value == undefined || value == 'undefined' ? acc2 : Object.assign(acc2, { [name]: value })), acc), {})]);
};
Util.mergeObjects = function(objArr, predicate = (dst, src, key) => (src[key] == '' ? undefined : src[key])) {
  let args = objArr;
  let obj = {};
  for(let i = 0; i < args.length; i++) {
    for(let key in args[i]) {
      const newVal = predicate(obj, args[i], key);
      if(newVal != undefined) obj[key] = newVal;
    }
  }
  return obj;
};
Util.getUserAgent = function(headers = req.headers) {
  const agent = useragent.parse(headers['user-agent']);
  return agent;
};
Util.factor = function(start, end) {
  let f = 1;
  for(let i = start; i <= end; i++) {
    f = f * i;
  }
  return f;
};
Util.factorial = function(n) {
  return Util.factor(1, n);
};
Util.increment = function(obj, key) {
  if(obj[key] >= 1) obj[key] == 0;
  obj[key]++;
  return obj[key];
};
Util.counter = function() {
  let i = 0;
  let self = function() {
    return i++;
  };
  return self;
};
Util.filterKeys = function(obj, pred = k => true) {
  let ret = {};
  if(pred instanceof RegExp) {
    let re = pred;
    pred = str => re.test(str);
  } else if(Util.isArray(pred)) {
    let a = pred;
    pred = str => a.indexOf(str) != -1;
  }
  for(let key in obj) if(pred(key, obj[key], obj)) ret[key] = obj[key];
  //Object.setPrototypeOf(ret, Object.getPrototypeOf(obj));
  return ret;
};
Util.filterMembers = function(obj, fn) {
  const pred = (k, v, o) => fn(v, k, o);
  return Util.filterKeys(obj, pred);
};
Util.filterOutMembers = function(obj, fn) {
  const pred = (v, k, o) => !fn(v, k, o);
  return Util.filterMembers(obj, pred);
};
Util.dumpMembers = obj => Util.filterOutMembers(obj, Util.isFunction);

Util.filterOutKeys = function(obj, arr) {
  if(typeof obj != 'object') return obj;
  const pred = typeof arr == 'function' ? (v, k, o) => arr(k, v, o) : arr instanceof RegExp ? (k, v) => arr.test(k) /*|| arr.test(v)*/ : Array.isArray(arr) ? key => arr.indexOf(key) != -1 : () => ({});
  return Util.filterOutMembers(obj, (v, k, o) => pred(k, v, o));
};
Util.removeKeys = function(obj, arr) {
  if(typeof obj != 'object') return obj;
  const pred = typeof arr == 'function' ? (v, k, o) => arr(k, v, o) : arr instanceof RegExp ? (k, v) => arr.test(k) /*|| arr.test(v)*/ : key => arr.indexOf(key) != -1;
  for(let key in obj) {
    if(pred(key, obj[key], obj)) delete obj[key];
  }
};
Util.getKeys = function(obj, arr) {
  let ret = {};
  for(let key of arr) ret[key] = obj[key];

  return ret;
};
Util.numbersConvert = function(str) {
  return str
    .split('')
    .map((ch, i) => (new RegExp('[ :,./]').test(ch) ? ch : String.fromCharCode((str.charCodeAt(i) & 0x0f) + 0x30)))
    .join('');
};
Util.entries = function(arg) {
  if(Util.isArray(arg) || Util.isObject(arg)) {
    if(typeof arg.entries == 'function') return arg.entries();
    else if(Util.isIterable(arg))
      return (function* () {
        for(let key in arg) yield [key, arg[key]];
      })();
    return Object.entries(arg);
  }
};
Util.keys = function(arg) {
  let ret;
  if(Util.isObject(arg)) {
    ret =
      typeof arg.keys == 'function'
        ? arg.keys
        : function* () {
            for(let key in this) yield key;
          };
  }
  if(ret) return ret.call(arg);
};
Util.values = function(arg) {
  let ret;
  if(Util.isObject(arg)) {
    ret =
      typeof arg.values == 'function'
        ? arg.values
        : function* () {
            for(let key in arg) yield arg[key];
          };
  }
  if(ret) return ret.call(arg);
};
Util.removeEqual = function(a, b) {
  let c = {};
  for(let key of Util.keys(a)) {
    if(b[key] === a[key]) continue;
    //console.log(`removeEqual '${a[key]}' === '${b[key]}'`);
    c[key] = a[key];
  }
  //console.log(`removeEqual`,c);

  return c;
};
Util.clear = obj => (typeof obj.splice == 'function' ? obj.splice(0, obj.length) : obj.clear());

Util.remove = (arr, item) => Util.removeIf(arr, (other, i, arr) => item === other);
Util.removeIf = function(arr, pred) {
  let count = 0;
  if(Util.isObject(arr) && typeof arr.splice == 'function') {
    let idx;
    for(count = 0; (idx = arr.findIndex(pred)) != -1; count++) arr.splice(idx, idx + 1);
  } else {
    for(let [key, value] of arr) {
      if(pred(value, key, arr)) {
        arr.delete(key);
        count++;
      }
    }
  }
  return count;
};
Util.traverse = function(o, fn) {
  if(typeof fn == 'function')
    return Util.foreach(o, (v, k, a) => {
      fn(v, k, a);
      if(typeof v === 'object') Util.traverse(v, fn);
    });
  function* walker(o, depth = 0) {
    for(let [k, v] of Util.entries(o)) {
      yield [v, k, o, depth];
      if(typeof v == 'object' && v !== null) yield* walker(v, depth + 1);
    }
  }
  return walker(o);
};
Util.traverseWithPath = function(o, rootPath = []) {
  for(let key of rootPath) o = o[key];

  function* walker(o, path) {
    for(let [k, v] of Util.entries(o)) {
      let p = [...path, k];
      yield [v, k, o, p];
      if(typeof v == 'object' && v !== null) yield* walker(v, p);
    }
  }

  return walker(o, []);
};
Util.indexByPath = function(o, p) {
  for(let key of p) o = o[key];
  return o;
};

Util.pushUnique = (arr, ...args) => args.reduce((acc, item) => (arr.indexOf(item) == -1 ? (arr.push(item), acc + 1) : acc), 0);

Util.insertSorted = function(arr, item, cmp = (a, b) => b - a) {
  let i = 0,
    len = arr.length;
  while(i < len) {
    if(cmp(item, arr[i]) >= 0) break;
    i++;
  }
  i < len ? arr.splice(i, 0, item) : arr.push(item);
  return i;
};
Util.inserter = (dest, next = (k, v) => {}) => {
  // if(typeof dest == 'function' && dest.map !== undefined) dest = dest.map;

  const insert =
    /*dest instanceof Map ||
    dest instanceof WeakMap ||*/
    typeof dest.set == 'function' && dest.set.length >= 2 ? (k, v) => dest.set(k, v) : Util.isArray(dest) ? (k, v) => dest.push([k, v]) : (k, v) => (dest[k] = v);
  let fn;
  fn = function(key, value) {
    insert(key, value);
    next(key, value);
    return fn;
  };
  fn.dest = dest;
  fn.insert = insert;
  return fn;
};

Util.keyIterator = obj => {
  let it;
  if(typeof obj.keys == 'function' && Util.isIterator((it = obj.keys()))) {
    return it;
  } else if(Util.isArray(obj)) {
    return Array.prototype.keys.call(obj);
  } else if('length' in obj) {
    return Array.prototype[Symbol.iterator].call(obj);
  }
};

Util.entryIterator = obj => {
  let it;
  if(typeof obj.entries == 'function' && Util.isIterator((it = obj.entries()))) {
    return it;
  } else if(Util.isArray(obj)) {
    return Array.prototype.entries.call(obj);
  } else if('length' in obj) {
    return (function* () {
      for(let key of Array.prototype[Symbol.iterator].call(obj)) yield [key, obj[key]];
    })();
  }
};

/*Util.bitIterator = function BitIterator(source, inBits, outBits) {
  const iterator = this instanceof BitIterator ? this : Object.create(BitIterator.prototype);

  iterator.bits = [0];
  iterator.size = 0;
  iterator.next = function(bitsWanted = outBits) {
    let output = { bits: [0], size: 0 };
    for(;;) {
      if(iterator.size == 0) fillBits(iterator);
      //console.log("iterator.bits =",iterator.bits, " iterator.size =",iterator.size);
      moveBits(iterator, output, bitsWanted);
      if(output.size == bitsWanted) break;
    }
    return output.bits;
  };
  function readBits(buffer, n) {
    n = Math.min(buffer.size, n);
    let size = 0,
      bits = [];
    while(n >= 16) {
      bits.push(buffer.bits.shift());
      buffer.size -= 16; n -= 16;
      size += 16;
    }

    if(n > 0) {
      const mask = (1 << n) - 1;
      bits.push(buffer.bits & mask);
      size += n;
      buffer.bits >>= n;
      buffer.size -= n;
    }
    return [bits, size];
  }
  const pad = '00000000000000000000000000000000';
  function writeBits(buffer, value, size) {
    buffer.bits = [...Util.partition((pad + value.toString(2)).slice(-32), 16)].map(n => parseInt(n, 2)).reverse();

    buffer.size = size;
    console.log("buffer.bits:",buffer.bits,"buffer.size:",buffer.size);
  }
  function moveBits(input, output, len) {
    let [bits, size] = readBits(input, len);
    writeBits(output, bits, size);
  }
  function fillBits(buffer) {
    const value = source();
    writeBits(buffer, value, inBits);
  }
  return iterator;
};
*/
Util.mapAdapter = getSetFunction => {
  let r = {
    get(key) {
      return getSetFunction(key);
    },
    set(key, value) {
      getSetFunction(key, value);
      return this;
    }
  };
  let tmp = getSetFunction();
  if(Util.isIterable(tmp) || Util.isPromise(tmp)) r.keys = () => getSetFunction();

  if(getSetFunction[Symbol.iterator]) r.entries = getSetFunction[Symbol.iterator];
  else {
    let g = getSetFunction();
    if(Util.isIterable(g) || Util.isGenerator(g)) r.entries = () => getSetFunction();
  }

  return Util.mapFunction(r);
};

/**
 * @param Array   forward
 * @param Array   backward
 *
 * component2path,  path2eagle  => component2eagle
 *  eagle2path, path2component =>
 */
Util.mapFunction = map => {
  let fn;
  fn = function(...args) {
    const [key, value] = args;
    switch (args.length) {
      case 0:
        return fn.keys();
      case 1:
        return fn.get(key);
      case 2:
        return fn.set(key, value);
    }
  };

  fn.map = (m => {
    while(Util.isFunction(m) && m.map !== undefined) m = m.map;
    return m;
  })(map);

  if(map instanceof Map || (Util.isObject(map) && typeof map.get == 'function' && typeof map.set == 'function')) {
    fn.set = (key, value) => (map.set(key, value), (k, v) => fn(k, v));
    fn.get = key => map.get(key);
  } else if(map instanceof Cache || (Util.isObject(map) && typeof map.match == 'function' && typeof map.put == 'function')) {
    fn.set = (key, value) => (map.put(key, value), (k, v) => fn(k, v));
    fn.get = key => map.match(key);
  } else if(Util.isObject(map) && typeof map.getItem == 'function' && typeof map.setItem == 'function') {
    fn.set = (key, value) => (map.setItem(key, value), (k, v) => fn(k, v));
    fn.get = key => map.getItem(key);
  } else {
    fn.set = (key, value) => ((map[key] = value), (k, v) => fn(k, v));
    fn.get = key => map[key];
  }

  fn.update = function(key, fn = (k, v) => v) {
    let oldValue = this.get(key);
    let newValue = fn(oldValue, key);
    if(oldValue != newValue) {
      if(newValue === undefined && typeof map.delete == 'function') map.delete(key);
      else this.set(key, newValue);
    }
    return newValue;
  };

  if(typeof map.entries == 'function') {
    fn.entries = function* () {
      for(let [key, value] of map.entries()) yield [key, value];
    };
    fn.values = function* () {
      for(let [key, value] of map.entries()) yield value;
    };
    fn.keys = function* () {
      for(let [key, value] of map.entries()) yield key;
    };
    fn[Symbol.iterator] = fn.entries;
    fn[inspectSymbol] = function() {
      return new Map(this.map(([key, value]) => [Util.isArray(key) ? key.join('.') : key, value]));
    };
  } else if(typeof map.keys == 'function') {
    if(Util.isAsync(map.keys) || Util.isPromise(map.keys())) {
      fn.keys = async () => [...(await map.keys())];

      fn.entries = async () => {
        let r = [];
        for(let key of await fn.keys()) r.push([key, await fn.get(key)]);
        return r;
      };
      fn.values = async () => {
        let r = [];
        for(let key of await fn.keys()) r.push(await fn.get(key));
        return r;
      };
    } else {
      fn.keys = function* () {
        for(let key of map.keys()) yield key;
      };

      fn.entries = function* () {
        for(let key of fn.keys()) yield [key, fn(key)];
      };
      fn.values = function* () {
        for(let key of fn.keys()) yield fn(key);
      };
    }
  }

  if(typeof fn.entries == 'function') {
    fn.filter = function(pred) {
      return Util.mapFunction(
        new Map(
          (function* () {
            let i = 0;
            for(let [key, value] of fn.entries()) if(pred([key, value], i++)) yield [key, value];
          })()
        )
      );
    };
    fn.map = function(t) {
      return Util.mapFunction(
        new Map(
          (function* () {
            let i = 0;

            for(let [key, value] of fn.entries()) yield t([key, value], i++);
          })()
        )
      );
    };
    fn.forEach = function(fn) {
      let i = 0;

      for(let [key, value] of this.entries()) fn([key, value], i++);
    };
  }
  if(typeof map.delete == 'function') fn.delete = key => map.delete(key);

  if(typeof map.has == 'function') fn.has = key => map.has(key);
  return fn;
};

Util.mapWrapper = (map, toKey = key => key, fromKey = key => key) => {
  let fn = Util.mapFunction(map);
  fn.set = (key, value) => (map.set(toKey(key), value), (k, v) => fn(k, v));
  fn.get = key => map.get(toKey(key));
  if(typeof map.keys == 'function') fn.keys = () => [...map.keys()].map(fromKey);
  if(typeof map.entries == 'function')
    fn.entries = function* () {
      for(let [key, value] of map.entries()) yield [fromKey(key), value];
    };
  if(typeof map.values == 'function')
    fn.values = function* () {
      for(let value of map.values()) yield value;
    };
  if(typeof map.has == 'function') fn.has = key => map.has(toKey(key));
  if(typeof map.delete == 'function') fn.delete = key => map.delete(toKey(key));

  fn.map = (m => {
    while(Util.isFunction(m) && m.map !== undefined) m = m.map;
    return m;
  })(map);

  return fn;
};

/**
 * @param Array   forward
 * @param Array   backward
 *
 * component2path,  path2eagle  => component2eagle
 *  eagle2path, path2component =>
 */
Util.mapCombinator = (forward, backward) => {
  let fn;
  fn = function(key, value) {
    if(value === undefined) return fn.get(key);
    return fn.set(key, value);
  };

  /* prettier-ignore */
  fn.get=  forward.reduceRight((a,m) => makeGetter(m, key => a(key)), a => a);
  return fn;
  function makeGetter(map, next = a => a) {
    return key => (false && console.log('getter', { map, key }), next(map.get(key)));
  }
};

Util.predicate = (fn_or_regex, pred) => {
  let fn = fn_or_regex;
  if(typeof fn_or_regex == 'string') fn_or_regex = new RegExp('^' + fn_or_regex + '$');
  if(fn_or_regex instanceof RegExp) {
    fn = arg => fn_or_regex.test(arg + '');
    fn.valueOf = function() {
      return fn_or_regex;
    };
  }
  if(typeof pred == 'function') return arg => pred(arg, fn);
  return fn;
};
Util.some = predicates => {
  predicates = predicates.map(Util.predicate);
  return value => predicates.some(pred => pred(value));
};
Util.every = predicates => {
  predicates = predicates.map(Util.predicate);
  return value => predicates.every(pred => pred(value));
};

Util.iterateMembers = function* (obj, predicate = (name, depth, obj, proto) => true, depth = 0) {
  let names = [];
  let pred = Util.predicate(predicate);
  const proto = Object.getPrototypeOf(obj);

  /* for(let name in obj) if(pred(name, depth, obj)) yield name;
   */
  let descriptors = Object.getOwnPropertyDescriptors(obj);
  for(let name in descriptors) {
    const { value, get, set, enumerable, configurable, writable } = descriptors[name];

    if(typeof get == 'function') continue;

    if(pred(name, depth, obj)) yield name;
  }
  //for(let symbol of Object.getOwnPropertySymbols(obj)) if(pred(symbol, depth, obj)) yield symbol;
  if(proto) yield* Util.iterateMembers(proto, predicate, depth + 1);
};

Util.and =
  (...predicates) =>
  (...args) =>
    predicates.every(pred => pred(...args));
Util.or =
  (...predicates) =>
  (...args) =>
    predicates.some(pred => pred(...args));

Util.members = Util.curry((pred, obj) => Util.unique([...Util.iterateMembers(obj, Util.tryPredicate(pred))]));

Util.memberNameFilter = (depth = 1, start = 0) =>
  Util.and(
    (m, l, o) => start <= l && l < depth + start,
    (m, l, o) => typeof m != 'string' || ['caller', 'callee', 'constructor', 'arguments'].indexOf(m) == -1,
    (name, depth, obj, proto) => obj != Object.prototype
  );

Util.getMemberNames = (obj, ...args) => {
  let filters = [];
  let depth = 1,
    start = 0;
  while(args.length > 0) {
    if(args.length >= 2 && typeof args[0] == 'number') {
      const n = args.splice(0, 2);
      depth = n[0];
      start = n[1];
      continue;
    }
    filters.push(args.shift());
  }
  filters.unshift(Util.memberNameFilter(depth, start));
  return Util.members(Util.and(...filters))(obj);
};
Util.getMemberEntries = (obj, ...args) => Util.getMemberNames(obj, ...args).map(name => [name, obj[name]]);

Util.objectReducer =
  (filterFn, accFn = (a, m, o) => ({ ...a, [m]: o[m] }), accu = {}) =>
  (obj, ...args) =>
    Util.members(filterFn(...args), obj).reduce(
      Util.tryFunction(
        (a, m) => accFn(a, m, obj),
        (r, a, m) => r,
        (r, a) => a
      ),
      accu
    );
Util.incrementer = (incFn = (c, n, self) => (self.count = c + n)) => {
  let self, incr;
  if(typeof incFn == 'number') {
    incr = incFn;
    incFn = (c, n, self) => (self.count = +c + +n * incr);
  }
  const inc = (i, n = 1) => self.incFn.call(self, i || 0, n, self);
  self = function Count(n = 1) {
    self.count = inc(self.count, n, self);
    return self;
  };
  self.incFn = incFn;
  self.valueOf = function() {
    return this.count;
  };
  return self;
};

Util.mapReducer = (setFn, filterFn = (key, value) => true, mapObj = new Map()) => {
  setFn = setFn || Util.setter(mapObj);
  let fn;
  let next = Util.tryFunction(((acc, mem, idx) => (filterFn(mem, idx) ? (setFn(idx, mem), acc) : null), r => r, () => mapObj));
  fn = function ReduceIntoMap(arg, acc = mapObj) {
    if(Util.isObject(arg) && typeof arg.reduce == 'function') return arg.reduce((acc, arg) => (Util.isArray(arg) ? arg : Util.members(arg)).reduce(reducer, acc), self.map);
    let c = Util.counter();
    for(let mem of arg) acc = next(acc, mem, c());
    return acc;
  };
  return Object.assign(fn, { setFn, filterFn, mapObj, next });
};

Util.getMembers = Util.objectReducer(Util.memberNameFilter);

Util.getMemberDescriptors = Util.objectReducer(Util.memberNameFilter, (a, m, o) => ({
  ...a,
  [m]: Object.getOwnPropertyDescriptor(o, m)
}));

Util.methodNameFilter = (depth = 1, start = 0) =>
  Util.and(
    (m, l, o) =>
      Util.tryCatch(
        () => typeof o[m] == 'function',
        b => b,
        () => false
      ),
    Util.memberNameFilter(depth, start)
  );

Util.getMethodNames = (obj, depth = 1, start = 0) => Util.members(Util.methodNameFilter(depth, start))(obj);

Util.getMethods = Util.objectReducer(Util.methodNameFilter);

Util.getMethodDescriptors = Util.objectReducer(Util.methodNameFilter, (a, m, o) => ({
  ...a,
  [m]: Object.getOwnPropertyDescriptor(o, m)
}));

Util.inherit = (dst, src, depth = 1) => {
  for(let k of Util.getMethodNames(src, depth)) dst[k] = src[k];
  return dst;
};
Util.inherits =
  typeof Object.create === 'function'
    ? function inherits(ctor, superCtor) {
        if(superCtor) {
          ctor.super_ = superCtor;
          ctor.prototype = Object.create(superCtor.prototype, {
            constructor: {
              value: ctor,
              enumerable: false,
              writable: true,
              configurable: true
            }
          });
        }
      } // old school shim for old browsers
    : function inherits(ctor, superCtor) {
        if(superCtor) {
          ctor.super_ = superCtor;
          let TempCtor = function() {};
          TempCtor.prototype = superCtor.prototype;
          ctor.prototype = new TempCtor();
          ctor.prototype.constructor = ctor;
        }
      };
//Util.bindMethods = (obj, methods, dest = {}) => Util.bindMethodsTo(obj, methods ?? obj, dest);
Util.bindMethods = (obj, methods, dest) => {
  dest ??= obj;
  if(Util.isArray(methods)) {
    for(let name of methods) if(typeof obj[name] == 'function') dest[name] = obj[name].bind(obj);
    return dest;
  }
  let names = Util.getMethodNames(methods);
  for(let name of names) if(typeof methods[name] == 'function') dest[name] = methods[name].bind(obj);
  return dest;
};
Util.getConstructor = obj => obj.constructor || Object.getPrototypeOf(obj).constructor;
Util.getPrototypeChain = function(obj, fn = p => p) {
  let ret = [];
  let proto;
  do {
    proto = obj.__proto__ || Object.getPrototypeOf(obj);
    ret.push(fn(proto, obj));
    if(proto === Object.prototype || proto.constructor === Object) break;
    obj = proto;
  } while(obj);

  return ret;
};
Util.getObjectChain = (obj, fn = p => p) => [fn(obj)].concat(Util.getPrototypeChain(obj, fn));

Util.getPropertyDescriptors = function(obj) {
  return Util.getObjectChain(obj, p => Object.getOwnPropertyDescriptors(p));
};

Util.getConstructorChain = (ctor, fn = (c, p) => c) => Util.getPrototypeChain(ctor, (p, o) => fn(o, p));

Util.weakAssign = function(...args) {
  let obj = args.shift();
  args.forEach(other => {
    for(let key in other) {
      if(obj[key] === undefined && other[key] !== undefined) obj[key] = other[key];
    }
  });
  return obj;
};

/*Util.getErrorStack = function(position = 2) {
  let stack=[];
  let error;
    Error.stackTraceLimit = 100;
     const oldPrepareStackTrace = Error.prepareStackTrace;
  Error.prepareStackTrace = (_, stack) => stack;
 try {

  throw new Error('my error');

 } catch(e) {
  console.log("e.stack",[...e.stack]);
  stack = e.stack;
 }
 Error.prepareStackTrace = oldPrepareStackTrace;

 return stack;
}*/
Util.exception = function Exception(...args) {
  let e, stack;
  let proto = Util.exception.prototype;

  if(args[0] instanceof Error) {
    let exc = args.shift();
    const { message, stack: callerStack } = exc;
    e = { message };
    //   e.proto = Object.getPrototypeOf(exc);

    if(callerStack) stack = callerStack;
  } else {
    const [message, callerStack] = args;
    e = { message };
    if(callerStack) stack = callerStack;
  }
  if(stack) e.stack = Util.stack(stack);

  return Object.setPrototypeOf(e, proto);
};

Util.define(
  Util.exception.prototype,
  {
    toString(color = false) {
      const { message, stack, proto } = this;
      return `${Util.fnName((proto && proto.constructor) || this.constructor)}: ${message}
Stack:${Util.stack.prototype.toString.call(stack, color, stack.columnWidths)}`;
    },
    [Symbol.toStringTag]() {
      return this.toString(false);
    },
    [inspectSymbol]() {
      return Util.exception.prototype.toString.call(this, true);
    }
  },
  true
);
Util.location = function Location(...args) {
  console.log('Util.location(', ...args, ')');
  let ret = this instanceof Util.location ? this : Object.setPrototypeOf({}, Util.location.prototype);
  if(args.length == 3) {
    const [fileName, lineNumber, columnNumber, functionName] = args;
    Object.assign(ret, { fileName, lineNumber, columnNumber, functionName });
  } else if(args.length == 1 && args[0].fileName !== undefined) {
    const { fileName, lineNumber, columnNumber, functionName } = args.shift();
    Object.assign(ret, { fileName, lineNumber, columnNumber, functionName });
  }
  if(Util.colorCtor) ret.colorCtor = Util.colorCtor;

  return ret;
};

// prettier-ignore
Util.location.palettes = [[[128, 128, 0], [255, 0, 255], [0, 255, 255] ], [[9, 119, 18], [139, 0, 255], [0, 165, 255]]];

Util.define(Util.location.prototype, {
  toString(color = false) {
    let { fileName, lineNumber, columnNumber, functionName } = this;
    console.log('this:', this, {
      fileName,
      lineNumber,
      columnNumber,
      functionName
    });
    fileName = fileName.replace(Util.makeURL({ location: '' }), '');
    let text = /*color ? new this.colorCtor() : */ '';
    const c = /*color ? (t, color) => text.write(t, color) :*/ t => (text += t);
    const palette = Util.location.palettes[Util.isBrowser() ? 1 : 0];
    if(functionName) c(functionName.replace(/\s*\[.*/g, '').replace(/^Function\./, '') + ' ', palette[1]);

    c(fileName, palette[0]);
    c(':', palette[1]);
    c(lineNumber, palette[2]);
    c(':', palette[1]);
    c(columnNumber, palette[2]);
    return text;
  },
  [Symbol.toStringTag]() {
    return Util.location.prototype.toString.call(this, false);
  },
  [inspectSymbol]() {
    return Util.location.prototype.toString.call(this, !Util.isBrowser());
  },
  getFileName() {
    return this.fileName;
  },
  getLineNumber() {
    return this.lineNumber;
  },
  getColumnNumber() {
    return this.columnNumber;
  }
});

Util.stackFrame = function StackFrame(frame) {
  //   console.debug('Util.stackFrame', frame, frame.getFunctionName, frame.getFileName);
  ['methodName', 'functionName', 'fileName', 'lineNumber', 'columnNumber', 'typeName', 'thisObj'].forEach(prop => {
    let fn = prop == 'thisObj' ? 'getThis' : 'get' + Util.ucfirst(prop);
    if(frame[prop] === undefined && typeof frame[fn] == 'function') frame[prop] = frame[fn]();
  });
  if(Util.colorCtor) frame.colorCtor = Util.colorCtor;

  return Object.setPrototypeOf(frame, Util.stackFrame.prototype);
};

Util.define(Util.stackFrame, {
  methodNames: ['getThis', 'getTypeName', 'getFunction', 'getFunctionName', 'getMethodName', 'getFileName', 'getLineNumber', 'getColumnNumber', 'getEvalOrigin', 'isToplevel', 'isEval', 'isNative', 'isConstructor', 'isAsync', 'isPromiseAll', 'getPromiseIndex']
});
Util.memoizedProperties(Util.stackFrame, {
  propertyMap() {
    return this.methodNames.map(method => [method, Util.lcfirst(method.replace(/^get/, ''))]).map(([method, func]) => [method, func == 'this' ? 'thisObj' : func]);
  }
});

Util.define(
  Util.stackFrame.prototype,
  {
    getFunction() {
      if(this.isConstructor) return this.functionName + '.constructor';

      return this.typeName ? `${this.typeName}.${this.methodName}` : this.functionName;
    },
    getMethodName() {
      return this.methodName;
    },
    getFunctionName() {
      return this.functionName;
    },
    getTypeName() {
      return this.typeName;
    },
    getFileName() {
      return this.fileName;
    },
    getLineNumber() {
      return this.lineNumber;
    },
    getColumnNumber() {
      return this.columnNumber;
    }
  },
  true
);

Util.define(
  Util.stackFrame.prototype,
  {
    colorCtor: null,
    get() {
      const { fileName, columnNumber, lineNumber } = this;
      return fileName ? `${fileName}:${lineNumber}:${columnNumber}` : null;
    },
    toString(color, opts = {}) {
      const { columnWidths = [0, 0, 0, 0], stripUrl } = opts;

      let text = color && this.colorCtor ? new this.colorCtor() : '';
      const c = color && this.colorCtor ? (t, color) => text.write(t, color) : t => (text += t);
      let fields = ['functionName', 'fileName', 'lineNumber', 'columnNumber'];
      const colors = [
        [0, 255, 0],
        [255, 255, 0],
        [0, 255, 255],
        [0, 255, 255]
      ];
      let { functionName, methodName, typeName, fileName, lineNumber, columnNumber } = this;
      //  console.log('toString:', { functionName, methodName, typeName, fileName, lineNumber, columnNumber });
      if(stripUrl && typeof fileName == 'string') fileName = fileName.replace(typeof stripUrl == 'string' ? stripUrl : /.*:\/\/[^\/]*\//, '');
      let colonList = [fileName, lineNumber, columnNumber]
        .map(p => ('' + p == 'undefined' ? undefined : p))
        .filter(p => p !== undefined && p != 'undefined' && ['number', 'string'].indexOf(typeof p) != -1)
        .join(':');
      let columns = [typeof this.getFunction == 'function' ? this.getFunction() : this.function, colonList];
      columns = columns.map((f, i) => (f + '')[i >= 2 ? 'padStart' : 'padEnd'](columnWidths[i] || 0, ' '));
      return columns.join(' ') + c('', 0);
    },
    getLocation() {
      return new Util.location(this);
    },
    /* prettier-ignore */ get location() {
      return this.getLocation();
    },
    [Symbol.toStringTag]() {
      return this.toString(false);
    },
    [inspectSymbol](...args) {
      return Util.stackFrame.prototype.toString.call(this, true, this.columnWidths);
    }
  },
  true
);
Util.scriptName = () =>
  Util.tryCatch(
    () => Util.getArgs(),
    args => args[0],
    () => Util.getURL()
  );
Util.getFunctionName = () => {
  const frame = Util.getCallerStack(2)[0];
  return frame.getFunctionName() || frame.getMethodName();
};
Util.getFunctionArguments = fn => {
  let head = (fn + '').replace(/(=>|{\n).*/g, '').replace(/^function\s*/, '');
  let args = head.replace(/^\((.*)\)\s*$/g, '$1').split(/,\s*/g);
  return args;
};

Util.scriptDir = () =>
  Util.tryCatch(
    () => Util.scriptName(),
    script => (script + '').replace(new RegExp('\\/[^/]*$', 'g'), ''),
    () => Util.getURL()
  );
Util.stack = function Stack(stack, offset) {
  //console.log('Util.stack (1)', stack);

  if(typeof stack == 'number') return Object.setPrototypeOf(new Array(stack), Util.stack.prototype);

  if(Util.platform == 'quickjs') {
    if(!stack) stack = getStack();
    if(!(typeof stack == 'string')) stack = stack + '';
  } else if(!stack) {
    if(offset === undefined) offset = 1;
    stack = getStack();
    const { propertyMap } = Util.stackFrame;
    //console.log('stack', stack + '');
    stack = [...stack].map(frame =>
      propertyMap
        .filter(([m, p]) => typeof frame[m] == 'function' && frame[m]() !== undefined)
        .reduce(
          (acc, [method, property]) => ({
            ...acc,
            /* prettier-ignore */ get [property]() {
              return frame[method]();
            }
          }),
          {}
        )
    );

    //console.debug('stack ctor:', [...stack]);
    //console.debug('stack frame[0]:', [...stack][0]);
  } else if(!(typeof stack == 'string')) stack = stackToString(stack, 0);
  function getStack() {
    let stack;
    const oldPrepareStackTrace = Error.prepareStackTrace;
    Error.prepareStackTrace = (_, stack) => stack;
    Error.stackTraceLimit = Infinity;

    stack = new Error().stack;
    Error.prepareStackTrace = oldPrepareStackTrace;
    return stack;
  }

  function stackToString(st, start = 0) {
    if(Util.isArray(st)) {
      st = [
        ...(function* () {
          for(let i = start; i < st.length; i++) yield st[i];
        })()
      ].join('\n');
    }
    return st;
  }

  //console.log('stack String:', offset, typeof stack, stack);

  if(typeof stack == 'number') {
    throw new Error();
  }
  //console.debug('stack:', typeof stack, stack);

  if(typeof stack == 'string') {
    stack = stack.trim().split(lineSplit);
    const re = new RegExp('.* at ([^ ][^ ]*) \\(([^)]*)\\)');
    stack = stack.map(frame =>
      typeof frame == 'string'
        ? frame
            .replace(/^\s*at\s+/, '')
            .split(/[()]+/g)
            .map(part => part.trim())
        : frame
    );
    stack = stack.map(frame => (Util.isArray(frame) ? (frame.length < 2 ? ['', ...frame] : frame).slice(0, 2) : frame));
    stack = stack.map(([func, file]) => [
      func,
      file
        .split(/:/g)
        .reverse()
        .map(n => (!isNaN(+n) ? +n : n))
    ]);
    stack = stack.map(([func, file]) => [func, file.length >= 3 ? file : file.length >= 2 ? ['', ...file] : ['', '', ...file]]);
    stack = stack.map(([func, [columnNumber, lineNumber, ...file]]) => ({
      functionName: func.replace(/Function\.Util/, 'Util'),
      methodName: func.replace(/.*\./, ''),
      fileName: file.reverse().join(':'),
      lineNumber,
      columnNumber
    }));
    //    console.log('Util.stack (2)', Util.inspect(stack[0]  ));

    stack = stack.map(({ methodName, functionName: func, fileName: file, columnNumber: column, lineNumber: line }) => ({
      functionName: func,
      methodName,
      fileName: file.replace(/.*:\/\/[^\/]*/g, ''),
      lineNumber: Util.ifThenElse(
        s => s != '',
        s => +s,
        () => undefined
      )(line + file.replace(/.*[^0-9]([0-9]*)$/g, '$1')),
      columnNumber: Util.ifThenElse(
        s => s != '',
        s => +s,
        () => undefined
      )(column)
    }));
  } else {
    //console.log('stack:', stack[0]);
    stack = stack.map(frame => new Util.stackFrame(frame)); //Util.getCallers(1, Number.MAX_SAFE_INTEGER, () => true, stack);
  }
  //  stack = stack.map(frame => Object.setPrototypeOf(frame, Util.stackFrame.prototype));
  stack = stack.map(frame => new Util.stackFrame(frame));

  if(offset > 0) stack = stack.slice(offset);
  stack = Object.setPrototypeOf(stack, Util.stack.prototype);
  //stack.forEach(frame => console.log("stack frame:",frame));
  //
  return stack;
};

Util.stack.prototype = Object.assign(Util.stack.prototype, Util.getMethods(new Array(), 1, 1));
Object.defineProperty(Util.stack, Symbol.species, { get: () => Util.stack });
Object.defineProperty(Util.stack.prototype, Symbol.species, {
  get: () => Util.stack
});
Object.defineProperty(Util.stack.prototype, Symbol.iterator, {
  *value() {
    for(let i = 0; i < this.length; i++) yield this[i];
  }
});

Util.stack.prototype = Object.assign(Util.stack.prototype, {
  toString(opts = {}) {
    const { colors = false, stripUrl = Util.makeURL({ location: '/' }) } = opts;
    const { columnWidths } = this;
    let a = [];

    for(let i = 0; i < this.length; i++)
      a.push(
        Util.stackFrame.prototype.toString.call(this[i], colors, {
          columnWidths,
          stripUrl
        })
      );
    let s = a.join('\n');
    return s + '\n';
  },
  [Symbol.toStringTag]() {
    return Util.stack.prototype.toString.call(this);
  },
  [inspectSymbol](...args) {
    const { columnWidths } = this;
    return '\n' + this.map(f => f.toString(!Util.isBrowser(), { columnWidths })).join('\n');
  },
  getFunctionName() {
    return this.functionName;
  },
  getMethodName() {
    return this.methodName;
  },
  getFileName() {
    return this.fileName;
  },
  getLineNumber() {
    return this.lineNumber;
  }
});

Object.defineProperties(Util.stack.prototype, {
  columnWidths: {
    get() {
      // console.log('this:', [...this]);
      return this.reduce((a, f) => ['getFunction'].map((fn, i) => Math.max(a[i], ((typeof f[fn] == 'function' ? f[fn]() : '') + '').length)), [0, 0, 0, 0]);
    }
  }
});

Util.getCallerStack = function(position = 2, limit = 1000, stack) {
  Error.stackTraceLimit = position + limit;
  if(position >= Error.stackTraceLimit) {
    throw new TypeError(`getCallerFile(position) requires position be less then Error.stackTraceLimit but position was: '${position}' and Error.stackTraceLimit was: '${Error.stackTraceLimit}'`);
  }
  const oldPrepareStackTrace = Error.prepareStackTrace;
  Error.prepareStackTrace = (_, stack) => stack;

  stack = Util.stack(stack, position);

  return stack.slice(0, limit);
};
Util.getCallerFile = function(position = 2) {
  let stack = Util.getCallerStack();
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[position];
    return frame ? `${frame.getFileName()}:${frame.getLineNumber()}` : undefined;
  }
};
Util.getCallerFunction = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[0];
    return frame ? frame.getFunction() : undefined;
  }
};
Util.getCallerFunctionName = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    const frame = stack[0];
    return frame ? frame.getMethodName() || frame.getFunctionName() : undefined;
  }
};
Util.getCallerFunctionNames = function(position = 2) {
  let stack = Util.getCallerStack(position + 1);
  if(stack !== null && typeof stack === 'object') {
    let ret = [];
    for(let i = 0; stack[i]; i++) {
      const frame = stack[i];
      const method = frame.getMethodName();
      ret.push(method ? frame.getFunction() + '.' + method : frame.getFunctionName());
    }
    return ret;
  }
};
Util.getCaller = function(index = 1, stack) {
  const methods = ['getThis', 'getTypeName', 'getFunction', 'getFunctionName', 'getMethodName', 'getFileName', 'getLineNumber', 'getColumnNumber', 'getEvalOrigin', 'isToplevel', 'isEval', 'isNative', 'isConstructor'];
  stack = stack || Util.getCallerStack(2, 1 + index, stack);
  let thisIndex = stack.findIndex(f => f.functionName.endsWith('getCaller'));
  index += thisIndex + 1;
  const frame = stack[index];
  return frame;
};
Util.getCallers = function(index = 1, num = Number.MAX_SAFE_INTEGER, stack) {
  const methods = ['getThis', 'getTypeName', 'getFunction', 'getFunctionName', 'getMethodName', 'getFileName', 'getLineNumber', 'getColumnNumber', 'getEvalOrigin', 'isToplevel', 'isEval', 'isNative', 'isConstructor'];
  stack = stack || Util.getCallerStack(2, num + index, stack);
  let thisIndex = stack.findIndex(f => ((f.functionName || f.methodName) + '').endsWith('getCaller'));
  index += thisIndex + 1;
  return stack.slice(index);
};

/*Object.defineProperty(Util, 'stackFrame', {
  get: function() {
  return this.getCallerStack(2);
  }
});*/
Util.getStackFrames = function(offset = 2) {
  let frames = Util.getCallerStack(0);
  frames = frames.map(frame => {
    if(Object.getPrototypeOf(frame) !== Util.stackFrame.prototype) frame = Util.stackFrame(frame);
    return frame;
  });

  return frames.slice(offset);
};
Util.getStackFrame = function(offset = 2) {
  return Util.getStackFrames(offset)[0];
};
Util.rotateLeft = function(x, n) {
  n = n & 0x1f;
  return (x << n) | ((x >> (32 - n)) & ~((-1 >> n) << n));
};
Util.rotateRight = function(x, n) {
  n = n & 0x1f;
  return Util.rotateLeft(x, 32 - n);
};
Util.hashString = function(string, bits = 32, mask = 0xffffffff) {
  let ret = 0;
  let bitc = 0;
  for(let i = 0; i < string.length; i++) {
    const code = string.charCodeAt(i);
    ret *= 186;
    ret ^= code;
    bitc += 8;
    ret = Util.rotateLeft(ret, 7) & mask;
  }
  return ret & 0x7fffffff;
};
Util.flatTree = function(tree, addOutput) {
  const ret = [];
  if(!addOutput) addOutput = arg => ret.push(arg);
  addOutput(Util.filterKeys(tree, key => key !== 'children'));
  if(typeof tree.children == 'object' && tree.children !== null && tree.children.length) for(let child of tree.children) Util.flatTree(child, addOutput);
  return ret;
};
Util.traverseTree = function(tree, fn, depth = 0, parent = null) {
  fn(tree, depth, parent);
  if(Util.isObject(tree.children) && tree.children.length > 0) for(let child of tree.children) Util.traverseTree(child, fn, depth + 1, tree);
};

Util.walkTree = function(node, pred, t, depth = 0, parent = null) {
  return (function* () {
    if(!pred) pred = i => true;
    if(!t)
      t = function(i) {
        i.depth = depth;
        return i;
      };
    if(pred(node, depth, parent)) {
      yield t(node);
      if(typeof node == 'object' && node !== null && typeof node.children == 'object' && node.children.length) {
        for(let child of [...node.children]) {
          yield* Util.walkTree(child, pred, t, depth + 1, node.parent_id);
        }
      }
    }
  })();
};

Util.isPromise = function(obj) {
  return (Boolean(obj) && typeof obj.then === 'function') || obj instanceof Promise;
};

/* eslint-disable no-use-before-define */
if(typeof setImmediate !== 'function') var setImmediate = fn => setTimeout(fn, 0);
Util.next = function(iter, observer, prev = undefined) {
  let item;
  try {
    item = iter.next(prev);
  } catch(err) {
    return observer.error(err);
  }
  const value = item.value;
  if(item.done) return observer.complete();
  if(isPromise(value)) {
    value
      .then(val => {
        observer.next(val);
        setImmediate(() => Util.next(iter, observer, val));
      })
      .catch(err => observer.error(err));
  } else {
    observer.next(value);
    setImmediate(() => Util.next(iter, observer, value));
  }
};
Util.getImageAverageColor = function(imageElement, options) {
  if(!imageElement) {
    return false;
  }
  options = options || {};
  const settings = {
    tooDark: (options.tooDark || 0.03) * 255 * 3 /* How dark is too dark for a pixel */,
    tooLight: (options.tooLight || 0.97) * 255 * 3 /*How light is too light for a pixel */,
    tooAlpha: (options.tooAlpha || 0.1) * 255 /*How transparent is too transparent for a pixel */
  };
  const w = imageElement.width;
  let h = imageElement.height;
  //Setup canvas and draw image onto it
  const context = document.createElement('canvas').getContext('2d');
  context.drawImage(imageElement, 0, 0, w, h);
  //Extract the rgba data for the image from the canvas
  const subpixels = context.getImageData(0, 0, w, h).data;
  const pixels = {
    r: 0,
    g: 0,
    b: 0,
    a: 0
  };
  let processedPixels = 0;
  const pixel = {
    r: 0,
    g: 0,
    b: 0,
    a: 0
  };
  let luma = 0; //Having luma in  the pixel object caused ~10% performance penalty for some reason
  //Loop through the rgba data
  for(let i = 0, l = w * h * 4; i < l; i += 4) {
    pixel.r = subpixels[i];
    pixel.g = subpixels[i + 1];
    pixel.b = subpixels[i + 2];
    pixel.a = subpixels[i + 4];
    //Only consider pixels that aren't black, white, or too transparent
    if(
      pixel.a > settings.tooAlpha &&
      (luma = pixel.r + pixel.g + pixel.b) > settings.tooDark && //Luma is assigned inside the conditional to avoid re-calculation when alpha is not met
      luma < settings.tooLight
    ) {
      pixels.r += pixel.r;
      pixels.g += pixel.g;
      pixels.b += pixel.b;
      pixels.a += pixel.a;
      processedPixels++;
    }
  }
  //Values of the channels that make up the average color
  let channels = {
    r: null,
    g: null,
    b: null,
    a: null
  };
  if(processedPixels > 0) {
    channels = {
      r: Math.round(pixels.r / processedPixels),
      g: Math.round(pixels.g / processedPixels),
      b: Math.round(pixels.b / processedPixels),
      a: Math.round(pixels.a / processedPixels)
    };
  }
  const o = Object.assign({}, channels, {
    toStringRgb() {
      //Returns a CSS compatible RGB string (e.g. '255, 255, 255')
      const { r, g, b } = this;
      return [r, g, b].join(', ');
    },
    toStringRgba() {
      //Returns a CSS compatible RGBA string (e.g. '255, 255, 255, 1.0')
      const { r, g, b, a } = this;
      return [r, g, b, a].join(', ');
    },
    toStringHex() {
      //Returns a CSS compatible HEX coloor string (e.g. 'FFA900')
      const toHex = function(d) {
        h = Math.round(d).toString(16);
        if(h.length < 2) {
          h = `0${h}`;
        }
        return h;
      };
      const { r, g, b } = this;
      return [toHex(r), toHex(g), toHex(b)].join('');
    }
  });
  return o;
};
Util.jsonToObject = function(jsonStr) {
  let ret = null;
  try {
    ret = JSON.parse(jsonStr);
  } catch(error) {
    let pos = +('' + error)
      .split('\n')
      .reverse()[0]
      .replace(/.*position\ ([0-9]+).*/, '$1');
    console.error('Unexpected token: ', jsonStr);
    console.error('Unexpected token at:', jsonStr.substring(pos));
    ret = null;
  }
  return ret;
};
Util.splitLines = function(str, max_linelen = Number.MAX_SAFE_INTEGER) {
  const tokens = str.split(/\s/g);
  let lines = [];
  let line = tokens.shift();
  for(; tokens.length; ) {
    if((line.length ? line.length + 1 : 0) + tokens[0].length > max_linelen) {
      lines.push(line);
      line = '';
    }
    if(line != '') line += ' ';
    line += tokens.shift();
  }
  if(line != '') lines.push(line);
  return lines;
};
Util.splitAt = function* (str, ...indexes) {
  let prev = 0;
  for(let index of indexes.sort((a, b) => a - b).concat([str.length])) {
    if(index >= prev) {
      yield str.substring(prev, index);
      if(index >= str.length) break;
      prev = index;
    }
  }
};
Util.decodeEscapes = function(text) {
  let matches = [...Util.matchAll(/([^\\]*)(\\u[0-9a-f]{4}|\\)/gi, text)];
  if(matches.length) {
    matches = matches.map(m => [...m].slice(1)).map(([s, t]) => s + String.fromCodePoint(parseInt(t.substring(2), 16)));
    text = matches.join('');
  }
  return text;
};

Util.stripXML = text =>
  text
    .replace(/<br(|\ *\/)>/gi, '\n')
    .replace(/<[^>]*>/g, '')
    .replace(/[\t\ ]+/g, ' ')
    .replace(/(\n[\t\ ]*)+\n/g, '\n');

Util.stripHTML = html =>
  html
    .replace(/\s*\n\s*/g, ' ')
    .replace(/<[^>]*>/g, '\n')
    .split(lineSplit)
    .map(p => p.trim())
    .filter(p => p != '');

Util.stripNonPrintable = text => text.replace(/[^\x20-\x7f\x0a\x0d\x09]/g, '');
Util.decodeHTMLEntities = function(text) {
  let entities = {
    amp: '&',
    apos: "'",
    '#x27': "'",
    '#x2F': '/',
    '#39': "'",
    '#47': '/',
    lt: '<',
    gt: '>',
    nbsp: ' ',
    quot: '"'
  };
  return text.replace(new RegExp('&([^;]+);', 'gm'), (match, entity) => entities[entity] || match);
};
Util.encodeHTMLEntities = (str, charset = '\u00A0-\u9999<>&') => str.replace(new RegExp(`[${charset}](?!#)`, 'gim'), i => '&#' + i.charCodeAt(0) + ';');

Util.stripAnsi = function(str) {
  return (str + '').replace(new RegExp('\x1b[[(?);]{0,2}(;?[0-9])*.', 'g'), '');
};
Util.proxy = (obj = {}, handler) =>
  new Proxy(obj, {
    get(target, key, receiver) {
      //console.log(`Util.proxy getting ${key}!`);
      return Reflect.get(target, key, receiver);
    },
    set(target, key, value, receiver) {
      //console.log(`Util.proxy setting ${key}!`);
      return Reflect.set(target, key, value, receiver);
    },
    ...handler
  });

Util.propertyLookup = (obj = {}, handler = key => null) =>
  Util.proxy(obj, {
    get(target, key, receiver) {
      return handler(key);
    }
  });

Util.traceProxy = (obj, handler) => {
  let proxy;
  handler = /*handler || */ function(name, args) {
    console.log(`Calling method '${name}':`, ...args);
  };
  //console.log('handler', { handler }, handler + '');
  proxy = new Proxy(obj, {
    get(target, key, receiver) {
      let member = Reflect.get(obj, key, receiver);
      if(0 && typeof member == 'function') {
        let method = member; // member.bind(obj);
        member = function() {
          //          handler.call(receiver, key, arguments);
          return method.apply(obj, arguments);
        };
        member = method.bind(obj);
        console.log('Util.traceProxy', key, (member + '').replace(/\n\s+/g, ' ').split(lineSplit)[0]);
      }
      return member;
    }
  });
  return proxy;
};

Util.proxyTree = function proxyTree(...callbacks) {
  const [setCallback, applyCallback = () => {}] = callbacks;
  const handler = {
    get(target, key) {
      return node([...this.path, key]);
    },
    set(target, key, value) {
      return setCallback(this.path, key, value);
    },
    apply(target, thisArg, args) {
      return applyCallback(this.path, ...args);
    }
  };
  function node(path) {
    return new Proxy(() => {}, { path, ...handler });
  }

  return node([]);
};

/*
 * Calls a constructor with an arbitrary number of arguments.
 *
 * This idea was borrowed from a StackOverflow answer:
 * http://stackoverflow.com/questions/1606797/use-of-apply-with-new-operator-is-this-possible/1608546#1608546
 *
 * And from this MDN doc:
 * https://developer.mozilla.org/en/JavaScript/Reference/Global_Objects/function/apply
 *
 * @param constructor- Constructor to call
 * @param arguments- any number of arguments
 * @return A 'new' instance of the constructor with the arguments passed
 */
Util.construct = constructor => {
  function F(args) {
    return constructor.apply(this, args);
  }

  F.prototype = constructor.prototype;

  // since arguments isn't a first-class array, we'll use a shim
  // Big thanks to Felix Geisendörfer for the idea:
  // http://debuggable.com/posts/turning-javascript-s-arguments-object-into-an-array:4ac50ef8-3bd0-4a2d-8c2e-535ccbdd56cb
  return new F(Array.prototype.slice.call(arguments, 1));
};

/*
 * Calls construct() with a constructor and an array of arguments.
 *
 * @param constructor- Constructor to call
 * @param array- an array of arguments to apply
 * @return A 'new' instance of the constructor with the arguments passed
 */
Util.constructApply = (constructor, array) => {
  let args = [].slice.call(array);
  return construct.apply(null, [constructor].concat(args));
};

Util.immutable = args => {
  const argsType = typeof args === 'object' && Util.isArray(args) ? 'array' : 'object';
  const errorText = argsType === 'array' ? "Error! You can't change elements of this array" : "Error! You can't change properties of this object";
  const handler = {
    set: () => {
      throw new Error(errorText);
    },
    deleteProperty: () => {
      throw new Error(errorText);
    },
    defineProperty: () => {
      throw new Error(errorText);
    }
  };
  return new Proxy(args, handler);
};

Util.immutableClass = (orig, ...proto) => {
  let name = Util.fnName(orig).replace(/Mutable/g, '');
  let imName = 'Immutable' + name;
  proto = proto || [];
  let initialProto = proto.map(p =>
    Util.isArrowFunction(p)
      ? p
      : ctor => {
          for(let n in p) ctor.prototype[n] = p[n];
        }
  );
  let body = `class ${imName} extends ${name} {\n  constructor(...args) {\n    super(...args);\n    if(new.target === ${imName})\n      return Object.freeze(this);\n  }\n};\n\n${imName}.prototype.constructor = ${imName};\n\nreturn ${imName};`;
  for(let p of initialProto) p(orig);
  let ctor; // = new Function(name, body)(orig);

  let imm = base => {
    let cls;
    cls = class extends base {
      constructor(...args) {
        super(...args);
        if(new.target === cls) return Object.freeze(this);
      }
    };
    return cls;
  };
  ctor = imm(orig);

  //console.log('immutableClass', { initialProto, body }, orig);
  let species = ctor;

  /* prettier-ignore */ //Object.assign(ctor, { [Symbol.species]: ctor });

  return ctor;
};

Util.partial = function partial(fn /*, arg1, arg2 etc */) {
  let partialArgs = [].slice.call(arguments, 1);
  if(!partialArgs.length) {
    return fn;
  }
  return function() {
    let args = [].slice.call(arguments);
    let derivedArgs = [];
    for(let i = 0; i < partialArgs.length; i++) {
      let thisPartialArg = partialArgs[i];
      derivedArgs[i] = thisPartialArg === undefined ? args.shift() : thisPartialArg;
    }
    return fn.apply(this, derivedArgs.concat(args));
  };
};

Util.clamp = Util.curry((min, max, value) => Math.max(min, Math.min(max, value)));

Util.coloring = (useColor = true) =>
  !useColor
    ? {
        code(...args) {
          return '';
        },
        text(text) {
          return text;
        },
        concat(...args) {
          let out = args.shift() || [''];
          if(typeof out == 'string') out = [out];
          for(let arg of args) {
            if(Util.isArray(arg)) {
              for(let subarg of arg) out[0] += subarg;
            } else out[0] += arg;
          }
          return out;
        }
      }
    : Util.isBrowser()
    ? {
        palette: ['rgb(0,0,0)', 'rgb(80,0,0)', 'rgb(0,80,0)', 'rgb(80,80,0)', 'rgb(0,0,80)', 'rgb(80,0,80)', 'rgb(0,80,80)', 'rgb(80,80,80)', 'rgb(0,0,0)', 'rgb(160,0,0)', 'rgb(0,160,0)', 'rgb(160,160,0)', 'rgb(0,0,160)', 'rgb(160,0,160)', 'rgb(0,160,160)', 'rgb(160,160,160)'],
        /*Util.range(0, 15).map(i =>
            `rgb(${Util.range(0, 2)
              .map(bitno => Util.getBit(i, bitno) * (i & 0x08 ? 160 : 80))
              .join(',')})`
        )*/ code(...args) {
          let css = '';
          let bold = 0;
          for(let arg of args) {
            let c = (arg % 10) + bold;
            let rgb = this.palette[c];
            //console.realLog("code:", {arg, c, rgb});
            if(arg >= 40) css += `background-color:${rgb};`;
            else if(arg >= 30) css += `color:${rgb};`;
            else if(arg == 1) bold = 8;
            else if(arg == 0) bold = 0;
            else throw new Error('No such color code:' + arg);
          }
          css += 'padding: 2px 0 2px 0;';
          return css;
        },
        text(text, ...color) {
          return [`%c${text}`, this.code(...color)];
        },
        concat(...args) {
          let out = args.shift() || [''];
          for(let arg of args) {
            if(Util.isArray(arg) && typeof arg[0] == 'string') out[0] += arg.shift();
            else if(Util.isObject(arg)) {
              out.push(arg);
              continue;
            }

            out = out.concat(arg);
          }
          return out;
        }
      }
    : {
        code(...args) {
          return `\x1b[${[...args].join(';')}m`;
        },
        text(text, ...color) {
          return this.code(...color) + text + this.code(0);
        },
        concat(...args) {
          return args.join('');
        }
      };

let color;
Util.colorText = (...args) => {
  if(!color) color = Util.coloring();
  return color.text(...args);
};
Util.decodeAnsi = (str, index) => {
  let ret = [];
  const len = str.length;
  if(index === undefined) index = str.lastIndexOf('\x1b');
  const isDigit = c => '0123456789'.indexOf(c) != -1;
  const notDigit = c => !isDigit(c);
  const findIndex = (pred, start) => {
    let i;
    for(i = start; i < len; i++) if(pred(str[i])) break;
    return i;
  };
  if(str[++index] == '[') {
    let newIndex;
    for(++index; index < len; index = newIndex) {
      let isNum = isDigit(str[index]);
      newIndex = isNum ? findIndex(notDigit, index) : index + 1;
      if(isNum) {
        let num = parseInt(str.substring(index, newIndex));
        ret.push(num);
      } else {
        ret.push(str[index]);
        break;
      }
      if(str[newIndex] == ';') newIndex++;
    }
  }
  return ret;
};
Util.stripAnsi = str => {
  let o = '';
  for(let i = 0; i < str.length; i++) {
    if(str[i] == '\x1b' && str[i + 1] == '[') {
      while(!/[A-Za-z]/.test(str[i])) i++;
      continue;
    }
    o += str[i];
  }
  return o;
};

Util.ansiCode = (...args) => {
  if(!color) color = Util.coloring();
  return color.code(...args);
};
Util.ansi = Util.coloring(true);
Util.wordWrap = (str, width, delimiter) => {
  // use this on single lines of text only
  if(str.length > width) {
    let p = width;
    for(; p > 0 && str[p] != ' '; p--) {}
    if(p > 0) {
      let left = str.substring(0, p);
      let right = str.substring(p + 1);
      return left + delimiter + Util.wordWrap(right, width, delimiter);
    }
  }
  return str;
};
Util.multiParagraphWordWrap = (str, width, delimiter) => {
  // use this on multi-paragraph lines of xcltext
  let arr = str.split(delimiter);
  for(let i = 0; i < arr.length; i++) if(arr[i].length > width) arr[i] = Util.wordWrap(arr[i], width, delimiter);
  return arr.join(delimiter);
};
Util.defineInspect = (proto, ...props) => {
  if(!Util.isBrowser()) {
    const c = Util.coloring();
    proto[inspectSymbol] = function() {
      const obj = this;
      return (
        c.text(Util.fnName(proto.constructor) + ' ', 1, 31) +
        Util.inspect(
          props.reduce((acc, key) => {
            acc[key] = obj[key];
            return acc;
          }, {}),
          {
            multiline: false,
            colors: true,
            colon: ':',
            spacing: '',
            separator: ', ',
            padding: ' '
          }
        )
      );
    };
  }
};

Util.inRange = Util.curry((a, b, value) => value >= a && value <= b);

Util.bindProperties = (proxy, target, props, gen) => {
  if(props instanceof Array) props = Object.fromEntries(props.map(name => [name, name]));
  const [propMap, propNames] = Util.isArray(props) ? [props.reduce((acc, name) => ({ ...acc, [name]: name }), {}), props] : [props, Object.keys(props)];

  if(!gen) gen = p => v => v === undefined ? target[propMap[p]] : (target[propMap[p]] = v);
  const propGetSet = propNames
    .map(k => [k, propMap[k]])

    .reduce(
      (a, [k, v]) => ({
        ...a,
        [k]: Util.isFunction(v) ? (...args) => v.call(target, k, ...args) : (gen && gen(k)) || ((...args) => (args.length > 0 ? (target[k] = args[0]) : target[k]))
      }),
      {}
    );

  /*  console.log(`Util.bindProperties`, { proxy, target, props, gen });*/
  //console.log(`Util.bindProperties`, { propMap, propNames, propGetSet });
  Object.defineProperties(
    proxy,
    propNames.reduce(
      (a, k) => {
        const prop = props[k];
        const get_set = propGetSet[k]; //typeof prop == 'function' ? prop : gen(prop);
        return {
          ...a,
          [k]: {
            get: get_set,
            set: get_set,
            enumerable: true
          }
        };
      },
      {
        __getter_setter__: { get: () => gen, enumerable: false },
        __bound_target__: { get: () => target, enumerable: false }
      }
    )
  );
  return proxy;
};

Util.weakKey = (function () {
  const map = new WeakMap();
  let index = 0;
  return obj => {
    let key = map.get(obj);
    if(!key) {
      key = 'weak-key-' + index++;
      map.set(obj, key);
    }
    return key;
  };
})();

Object.assign(Util.is, {
  array: Util.isArray,
  bool: Util.isBool,
  constructor: Util.isConstructor,
  date: Util.isDate,
  email: Util.isEmail,
  empty: Util.isEmpty,
  nonEmpty: Util.isNonEmpty,
  emptyString: Util.isEmptyString,
  generator: Util.isGenerator,
  iterable: Util.isIterable,
  map: Util.isMap,
  nativeFunction: Util.isNativeFunction,
  object: Util.isObject,
  promise: Util.isPromise,
  function: Util.isFunction,
  string: Util.isString,
  on: val => val == 'on' || val == 'yes' || val === 'true' || val === true,
  off: val => val == 'off' || val == 'no' || val === 'false' || val === false,
  true: val => val === 'true' || val === true,
  false: val => val === 'false' || val === false
});

class AssertionFailed extends Error {
  constructor(message, stack) {
    super(/*'@ ' + location + ': ' +*/ message);
    //this.location = location;
    this.type = 'Assertion failed';

    stack = stack || this.stack;

    this.stack = stack;
  }
}

Util.assert = function assert(val, message) {
  if(typeof val == 'function') {
    message = message || val + '';
    val = val();
  }
  if(!val) throw new AssertionFailed(message || `val == ${val}`);
};
Util.assertEqual = function assertEqual(val1, val2, message) {
  if(val1 != val2) throw new AssertionFailed(message || `${val1} != ${val2}`);
};

Util.assignGlobal = () => Util.weakAssign(Util.getGlobalObject(), Util);

Util.weakMapper = function(createFn, map = new WeakMap(), hitFn) {
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

Util.merge = function(...args) {
  let ret;
  let isMap = args[0] instanceof Map;
  let t = isMap ? a => new Map(Object.entries(a)) : a => a;

  if(isMap) {
    /*  if(!args.every(arg => Util.isObject(arg) && arg instanceof Map))
    args =args.map(arg => new Map(Util.entries(arg)));
*/
    ret = new Map();

    for(let arg of args) for (let [key, value] of Util.entries(arg)) ret.set(key, value);
  } else {
    ret = args.reduce((acc, arg) => ({ ...acc, ...arg }), {});
  }

  return ret;
};

Util.transformer = (a, ...l) =>
  (l || []).reduce(
    (c, f) =>
      function(...v) {
        return f.apply(this, [c.apply(this, v), ...v]);
      },
    a
  );

/* XXX Util.copyTextToClipboard = (i, t) => {
  if(!Util.isBrowser()) {
    return import('./childProcess.js').then(async module => {
      let fs, std;
      let childProcess = await module.PortableChildProcess((a, b, c) => {
        fs = b;
        std = c;
      });
      console.log('childProcess', { childProcess, fs, std });
      let proc = childProcess('xclip', ['-in'], {
        block: false,
        stdio: ['pipe'],
        env: { DISPLAY: Util.getEnv('DISPLAY') }
      });
      console.log('proc.stdin', proc.stdin);

      console.log('write =', await fs.write(proc.stdin, i));
      await fs.close(proc.stdin);
      return await proc.wait();
    });
  }
  let doc = Util.tryCatch(() => document);
  if(!doc) return;
  if(!t) t = doc.body;
  const e = doc.createElement('textarea');
  const prev = doc.activeElement;
  e.value = i;
  e.setAttribute('readonly', '');
  e.style.contain = 'strict';
  e.style.position = 'absolute';
  e.style.left = '-9999px';
  e.style.fontSize = '12pt';
  const s = doc.getSelection();
  let orig = false;
  if(s.rangeCount > 0) {
    orig = s.getRangeAt(0);
  }
  t.append(e);
  e.select();
  e.selectionStart = 0;
  e.selectionEnd = i.length;
  let isSuccess = false;
  try {
    isSuccess = doc.execCommand('copy');
  } catch(_) {}
  e.remove();
  if(orig) {
    s.removeAllRanges();
    s.addRange(orig);
  }
  if(prev) {
    prev.focus();
  }
  return isSuccess;
};*/

Util.toPlainObject = obj => Util.toPlainObjectT(obj, v => (Util.isObject(v) ? Util.toPlainObject(v) : v));

Util.toBuiltinObject = obj => (Array.isArray(obj) ? obj.map(Util.toBuiltinObject) : Util.toPlainObjectT(obj, v => (Util.isObject(v) ? Util.toBuiltinObject(v) : v)));

Util.toPlainObjectT = (obj, t = (v, n) => v) => [...Object.getOwnPropertyNames(obj)].reduce((acc, k) => ({ ...acc, [k]: t(obj[k], k) }), {});

Util.timer = msecs => {
  let ret, id, rej, createdTime, startTime, stopTime, endTime, res, delay, n, timer;
  createdTime = new Date();
  const remaining = () => {
    let r = startTime + msecs - (typeof stopTime == 'number' ? stopTime : new Date());
    return r >= 0 ? r : 0;
  };
  const finish = callback => {
    stopTime = new Date();
    if(stopTime.valueOf() > endTime.valueOf()) stopTime = endTime;
    if(typeof callback == 'function') callback(stopTime);
    res((n = remaining()));
  };
  const log = (method, ...args) => console.log(`${Date.now() - createdTime.valueOf()} timer#${id}.${method}`, ...args.map(obj => Util.toPlainObject(obj || {}, v => v || (v instanceof Date ? `+${v.valueOf() - createdTime}` : v))));
  const timeout = (msecs, tmr = timer) => {
    let now = Date.now();
    if(!startTime) startTime = new Date(now);
    endTime = new Date(now + msecs);
    stopTime = undefined;
    id = setTimeout(() => {
      finish(typeof tmr.callback == 'function' ? (...args) => tmr.callback(...args) : () => {});
      log(`finish`, tmr);
    }, msecs);
    log('start', tmr);
  };
  const add = (arr, ...items) => [...(arr ? arr : []), ...items];

  timer = {
    subscribers: [],
    /* prettier-ignore */ get delay() {
      return delay;
    },
    /* prettier-ignore */ get created() {
      return createdTime;
    },
    /* prettier-ignore */ get start() {
      return startTime || new Date(endTime.valueOf() - delay);
    },
    /* prettier-ignore */ get stop() {
      return stopTime instanceof Date ? stopTime : undefined;
    },
    /* prettier-ignore */ get elapsed() {
      return delay + (stopTime || new Date()).valueOf() - endTime.valueOf();
    },
    /* prettier-ignore */ get end() {
      return endTime;
    },
    /* prettier-ignore */ get remain() {
      return endTime.valueOf() - (stopTime || new Date()).valueOf();
    },
    cancel() {
      log('cancel', this);
      clearTimeout(id);
      finish();
      return this;
    },
    pause() {
      let { remain, pause } = this;
      stopTime = new Date();
      clearTimeout(id);
      this.resume = function() {
        timeout(remain, this);
        this.pause = pause;
        delete this.resume;
        delete this.restart;
        log('resume', this);
        return this;
      };
      this.restart = function() {
        timeout(delay, this);
        this.pause = pause;
        delete this.resume;
        delete this.restart;
        log('restart', this);
        return this;
      };
      delete this.pause;
      log('pause', this);
      return this;
    },
    callback(...args) {
      log('callback', this);
      const { subscribers } = this;
      for(let f of subscribers) f.call(this, ...args);
      return this;
    },
    subscribe(f) {
      const { subscribers } = this;
      if(subscribers.indexOf(f) == -1) subscribers.push(f);
      return this;
    },
    unsubscribe(f) {
      const { subscribers } = this;
      let idx = subscribers.indexOf(f);
      if(idx != -1) subscribers.splice(idx, idx + 1);
      return this;
    }
  };
  const start = () =>
    new Promise((resolve, reject) => {
      res = resolve;
      rej = reject;
      timeout((delay = msecs));
    });
  ret = start();
  return Util.define(ret, timer);
};
/**
 * ???????????''
 * new Promise(Util.thenableReject('ERROR').then)
 *
 * @param      {<type>}  error   The error
 */
Util.thenableReject = error => ({
  then: (resolve, reject) => reject(error)
});
Util.wrapGenerator = fn =>
  Util.isGenerator(fn)
    ? function(...args) {
        return [...fn.call(this, ...args)];
      }
    : fn;

Util.wrapGeneratorMethods = obj => {
  for(let name of Util.getMethodNames(obj, 1, 0)) obj[name] = Util.wrapGenerator(obj[name]);
  return obj;
};

Util.decorateIterable = (proto, generators = false) => {
  const methods = {
    forEach(fn, thisArg) {
      for(let [i, item] of this.entries()) fn.call(thisArg, item, i, this);
    },
    *map(fn, thisArg) {
      for(let [i, item] of this.entries()) yield fn.call(thisArg, item, i, this);
    },
    *filter(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred.call(thisArg, item, i, this)) yield item;
    },
    findIndex(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred(item, i, this)) return i;
      return -1;
    },
    indexOf(item, startIndex = -1) {
      return this.findIndex((e, i) => i >= startIndex && e == item);
    },
    find(pred, thisArg) {
      let idx = this.findIndex(pred, thisArg);
      if(idx != -1) return typeof this.item == 'function' ? this.item(idx) : this[idx];
    },
    every(pred, thisArg) {
      for(let [i, item] of this.entries()) if(!pred(item, i++, this)) return false;
      return true;
    },
    some(pred, thisArg) {
      for(let [i, item] of this.entries()) if(pred(item, i, this)) return true;
      return false;
    },
    reduce(fn, accu) {
      for(let [i, item] of this.entries()) accu = fn(accu, item, i, this);
      return accu;
    },
    *entries() {
      let i = 0;
      for(let item of this) yield [i++, item];
    },
    *keys() {
      for(let [i, item] of this.entries()) yield i;
    },
    *values() {
      for(let [i, item] of this.entries()) yield item;
    }
  };
  Util.define(proto, methods, false);
  if(!generators) {
    for(let name in methods) {
      if(typeof name == 'symbol') continue;
      if(name == 'entries') continue;
      let gen = proto[name];
      proto[name] = Util.wrapGenerator(gen);
    }
  }

  return proto;
};

Util.swap = (a, b) => [b, a];
Util.swapArray = ([a, b]) => [b, a];

Util.cacheAdapter = (st, defaultOpts = {}) => {
  if(typeof st == 'string')
    st = Util.tryCatch(
      () => window.caches,
      async c => c.open(st),
      () => null
    );
  return {
    async getItem(request, opts = {}) {
      if(typeof request == 'number') request = await this.key(request);
      return await (await st).match(request, { ...defaultOpts, ...opts });
    },
    async setItem(request, response) {
      return await (await st).put(request, response);
    },
    async addItem(request) {
      await (await st).add(request);
      let response = await this.getItem(request);
      if(response) response = response.clone();
      return response;
    },
    async removeItem(request, opts = {}) {
      if(typeof request == 'number') request = await this.key(request);
      return await (await st).delete(request, { ...defaultOpts, ...opts });
    },
    async key(index) {
      return (await (await st).keys())[index];
    },
    async keys(urls = false, t = a => a) {
      let keys = await (await st).keys();
      if(urls) keys = keys.map(response => response.url);
      if(typeof t == 'function') keys = keys.map(r => t(r));

      return keys;
    },
    async clear() {
      let keys = await (await st).keys();
      for(let key of keys) await this.removeItem(key);
    }
  };
};
Util.cachedFetch = (allOpts = {}) => {
  let { cache = 'fetch', fetch = Util.getGlobalObject('fetch'), debug, print, ...opts } = allOpts;
  const storage = Util.cacheAdapter(cache);
  const baseURL = Util.memoize(() => Util.makeURL({ location: '' }));

  let self = async function CachedFetch(request, opts = {}) {
    let response;
    try {
      if(typeof request == 'string') request = new Request(request, { ...self.defaultOpts, ...opts });

      if(!request.url.startsWith(baseURL())) {
        request = new Request(request.url, { ...request, mode: 'no-cors' });
      }
      response = await storage.getItem(request, {
        ...self.defaultOpts,
        ...opts
      });

      if(response == undefined) {
        response = await /*self.*/ fetch(request, {
          ...self.defaultOpts,
          ...opts
        });

        if(response) {
          let item = response.clone();
          item.time = new Date();
          storage.setItem(request, item);
        }
      } else {
        response.cached = true;
      }
    } catch(err) {
      throw new Error(`CachedFetch: ` + (request.url || request) + ' ' + err.message);
    }
    return response;
  };
  if(debug)
    self = Util.printReturnValue(self, {
      print: print || ((returnValue, fn, ...args) => console.debug(`cachedFetch[${cache}] (`, ...args, `) =`, returnValue))
    });

  Util.define(self, { fetch, cache, storage, opts });
  return self;
};

Util.proxyObject = (root, handler) => {
  const ptr = path => path.reduce((a, i) => a[i], root);
  const nodes = Util.weakMapper(
    (value, path) =>
      new Proxy(value, {
        get(target, key) {
          let prop = value[key];
          if(Util.isObject(prop) || Util.isArray(prop)) return new node([...path, key]);
          return handler && handler.get ? handler.get(prop, key) : prop;
        }
      })
  );
  function node(path) {
    let value = ptr(path);
    //console.log("node:",{path,value});
    return nodes(value, path);
  }
  return node([]);
};
Util.parseXML = function(xmlStr) {
  return Util.tryCatch(
    () => new DOM(),
    parser => parser.parseFromString(xmlStr, 'application/xml')
  );
};

Util.weakAssoc = (fn = (value, ...args) => Object.assign(value, ...args)) => {
  let mapper = Util.tryCatch(
    () => new WeakMap(),
    map => Util.weakMapper((obj, ...args) => Util.merge(...args), map),
    () =>
      (obj, ...args) =>
        Util.define(obj, ...args)
  );
  let self = (obj, ...args) => {
    let value = mapper(obj, ...args);
    return fn(value, ...args);
  };
  self.mapper = mapper;

  return self;
};
Util.getArgv = Util.memoize(() =>
  Util.tryCatch(
    () => {
      let a = process.argv;
      if(!Util.isArray(a)) throw new Error();
      return a;
    },
    a => a,
    () =>
      Util.tryCatch(
        () => thisFilename(),
        fn => [fn],
        () =>
          Util.tryCatch(
            () => scriptArgs,
            a => ['qjs', ...a]
          )
      )
  )
);
Util.getArgs = Util.memoize(() =>
  Util.tryCatch(
    () => {
      let a = process.argv;
      if(!Util.isArray(a)) throw new Error();
      return a;
    },
    a => a.slice(1),
    () => Util.tryCatch(() => scriptArgs)
  )
);
/*  options Object/Map

    option Array [has_arg,callback,val]

*/
Util.getOpt = (options = {}, args) => {
  let short, long;
  let result = {};
  let positional = (result['@'] = []);
  if(!(options instanceof Array)) options = Object.entries(options);
  const findOpt = arg => options.find(([optname, option]) => (Array.isArray(option) ? option.indexOf(arg) != -1 : false) || arg == optname);
  let [, params] = options.find(opt => opt[0] == '@') || [];
  if(typeof params == 'string') params = params.split(',');
  // console.log('Util.getOpt options', options);
  // console.log('Util.getOpt params', params);
  for(let i = 0; i < args.length; i++) {
    const arg = args[i];
    let opt;
    if(arg[0] == '-') {
      let name, value, start, end;
      if(arg[1] == '-') long = true;
      else short = true;
      //console.log('Util.getOpt', { arg, short, long });
      start = short ? 1 : 2;
      if(short) end = 2;
      else if((end = arg.indexOf('=')) == -1) end = arg.length;
      name = arg.substring(start, end);
      //console.log('Util.getOpt', { start, end, name });
      if((opt = findOpt(name))) {
        //console.log('Util.getOpt', { opt });
        const [has_arg, handler] = opt[1];
        if(has_arg) {
          if(arg.length > end) value = arg.substring(end + (arg[end] == '='));
          else value = args[++i];
        } else {
          value = true;
        }
        //console.log('Util.getOpt #1', { name, handler });
        Util.tryCatch(
          () => handler(value, result[opt[0]], options, result),
          v => (value = v),
          () => null
        );
        //console.log('Util.getOpt #2', { name, value, fn: typeof opt[1] + ' ' + opt[1] + '' });
        result[opt[0]] = value;
        continue;
      }
    }
    if(params.length) {
      const param = params.shift();
      // console.log('Util.getOpt', { positional, param });
      if((opt = findOpt(param))) {
        const [, [, handler]] = opt;
        let value = arg;
        //console.log('Util.getOpt #3', { param, handler });
        if(typeof handler == 'function')
          value = Util.tryCatch(
            () => handler(value, result[opt[0]], options, result),
            v => v
          );
        const name = opt[0];
        //console.log('Util.getOpt #4', { name, value });
        result[opt[0]] = value;
        continue;
      }
    }
    result['@'] = [...(result['@'] ?? []), arg];
  }
  //console.log('Util.getOpt', { result });
  return result;
};
Util.getEnv = async varName =>
  Util.tryCatch(
    () => process.env,
    async e => e[varName],
    () => false /* XXX (globalThis.std ? std.getenv(varName) : Util.tryCatch(async () => await import('std').then(std => std.getenv(varName)))) */
  );
Util.getEnvVars = async () =>
  Util.tryCatch(
    () => process.env,
    async e => e,
    () => false
    // XXX     Util.tryCatch(
    //        async () =>
    //          await import('./childProcess.js').then(async ({ PortableChildProcess }) => {
    //            let childProcess = await PortableChildProcess();
    //            (await import('./filesystem.js')).default(fs => (Util.globalThis().filesystem = fs));
    //            let proc = childProcess('env', [], {
    //              block: false,
    //              stdio: [null, 'pipe']
    //            });
    //            let data = '\n';
    //            for await(let output of await filesystem.asyncReader(proc.stdout)) data += filesystem.bufferToString(output);
    //            let matches = [...Util.matchAll(/(^|\n)[A-Za-z_][A-Za-z0-9_]*=.*/gm, data)];
    //            let indexes = matches.map(match => match.index);
    //            let ranges = indexes.reduce((acc, idx, i, a) => [...acc, [idx + 1, a[i + 1]]], []);
    //            let vars = ranges
    //              .map(r => data.substring(...r))
    //              .map(line => {
    //                let eqPos = line.indexOf('=');
    //                return [line.substring(0, eqPos), line.substring(eqPos + 1)];
    //              });
    //            return Object.fromEntries(vars);
    //          })
    //      )
  );

Util.safeFunction = (fn, trapExceptions, thisObj) => {
  const isAsync = Util.isAsync(fn);
  let exec = isAsync
    ? async function(...args) {
        return await fn.call(this || thisObj, ...args);
      }
    : function(...args) {
        return fn.call(this || thisObj, ...args);
      };
  if(trapExceptions) {
    const handleException = typeof trapExceptions == 'function' ? trapExceptions : Util.putError;
    Error.stackTraceLimit = Infinity;
    exec = Util.tryFunction(
      exec, //async (...args) => { Error.stackTraceLimit=Infinity;  return await exec(...args); },
      a => a,
      error => {
        if(Util.isObject(error)) {
          if(error.stack !== undefined) error.stack = new Util.stack(error.stack);
          handleException(error);
        }
      }
    );
  }
  return exec;
};
Util.safeCall = (fn, ...args) => Util.safeApply(fn, args);
Util.safeApply = (fn, args = []) => Util.safeFunction(fn, true)(...args);

Util.exit = exitCode => {
  const { callExitHandlers } = Util;
  //console.log('Util.exit', { exitCode, callExitHandlers });
  if(callExitHandlers) callExitHandlers(exitCode);
  const stdExit = std => {
    std.gc();
    std.exit(exitCode);
  };
  if(globalThis.std) return stdExit(globalThis.std);
  return;
  /* XXX import('std')
    .then(stdExit)
    .catch(() =>*/ Util.tryCatch(
    () => [process, process.exit],
    ([obj, exit]) => exit.call(obj, exitCode),
    () => false
  );
};
Util.atexit = handler => {
  const { handlers } = Util.callMain;
  Util.pushUnique(handlers, handler);
  if(typeof Util.trapExit == 'function') Util.trapExit();
};
Util.callMain = async (fn, trapExceptions) =>
  await Util.safeFunction(
    async (...args) => {
      Util.callMain.handlers = [];
      const { handlers } = Util.callMain;
      const callExitHandlers = (Util.callExitHandlers = Util.once(async ret => {
        if(handlers) for(const handler of handlers) await handler(ret);
        // Util.exit(ret);
      }));
      Util.trapExit = Util.once(() => Util.signal(15, callExitHandlers));
      /* XXX if(Util.getPlatform() == 'quickjs') await import('std').then(module => module.gc()); */
      let ret = await fn(...args);
      await callExitHandlers(ret);
    },
    trapExceptions &&
      (typeof trapExceptions == 'function'
        ? trapExceptions
        : err => {
            let { message, stack } = err;
            stack = new Util.stack(err.stack);
            const scriptDir = Util.tryCatch(
              () => process.argv[1],
              argv1 => argv1.replace(/\/[^\/]*$/g, '')
            );
            console.log('Exception:', message, '\nStack:' + (stack.toString({ colors: true, stripUrl: `file://${scriptDir}/` }) + '').replace(/(^|\n)/g, '\n  '));
            Util.exit(1);
          })
  )(...Util.getArgs().slice(1));

Util.printReturnValue = (fn, opts = {}) => {
  const {
    print = (returnValue, fn, ...args) => {
      let stack = Util.getCallerStack();

      (console.debug || console.log)('RETURN VAL:', /*Util.inspect(*/ returnValue /*, { colors: false })*/, {
        /*fn,
         */ args /*,
        stack*/
      });
    }
  } = opts;
  let self;
  self = (...args) => {
    let returnValue = fn(...args);

    print.call(self, returnValue, fn, ...args);
    return returnValue;
    /*fn = Util.tryFunction(fn, (returnValue, ...args) => {
      print.call(self, returnValue, fn, ...args);
      return returnValue;
    });

    return fn(...args);*/
  };
  Util.define(self, { fn, opts });
  return self;
};
Util.callMain.handlers = [];

Util.replaceAll = (needles, haystack) => {
  return Util.entries(needles)
    .map(([re, str]) => [typeof re == 'string' ? new RegExp(re, 'g') : re, str])
    .reduce((acc, [match, replacement]) => acc.replace(match, replacement), haystack);
};

Util.quote = (str, q = '"') => {
  return q + str.replace(new RegExp(q, 'g'), '\\' + q) + q;
};

Util.escape = (str, pred = codePoint => codePoint < 32 || codePoint > 0xff) => {
  let s = '';
  for(let i = 0; i < str.length; i++) {
    let code = str.codePointAt(i);
    if(!pred(code)) {
      s += str[i];
      continue;
    }

    if(code == 0) s += `\\0`;
    else if(code == 10) s += `\\n`;
    else if(code == 13) s += `\\r`;
    else if(code == 9) s += `\\t`;
    else if(code <= 0xff) s += `\\x${('0' + code.toString(16)).slice(-2)}`;
    else s += `\\u${('0000' + code.toString(16)).slice(-4)}`;
  }
  return s;
};
Util.escapeRegex = string => string.replace(/[-\/\\^$*+?.()|[\]{}]/g, '\\$&');

Util.consolePrinter = function ConsolePrinter(log = console.log) {
  let self;

  self = function(...args) {
    self.add(...args);
    self.print();
    self.clear();
  };

  delete self.length;

  Object.setPrototypeOf(self, Util.extend(Util.consolePrinter.prototype, Util.getMethods(Object.getPrototypeOf(self), 1, 0)));
  self.splice(0, self.length, '');
  self.log = (...args) => log(...args);

  return self;
};
Object.assign(Util.consolePrinter.prototype, Util.getMethods(Array.prototype));

Util.consoleJoin = function(...args) {
  let out = 'push' in this ? this : [];
  if(out.length == 0) out.push('');
  let match = Util.matchAll(/%(?:o|O|d|i|s|f|s|d|c)/g);
  for(let [fmt, ...styles] of args) {
    console.log('Util.consoleJoin', { fmt, styles, out });
    let substs = [...match(fmt)];
    if(substs.length != styles.length) {
      const code = [substs.length, styles.length];
      //console.log("substs:",substs);
      throw new Error(`${code.join(' != ')} ${code.join(', ')}`);
    }
    if(out[0]) out[0] += ' ';
    out[0] += fmt;

    for(let style of styles) Array.prototype.push.call(out, style);
    // Array.prototype.splice.call(out, out.length, 0, ...styles);
    //console.log('Util.consoleJoin', [...out]);
  }
  return out;
};

Util.consoleConcat = function(...args) {
  let self;
  self = function ConsoleConcat(...args) {
    if(args.length == 1 && Array.isArray(args[0])) args = args[0];
    return self.add(...args);
  };
  self.add = Util.consoleJoin;
  /*  function concat(out, args) {
 console.log('concat', { out: [...out], args: [...args] });
   while(args.length) {
      let arg = args.shift();
      if(typeof arg == 'string') {
        let matches = [...Util.matchAll(/%[cos]/g, arg)];
        if(matches.length > 0 && args.length >= matches.length) {
          out[0] += arg;
          out.splice(out.length, 0, ...args.splice(0, matches.length));
        } else {
          out[0] += arg.replace(/%/g, '%%');
        }
      } else if(Util.isArray(arg) && typeof arg[0] == 'string' && /%[cos]/.test(arg[0])) {
        concat(out, arg);
      } else {
        out[0] += ' %o';
        out.push(arg);
      }
    }
    return out;
  }
*/ delete self.length;
  Object.setPrototypeOf(self, Util.extend(Util.consoleConcat.prototype, Object.getPrototypeOf(self)));
  //self.push('');
  if(args.length) self.add(...args);
  return self;
};

Util.consoleConcat.prototype = Object.assign(Util.consoleConcat.prototype, Util.getMethods(Array.prototype, 1, 0), {
  [inspectSymbol]() {
    return [this, [...this]];
  },
  [Symbol.iterator]() {
    return Array.prototype[Symbol.iterator].call(this);
  },
  clear() {
    return this.splice(0, this.length);
  },
  print(log = (...args) => console.info(...args)) {
    log(...this);
  }
});
Util.consolePrinter.prototype.length = 1;
Util.consolePrinter.prototype[0] = '';
Object.assign(Util.consolePrinter.prototype, Util.consoleConcat.prototype, {
  print() {
    const a = [...this];
    const i = a.map(i => Util.inspect(i));
    console.debug('a: ' + i.shift(), ...i);

    Util.consoleConcat.prototype.print.call(this, this.log);
  },
  output() {
    const a = [...this];
    this.clear();
    return a;
  },
  add(...args) {
    let { i = 0 } = this;

    for(; args.length > 0; i++) {
      let arg = args.shift();
      //  console.debug('arg:', i, typeof(arg) == 'string'  ? Util.abbreviate(arg) : arg);

      if(Util.isArray(arg) && /%c/.test(arg[0])) {
        this.i = i;
        this.add(...arg);
        continue;
      }
      if(i > 0) this[0] += ' ';
      if(typeof arg != 'string') {
        this[0] += '%o';
        this.push(arg);
      } else {
        this[0] += arg;
        if(/color:/.test(this[0])) {
          throw new Error(`this[0] is CSS: i=${i}\nthis[0] = "${this[0]}"\narg= ${typeof arg} "${(arg + '').replace(lineSplit, '\\n')}"`);
        }

        const matches = [...Util.matchAll(['%c', '%o'], arg)];
        console.debug('matches.length:', matches.length, ' args.length:', args.length);

        if(matches.length > 0) {
          const styles = args.splice(0, matches.length);
          this.splice(this.length, 0, ...styles);
        }
      }
    }
  }
});

Util.booleanAdapter = (getSetFn, trueValue = 1, falseValue = 0) =>
  function(value) {
    if(value !== undefined) {
      getSetFn(value ? trueValue : falseValue);
    } else {
      let ret = getSetFn();
      if(ret === trueValue) return true;
      if(ret === falseValue) return false;
    }
  };

Util.getSet = (get, set = () => {}, thisObj) =>
  function(...args) {
    if(args.length > 0) return set.call(thisObj || this, ...args);
    return get.call(thisObj || this);
  };

Util.deriveGetSet = (fn, get = v => v, set = v => v, thisObj) =>
  Util.getSet(
    () => get(fn()),
    v => fn(set(v)),
    thisObj
  );
Util.extendFunction = (handler = () => {}) =>
  class ExFunc extends Function {
    constructor() {
      super('...args', 'return this.__self__.__call__(...args)');
      var self = this.bind(this);
      this.__self__ = self;
      return self;
    }

    // Example `__call__` method.
    __call__(...args) {
      return handler(...args);
    }
  };
Util.isatty = async fd => {
  let ret;
  for(let module of ['os', 'tty']) {
    try {
      ret = await import(module).then(mod => mod.isatty(fd));
    } catch(err) {
      ret = undefined;
    }
    if(ret !== undefined) break;
  }
  return ret;
};
Util.ttyGetWinSize = (fd = 1) => {
  let ret;
  if(Util.getPlatform() == 'quickjs') return import('os').then(m => m.ttyGetWinSize(fd));
  const stream = process[['stdin', 'stdout', 'stderr'][fd] || 'stdout'];
  return new Promise(stream.cols ? (resolve, reject) => resolve([stream.cols, stream.rows]) : (resolve, reject) => resolve(stream?.getWindowSize?.()));
};
Util.ttySetRaw = globalThis.os
  ? os.ttySetRaw
  : (fd = 0, mode = true) => {
      let ret;
      const stream = typeof fd == 'number' ? process[['stdin', 'stdout', 'stderr'][fd] || 'stdin'] : fd;
      return stream?.setRawMode?.(mode);
    };
Util.stdio = (fd, mode = true) => {
  if(Util.getPlatform() == 'quickjs') return std[['in', 'out', 'err'][fd]];

  let ret;
  const stream = typeof fd == 'number' ? process[['stdin', 'stdout', 'stderr'][fd] || 'stdin'] : fd;
  return stream?.setRawMode?.(mode);
};

Util.signal = (num, act) => {
  //console.log('Util.signal', { num, act });
  let ret;
  return import('os')
    .then(m => {
      if(typeof num == 'string' && num in m) num = m[num];

      m.signal(num, act);
    })
    .catch(() => process.on(num, act));
};

/**
 * Measure the average execution time of a function
 * @param {Function} fn A function for performance measurement
 * @param {Array} args Function arguments
 * @param {Object} options
 * @returns {Number} Result in milliseconds
 */
Util.timeit = (fn, args = [], options = {}) => {
  const valid = fn && typeof fn === 'function';
  if(!valid) throw new Error('No function provided.');

  const NS_PER_SEC = 1e9;
  const { e, r, l, d } = { e: 1000, r: 1, l: true, d: 6, ...options };
  const { hrtime } = Util;

  let results = [];
  for(let i = 0; i < r; i++) {
    const start = hrtime();
    for(let i = 1; i < e; i++) {
      fn(args);
    }
    const diff = hrtime(start);
    const elapsed = (diff[0] * NS_PER_SEC + diff[1]) * 0.000001;
    const result = elapsed / e;
    results.push(+(Math.round(result + `e+${6}`) + `e-${6}`));
  }
  const ms = results.reduce((p, c) => p + c, 0) / results.length;

  if(l) {
    console.log(`Function   : ${fn.name}()`);
    console.log(`Average    : ${ms.toFixed(d)}ms`);
    console.log(`Repetitions: ${r}`);
    console.log(`Executions : ${e}`);
  }

  return ms;
};

Util.lazyProperty = (obj, name, getter, opts = {}) => {
  const replaceProperty = value => {
    delete obj[name];
    Object.defineProperty(obj, name, { value, ...opts });
    return value;
  };
  const isAsync = Util.isAsync(getter);
  //console.log(`Util.lazyProperty name=${name} isAsync=${isAsync} getter=${getter}`);

  return Object.defineProperty(obj, name, {
    get: isAsync
      ? async function() {
          return replaceProperty(await getter.call(obj, name));
        }
      : function() {
          const value = getter.call(obj, name);
          let isPromise = Util.isObject(value) && value instanceof Promise;
          //console.log(`Util.lazyProperty`, name, value, isPromise);
          if(isPromise) {
            value.then(v => {
              replaceProperty(v);
              //console.log(`Util.lazyProperty resolved `, obj[name]);
              return v;
            });
            return value;
          }
          return replaceProperty(value);
        },
    configurable: true,
    ...opts
  });
};

Util.lazyProperties = (obj, gettersObj, opts = {}) => {
  opts = { enumerable: false, ...opts };
  for(let prop in gettersObj) {
    // console.log('Util.lazyProperties', { prop });
    Util.lazyProperty(obj, prop, gettersObj[prop], opts);
  }
  return obj;
};

Util.calcHRTime = (f = (a, b) => a + b) =>
  function(a, b) {
    const ms = f(a[1], b[1]);
    const div = Math.floor(ms / 1e9);
    const rem = ms % 1e9;

    return [f(a[0], b[0]) + div, rem];
  };
Util.addHRTime = Util.calcHRTime((a, b) => a + b);
Util.subHRTime = Util.calcHRTime((a, b) => a - b);

Util.getHRTime = Util.memoize(() => {
  const { now } = Util;

  class HighResolutionTime extends Array {
    constructor(secs = 0, nano = 0) {
      super(2);
      this[0] = secs;
      this[1] = nano;
      return Object.freeze(this);
    }
    static create(s, n) {
      const sign = Math.sign(s * 1e3 + n * 1e-6);
      s *= sign;
      n *= sign;
      if(n < 0) {
        s--;
        n += 1e9;
      }
      if(n >= 1e9) {
        s++;
        n -= 1e9;
      }
      return new HighResolutionTime(s * sign, n * sign);
    }
    /* prettier-ignore */ get seconds() {
      const [s, n] = this;
      return s + n * 1e-9;
    }
    /* prettier-ignore */ get milliseconds() {
      const [s, n] = this;
      return s * 1e3 + n * 1e-6;
    }
    /* prettier-ignore */ get nanoseconds() {
      const [s, n] = this;
      return s * 1e9 + n;
    }
    [Symbol.toPrimitive]() {
      return this.milliseconds;
    }
    diff(o) {
      let s = o[0] - this[0];
      let n = o[1] - this[1];
      return HighResolutionTime.create(s, n);
    }
    sum(o) {
      /*     let s = o[0] + this[0];
      let n = o[1] + this[1];*/
      return HighResolutionTime.create(...Util.addHRTime(o, this));
    }
    since(o) {
      let s = this[0] - o[0];
      let n = this[1] - o[1];
      return HighResolutionTime.create(s, n);
    }
    toString() {
      let secs = this.seconds;
      let msecs = (secs % 1) * 1e3;
      let nsecs = (msecs % 1) * 1e6;
      let ret = secs >= 1 ? `${Math.floor(secs)}s ` : '';
      return ret + `${Util.roundTo(msecs, 0.001)}ms`;
    }
    inspect() {
      return [this.seconds, this.nanoseconds];
    }
    [inspectSymbol]() {
      return [this.seconds, this.nanoseconds];
      let secs = this.seconds;
      let msecs = (secs % 1) * 1e3;
      let nsecs = (msecs % 1) * 1e6;
      return `${Math.floor(secs)}s ${Util.roundTo(msecs, 0.001)}ms`;
      return `${Math.floor(secs)}s ${Math.floor(msecs)}ms ${Math.floor(nsecs)}ns`;
    }
  }
  Util.getGlobalObject().HighResolutionTime = HighResolutionTime;

  return Util.isAsync(now)
    ? async function hrtime(previousTimestamp) {
        var clocktime = await now();
        var secs = Math.floor(Number(clocktime / 1000));
        var nano = Math.floor(Number(clocktime % 1000) * 1e6);
        let ts = new HighResolutionTime(secs, nano);
        if(previousTimestamp) ts = ts.since(previousTimestamp);
        return ts;
      }
    : function hrtime(previousTimestamp) {
        var clocktime = now();
        var secs = Math.floor(clocktime / 1000);
        var nano = Math.floor((clocktime % 1000) * 1e6);
        let ts = new HighResolutionTime(secs, nano);
        if(previousTimestamp) ts = ts.since(previousTimestamp);
        return ts;
      };
});

Util.lazyProperty(Util, 'animationFrame', () => {
  const { now } = Util;

  return (minDelay = 0) => {
    if(minDelay <= 0) return new Promise(resolve => requestAnimationFrame(resolve));
    const start = now();

    return new Promise(resolve => {
      requestAnimationFrame(animationFrame);

      function animationFrame(t) {
        if(t - start >= minDelay) resolve(t);
        requestAnimationFrame(animationFrame);
      }
    });
  };
});

Util.lazyProperty(Util, 'hrtime', Util.getHRTime);
//Util.startTime = Util.hrtime();

Util.lazyProperty(
  Util,
  'now',
  (Util.getNow = () => {
    const g = Util.getGlobalObject();
    // polyfil for window.performance.now
    var performance = g.performance || {};
    var performanceNow = performance.now || performance.mozNow || performance.msNow || performance.oNow || performance.webkitNow;

    if(performanceNow) {
      //console.log('performanceNow', performanceNow);
      performanceNow = performanceNow.bind(performance); //Util.bind(performanceNow, performance);
    }

    if(!performanceNow && g.cv?.getTickCount) {
      const freq = g.cv.getTickFrequency() / 1000;
      const mul = 1 / freq;
      const getTicks = g.cv.getTickCount;
      performanceNow = () => getTicks() * mul;
    }
    if(!performanceNow && Util.getPlatform() == 'quickjs') {
      let gettime;
      const CLOCK_REALTIME = 0;
      const CLOCK_MONOTONIC = 1;
      const CLOCK_MONOTONIC_RAW = 4;
      const CLOCK_BOOTTIME = 7;

      console.log('STACK:', Util.getCallerStack());

      performanceNow = async function(clock = CLOCK_MONOTONIC_RAW) {
        /* XXX
        if(!gettime) {
          const { dlsym, RTLD_DEFAULT, define, call } = await import('ffi.so');
          const clock_gettime = dlsym(RTLD_DEFAULT, 'clock_gettime');
          define('clock_gettime', clock_gettime, null, 'int', 'int', 'void *');
          gettime = (clk_id, tp) => call('clock_gettime', clk_id, tp);
        }*/
        let data = new ArrayBuffer(16);

        gettime(clock, data);
        let [secs, nsecs] = new BigUint64Array(data, 0, 2);

        let t = /*BigFloat*/ secs * 1e3 + nsecs * 1e-6;
        return t;
      };
    }

    if(!performanceNow) {
      const getTime = Date.now;
      performanceNow = getTime;
    }

    return performanceNow;
  })
);

Util.formatColumns = a => {
  let maxWidth = a.reduce((acc, row, i) => row.map((col, j) => Math.max(acc[j] || 0, (col + '').length)));

  // console.debug(maxWidth);

  return a.map(row => row.map((col, j) => (col + '').padEnd(maxWidth[j])).join(' ')).join('\n');
};

Util.getPlatform = () =>
  Util.tryCatch(
    () => process.versions.node,
    () => 'node',
    Util.tryCatch(
      () => globalThis.scriptArgs[0],
      () => 'quickjs',
      Util.tryCatch(
        () => window.navigator,
        () => 'browser',
        () => undefined
      )
    )
  );

Util.defineGetter(Util, 'platform', Util.memoize(Util.getPlatform));
Util.defineGetter(
  Util,
  'env',
  Util.memoize(async () => {
    let env = await Util.getEnvVars();
    Util.define(Util, 'env', env);
    return env;
  })
);

Util.colIndexes = line => [...line].reduce(([prev, cols], char, i) => [char, [...cols, ...(/\s/.test(prev) && /[^\s]/.test(char) ? [i] : [])]], [' ', []])[1];

Util.colSplit = (line, indexes) => {
  indexes = indexes || Util.colIndexes(line);
  let ret = [];
  for(let i = 0; i < indexes.length; i++) {
    let col = indexes[i];
    let next = indexes[i + 1] || line.length;

    ret.push(line.substring(col, next));
  }
  return ret;
};

Util.bitsToNames = (flags, map = (name, flag) => name) => {
  const entries = [...Util.entries(flags)];

  return function* (value) {
    for(let [name, flag] of entries) if(value & flag && (value & flag) == flag) yield map(name, flag);
  };
};

// time a given function
Util.instrument = (
  fn,
  log = (duration, name, args, ret) => console.log(`function '${name}'` + (ret !== undefined ? ` {= ${Util.abbreviate(Util.escape(ret + ''))}}` : '') + ` timing: ${duration.toFixed(3)}ms`),
  logInterval = 0 //1000
) => {
  const { now, hrtime, functionName } = Util;
  let last = now();
  let duration = 0,
    times = 0;
  const name = functionName(fn) || '<anonymous>';
  const isAsync = Util.isAsync(fn) || Util.isAsync(now);
  const doLog = isAsync
    ? async (args, ret) => {
        let t = await now();
        if(t - (await last) >= logInterval) {
          log(duration / times, name, args, ret);
          duration = times = 0;
          last = t;
        }
      }
    : (args, ret) => {
        let t = now();
        //console.log('doLog', { passed: t - last, logInterval });
        if(t - last >= logInterval) {
          log(duration / times, name, args, ret);
          duration = times = 0;
          last = t;
        }
      };

  return isAsync
    ? async function(...args) {
        const start = await now();
        let ret = await fn.apply(this, args);
        duration += (await now()) - start;
        times++;
        await doLog(args, ret);
        return ret;
      }
    : function(...args) {
        const start = hrtime();
        let ret = fn.apply(this, args);
        duration += now() - start;
        times++;
        doLog(args, ret);
        return ret;
      };
};

Util.trace = (fn, enter, leave, both = () => {}) => {
  enter = enter || ((name, args) => console.log(`function '${name}' (${args.map(arg => inspect(arg)).join(', ')}`));

  leave = leave || ((name, ret) => console.log(`function '${name}'` + (ret !== undefined ? ` {= ${Util.abbreviate(Util.escape(ret + ''))}}` : '')));

  let orig = fn;

  return function(...args) {
    let ret;
    both('enter', fn.name, args);
    enter(fn.name, args);

    ret = orig.call(this, ...args);
    both('leave', fn.name, ret);
    leave(fn.name, ret);
    return ret;
  };
};

Util.bind = function(f, ...args) {
  let ret,
    boundThis = args[0];

  if(args.length < 2)
    ret = function() {
      if(new.target /*this instanceof ret*/) {
        let ret_ = f.apply(this, arguments);
        return Object(ret_) === ret_ ? ret_ : this;
      } else return f.apply(boundThis, arguments);
    };
  else {
    let boundArgs = new Array(args.length - 1);
    for(let i = 1; i < args.length; i++) boundArgs[i - 1] = args[i];

    ret = function() {
      let boundLen = boundArgs.length,
        args = new Array(boundLen + arguments.length),
        i;
      for(i = 0; i < boundLen; i++) args[i] = boundArgs[i];
      for(i = 0; i < arguments.length; i++) args[boundLen + i] = arguments[i];

      if(new.target /*this instanceof ret*/) {
        let ret_ = f.apply(this, args);
        return Object(ret_) === ret_ ? ret_ : this;
      } else return f.apply(boundThis, args);
    };
  }

  ret.prototype = f.prototype;
  return ret;
};

Util.bytesToUTF8 = function* (bytes) {
  if(bytes instanceof ArrayBuffer) bytes = new Uint8Array(bytes);
  let state = 0,
    val = 0;
  for(const c of bytes) {
    if(state !== 0 && c >= 0x80 && c < 0xc0) {
      val = (val << 6) | (c & 0x3f);
      state--;
      if(state === 0) yield val;
    } else if(c >= 0xc0 && c < 0xf8) {
      state = 1 + (c >= 0xe0) + (c >= 0xf0);
      val = c & ((1 << (6 - state)) - 1);
    } else {
      state = 0;
      yield c;
    }
  }
};
Util.codePointsToString = codePoints => {
  let s = '';
  for(let c of codePoints) s += String.fromCodePoint(c);
  return s;
};
Util.bufferToString = b => Util.codePointsToString(Util.bytesToUTF8(b));

Util.levenshteinDistance = function levenshteinDistance(a, b) {
  if(!a || !b) return (a || b).length;
  var m = [];
  for(var i = 0; i <= b.length; i++) {
    m[i] = [i];
    if(i === 0) continue;
    for(var j = 0; j <= a.length; j++) {
      m[0][j] = j;
      if(j === 0) continue;
      m[i][j] = b.charAt(i - 1) == a.charAt(j - 1) ? m[i - 1][j - 1] : Math.min(m[i - 1][j - 1] + 1, m[i][j - 1] + 1, m[i - 1][j] + 1);
    }
  }
  return m[b.length][a.length];
};

Util.padTrunc = (...args) => {
  let [len, s] = args;
  const end = len >= 0;
  len = Math.abs(len);
  if(args.length < 2) {
    return (s, pad = ' ') => {
      s = s + '';
      len ??= s.length;
      return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, pad);
    };
  } else {
    s = s + '';
    len ??= s.length;
    return s.length > len ? s.slice(0, len) : s['pad' + (end ? 'End' : 'Start')](len, ' ');
  }
};

Util.setReadHandler = (fd, handler) => (Util.getPlatform() == 'quickjs' ? import('os').then(os => os.setReadHandler(fd, handler)) : fd.on('data', handler));

Util();
/* ---------------------------- end of 'util.js' ---------------------------- */

/* --------------------------- start of 'node.js' --------------------------- */
class Node {
  static parents(node) {
    return (function* () {
      let n = node;
      do {
        if(n) yield n;
      } while(n && (n = n.parentNode));
    })();
  }

  static depth(node) {
    let r = 0;
    while(node && node.parentNode) {
      r++;
      node = node.parentNode;
    }
    return r;
  }

  static attrs(node) {
    return node.attributes && node.attributes.length > 0
      ? Array.from(node.attributes).reduce(
          (acc, attr) => ({
            ...acc,
            [attr.name]: isNaN(parseFloat(attr.value)) ? attr.value : parseFloat(attr.value)
          }),
          {}
        )
      : {};
  }

  static *map(map, propFn) {
    if(!propFn && 'getPropertyValue' in map) propFn = k => [k, map.getPropertyValue(k)];

    if(!propFn && typeof map.item == 'function')
      propFn = (k, i) => {
        let { name, value } = map.item(i);
        return [name, value];
      };

    for(let i = 0; i < map.length; i++) yield propFn(map[i], i, map);
  }
}

Node;
/* ---------------------------- end of 'node.js' ---------------------------- */

/* --------------------------- start of 'trbl.js' --------------------------- */

/* --------------------------- start of 'rect.js' --------------------------- */

/* -------------------------- start of 'point.js' --------------------------- */

const SymSpecies = Util.tryCatch(
  () => Symbol,
  sym => sym.species
);

const CTOR = obj => {
  if(obj[SymSpecies]) return obj[SymSpecies];
  let p = Object.getPrototypeOf(obj);
  if(p[SymSpecies]) return p[SymSpecies];
  return p.constructor;
};

function Point(...args) {
  let isNew = this instanceof Point;
  args = args[0] instanceof Array ? args.shift() : args;
  let p = isNew ? this : new Point(...args);
  let arg = args.shift();

  if(!new.target) if (arg instanceof Point) return arg;

  if(typeof arg === 'undefined') {
    p.x = arg;
    p.y = args.shift();
  } else if(typeof arg === 'number') {
    p.x = parseFloat(arg);
    p.y = parseFloat(args.shift());
  } else if(typeof arg === 'string') {
    const matches = [...arg.matchAll(/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g)];

    p.x = parseFloat(matches[0]);
    p.y = parseFloat(matches[1]);
  } else if(typeof arg == 'object' && arg !== null && (arg.x !== undefined || arg.y !== undefined)) {
    p.x = arg.x;
    p.y = arg.y;
  } else if(typeof arg == 'object' && arg !== null && arg.length > 0 && x !== undefined && y !== undefined) {
    p.x = parseFloat(arg.shift());
    p.y = parseFloat(arg.shift());
  } else if(typeof args[0] === 'number' && typeof args[1] === 'number') {
    p.x = args[0];
    p.y = args[1];
    args.shift(2);
  } else {
    p.x = 0;
    p.y = 0;
  }
  if(p.x === undefined) p.x = 0;
  if(p.y === undefined) p.y = 0;
  if(isNaN(p.x)) p.x = undefined;
  if(isNaN(p.y)) p.y = undefined;

  if(!isNew) {
    /* if(p.prototype == Object) p.prototype = Point.prototype;
    else Object.assign(p, Point.prototype);*/
    return p;
  }
}

Point.getOther = args => (console.debug('getOther', ...args), typeof args[0] == 'number' ? [{ x: args[0], y: args[1] }] : args);

Object.defineProperties(Point.prototype, {
  X: {
    get() {
      return this.x;
    }
  },
  Y: {
    get() {
      return this.y;
    }
  }
});

Point.prototype.move = function(x, y) {
  this.x += x;
  this.y += y;
  return this;
};
Point.prototype.moveTo = function(x, y) {
  this.x = x;
  this.y = y;
  return this;
};
Point.prototype.clear = function(x, y) {
  this.x = 0;
  this.y = 0;
  return this;
};
Point.prototype.set = function(fn) {
  if(typeof fn != 'function') {
    Point.apply(this, [...arguments]);
    return this;
  }
  return fn(this.x, this.y);
};
Point.prototype.clone = function() {
  const ctor = this[Symbol.species] || this.constructor[Symbol.species];

  return new ctor({ x: this.x, y: this.y });
};
Point.prototype.sum = function(...args) {
  const p = new Point(...args);
  let r = new Point(this.x, this.y);
  r.x += p.x;
  r.y += p.y;
  return r;
};
Point.prototype.add = function(...args) {
  const other = new Point(...args);
  this.x += other.x;
  this.y += other.y;
  return this;
};
Point.prototype.diff = function(arg) {
  let { x, y } = this;
  let fn = function(other) {
    let r = new Point(x, y);
    return r.sub(other);
  };
  if(arg) return fn(arg);
  return fn;
};
Point.prototype.sub = function(...args) {
  const other = new Point(...args);
  this.x -= other.x;
  this.y -= other.y;
  return this;
};
Point.prototype.prod = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  return new Point(this.x * o.x, this.y * o.y);
};
Point.prototype.mul = function(f) {
  const o = isPoint(f) ? f : { x: f, y: f };
  this.x *= o.x;
  this.y *= o.y;
  return this;
};
Point.prototype.quot = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  return new Point(this.x / other.x, this.y / other.y);
};
Point.prototype.div = function(other) {
  other = isPoint(other) ? other : { x: other, y: other };
  this.x /= other.x;
  this.y /= other.y;
  return this;
};
Point.prototype.comp = function() {
  return new Point({ x: -this.x, y: -this.y });
};
Point.prototype.neg = function() {
  this.x *= -1;
  this.y *= -1;
  return this;
};
Point.prototype.distanceSquared = function(other = { x: 0, y: 0 }) {
  return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
};
Point.prototype.distance = function(other = { x: 0, y: 0 }) {
  return Math.sqrt(Point.prototype.distanceSquared.call(this, Point(other)));
};
Point.prototype.equals = function(other) {
  let { x, y } = this;
  return +x == +other.x && +y == +other.y;
};
Point.prototype.round = function(precision = 0.001, digits, type) {
  let { x, y } = this;
  digits = digits || Util.roundDigits(precision);
  type = type || 'round';
  this.x = Util.roundTo(x, precision, digits, type);
  this.y = Util.roundTo(y, precision, digits, type);
  return this;
};
Point.prototype.ceil = function() {
  let { x, y } = this;
  this.x = Math.ceil(x);
  this.y = Math.ceil(y);
  return this;
};
Point.prototype.floor = function() {
  let { x, y } = this;
  this.x = Math.floor(x);
  this.y = Math.floor(y);
  return this;
};

Point.prototype.dot = function(other) {
  return this.x * other.x + this.y * other.y;
};

Point.prototype.values = function() {
  return [this.x, this.y];
};
Point.prototype.fromAngle = function(angle, dist = 1.0) {
  this.x = Math.cos(angle) * dist;
  this.y = Math.sin(angle) * dist;
  return this;
};
Point.prototype.toAngle = function(deg = false) {
  return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
};
Point.prototype.angle = function(other, deg = false) {
  other = other || { x: 0, y: 0 };
  return Point.prototype.diff.call(this, other).toAngle(deg);
};
Point.prototype.rotate = function(angle, origin = { x: 0, y: 0 }) {
  this.x -= origin.x;
  this.y -= origin.y;
  let c = Math.cos(angle),
    s = Math.sin(angle);
  let xnew = this.x * c - this.y * s;
  let ynew = this.x * s + this.y * c;
  this.x = xnew;
  this.y = ynew;
  return this;
};
Util.defineGetter(Point.prototype, Symbol.iterator, function() {
  const { x, y } = this;
  let a = [x, y];
  return a[Symbol.iterator].bind(a);
});

/*Point.prototype.valueOf = function(shl = 16) {
  const { x, y } = this;
  return x | (y << shl);
};
*/ Point.prototype.toString = function(opts = {}) {
  const { precision = 0.001, unit = '', separator = ',', left = '', right = '', pad = 0 } = opts;
  let x = Util.roundTo(this.x, precision);
  let y = Util.roundTo(this.y, precision);
  if(pad > 0) {
    x = x + '';
    y = y + '';
    if(y[0] != '-') y = ' ' + y;
    if(x[0] != '-') x = ' ' + x;
  }
  //console.debug("toString", {x,y}, {pad});
  return `${left}${(x + '').padStart(pad, ' ')}${unit}${separator}${(y + '').padEnd(pad, ' ')}${unit}${right}`;
};
Point.prototype[Symbol.toStringTag] = 'Point';
Point.prototype.toSource = function(opts = {}) {
  const { asArray = false, plainObj = false, pad = a => a /*a.padStart(4, ' ')*/, showNew = true } = opts;
  let x = pad(this.x + '');
  let y = pad(this.y + '');
  let c = t => t;
  if(typeof this != 'object' || this === null) return '';
  if(asArray) return `[${x},${y}]`;
  if(plainObj) return `{x:${x},y:${y}}`;

  return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(', 1, 36)}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
};

/*Point.prototype.toSource = function() {
  return '{x:' + this.x + ',y:' + this.y + '}';
};*/
Point.prototype.toObject = function(proto = Point.prototype) {
  const { x, y } = this;
  const obj = { x, y };
  Object.setPrototypeOf(obj, proto);
  return obj;
};
Point.prototype.toCSS = function(precision = 0.001, edges = ['left', 'top']) {
  return {
    [edges[0]]: Util.roundTo(this.x, precision) + 'px',
    [edges[1]]: Util.roundTo(this.y, precision) + 'px'
  };
};
Point.prototype.toFixed = function(digits) {
  return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
};
Point.prototype.isNull = function() {
  return this.x == 0 && this.y == 0;
};
Point.prototype.inside = function(rect) {
  return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
};
Point.prototype.transform = function(m, round = true) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  //if(Util.isObject(m) && typeof m.transform_point == 'function') return m.transform_point(this);

  const x = m[0] * this.x + m[1] * this.y + m[2];
  const y = m[3] * this.x + m[4] * this.y + m[5];

  this.x = x;
  this.y = y;
  if(round) Point.prototype.round.call(this, 1e-13, 13);

  return this;
};
Point.prototype.scaleTo = function(minmax) {
  return new Point({
    x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1),
    y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1)
  });
};
Point.prototype.normalize = function() {
  let d = Point.prototype.distance.call(this);
  return Point.prototype.div.call(this, { x: d, y: d });
};
Point.prototype.normal = function() {
  let d = Point.prototype.distance.call(this);
  return new Point({ x: this.x / d, y: this.y / d });
};

Point.fromString = str => new Point(...str.split(/[^-.0-9]+/g).map(n => +n));
Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
Point.angle = (point, other, deg = false) => Point.prototype.angle.call(point, other, deg);
Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
Point.sub = (point, other) => Point.prototype.sub.call(point, other);
Point.prod = (a, b) => Point.prototype.prod.call(a, b);
Point.quot = (a, b) => Point.prototype.quot.call(a, b);
Point.equals = (a, b) => Point.prototype.equals.call(a, b);
Point.round = (point, prec, digits, type) => Point.prototype.round.call(point, prec, digits, type);
Point.fromAngle = (angle, f) => new Point().fromAngle(angle, f);

for(let name of [
  'clone',
  'comp',
  'neg',
  'sides',
  'dimension',
  'toString',
  //'toSource',
  'toCSS',
  'sub',
  'diff',
  'add',
  'sum',
  'distance'
]) {
  Point[name] = (point, ...args) => Point.prototype[name].call(Point(point), ...args);
}
Point.interpolate = (p1, p2, a) => {
  a = Util.clamp(0, 1, a);
  return new Point(p1.x * (1.0 - a) + p2.x * a, p1.y * (1.0 - a) + p2.y * a);
};

Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) => `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;

const isPoint = o => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);

Point.isPoint = isPoint;

Point.prototype[Util.inspectSymbol] = function(depth, options) {
  const { x, y } = this;
  return /*Object.setPrototypeOf*/ { x, y } /*, Point.prototype*/;
};

Point.bind = (...args) => {
  const keys = ['x', 'y'];
  let [o, p] = args;
  if(p == null) p = keys;
  const { x, y } = (Util.isArray(p) && p.reduce((acc, name, i) => ({ ...acc, [keys[i]]: name }), {})) || p;
  //console.debug('Point.bind', { keys, o, p, x, y });
  return Object.setPrototypeOf(Util.bindProperties({}, o, { x, y }), Point.prototype);
};
Point;

Util.defineGetter(Point, Symbol.species, function() {
  return this;
});

const ImmutablePoint = Util.immutableClass(Point);
Util.defineGetter(ImmutablePoint, Symbol.species, () => ImmutablePoint);
/* --------------------------- end of 'point.js' ---------------------------- */

//import { PointList } from './pointList.js';

/* --------------------------- start of 'line.js' --------------------------- */

/* --------------------------- start of 'bbox.js' --------------------------- */

/* --------------------------- start of 'size.js' --------------------------- */

function Size(arg) {
  let obj = this instanceof Size ? this : {};
  let args = [...arguments];
  if(args.length == 1 && Util.isObject(args[0]) && args[0].length !== undefined) {
    args = args[0];
    arg = args[0];
  }
  if(typeof arg == 'object') {
    if(arg.width !== undefined || arg.height !== undefined) {
      arg = args.shift();
      obj.width = arg.width;
      obj.height = arg.height;
    } else if(arg.x2 !== undefined && arg.y2 !== undefined) {
      arg = args.shift();
      obj.width = arg.x2 - arg.x;
      obj.height = arg.y2 - arg.y;
    } else if(arg.bottom !== undefined && arg.right !== undefined) {
      arg = args.shift();
      obj.width = arg.right - arg.left;
      obj.height = arg.bottom - arg.top;
    }
  } else {
    while(typeof arg == 'object' && (arg instanceof Array || 'length' in arg)) {
      args = [...arg];
      arg = args[0];
    }
    if(args && args.length >= 2) {
      let w = args.shift();
      let h = args.shift();
      if(typeof w == 'object' && 'baseVal' in w) w = w.baseVal.value;
      if(typeof h == 'object' && 'baseVal' in h) h = h.baseVal.value;
      obj.width = typeof w == 'number' ? w : parseFloat(w.replace(/[^-.0-9]*$/, ''));
      obj.height = typeof h == 'number' ? h : parseFloat(h.replace(/[^-.0-9]*$/, ''));
      Object.defineProperty(obj, 'units', {
        value: {
          width: typeof w == 'number' ? 'px' : w.replace(obj.width.toString(), ''),
          height: typeof h == 'number' ? 'px' : h.replace(obj.height.toString(), '')
        },
        enumerable: false
      });
    }
  }
  if(isNaN(obj.width)) obj.width = undefined;
  if(isNaN(obj.height)) obj.height = undefined;
  if(!(obj instanceof Size)) return obj;
}

Size.getOther = args => (/*console.debug('getOther', ...args), */ typeof args[0] == 'number' ? [{ width: args[0], height: args[1] }] : args);

Size.prototype.width = NaN;
Size.prototype.height = NaN;
Size.prototype.units = null;
Size.prototype[Symbol.toStringTag] = 'Size';

Size.prototype.convertUnits = function(w = 'window' in globalThis ? window : null) {
  if(w === null) return this;
  const view = {
    vw: w.innerWidth,
    vh: w.innerHeight,
    vmin: w.innerWidth < w.innerHeight ? w.innerWidth : w.innerHeight,
    vmax: w.innerWidth > w.innerHeight ? w.innerWidth : w.innerHeight
  };
  if(view[this.units.width] !== undefined) {
    this.width = (this.width / 100) * view[this.units.width];
    delete this.units.width;
  }
  if(view[this.units.height] !== undefined) {
    this.height = (this.height / 100) * view[this.units.height];
    delete this.units.height;
  }
  return size;
};

Size.prototype.aspect = function() {
  return this.width / this.height;
};
Size.prototype.toCSS = function(units) {
  let ret = {};
  units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
  if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
  if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
  return ret;
};
Size.prototype.transform = function(m) {
  const [xx, xy, , yx, yy] = m;
  const { width, height } = this;

  //console.log("Size.transform", { width, height, xx, xy, yx, yy});
  this.width = xx * width + xy * height;
  this.height = yx * width + yy * height;
  if(round) Size.prototype.round.call(this, 1e-13, 13);
  return this;
};
Size.prototype.isSquare = function() {
  return Math.abs(this.width - this.height) < 1;
};
Size.prototype.isNull = function() {
  return this.width == 0 && this.height == 0;
};
Size.prototype.area = function() {
  return this.width * this.height;
};
Size.prototype.resize = function(width, height) {
  this.width = width;
  this.height = height;
  return this;
};
Size.prototype.equals = function(other) {
  let { width, height } = this;
  return +width == +other.width && +height == +other.height;
};
Size.prototype.sum = function(other) {
  return new Size(this.width + other.width, this.height + other.height);
};
Size.prototype.add = function(...args) {
  for(let other of Size.getOther(args)) {
    this.width += other.width;
    this.height += other.height;
  }
  return this;
};
Size.prototype.diff = function(other) {
  return new Size(this.width - other.width, this.height - other.height);
};
Size.prototype.sub = function(...args) {
  for(let other of Size.getOther(args)) {
    this.width -= other.width;
    this.height -= other.height;
  }
  return this;
};
Size.prototype.clone = function(__proto__ = Size.prototype) {
  const { width, height } = this;
  // return new Size(width, height); // { width,height, __proto__ };
  return Object.setPrototypeOf({ width, height }, __proto__);
};
Size.prototype.prod = function(...args) {
  return Size.prototype.clone.call(this).mul(...args);
};
Size.prototype.mul = function(...args) {
  for(let f of Size.getOther(args)) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    this.width *= o.width;
    this.height *= o.height;
  }
  return this;
};
Size.prototype.quot = function(other) {
  return new Size(this.width / other.width, this.height / other.height);
};
Size.prototype.inverse = function(other) {
  return new Size(1 / this.width, 1 / this.height);
};
Size.prototype.div = function(...args) {
  for(let f of Size.getOther(args)) {
    this.width /= f;
    this.height /= f;
  }
  return this;
};
Size.prototype.round = function(precision = 1, digits, type) {
  let { width, height } = this;
  this.width = Util.roundTo(width, precision, digits, type);
  this.height = Util.roundTo(height, precision, digits, type);
  return this;
};
Size.prototype.bounds = function(other) {
  let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
  let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];

  let scale = h / this.height;
  this.mul(scale);
  return this;
};

Size.prototype.fit = function(size) {
  size = new Size(size);
  let factors = Size.prototype.fitFactors.call(this, size);
  let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
  return ret;
};

Size.prototype.fitHeight = function(other) {
  other = new Size(other);
  let scale = other.height / this.height;
  this.mul(scale);
  return [this.width, other.width];
};
Size.prototype.fitWidth = function(other) {
  other = new Size(other);
  let scale = other.width / this.width;
  this.mul(scale);
  return [this.height, other.height];
};
Size.prototype.fitFactors = function(other) {
  const hf = other.width / this.width;
  const vf = other.height / this.height;
  return [hf, vf];
};
Size.prototype.toString = function(opts = {}) {
  const { unit = '', separator = ' \u2715 ', left = '', right = '' } = opts;
  const { width, height, units = { width: unit, height: unit } } = this;
  return `${left}${width}${(Util.isObject(units) && units.width) || unit}${separator}${height}${(Util.isObject(units) && units.height) || unit}${right}`;
};
Size.prototype[Util.inspectSymbol] = function(depth, options) {
  const { width, height } = this;
  return Object.setPrototypeOf({ width, height }, Size.prototype);
};
/*Size.prototype[Symbol.iterator] = function() {
    let [width,height]= this;
    return [width,height][Symbol.iterator]();
  }*/
Size.fromString = str => {
  const matches = [...Util.matchAll(/[-.\d]+/g, str)];
  return new Size(...matches.map(m => +m[0]));
};
Size.prototype.toObject = function() {
  const { width, height } = this;
  return { width, height };
};
Size.area = sz => Size.prototype.area.call(sz);
Size.aspect = sz => Size.prototype.aspect.call(sz);

Size.bind = (...args) => {
  const o = args[0] instanceof Size ? args.shift() : new Size();
  const gen = Util.isFunction(args[args.length - 1]) && args.pop();
  const p = args.length > 1 ? args.pop() : ['width', 'height'];
  const t = args.pop();
  gen = gen || (k => v => v === undefined ? t[k] : (t[k] = v));

  // const [  p = ['width', 'height']  ] = args[0] instanceof Size ? args : [new Size(), ...args];
  console.debug('Size.bind', { args, o, t, p, gen });
  const { width, height } = Util.isArray(p) ? p.reduce((acc, name) => ({ ...acc, [name]: name }), {}) : p;
  return Util.bindProperties(new Size(0, 0), t, { width, height }, gen);
};

for(let method of Util.getMethodNames(Size.prototype)) if(method != 'toString') Size[method] = (size, ...args) => Size.prototype[method].call(size || new Size(size), ...args);

const isSize = o => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
  Size[name] = (size, ...args) => Size.prototype[name].call(size || new Size(size), ...args);
}

Util.defineGetter(Size, Symbol.species, function() {
  return this;
});

const ImmutableSize = Util.immutableClass(Size);
Util.defineGetter(ImmutableSize, Symbol.species, () => ImmutableSize);
/* ---------------------------- end of 'size.js' ---------------------------- */

class BBox {
  static fromPoints(pts) {
    let pt = pts.shift();
    let bb = new BBox(pt.x, pt.y, pt.x, pt.y);
    bb.update(pts);
    return bb;
  }

  static fromRect(rect) {
    return new BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);
  }

  static create(arg) {
    if(isRect(arg)) return new BBox(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height);

    if(Util.isArray(arg) || Util.isIterable(arg)) return BBox.of(...arg);
  }

  static of(...args) {
    return BBox.from(args);
  }

  constructor(...args) {
    if(args.length == 4 && args.every(arg => Number.isFinite(arg))) {
      const [x1, y1, x2, y2] = args;

      this.x1 = Math.min(x1, x2);
      this.y1 = Math.min(y1, y2);
      this.x2 = Math.max(x1, x2);
      this.y2 = Math.max(y1, y2);
    } else if(isBBox(args[0])) {
      const { x1, y1, x2, y2 } = args[0];
      this.x1 = x1;
      this.y1 = y1;
      this.x2 = x2;
      this.y2 = y2;
    } else {
      this.x1 = undefined;
      this.y1 = undefined;
      this.x2 = undefined;
      this.y2 = undefined;
      if(args.length > 0) this.updateList([...args]);
    }

    Util.define(this, 'objects', {});
  }

  getObjects() {
    return new Map(Object.entries(this.objects));
  }

  updateList(list, offset = 0.0, objFn = item => item, t = a => a) {
    for(let arg of list) this.update(t(arg), offset, objFn(arg));
    return this;
  }

  update(arg, offset = 0.0, obj = null) {
    //console.log('BBox.update', { arg, offset, obj });
    if(Util.isArray(arg)) return this.updateList(arg, offset);
    else if(Util.isObject(arg)) {
      if(typeof arg.bbox == 'function') {
        arg = arg.bbox();
      } else if(isBBox(arg.objects)) {
        this.updateList(Object.values(arg.objects), offset);
      } else {
        if(arg.x2 !== undefined && arg.y2 != undefined) this.updateXY(arg.x2, arg.y2, 0, name => (this.objects[name] = obj || arg));
        if(arg.x1 !== undefined && arg.y1 != undefined) this.updateXY(arg.x1, arg.y1, 0, name => (this.objects[name] = obj || arg));
        if(arg.x !== undefined && arg.y != undefined) this.updateXY(arg.x, arg.y, offset, name => (this.objects[name] = obj || arg));
      }
    }

    return this;
  }

  updateXY(x, y, offset = 0, set = () => {}) {
    let updated = {};
    if(this.x1 === undefined || this.x1 > x - offset) {
      this.x1 = x - offset;
      set('x1');
    }
    if(this.x2 === undefined || this.x2 < x + offset) {
      this.x2 = x + offset;
      set('x2');
    }
    if(this.y1 === undefined || this.y1 > y - offset) {
      this.y1 = y - offset;
      set('y1');
    }
    if(this.y2 === undefined || this.y2 < y + offset) {
      this.y2 = y + offset;
      set('y2');
    }
    //if(Object.keys(updated)) console.log(`BBox update ${x},${y} `, updated);
    return this;
  }

  get center() {
    return new Point({
      x: this.x + this.width / 2,
      y: this.y + this.height / 2
    });
  }

  relative_to(x, y) {
    return new BBox(this.x1 - x, this.y1 - y, this.x2 - x, this.y2 - y);
  }

  get x() {
    return this.x1;
  }

  get width() {
    return /*Math.abs*/ this.x2 - this.x1;
  }

  get y() {
    return this.y1 < this.y2 ? this.y1 : this.y2;
  }

  get height() {
    return /*Math.abs*/ this.y2 - this.y1;
  }

  set x(x) {
    let ix = x - this.x1;
    this.x1 += ix;
    this.x2 += ix;
  }

  set width(w) {
    this.x2 = this.x1 + w;
  }

  set y(y) {
    let iy = y - this.y1;
    this.y1 += iy;
    this.y2 += iy;
  }

  set height(h) {
    this.y2 = this.y1 + h;
  }

  get rect() {
    const { x1, y1, x2, y2 } = this;
    return {
      x: x1,
      y: y1,
      width: x2 - x1,
      height: y2 - y1
    };
  }

  get size() {
    const { x1, y1, x2, y2 } = this;
    return new Size(x2 - x1, y2 - y1);
  }

  toRect(proto) {
    let r = this.rect;
    return Object.setPrototypeOf(r, proto || Object.prototype);
  }

  toSize(ctor = obj => Object.setPrototypeOf(obj, Object.prototype)) {
    let width = this.x2 - this.x1;
    let height = this.y2 - this.y1;
    return ctor({ width, height });
  }

  toObject() {
    const { x1, y1, x2, y2 } = this;
    let obj = Object.create(null);
    obj.x1 = x1;
    obj.y1 = y1;
    obj.x2 = x2;
    obj.y2 = y2;
    return obj;
  }
  toString() {
    return `${this.x1} ${this.y1} ${this.x2} ${this.y2}`;
  }

  transform(fn = arg => arg, out) {
    if(!out) out = this;
    for(let prop of ['x1', 'y1', 'x2', 'y2']) {
      const v = this[prop];
      out[prop] = fn(v);
    }
    return this;
  }

  round(fn = arg => Math.round(arg)) {
    let ret = new BBox();
    this.transform(fn, ret);
    return ret;
  }

  move(x, y) {
    this.x1 += x;
    this.y1 += y;
    this.x2 += x;
    this.y2 += y;
    return this;
  }

  static from(iter, tp = p => p) {
    if(typeof iter == 'object' && iter[Symbol.iterator]) iter = iter[Symbol.iterator]();

    let r = new BBox();
    let result = iter.next();
    let p;
    if(result.value) {
      p = tp(result.value);
      r.x1 = p.x1 !== undefined ? p.x1 : p.x;
      r.x2 = p.x2 !== undefined ? p.x2 : p.x;
      r.y1 = p.y1 !== undefined ? p.y1 : p.y;
      r.y2 = p.y2 !== undefined ? p.y2 : p.y;
    }
    while(true) {
      result = iter.next();
      if(!result.value) break;
      p = tp(result.value);

      r.update(p);
    }
    return r;
  }

  *[Symbol.iterator]() {
    let [x1, x2, y1, y2] = this;
    for(let prop of [x1, x2, y1, y2]) yield prop;
  }
}
const isBBox = (bbox, testFn = (prop, name, obj) => name in obj) => Util.isObject(bbox) && ['x1', 'y1', 'x2', 'y2'].every(n => testFn(bbox[n], n, bbox));

BBox;
/* ---------------------------- end of 'bbox.js' ---------------------------- */

function Line(...args) {
  if(!new.target) if (args[0] instanceof Line) return args[0];

  let [x1, y1, x2, y2] = args;
  let obj;
  let arg;
  let ret;
  if(args.length >= 4 && args.every(arg => !isNaN(parseFloat(arg)))) {
    arg = { x1, y1, x2, y2 };
  } else if(args.length == 1) {
    arg = args[0];
  }
  obj = new.target ? this : null /* new Line()*/;

  //obj = this || { ...arg };

  if(obj === null) obj = Object.create(Line.prototype);

  if(Object.getPrototypeOf(obj) !== Line.prototype) Object.setPrototypeOf(obj, Line.prototype);

  //if(!('a' in obj) || !('b' in obj)) throw new Error('no a/b prop');
  if(arg && arg.x1 !== undefined && arg.y1 !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x1, y1, x2, y2 } = arg;
    obj.x1 = parseFloat(x1);
    obj.y1 = parseFloat(y1);
    obj.x2 = parseFloat(x2);
    obj.y2 = parseFloat(y2);
    ret = 1;
  } else if(isPoint(args[0]) && isPoint(args[1])) {
    args = args.map(a => Point(a));

    obj.x1 = args[0].x;
    obj.y1 = args[0].y;
    obj.x2 = args[1].x;
    obj.y2 = args[1].y;
    ret = 2;
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every(arg => !isNaN(+arg))) {
    const [x1, y1, x2, y2] = arg.map(a => +a);
    obj.x1 = x1;
    obj.y1 = y1;
    obj.x2 = x2;
    obj.y2 = y2;
    ret = 4;
  } else {
    ret = 0;
  }

  if(!('a' in obj) || obj.a === undefined)
    Object.defineProperty(obj, 'a', {
      value: new Point(obj.x1, obj.y1),
      enumerable: false
    });
  if(!('b' in obj) || obj.b === undefined)
    Object.defineProperty(obj, 'b', {
      value: new Point(obj.x2, obj.y2),
      enumerable: false
    });

  if(!isLine(obj)) {
    //console.log('ERROR: is not a line: ', Util.inspect(arg), Util.inspect(obj));
  }

  if(!['x1', 'y1', 'x2', 'y2'].every(prop => !isNaN(+obj[prop]))) return null;

  /*  if(this !== obj)*/ return obj;
}

const isLine = obj => (Util.isObject(obj) && ['x1', 'y1', 'x2', 'y2'].every(prop => obj[prop] !== undefined)) || ['a', 'b'].every(prop => isPoint(obj[prop]));

/*
Object.defineProperty(Line.prototype, 'a', { value: new Point(), enumerable: true });
Object.defineProperty(Line.prototype, 'b', { value: new Point(), enumerable: true });
*/

Line.prototype.intersect = function(other) {
  const ma = (this[0].y - this[1].y) / (this[0].x - this[1].x);
  const mb = (other[0].y - other[1].y) / (other[0].x - other[1].x);
  if(ma - mb < Number.EPSILON) return undefined;
  return new Point({
    x: (ma * this[0].x - mb * other[0].x + other[0].y - this[0].y) / (ma - mb),
    y: (ma * mb * (other[0].x - this[0].x) + mb * this[0].y - ma * other[0].y) / (mb - ma)
  });
};

Object.defineProperty(Line.prototype, 'a', {
  get() {
    return Line.a(this);
  },
  set(value) {
    if(!(value instanceof Point)) value = Point(value);
    this.x1 = value.x;
    this.y1 = value.y;
  },
  enumerable: false
});
Object.defineProperty(Line.prototype, 'b', {
  get() {
    return Line.b(this);
  },
  set(value) {
    if(!(value instanceof Point)) value = Point(value);
    this.x2 = value.x;
    this.y2 = value.y;
  },
  enumerable: false
});

Object.defineProperty(Line.prototype, 0, {
  get() {
    return this.a;
  },
  set(v) {
    this.a = v;
  },
  enumerable: false
});
Object.defineProperty(Line.prototype, 1, {
  get() {
    return this.b;
  },
  set(v) {
    this.b = v;
  },
  enumerable: false
});
/*Object.defineProperty(Line.prototype, 'x1', {get() {return this.a && this.a.x; }, set(v) {if(!this.a) Object.defineProperty(this, 'a', { value: new Point(), enumerable: false }); this.a.x = v; }, enumerable: true });
Object.defineProperty(Line.prototype, 'y1', {get() {return this.a && this.a.y; }, set(v) {if(!this.a) Object.defineProperty(this, 'a', {value: new Point(), enumerable: false }); this.a.y = v; }, enumerable: true });
Object.defineProperty(Line.prototype, 'x2', {get() {return this.b && this.b.x; }, set(v) {if(!this.b) Object.defineProperty(this, 'b', {value: new Point(), enumerable: false }); this.b.x = v; }, enumerable: true });
Object.defineProperty(Line.prototype, 'y2', {get() {return this.b && this.b.y; }, set(v) {if(!this.b) Object.defineProperty(this, 'b', {value: new Point(), enumerable: false }); this.b.y = v; }, enumerable: true });
*/
Line.prototype.direction = function() {
  let dist = Point.prototype.distance.call(this.a, this.b);
  return Point.prototype.quot.call(Line.prototype.getSlope.call(this), dist);
};
Line.prototype.getVector = function() {
  return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
};
Object.defineProperty(Line.prototype, 'vector', {
  get: Line.prototype.getVector
});
Line.prototype.getSlope = function() {
  return (this.y2 - this.y1) / (this.x2 - this.x1);
};
Object.defineProperty(Line.prototype, 'slope', {
  get() {
    return new Point(this.x2 - this.x1, this.y2 - this.y1);
  }
});
Line.prototype.yIntercept = function() {
  let v = Line.prototype.getVector.call(this);
  if(v.x !== 0) {
    let slope = v.y / v.x;
    return [this.a.y - this.a.x * slope, slope || 0];
  }
};
Line.prototype.xIntercept = function() {
  let v = Line.prototype.getVector.call(this);
  if(v.y !== 0) {
    let slope = v.x / v.y;
    return [this.a.x - this.a.y * slope, slope || 0];
  }
};
Line.prototype.isHorizontal = function() {
  return Line.prototype.getVector.call(this).y === 0;
};
Line.prototype.isVertical = function() {
  return Line.prototype.getVector.call(this).x === 0;
};

Line.prototype.isNull = function() {
  return this.x1 == 0 && this.y1 == 0 && this.x2 == 0 && this.y2 == 0;
};
Line.prototype.equations = function() {
  let intercept = {
    y: Line.prototype.yIntercept.call(this),
    x: Line.prototype.xIntercept.call(this)
  };
  let equations = [];
  for(let axis in intercept) {
    if(intercept[axis]) {
      let [c0, m] = intercept[axis];
      let rhs = `${c0}`;
      if(m !== 0) rhs += ` + ${m} * ${axis == 'y' ? 'x' : 'y'}`;
      equations.push(`${axis} = ${rhs}`);
    }
  }
  return equations;
};
Line.prototype.functions = function() {
  let i;
  let fns = {};
  if((i = Line.prototype.yIntercept.call(this))) {
    let [y0, myx] = i;
    fns.y = x => y0 + myx * x;
  } else {
    let { y } = this.a;
    fns.y = new Function('x', `return ${y}`);
  }
  if((i = Line.prototype.xIntercept.call(this))) {
    let [x0, mxy] = i;
    fns.x = y => x0 + mxy * y;
  } else {
    let { x } = this.a;
    fns.x = new Function('y', `return ${x}`); //y => x;
  }
  return fns;
};
Line.prototype.angle = function() {
  return Point.prototype.angle.call(Line.prototype.getVector.call(this));
};
Line.prototype.getLength = function() {
  const { a, b } = this;
  const { x1, y1, x2, y2 } = this;
  //console.log("a:",a, " b:",b);
  //console.log('a:', a, ' b:', b);
  //console.log('this:', this);
  return Point.prototype.distance.call(a, b);
};
Line.prototype.endpointDist = function(point) {
  return Math.min(point.distance(this.a), point.distance(this.b));
};
Line.prototype.matchEndpoints = function(arr) {
  const { a, b } = this;
  return [...arr.entries()].filter(([i, otherLine]) => !Line.prototype.equals.call(this, otherLine) && (Point.prototype.equals.call(a, otherLine.a) || Point.prototype.equals.call(b, otherLine.b) || Point.prototype.equals.call(b, otherLine.a) || Point.prototype.equals.call(a, otherLine.b)));
};

Line.prototype.distanceToPointSquared = function(p) {
  const { a, b } = this;
  let l2 = Point.prototype.distanceSquared.call(a, b);
  if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
  let t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
  t = Math.max(0, Math.min(1, t));
  return Point.prototype.distanceSquared.call(p, new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)));
};
Line.prototype.distanceToPoint = function(p) {
  return Math.sqrt(Line.prototype.distanceToPointSquared.call(this, p));
};

Object.defineProperty(Line.prototype, 'len', {
  get: Line.prototype.getLength
});
Object.defineProperty(Line.prototype, 'cross', {
  get() {
    const { x1, x2, y1, y2 } = this;
    return x1 * y2 - y1 * x2;
  }
});
Object.defineProperty(Line.prototype, 'dot', {
  get() {
    const { x1, x2, y1, y2 } = this;
    return x1 * x2 + y1 * y2;
  }
});

Line.prototype.pointAt = function(pos) {
  return Point.interpolate(...this.toPoints(), pos);

  //return new Point(pos * (this.x2 - this.x1) + this.x1, pos * (this.y2 - this.y1) + this.y1);
};
Line.prototype.transform = function(m) {
  this.a = this.a.transform(m);
  this.b = this.b.transform(m);

  if(round) Line.prototype.round.call(this, 1e-13, 13);

  return this;
};
Line.prototype.bbox = function() {
  const { x1, y1, x2, y2 } = this;
  return new BBox(x1, y1, x2, y2);
};

Line.prototype.points = function() {
  const { a, b } = this;
  return [a, b];
};

Line.prototype[Symbol.for('nodejs.util.inspect.custom')] = function(n, options = {}) {
  const { x1, y1, x2, y2 } = this;
  return this[Symbol.toStringTag] + ' ' + Util.inspect({ x1, y1, x2, y2 }, options) + ' }';
};
Line.prototype.toString = function(opts = {}) {
  let { separator = ', ', brackets, pad = 6, ...options } = opts;

  if(typeof brackets != 'function') brackets = brackets ? s => `[ ${s} ]` : s => s;

  const { x1, y1, x2, y2 } = this;
  return (
    brackets(
      Point.toString(this.a || Point(x1, y1), {
        ...options,
        /*separator,*/ pad: 0
      })
    ) +
    separator +
    brackets(
      Point.toString(this.b || Point(x2, y2), {
        ...options,
        /*separator,*/ pad: 0
      })
    )
  );
};
Line.prototype.toSource = function() {
  const { x1, y1, x2, y2 } = this;
  return `new Line(${[x1, y1, x2, y2].join(',')})`;
};
Line.prototype.reverse = function() {
  const { a, b } = this;
  this.b = a;
  this.a = b;
  if(this.curve !== undefined) this.curve = -this.curve;
  if(this.width !== undefined) this.width = this.width;
  return this;
};
Line.prototype.toObject = function(t = num => num) {
  const { x1, y1, x2, y2 } = this;
  const obj = { x1: t(x1), y1: t(y1), x2: t(x2), y2: t(y2) };
  //Object.setPrototypeOf(obj, Line.prototype);
  return obj;
};
Line.prototype.clone = function() {
  const ctor = this.constructor[Symbol.species];
  const { x1, y1, x2, y2 } = this;
  let ret = new ctor(x1, y1, x2, y2);

  if(this.curve !== undefined) ret.curve = this.curve;

  return ret;
};

Line.prototype.round = function(precision = 0.001, digits, type) {
  let { x1, y1, x2, y2 } = this;
  this.x1 = Util.roundTo(x1, precision, digits, type);
  this.y1 = Util.roundTo(y1, precision, digits, type);
  this.x2 = Util.roundTo(x2, precision, digits, type);
  this.y2 = Util.roundTo(y2, precision, digits, type);
  return this;
};
Line.prototype.sum = function(...args) {
  let r = new Line(...this);
  return Line.prototype.add.call(r, ...args);
};
Line.prototype.add = function(...args) {
  let other;
  if((other = Line(...args))) {
    this.x1 += other.x1;
    this.y1 += other.y1;
    this.x2 += other.x2;
    this.y2 += other.y2;
  } else if((other = Point(...args))) {
    this.x1 += other.x;
    this.y1 += other.y;
    this.x2 += other.x;
    this.y2 += other.y;
  }
  return this;
};
Line.prototype.diff = function(...args) {
  let r = new Line(...this);
  return Line.prototype.sub.call(r, ...args);
};
Line.prototype.sub = function(...args) {
  let other;
  if((other = Line(...args))) {
    this.x1 -= other.x1;
    this.y1 -= other.y1;
    this.x2 -= other.x2;
    this.y2 -= other.y2;
  } else if((other = Point(...args))) {
    this.x1 -= other.x;
    this.y1 -= other.y;
    this.x2 -= other.x;
    this.y2 -= other.y;
  }
  return this;
};
Line.prototype.prod = function(...args) {
  let r = new Line(...this);
  return Line.prototype.mul.call(r, ...args);
};
Line.prototype.mul = function(...args) {
  const o = args.length == 1 && typeof args[0] == 'number' ? { x: args[0], y: args[0] } : new Point(...args);
  this.x1 *= o.x;
  this.y1 *= o.y;
  this.x2 *= o.x;
  this.y2 *= o.y;
  return this;
};
Line.prototype.quot = function(...args) {
  let r = new Line(...this);
  return Line.prototype.div.call(r, ...args);
};
Line.prototype.div = function(...args) {
  const o = args.length == 1 && typeof args[0] == 'number' ? { x: args[0], y: args[0] } : new Point(...args);
  this.x1 /= o.x;
  this.y1 /= o.y;
  this.x2 /= o.x;
  this.y2 /= o.y;
  return this;
};
Line.prototype.some = function(pred) {
  return pred(this.a) || pred(this.b);
};
Line.prototype.every = function(pred) {
  return pred(this.a) && pred(this.b);
};
Line.prototype.includes = function(point) {
  return Point.prototype.equals.call(this.a, point) || Point.prototype.equals.call(this.b, point);
};
Line.prototype.equals = function(...args) {
  let other = Line(...args);
  if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b)) return 1;
  if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a)) return -1;
  return 0;
};
Line.prototype.indexOf = function(point) {
  let i = 0;
  for(let p of [this.a, this.b]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }
  return -1;
};
Line.prototype.lastIndexOf = function(point) {
  let i = 0;
  for(let p of [this.b, this.a]) {
    if(Point.prototype.equals.call(p, point)) return i;
    i++;
  }
  return -1;
};
Line.prototype.map = function(fn) {
  let i = 0;
  let r = [];
  for(let p of [this.a, this.b]) {
    r.push(fn(p, i, this));
    i++;
  }
  return new Line(...r);
};
Line.prototype.swap = function(fn) {
  let line = new Line(this.b, this.a);
  if(this.curve !== undefined) line.curve = -this.curve;
  if(this.width !== undefined) line.width = this.width;
  return line;
};
Line.prototype.toPoints = function(ctor = Array.of) {
  const { x1, y1, x2, y2 } = this;
  return ctor({ x: x1, y: y1 }, { x: x2, y: y2 });
};
Line.prototype[Symbol.iterator] = function() {
  const { x1, y1, x2, y2 } = this;
  return [x1, y1, x2, y2][Symbol.iterator]();
};

for(let name of ['direction', 'round', 'slope', 'angle', 'bbox', 'points', 'inspect', 'toString', 'toObject', 'toSource', 'distanceToPointSquared', 'distanceToPoint']) {
  Line[name] = (line, ...args) => Line.prototype[name].call(line || new Line(line), ...args);
}

Util.defineInspect(Line.prototype, 'x1', 'y1', 'x2', 'y2');

Line.a = Util.memoize(line => Point.bind(line, ['x1', 'y1']), new WeakMap());
Line.b = Util.memoize(line => Point.bind(line, ['x2', 'y2']), new WeakMap());
Line.from = obj => {
  let l = new Line(obj);

  for(let extra of ['curve', 'width']) {
    if(typeof obj[extra] == 'number') l[extra] = obj[extra];
    else if(typeof obj[extra] == 'string' && !isNaN(+obj[extra])) l[extra] = +obj[extra];
  }
  return l;
};

Line.bind = (o, p, gen) => {
  const [x1, y1, x2, y2] = p || ['x1', 'y1', 'x2', 'y2'];
  if(!gen) gen = k => v => v === undefined ? o[k] : (o[k] = v);

  let proxy = { a: Point.bind(o, [x1, y1]), b: Point.bind(o, [x2, y2]) };
  Util.bindProperties(proxy, o, { x1, y1, x2, y2 }, gen);
  return Object.setPrototypeOf(proxy, Line.prototype);
};

Util.defineGetter(Line, Symbol.species, function() {
  return this;
});

const ImmutableLine = Util.immutableClass(Line);
Util.defineGetter(ImmutableLine, Symbol.species, () => ImmutableLine);
/* ---------------------------- end of 'line.js' ---------------------------- */

/* -------------------------- start of 'align.js' --------------------------- */
function Align(arg) {}

Align.CENTER = 0;
Align.LEFT = 1;
Align.RIGHT = 2;

Align.MIDDLE = 0;
Align.TOP = 4;
Align.BOTTOM = 8;

Align.horizontal = alignment => alignment & (Align.LEFT | Align.RIGHT);
Align.vertical = alignment => alignment & (Align.TOP | Align.BOTTOM);

function AlignToString(value) {
  return [['CENTER', 'LEFT', 'RIGHT'][value & 0x3], ['MIDDLE', 'TOP', 'BOTTOM'][(value >> 2) & 0x03]];
}

const Anchor = Align;
/* --------------------------- end of 'align.js' ---------------------------- */

function Rect(arg) {
  let obj = this instanceof Rect ? this : {};
  let args = arg instanceof Array ? arg : [...arguments];
  let ret;

  if(typeof args[0] == 'number') arg = args;
  else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

  ['x', 'y', 'width', 'height'].forEach(field => {
    if(typeof obj[field] != 'number') obj[field] = 0;
  });

  if(arg && arg.x1 !== undefined && arg.y1 !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x1, y1, x2, y2 } = arg;
    obj.x = x1;
    obj.y = y1;
    obj.width = x2 - x1;
    obj.height = y2 - y1;
    ret = 1;
  } else if(arg && arg.x !== undefined && arg.y !== undefined && arg.x2 !== undefined && arg.y2 !== undefined) {
    const { x, y, x2, y2 } = arg;
    obj.x = x;
    obj.y = y;
    obj.width = x2 - x;
    obj.height = y2 - y;
    ret = 1;
  } else if(isPoint(arg) && arg.y !== undefined && arg.width !== undefined && arg.height !== undefined) {
    obj.x = parseFloat(arg.x);
    obj.y = parseFloat(arg.y);
    obj.width = parseFloat(arg.width);
    obj.height = parseFloat(arg.height);
    ret = 1;
  } else if(arg && arg.length >= 4 && arg.slice(0, 4).every(arg => !isNaN(parseFloat(arg)))) {
    let x = arg.shift();
    let y = arg.shift();
    let w = arg.shift();
    let h = arg.shift();
    obj.x = typeof x === 'number' ? x : parseFloat(x);
    obj.y = typeof y === 'number' ? y : parseFloat(y);
    obj.width = typeof w === 'number' ? w : parseFloat(w);
    obj.height = typeof h === 'number' ? h : parseFloat(h);
    ret = 4;
  } else if(arg && arg.length >= 2 && arg.slice(0, 2).every(arg => !isNaN(parseFloat(arg)))) {
    obj.x = 0;
    obj.y = 0;
    obj.width = typeof arg[0] === 'number' ? arg[0] : parseFloat(arg[0]);
    obj.height = typeof arg[1] === 'number' ? arg[1] : parseFloat(arg[1]);
    ret = 2;
  } else if(arg instanceof Array) {
    let argc;
    let argi = 0;
    if(arg.length >= 4) {
      argc = typeof x == 'number' ? 2 : 1;
      Point.apply(obj, arg.slice(0, argc));
      argi = argc;
    }
    argc = typeof arg[argi] == 'number' ? 2 : 1;
    Size.apply(obj, arg.slice(argi, argc));
    ret = argi + argc;
  }

  if(typeof obj.x != 'number' || isNaN(obj.x)) obj.x = 0;
  if(typeof obj.y != 'number' || isNaN(obj.y)) obj.y = 0;
  if(typeof obj.width != 'number' || isNaN(obj.width)) obj.width = 0;
  if(typeof obj.height != 'number' || isNaN(obj.height)) obj.height = 0;

  /*  if(obj.round === undefined) {
    Object.defineProperty(obj, 'round', {
      value: function() {
        return Rect.round(this);
      },
      enumerable: true,
      writable: false
    });
  }*/
  return obj;
  if(!(this instanceof Rect) || new.target === undefined) return obj;
}
Rect.prototype = {
  ...Size.prototype,
  ...Point.prototype,
  ...Rect.prototype
};
Rect.fromString = str => {
  const matches = [...Util.matchAll(/[-.\d]+/g, str)];
  return new Rect(...matches.map(m => +m[0]));
};
Rect.prototype[Symbol.toStringTag] = 'Rect';

Rect.prototype.clone = function(fn) {
  const ctor = this.constructor[Symbol.species] || this.constructor;
  let ret = new ctor(this.x, this.y, this.width, this.height);
  if(fn) fn(ret);
  return ret;
};

Rect.prototype.corners = function() {
  const rect = this;
  return [
    { x: rect.x, y: rect.y },
    { x: rect.x + rect.width, y: rect.y },
    { x: rect.x + rect.width, y: rect.y + rect.height },
    { x: rect.x, y: rect.y + rect.height }
  ];
};

if(Rect.prototype.isSquare === undefined) {
  Rect.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
  };
}
Rect.prototype.constructor = Rect;
Rect.prototype.getArea = function() {
  return this.width * this.height;
};
Rect.prototype.toSource = function(opts = {}) {
  const { color = true } = opts;
  const c = Util.coloring(color);
  const { x, y, width, height } = this;
  return c.concat(c.text('new', 1, 31), c.text('Rect', 1, 33), `(${x},${y},${width},${height})`);
};

Object.defineProperty(Rect.prototype, 'x1', {
  get() {
    return this.x;
  },
  set(value) {
    const extend = this.x - value;
    this.width += extend;
    this.x -= extend;
  },
  enumerable: true
});
Object.defineProperty(Rect.prototype, 'x2', {
  get() {
    return this.x + this.width;
  },
  set(value) {
    this.width = value - this.x;
  },
  enumerable: true
});
Object.defineProperty(Rect.prototype, 'y1', {
  get() {
    return this.y;
  },
  set(value) {
    const extend = this.y - value;
    this.height += extend;
    this.y -= extend;
  }
});
Object.defineProperty(Rect.prototype, 'y2', {
  get() {
    return this.y + this.height;
  },
  set(value) {
    this.height = value - this.y;
  }
});
Object.defineProperty(Rect.prototype, 'area', {
  get() {
    return Rect.prototype.getArea.call(this);
  }
});

const getSize = Util.memoize(rect =>
  Util.bindProperties(new Size(0, 0), rect, ['width', 'height'], k => {
    // console.log('gen', { k });
    return v => {
      return v !== undefined ? (rect[k] = v) : rect[k];
    };
  })
);

const getPoint = Util.memoize(rect => Util.bindProperties(new Point(0, 0), rect, ['x', 'y'], k => v => v !== undefined ? (rect[k] = v) : rect[k]));

Object.defineProperty(Rect.prototype, 'center', {
  get() {
    return Rect.center(this);
  }
});
Rect.prototype.getSize = Util.memoize;
Object.defineProperty(Rect.prototype, 'size', {
  get() {
    let ret = getSize(this);
    //console.log('getSize( ) =', ret);
    return ret;
  }
});
Object.defineProperty(Rect.prototype, 'point', {
  get() {
    let ret = getPoint(this);
    //console.log('getPoint( ) =', ret);
    return ret;
  }
});
/*Object.defineProperty(Rect.prototype, 'size', {
  get() {
    const rect = this;
    const size = new Size(rect.width, rect.height);
    Object.defineProperties(size, {
      width: {
        get() {
          return rect.width;
        },
        set(value) {
          return (rect.width = +value);
        },
        enumerable: true
      },
      height: {
        get() {
          return rect.height;
        },
        set(value) {
          return (rect.height = +value);
        },
        enumerable: true
      }
    });
    return size;
  }
});*/
Rect.prototype.points = function(ctor = items => Array.from(items)) {
  const c = this.corners();
  return ctor(c);
};
Rect.prototype.toCSS = Rect.toCSS;
Rect.prototype.equals = function(...args) {
  return Point.prototype.equals.call(this, ...args) && Size.prototype.equals.call(this, ...args);
};
Rect.prototype.scale = function(factor) {
  let width = this.width * factor;
  let height = this.height * factor;

  this.x += (width - this.width) / 2;
  this.y += (height - this.height) / 2;
  this.width = width;
  this.height = height;
  return this;
};
Rect.prototype.mul = function(...args) {
  Point.prototype.mul.call(this, ...args);
  Size.prototype.mul.call(this, ...args);
  return this;
};

Rect.prototype.div = function(...args) {
  Point.prototype.div.call(this, ...args);
  Size.prototype.div.call(this, ...args);
  return this;
};
Rect.prototype.outset = function(trbl) {
  if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
  this.x -= trbl.left;
  this.y -= trbl.top;
  this.width += trbl.left + trbl.right;
  this.height += trbl.top + trbl.bottom;
  return this;
};
Rect.prototype.inset = function(trbl) {
  if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
  if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
    this.x += trbl.left;
    this.y += trbl.top;
    this.width -= trbl.left + trbl.right;
    this.height -= trbl.top + trbl.bottom;
  }
  return this;
};
Rect.prototype.inside = function(point) {
  return Rect.inside(this, point);
};
Rect.CONTAIN = 16;
Rect.COVER = 32;

Rect.prototype.fit = function(other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
  let factors = Size.prototype.fitFactors.call(this, new Size(other)).sort((a, b) => a - b);
  // console.log('Rect.prototype.fit:', this, ...factors, { factors, other, align });

  let rects = factors.reduce((acc, factor) => {
    let rect = new Rect(0, 0, this.width, this.height);
    rect.width *= factor;
    rect.height *= factor;

    rect.align(other, align & 0x0f);

    acc.push(rect);
    return acc;
  }, []);

  //console.log('rects:', rects);

  return rects;
};

Rect.prototype.pointFromCenter = function(point) {
  Point.prototype.sub.call(point, this.center);
  point.x /= this.width;
  point.y /= this.height;
  return point;
};
Rect.prototype.toCSS = function() {
  return {
    ...Point.prototype.toCSS.call(this),
    ...Size.prototype.toCSS.call(this)
  };
};

Rect.prototype.toSVG = function(factory, attrs = { stroke: '#000', fill: 'none' }, parent = null, prec) {
  const { x, y, width, height } = this;
  if(!factory) factory = SVG.factory(document.body);

  console.log('Rect.toSVG', factory);

  return factory('rect', { ...attrs, x, y, width, height }, parent, prec);
};

Rect.prototype.toTRBL = function() {
  return {
    top: this.y,
    right: this.x + this.width,
    bottom: this.y + this.height,
    left: this.x
  };
};
Rect.prototype.toArray = function() {
  const { x, y, width, height } = this;
  return [x, y, width, height];
};
Rect.prototype.toPoints = function(...args) {
  let ctor = Util.isConstructor(args[0])
    ? (() => {
        let arg = args.shift();
        return points => new arg(points);
      })()
    : points => Array.from(points);
  let num = typeof args[0] == 'number' ? args.shift() : 4;
  const { x, y, width, height } = this;
  let a = num == 2 ? [new Point(x, y), new Point(x + width, y + height)] : [new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)];
  return ctor(a);
};
Rect.prototype.toLines = function(ctor = lines => Array.from(lines, points => new Line(...points))) {
  let [a, b, c, d] = Rect.prototype.toPoints.call(this);
  return ctor([
    [a, b],
    [b, c],
    [c, d],
    [d, a]
  ]);
};
Rect.prototype.align = function(align_to, a = 0) {
  const xdiff = (align_to.width || 0) - this.width;
  const ydiff = (align_to.height || 0) - this.height;
  let oldx = this.x;
  let oldy = this.y;

  switch (Align.horizontal(a)) {
    case Align.LEFT:
      this.x = align_to.x;
      break;
    case Align.RIGHT:
      this.x = align_to.x + xdiff;
      break;
    default:
      this.x = align_to.x + xdiff / 2;
      break;
  }
  switch (Align.vertical(a)) {
    case Align.TOP:
      this.y = align_to.y;
      break;
    case Align.BOTTOM:
      this.y = align_to.y + ydiff;
      break;
    default:
      this.y = align_to.y + ydiff / 2;
      break;
  }

  /*  this.tx = this.x - oldx;
  this.ty = this.y - oldy;*/
  return this;
};

Rect.prototype.round = function(precision = 0.001, digits, type) {
  let { x1, y1, x2, y2 } = this.toObject(true);
  let a = Point.round({ x: -x1, y: -y1 }, precision, digits, type);
  let b = Point.round({ x: x2, y: y2 }, precision, digits, type);
  this.x = -a.x;
  this.y = -a.y;
  this.width = b.x - this.x;
  this.height = b.y - this.y;
  return this;
};
Rect.prototype.toObject = function(bb = false) {
  if(bb) {
    const { x1, y1, x2, y2 } = this;
    return { x1, y1, x2, y2 };
  }
  const { x, y, width, height } = this;
  return { x, y, width, height };
};
Rect.prototype.bbox = function() {
  return this.toObject(true);
};

Rect.prototype.transform = function(m) {
  if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
  Matrix.prototype.transform_rect.call(m, this);

  // if(round) Rect.prototype.round.call(this, 1e-13, 13);
  return this;
};

Rect.prototype[Symbol.iterator] = function* () {
  let { x, y, width, height } = this;
  for(let prop of [x, y, width, height]) yield prop;
};
Rect.prototype[Util.inspectSymbol] = function(depth, options) {
  const { x, y, width, height } = this;
  return /*Object.setPrototypeOf*/ { x, y, width, height } /*, { [Symbol.toStringTag]: 'Rect' }*/;
};
Rect.isBBox = rect => !(rect instanceof Rect) && ['x1', 'x2', 'y1', 'y2'].every(prop => prop in rect);
Rect.assign = (to, rect) => Object.assign(to, new Rect(rect).toObject(Rect.isBBox(to)));
Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
Rect.toCSS = rect => Rect.prototype.toCSS.call(rect);

Rect.round = (rect, ...args) => Rect.assign(rect, new Rect(rect).round(...args));
Rect.inset = (rect, trbl) => Rect.assign(rect, new Rect(rect).inset(trbl));
Rect.outset = (rect, trbl) => Rect.assign(rect, new Rect(rect).outset(trbl));

Rect.center = rect => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
Rect.bind = rect => {
  let obj = new Rect();
};

Rect.inside = (rect, point) => point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
Rect.from = function(obj) {
  //const { x1,y1,x2,y2 } = obj;
  const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];

  const h = fn(obj.x1, obj.x2);
  const v = fn(obj.y1, obj.y2);

  const [x1, x2, y1, y2] = [...h, ...v];

  return new Rect(x1, y1, x2 - x1, y2 - y1); //h[0], v[0], h[1] - h[0], v[1] - v[0]);
};

Rect.fromCircle = function(...args) {
  const { x, y } = Point(args);
  const radius = args.shift();

  return new Rect(x - radius, y - radius, radius * 2, radius * 2);
};

for(let name of [
  'clone',
  'corners',
  'isSquare',
  'getArea',
  // 'toString',
  //'toSource',
  'points',
  'toCSS',
  'toTRBL',
  'toPoints',
  'equals'
]) {
  Rect[name] = (rect, ...args) => Rect.prototype[name].call(rect || new Rect(rect), ...args);
}

Rect.toSource = (rect, opts = {}) => {
  const { sep = ', ', inner = false, spc = ' ', colon = ':' } = opts;
  let props = `x${colon}${spc}${rect.x}${sep}y${colon}${spc}${rect.y}${sep}width${colon}${spc}${rect.width}${sep}height${colon}${spc}${rect.height}`;
  if(inner) return props;
  return `{${sep}${props}${sep}}`;
};

Rect.bind = (...args) => {
  const [o, p, gen = k => v => v === undefined ? o[k] : (o[k] = v)] = args[0] instanceof Rect ? [new Rect(), ...args] : args;

  const [x, y, width, height] = p || ['x', 'y', 'width', 'height'];
  let pt = Point.bind(o, ['x', 'y'], gen);
  let sz = Size.bind(o, ['width', 'height'], gen);
  let proxy = new Rect(pt, sz);
  return proxy;
};

Rect.scale = Util.curry((rect, sx, sy) => Matrix.scale(sx, sy).transform_rect(rect));
Rect.resize = Util.curry((rect, width, height) => {
  rect.width = width;
  rect.height = height;
  return rect;
});
Rect.translate = Util.curry((rect, x, y) => Matrix.translate(f, f).transform_rect(rect));

for(let f of ['scale', 'resize', 'translate']) {
  Rect.prototype[f] = function(...args) {
    Rect[f](this, ...args);
    return this;
  };
}

const isRect = (rect, testFn = (prop, name, obj) => name in obj) => Util.isObject(rect) && ['x', 'y', 'width', 'height'].every(n => testFn(rect[n], n, rect));

Util.defineGetter(Rect, Symbol.species, function() {
  return this;
});

const ImmutableRect = Util.immutableClass(Rect);

delete ImmutableRect[Symbol.species];

Util.defineGetter(ImmutableRect, Symbol.species, () => ImmutableRect);

Rect.prototype.toString = function(opts = {}) {
  if(typeof opts == 'string') opts = { separator: opts };
  const { precision = 0.001, unit = '', separator = ' ', left = '', right = '' } = opts;
  let { x, y, width, height } = this;
  let props = [x, y, width, height];
  return left + props.map(p => p + unit).join(' ') + right;
};
/* ---------------------------- end of 'rect.js' ---------------------------- */

/**
 * Type for TopRightBottomLeft (paddings and margins)
 *
 * @param {string,object,array} arg [description]
 */
function TRBL(arg) {
  let ret = this instanceof TRBL ? this : {};
  let args = [...arguments];
  // console.log("TRBL",{arg})

  if(typeof arg === 'object' && !Util.isArray(arg)) {
    Object.keys(arg).forEach(k => {
      const matches = /(top|right|bottom|left)/i.exec(k);
      //console.log("TRBL.constructor",{arg,matches,k});
      ret[matches[0].toLowerCase()] = parseInt(arg[k]);
    });
  } else if(arg) {
    if(args.length > 1) arg = args;

    if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
    else if(arg.length == 4) arg = arg.map(v => parseInt(v.replace(/[a-z]*$/g, '')));

    ret.top = arg[0];
    ret.right = arg[1];
    ret.bottom = arg[2];
    ret.left = arg[3];
  }

  if(isNaN(ret.top)) ret.top = 0;
  if(isNaN(ret.right)) ret.right = 0;
  if(isNaN(ret.bottom)) ret.bottom = 0;
  if(isNaN(ret.left)) ret.left = 0;

  /*   ['toString','toSource'].forEach((name) =>
    Object.defineProperty(ret, name, { enumerable: true, value: TRBL.prototype[name] })
  ); */

  //console.log('ret: ', ret);

  if(!this || this === TRBL) return Object.assign(ret, TRBL.prototype);
}

TRBL.prototype.null = function() {
  return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
};
TRBL.null = trbl => TRBL.prototype.null.call(trbl);

TRBL.neg = (trbl = this) => ({
  top: -trbl.top,
  right: -trbl.right,
  bottom: -trbl.bottom,
  left: -trbl.left
});

TRBL.prototype.isNaN = function() {
  return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
};
Object.defineProperty(TRBL.prototype, 'inset', {
  get() {
    return rect => Rect.inset(rect, this);
  }
});

Object.defineProperty(TRBL.prototype, 'outset', {
  get() {
    return rect => Rect.outset(rect, this);
  }
});

/*TRBL.prototype.outset = function() {
  return this.inset.call(TRBL.neg(this));
};*/

TRBL.prototype.add = function(other) {
  this.top += other.top;
  this.right += other.right;
  this.bottom += other.bottom;
  this.left += other.left;
};

TRBL.prototype.union = function(other) {
  this.top = other.top < this.top ? other.top : this.top;
  this.right = other.right > this.right ? other.right : this.right;
  this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
  this.left = other.left < this.left ? other.left : this.left;
};

TRBL.prototype.toRect = function() {
  return new Rect({
    x: this.left,
    y: this.top,
    width: this.right - this.left,
    height: this.bottom - this.top
  });
};
TRBL.prototype.toRect = function() {
  return new Rect({
    x: this.left,
    y: this.top,
    width: this.right - this.left,
    height: this.bottom - this.top
  });
};

TRBL.union = (trbl, other) => ({
  top: other.top < trbl.top ? other.top : trbl.top,
  right: other.right > trbl.right ? other.right : trbl.right,
  bottom: other.bottom > trbl.bottom ? other.bottom : trbl.bottom,
  left: other.left < trbl.left ? other.left : trbl.left
});

TRBL.toRect = trbl => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

TRBL.prototype.toString = function(unit = 'px') {
  return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
};
TRBL.prototype.toSource = function() {
  return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
};

for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
  TRBL[name] = points => TRBL.prototype[name].call(points);
}

function isTRBL(obj) {
  return top in obj && right in obj && bottom in obj && left in obj;
}

Util.defineGetter(TRBL, Symbol.species, function() {
  return this;
});
const ImmutableTRBL = Util.immutableClass(TRBL);
Util.defineGetter(ImmutableTRBL, Symbol.species, () => ImmutableTRBL);
/* ---------------------------- end of 'trbl.js' ---------------------------- */

/* ------------------------- start of 'iterator.js' ------------------------- */
//Example usage:
//
//void async function() {
//let [clicks, onclick] = iterator()
//document.querySelector('button').addEventListener('click', onclick)
//for await (let click of clicks) console.log(click)
//}()

function iterator() {
  let done = false;
  let events = [];
  let resolve;
  let promise;

  defer();

  return [read(), write, end];

  function defer() {
    promise = new Promise(r => (resolve = r));
  }

  async function* read() {
    await promise;
    yield* events.splice(0);
    if(!done) yield* read();
  }

  function write(event) {
    events.push(event);
    resolve();
    defer();
  }

  function end() {
    done = true;
    resolve();
  }
}

function eventIterator(element, ...events) {
  let [iter, push, end] = iterator();

  events.forEach(name => element.addEventListener(name, push));

  iter.stop = () => {
    events.forEach(name => element.removeEventListener(name, push));
    end();
  };

  return iter;
}

iterator;
/* -------------------------- end of 'iterator.js' -------------------------- */

/**
 * Class for element.
 *
 * @class      Element (name)
 */
class Element extends Node {
  static EDGES = {
    upperLeft: 0,
    upperCenter: 0.5,
    upperRight: 1,
    centerRight: 1.5,
    lowerRight: 2,
    lowerCenter: 2.5,
    lowerLeft: 3,
    centerLeft: 3.5
  };

  static edges = arg => Element.getEdgesXYWH(Element.rect(arg));
  static Axis = { H: 0, V: 2 };

  static margin = element => Element.getTRBL(element, 'margin');
  static padding = element => Element.getTRBL(element, 'padding');
  static border = element => Element.getTRBL(element, 'border');

  static wrap(e) {
    let names;
    if(!names) names = Util.getMethodNames(Element, 1, 0);
    if(typeof e == 'string') e = Element.find(e);
    let props = names.reduce((acc, name) => {
      if(typeof acc[name] != 'function') {
        try {
          delete acc[name];
          acc[name] = function(...args) {
            args.unshift(this);
            return Element[name].call(Element, ...args);
          };
        } catch(err) {}
      }
      return acc;
    }, e);
    //console.log("props:",props);
    return e;
  }

  static create(...args) {
    if(args.length == 2 && typeof args[0] == 'string' && isElement(args[1])) {
      let ret = document.createTextNode(args[0]);
      args[1].appendChild(ret);

      return ret;
    }

    let { tagName, ns, children, ...props } = typeof args[0] == 'object' ? args.shift() : { tagName: args.shift(), ...args.shift() };
    let parent = args.shift();
    parent = typeof parent == 'string' ? Element.find(parent) : parent;

    //console.log('Element.create ', { tagName, props, parent, ns });

    let d = document || window.document;
    let e = ns ? d.createElementNS(ns, tagName) : d.createElement(tagName);
    for(let k in props) {
      const value = props[k];
      if(k == 'parent') {
        parent = props[k];
        continue;
      } else if(k == 'className') k = 'class';
      if(k == 'style' && typeof value === 'object') Element.setCSS(e, value);
      else if(k == 'innerText') e.appendChild(document.createTextNode(value));
      else if(k.startsWith('on') || k.startsWith('inner')) e[k] = value;
      else {
        //        ns ? e.setAttributeNS(ns || null, k, value) : e.setAttribute(k,value);
        e.setAttribute(k, value);
      }
    }
    if(children && children.length)
      children.forEach(obj => {
        Element.create(obj, e);
      });

    if(parent && parent.appendChild) parent.appendChild(e);
    // console.log('Element.create ', e);
    return e;
  }

  static walkUp(elem, pred = e => true) {
    if(typeof elem == 'string') elem = Element.find(elem);
    let depth = 0;
    if(typeof pred == 'number') {
      let n = pred;
      pred = (e, d) => d == n;
    }
    let ret = [];
    while(elem) {
      let value = elem;
      let finish = false;
      try {
        if(
          (pred(
            elem,
            depth,
            v => (value = v),
            stop => (finish = true)
          ) &&
            !finish) ||
          value !== elem
        )
          ret.push(value);
      } catch(err) {
        return err;
      }
      if(finish) break;
      elem = elem.parentElement;
      depth++;
    }
    return ret.length ? ret : null;
  }

  static *skip(elem, fn = (e, next) => next(e.parentElement)) {
    elem = typeof elem == 'string' ? Element.find(elem) : elem;
    //let [iter,push] = new iterator();
    let emit = n => (elem = n);

    while(elem) {
      yield elem;
      fn(elem, emit);
    }
  }

  static walk(elem, fn, accu = {}) {
    if(typeof elem == 'string') elem = Element.find(elem);
    const root = elem;
    //const rootPath = Element.xpath(elem);
    let depth = 0;
    while(elem) {
      accu = fn(this.wrap(elem), accu, root, depth);
      if(elem.firstElementChild) depth++;
      elem =
        elem.firstElementChild ||
        elem.nextElementSibling ||
        (function () {
          do {
            if(!(elem = elem.parentElement)) break;
            depth--;
          } while(depth > 0 && !elem.nextElementSibling);
          return elem && elem != root ? elem.nextElementSibling : null;
        })();
    }
    return accu;
  }

  static *iterator(elem, predicate = (e, d, r) => true, getProp) {
    if(getProp == 'node') getProp = (obj, prop) => obj[prop.replace(/Element$/, 'Node').replace(/Element/, '')];

    if(!getProp) getProp = (obj, prop) => obj[prop];
    if(typeof elem == 'string') elem = Element.find(elem);
    const root = elem;
    let depth = 0;
    while(elem) {
      if(predicate(elem, depth, root)) yield this.wrap(elem);
      if(getProp(elem, 'firstElementChild')) depth++;
      elem =
        getProp(elem, 'firstElementChild') ||
        getProp(elem, 'nextElementSibling') ||
        (function () {
          do {
            if(!(elem = getProp(elem, 'parentElement'))) break;
            depth--;
          } while(depth > 0 && !getProp(elem, 'nextElementSibling'));
          return elem && elem != root ? getProp(elem, 'nextElementSibling') : null;
        })();
    }
  }

  static *childIterator(elem, element = true) {
    element = element ? 'Element' : '';
    if(elem['first' + element + 'Child']) {
      for(let c = elem['first' + element + 'Child']; c; c = c['next' + element + 'Sibling']) yield c;
    } else {
      let children = [...elem.children];
      for(let i = 0; i < children.length; i++) yield children[i];
    }
  }

  static fromObject(obj, parent) {
    const { tagName, attributes = {}, children = [] } = obj;

    let element = Element.create(tagName, attributes, parent);

    for(let child of children) {
      if(typeof child != 'object') {
        element.appendChild(document.createTextNode(child));
      } else {
        if(!child.tagName) console.log('child:', child);
        Element.fromObject(child, element);
      }
    }
    return element;
  }

  static toObject(elem, opts = {}) {
    const { no_children = false, predicate } = opts;
    if(typeof elem == 'string') elem = Element.find(elem);
    let l = [];
    if(!no_children) {
      l = [...this.childIterator(elem, false)];
      if(predicate) l = l.filter(predicate);
      l = l.reduce((l, c) => (Util.isObject(c) && c.nodeType == 1 ? l.push(Element.toObject(c, opts)) : (c.textContent + '').trim() != '' ? l.push(c.textContent) : undefined, l), []);
    }

    let attributes = (opts ? opts.namespaceURI : document.body.namespaceURI) != elem.namespaceURI ? { ns: elem.namespaceURI } : {};
    let a = 'length' in elem.attributes ? Element.attr(elem) : elem.attributes;
    for(let key in a) attributes[key] = '' + a[key];
    return {
      tagName: /[a-z]/.test(elem.tagName) ? elem.tagName : elem.tagName.toLowerCase(),
      ...attributes,
      ...(l.length > 0 ? { children: l } : {})
    };
  }

  static toCommand(elem, opts = {}) {
    let { parent = '', varName, recursive = true, cmd = 'Element.create', quote = "'" } = opts;
    let o = Element.toObject(elem, { children: false });
    let s = '';
    let { tagName, ns, children, ...attributes } = o;
    let v = '';
    s = Object.keys(ns ? { ns, ...attributes } : attributes)
      .map(k => '' + k + ':' + quote + '' + attributes[k] + '' + quote + '')
      .join(', ');
    s = '' + cmd + '(' + tagName + ', {' + s + '';
    let c = elem.children;
    if(c.length >= 1) s = '' + s + ', [\n  ' + c.map(e => Element.toCommand(e, opts).replace(/\n/g, '\n  ')).join(',\n  ') + '\n]';
    s += parent ? ', ' + parent : '';
    if(elem.firstElementChild && varName) {
      v = parent ? String.fromCharCode(parent.charCodeAt(0) + 1) : varName;
      s = '' + v + ' = ' + s + '';
    }
    return s.replace(new RegExp(';*$', 'g'), '');
  }

  static find(arg, parent, globalObj = Util.getGlobalObject()) {
    if(typeof parent == 'string') parent = Element.find(parent);
    if(!parent && globalObj.document)
      parent =
        globalObj.document ||
        Util.tryCatch(
          () => document,
          d => d
        );

    if(typeof arg != 'string') throw new Error(arg + '');

    if(arg.startsWith('/')) arg = arg.substring(1).replace(/\//g, ' > ');

    return parent.querySelector(arg);
  }

  static findAll(arg, parent) {
    parent = typeof parent == 'string' ? Element.find(parent) : parent;
    return [...(parent && parent.querySelectorAll ? parent.querySelectorAll(arg) : document.querySelectorAll(arg))];
  }

  /**
   * Sets or gets attributes
   *
   * @param      {<type>}  element       The element
   * @param      {<type>}  [attrs=null]  The attributes
   * @return     {<type>}  { description_of_the_return_value }
   */
  static attr(e, attrs_or_name) {
    const elem = typeof e === 'string' ? Element.find(e) : e;
    //console.log('Element.attr', { elem, attrs_or_name });
    if(!Util.isArray(attrs_or_name) && typeof attrs_or_name === 'object' && elem) {
      for(let key in attrs_or_name) {
        const name = Util.decamelize(key, '-');
        const value = attrs_or_name[key];
        /*        console.log('attr(', elem, ', ', { name, key, value, }, ')')
         */ if(key.startsWith('on') && !/svg/.test(elem.namespaceURI)) elem[key] = value;
        else if(elem.setAttribute) elem.setAttribute(name, value);
        else elem[key] = value;
      }
      return elem;
    }
    if(typeof attrs_or_name === 'function') {
      attrs_or_name(elem.attributes, elem);
      return elem;
    } else if(typeof attrs_or_name === 'string') {
      attrs_or_name = [attrs_or_name];
    } else if(Util.isObject(elem) && 'getAttributeNames' in elem) {
      attrs_or_name = elem.getAttributeNames();
    } else {
      attrs_or_name = [];
      if(Util.isObject(elem) && Util.isArray(elem.attributes)) for(let i = 0; i < elem.attributes.length; i++) attrs_or_name.push(elem.attributes[i].name);
    }
    let ret = attrs_or_name.reduce((acc, name) => {
      const key = /*Util.camelize*/ name;
      const value = elem && elem.getAttribute ? elem.getAttribute(name) : elem[key];
      acc[key] = /^-?[0-9]*\.[0-9]\+$/.test(value) ? parseFloat(value) : value;
      return acc;
    }, {});
    if(typeof arguments[1] == 'string') return ret[attrs_or_name[0]];
    return ret;
  }

  static getRect(elem) {
    let e = elem;
    while(e) {
      if(e.style) {
        if(e.style.position == '') e.style.position = 'relative';
        if(e.style.left == '') e.style.left = '0px';
        if(e.style.top == '') e.style.top = '0px';
      }
      e = e.offsetParent || e.parentNode;
    }
    const bbrect = elem.getBoundingClientRect();
    //console.log('getRect: ', { bbrect });
    return {
      x: bbrect.left + window.scrollX,
      y: bbrect.top + window.scrollY,
      width: bbrect.right - bbrect.left,
      height: bbrect.bottom - bbrect.top
    };
  }

  /**
   * Gets the rectangle.
   *
   * @param      {<type>}  e
   * lement  The element
   * @return     {Object}  The rectangle.
   */
  static rect(...args) {
    let [element, options = {}] = args;
    if(args.length > 1 && (isRect(args.slice(1)) || isRect(args[1]))) return Element.setRect(...args);
    let { round = true, relative_to = null, relative = false, scroll_offset = true, border = true, margin = false } = options;
    const e = typeof element === 'string' ? Element.find(element) : element;
    if(!e || !e.getBoundingClientRect) return null; //new Rect(0, 0, 0, 0);

    const bb = e.getBoundingClientRect();

    let r = TRBL.toRect(bb);
    if(relative) relative_to = e.parentElement;

    if(relative_to && relative_to !== null /*&& Element.isElement(relative_to)*/) {
      const off = Element.rect(relative_to);
      r.x -= off.x;
      r.y -= off.y;
    }
    //console.log("Element.rect(", r, ")");

    if(!border) {
      const border = Element.border(e);
      Rect.inset(r, border);

      //console.log("Element.rect(", r, ") // with border = ", border);
    } else if(margin) {
      const margin = Element.margin(e);
      Rect.outset(r, margin);

      //console.log("Element.rect(", r, ") // with border = ", border);
    }

    const { scrollTop, scrollY } = window;
    if(scroll_offset) {
      r.y += scrollY;
    }
    r = new Rect(round ? Rect.round(r) : r);
    //console.log('Element.rect(', element, ') =', r);
    return r;
  }

  static setRect(element, rect, opts = {}) {
    let { anchor, unit = 'px', scale } = opts;
    const e = typeof element === 'string' ? Element.find(element) : element;
    //console.log("Element.setRect(", element, ",", rect, ", ", anchor, ") ");
    if(typeof anchor == 'string') {
      e.style.position = anchor;
      anchor = 0;
    }
    if(scale) Rect.scale(rect, scale, scale);

    anchor = anchor || Anchor.LEFT | Anchor.TOP;
    const position = element.style && element.style.position;

    /*|| rect.position || "relative"*/
    const pelement = position == 'fixed' ? e.documentElement || document.body : e.parentNode;
    const prect = Element.rect(pelement, { round: false });
    //Rect.align(rect, prect, anchor);

    /* const stack = Util.getCallers(3, 4);*/
    const ptrbl = Rect.toTRBL(prect);
    const trbl = Rect.toTRBL(rect);
    //console.log("Element.setRect ", { trbl, ptrbl });
    let css = {};
    let remove;
    switch (Anchor.horizontal(anchor)) {
      case Anchor.LEFT:
      default:
        css.left = Math.round(trbl.left /* - ptrbl.left*/) + unit;
        remove = 'right';
        break;
      case Anchor.RIGHT:
        css.right = Math.round(trbl.right - ptrbl.right) + unit;
        remove = 'left';
        break;
    }
    switch (Anchor.vertical(anchor)) {
      case Anchor.TOP:
      default:
        css.top = Math.round(trbl.top /* - ptrbl.top*/) + unit;
        remove = 'bottom';
        break;
      case Anchor.BOTTOM:
        css.bottom = Math.round(trbl.bottom - ptrbl.bottom) + unit;
        remove = 'top';
        break;
    }
    if(e.style) {
      if(e.style.removeProperty) e.style.removeProperty(remove);
      else e.style[remove] = undefined;
    }
    //css.position = position;
    css.width = Math.round(rect.width) + (unit || unit);
    css.height = Math.round(rect.height) + (unit || unit);
    //console.log("Element.setRect ", css);
    Element.setCSS(e, css);
    //Object.assign(e.style, css);
    //Element.setCSS(e, css);
    return e;
  }

  static position(element, edges = ['left', 'top']) {
    console.log('Element.position ', { element, edges });
    const rect = Element.rect(element);
    if(rect) {
      //if(typeof element == 'string') element = Element.find(element);
      const trbl = rect.toTRBL();
      const [x, y] = edges.map(e => (e == 'right' ? window.innerWidth - trbl[e] : e == 'bottom' ? window.innerHeight - trbl[e] : trbl[e]));
      return new Point({ x, y });
    }
  }

  static move(element, point, pos, edges = ['left', 'top']) {
    let [e, ...rest] = [...arguments];
    let { x = Element.position(element, edges).x, y = Element.position(element, edges).y } = new Point(rest);
    let to = { x, y };
    let position = rest.shift() || Element.getCSS(element, 'position') || 'relative';
    let off;
    //console.log('Element.move ', { element, to, position });
    const getValue = prop => {
      const property = Element.getCSS(element, prop);
      if(property === undefined) return undefined;
      const matches = /([-0-9.]+)(.*)/.exec(property) || [];
      //console.log({ match, value, unit });
      return parseFloat(matches[1]);
    };

    const current = new Point({
      x: getValue(edges[0]) || 0,
      y: getValue(edges[1]) || 0
    });
    off = new Point(Element.position(element, edges));
    //off = Point.diff(off, current);
    Point.add(current, Point.diff(to, off));

    /*
    if(position == 'relative') {
      to.x -= off.x;
      to.y -= off.y;
    }*/
    let css = Point.toCSS(current, 1, edges);
    console.log('Element.move: ', { position, to, css, off, current, edges });
    //console.log('move newpos: ', Point.toCSS(pt));
    Element.setCSS(element, { ...css, position });
    return element;
  }

  static moveRelative(element, to, edges = ['left', 'top'], callback) {
    let e = typeof element == 'string' ? Element.find(element) : element;
    let origin = to ? new Point(to) : Object.fromEntries(Object.entries(Element.getCSS(element, edges)).map(([k, v]) => ['xy'[edges.indexOf(k)] || k, v ? +v.replace(/[a-z]*$/, '') : 0]));
    const f = [edges[0] == 'left' ? 1 : -1, edges[1] == 'top' ? 1 : -1];
    //console.log('moveRelative', { e, to, edges, f, origin });
    function move(x, y) {
      let pos = new Point(origin.x + x * f[0], origin.y + y * f[1]);
      if(move.first === undefined) move.first = pos;
      if(typeof callback == 'function') {
        callback.call(this, pos, move.last, move.first);
        move.last = pos;
        return;
      }
      move.last = pos;

      let css = pos.toCSS(1, edges);
      //      console.log('move', { pos, css });
      Element.setCSS(e, css);
      return;
    }
    move.origin = origin;
    move.cancel = () => move.call(move, 0, 0);
    move.jump = () => Element.moveRelative(e, to, edges);
    move.element = element;
    return move;
  }

  static resize(element, ...dimensions) {
    let e = typeof element == 'string' ? Element.find(element) : element;
    let size = new Size(...dimensions);
    const css = Size.toCSS(size);
    //console.log("Element.resize: ", { e, size, css });
    Element.setCSS(e, css);
    return e;
  }

  static resizeRelative(element, to, f = 1, callback) {
    let e = typeof element == 'string' ? Element.find(element) : element;
    let origin = new Size(to || Element.rect(e));
    // console.log('resizeRelative', { e, to, origin, f });
    function resize(width, height, rel = true) {
      let size = new Size(width, height);
      if(rel) size = origin.sum(size.prod(f, f));
      let css = size.toCSS(1, ['px', 'px']);
      resize.css = css;
      //      console.log('resizeRelative', { width, height, size, css });
      if(typeof callback == 'function') callback(size, resize.last);

      resize.last = size;

      return Element.setCSS(e, css);
    }
    resize.origin = origin;
    resize.cancel = () => resize(0, 0);
    resize.jump = () => Element.resizeRelative(e, to);
    return resize;
  }

  static getEdgesXYWH({ x, y, w, h }) {
    return [
      { x, y },
      { x: x + w, y },
      { x: x + w, y: y + h },
      { x, y: y + h }
    ];
  }

  static getEdge({ x, y, w, h }, which) {
    return [
      { x, y },
      { x: x + w / 2, y },
      { x: x + w, y },
      { x: x + w, y: y + h / 2 },
      { x: x + w, y: y + h },
      { x: x + w / 2, y: y + h },
      { x, y: y + h },
      { x, y: y + h / 2 }
    ][Math.floor(which * 2)];
  }

  static getPointsXYWH({ x, y, w, h }) {
    return [
      { x, y },
      { x: x + w, y: y + h }
    ];
  }

  static cumulativeOffset(element, relative_to = null) {
    if(typeof element == 'string') element = Element.find(element);
    let p = { x: 0, y: 0 };
    do {
      p.y += element.offsetTop || 0;
      p.x += element.offsetLeft || 0;
    } while((element = element.offsetParent) && element != relative_to);
    return p;
  }

  static getTRBL(element, prefix = '') {
    if(typeof element == 'string') element = Element.find(element);

    const names = ['Top', 'Right', 'Bottom', 'Left'].map(pos => prefix + (prefix == '' ? pos.toLowerCase() : pos + (prefix == 'border' ? 'Width' : '')));
    const getCSS = prefix == '' ? () => ({}) : Util.memoize(() => Element.getCSS(element));

    let entries = names.map(prop => [Util.decamelize(prop).split('-'), element.style.getPropertyValue(prop) || getCSS()[prop]]);
    //console.log('getTRBL', { names, entries });
    entries = entries.map(([prop, value]) => [prop[1] || prop[0], typeof value == 'string' ? +value.replace(/px$/, '') : value]);

    return /*new TRBL*/ Object.fromEntries(entries);
  }

  static setTRBL(element, trbl, prefix = 'margin') {
    const attrs = ['Top', 'Right', 'Bottom', 'Left'].reduce((acc, pos) => {
      const name = prefix + (prefix == '' ? pos.toLowerCase() : pos);
      return { ...acc, [name]: trbl[pos.toLowerCase()] };
    }, {});
    //console.log('Element.setTRBL ', attrs);
    return Element.setCSS(element, attrs);
  }

  static setCSS(element, prop, value) {
    if(typeof element == 'string') element = Element.find(element);
    if(!isElement(element)) return false;
    if(typeof prop == 'string' && typeof value == 'string') prop = { [prop]: value };

    //console.log("Element.setCSS ", { element, prop });

    for(let key in prop) {
      let value = prop[key];
      const propName = Util.decamelize(key);
      if(typeof value == 'function') {
        if('subscribe' in value) {
          value.subscribe = newval => element.style.setProperty(propName, newval);
          value = value();
        }
      }
      if(element.style) {
        if(value !== undefined) {
          if(element.style.setProperty) element.style.setProperty(propName, value);
          else element.style[Util.camelize(propName)] = value;
        } else {
          if(element.style.removeProperty) element.style.removeProperty(propName);
          else delete element.style[Util.camelize(propName)];
        }
      }
    }
    return element;
  }

  static getCSS(element, property = undefined, receiver = null) {
    element = typeof element == 'string' ? Element.find(element) : element;

    const w = window !== undefined ? window : globalThis.window;
    const d = document !== undefined ? document : global.document;
    // console.log('Element.getCSS ', { element,property });

    let parent = Util.isObject(element) ? element.parentElement || element.parentNode : null;

    let style;

    let estyle = Util.tryCatch(() => (Util.isObject(w) && w.getComputedStyle ? w.getComputedStyle(element) : d.getComputedStyle(element)));
    if(property == undefined) {
      let pstyle = Util.tryCatch(() => (parent && parent.tagName ? (/*Util.toHash*/ w && w.getComputedStyle ? w.getComputedStyle(parent) : d.getComputedStyle(parent)) : {}));

      if(!estyle || !pstyle) return null;
      //let styles = [estyle,pstyle].map(s => Object.fromEntries([...Node.map(s)].slice(0,20)));

      style = Util.removeEqual(estyle, pstyle);
    } else {
      style = estyle;
    }

    if(!style) return null;
    let keys = Object.keys(style).filter(k => !/^__/.test(k));
    //console.log("style: ", style);
    // console.log("style: ", style);
    //console.log("Element.getCSS ", style);
    if(typeof property == 'string') property = Util.camelize(property);

    let ret = {};
    if(receiver == null) {
      receiver = result => {
        if(typeof result == 'object') {
          try {
            Object.defineProperty(result, 'cssText', {
              get() {
                return Object.entries(this)
                  .map(([k, v]) => `${Util.decamelize(k, '-')}: ${v};\n`)
                  .join('');
              },
              enumerable: false
            });
          } catch(err) {}
        }
        return result;
      };
    }
    if(property !== undefined) {
      ret =
        typeof property === 'string'
          ? style[property]
          : property.reduce((ret, key) => {
              ret[key] = style[key];
              return ret;
            }, {});
    } else {
      for(let i = 0; i < keys.length; i++) {
        const stylesheet = keys[i];
        const key = Util.camelize(stylesheet);
        const val = style[stylesheet] || style[key];
        if(val && val.length > 0 && val != 'none') ret[key] = val;
      }
    }
    return receiver(ret);
  }

  static xpath(elt, relative_to = null) {
    let path = '';
    let doc = elt.ownerDocument || document;
    let ns = doc.lookupNamespaceURI('');

    for(let e of this.skip(elt, (e, next) => next(e.parentElement !== relative_to && e.parentElement))) path = '/' + (e.namespaceURI != ns ? e.namespaceURI.replace(/.*\//g, '') + ':' : '') + Element.unique(e) + path;

    return path;
  }

  static selector(elt, opts = {}) {
    const { relative_to = null, use_id = false } = opts;
    let sel = '';
    for(; elt && elt.nodeType == 1; elt = elt.parentNode) {
      if(sel != '') sel = ' > ' + sel;
      let xname = Element.unique(elt, { idx: false, use_id });
      if(use_id === false) xname = xname.replace(/#.*/g, '');
      sel = xname + sel;
      if(elt == relative_to) break;
    }
    return sel;
  }
  static depth(elem, relative_to = document.body) {
    let count = 0;
    while(elem != relative_to && (elem = elem.parentNode)) count++;
    return count;
  }

  static dump(elem) {
    let str = '';
    function dumpElem(child, accu, root, depth) {
      const rect = Rect.round(Element.rect(child, elem));
      accu += '  '.repeat((depth > 0 ? depth : 0) + 1) + ' ' + Element.xpath(child, child);
      [...child.attributes].forEach(attr => (accu += ' ' + attr.name + "='" + attr.value + "'"));
      if(Rect.area(rect) > 0) accu += ' ' + Rect.toString(rect);
      ['margin', 'border', 'padding'].forEach(name => {
        let trbl = Element.getTRBL(elem, 'margin');
        if(!trbl.null()) accu += ' ' + name + ': ' + trbl + '';
      });
      return accu;
    }
    str = dumpElem(elem, '');
    str = Element.walk(
      elem.firstElementChild,
      (e, a, r, d) => {
        if(e && e.attributes) return dumpElem(e, a + '\n', r, d);
        return null;
      },
      str
    );
    return str;
  }

  static skipper(fn, pred = (a, b) => a.tagName == b.tagName) {
    return function(elem) {
      let next = fn(elem);
      for(; next; next = fn(next)) if(pred(elem, next)) return next;
      return null;
    };
  }

  static prevSibling(sib) {
    return sib.previousElementSibling;
  }
  static nextSibling(sib) {
    return sib.nextElementSibling;
  }

  static idx(elt) {
    let count = 1;
    let sib = elt.previousElementSibling;
    for(; sib; sib = sib.previousElementSibling) {
      if(sib.tagName == elt.tagName) count++;
    }
    return count;
  }

  static name(elem) {
    let name = elem.tagName.toLowerCase();
    if(elem.id && elem.id.length) name += '#' + elem.id;
    else if(elem.class && elem.class.length) name += '.' + elem.class;
    return name;
  }

  static unique(elem, opts = {}) {
    const { idx = true, use_id = true } = opts;
    let name = elem.tagName.toLowerCase();
    if(use_id && elem.id && elem.id.length) return name + `[@id='${elem.id}']`;

    const classNames = [...elem.classList]; //String(elem.className).split(new RegExp("/[ \t]/"));
    if(classNames.length > 0) return name + `[@class='${elem.classList.value}']`;
    /*
    for(let i = 0; i < classNames.length; i++) {
      let res = document.getElementsByClassName(classNames[i]);
      if(res && res.length === 1) return name + `[@class~='${classNames[i]}']`;
    }*/
    if(idx) {
      if(elem.nextElementSibling || elem.previousElementSibling) {
        return name + '[' + Element.idx(elem) + ']';
      }
    }
    return name;
  }

  static factory(delegate = {}, parent = null) {
    let root = parent;
    if(root === null) {
      if(typeof delegate.append_to !== 'function') {
        root = delegate;
        delegate = {};
      } else {
        root = 'body';
      }
    }
    const { append_to, create, setattr, setcss } = delegate;
    if(typeof root === 'string') root = Element.find(root);
    if(!delegate.root) delegate.root = root;
    if(!delegate.append_to) {
      delegate.append_to = function(elem, parent) {
        if(!parent) parent = root;
        if(parent) parent.appendChild(elem);
        if(!this.root) this.root = elem;
      };
    }
    if(!delegate.create) delegate.create = tag => document.createElement(tag);
    if(!delegate.setattr) {
      delegate.setattr = (elem, attr, value) => {
        //console.log('setattr ', { attr, value });
        elem.setAttribute(attr, value);
      };
    }

    if(!delegate.setcss) delegate.setcss = (elem, css) => Object.assign(elem.style, css); //Element.setCSS(elem, css);

    delegate.bound_factory = (tag, attr = {}, parent = null) => {
      if(typeof tag == 'object') {
        const { tagName, ...a } = tag;
        attr = a;
        tag = tagName;
      }
      const { style, children, className, innerHTML, ...props } = attr;
      let elem = delegate.create(tag);
      if(style) delegate.setcss(elem, style);
      if(children && children.length) {
        for(let i = 0; i < children.length; i++) {
          if(typeof children[i] === 'string') {
            elem.innerHTML += children[i];
          } else {
            const { tagName, parent, ...childProps } = children[i];
            delegate.bound_factory(tagName, childProps, elem);
          }
        }
      }
      if(innerHTML) elem.innerHTML += innerHTML;
      if(className && elem) {
        if(elem.classList) elem.classList.add(className);
        else if(elem.attributes.class) elem.attributes.class.value += ' ' + className;
      }
      for(let k in props) delegate.setattr(elem, k, props[k]);

      /*console.log("bound_factory: ", { _this: this, tag, style, children, parent, props, to, append_to: this.append_to });*/
      if(delegate.append_to) delegate.append_to(elem, parent);
      return elem;
    };

    /*  console.log("delegate: ", delegate);*/
    /*    let proxy = function() {
      let obj = this && this.create ? this : delegate;
      return obj.bound_factory.apply(obj, arguments);
    };*/
    delegate.bound_factory.delegate = delegate;
    return delegate.bound_factory; //.bind(delegate);
  }

  static remove(element) {
    const e = typeof element === 'string' ? Element.find(element) : element;
    if(e && e.parentNode) {
      const parent = e.parentNode;
      parent.removeChild(e);
      return true;
    }
    return false;
  }

  static isat(e, x, y, options) {
    let args = [...arguments];
    let element = args.shift();
    let point = Point(args);
    const o = args[0] || { round: false };
    const rect = Element.rect(element, o);
    return Rect.inside(rect, point);
  }

  static at(x, y, options) {
    if(isElement(x)) return Element.isat.apply(Element, arguments);
    let args = [...arguments];
    const p = Point(args);
    const w = globalThis.window;
    const d = w.document;
    const s = o.all
      ? e => {
          if(ret == null) ret = [];
          ret.push(e);
        }
      : (e, depth) => {
          e.depth = depth;
          if(ret === null || depth >= ret.depth) ret = e;
        };
    let ret = null;
    return new Promise((resolve, reject) => {
      let element = null;
      Element.walk(d.body, (e, accu, root, depth) => {
        const r = Element.rect(e, { round: true });
        if(Rect.area(r) == 0) return;
        if(Rect.inside(r, p)) s(e, depth);
      });
      if(ret !== null) resolve(ret);
      else reject();
    });
  }

  /*
      e=Util.shuffle(Element.findAll('rect'))[0]; r=Element.rect(e); a=rect(r, new dom.HSLA(200,100,50,0.5));
      t=Element.transition(a, { transform: 'translate(100px,100px) scale(2,2) rotate(45deg)' }, 10000, ctx => console.log("run",ctx)); t.then(done => console.log({done}))

*/
  static transition(element, css, time, easing = 'linear', callback = null) {
    let args = [...arguments];
    const e = typeof element === 'string' ? Element.find(args.shift()) : args.shift();
    let a = [];
    const t = typeof time == 'number' ? `${time}ms` : time;
    let ctx = { e, t, from: {}, to: {}, css };
    args.shift();
    args.shift();

    easing = typeof args[0] == 'function' ? 'linear' : args.shift();
    callback = args.shift();

    for(let prop in css) {
      const name = Util.decamelize(prop);
      a.push(`${name} ${t} ${easing}`);
      ctx.from[prop] = e.style.getProperty ? e.style.getProperty(name) : e.style[prop];
      ctx.to[name] = css[prop];
    }
    const tlist = a.join(', ');

    //console.log("Element.transition", { ctx, tlist });

    let cancel;
    let ret = new Promise((resolve, reject) => {
      let trun = function(e) {
        this.event = e;
        //console.log("Element.transitionRun event", this);
        callback(this);
      };
      let tend = function(e) {
        this.event = e;
        //console.log("Element.transitionEnd event", this);
        this.e.removeEventListener('transitionend', this);
        this.e.style.setProperty('transition', '');
        delete this.cancel;
        resolve(this);
      };

      e.addEventListener('transitionend', (ctx.cancel = tend).bind(ctx));

      if(typeof callback == 'function') e.addEventListener('transitionrun', (ctx.run = trun).bind(ctx));

      cancel = () => ctx.cancel();

      if(e.style && e.style.setProperty) e.style.setProperty('transition', tlist);
      else e.style.transition = tlist;

      Object.assign(e.style, css);
    });
    ret.cancel = cancel;
    return ret;
  }

  static toString(e, opts = {}) {
    const { indent = '  ', newline = '', depth = 0 } = opts;

    let o = e.__proto__ === Object.prototype ? e : Element.toObject(e);
    const { tagName, ns, children = [], ...a } = o;
    let i = newline != '' ? indent.repeat(depth) : '';
    let s = i;

    s += `<${tagName}`;
    s += Object.entries(a)
      .map(([name, value]) => ` ${name}="${value}"`)
      .join('');
    s += children.length ? `>` : ` />`;
    if(children.length) s += newline + children.map(e => (typeof e == 'string' ? i + indent + e + newline : Element.toString(e, { ...opts, depth: depth + 1 }))).join('') + i + `</${tagName}>`;
    s += newline;
    return s;
  }

  static clipboardCopy = text =>
    new Promise((resolve, reject) => {
      if(navigator.clipboard) {
        return navigator.clipboard
          .writeText(text)
          .then(() => resolve(true))
          .catch(err => reject(err !== undefined ? err : new DOMException('The request is not allowed', 'NotAllowedError')));
      }
      let ok = false;
      try {
        const d = document,
          w = window;
        let e = d.createElement('e');
        e.textContent = text;
        e.style.whiteSpace = 'pre';
        d.body.appendChild(e);
        let s = w.getSelection();
        let r = w.d.createRange();
        s.removeAllRanges();
        r.selectNode(e);
        s.addRange(r);
        try {
          ok = w.d.execCommand('copy');
        } catch(err) {
          console.log('error', err);
        }
        s.removeAllRanges();
        w.d.body.removeChild(e);
      } catch(err) {}
      if(ok) resolve(ok);
      else reject(new DOMException('The request is not allowed', 'NotAllowedError'));
    });

  static *children(elem, tfn = e => e) {
    if(typeof elem == 'string') elem = Element.find(elem);
    for(let e = elem.firstElementChild; e; e = e.nextElementSibling) yield tfn(e);
  }

  static *recurse(elem, tfn = e => e) {
    if(typeof elem == 'string') elem = Element.find(elem);
    let root = elem;
    do {
      elem =
        elem.firstElementChild ||
        elem.nextElementSibling ||
        (function () {
          do {
            if(!(elem = elem.parentElement)) break;
          } while(!elem.nextSibling);
          return elem && elem != root ? elem.nextElementSibling : null;
        })();

      if(elem !== null) yield tfn(elem);
    } while(elem);
  }
}

function isElement(e) {
  return Util.isObject(e) && e.tagName !== undefined;
}

Object.assign(globalThis, {
  Element,
  isElement,
  Util,
  Node,
  TRBL,
  isTRBL,
  ImmutableTRBL,
  Rect,
  isRect,
  ImmutableRect,
  Point,
  isPoint,
  ImmutablePoint,
  Line,
  isLine,
  ImmutableLine,
  BBox,
  isBBox,
  Size,
  isSize,
  ImmutableSize,
  Align,
  AlignToString,
  Anchor,
  iterator,
  eventIterator
});

let CACHE_PREFIX = 'lscache-';

let CACHE_SUFFIX = '-cacheexpiration';

let EXPIRY_RADIX = 10;

function supportsStorage() {
  let key = '__lscachetest__';
  let value = key;

  if(this.cachedStorage !== undefined) {
    return this.cachedStorage;
  }

  try {
    if(!localStorage) {
      return false;
    }
  } catch(ex) {
    return false;
  }

  try {
    Implementations.localStorage.setItem.call(this, key, value);
    Implementations.localStorage.removeItem.call(this, key);
    this.cachedStorage = true;
  } catch(e) {
    if(Implementations.localStorage.supported()) {
      this.cachedStorage = true;
    } else {
      this.cachedStorage = false;
    }
  }
  return this.cachedStorage;
}

function isOutOfSpace(e) {
  return e && (e.name === 'QUOTA_EXCEEDED_ERR' || e.name === 'NS_ERROR_DOM_QUOTA_REACHED' || e.name === 'QuotaExceededError');
}

function supportsJSON() {
  if(this.cachedJSON === undefined) {
    this.cachedJSON = window.JSON != null;
  }
  return this.cachedJSON;
}

function escapeRegExpSpecialCharacters(text) {
  return text.replace(/[[\]{}()*+?.\\^$|]/g, '\\$&');
}

function expirationKey(key) {
  return key + CACHE_SUFFIX;
}

function currentTime() {
  return Math.floor(new Date().getTime() / this.expiryMilliseconds);
}

const Implementations = {
  browserCache: {
    supported() {
      return /native/.test(window.caches.constructor + '');
    },

    async response(result, clone) {
      let tmp;

      if(clone && Util.isObject(result) && typeof result.clone == 'function') {
        try {
          tmp = result.clone();
          if(!tmp) tmp = Object.create(Object.getPrototypeOf(result), Object.getOwnPropertyDescriptors());

          if(!tmp) throw new Error('clone failed');
          result = tmp;
        } catch(error) {}
      }

      return result;
    },

    request(obj) {
      if(obj instanceof Request || (Util.isObject(obj) && typeof obj.url == 'string')) obj = obj.url;
      return obj;
    },

    async getItem(request, opts = {}) {
      if(request.url) request = request.url;
      let response = await this.cache.match(request, opts);

      if(response) {
        let { status, type, ok, statusText, headers } = response;
        let text;

        response.cached = true;
      }
      return response;
    },

    async setItem(request, response) {
      const { cache } = this;

      if(!cache) throw new Error(`Cache = ${cache}`);

      if(!(response instanceof Response))
        response = new Response(response, {
          url: request.url,
          status: 200,
          statusText: 'OK',
          headers: { 'Content-Type': type }
        });
      if(typeof cache.put == 'function') return await cache.put(request, response);
    },

    async removeItem(request) {
      const { cache } = this;
      return await cache.delete(request, {});
    },

    async eachKey(fn) {
      const { cache } = this;

      let keys = await cache.keys();
      for await(let request of keys) await fn(request);
    }
  },
  localStorage: {
    supported() {
      return /native/.test(localStorage.constructor + '');
    },

    response(r) {
      return r;
    },

    request(r) {
      return r;
    },

    getItem(key) {
      const { obj } = this;
      return localStorage.getItem(CACHE_PREFIX + obj.cacheBucket + key);
    },

    setItem(key, value) {
      const { obj } = this;
      localStorage.removeItem(CACHE_PREFIX + obj.cacheBucket + key);
      localStorage.setItem(CACHE_PREFIX + obj.cacheBucket + key, value);
    },

    removeItem(key) {
      const { obj } = this;
      localStorage.removeItem(CACHE_PREFIX + obj.cacheBucket + key);
    },

    eachKey(fn) {
      const { obj } = this;
      let prefixRegExp = new RegExp('^' + CACHE_PREFIX + escapeRegExpSpecialCharacters(obj.cacheBucket) + '(.*)');
      for(let i = localStorage.length - 1; i >= 0; --i) {
        let key = localStorage.key(i);
        key = key && key.match(prefixRegExp);
        key = key && key[1];
        if(key && key.indexOf(CACHE_SUFFIX) < 0) {
          fn.call(this, key, expirationKey.call(this, key));
        }
      }
    }
  }
};

function flushItem(key) {
  let exprKey = expirationKey.call(this, key);

  removeItem.call(this, key);
  removeItem.call(this, exprKey);
}

function flushExpiredItem(key) {
  let exprKey = expirationKey.call(this, key);
  let expr = this.getItem.call(this, exprKey);

  if(expr) {
    let expirationTime = parseInt(expr, EXPIRY_RADIX);

    if(currentTime.call(this) >= expirationTime) {
      removeItem.call(this, key);
      removeItem.call(this, exprKey);
      return true;
    }
  }
}

function warn(message, err) {
  if(!this.warnings) return;
  if(!('console' in window) || typeof window.console.warn !== 'function') return;
  window.console.warn.call(this, 'lscache - ' + message);
  if(err) window.console.warn.call(this, 'lscache - The error was: ' + err.message);
}

function calculateMaxDate(expiryMilliseconds) {
  return Math.floor(8.64e15 / expiryMilliseconds);
}
function lscache(cache) {
  const obj = new.target ? this : lscache;

  const impl = { ...Implementations.localStorage, cache };

  obj.impl = impl;
  return obj;
}

function brcache(cache) {
  let cacheName, tmp;

  const obj = new.target ? this : brcache;

  let impl = {
    ...Implementations.browserCache
  };

  if(Util.isObject(cache) && typeof cache.match == 'function') {
    impl.cache = cache;
  } else if(typeof cache == 'string') {
    cacheName = obj.cacheName = cache;
    tmp = Util.tryCatch(() => window.caches);

    Util.define(impl, {
      async getCache() {
        if(tmp && typeof tmp.open == 'function') return await tmp.open(cacheName);
      },
      get cache() {
        return this.getCache();
      }
    });
  }

  obj.impl = impl;
  obj.cache = cache;

  if(typeof obj.keys != 'function') Object.assign(obj, Util.getMethods(BaseCache.prototype, 1, 0));

  Object.assign(obj, { cacheBucket: '', warnings: false, hits: {} });

  return obj;
}

class BaseCache {
  expiryMilliseconds = 60 * 1000;

  get maxDate() {
    return calculateMaxDate(this.expiryMilliseconds);
  }

  get supportsStorage() {
    return supportsStorage.call(this);
  }
  get supportsJSON() {
    return supportsJSON.call(this);
  }

  incrementHits(key) {
    let hits = Util.mapFunction(this.hits);

    return hits.update(key, (v = 0) => v + 1);
  }

  set(key, value, time) {
    const { impl, cache, cacheBucket } = this;
    //console.info('BaseCache.set', { key, value, time, impl, cache });

    try {
      value = typeof value == 'string' ? value : value instanceof Response ? value : JSON.stringify(value);
    } catch(e) {
      return false;
    }

    try {
      impl.setItem.call(this, key, value);
    } catch(e) {
      if(isOutOfSpace(e)) {
        let storedKeys = [];
        let storedKey;
        impl.eachKey.call(this, (key, exprKey) => {
          let expiration = impl.getItem.call(this, exprKey);
          if(expiration) {
            expiration = parseInt(expiration, EXPIRY_RADIX);
          } else {
            expiration = this.maxDate;
          }
          storedKeys.push({
            key,
            size: (impl.getItem.call(this, key) || '').length,
            expiration
          });
        });
        storedKeys.sort((a, b) => b.expiration - a.expiration);

        let targetSize = (value || '').length;
        while(storedKeys.length && targetSize > 0) {
          storedKey = storedKeys.pop();
          impl.warn.call(this, "Cache is full, removing item with key '" + key + "'");
          impl.flushItem.call(this, storedKey.key);
          targetSize -= storedKey.size;
        }
        try {
          impl.setItem.call(this, key, value);
        } catch(e) {
          impl.warn.call(this, "Could not add item with key '" + key + "', perhaps it's too big?", e);
          return false;
        }
      } else {
        impl.warn.call(this, "Could not add item with key '" + key + "'", e);
        return false;
      }
    }

    if(time) {
      impl.setItem.call(this, expirationKey(key), (currentTime.call(this) + time).toString(EXPIRY_RADIX));
    } else {
      impl.removeItem.call(this, expirationKey(key));
    }
    return true;
  }

  get(key) {
    const { impl, cache } = this;

    // console.info('BaseCache.get', { key, impl, cache });

    if(flushExpiredItem.call(impl, key)) {
      return null;
    }
    return impl.getItem.call(this, key).then(response => {
      let time = Date.now();
      this.incrementHits(response.url);

      response = response.clone();
      response.cached = true;
      response.time = time;
      return response;
    });
  }

  remove(key) {
    const { impl, cache } = this;
    if(!this.supportsStorage) return;

    impl.removeItem.call(this, key);
  }
  async clear() {
    const { impl } = this;
    for(let key of await this.keys()) impl.removeItem.call(this, key);
  }

  supported() {
    return this.supportsStorage;
  }

  flush() {
    const { impl, cache } = this;
    if(!this.supportsStorage) return;

    impl.eachKey.call(this, key => impl.flushItem.call(this, key));
  }

  flushExpired() {
    const { impl, cache } = this;
    if(!this.supportsStorage) return;

    impl.eachKey.call(this, key => flushExpiredItem.call(impl, key));
  }

  setBucket(bucket) {
    this.cacheBucket = bucket;
  }

  resetBucket() {
    this.cacheBucket = '';
  }

  getExpiryMilliseconds() {
    return this.expiryMilliseconds;
  }

  setExpiryMilliseconds(milliseconds) {
    this.expiryMilliseconds = milliseconds;
    this.maxDate = impl.calculateMaxDate.call(this, this.expiryMilliseconds);
  }

  enableWarnings(enabled) {
    this.warnings = enabled;
  }
  async keys() {
    const { impl, cache } = this;
    let keys = [];
    let lscache = this;

    await impl.eachKey.call(this, key => keys.push(impl.request.call(this, key)));
    return keys;
  }
}

Object.setPrototypeOf(lscache.prototype, new (class LocalStorageCache extends BaseCache {})());

brcache.prototype = new (class BrowserCache extends BaseCache {})();

brcache.impl = Implementations.browserCache;

async function CachedFetch(fn, cacheObj) {
  let request, response;
  let url, cacheName, tmp;

  if(typeof cacheObj == 'string') {
    cacheName = cacheObj;
    tmp = await Util.tryCatch(() => window.caches.open(cacheName));
    if(Util.isObject(tmp) && typeof tmp.match == 'function') cacheObj = tmp;
  }

  let browserCache = new brcache(cacheObj);
  let { impl } = cacheObj;
  if(impl) {
    impl.obj = cacheObj;
    impl.cache = null;
  }

  async function self(...args) {
    let cached = false;
    request = args[0] instanceof Request ? args.shift() : new Request(...args);
    request.time = Date.now();
    response = null;
    try {
      response = await browserCache.get(request);
      url = impl.request.call(this, request);

      cacheObj.hits[url] = (cacheObj.hits[url] || 0) + 1;
    } catch(error) {}
    if(response) {
      cached = true;
    } else {
      try {
        response = await fetch(request);
        response.time = Date.now();
      } catch(error) {}
      if(response) {
        let r = await browserCache.set(request, response.clone());
      }
    }

    if(cached) response.cached = true;
    response.request = request;

    return response;
  }
  self.cache = browserCache;
  self.impl = impl;

  return self;
}

Object.assign(globalThis, { lscache, brcache, BaseCache, CachedFetch });

let computedTracker = [];

let effects = [];

function trkl(initValue) {
  let value = initValue;
  let subscribers = [];

  let self = function(...args) {
    return args.length ? write(args[0], this) : read();
  };

  self.subscribe = subscribe;

  self['bind_to'] = (obj, prop) => {
    Object.defineProperty(obj, prop, {
      enumerable: true,
      configurable: true,
      get: self,
      set: self
    });
    return self;
  };

  function subscribe(subscriber, immediate) {
    if(subscribers.indexOf(subscriber) == -1) subscribers.push(subscriber);
    if(immediate) subscriber(value);

    return this;
  }

  self.unsubscribe = function(subscriber) {
    remove(subscribers, subscriber);
    return this;
  };

  function write(newValue, thisObj) {
    let oldValue = value;

    if(newValue === oldValue && (newValue === null || typeof newValue !== 'object')) {
      return;
    }

    value = newValue;
    effects.push.apply(effects, subscribers);

    let subCount = subscribers.length;
    for(let i = 0; i < subCount; i++) {
      let fn = effects.pop();

      fn.call(thisObj, value, oldValue);
    }
  }

  function read() {
    let runningComputation = computedTracker[computedTracker.length - 1];
    if(runningComputation) {
      subscribe(runningComputation._subscriber);
    }
    return value;
  }
  Object.setPrototypeOf(self, trkl.prototype);
  self.subscribers = subscribers;

  return self;
}

trkl.prototype = Object.create({ ...Function.prototype, constructor: trkl });
trkl.is = arg => typeof arg == 'function' && typeof arg.subscribe == 'function';

trkl.getset = function(arg) {
  let trkl = arg || new trkl(arg);
  return Object.create(
    {
      get: () => trkl(),
      set: value => trkl(value)
    },
    {}
  );
};

trkl.computed = function(fn) {
  let self = trkl();
  let computationToken = { _subscriber: runComputed };

  runComputed();
  return self;

  function runComputed() {
    detectCircularity(computationToken);
    computedTracker.push(computationToken);
    let errors, result;
    try {
      result = fn();
    } catch(e) {
      errors = e;
    }
    computedTracker.pop();
    if(errors) {
      throw errors;
    }
    self(result);
  }
};

trkl.from = function(executor) {
  let self = trkl();
  executor(self);
  return self;
};

trkl.property = function(object, name, options = { enumerable: true, configurable: true, deletable: false }) {
  const { value, ...opts } = options;
  let self = trkl(value);
  Object.defineProperty(object, name, {
    ...opts,
    get: self,
    set: self
  });
  if(options.deletable) {
    trkl.subscribe(value => (value === undefined ? self.delete() : undefined));
    self.delete = () => {
      delete object[name];
      self(null);
    };
  }
  return self;
};

trkl.bind = function(object, name, handler) {
  let self = handler;
  if(typeof name == 'object')
    Object.defineProperties(
      object,
      Object.keys(name).reduce(
        (acc, key) => ({
          ...acc,
          [key]: {
            get: name[key],
            set: name[key],
            enumerable: true,
            configurable: true
          }
        }),
        {}
      )
    );
  else
    Object.defineProperty(object, name, {
      enumerable: true,
      configurable: true,
      get: self,
      set: self
    });
  return object;
};

trkl.object = function(handlers, ret = {}) {
  for(let prop in handlers) {
    ret[prop] = handlers[prop]();

    trkl.bind(ret, prop, handlers[prop]);
  }

  return ret;
};

function detectCircularity(token) {
  if(computedTracker.indexOf(token) !== -1) {
    throw Error('Circular computation detected');
  }
}

function remove(array, item) {
  let position = array.indexOf(item);
  if(position !== -1) {
    array.splice(position, 1);
  }
}

Object.assign(globalThis, { trkl });

Object.assign(globalThis, { trkl });

const RETURN_VALUE_PATH = 0;
const RETURN_PATH = 1 << 24;
const RETURN_VALUE = 2 << 24;
const RETURN_PATH_VALUE = 3 << 24;
const RETURN_MASK = 7 << 24;
const PATH_AS_STRING = 1 << 28;
const NO_THROW = 1 << 29;
const MAXDEPTH_MASK = 0xffffff;

function ReturnValuePath(value, path, flags) {
  if(flags & PATH_AS_STRING) path = path.join('.');

  switch (flags & RETURN_MASK) {
    case RETURN_VALUE_PATH:
      return [value, path];
    case RETURN_PATH_VALUE:
      return [path, value];
    case RETURN_PATH:
      return path;
    case RETURN_VALUE:
      return value;
  }
}

function ReturnValuePathFunction(flags) {
  switch (flags & RETURN_MASK) {
    case RETURN_VALUE_PATH:
      return (value, path) => [value, path];
    case RETURN_PATH_VALUE:
      return (value, path) => [path, value];
    case RETURN_PATH:
      return (value, path) => path;
    case RETURN_VALUE:
      return (value, path) => value;
  }
}

const isPlainObject = obj => {
  if((obj != null ? obj.constructor : void 0) == null) return false;
  return obj.constructor.name === 'Object';
};

const clone = obj => {
  let out, v, key;
  out = Array.isArray(obj) ? [] : {};
  for(key in obj) {
    v = obj[key];
    out[key] = typeof v === 'object' && v !== null ? clone(v) : v;
  }
  return out;
};

const equals = (a, b) => {
  let i, k, size_a, j, ref;
  if(a === b) {
    return true;
  } else if(Util.isArray(a)) {
    if(!(Util.isArray(b) && a.length === b.length)) {
      return false;
    }
    for(i = j = 0, ref = a.length; 0 <= ref ? j < ref : j > ref; i = 0 <= ref ? ++j : --j) {
      if(!equals(a[i], b[i])) {
        return false;
      }
    }
    return true;
  } else if(isPlainObject(a)) {
    size_a = Util.size(a);
    if(!(isPlainObject(b) && size_a === Util.size(b))) {
      return false;
    }
    for(k in a) {
      if(!equals(a[k], b[k])) {
        return false;
      }
    }
    return true;
  }
  return false;
};

const extend = (...args) => {
  let destination, k, source, sources, j, len;
  (destination = args[0]), (sources = 2 <= args.length ? Array.prototype.slice.call(args, 1) : []);
  for(j = 0, len = sources.length; j < len; j++) {
    source = sources[j];
    for(k in source) {
      if(isPlainObject(destination[k]) && isPlainObject(source[k])) {
        extend(destination[k], source[k]);
      } else {
        destination[k] = clone(source[k]);
      }
    }
  }
  return destination;
};

const select = (root, filter, flags = 0) => {
  let fn = ReturnValuePathFunction(flags);

  function SelectFunction(root, filter, path = []) {
    let k,
      selected = [];
    try {
      if(filter(root, path)) selected.push(fn(root, path));
    } catch(e) {}
    if(root !== null && { object: true }[typeof root]) for(k in root) selected = selected.concat(SelectFunction(root[k], filter, path.concat([isNaN(+k) ? k : +k])));
    return selected;
  }
  return SelectFunction(root, filter);
};

const find = (node, filter, flags = 0, root) => {
  let k,
    ret,
    result = null;
  let path = [];
  if(!root) {
    root = node;
    result = ReturnValuePath(null, null, flags);
  }
  ret = filter(node, path, root);

  if(ret === -1) return -1;
  else if(ret) result = ReturnValuePath(node, path, flags);
  else if(typeof node == 'object' && node != null) {
    for(k in node) {
      result = find(node[k], filter, [...path, k], root);
      if(result) break;
    }
  }
  return result;
};

const forEach = function(...args) {
  const [value, fn, path = []] = args;
  let root = args[3] ?? value;

  fn(value, path, root);

  if(Util.isObject(value)) for(let k in value) forEach(value[k], fn, path.concat([isNaN(+k) ? k : +k]), root);
};

const iterate = function* (...args) {
  let [value, filter = v => true, flags = RETURN_VALUE_PATH, path = []] = args;

  let r,
    root = args[4] ?? value;

  if((r = filter(value, path, root))) yield [value, path, root];
  if(r !== -1)
    if(Util.isObject(value)) {
      for(let k in value) yield* iterate(value[k], filter, flags, path.concat([isNaN(+k) ? k : +k]), root);
    }
};

const flatten = (iter, dst = {}, filter = (v, p) => typeof v != 'object' && v != null, map = (p, v) => [p.join('.'), v]) => {
  let insert;
  if(!iter.next) iter = iterate(iter, filter);

  if(typeof dst.set == 'function') insert = (name, value) => dst.set(name, value);
  else if(typeof dst.push == 'function') insert = (name, value) => dst.push([name, value]);
  else insert = (name, value) => (dst[name] = value);

  for(let [value, path] of iter) insert(...map(path, value));

  return dst;
};

const get = (root, path) => {
  let j, len;
  path = typeof path == 'string' ? path.split(/[\.\/]/) : [...path];
  for(j = 0, len = path.length; j < len; j++) {
    let k = path[j];
    root = root[k];
  }
  return root;
};

const set = (root, path, value) => {
  path = typeof path == 'string' ? path.split(/[\.\/]/) : [...path];

  if(path.length == 0) return Object.assign(root, value);

  for(let j = 0, len = path.length; j + 1 < len; j++) {
    let pathElement = isNaN(+path[j]) ? path[j] : +path[j];
    if(!(pathElement in root)) root[pathElement] = /^[0-9]+$/.test(path[j + 1]) ? [] : {};
    root = root[pathElement];
  }
  let lastPath = path.pop();
  root[lastPath] = value;
  return root;
  return (root[lastPath] = value);
};

const delegate = (root, path) => {
  if(path) {
    const last = path.pop();
    const obj = get(root, path);
    return function(value) {
      return value !== undefined ? (obj[last] = value) : obj[last];
    };
  }
  return function(path, value) {
    return value !== undefined ? obj.set(root, path, value) : obj.get(root, path);
  };
};

const transform = (obj, filter, t) => {
  let k,
    transformed,
    v,
    j,
    len,
    path = arguments[3] == [];
  if(filter(obj, path)) {
    return t(obj, path);
  } else if(Util.isArray(obj)) {
    transformed = [];
    for(j = 0, len = obj.length; j < len; j++) {
      v = obj[j];
      transformed.push(transform(v, filter, t, [...path, j]));
    }
    return transformed;
  } else if(isPlainObject(obj)) {
    transformed = {};
    q;
    for(k in obj) {
      v = obj[k];
      transformed[k] = transform(v, filter, [...path, k]);
    }
    return transformed;
  }
  return obj;
};

const unset = (object, path) => {
  if(object && typeof object === 'object') {
    let parts = typeof path == 'string' ? path.split('.') : path;

    if(parts.length > 1) {
      unset(object[parts.shift()], parts);
    } else {
      if(Util.isArray(object) && Util.isNumeric(path)) object.splice(+path, 1);
      else delete object[path];
    }
  }
  return object;
};

const unflatten = (map, obj = {}) => {
  for(let [path, value] of map) {
    set(obj, path, value);
  }
  return obj;
};

globalThis.deep = {
  isPlainObject,
  clone,
  equals,
  extend,
  select,
  forEach,
  find,
  get,
  set,
  transform,
  iterate,
  flatten,
  unflatten,
  unset,
  RETURN_PATH,
  RETURN_VALUE,
  RETURN_PATH_VALUE,
  RETURN_VALUE_PATH
};

Object.assign(globalThis, {
  RETURN_VALUE_PATH,
  RETURN_PATH,
  RETURN_VALUE,
  RETURN_PATH_VALUE,
  RETURN_MASK,
  PATH_AS_STRING,
  NO_THROW,
  MAXDEPTH_MASK,
  isPlainObject,
  clone,
  equals,
  extend,
  select,
  find,
  forEach,
  iterate,
  flatten,
  get,
  set,
  delegate,
  transform,
  unset,
  unflatten,
  Util
});
