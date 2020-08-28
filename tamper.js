// ==UserScript==
// @name        Util & DOM
// @namespace   rsenn
// @match       *://*/*
// @grant       none
// @version     1.0.1.8
// @author      Roman Senn
// @description 7/26/2020, 9:56:19 PM
// @updateURL    http://127.0.0.1:3000/tamper.js

// ==/UserScript==

/*

[...Element.iterator(document, e=> e.nodeType == 1)].map(e => Element.rect(e))


[...Element.iterator(document, e=> e.nodeType == 1)]
.map(e => { let r=Element.rect(e); return [e, r && r.area]; })
.filter(([e,a]) => e && e.textContent && a > 1000)
.map(([e,a]) => [e,e &&Element.xpath(e), Element.walkUp(e, e=>true).length,a])

*/
((g = globalThis || window) => {
  g.addEventListener('load', () => {
    g.Size = Size;

    g.Rect = Rect;
    g.Util = Util;
    g.TRBL = TRBL;
  });

  /**
   * Class for utility.
   *
   * @class      Util (name)
   */
  function Util(g) {
    Util.assignGlobal(g);
    //if(g) Util.globalObject = g;
  }

  Util.formatAnnotatedObject = function (subject, o) {
    const { indent = '  ', spacing = ' ', separator = ',', newline = '\n', maxlen = 30, depth = 1 } = o;
    const i = indent.repeat(Math.abs(1 - depth));
    let nl = newline != '' ? newline + i : spacing;
    const opts = {
      newline: depth >= 0 ? newline : '',
      depth: depth - 1
    };
    if(subject && subject.toSource !== undefined) return subject.toSource();
    if(subject instanceof Date) return `new Date('${new Date().toISOString()}')`;
    if(typeof subject == 'string') return `'${subject}'`;
    if(typeof subject == 'number') return subject;
    if(subject != null && subject.y2 !== undefined) return `rect[${spacing}${subject.x}${separator}${subject.y} | ${subject.x2}${separator}${subject.y2} (${subject.w}x${subject.h}) ]`;
    if(Util.isObject(subject) && 'map' in subject && typeof subject.map == 'function') return `[${nl}${subject.map((i) => Util.formatAnnotatedObject(i, opts)).join(separator + nl)}]`;
    if(typeof subject === 'string' || subject instanceof String) return `'${subject}'`;
    let longest = '';
    let r = [];
    for(let k in subject) {
      if(k.length > longest.length) longest = k;
      let s = '';
      if(typeof subject[k] === 'symbol') {
        s = 'Symbol';
      } else if(typeof subject[k] === 'string' || subject[k] instanceof String) {
        s = `'${subject[k]}'`;
      } else if(typeof subject[k] === 'function') {
        s = Util.fnName(s) || 'function';
        s += '()';
      } else if(typeof subject[k] === 'number' || typeof subject[k] === 'boolean') {
        s = `${subject[k]}`;
      } else if(subject[k] === null) {
        s = 'null';
      } else if(subject[k] && subject[k].length !== undefined) {
        try {
          s = depth <= 0 ? `Array(${subject[k].length})` : `[ ${subject[k].map((item) => Util.formatAnnotatedObject(item, opts)).join(', ')} ]`;
        } catch(err) {
          s = `[${subject[k]}]`;
        }
      } else if(subject[k] && subject[k].toSource !== undefined) {
        s = subject[k].toSource();
      } else if(opts.depth >= 0) {
        s = s.length > maxlen ? `[Object ${Util.objName(subject[k])}]` : Util.formatAnnotatedObject(subject[k], opts);
      }
      r.push([k, s]);
    }
    let padding = (x) => (opts.newline != '' ? Util.pad(x, longest.length, spacing) : spacing);
    let j = separator + spacing;
    if(r.length > 6) {
      nl = opts.newline + i;
      j = separator + (opts.newline || spacing) + i;
    }
    let ret = '{' + opts.newline + r.map((arr) => padding(arr[0]) + arr[0] + ':' + spacing + arr[1]).join(j) + opts.newline + i + '}';
    return ret;
  };
  Util.curry = (fn, arity) => {
    if(arity == null) arity = fn.length;
    let ret = function curried(...args) {
      let thisObj = this;
      if(args.length >= arity) return fn.apply(this, args);

      let n = arity - args.length;
      let a = Array.from({ length: n }, (v, i) => String.fromCharCode(65 + i));
      let Curried = function (...a) {
        return curried.apply(thisObj, a);
      }; //;
      return [
        function () {
          return Curried(...args);
        },
        function (a) {
          return Curried(...args, a);
        },
        function (a, b) {
          return Curried(...args, a, b);
        },
        function (a, b, c) {
          return r(...args, a, b, c);
        },
        function (a, b, c, d) {
          return Curried(...args, a, b, c, d);
        }
      ][n];
      return new Function(...a, `const { curried,thisObj,args} = this; return curried.apply(thisObj, args.concat([${a.join(',')}]))`).bind({ args, thisObj, curried });
    };
    Object.defineProperty(ret, 'length', {
      value: arity,
      configurable: true,
      writable: true,
      enumerable: false
    });
    return ret;
  };
  Util.arityN = (fn, n) => {
    const arityFn = [
      function (fn) {
        return function () {
          return fn();
        };
      },
      function (fn) {
        return function (a) {
          return fn(a);
        };
      },
      function (fn) {
        return function (a, b) {
          return fn(a, b);
        };
      },
      function (fn) {
        return function (a, b, c) {
          return fn(a, b, c);
        };
      },
      function (fn) {
        return function (a, b, c, d) {
          return fn(a, b, c, d);
        };
      },
      function (fn) {
        return function (a, b, c, d, e) {
          return fn(a, b, c, d, e);
          H;
        };
      }
    ];
    if(n && n <= 5) return arityFn[n](fn);
    return fn;
  };

  /*Util.compose2 = () => {
  let i;
  return (f, g, j) => {
    if(j == 0) i = 0;
    console.log('f,g', f + '', g + '');
    let gfn = args => {
      let r = g(...args.slice(i, i + g.length));
      console.log('g(', ...args.slice(i, i + g.length).reduce((a, p) => (a.length ? [...a, ',', p] : [p]), []), ')');
      i += g.length;
      return r;
    };
    let ffn = args => {
      let r = f(...args);
      console.log('f(', ...args.reduce((a, p) => (a.length ? [...a, ',', p] : [p]), []), ')');
      return r;
    };
    return function(...args) {
      console.log('args', { args });
      let r;
      if(f) {
        r = ffn(args);
        args.unshift(r);
      }
      r = gfn(args);
      return r;
    };
  };
};
Util.compose = (...functions) => {
  const funcs = functions.filter(fn => typeof fn === 'function');
  let lastIdx = funcs.length - 1;
  let arity = 0;
  if(funcs.length <= 0) throw new Error('No funcs passed');
  //if (lastIdx >= 0 && funcs[lastIdx]) arity = funcs[lastIdx].length;
  arity = funcs.reduce((n, f) => n + f.length, 0);
  console.log('arity:', arity);
  return Util.arityN(funcs.reduce(Util.compose2()), arity);
};*/
  Util.memoize = (fn) => {
    let cache = {};
    return (...args) => {
      let n = args[0]; // just taking one argument here
      if(n in cache) {
        //console.log('Fetching from cache');
        return cache[n];
      }
      //console.log('Calculating result');
      let result = fn(n);
      cache[n] = result;
      return result;
    };
  };

  Util.getGlobalObject = Util.memoize(() =>
    Util.tryCatch(() => global,
      (g) => g,
      (err) =>
        Util.tryCatch(() => globalThis,
          (g) => g,
          (err) =>
            Util.tryCatch(() => window,
              (g) => g,
              (err) => console.log('Util.getGlobalObject:', err)
            )
        )
    )
  );

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
    else location = Util.getStackFrame().getLocation();
    let locationStr = location.toString(true);
    let c = locationStr[Symbol.for('nodejs.util.inspect.custom')]();
    c.append([' ']);
    let filters = Util.log.filters;
    let results = filters.map((f) => f.test(locationStr));
    if(filters.every((f) => !f.test(locationStr))) return;
    args = args.reduce((a, p) => {
      if(Util.isObject(p) && p[Util.log.methodName]) p = p[Util.log.methodName]();
      a.append([p]);
      return a;
    }, c);
    args.toConsole();
  };

  Object.defineProperty(Util.log, 'methodName', {
    get: () => (Util.isBrowser() ? 'toConsole' : 'toAnsi256')
  });

  Util.log.filters = [/.*/];
  Util.log.setFilters = function (args) {
    this.filters = [...args].map((arg) => (arg instanceof RegExp ? arg : new RegExp(arg)));
  };
  Util.log.getFilters = function () {
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
  Util.logBase = function (n, base) {
    return Math.log(n) / Math.log(base);
  };
  Util.generalLog = function (n, x) {
    return Math.log(x) / Math.log(n);
  };
  Util.toSource = function (arg, opts = {}) {
    const { quote = "'", colors = false, multiline = false } = opts;
    const c = Util.coloring(colors);
    if(Util.isArray(arg)) {
      let o = '';
      for(let item of arg) {
        if(o.length > 0) o += ', ';
        o += Util.toSource(item, opts);
      }
      return `[${o}]`;
    }
    if(typeof arg == 'string') return c.text(`${quote}${arg}${quote}`, 1, 36);
    if(arg && arg.x !== undefined && arg.y !== undefined) return `[${c.text(arg.x, 1, 32)},${c.text(arg.y, 1, 32)}]`;
    //if(arg && arg.toSource) return arg.toSource();
    if(typeof arg == 'object') {
      let o = '';
      for(const prop of Object.getOwnPropertyNames(arg)) {
        //if(!Object.hasOwnProperty(arg,prop)) continue;
        let s = Util.toSource(arg[prop], opts);
        let m = new RegExp(`^\\*?${prop}\\s*\\(`).test(s);
        if(o != '') o += m && multiline ? '\n  ' : ', ';
        if(m) o += s;
        else o += prop + ': ' + s;
      }
      return multiline ? `{\n  ${o}\n}` : `{ ${o} }`;
    }
    let cls = arg && arg.constructor && Util.fnName(arg.constructor);
    return arg + '';
  };
  Util.debug = function (message) {
    const args = [...arguments];
    let cache = [];
    const removeCircular = function (key, value) {
      if(typeof value === 'object' && value !== null) {
        if(cache.indexOf(value) !== -1) return;
        cache.push(value);
      }
      return value;
    };
    const str = args
      .map((arg) => (typeof arg === 'object' ? JSON.toString(arg, removeCircular) : arg))
      .join(' ')
      .replace(/\n/g, '');
    //console.log("STR: "+str);
    //console.log.call(console, str);
    //Util.log.apply(Util, args)
  };
  Util.type = function ({ type }) {
    return (type && String(type).split(/[ ()]/)[1]) || '';
  };
  Util.functionName = function (fn) {
    const matches = /function\s*([^(]*)\(.*/g.exec(String(fn));
    if(matches && matches[1]) return matches[1];
    return null;
  };
  Util.className = function (obj) {
    let proto;
    //console.log("class:", obj);
    try {
      proto = Object.getPrototypeOf(obj);
    } catch(err) {
      try {
        proto = obj.prototype;
      } catch(err) {}
    }
    if(Util.isObject(proto) && 'constructor' in proto) return Util.fnName(proto.constructor);
  };
  Util.unwrapComponent = function (c) {
    for(;;) {
      if(c.wrappedComponent) c = c.wrappedComponent;
      else if(c.WrappedComponent) c = c.WrappedComponent;
      else break;
    }
    return c;
  };
  Util.componentName = function (c) {
    for(;;) {
      if(c.displayName || c.name) {
        return (c.displayName || c.name).replace(/.*\(([A-Za-z0-9_]+).*/, '$1');
      } else if(c.wrappedComponent) c = c.wrappedComponent;
      else if(c.WrappedComponent) c = c.WrappedComponent;
      else break;
    }
    return Util.fnName(c);
  };
  Util.count = function (s, ch) {
    return (String(s).match(new RegExp(ch, 'g')) || Util.array()).length;
  };
  Util.parseNum = function (str) {
    let num = parseFloat(str);
    if(isNaN(num)) num = 0;
    return num;
  };
  Util.minmax = function (num, min, max) {
    return Math.min(Math.max(num, min), max);
  };
  Util.getExponential = function (num) {
    let str = typeof num == 'string' ? num : num.toExponential();
    const matches = /e\+?(.*)$/.exec(str);
    //console.log("matches: ", matches);
    return parseInt(matches[1]);
  };
  Util.getNumberParts = function (num) {
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
  Util.pow2 = function (n) {
    return Math.pow(2, n);
  };
  Util.pow10 = function (n) {
    return n >= 0 ? Math.pow(10, n) : 1 / Math.pow(10, -n);
  };
  Util.bitValue = function (n) {
    return Util.pow2(n - 1);
  };
  Util.toBinary = function (num) {
    return parseInt(num).toString(2);
  };
  Util.toBits = function (num) {
    let a = Util.toBinary(num).split('').reverse();
    return Array.from(Object.assign({}, a, { length: 50 }), (bit) => (bit ? 1 : 0));
  };
  Util.getBit = function (v, n) {
    let s = v.toString(2);
    return n < s.length ? parseInt(s[s.length - n - 1]) : 0;
  };
  Util.isSet = function (v, n) {
    return Util.getBit(v, n) == 1;
  };
  Util.bitCount = function (n) {
    return Util.count(Util.toBinary(n), '1');
  };
  Util.toggleBit = function (num, bit) {
    const n = Number(num);
    return Util.isSet(n, bit) ? n - Util.pow2(bit) : n + Util.pow2(bit);
  };
  Util.setBit = function (num, bit) {
    const n = Number(num);
    return Util.isSet(n, bit) ? n : n + Util.pow2(bit);
  };
  Util.clearBit = function (num, bit) {
    const n = Number(num);
    return Util.isSet(n, bit) ? n - Util.pow2(bit) : n;
  };
  Util.range = function (start, end) {
    if(start > end) {
      let ret = [];
      while(start >= end) ret.push(start--);
      return ret;
    }
    const r = Array.from({ length: end - start + 1 }, (v, k) => k + start);
    //console.log("Util.range ", r);
    return r;
  };
  Util.set = function (obj, prop, value) {
    const set = obj instanceof Map ? (prop, value) => obj.set(prop, value) : (prop, value) => (obj[prop] = value);
    if(arguments.length == 1)
      return (prop, value) => {
        set(prop, value);
        return set;
      };
    if(arguments.length == 2) return (value) => set(prop, value);
    return set(prop, value);
  };
  Util.get = Util.curry((obj, prop) => (obj instanceof Map ? obj.get(prop) : obj[prop]));

  Util.inspect = (obj, opts = false) => {
    const { indent = '  ', newline = '\n', depth = 2, spacing = ' ' } = typeof opts == 'object' ? opts : { indent: '', newline: '', depth: typeof opts == 'number' ? opts : 10, spacing: ' ' };

    return Util.formatAnnotatedObject(obj, { indent, newline, depth, spacing });
  };
  Util.bitArrayToNumbers = function (arr) {
    let numbers = [];
    for(let i = 0; i < arr.length; i++) {
      const number = i + 1;
      if(arr[i]) numbers.push(number);
    }
    return numbers;
  };
  Util.bitsToNumbers = function (bits) {
    let a = Util.toBinary(bits).split('');
    let r = [];
    //return a;
    a.forEach((val, key, arr) => val == '1' && r.unshift(a.length - key));
    return r;
  };
  Util.shuffle = function (arr, rnd = Util.rng) {
    arr.sort((a, b) => 0.5 - rnd());
    return arr;
  };
  Util.sortNum = function (arr) {
    arr.sort((a, b) => a - b);
    //console.log("Util.sortNum ", { arr });
    return arr;
  };
  Util.draw = (arr, n = 1, rnd = Util.rng) => {
    let pos = Util.randInt(0, arr.length - n - 1, rnd);
    const r = arr.splice(pos, n);
    return n == 1 ? r[0] : r;
  };
  Util.is = function (what, ...pred) {
    let fnlist = pred.map((type) => this.is[type]);
    console.log('fnlist:', fnlist);
    return fnlist.every((fn) => fn(what));
  };

  Util.onoff = function (val) {
    if(Util.is.on(val)) return true;
    if(Util.is.off(val)) return false;
    return undefined;
  };
  Util.numbersToBits = function (arr) {
    return arr.reduce((bits, num) => bits + Util.bitValue(num), 0);
  };
  Util.randomNumbers = function ([start, end], draws) {
    const r = Util.draw(Util.range(start, end), draws);
    //console.log("Util.randomNumbers ", { start, end, draws, r });
    return r;
  };
  Util.randomBits = function (r = [1, 50], n = 5) {
    return Util.numbersToBits(Util.randomNumbers(r, n));
  };
  Util.padFn = function (len, char = ' ', fn = (str, pad) => pad) {
    return (s, n = len) => {
      let m = Util.stripAnsi(s).length;
      s = s ? s.toString() : '' + s;
      return fn(s, m < n ? char.repeat(n - m) : '');
    };
  };
  Util.pad = function (s, n, char = ' ') {
    return Util.padFn(n, char)(s);
  };
  Util.abbreviate = function (str, max = 40, suffix = '...') {
    str = '' + str;
    if(str.length > max) {
      return str.substring(0, max - suffix.length) + suffix;
    }
    return str;
  };
  Util.trim = function (str, charset) {
    const r1 = RegExp(`^[${charset}]*`);
    const r2 = RegExp(`[${charset}]*$`);
    return str.replace(r1, '').replace(r2, '');
  };
  Util.trimRight = function (str, charset) {
    const r2 = RegExp(`[${charset}]*$`);
    return str.replace(r2, '');
  };
  Util.indent = (text, space = '  ') => {
    text = text.trim();
    if(!/\n/.test(text)) return text;
    return text.replace(/(\n)/g, '\n' + space) + '\n';
  };
  Util.define = (obj, ...args) => {
    if(typeof args[0] == 'object') {
      //let args = [...arguments].slice(1);
      for(let arg of args) {
        let adecl = Object.getOwnPropertyDescriptors(arg);
        let odecl = {};

        for(let prop in adecl) {
          //delete obj[prop];

          if(Object.getOwnPropertyDescriptor(obj, prop)) delete odecl[prop];
          else odecl[prop] = { ...adecl[prop], enumerable: false, configurable: true, writeable: true };

          //odecl[prop].enumerable=false;
        }
        //console.log('odecl:', odecl);
        Object.defineProperties(obj, odecl);
      }

      /* console.log('Util.define', { decl, keys: Object.getOwnPropertyNames(obj) });
    console.log('Util.define', { decl, values: Object.getOwnPropertyNames(obj).map(key => obj[key]) });*/

      //for(let prop in key) Util.define(obj, prop, key[prop], Util.isBool(value) ? value : false);
      return obj;
    }
    const [key, value, enumerable = false] = args;

    /*obj[key] === undefined &&*/
    Object.defineProperty(obj, key, {
      enumerable,
      configurable: true,
      writable: true,
      value
    });
    return obj;
  };
  Util.copyWhole = (dst, ...args) => {
    let chain = [];
    for(let src of args) chain = chain.concat(Util.getPrototypeChain(src).reverse());
    console.log('chain:', ...chain);
    Util.define(dst, ...chain);
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
        if(extender.hasOwnProperty(key)) {
          let value = extender[key];
          if(deep && Util.isCloneable(value)) {
            let base = Array.isArray(value) ? [] : {};
            result[key] = extend(true, result.hasOwnProperty(key) && !Util.isUnextendable(result[key]) ? result[key] : base, value);
          } else {
            result[key] = value;
          }
        }
      }
    }
    return result;
  };

  Util.isCloneable = (obj) => Array.isArray(obj) || {}.toString.call(obj) == '[object Object]';

  Util.isUnextendable = (val) => !val || (typeof val != 'object' && typeof val != 'function');

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
    for(let [name, fn] of Util.iterateMembers(functions,

      Util.tryPredicate((key, depth) => obj[key] === undefined && typeof functions[key] == 'function' && pred(key, depth, functions) && [key, value])
    )) {
      const value = function (...args) {
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
  Util.defineGetter = (obj, key, get, enumerable = false) =>
    obj[key] === undefined &&
    Object.defineProperty(obj, key, {
      enumerable,
      configurable: true,
      get
    });
  Util.defineGetterSetter = (obj, key, get, set, enumerable = false) =>
    obj[key] === undefined &&
    Object.defineProperty(obj, key, {
      get,
      set,
      enumerable
    });
  Util.extendArray = function (arr = Array.prototype) {
    /*  Util.define(arr, 'tail', function() {
    return this[this.length - 1];
  });*/
    Util.define(arr, 'match', function (pred) {
      return Util.match(this, pred);
    });
    Util.define(arr, 'clear', function () {
      this.splice(0, this, length);
      return this;
    });
    Util.define(arr, 'unique', function () {
      return this.filter((item, i, a) => a.indexOf(item) == i);
    });
    Util.defineGetterSetter(arr,
      'tail',
      function () {
        return Util.tail(this);
      },
      function (value) {
        if(this.length == 0) this.push(value);
        else this[this.length - 1] = value;
      }
    );

    /*Util.define(arr, 'inspect', function(opts = {}) {
    return Util.inspect(this, { depth: 100, ...opts });
  });*/
  };
  Util.adapter = function (obj, getLength = (obj) => obj.length, getKey = (obj, index) => obj.key(index), getItem = (obj, key) => obj[key], setItem = (obj, index, value) => (obj[index] = value)) {
    const adapter = {
      get length() {
        return getLength(obj);
      },
      get instance() {
        return obj;
      },
      key(i) {
        return getKey(obj, i);
      },
      get(key) {
        return getItem(obj, key);
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
      }, [Symbol.iterator]() {
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
  Util.adapter.localStorage = function (s) {
    s = Util.tryCatch(() => !s && globalThis.window,
      (w) => w.localStorage,
      () => s
    );

    return Util.adapter(s,
      (l) => l.length,
      (l, i) => l.key(i),
      (l, key) => JSON.parse(l.getItem(key)),
      (l, key, v) => l.setItem(key, JSON.toString(v))
    );
  };
  let doExtendArray = Util.extendArray;
  Util.array = function (a) {
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
  Util.arrayFromEntries = (entries) =>
    Array.from(entries.map(([k, v]) => k),
      (key) => entries.find(([k, v]) => k === key)[1]
    );

  Util.toMap = function (hash = {}, fn) {
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
  Util.extendMap = function (map) {
    if(map.entries === undefined) {
      map.entries = function* iterator() {
        for(let entry of map) {
          if(entry.name !== undefined && entry.value !== undefined) yield [entry.name, entry.value];
          else if(entry[0] !== undefined && entry[1] !== undefined) yield entry;
          else yield [entry, map[entry]];
        }
      };
    }
    map.toObject = function () {
      return Object.fromEntries(this.entries());
    };
    map.match = function (...args) {
      return Util.match.apply(this, args);
    };
  };
  Util.fromEntries = Object.fromEntries
    ? Object.fromEntries
    : (entries) => {
        let ret = {};
        for(let [k, v] of entries) {
          ret[k] = v;
        }
        return ret;
      };

  Util.objectFrom = function (any) {
    if('toJS' in any) any = any.toJS();
    else if(Util.isArray(any)) return Util.fromEntries(any);
    else if('entries' in any) return Util.fromEntries(any.entries());
    return Object.assign({}, any);
  };
  Util.tail = function (arr) {
    return arr && arr.length > 0 ? arr[arr.length - 1] : null;
  };
  Util.splice = function (str, index, delcount, insert) {
    const chars = str.split('');
    Array.prototype.splice.apply(chars, arguments);
    return chars.join('');
  };
  Util.keyOf = function (obj, prop) {
    const keys = Object.keys(obj);
    for(let k in keys) {
      if(obj[k] === prop) return k;
    }
    return undefined;
  };
  Util.rotateRight = function (arr, n) {
    arr.unshift(...arr.splice(n, arr.length));
    return arr;
  };
  Util.repeater = function (n, what) {
    if(typeof what == 'function')
      return (function* () {
        for(let i = 0; i < n; i++) yield what();
      })();
    return (function* () {
      for(let i = 0; i < n; i++) yield what;
    })();
  };
  Util.repeat = function (n, what) {
    return [...Util.repeater(n, what)];
  };
  Util.arrayDim = function (dimensions, init) {
    let args = [...dimensions];
    args.reverse();
    let ret = init;
    while(args.length > 0) {
      const n = args.shift();
      ret = Util.repeat(n, ret);
    }
    return ret;
  };
  Util.flatten = function (arr) {
    let ret = [];
    for(let i = 0; i < arr.length; i++) {
      ret = [...ret, ...arr[i]];
    }
    return ret;
  };
  Util.chunkArray = function (a, size) {
    let r = [];
    for(let i = 0; i < a.length; i += size) r.push(a.slice(i, i + size));
    return r;
  };
  Util.chances = function (numbers, matches) {
    const f = Util.factorial;
    return f(numbers) / (f(matches) * f(numbers - matches));
  };
  Util.sum = function (arr) {
    return arr.reduce((acc, n) => acc + n, 0);
  };

  Util.expr = (fn) => {
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
      console.log('nargs:', nargs);
      console.log('nums.length:', nums.length);
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
  Util.mod = Util.curry((a, b) => a % b);
  Util.pow = Util.curry((a, b) => a ** b);

  /*Util.define(String.prototype,
  'splice',
  function(index, delcount, insert) {
    return Util.splice.apply(this, [this, ...arguments]);
  }
);*/
  Util.fnName = function (f, parent) {
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

  Util.objName = function (o) {
    if(o === undefined || o == null) return `${o}`;
    if(typeof o === 'function' || o instanceof Function) return Util.fnName(o);
    if(o.constructor) return Util.fnName(o.constructor);
    const s = `${o.type}`;
    return s;
  };
  Util.findKey = function (obj, value) {
    let pred = typeof value == 'function' ? value : (v) => v === value;
    for(let k in obj) if(pred(obj[k], k)) return k;
  };
  Util.find = function (arr, value, prop = 'id', acc = Util.array()) {
    let pred;
    if(typeof value == 'function') pred = value;
    else if(prop && prop.length !== undefined) {
      pred = function (obj) {
        if(obj[prop] == value) return true;
        return false;
      };
    } else pred = (obj) => obj[prop] == value;
    for(let v of arr) {
      //console.log("v: ", v, "k:", k);
      /*if(Util.isArray(v)) {
      for(let i = 0; i < v.length; i++)
        if(pred(v[i]))
          return v[i];
    } else */ {
        if(pred(v)) return v;
      }
    }
    return null;
  };
  Util.match = function (arg, pred) {
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
  Util.toHash = function (map, keyTransform = (k) => Util.camelize('' + k)) {
    let ret = {};
    Util.foreach(map, (v, k) => (ret[keyTransform(k)] = v));
    return ret;
  };
  Util.indexOf = function (obj, prop) {
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
  Util.toString = (obj, opts = {}) => {
    const { quote = '"', multiline = false, indent = '', colors = true, stringColor = [1, 36], spacing = '', padding = '', separator = ',', colon = ':', depth = 10 } = { ...Util.toString.defaultOpts, ...opts };

    if(depth < 0) {
      if(Util.isArray(obj)) return `[...${obj.length}...]`;
      if(Util.isObject(obj)) return `{ ..${Object.keys(obj).length}.. }`;
      return '' + obj;
    }
    const { c = Util.coloring(colors) } = opts;

    const sep = multiline && depth > 0 ? (space = false) => '\n' + indent + (space ? '  ' : '') : (space = false) => (space ? spacing : '');
    if(Util.isArray(obj)) {
      let i,
        s = c.text(`[`, 1, 36);
      for(i = 0; i < obj.length; i++) {
        s += i > 0 ? c.text(separator, 1, 36) : padding;
        s += sep(true);
        s += Util.toString(obj[i], { ...opts, c, depth: depth - 1 }, indent + (multiline ? '  ' : ''));
      }
      return s + (i > 0 ? sep() + padding : '') + `]`;
    } else if(typeof obj == 'function' || obj instanceof Function || Util.className(obj) == 'Function') {
      obj = '' + obj;
      if(!multiline) obj = obj.replace(/(\n| anonymous)/g, '');
      return obj;
    } else if(typeof obj == 'string') {
      return JSON.toString(obj);
    } else if(!Util.isObject(obj)) {
      return '' + obj;
    }
    let s = c.text(`{` + padding, 1, 36);
    let i = 0;
    for(let key in obj) {
      const value = obj[key];
      s += i > 0 ? c.text(separator + sep(true), 36) : '';

      s += `${c.text(key, 1, 33)}${c.text(colon, 1, 36)}` + spacing;
      /*if(Util.isArray(value)) s+= Util.toString(value);
      else*/ if(Util.isObject(value)) s += Util.toString(value, { ...opts, c, depth: depth - 1 }, multiline ? '  ' : '');
      else if(typeof value == 'string') s += c.text(`${quote}${value}${quote}`, ...stringColor);
      else if(typeof value == 'number') s += c.text(value, 1, 32);
      else s += value;
      i++;
    }
    return s + sep(false) + c.text(`${padding}}`, 1, 36);
  };
  Util.toString.defaultOpts = {
    spacing: ' ',
    padding: ' '
  };

  Util.dump = function (name, props) {
    const args = [name];
    for(let key in props) {
      f;
      args.push(`\n\t${key}: `);
      args.push(props[key]);
    }
    const w = Util.tryCatch(() => globalThis.window,
      (w) => w,
      () => null
    );

    if(w) {
      //if(window.alert !== undefined)
      //alert(args);
      if(w.console !== undefined) w.console.log(...args);
    }
  };
  Util.ucfirst = function (str) {
    if(typeof str != 'string') str = String(str);
    return str.substring(0, 1).toUpperCase() + str.substring(1);
  };
  Util.lcfirst = function (str) {
    return str.substring(0, 1).toLowerCase() + str.substring(1);
  };
  Util.typeOf = function (v) {
    if(Util.isObject(v) && Object.getPrototypeOf(v) != Object.prototype) return `${Util.className(v)}`;
    return Util.ucfirst(typeof v);
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

  Util.decamelize = function (str, separator = '-') {
    return /.[A-Z]/.test(str)
      ? str
          .replace(/([a-z\d])([A-Z])/g, `$1${separator}$2`)
          .replace(/([A-Z]+)([A-Z][a-z\d]+)/g, `$1${separator}$2`)
          .toLowerCase()
      : str;
  };
  Util.ifThenElse = function (pred = (value) => !!value, _then = () => {}, _else = () => {}) {
    return function (value) {
      let result = pred(value);
      let ret = result ? _then(value) : _else(value);
      return ret;
    };
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
        ? function (...args) {
            //console.log("bucketInsert:",map,args);
            for(let [k, v] of args) {
              let a;
              map.has(k) ? (a = map.get(k)) : map.set(k, (a = []));
              a.push(v);
            }
            return inserter;
          }
        : function (...args) {
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
          await new Promise((r) => (resolve = r));
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
        for await (const event of this.loop) {
          console.log('event:', event);
        }
      }
    };
  };
  Util.isEmail = function (v) {
    return /^[\w-]+(\.[\w-]+)*@[\w-]+(\.[\w-]+)+$/.test(v);
  };
  Util.isString = function (v) {
    return Object.prototype.toString.call(v) == '[object String]';
  };

  /**
   * Determines whether the specified v is numeric.
   *
   * @param      {<type>}   v       { parameter_description }
   * @return     {boolean}  True if the specified v is numeric, False otherwise.
   */
  Util.isNumeric = (v) => /^[-+]?(0x|0b|0o|)[0-9]*\.?[0-9]+(|[Ee][-+]?[0-9]+)$/.test(v + '');

  Util.isObject = (obj, proto = null) => typeof obj === 'object' && obj !== null && (proto === null || Object.getPrototypeOf(obj) === proto);
  Util.isFunction = (fn) => !!(fn && fn.constructor && fn.call && fn.apply);

  Util.isAsync = (fn) => typeof fn == 'function' && /async/.test(fn + '') /*|| fn() instanceof Promise*/;

  Util.isArrowFunction = (fn) => (Util.isFunction(fn) && !('prototype' in fn)) || /\ =>\ /.test(('' + fn).replace(/\n.*/g, ''));

  Util.isEmptyString = (v) => Util.isString(v) && (v == '' || v.length == 0);

  Util.isEmpty = function (v) {
    if(typeof v == 'object' && !!v && v.constructor == Object && Object.keys(v).length == 0) return true;
    if(!v || v === null) return true;
    if(typeof v == 'object' && v.length !== undefined && v.length === 0) return true;
    return false;
  };
  Util.isNonEmpty = (v) => !Util.isEmpty(v);
  Util.isIpAddress = (v) => {
    const n = (v + '').split('.').map((i) => +i);
    return n.length == 4 && n.every((i) => !isNaN(i) && i >= 0 && i <= 255);
  };
  Util.isPortNumber = (v) => {
    const n = +v;
    return !isNaN(n) && n >= 0 && n <= 65535;
  };

  Util.hasProps = function (obj, props) {
    const keys = Object.keys(obj);
    return props ? props.every((prop) => 'prop' in obj) : keys.length > 0;
  };
  Util.validatePassword = function (value) {
    return value.length > 7 && /^(?![\d]+$)(?![a-zA-Z]+$)(?![!#$%^&*]+$)[\da-zA-Z!#$ %^&*]/.test(value) && !/\s/.test(value);
  };
  Util.clone = function (obj, proto) {
    if(Util.isArray(obj)) return obj.slice();
    else if(typeof obj == 'object') return Object.create(proto || obj.constructor.prototype || Object.getPrototypeOf(obj), Object.getOwnPropertyDescriptors(obj));
  };
  //deep copy
  Util.deepClone = function (data) {
    return JSON.parse(JSON.toString(data));
  };
  //Function
  Util.findVal = function (object, propName, maxDepth = 10) {
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
  Util.deepCloneObservable = function (data) {
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
  Util.toArray = function (observableArray) {
    return observableArray.slice();
  };

  /**
   * Convert the original array to tree
   * @param data original array
   * @param id id field
   * @param pId parent id field
   * @param appId the parent id value of the level one array
   */
  Util.arryToTree = function (data, id, pId, appId) {
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
  Util.to3wei = function (a, old, id, pId) {
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
  Util.arrExchangePos = function (arr, i, j) {
    arr[i] = arr.splice(j, 1, arr[i])[0];
  };
  Util.arrRemove = function (arr, i) {
    const index = arr.indexOf(i);
    if(index > -1) arr.splice(index, 1);
  };
  Util.move = function (src, dst = []) {
    let items = src.splice(0, src.length);
    dst.splice(dst.length, 0, ...items);
    return dst;
  };
  Util.moveIf = function (src, pred, dst = []) {
    let items = src.splice(0, src.length);
    let i = 0;
    for(let item of items) (pred(item, i++) ? src : dst).push(item);

    return dst;
  };
  Util.removeEqual = function (a, b) {
    let c = {};
    for(let key of Util.keys(a)) {
      if(b[key] === a[key]) continue;
      //console.log(`removeEqual '${a[key]}' === '${b[key]}'`);
      c[key] = a[key];
    }
    //console.log(`removeEqual`,c);

    return c;
  };
  //Remove the storage when logging out
  Util.logOutClearStorage = function () {
    localStorage.removeItem('userToken');
    localStorage.removeItem('userLoginPermission');
    localStorage.removeItem('ssoToken');
    localStorage.removeItem('userId');
    localStorage.removeItem('userInfo');
    localStorage.removeItem('userGroupList');
    localStorage.removeItem('gameAuthList');
  };
  //Take the cookies
  Util.getCookie = function (cookie, name) {
    let arr = cookie.match(new RegExp(`(^| )${name}=([^;]*)(;|$)`));
    if(arr != null) return unescape(arr[2]);
    return null;
  };
  Util.parseCookie = function (c = document.cookie) {
    if(!(typeof c == 'string' && c && c.length > 0)) return {};
    let key = '';
    let value = '';
    const ws = ' \r\n\t';
    let i = 0;
    let ret = {};
    const skip = (pred = (char) => ws.indexOf(char) != -1) => {
      let start = i;
      while(i < c.length && pred(c[i])) i++;
      let r = c.substring(start, i);
      return r;
    };
    do {
      let str = skip((char) => char != '=' && char != ';');
      if(c[i] == '=' && str != 'path') {
        i++;
        key = str;
        value = skip((char) => char != ';');
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
  Util.encodeCookie = (c) =>
    Object.entries(c)
      .map(([key, value]) => `${key}=${encodeURIComponent(value)}`)
      .join('; ');
  Util.setCookies = (c) =>
    Object.entries(c).forEach(([key, value]) => {
      document.cookie = `${key}=${value}`;
      //console.log(`Setting cookie[${key}] = ${value}`);
    });
  Util.clearCookies = function (c) {
    return Util.setCookies(Object.keys(Util.parseCookie(c)).reduce((acc, name) => Object.assign(acc, { [name]: `; max-age=0; expires=${new Date().toUTCString()}` }), {}));
  };
  Util.deleteCookie = function (name) {
    const w = Util.tryCatch(() => globalThis.window,
      (w) => w,
      () => null
    );

    if(w) document.cookie = `${name}=; expires=Thu, 01 Jan 1970 00:00:01 GMT;`;
  };
  Util.accAdd = function (arg1, arg2) {
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
  Util.Subtr = function (arg1, arg2) {
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
  Util.accDiv = function (arg1, arg2) {
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
  Util.accMul = function (arg1, arg2) {
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
  Util.dateFormatter = function (date, formate) {
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
  Util.numberFormatter = function (numStr) {
    let numSplit = numStr.split('.');
    return numSplit[0].replace(/\B(?=(\d{3})+(?!\d))/g, ',').concat(`.${numSplit[1]}`);
  };
  Util.searchObject = function (object, matchCallback, currentPath, result, searched) {
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
    Util.tryCatch(() => process.argv[1],
      () => 'file://' + Util.scriptDir(),
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
  );
  Util.parseQuery = function (url = Util.getURL()) {
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
  Util.encodeQuery = function (data) {
    const ret = [];
    for(let d in data) ret.push(`${encodeURIComponent(d)}=${encodeURIComponent(data[d])}`);
    return ret.join('&');
  };
  Util.parseURL = function (href = this.getURL()) {
    const matches = /^([^:]*):\/\/([^/:]*)(:[0-9]*)?(\/?.*)/.exec(href);
    if(!matches) return null;
    const argstr = matches[4].indexOf('?') != -1 ? matches[4].replace(/^[^?]*\?/, '') : ''; /* + "&test=1"*/
    const pmatches =
      typeof argstr === 'string'
        ? argstr
            .split(/&/g)
            .map((part) => {
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
    return {
      protocol: matches[1],
      host: matches[2],
      port: typeof matches[3] === 'string' ? parseInt(matches[3].substring(1)) : 443,
      location: matches[4].replace(/\?.*/, ''),
      query: params,
      href(override) {
        if(typeof override === 'object') Object.assign(this, override);
        const qstr = Util.encodeQuery(this.query);
        return (this.protocol ? `${this.protocol}://` : '') + (this.host ? this.host : '') + (this.port ? `:${this.port}` : '') + `${this.location}` + (qstr != '' ? `?${qstr}` : '');
      }
    };
  };
  Util.makeURL = function (...args) {
    let href = typeof args[0] == 'string' ? args.shift() : Util.getURL();
    let url = Util.parseURL(href);
    let obj = typeof args[0] == 'object' ? args.shift() : {};
    Object.assign(url, obj);
    return url.href();

    /*
  let href = typeof args[0] === "string" ? args.shift() : this.getURL();
  let urlObj = null;
  urlObj = this.parseURL(href);
  return urlObj ? urlObj.href(args[0]) : null;*/
  };
  Util.numberFromURL = function (url, fn) {
    const obj = typeof url === 'object' ? url : this.parseURL(url);
    const nr_match = RegExp('.*[^0-9]([0-9]+)$').exec(url.location);
    const nr_arg = nr_match ? nr_match[1] : undefined;
    const nr = nr_arg && parseInt(nr_arg);
    if(!isNaN(nr) && typeof fn === 'function') fn(nr);
    return nr;
  };
  Util.tryPromise = (fn) => new Promise((resolve, reject) => Util.tryCatch(fn, resolve, reject));

  Util.tryFunction = (fn, resolve = (a) => a, reject = () => null) => {
    if(typeof resolve != 'function') {
      let rval = resolve;
      resolve = () => rval;
    }
    if(typeof reject != 'function') {
      let cval = reject;
      reject = () => cval;
    }
    return Util.isAsync(fn)
      ? async function (...args) {
          let ret;
          try {
            ret = await fn(...args);
          } catch(err) {
            return reject(err, ...args);
          }
          return resolve(ret, ...args);
        }
      : function (...args) {
          let ret;
          try {
            ret = fn(...args);
          } catch(err) {
            return reject(err, ...args);
          }
          return resolve(ret, ...args);
        };
  };
  Util.tryCatch = (fn, resolve = (a) => a, reject = () => null, ...args) => Util.tryFunction(fn, resolve, reject)(...args);
  Util.putError = (err) => {
    let s = Util.stack(err.stack);

    let e = Util.exception(err);

    console.error('ERROR:', e.message, '\nstack:\n', s.toString());
  };
  Util.putStack = (stack) => {
    stack = stack || Util.stack(new Error().stack);
    console.error('STACK TRACE:', stack.toString());
  };

  Util.trap = (() => {
    Error.stackTraceLimit = 100;
    return (fn) => /* prettier-ignore */ Util.tryFunction(fn, ret => ret, Util.putError);
  })();

  Util.tryPredicate = (fn, defaultRet) =>
    Util.tryFunction(fn,
      (ret) => ret,
      () => defaultRet
    );

  Util.isBrowser = function () {
    let ret = false;

    Util.tryCatch(() => window,
      (w) => (Util.isObject(w) ? (ret = true) : undefined),
      () => {}
    );
    Util.tryCatch(() => document,
      (w) => (Util.isObject(w) ? (ret = true) : undefined),
      () => {}
    );
    return ret;
    //return !!(globalThis.window && globalThis.window.document);
  };
  Util.isServer = function () {
    return !Util.isBrowser();
  };
  Util.isMobile = function () {
    return true;
  };
  Util.uniquePred = (cmp = null) => (cmp === null ? (el, i, arr) => arr.indexOf(el) === i : (el, i, arr) => arr.findIndex((item) => cmp(el, item)) === i);
  Util.unique = (arr, cmp) => arr.filter(Util.uniquePred(cmp));

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
  Util.distinct = function (arr) {
    return Array.prototype.filter.call(arr, (value, index, me) => me.indexOf(value) === index);
  };
  Util.rangeMinMax = function (arr, field) {
    const numbers = [...arr].map((obj) => obj[field]);
    return [Math.min(...numbers), Math.max(...numbers)];
  };

  Util.remap = (...args) => {
    const getR = () => (Util.isArray(args[0]) ? args.shift() : args.splice(0, 2));
    const from = getR(),
      to = getR();

    const f = [to[1] - to[0], from[1] - from[0]];
    const factor = f[0] / f[1];

    const r = (val) => (val - from[0]) * factor + to[0];

    return r;
  };
  Util.mergeLists = function (arr1, arr2, key = 'id') {
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
  Util.throttle = function (fn, wait) {
    let time = Date.now();
    return function () {
      if(time + wait - Date.now() < 0) {
        fn();
        time = Date.now();
      }
    };
  };
  Util.foreach = function (o, fn) {
    for(let [k, v] of Util.entries(o)) {
      if(fn(v, k, o) === false) break;
    }
  };
  Util.all = function (obj, pred) {
    for(let k in obj) if(!pred(obj[k])) return false;
    return true;
  };
  Util.isGenerator = function (fn) {
    return (typeof fn == 'function' && /^[^(]*\*/.test(fn.toString())) || (['function', 'object'].indexOf(typeof fn) != -1 && fn.next !== undefined);
  };
  Util.isIterable = (obj) => {
    try {
      for(let item of obj) return true;
    } catch(err) {}
    return false;
  };
  Util.isNativeFunction = (x) => typeof x == 'function' && /\[(native\ code|[^\n]*)\]/.test(x + '');
  Util.isConstructor = (x) => {
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
      members = Util.getMemberNames(proto).filter((m) => m !== 'constructor');
      //console.log('members:', !!ret, members, Util.fnName(x));
      return !!ret && members.length > 0;
    }
  };

  Util.filter = function (a, pred) {
    if(Util.isGenerator(a))
      return (function* () {
        for(let item of a) if(pred(item)) yield item;
      })();
    let isa = Util.isArray(a);
    if(isa)
      return (function* () {
        for(let [k, v] of a.entries()) if(pred(v, k, a)) yield v;
      })();
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
  Util.mapFunctional = (fn) =>
    function* (arg) {
      for(let item of arg) yield fn(item);
    };
  Util.map = (obj, fn) => {
    let ret = (a) => a;
    if(typeof obj == 'function') return Util.mapFunctional(...arguments);

    if(typeof obj.map == 'function') return obj.map(fn);

    if(typeof obj.entries == 'function') {
      const ctor = obj.constructor;
      obj = obj.entries();
      ret = (a) => new ctor([...a]);
      //    ret = a => new ctor(a);
    }

    /*console.log("obj",(obj));
console.log("isGenerator",Util.isGenerator(obj));*/

    if(Util.isGenerator(obj))
      return ret((function* () {
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

  Util.entriesToObj = function (arr) {
    return [...arr].reduce((acc, item) => {
      const k = item[0];
      const v = item[1];
      acc[k] = v;
      return acc;
    }, {});
  };
  Util.isDate = function (d) {
    return d instanceof Date || (typeof d == 'string' && /[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]T[0-9][0-9]:[0-9][0-9]:[0-9][0-9]/.test(d));
  };
  Util.parseDate = function (d) {
    if(Util.isDate(d)) {
      d = new Date(d);
    }
    return d;
    //return /^[0-9]+$/.test(d) ? Util.fromUnixTime(d) : new Date(d);
  };
  Util.isoDate = function (date) {
    try {
      const minOffset = date.getTimezoneOffset();
      const milliseconds = date.valueOf() - minOffset * 60 * 1000;
      date = new Date(milliseconds);
      return date.toISOString().replace(/T.*/, '');
    } catch(err) {}
    return null;
  };
  Util.toUnixTime = function (dateObj, utc = false) {
    if(!(dateObj instanceof Date)) dateObj = new Date(dateObj);
    let epoch = Math.floor(dateObj.getTime() / 1000);
    if(utc) epoch += dateObj.getTimezoneOffset() * 60;
    return epoch;
  };
  Util.unixTime = function (utc = false) {
    return Util.toUnixTime(new Date(), utc);
  };
  Util.fromUnixTime = function (epoch, utc = false) {
    let t = parseInt(epoch);
    let d = new Date(0);
    utc ? d.setUTCSeconds(t) : d.setSeconds(t);
    return d;
  };
  Util.formatTime = function (date = new Date(), format = 'HH:MM:SS') {
    let n;
    let out = '';
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
  Util.leapYear = function (year) {
    if(year % 400 == 0) return true;
    if(year % 100 == 0) return false;
    if(year % 4 == 0) return true;
    return false;
  };
  Util.timeSpan = function (s) {
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
    ret = `${`0${hours}`.substring(0, 2)}:${`0${minutes}`.substring(0, 2)}:${`0${seconds}`.substring(0, 2)}`;
    if(days) ret = `${days} days ${ret}`;
    if(weeks) ret = `${weeks} weeks ${ret}`;
    return ret;
  };
  Util.rng = Math.random;
  Util.randFloat = function (min, max, rnd = Util.rng) {
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

  Util.hex = function (num, numDigits = 0) {
    let n = typeof num == 'number' ? num : parseInt(num);
    return ('0'.repeat(numDigits) + n.toString(16)).slice(-numDigits);
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
  Util.roundTo = function (value, prec, digits, type = 'round') {
    const fn = Math[type];
    if(prec == 1) return fn(value);

    /*  const decimals = Math.log10(prec);
  const digits = Math.ceil(-decimals);
  console.log('digits:', digits);*/
    let ret = fn(value / prec) * prec;

    if(typeof digits == 'number') ret = +ret.toFixed(digits);
    return ret;
  };
  Util.base64 = (() => {
    const w = Util.tryCatch(() => globalThis.window,
      (w) => w,
      () => null
    );

    return {
      encode: (utf8) => {
        if(w) return w.btoa(w.unescape(w.encodeURIComponent(utf8)));
        return Buffer.from(utf8).toString('base64');
      },
      decode: (base64) => {
        if(w) return w.decodeURIComponent(w.escape(w.atob(base64)));
      }
    };
  })();

  Util.formatRecord = function (obj) {
    let ret = {};
    for(let key in obj) {
      let val = obj[key];
      if(val instanceof Array) val = val.map((item) => Util.formatRecord(item));
      else if(/^-?[0-9]+$/.test(val)) val = parseInt(val);
      else if(/^-?[.0-9]+$/.test(val)) val = parseFloat(val);
      else if(val == 'true' || val == 'false') val = Boolean(val);
      ret[key] = val;
    }
    return ret;
  };
  Util.isArray = function (obj) {
    return (obj && !Util.isGetter(obj, 'length') && Util.isObject(obj) && 'length' in obj && !(obj instanceof String) && !(obj instanceof Function) && typeof obj == 'function') || obj instanceof Array;
  };
  Util.equals = function (a, b) {
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
  Util.isBool = (value) => value === true || value === false;
  Util.size = function (obj) {
    if(Util.isObject(obj)) {
      if('length' in obj) return obj.length;
      return Object.keys(obj).length;
    }
    return undefined;
  };
  Util.isMap = function (obj) {
    return (obj && obj.get !== undefined && obj.keys !== undefined) || obj instanceof Map;
  };
  Util.effectiveDeviceWidth = function () {
    let deviceWidth = window.orientation == 0 ? window.screen.width : window.screen.height;
    //iOS returns available pixels, Android returns pixels / pixel ratio
    //http://www.quirksmode.org/blog/archives/2012/07/more_about_devi.html
    if(navigator.userAgent.indexOf('Android') >= 0 && window.devicePixelRatio) {
      deviceWidth = deviceWidth / window.devicePixelRatio;
    }
    return deviceWidth;
  };
  Util.getFormFields = function (initialState) {
    return Util.mergeObjects([initialState, [...document.forms].reduce((acc, { elements }) => [...elements].reduce((acc2, { name, value }) => (name == '' || value == undefined || value == 'undefined' ? acc2 : Object.assign(acc2, { [name]: value })), acc), {})]);
  };
  Util.mergeObjects = function (objArr, predicate = (dst, src, key) => (src[key] == '' ? undefined : src[key])) {
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
  Util.getUserAgent = function (headers = req.headers) {
    const agent = useragent.parse(headers['user-agent']);
    return agent;
  };
  Util.factor = function (start, end) {
    let f = 1;
    for(let i = start; i <= end; i++) {
      f = f * i;
    }
    return f;
  };
  Util.factorial = function (n) {
    return Util.factor(1, n);
  };
  Util.lottoChances = function (numbers, draws) {
    const f = Util.factorial;
    return f(numbers) / (f(numbers - draws) * f(draws));
  };
  Util.increment = function (obj, key) {
    if(obj[key] >= 1) obj[key] == 0;
    obj[key]++;
    return obj[key];
  };
  Util.counter = function () {
    this.i = 0;
    this.incr = function () {
      this.i++;
      return this.i;
    };
  };
  Util.filterKeys = function (obj, pred = (k) => true) {
    let args = [...arguments];
    obj = args.shift();
    let ret = {};
    if(pred instanceof RegExp) {
      let re = pred;
      pred = (str) => re.test(str);
    } else if(Util.isArray(pred)) {
      let a = pred;
      pred = (str) => a.indexOf(str) != -1;
    }
    for(let key in obj) if(pred(key)) ret[key] = obj[key];
    Object.setPrototypeOf(ret, Object.getPrototypeOf(obj));
    return ret;
  };
  Util.filterOutKeys = function (obj, arr) {
    if(typeof obj != 'object') return obj;

    return Util.filterKeys(obj, (key) => arr.indexOf(key) == -1);
  };
  Util.getKeys = function (obj, arr) {
    let ret = {};
    for(let key of arr) ret[key] = obj[key];

    return ret;
  };
  Util.numbersConvert = function (str) {
    return str
      .split('')
      .map((ch, i) => (/[ :,./]/.test(ch) ? ch : String.fromCharCode((str.charCodeAt(i) & 0x0f) + 0x30)))
      .join('');
  };
  Util.entries = function (arg) {
    let ret;
    if(Util.isObject(arg)) {
      ret =
        typeof arg.entries == 'function'
          ? arg.entries
          : function* () {
              for(let key in arg) yield [key, arg[key]];
            };
    }
    if(ret) return ret.call(arg);
  };
  Util.keys = function (arg) {
    let ret;
    if(Util.isObject(arg)) {
      ret =
        typeof arg.keys == 'function'
          ? arg.keys
          : function* () {
              for(let key in arg) yield key;
            };
    }
    if(ret) return ret.call(arg);
  };
  Util.values = function (arg) {
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

  Util.traverse = function (o, fn) {
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
  Util.traverseWithPath = function (o, rootPath = []) {
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
  Util.indexByPath = function (o, p) {
    for(let key of p) o = o[key];
    return o;
  };
  Util.pushUnique = function (...args) {
    arr = args.shift();
    args.forEach((item) => {
      if(arr.indexOf(item) == -1) arr.push(item);
    });
    return arr;
  };
  Util.insertSorted = function (arr, item, cmp = (a, b) => b - a) {
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
    fn = function (key, value) {
      insert(key, value);
      next(key, value);
      return fn;
    };
    fn.dest = dest;
    fn.insert = insert;
    return fn;
  };

  Util.mapAdapter = (getSetFunction) => {
    let r = {
      get(key) {
        return getSetFunction(key);
      },
      set(key, value) {
        getSetFunction(key, value);
        return this;
      }
    };
    return Util.mapFunction(r);
  };

  /**
   * @param Array   forward
   * @param Array   backward
   *
   * component2path,  path2eagle  => component2eagle
   *  eagle2path, path2component =>
   */
  Util.mapFunction = (map) => {
    let fn;
    fn = function (key, value) {
      if(value === undefined) return fn.get(key);
      return fn.set(key, value);
    };

    fn.map = ((m) => {
      while(Util.isFunction(m) && m.map !== undefined) m = m.map;
      return m;
    })(map);

    fn.set = (key, value) => (map.set(key, value), (k, v) => fn(k, v));
    fn.get = (key) => map.get(key);

    if(typeof map.keys == 'function') fn.keys = () => map.keys();
    if(typeof map.has == 'function') fn.has = (key) => map.has(key);
    return fn;
  };

  Util.mapWrapper = (map, toKey = (key) => key, fromKey = (key) => key) => {
    let fn = Util.mapFunction(map);
    fn.set = (key, value) => (map.set(toKey(key), value), (k, v) => fn(k, v));
    fn.get = (key) => map.get(toKey(key));
    if(typeof map.keys == 'function') fn.keys = () => [...map.keys()].map(fromKey);
    if(typeof map.has == 'function') fn.has = (key) => map.has(toKey(key));

    fn.map = ((m) => {
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
    fn = function (key, value) {
      if(value === undefined) return fn.get(key);
      return fn.set(key, value);
    };

    /* prettier-ignore */
    fn.get=  forward.reduceRight((a,m) => makeGetter(m, key => a(key)), a => a);
    return fn;
    function makeGetter(map, next = (a) => a) {
      return (key) => (false && console.log('getter', { map, key }), next(map.get(key)));
    }
  };

  Util.predicate = (fn_or_regex) => {
    let fn;
    if(fn_or_regex instanceof RegExp) fn = (...args) => fn_or_regex.test(args + '');
    else fn = fn_or_regex;
    return fn;
  };
  Util.iterateMembers = function* (obj, predicate = (name, depth, obj, proto) => true, depth = 0) {
    let names = [];
    let pred = Util.predicate(predicate);
    const proto = Object.getPrototypeOf(obj);

    for(let name in obj) if(pred(name, depth, obj)) yield name;
    for(let name of Object.getOwnPropertyNames(obj)) {
      if(pred(name, depth, obj)) yield name;
    }
    for(let symbol of Object.getOwnPropertySymbols(obj)) if(pred(symbol, depth, obj)) yield symbol;
    if(proto) yield* Util.iterateMembers(proto, predicate, depth + 1);
  };

  Util.and = (...predicates) => (...args) => predicates.every((pred) => pred(...args));
  Util.or = (...predicates) => (...args) => predicates.some((pred) => pred(...args));

  Util.members = Util.curry((pred, obj) => Util.unique([...Util.iterateMembers(obj, Util.tryPredicate(pred))]));

  Util.memberNameFilter = (depth = 1, start = 0) =>
    Util.and((m, l, o) => start <= l && l < depth + start,
      (m, l, o) => typeof m != 'string' || ['caller', 'callee', 'constructor', 'arguments'].indexOf(m) == -1,
      (name, depth, obj, proto) => obj != Object.prototype
    );

  Util.getMemberNames = (obj, depth = Number.Infinity, start = 0) => Util.members(Util.memberNameFilter(depth, start))(obj);

  Util.objectReducer = (filterFn, accFn = (a, m, o) => ({ ...a, [m]: o[m] }), accu = {}) => (obj, ...args) =>
    Util.members(filterFn(...args), obj).reduce(Util.tryFunction(
        (a, m) => accFn(a, m, obj),
        (r, a, m) => r,
        (r, a) => a
      ),
      accu
    );

  Util.getMembers = Util.objectReducer(Util.memberNameFilter);

  Util.getMemberDescriptors = Util.objectReducer(Util.memberNameFilter, (a, m, o) => ({
    ...a,
    [m]: Object.getOwnPropertyDescriptor(o, m)
  }));

  Util.methodNameFilter = (depth = 1, start = 0) => Util.and((m, l, o) => typeof o[m] == 'function', Util.memberNameFilter(depth, start));

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

  Util.bindMethods = function (methods, obj) {
    for(let name in methods) {
      methods[name] = methods[name].bind(obj);
    }
    return methods;
  };
  Util.bindMethodsTo = function (dest, obj, methods) {
    for(let name in methods) {
      dest[name] = methods[name].bind(obj);
    }
    return dest;
  };
  Util.getConstructor = (obj) => obj.constructor || Object.getPrototypeOf(obj).constructor;
  Util.getPrototypeChain = function (obj, fn = (p) => p) {
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

  Util.getConstructorChain = (ctor, fn = (c, p) => c) => Util.getPrototypeChain(ctor, (p, o) => fn(o, p));

  Util.weakAssign = function (obj) {
    let args = [...arguments];
    obj = args.shift();
    args.forEach((other) => {
      for(let key in other) {
        if(obj[key] === undefined) obj[key] = other[key];
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
      e.proto = Object.getPrototypeOf(exc);

      if(callerStack) stack = callerStack;
    } else {
      const [message, callerStack] = args;
      e = { message };
      if(callerStack) stack = callerStack;
    }
    if(stack) e.stack = Util.stack(stack);

    return Object.setPrototypeOf(e, proto);
  };

  Util.define(Util.exception.prototype, {
    toString(color = false) {
      const { message, stack, proto } = this;
      return `${Util.fnName(proto.constructor || this.constructor)}: ${message}
Stack:${Util.stack.prototype.toString.call(stack, color, stack.columnWidths)}`;
    }, [Symbol.toStringTag]() {
      return this.toString(false);
    }, [Symbol.for('nodejs.util.inspect.custom')]() {
      return Util.exception.prototype.toString.call(this, true);
    }
  });
  Util.location = function Location(...args) {
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
      fileName = fileName.replace(new RegExp(Util.getURL() + '/', 'g'), '');
      let text = color ? new this.colorCtor() : '';
      const c = color ? (t, color) => text.write(t, color /*, [0, 0, 0]*/) : (t) => (text += t);
      const palette = Util.location.palettes[Util.isBrowser() ? 1 : 0];
      if(functionName) c(functionName.replace(/\s*\[.*/g, '').replace(/^Function\./, '') + ' ', palette[1]);

      c(fileName, palette[0]);
      c(':', palette[1]);
      c(lineNumber, palette[2]);
      c(':', palette[1]);
      c(columnNumber, palette[2]);
      return text;
    }, [Symbol.toStringTag]() {
      return Util.location.prototype.toString.call(this, false);
    }, [Symbol.for('nodejs.util.inspect.custom')]() {
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
    ['methodName', 'functionName', 'fileName', 'lineNumber', 'columnNumber', 'typeName'].forEach((prop) => {
      let fn = 'get' + Util.ucfirst(prop);
      if(frame[prop] === undefined && typeof frame[fn] == 'function') frame[prop] = frame[fn]();
    });
    if(Util.colorCtor) frame.colorCtor = Util.colorCtor;

    return Object.setPrototypeOf(frame, Util.stackFrame.prototype);
  };
  Util.define(Util.stackFrame.prototype, {
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
  });

  Util.define(Util.stackFrame.prototype, {
    colorCtor: null,
    get() {
      const { fileName, columnNumber, lineNumber } = this;
      return fileName ? `${fileName}:${lineNumber}:${columnNumber}` : null;
    },
    toString(color, columnWidths = [0, 0, 0, 0]) {
      let text = color ? new this.colorCtor() : '';
      const c = color ? (t, color) => text.write(t, color) : (t) => (text += t);
      let fields = ['functionName', 'fileName', 'lineNumber', 'columnNumber'];
      const colors = [
        [0, 255, 0],
        [255, 255, 0],
        [0, 255, 255],
        [0, 255, 255]
      ];

      //const { functionName, fileName, columnNumber, lineNumber } = this;
      let columns = fields.map((fn) => this[fn]);

      columns = columns.map((f, i) => (f + '')[i >= 2 ? 'padStart' : 'padEnd'](columnWidths[i] || 0, ' '));
      // columns = columns.map((fn, i) => c(fn, colors[i]));

      const [functionName, fileName, lineNumber, columnNumber] = columns;
      //console.log('stackFrame.toString', { color ,columnWidths, functionName, fileName, lineNumber, columnNumber});

      return `${functionName} ${fileName}:${lineNumber}:${columnNumber}` + c('', 0);
    },
    getLocation() {
      return new Util.location(this);
    },
    get location() {
      return this.getLocation();
    }, [Symbol.toStringTag]() {
      return this.toString(false);
    }, [Symbol.for('nodejs.util.inspect.custom')](...args) {
      return Util.stackFrame.prototype.toString.call(this, true, this.columnWidths);
    }
  });
  Util.scriptName = () =>
    Util.tryCatch(() => process.argv[1],
      (script) => script + '',
      () => Util.getURL()
    );
  Util.getFunctionName = () => {
    const frame = Util.getCallerStack(2)[0];
    return frame.getFunctionName() || frame.getMethodName();
  };

  Util.scriptDir = () =>
    Util.tryCatch(() => Util.scriptName(),
      (script) => (script + '').replace(/\/[^/]*$/g, ''),
      () => Util.getURL()
    );
  Util.stack = function Stack(stack) {
    if(Util.isArray(stack)) {
      stack = [...stack].map((f) => f + '').join('\n');
    }

    if(typeof stack == 'string') {
      stack = stack.split(/\n/g).slice(1);
      const re = new RegExp('.* at ([^ ][^ ]*) \\(([^)]*)\\)');
      stack = stack.map((frame) =>
        typeof frame == 'string'
          ? frame
              .replace(/^\s*at\s+/, '')
              .split(/[()]+/g)
              .map((part) => part.trim())
          : frame
      );
      stack = stack.map((frame) => (Util.isArray(frame) ? (frame.length < 2 ? ['', ...frame] : frame).slice(0, 2) : frame));
      stack = stack.map(([func, file]) => [
        func,
        file
          .split(/:/g)
          .reverse()
          .map((n) => (!isNaN(+n) ? +n : n))
      ]);
      stack = stack.map(([func, file]) => [func, file.length >= 3 ? file : ['', '', ...file]]);
      stack = stack.map(([func, [columnNumber, lineNumber, ...file]]) => ({
        functionName: func,
        fileName: file.reverse().join(':'),
        lineNumber,
        columnNumber
      }));
      stack = stack.map(({ functionName: func, fileName: file, columnNumber: column, lineNumber: line }) => ({
        functionName: func,
        fileName: file.replace(new RegExp(Util.getURL() + '/', 'g'), ''),
        lineNumber: line,
        columnNumber: column
      }));
    } else {
      stack = Util.getCallers(2, Number.MAX_SAFE_INTEGER, () => true, stack);
    }
    stack = stack.map((frame) => Object.setPrototypeOf(frame, Util.stackFrame.prototype));

    //stack =stack.map(f => f+'');

    stack = Object.setPrototypeOf(stack, Util.stack.prototype);
    //console.log('Util.stack:', stack.toString(true));
    return stack;
  };

  Util.stack.prototype = Object.assign(Util.stack.prototype, Util.getMethods(new Array(), 1, 1));
  Object.defineProperty(Util.stack, Symbol.species, { get: () => Util.stack });

  Util.stack.prototype = Object.assign(Util.stack.prototype, {
    toString(color = false) {
      let columns = this.columnWidths;
      let a = [...this].map((frame) => Util.stackFrame.prototype.toString.call(frame, color, columns));
      let s = a.join('\n');
      return s + '\n';
    }, [Symbol.toStringTag]() {
      return Util.stack.prototype.toString.call(this);
    }, [Symbol.for('nodejs.util.inspect.custom')](...args) {
      //const fields = ['functionName','fileName','lineNumber','columnNumber'];
      //this.columnWidths = this.reduce((a,f) => fields.slice(0,1).map((fn,i) => Math.max(a[i],(f[fn]+'').length)), [0,0,0,0]);
      return '\n' + this.map((f) => f.toString(!Util.isBrowser(), this.columnWidths)).join('\n');
    }
  });

  Object.defineProperties(Util.stack.prototype, {
    columnWidths: {
      get() {
        return this.reduce((a, f) => ['functionName'].map((fn, i) => Math.max(a[i], (f[fn] + '').length)), [0, 0, 0, 0]);
      }
    }
  });

  Util.getCallerStack = function (position = 2) {
    Error.stackTraceLimit = 100;
    if(position >= Error.stackTraceLimit) {
      throw new TypeError(`getCallerFile(position) requires position be less then Error.stackTraceLimit but position was: \`${position}\` and Error.stackTraceLimit was: \`${Error.stackTraceLimit}\``);
    }
    const oldPrepareStackTrace = Error.prepareStackTrace;
    Error.prepareStackTrace = (_, stack) => stack;
    let stack = new Error().stack;

    stack = Util.stack(stack);
    stack = stack.slice(position);

    /*
  stack.forEach(frame => {
    Util.define(frame, 'toString', function() {
      const { position } = this;
      let s = position;
      if(this.getMethodName()) s = this.getMethodName() + ' ' + s;
      else if(this.getFunctionName()) s = this.getFunctionName() + ' ' + s;

      const w =
        Util.tryCatch(() => globalThis.window,
          w => w,
          () => null
        ) || {};

      w.frameProto = Object.getPrototypeOf(frame);

      w.CallSite = w.frameProto.constructor;
      if(this.getTypeName()) s = this.getTypeName() + '.' + s;

      return s;
    });
    Util.define(frame, {
      get location() {
        const fileName = this.getFileName().replace(new RegExp(Util.getURL() + '/', 'g'), '');
        const lineNumber = this.getLineNumber();
        const columnNumber = this.getColumnNumber();

        return [fileName, lineNumber, columnNumber].join(':');
      }
    });

    Util.defineGetter(frame, 'position', function() {
      return this.getFileName() ? `${this.getFileName()}:${this.getLineNumber()}:${this.getColumnNumber()}` : null;
    });
  });
  Util.define(stack, {
    toString() {
      return this.map(frame => frame.toString()).join('\n');
    },

    [Symbol.toStringTag]() {
      return this.toString();
    }
  });
  Util.tryCatch(() => (window.stack = stack));*/
    return stack;
  };
  Util.getCallerFile = function (position = 2) {
    let stack = Util.getCallerStack();
    if(stack !== null && typeof stack === 'object') {
      const frame = stack[position];
      return frame ? `${frame.getFileName()}:${frame.getLineNumber()}` : undefined;
    }
  };
  Util.getCallerFunction = function (position = 2) {
    let stack = Util.getCallerStack(position + 1);
    if(stack !== null && typeof stack === 'object') {
      const frame = stack[0];
      return frame ? frame.getFunction() : undefined;
    }
  };
  Util.getCallerFunctionName = function (position = 2) {
    let stack = Util.getCallerStack(position + 1);
    if(stack !== null && typeof stack === 'object') {
      const frame = stack[0];
      return frame ? frame.getMethodName() || frame.getFunctionName() : undefined;
    }
  };
  Util.getCallerFunctionNames = function (position = 2) {
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
  Util.getCaller = function (index = 1, stack) {
    const methods = ['getThis', 'getTypeName', 'getFunction', 'getFunctionName', 'getMethodName', 'getFileName', 'getLineNumber', 'getColumnNumber', 'getEvalOrigin', 'isToplevel', 'isEval', 'isNative', 'isConstructor'];
    //stack = stack || Util.getCallerStack(index+1);
    //console.log("stack:", stack)

    const frame = stack[index];
    //console.log("frame keys:",frame, Util.getMemberNames(frame, 0, 10));
    return methods.reduce((acc, m) => {
      if(frame[m]) {
        const name = m == 'getThis' ? 'thisObj' : Util.lcfirst(m.replace(/^(get|is)/, ''));

        let value = frame[m]();
        if(typeof value == 'string') value = value.replace('file://' + process.cwd() + '/', '');
        if(value !== undefined) {
          acc[name] = value;
        }
      }
      return acc;
    }, Util.stack.prototype);
  };
  Util.getCallers = function (start = 2, num = Number.MAX_SAFE_INTEGER, pred = () => true, stack) {
    stack = stack || Util.getCallerStack(start + 1);
    let ret = [];
    let i = 0;
    while(i < num && stack[i] !== undefined) {
      try {
        let frame = Util.getCaller(i, stack);
        if(pred(frame)) {
          //if(frame === null) break;
          ret.push(frame);
        }
      } catch(err) {}
      i++;
    }
    ret.toString = function () {
      return this.map((frame) => frame.toString()).join('\n');
    };
    ret[Symbol.toStringTag] = function () {
      return this.toString();
    };
    return ret;
  };

  /*Object.defineProperty(Util, 'stackFrame', {
  get: function() {
  return this.getCallerStack(2);
  }
});*/
  Util.getStackFrame = function (offset = 2) {
    let frames = Util.getCallerStack(0);
    frames = frames.map((frame) => {
      if(Object.getPrototypeOf(frame) !== Util.stackFrame.prototype) frame = Util.stackFrame(frame);
      return frame;
    });

    return frames[offset];
  };
  Util.rotateLeft = function (x, n) {
    n = n & 0x1f;
    return (x << n) | ((x >> (32 - n)) & ~((-1 >> n) << n));
  };
  Util.rotateRight = function (x, n) {
    n = n & 0x1f;
    return Util.rotateLeft(x, 32 - n);
  };
  Util.hashString = function (string, bits = 32, mask = 0xffffffff) {
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
  Util.flatTree = function (tree, addOutput) {
    const ret = [];
    if(!addOutput) addOutput = (arg) => ret.push(arg);
    addOutput(Util.filterKeys(tree, (key) => key !== 'children'));
    if(typeof tree.children == 'object' && tree.children !== null && tree.children.length) for(let child of tree.children) Util.flatTree(child, addOutput);
    return ret;
  };
  Util.traverseTree = function (tree, fn, depth = 0, parent = null) {
    fn(tree, depth, parent);
    if(Util.isObject(tree, tree.childre) && tree.children.length > 0) for(let child of tree.children) Util.traverseTree(child, fn, depth + 1, tree);
  };

  Util.walkTree = function (node, pred, t, depth = 0, parent = null) {
    return (function* () {
      if(!pred) pred = (i) => true;
      if(!t)
        t = function (i) {
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

  Util.isPromise = function (obj) {
    return (Boolean(obj) && typeof obj.then === 'function') || obj instanceof Promise;
  };

  /* eslint-disable no-use-before-define */
  if(typeof setImmediate !== 'function') var setImmediate = (fn) => setTimeout(fn, 0);
  Util.next = function (iter, observer, prev = undefined) {
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
        .then((val) => {
          observer.next(val);
          setImmediate(() => Util.next(iter, observer, val));
        })
        .catch((err) => observer.error(err));
    } else {
      observer.next(value);
      setImmediate(() => Util.next(iter, observer, value));
    }
  };
  Util.getImageAverageColor = function (imageElement, options) {
    if(!imageElement) {
      return false;
    }
    options = options || {};
    const settings = {
      tooDark: (options.tooDark || 0.03) * 255 * 3, //How dark is too dark for a pixel
      tooLight: (options.tooLight || 0.97) * 255 * 3, //How light is too light for a pixel
      tooAlpha: (options.tooAlpha || 0.1) * 255 //How transparent is too transparent for a pixel
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
    let luma = 0; //Having luma in the pixel object caused ~10% performance penalty for some reason
    //Loop through the rgba data
    for(let i = 0, l = w * h * 4; i < l; i += 4) {
      pixel.r = subpixels[i];
      pixel.g = subpixels[i + 1];
      pixel.b = subpixels[i + 2];
      pixel.a = subpixels[i + 4];
      //Only consider pixels that aren't black, white, or too transparent
      if(pixel.a > settings.tooAlpha &&
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
        const toHex = function (d) {
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
  Util.jsonToObject = function (jsonStr) {
    let ret = null;
    try {
      ret = JSON.parse(jsonStr);
    } catch(error) {
      let pos = +('' + error)
        .split('\n')
        .reverse()[0]
        .replace(/.*position ([0-9]+).*/, '$1');
      console.error('Unexpected token: ', jsonStr);
      console.error('Unexpected token at:', jsonStr.substring(pos));
      ret = null;
    }
    return ret;
  };
  Util.splitLines = function (str, max_linelen = Number.MAX_SAFE_INTEGER) {
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
  Util.matchAll = Util.curry(function* (re, str) {
    re = new RegExp(re + '', 'g');
    let match;
    while((match = re.exec(str)) != null) yield match;
  });
  Util.decodeEscapes = function (text) {
    let matches = [...Util.matchAll(/([^\\]*)(\\u[0-9a-f]{4}|\\)/gi, text)];
    if(matches.length) {
      matches = matches.map((m) => [...m].slice(1)).map(([s, t]) => s + String.fromCodePoint(parseInt(t.substring(2), 16)));
      text = matches.join('');
    }
    return text;
  };

  Util.stripXML = (text) =>
    text
      .replace(/<br(|\ *\/)>/gi, '\n')
      .replace(/<[^>]*>/g, '')
      .replace(/[\t ]+/g, ' ')
      .replace(/(\n[\t ]*)+\n/g, '\n');
  Util.stripNonPrintable = (text) => text.replace(/[^\x20-\x7f\x0a\x0d\x09]/g, '');
  Util.decodeHTMLEntities = function (text) {
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
    return text.replace(/&([^;]+);/gm, (match, entity) => entities[entity] || match);
  };
  Util.encodeHTMLEntities = (str, charset = '\u00A0-\u9999<>&') => str.replace(new RegExp(`[${charset}](?!#)`, 'gim'), (i) => '&#' + i.charCodeAt(0) + ';');

  Util.stripAnsi = function (str) {
    return (str + '').replace(/\x1B[[(?);]{0,2}(;?\d)*./g, '');
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

  Util.propertyLookup = (obj = {}, handler = (key) => null) =>
    Util.proxy(obj, {
      get(target, key, receiver) {
        return handler(key);
      }
    });

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
  Util.construct = (constructor) => {
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

  Util.immutable = (args) => {
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
    let initialProto = proto.map((p) =>
      Util.isArrowFunction(p)
        ? p
        : (ctor) => {
            for(let n in p) ctor.prototype[n] = p[n];
          }
    );
    let body = ` class ${imName} extends ${name} {
  constructor(...args) {
    super(...args);
    //this.constructor = ${imName};
    //this[Symbol.species] = ${imName}[Symbol.species] || ${imName};
    if(new.target === ${imName})
      return Object.freeze(this);
  }
};

${imName}.prototype.constructor = ${imName};

return ${imName};`;
    for(let p of initialProto) p(orig);
    let ctor; // = new Function(name, body)(orig);
    //console.log('immutableClass', { initialProto, body }, orig);
    let species = ctor;
    /* prettier-ignore */ Util.defineGetterSetter(ctor, Symbol.species, () => species, (value) => {species = value; });
    return ctor;
  };

  Util.partial = function partial(fn /*, arg1, arg2 etc */) {
    let partialArgs = [].slice.call(arguments, 1);
    if(!partialArgs.length) {
      return fn;
    }
    return function () {
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
            return args.join('');
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
            let out = args.shift();
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
            return `\u001b[${[...args].join(';')}m`;
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
  Util.stripAnsi = (str) => {
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

  Util.defineInspect = (proto, ...props) => {
    if(!Util.isBrowser()) {
      const c = Util.coloring();
      proto[Symbol.for('nodejs.util.inspect.custom')] = function () {
        const obj = this;
        return (c.text(Util.fnName(proto.constructor) + ' ', 1, 31) +
          Util.toString(props.reduce((acc, key) => {
              acc[key] = obj[key];
              return acc;
            }, {}),
            { multiline: false, colon: ':', spacing: '', separator: ', ', padding: ' ' }
          )
        );
      };
    }
  };

  Util.inRange = Util.curry((a, b, value) => value >= a && value <= b);

  Util.bindProperties = (proxy, target, props, gen) => {
    if(props instanceof Array) props = Object.fromEntries(props.map((name) => [name, name]));
    const propNames = Object.keys(props);
    if(!gen) gen = (p) => (v) => (v === undefined ? target[p] : (target[p] = v));
    Object.defineProperties(proxy,
      propNames.reduce(
        (a, k) => {
          const prop = props[k];
          const get_set = typeof prop == 'function' ? prop : gen(prop);
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
    return (obj) => {
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
    on: (val) => val == 'on' || val === 'true' || val === true,
    off: (val) => val == 'off' || val === 'false' || val === false,
    true: (val) => val === 'true' || val === true,
    false: (val) => val === 'false' || val === false
  });

  Util.assignGlobal = () => Util.weakAssign(Util.getGlobalObject(), Util);

  Util.weakMapper = (createFn, map = new WeakMap()) => {
    let self = function (obj, ...args) {
      let ret;
      if(map.has(obj)) {
        ret = map.get(obj);
      } else {
        ret = createFn(obj, ...args);
        //if(ret !== undefined)
        map.set(obj, ret);
      }
      return ret;
    };
    self.set = (k, v) => map.set(k, v);
    self.get = (k) => map.get(k);
    self.map = map;
    return self;
  };
  Util.merge = (...args) => args.reduce((acc, arg) => ({ ...acc, ...arg }), {});

  Util.weakAssoc = (fn = (value, ...args) => Object.assign(value, ...args)) => {
    let mapper = Util.tryCatch(() => new WeakMap(),
      (map) => Util.weakMapper((obj, ...args) => Util.merge(...args), map),
      () => (obj, ...args) => Util.define(obj, ...args)
    );
    return (obj, ...args) => {
      let value = mapper(obj, ...args);
      return fn(value, ...args);
    };
  };
  Util.transformer = (a, ...l) =>
    (l || []).reduce((c, f) =>
        function (...v) {
          return f.apply(this, [c.apply(this, v), ...v]);
        },
      a
    );
  Util.proxyObject = (root, handler) => {
    const ptr = (path) => path.reduce((a, i) => a[i], root);
    const nodes = Util.weakMapper((value, path) =>
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

      let proxy = nodes(value, path);

      return proxy;
    }

    return node([]);
  };
  Util.parseXML = (xmlStr) =>
    Util.tryCatch(() => new DOMParser(),
      (parser) => parser.parseFromString(xmlStr, 'application/xml')
    );

  Util.once = (fn) => {
    let done = false;
    let ret;

    return (...args) => {
      if(!done) {
        ret = fn(...args);
        done = true;
      }
      return ret;
    };
  };
  Util.copyTextToClipboard = (i, { target: t } = {}) => {
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
  };

  Util.toPlainObject = (obj, t = (v, n) => v) => [...Util.getMemberNames(obj)].reduce((acc, k) => ({ ...acc, [k]: t(obj[k], k) }), {});

  Util.timer = (msecs) => {
    let ret, id, rej, createdTime, startTime, stopTime, endTime, res, delay, n, timer;
    createdTime = new Date();
    const remaining = () => {
      let r = startTime + msecs - (typeof stopTime == 'number' ? stopTime : new Date());
      return r >= 0 ? r : 0;
    };
    const finish = (callback) => {
      stopTime = new Date();
      if(stopTime.valueOf() > endTime.valueOf()) stopTime = endTime;
      if(typeof callback == 'function') callback(stopTime);
      res((n = remaining()));
    };
    const log = (method, ...args) => console.log(`${Date.now() - createdTime.valueOf()} timer#${id}.${method}`, ...args.map((obj) => Util.toPlainObject(obj || {}, (v) => v || (v instanceof Date ? `+${v.valueOf() - createdTime}` : v))));
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
      /* prettier-ignore */ get delay() { return delay; },
      /* prettier-ignore */ get created() { return createdTime; },
      /* prettier-ignore */ get start() { return startTime || new Date(endTime.valueOf() - delay); },
      /* prettier-ignore */ get stop() { return stopTime instanceof Date ? stopTime : undefined; },
      /* prettier-ignore */ get elapsed() { return delay  + (stopTime || new Date()).valueOf() - endTime.valueOf(); },
      /* prettier-ignore */ get end() {return endTime; },
      /* prettier-ignore */ get remain() {return endTime.valueOf() - (stopTime || new Date()).valueOf(); },
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
        this.resume = function () {
          timeout(remain, this);
          this.pause = pause;
          delete this.resume;
          delete this.restart;
          log('resume', this);
          return this;
        };
        this.restart = function () {
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
  Util.thenableReject = (error) => ({
    then: (resolve, reject) => reject(error)
  });
  Util.wrapGenerator = (fn) =>
    Util.isGenerator(fn)
      ? function (...args) {
          return [...fn.call(this, ...args)];
        }
      : fn;

  Util.decorateIterable = (proto, generators = false) => {
    const methods = {
      forEach(fn, thisArg) {
        let i = 0;
        for(let item of this) fn.call(thisArg, item, i++, this);
      },
      *map(fn, thisArg) {
        let i = 0;

        for(let item of this) yield fn.call(thisArg, item, i++, this);
      },
      *filter(pred, thisArg) {
        let i = 0;

        for(let item of this) if(pred.call(thisArg, item, i++, this)) yield item;
      },
      findIndex(pred, thisArg) {
        let i = 0;
        for(let item of this) if(pred(item, i++, this)) return i;
        return -1;
      },
      find(pred, thisArg) {
        let i = 0;
        for(let item of this) if(pred(item, i++, this)) return item;
      },
      every(pred, thisArg) {
        let i = 0;
        for(let item of this) if(!pred(item, i++, this)) return false;
        return true;
      },
      some(pred, thisArg) {
        let i = 0;
        for(let item of this) if(pred(item, i++, this)) return true;
        return false;
      },
      reduce(fn, accu) {
        let i = 0;
        for(let item of this) accu = fn(accu, item, i++, this);
        return accu;
      }
    };
    Util.define(proto, methods);
    if(!generators) {
      for(let name in methods) {
        let gen = proto[name];
        proto[name] = Util.wrapGenerator(gen);
      }
    }

    return proto;
  };

  console.log('Util installed!');

  /**
   * Type for TopRightBottomLeft (paddings and margins)
   *
   * @param {string,object,array} arg [description]
   */
  function TRBL(arg) {
    let ret = this instanceof TRBL ? this : {};
    let args = [...arguments];

    if(typeof arg === 'object' && !arg instanceof Array) {
      Object.keys(arg).forEach((k) => {
        const matches = /(top|right|bottom|left)/i.exec(k);
        ret[matches[0].toLowerCase()] = parseInt(arg[k]);
      });
    } else if(arg) {
      if(args.length > 1) arg = args;

      if(typeof arg === 'string') arg = [...arg.matchAll(/^[0-9.]+(|px|em|rem|pt|cm|mm)$/g)];
      else if(arg.length == 4) arg = arg.map((v) => parseInt(v));

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

    //Util.log('ret: ', ret);

    if(!this || this === TRBL) return Object.assign(ret, TRBL.prototype);
  }

  TRBL.prototype.null = function () {
    return this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0;
  };
  TRBL.null = (trbl) => TRBL.prototype.null.call(trbl);

  TRBL.neg = (trbl = this) => ({
    top: -trbl.top,
    right: -trbl.right,
    bottom: -trbl.bottom,
    left: -trbl.left
  });

  TRBL.prototype.isNaN = function () {
    return isNaN(this.top) || isNaN(this.right) || isNaN(this.bottom) || isNaN(this.left);
  };
  Object.defineProperty(TRBL.prototype, 'inset', {
    get() {
      return (rect) => Rect.inset(rect, this);
    }
  });

  Object.defineProperty(TRBL.prototype, 'outset', {
    get() {
      return (rect) => Rect.outset(rect, this);
    }
  });

  /*TRBL.prototype.outset = function() {
  return this.inset.call(TRBL.neg(this));
};*/

  TRBL.prototype.add = function (other) {
    this.top += other.top;
    this.right += other.right;
    this.bottom += other.bottom;
    this.left += other.left;
  };

  TRBL.prototype.union = function (other) {
    this.top = other.top < this.top ? other.top : this.top;
    this.right = other.right > this.right ? other.right : this.right;
    this.bottom = other.bottom > this.bottom ? other.bottom : this.bottom;
    this.left = other.left < this.left ? other.left : this.left;
  };

  TRBL.prototype.toRect = function () {
    return new Rect({
      x: this.left,
      y: this.top,
      width: this.right - this.left,
      height: this.bottom - this.top
    });
  };
  TRBL.prototype.toRect = function () {
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

  TRBL.toRect = (trbl) => new Rect(trbl.left, trbl.top, trbl.right - trbl.left, trbl.bottom - trbl.top);

  TRBL.prototype.toString = function (unit = 'px') {
    return '' + this.top + '' + unit + ' ' + this.right + '' + unit + ' ' + this.bottom + '' + unit + ' ' + this.left + unit;
  };
  TRBL.prototype.toSource = function () {
    return '{top:' + this.top + ',right:' + this.right + ',bottom:' + this.bottom + ',left:' + this.left + '}';
  };

  for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
    TRBL[name] = (points) => TRBL.prototype[name].call(points);
  }

  function isTRBL(obj) {
    return top in obj && right in obj && bottom in obj && left in obj;
  }

  g.Point = Point;
  function Point(arg) {
    let args = arg instanceof Array ? arg : [...arguments];
    let p = this instanceof Point ? this : null;
    arg = args.shift();

    if(p === null) {
      if(arg instanceof Point) return arg;
      p = {};
    }

    if(typeof arg === 'undefined') {
      p.x = arg;
      p.y = args.shift();
    } else if(typeof arg === 'number') {
      p.x = parseFloat(arg);
      p.y = parseFloat(args.shift());
    } else if(typeof arg === 'string') {
      const matches = [...arg.matchAll(new RegExp('/([-+]?d*.?d+)(?:[eE]([-+]?d+))?/g'))];
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

    if(!this || this === Point) {
      if(p.prototype == Object) p.prototype = Point.prototype;
      else Object.assign(p, Point.prototype);
      return p;
    }
  }

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

  Point.prototype.move = function (x, y) {
    this.x += x;
    this.y += y;
    return this;
  };
  Point.prototype.moveTo = function (x, y) {
    this.x = x;
    this.y = y;
    return this;
  };
  Point.prototype.clear = function (x, y) {
    this.x = 0;
    this.y = 0;
    return this;
  };
  Point.prototype.set = function (fn) {
    if(typeof fn != 'function') {
      Point.apply(this, [...arguments]);
      return this;
    }
    return fn(this.x, this.y);
  };
  Point.prototype.clone = function () {
    const ctor = this[Symbol.species] || this.constructor[Symbol.species];

    return new ctor({ x: this.x, y: this.y });
  };
  Point.prototype.sum = function (...args) {
    const p = new Point(...args);
    let r = new Point(this.x, this.y);
    r.x += p.x;
    r.y += p.y;
    return r;
  };
  Point.prototype.add = function (...args) {
    const other = new Point(...args);
    this.x += other.x;
    this.y += other.y;
    return this;
  };
  Point.prototype.diff = function (...args) {
    const p = new Point(...args);
    let r = new Point(this.x, this.y);
    //Util.log({ t: this, args, p, r });
    r.x -= p.x;
    r.y -= p.y;
    return r;
  };
  Point.prototype.sub = function (...args) {
    const other = new Point(...args);
    this.x -= other.x;
    this.y -= other.y;
    return this;
  };
  Point.prototype.prod = function (f) {
    const o = isPoint(f) ? f : { x: f, y: f };
    return new Point(this.x * o.x, this.y * o.y);
  };
  Point.prototype.mul = function (f) {
    const o = isPoint(f) ? f : { x: f, y: f };
    this.x *= o.x;
    this.y *= o.y;
    return this;
  };
  Point.prototype.quot = function (other) {
    other = isPoint(other) ? other : { x: other, y: other };
    return new Point(this.x / other.x, this.y / other.y);
  };
  Point.prototype.div = function (other) {
    other = isPoint(other) ? other : { x: other, y: other };
    this.x /= other.x;
    this.y /= other.y;
    return this;
  };
  Point.prototype.comp = function () {
    return new Point({ x: -this.x, y: -this.y });
  };
  Point.prototype.neg = function () {
    this.x *= -1;
    this.y *= -1;
    return this;
  };
  Point.prototype.distanceSquared = function (other = { x: 0, y: 0 }) {
    return (other.y - this.y) * (other.y - this.y) + (other.x - this.x) * (other.x - this.x);
  };
  Point.prototype.distance = function (other = { x: 0, y: 0 }) {
    return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
  };
  Point.prototype.equals = function (other) {
    //Util.log(`Point.equals ${this} ${other}`);
    return +this.x == +other.x && +this.y == +other.y;
  };
  Point.prototype.round = function (precision = 0.001, digits, type = 'round') {
    let { x, y } = this;
    this.x = Util.roundTo(x, precision, digits, type);
    this.y = Util.roundTo(y, precision, digits, type);
    return this;
  };
  Point.prototype.sides = function () {
    return {
      top: this.y,
      right: this.x + this.w1idth,
      bottom: this.y + this.height,
      left: this.x
    };
  };
  Point.prototype.dot = function (other) {
    return this.x * other.x + this.y * other.y;
  };
  Point.prototype.fromAngle = function (angle, dist = 1.0) {
    this.x = Math.cos(angle) * dist;
    this.y = Math.sin(angle) * dist;
    return this;
  };
  Point.prototype.toAngle = function (deg = false) {
    return Math.atan2(this.x, this.y) * (deg ? 180 / Math.PI : 1);
  };
  Point.prototype.angle = function (other, deg = false) {
    other = other || { x: 0, y: 0 };
    return Point.prototype.diff.call(this, other).toAngle(deg);
  };
  Point.prototype.rotate = function (angle, origin = { x: 0, y: 0 }) {
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
  Point.prototype.dimension = function () {
    return [this.width, this.height];
  };
  Util.defineGetter(Point.prototype, Symbol.iterator, function () {
    const { x, y } = this;
    let a = [x, y];
    return a[Symbol.iterator].bind(a);
  });

  Point.prototype.toString = function (opts = {}) {
    const { precision = 0.001, unit = '', separator = ',', left = '', right = '' } = opts;
    const x = Util.roundTo(this.x, precision);
    const y = Util.roundTo(this.y, precision);
    return `${left}${x}${unit}${separator}${y}${unit}${right}`;
  };
  Util.defineGetterSetter(Point.prototype,
    Symbol.toStringTag,
    function () {
      return `Point{ ${Point.prototype.toSource.call(this)}`;
    },
    () => {},
    false
  );

  Point.prototype.toSource = function (opts = {}) {
    const { asArray = false, pad = (a) => a /*a.padStart(4, ' ')*/, showNew = true } = opts;
    let x = pad(this.x + '');
    let y = pad(this.y + '');

    if(typeof this != 'object' || this === null) return '';
    if(asArray) return `[${x},${y}]`;
    return `${Util.colorText(showNew ? 'new ' : '', 1, 31)}${Util.colorText('Point', 1, 33)}${Util.colorText('(', 1, 36)}${Util.colorText(x, 1, 32)}${Util.colorText(',', 1, 36)}${Util.colorText(y, 1, 32)}${Util.colorText(')', 1, 36)}`;
  };

  /*Point.prototype.toSource = function() {
  return '{x:' + this.x + ',y:' + this.y + '}';
};*/
  Point.prototype.toObject = function () {
    const { x, y } = this;
    const obj = { x, y };
    Object.setPrototypeOf(obj, Point.prototype);
    return obj;
  };
  Point.prototype.toCSS = function (precision = 0.001) {
    return {
      left: Util.roundTo(this.x, precision) + 'px',
      top: Util.roundTo(this.y, precision) + 'px'
    };
  };
  Point.prototype.toFixed = function (digits) {
    return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
  };
  Point.prototype.isNull = function () {
    return this.x == 0 && this.y == 0;
  };
  Point.prototype.inside = function (rect) {
    return this.x >= rect.x && this.x < rect.x + rect.width && this.y >= rect.y && this.y < rect.y + rect.height;
  };

  /*Point.prototype.transform = function(m) {
  Matrix.prototype.transform_point.call(m, this);
  return this;
};*/
  Point.prototype.normalize = function (minmax) {
    return new Point({
      x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1),
      y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1)
    });
  };

  Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
  Point.angle = (point, other, deg = false) => Point.prototype.angle.call(point, other, deg);
  Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
  Point.sub = (point, other) => Point.prototype.sub.call(point, other);
  Point.prod = (a, b) => Point.prototype.prod.call(a, b);
  Point.quot = (a, b) => Point.prototype.quot.call(a, b);
  Point.equals = (a, b) => {
    let ret = Point.prototype.equals.call(a, b);
    return ret;
  };
  Point.round = (point, prec) => Point.prototype.round.call(point, prec);
  Point.fromAngle = (angle, f) => Point.prototype.fromAngle.call(new Point(0, 0), angle, f);

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
    Point[name] = (point, ...args) => Point.prototype[name].call(point || new Point(point), ...args);
  }

  Point.toSource = (point) => `{ x:${point.x}, y: ${point.y} }`;

  const isPoint = (o) => o && ((o.x !== undefined && o.y !== undefined) || ((o.left !== undefined || o.right !== undefined) && (o.top !== undefined || o.bottom !== undefined)) || o instanceof Point || Object.getPrototypeOf(o).constructor === Point);

  Point.isPoint = isPoint;
  Util.defineInspect(Point.prototype, 'x', 'y');

  Point.bind = (o, p, gen) => {
    const [x, y] = p || ['x', 'y'];
    if(!gen) gen = (k) => (v) => (v === undefined ? o[k] : (o[k] = v));
    return Util.bindProperties(new Point(0, 0), o, { x, y }, gen);
  };
  Util.defineGetter(Point, Symbol.species, function () {
    return this;
  });

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

  Size.prototype.width = NaN;
  Size.prototype.height = NaN;
  Size.prototype.units = null;

  Size.prototype.convertUnits = function (w = 'window' in global ? window : null) {
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

  Size.prototype.aspect = function () {
    return this.width / this.height;
  };
  Size.prototype.toCSS = function (units) {
    let ret = {};
    units = typeof units == 'string' ? { width: units, height: units } : units || this.units || { width: 'px', height: 'px' };
    if(this.width !== undefined) ret.width = this.width + (units.width || 'px');
    if(this.height !== undefined) ret.height = this.height + (units.height || 'px');
    return ret;
  };
  Size.prototype.transform = function (m) {
    this.width = m.xx * this.width + m.yx * this.height;
    this.height = m.xy * this.width + m.yy * this.height;
    return this;
  };
  Size.prototype.isSquare = function () {
    return Math.abs(this.width - this.height) < 1;
  };
  Size.prototype.area = function () {
    return this.width * this.height;
  };
  Size.prototype.resize = function (width, height) {
    this.width = width;
    this.height = height;
    return this;
  };

  Size.prototype.sum = function (other) {
    return new Size(this.width + other.width, this.height + other.height);
  };
  Size.prototype.add = function () {
    for(let other of [...arguments]) {
      this.width += other.width;
      this.height += other.height;
    }
    return this;
  };
  Size.prototype.diff = function (other) {
    return new Size(this.width - other.width, this.height - other.height);
  };
  Size.prototype.sub = function () {
    for(let other of [...arguments]) {
      this.width -= other.width;
      this.height -= other.height;
    }
    return this;
  };
  Size.prototype.prod = function (f) {
    const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
    return new Size(this.width * o.width, this.height * o.height);
  };
  Size.prototype.mul = function (...args) {
    for(let f of args) {
      const o = isSize(f) ? f : isPoint(f) ? { width: f.x, height: f.y } : { width: f, height: f };
      this.width *= o.width;
      this.height *= o.height;
    }
    return this;
  };
  Size.prototype.quot = function (other) {
    return new Size(this.width / other.width, this.height / other.height);
  };
  Size.prototype.inverse = function (other) {
    return new Size(1 / this.width, 1 / this.height);
  };
  Size.prototype.div = function (f) {
    for(let f of [...arguments]) {
      this.width /= f;
      this.height /= f;
    }
    return this;
  };
  Size.prototype.round = function (precision = 0.001, digits) {
    let { width, height } = this;
    this.width = Util.roundTo(width, precision, digits);
    this.height = Util.roundTo(height, precision, digits);
    return this;
  };
  Size.prototype.bounds = function (other) {
    let w = [Math.min(this.width, other.width), Math.max(this.width, other.width)];
    let h = [Math.min(this.height, other.height), Math.max(this.height, other.height)];

    let scale = h / this.height;
    this.mul(scale);
    return this;
  };

  Size.prototype.fit = function (size) {
    size = new Size(size);
    let factors = Size.prototype.fitFactors.call(this, size);
    let ret = [Size.prototype.prod.call(this, factors[0]), Size.prototype.prod.call(this, factors[1])];
    return ret;
  };

  Size.prototype.fitHeight = function (other) {
    other = new Size(other);
    let scale = other.height / this.height;
    this.mul(scale);
    return [this.width, other.width];
  };
  Size.prototype.fitWidth = function (other) {
    other = new Size(other);
    let scale = other.width / this.width;
    this.mul(scale);
    return [this.height, other.height];
  };
  Size.prototype.fitFactors = function (other) {
    const hf = other.width / this.width;
    const vf = other.height / this.height;
    return [hf, vf];
  };
  Size.prototype.toString = function (opts = {}) {
    const { unit = '', separator = 'x', left = '', right = '' } = opts;
    const { width, height } = this;
    return `${left}${width}${unit}${separator}${height}${unit}${right}`;
  };
  Size.area = (sz) => Size.prototype.area.call(sz);
  Size.aspect = (sz) => Size.prototype.aspect.call(sz);

  Size.bind = (o, p, gen) => {
    const [width, height] = p || ['width', 'height'];
    if(!gen) gen = (k) => (v) => (v === undefined ? o[k] : (o[k] = v));
    return Util.bindProperties(new Size(0, 0), o, { width, height }, gen);
  };

  for(let method of Util.getMethodNames(Size.prototype)) Size[method] = (size, ...args) => Size.prototype[method].call(size || new Size(size), ...args);

  const isSize = (o) => o && ((o.width !== undefined && o.height !== undefined) || (o.x !== undefined && o.x2 !== undefined && o.y !== undefined && o.y2 !== undefined) || (o.left !== undefined && o.right !== undefined && o.top !== undefined && o.bottom !== undefined));

  for(let name of ['toCSS', 'isSquare', 'round', 'sum', 'add', 'diff', 'sub', 'prod', 'mul', 'quot', 'div']) {
    Size[name] = (size, ...args) => Size.prototype[name].call(size || new Size(size), ...args);
  }

  function Rect(arg) {
    let obj = this instanceof Rect ? this : {};
    let args = arg instanceof Array ? arg : [...arguments];
    let ret;

    if(typeof args[0] == 'number') arg = args;
    else if(Util.isObject(args[0]) && args[0].length !== undefined) arg = args.shift();

    ['x', 'y', 'width', 'height'].forEach((field) => {
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
    } else if(arg && arg.length >= 4 && arg.slice(0, 4).every((arg) => !isNaN(parseFloat(arg)))) {
      let x = arg.shift();
      let y = arg.shift();
      let w = arg.shift();
      let h = arg.shift();
      obj.x = typeof x === 'number' ? x : parseFloat(x);
      obj.y = typeof y === 'number' ? y : parseFloat(y);
      obj.width = typeof w === 'number' ? w : parseFloat(w);
      obj.height = typeof h === 'number' ? h : parseFloat(h);
      ret = 4;
    } else if(arg && arg.length >= 2 && arg.slice(0, 2).every((arg) => !isNaN(parseFloat(arg)))) {
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

  Rect.prototype.clone = function (fn) {
    const ctor = this.constructor[Symbol.species];
    let ret = new ctor(this.x, this.y, this.width, this.height);
    if(fn) fn(ret);
    return ret;
  };
  Rect.prototype.corners = function () {
    const rect = this;
    return [
      { x: rect.x, y: rect.y },
      { x: rect.x + rect.width, y: rect.y },
      { x: rect.x + rect.width, y: rect.y + rect.height },
      { x: rect.x, y: rect.y + rect.height }
    ];
  };

  if(Rect.prototype.isSquare === undefined) {
    Rect.prototype.isSquare = function () {
      return Math.abs(this.width - this.height) < 1;
    };
  }
  Rect.prototype.constructor = Rect;
  Rect.prototype.getArea = function () {
    return this.width * this.height;
  };
  Rect.prototype.toString = function (opts = {}) {
    if(typeof opts == 'string') opts = { separator: opts };
    const { precision = 0.001, unit = '', separator = ' ', left = '', right = '' } = opts;

    return left + Point.prototype.toString.call(this, opts) + separator + Size.prototype.toString.call(this, opts) + right;
  };
  Rect.prototype.toSource = function (opts = {}) {
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
  Object.defineProperty(Rect.prototype, 'center', {
    get() {
      return Rect.center(this);
    }
  });
  Object.defineProperty(Rect.prototype, 'size', {
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
  });
  Rect.prototype.points = function (ctor = (items) => Array.from(items)) {
    const c = this.corners();
    return ctor(c);
  };
  Rect.prototype.toCSS = Rect.toCSS;

  Rect.prototype.scale = function (factor) {
    let width = this.width * factor;
    let height = this.height * factor;

    this.x += (width - this.width) / 2;
    this.y += (height - this.height) / 2;
    this.width = width;
    this.height = height;
    return this;
  };
  Rect.prototype.mul = function (...args) {
    Point.prototype.mul.call(this, ...args);
    Size.prototype.mul.call(this, ...args);
    return this;
  };

  Rect.prototype.div = function (...args) {
    Point.prototype.div.call(this, ...args);
    Size.prototype.div.call(this, ...args);
    return this;
  };
  Rect.prototype.outset = function (trbl) {
    if(typeof trbl == 'number') trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
    this.x -= trbl.left;
    this.y -= trbl.top;
    this.width += trbl.left + trbl.right;
    this.height += trbl.top + trbl.bottom;
    return this;
  };
  Rect.prototype.inset = function (trbl) {
    if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);
    if(trbl.left + trbl.right < this.width && trbl.top + trbl.bottom < this.height) {
      this.x += trbl.left;
      this.y += trbl.top;
      this.width -= trbl.left + trbl.right;
      this.height -= trbl.top + trbl.bottom;
    }
    return this;
  };
  Rect.prototype.inside = function (point) {
    return Rect.inside(this, point);
  };
  Rect.CONTAIN = 16;
  Rect.COVER = 32;

  Rect.prototype.fit = function (other, align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN) {
    let factors = Size.prototype.fitFactors.call(this, new Size(other)).sort((a, b) => a - b);
    //Util.log('factors:', factors);

    let rects = factors.reduce((acc, factor) => {
      let rect = new Rect(0, 0, this.width, this.height);
      rect.mul(factor);
      rect.align(other, align & 0x0f);

      acc.push(rect);
      return acc;
    }, []);

    //Util.log("rects:", rects);

    return rects;
  };

  Rect.prototype.pointFromCenter = function (point) {
    Point.prototype.sub.call(point, this.center);
    point.x /= this.width;
    point.y /= this.height;
    return point;
  };
  Rect.prototype.toCSS = function () {
    return {
      ...Point.prototype.toCSS.call(this),
      ...Size.prototype.toCSS.call(this)
    };
  };

  Rect.prototype.toTRBL = function () {
    return {
      top: this.y,
      right: this.x + this.width,
      bottom: this.y + this.height,
      left: this.x
    };
  };
  Rect.prototype.toArray = function () {
    const { x, y, width, height } = this;
    return [x, y, width, height];
  };
  Rect.prototype.toPoints = function (ctor = (points) => Array.from(points)) {
    const { x, y, width, height } = this;
    return ctor([new Point(x, y), new Point(x + width, y), new Point(x + width, y + height), new Point(x, y + height)]);
  };
  Rect.prototype.toLines = function (ctor = (lines) => Array.from(lines, (points) => new Line(...points))) {
    let [a, b, c, d] = Rect.prototype.toPoints.call(this);
    return ctor([
      [a, b],
      [b, c],
      [c, d],
      [d, a]
    ]);
  };
  Rect.prototype.align = function (align_to, a = 0) {
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
      default: this.x = align_to.x + xdiff / 2;
        break;
    }
    switch (Align.vertical(a)) {
      case Align.TOP:
        this.y = align_to.y;
        break;
      case Align.BOTTOM:
        this.y = align_to.y + ydiff;
        break;
      default: this.y = align_to.y + ydiff / 2;
        break;
    }

    /*  this.tx = this.x - oldx;
  this.ty = this.y - oldy;*/
    return this;
  };

  Rect.prototype.round = function (precision = 0.001, digits, type = 'round') {
    let { x1, y1, x2, y2 } = this.toObject(true);
    let a = new Point(x1, y1).round(precision, digits, type);
    let b = new Point(x2, y2).round(precision, null, type);
    this.x = a.x;
    this.y = a.y;
    this.width = +(b.x - this.x).toFixed(digits);
    this.height = +(b.y - this.y).toFixed(digits);
    return this;
  };
  Rect.prototype.toObject = function (bb = false) {
    if(bb) {
      const { x1, y1, x2, y2 } = this;
      return { x1, y1, x2, y2 };
    }
    const { x, y, width, height } = this;
    return { x, y, width, height };
  };

  Rect.round = (rect) => Rect.prototype.round.call(rect);
  Rect.align = (rect, align_to, a = 0) => Rect.prototype.align.call(rect, align_to, a);
  Rect.toCSS = (rect) => Rect.prototype.toCSS.call(rect);
  Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
  Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);

  Rect.center = (rect) => new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);
  Rect.bind = (rect) => {
    let obj = new Rect();
  };

  Rect.inside = (rect, point) => point.x >= rect.x && point.x <= rect.x + rect.width && point.y >= rect.y && point.y <= rect.y + rect.height;
  Rect.from = function (obj) {
    //const { x1,y1,x2,y2 } = obj;
    const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];

    const h = fn(obj.x1, obj.x2);
    const v = fn(obj.y1, obj.y2);

    const [x1, x2, y1, y2] = [...h, ...v];

    return new Rect({ x1, y1, x2, y2 }); //h[0], v[0], h[1] - h[0], v[1] - v[0]);
  };

  Rect.fromCircle = function (...args) {
    const { x, y } = Point(args);
    const radius = args.shift();

    return new Rect(x - radius, y - radius, radius * 2, radius * 2);
  };

  for(let name of [
    'clone',
    'corners',
    'isSquare',
    'getArea',
    'toString',
    //'toSource',
    'points',
    'toCSS',
    'toTRBL',
    'toPoints'
  ]) {
    Rect[name] = (rect, ...args) => Rect.prototype[name].call(rect || new Rect(rect), ...args);
  }

  Rect.toSource = (rect, opts = {}) => {
    const { sep = ', ', inner = false, spc = ' ', colon = ':' } = opts;
    let props = `x${colon}${spc}${rect.x}${sep}y${colon}${spc}${rect.y}${sep}width${colon}${spc}${rect.width}${sep}height${colon}${spc}${rect.height}`;
    if(inner) return props;
    return `{${sep}${props}${sep}}`;
  };

  Rect.bind = (o, p, gen) => {
    const [x, y, width, height] = p || ['x', 'y', 'width', 'height'];
    if(!gen) gen = (k) => (v) => (v === undefined ? o[k] : (o[k] = v));
    let pt = Point.bind(o, [x, y], gen);
    let sz = Size.bind(o, [width, height], gen);
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
    Rect.prototype[f] = function (...args) {
      Rect[f](this, ...args);
      return this;
    };
  }

  Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');

  const isRect = (rect) => isPoint(rect) && isSize(rect);

  g.Element = class Element extends Node {
    static wrap(e) {
      if(!this.methods) this.methods = Util.static({}, this, this, (k, fn) => k != 'wrap' && fn.length > 0);

      if(typeof e == 'string') e = Element.find(e);
      return Util.extend(e, this.methods);
    }

    static create() {
      let args = [...arguments];
      let { tagName, ns, children, ...props } = typeof args[0] == 'object' ? args.shift() : { tagName: args.shift(), ...args.shift() };
      let parent = args.shift();
      parent = typeof parent == 'string' ? Element.find(parent) : parent;

      //console.log('Element.create ', { tagName, props, parent });

      let d = document || window.document;
      let e = ns ? d.createElementNS(ns, tagName) : d.createElement(tagName);
      for(let k in props) {
        const value = props[k];
        if(k == 'parent') {
          parent = props[k];
          continue;
        } else if(k == 'className') k = 'class';
        if(k == 'style' && typeof value === 'object') Element.setCSS(e, value);
        else if(k.startsWith('on') || k.startsWith('inner')) e[k] = value;
        else e.setAttribute(k, value);
      }
      if(children && children.length) children.forEach((obj) => Element.create(obj, e));

      if(parent && parent.appendChild) parent.appendChild(e);
      return e;
    }

    static walkUp(elem, pred = (e) => true) {
      if(typeof elem == 'string') elem = Element.find(elem);
      let depth = 0;
      if(typeof pred == 'number') {
        let n = pred;
        pred = (e, d) => d == n;
      }
      let ret = [];
      while(elem) {
        if(pred(elem, depth)) ret.push(elem);
        elem = elem.parentElement;
        depth++;
      }
      return ret.length ? ret : null;
    }

    static *skip(elem, fn = (e, next) => next(e.parentElement)) {
      elem = typeof elem == 'string' ? Element.find(elem) : elem;
      //let [iter,push] = new iterator();
      let emit = (n) => (elem = n);

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

    static *childIterator(elem) {
      if(elem.firstElementChild) {
        for(let c = elem.firstElementChild; c; c = c.nextElementSibling) yield c;
      } else {
        let children = [...elem.children];
        for(let i = 0; i < children.length; i++) yield children[i];
      }
    }

    static toObject(elem, opts = { children: true }) {
      elem = Element.find(elem);
      let children = [];
      if(opts.children) {
        [...this.childIterator(elem)].forEach((c) => (Util.isObject(c) && 'tagName' in c ? children.push(Element.toObject(c, elem)) : (c.textContent + '').trim() != '' ? children.push(c.textContent) : undefined));
      }
      let attributes = (opts ? opts.namespaceURI : document.body.namespaceURI) != elem.namespaceURI ? { ns: elem.namespaceURI } : {};
      let a = 'length' in elem.attributes ? Element.attr(elem) : elem.attributes;
      for(let key in a) attributes[key] = '' + a[key];
      return {
        tagName: /[a-z]/.test(elem.tagName) ? elem.tagName : elem.tagName.toLowerCase(),
        ...attributes,
        ...(children.length > 0 ? { children } : {})
      };
    }

    static toCommand(elem, opts = {}) {
      let { parent = '', varName, recursive = true, cmd = 'Element.create', quote = "'" } = opts;
      let o = Element.toObject(elem, { children: false });
      let s = '';
      let { tagName, ns, children, ...attributes } = o;
      let v = '';
      s = Object.keys(ns ? { ns, ...attributes } : attributes)
        .map((k) => `${k}:${quote}${attributes[k]}${quote}`)
        .join(', ');
      s = `${cmd}('${tagName}', {${s}}`;
      let c = elem.children;
      if(c.length >= 1) s = `${s}, [\n  ${c.map((e) => Element.toCommand(e, opts).replace(/\n/g, '\n  ')).join(',\n  ')}\n]`;
      s = `${s}${parent ? `, ${parent}` : ''})`;
      if(elem.firstElementChild && varName) {
        v = parent ? String.fromCharCode(parent.charCodeAt(0) + 1) : varName;
        s = `${v} = ${s}`;
      }
      return s.replace(/;*$/g, '');
    }

    static find(arg, parent, globalObj = Util.getGlobalObject()) {
      if(typeof parent == 'string') parent = Element.find(parent);
      if(!parent && globalObj.document) parent = globalObj.document;

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
    static rect(elem, options = {}) {
      let args = [...arguments];
      let element = args.shift();
      if(args.length > 0 && (isRect(args) || isRect(args[0]))) return Element.setRect.apply(Element, arguments);
      let { round = true, relative_to = null, relative = false, scroll_offset = true } = options;
      const e = typeof element === 'string' ? Element.find(element) : element;
      if(!e || !e.getBoundingClientRect) {
        return null; //new Rect(0, 0, 0, 0);
      }
      const bb = e.getBoundingClientRect();

      let r = TRBL.toRect(bb);
      if(relative) relative_to = e.parentElement;

      if(relative_to && relative_to !== null /*&& Element.isElement(relative_to)*/) {
        const off = Element.rect(relative_to);
        r.x -= off.x;
        r.y -= off.y;
      }

      //console.log("Element.rect(", r, ")");

      if(options.border) {
        const border = Element.border(e);
        Rect.outset(r, border);

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
        default: css.left = Math.round(trbl.left /* - ptrbl.left*/) + unit;
          remove = 'right';
          break;
        case Anchor.RIGHT:
          css.right = Math.round(trbl.right - ptrbl.right) + unit;
          remove = 'left';
          break;
      }
      switch (Anchor.vertical(anchor)) {
        case Anchor.TOP:
        default: css.top = Math.round(trbl.top /* - ptrbl.top*/) + unit;
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

    static position(element, pos = 'absolute') {
      if(typeof element == 'string') element = Element.find(element);
      const { x, y } = element.getBoundingClientRect();
      return new Point({ x, y });
    }

    static move(element, point, pos) {
      let [e, ...rest] = [...arguments];
      let { x = Element.position(element).x, y = Element.position(element).y } = new Point(rest);
      let to = { x, y };
      let position = rest.shift() || Element.getCSS(element, 'position') || 'relative';
      let off;
      //console.log('Element.move ', { element, to, position });
      const getValue = (prop) => {
        const property = Element.getCSS(element, prop);
        if(property === undefined) return undefined;
        const matches = /([-0-9.]+)(.*)/.exec(property) || [];
        //console.log({ match, value, unit });
        return parseFloat(matches[1]);
      };

      const current = new Point({
        x: getValue('left') || 0,
        y: getValue('top') || 0
      });
      off = new Point(Element.rect(element, { round: false }));
      //off = Point.diff(off, current);
      Point.add(current, Point.diff(to, off));

      /*
    if(position == 'relative') {
      to.x -= off.x;
      to.y -= off.y;
    }*/
      let css = Point.toCSS(current);
      //console.log("Element.move: ", { position, to, css, off, current });
      //console.log('move newpos: ', Point.toCSS(pt));
      Element.setCSS(element, { ...css, position });
      return element;
    }

    static moveRelative(element, to, position) {
      let e = typeof element == 'string' ? Element.find(element) : element;

      let pos = Object.freeze(new Rect(to || Element.rect(e)));
      function move(x, y) {
        let rect = new Rect(pos.x + x, pos.y + y, pos.width, pos.height);
        move.last = rect;
        return Element.move(e, rect, position);
      }
      move.pos = pos;
      move.cancel = () => move(0, 0);
      move.jump = () => Element.moveRelative(e);

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
      const names = ['Top', 'Right', 'Bottom', 'Left'].map((pos) => prefix + (prefix == '' ? pos.toLowerCase() : pos + (prefix == 'border' ? 'Width' : '')));
      return new TRBL(Element.getCSS(element, names));
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
            value.subscribe = (newval) => element.style.setProperty(propName, newval);
            value = value();
          }
        }
        if(element.style) {
          if(element.style.setProperty) element.style.setProperty(propName, value);
          else element.style[Util.camelize(propName)] = value;
        }
      }
      return element;
    }

    static getCSS(element, property = undefined, receiver = null) {
      element = typeof element == 'string' ? Element.find(element) : element;

      const w = window !== undefined ? window : globalThis.window;
      const d = document !== undefined ? document : globalThis.document;
      //console.log('Element.getCSS ', { w, d, element });

      let parent = Util.isObject(element) ? element.parentElement || element.parentNode : null;

      let estyle = Util.tryPredicate(() => (Util.isObject(w) && w.getComputedStyle ? w.getComputedStyle(element) : d.getComputedStyle(element)), null);
      let pstyle = Util.tryPredicate(() => (parent && parent.tagName ? (/*Util.toHash*/ w && w.getComputedStyle ? w.getComputedStyle(parent) : d.getComputedStyle(parent)) : {}), null);

      if(!estyle || !pstyle) return null;
      //let styles = [estyle,pstyle].map(s => Object.fromEntries([...Node.map(s)].slice(0,20)));

      let style = Util.tryPredicate(() => Util.removeEqual(estyle, pstyle), null);

      if(!style) return null;
      let keys = Object.keys(style).filter((k) => !/^__/.test(k));
      //console.log("style: ", style);
      //console.log("Element.getCSS ", style);

      let ret = {};
      if(receiver == null) {
        receiver = (result) => {
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
      for(let e of this.skip(elt, (e, next) => next(e.parentElement !== relative_to && e.parentElement))) path = '/' + Element.unique(e) + path;

      //console.log('relative_to: ', relative_to);
      /*    for(; elt && elt.nodeType == 1; elt = elt.parentNode) {
      const xname = Element.unique(elt);
      path = xname + path;
      if(elt == relative_to) {
        break;
      }
      path = '/' + path;
    }*/
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
        [...child.attributes].forEach((attr) => (accu += ' ' + attr.name + "='" + attr.value + "'"));
        if(Rect.area(rect) > 0) accu += ' ' + Rect.toString(rect);
        ['margin', 'border', 'padding'].forEach((name) => {
          let trbl = Element.getTRBL(elem, 'margin');
          if(!trbl.null()) accu += ' ' + name + ': ' + trbl + '';
        });
        return accu;
      }
      str = dumpElem(elem, '');
      str = Element.walk(elem.firstElementChild,
        (e, a, r, d) => {
          if(e && e.attributes) return dumpElem(e, a + '\n', r, d);
          return null;
        },
        str
      );
      return str;
    }

    static skipper(fn, pred = (a, b) => a.tagName == b.tagName) {
      return function (elem) {
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
      const { idx = false, use_id = true } = opts;
      let name = elem.tagName.toLowerCase();
      if(use_id && elem.id && elem.id.length) return name + '#' + elem.id;
      const classNames = [...elem.classList]; //String(elem.className).split(new RegExp("/[ \t]/"));
      for(let i = 0; i < classNames.length; i++) {
        let res = document.getElementsByClassName(classNames[i]);
        if(res && res.length === 1) return name + '.' + classNames[i];
      }
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
        delegate.append_to = function (elem, parent) {
          if(!parent) parent = root;
          if(parent) parent.appendChild(elem);
          if(!this.root) this.root = elem;
        };
      }
      if(!delegate.create) delegate.create = (tag) => document.createElement(tag);
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
        ? (e) => {
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
        let trun = function (e) {
          this.event = e;
          //console.log("Element.transitionRun event", this);
          callback(this);
        };
        let tend = function (e) {
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

    static toString(e) {
      let o = e.__proto__ === Object.prototype ? e : Element.toObject(e);
      const { tagName, ns, children = [], ...a } = o;
      let s = `<${tagName}`;
      s += Object.entries(a)
        .map(([name, value]) => ` ${name}="${value}"`)
        .join('');
      s += children.length ? `>` : ` />`;
      if(children.length) s += children.map(Element.toString).join('') + `</${tagName}`;
      return s;
    }

    static clipboardCopy = (text) =>
      new Promise((resolve, reject) => {
        if(navigator.clipboard) {
          return navigator.clipboard
            .writeText(text)
            .then(() => resolve(true))
            .catch((err) => reject(err !== undefined ? err : new DOMException('The request is not allowed', 'NotAllowedError')));
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
  };

  Element.children = function* (elem, tfn = (e) => e) {
    if(typeof elem == 'string') elem = Element.find(elem);
    for(let e = elem.firstElementChild; e; e = e.nextElementSibling) yield tfn(e);
  };

  Element.recurse = function* (elem, tfn = (e) => e) {
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
  };

  Element.EDGES = {
    upperLeft: 0,
    upperCenter: 0.5,
    upperRight: 1,
    centerRight: 1.5,
    lowerRight: 2,
    lowerCenter: 2.5,
    lowerLeft: 3,
    centerLeft: 3.5
  };

  Element.edges = (arg) => Element.getEdgesXYWH(Element.rect(arg));
  Element.Axis = { H: 0, V: 2 };

  Element.margin = (element) => Element.getTRBL(element, 'margin');
  Element.padding = (element) => Element.getTRBL(element, 'padding');
  Element.border = (element) => Element.getTRBL(element, 'border');

  function isElement(e) {
    return Util.isObject(e) && e.tagName !== undefined;
  }
})();
