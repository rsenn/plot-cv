// ==UserScript==

// @name         lib/geom.js
// @namespace    create-tamper
// @version      0.2
// @description  geom.js, align.js, bbox.js, util.js, graph.js, intersection.js, point.js, line.js, lineList.js, element.js, node.js, trbl.js, rect.js, size.js, iterator.js, pointList.js, matrix.js, circle.js, polygonFinder.js, polygon.js, sweepLine.js, transformation.js, vector.js, simplify.js
// @author       You
// @match        *://*/*
// @exclude      *://127.0.0.1*/*
// @updateURL    http://127.0.0.1:3000/tamper.js
// @grant        none
// @run-at       document-end
// ==/UserScript==

/* jshint esversion: 6 */
/* jshint ignore:start */

(function (globalObj) {
  /* --- concatenated 'lib/geom/align.js' --- */
  Align.CENTER = 0;
  Align.LEFT = 1;
  Align.RIGHT = 2;
  Align.MIDDLE = 0;
  Align.TOP = 4;
  Align.BOTTOM = 8;
  Align.horizontal = alignment => alignment & (Align.LEFT | Align.RIGHT);
  Align.vertical = alignment => alignment & (Align.TOP | Align.BOTTOM);
  const Anchor = Align;

  /* --- concatenated 'lib/util.js' --- */
  /**
   * Class for utility.
   *
   * @class      Util (name)
   */
  //if(g) Util.globalObject = g;

  Util.formatAnnotatedObject = function(subject, o) {
    const {
      indent = '  ',
      spacing = ' ',
      separator = ',',
      newline = '\n',
      maxlen = 30,
      depth = 1,
      level = 0
    } = o;
    const i = indent.repeat(o.level || 0);
    let nl = newline != '' ? newline + i : spacing;
    const opts = {
      ...o,
      newline: depth >= 0 ? newline : '',
      depth: depth - 1,
      level: level + 1
    };
    if(subject && subject.toSource !== undefined) return subject.toSource();
    if(subject instanceof Date)
      return `new Date('${new Date().toISOString()}')`;
    if(typeof subject == 'string') return `'${subject}'`;
    if(typeof subject == 'number') return subject;
    if(subject != null && subject.y2 !== undefined)
      return `rect[${spacing}${subject.x}${separator}${subject.y} | ${subject.x2}${separator}${subject.y2} (${subject.w}x${subject.h}) ]`;
    if(Util.isObject(subject) &&
      'map' in subject &&
      typeof subject.map == 'function'
    )
      return `[${nl}${subject
        .map(i => Util.formatAnnotatedObject(i, opts))
        .join(separator + nl)}]`;
    if(typeof subject === 'string' || subject instanceof String)
      return `'${subject}'`;
    let longest = '';
    let r = [];

    for(let k in subject) {
      const v = subject.k;
      if(k.length > longest.length) longest = k;
      let s = '';

      if(typeof v === 'symbol') {
        s = 'Symbol';
      } else if(typeof v === 'string' || v instanceof String) {
        s = `'${v}'`;
      } else if(typeof v === 'function') {
        s = (v + '').replaceAll('\n', '\n' + i);
        s = (Util.fnName(s) || 'function') + '()';
      } else if(typeof v === 'number' || typeof v === 'boolean') {
        s = `${v}`;
      } else if(v === null) {
        s = 'null';
      } else if(v && v.length !== undefined) {
        try {
          s =
            depth <= 0
              ? `Array(${v.length})`
              : `[ ${v
                  .map(item => Util.formatAnnotatedObject(item, opts))
                  .join(', ')} ]`;
        } catch(err) {
          s = `[${v}]`;
        }
      } else if(v && v.toSource !== undefined) {
        s = v.toSource();
      } else if(opts.depth >= 0) {
        s =
          s.length > maxlen
            ? `[Object ${Util.objName(v)}]`
            : Util.formatAnnotatedObject(v, opts);
      } else {
        let c = Util.className(v);
        let t = Util.ucfirst(typeof v);
        s = `[${t}${c !== t ? ' ' : ''}${c !== t ? c : ''}]`;
      }

      if(s == '') s = typeof v;
      r.push([k, s]);
    }

    let padding = x =>
      indent +
      (opts.newline != '' ? Util.pad(x, longest.length, spacing) : spacing);
    let j = separator + spacing;

    if(r.length > 6) {
      nl = opts.newline + i;
      j = separator + (opts.newline != '' ? nl : spacing);
    }

    let ret =
      '{' +
      opts.newline +
      r.map(arr => padding(arr[0]) + arr[0] + ':' + spacing + arr[1]).join(j) +
      opts.newline +
      i +
      '}';
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
      };

      //;

      return [
        function() {
          return Curried(...args);
        },
        function (a) {
          return Curried(...args, ...a);
        },
        function (a, b) {
          return Curried(...args, ...a, ...b);
        },
        function (a, b, c) {
          return r(...args, ...a, ...b, ...c);
        },
        function (a, b, c, d) {
          return Curried(...args, ...a, ...b, ...c, ...d);
        }
      ].n;

      return new Function(...a,
        ...`const { curried,thisObj,args} = this; return curried.apply(thisObj, args.concat([${a.join(
          ','
        )}]))`
      ).bind({ args, thisObj, curried });
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
      function (fn) {
        return function(a) {
          return fn(a);
        };
      },
      function (fn) {
        return function(a, b) {
          return fn(a, b);
        };
      },
      function (fn) {
        return function(a, b, c) {
          return fn(a, b, c);
        };
      },
      function (fn) {
        return function(a, b, c, d) {
          return fn(a, b, c, d);
        };
      },
      function (fn) {
        return function(a, b, c, d, e) {
          return fn(a, b, c, d, e);
          H;
        };
      }
    ];

    if(n && n <= 5) return arityFn.n(fn);
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

          if(!self.fn) self.fn = key => obj.key;
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
            if(typeof obj.set == 'function')
              self.fn = (key, value) => obj.set(key, value);
          }
        }

        if(!self.fn) self.fn = (key, value) => ((obj.key = value), obj);
        return self.fn(key, value);
      };

    if(target !== undefined) self.target = target;
    return self;
  };
  Util.remover = target =>
    typeof target == 'object' && target !== null
      ? typeof target.delete == 'function'
        ? key => target.delete(key)
        : key => delete target.key
      : null;
  Util.hasFn = target =>
    typeof target == 'object' && target !== null
      ? typeof target.has == 'function'
        ? key => target.has(key)
        : key => key in target
      : null;

  Util.adder = target => {
    let self;

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
          if(typeof target.add == 'function')
            self.fn = (obj, arg) => (obj.add(arg), undefined);
          else if(typeof target.push == 'function')
            self.fn = (obj, arg) => (obj.push(arg), undefined);
        }
      }

      let isNum = Util.isNumeric(a);

      //console.debug('ChooseFn', { a, o, f: self.fn });

      if(!self.fn) {
        if(typeof o == 'string')
          self.fn = (obj, arg) => (obj == '' ? '' : obj + ', ') + arg;
        else if(a)
          self.fn = (obj, arg) =>
            (obj || (isNum || typeof arg == 'number' ? 0 : '')) + isNum
              ? +arg
              : ',' + arg;
      }
    }
  };

  Util.updater = (target, get, set, fn) => {
    let value;
    get = get || Util.getter(target);
    set = set || Util.setter(target);
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
    set = set || Util.setter(target);
    let value;
    return key =>
      (value = has.call(target, key)
        ? get.call(target, key)
        : ((value = create(key, target)), set.call(target, key, value), value));
  };

  Util.memoize = (fn, storage = new Map()) => {
    let self;
    const getter =
      typeof storage.get == 'function'
        ? storage.get
        : typeof storage == 'function'
        ? storage
        : Util.getter(storage);
    const setter =
      typeof storage.set == 'function'
        ? storage.set
        : typeof storage == 'function'
        ? storage
        : Util.setter(storage);

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

  Util.once = function(fn, thisArg = this) {
    let ran = false;
    let ret;

    return function(...args) {
      if(!ran) {
        ret = fn.call(thisArg, ...args);
        ran = true;
      }

      return ret;
    };
  };

  Util.getGlobalObject = Util.memoize(arg => {
    const retfn =
      typeof arg == 'function'
        ? arg
        : typeof arg == 'string'
        ? g => g.arg
        : g => g;
    return Util.tryCatch(() => global,
      retfn,
      err =>
        Util.tryCatch(() => globalThis,
          retfn,
          err =>
            Util.tryCatch(() => window,
              retfn,
              err => console.log('Util.getGlobalObject:', err)
            )
        )
    );
  });

  Util.isDebug = Util.memoize(() => {
    if(process !== undefined && process.env.NODE_ENV === 'production')
      return false;
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
    let c = [
      (locationStr[Symbol.for('nodejs.util.inspect.custom')] ||
        locationStr.toString
      ).call(locationStr)
    ];
    c.push(' ');
    let filters = Util.log.filters;
    let results = filters.map(f => f.test(locationStr));
    if(filters.every(f => !f.test(locationStr))) return;
    console.log('log', { args, c });
    Util.putStack();

    args = args.reduce((a, p, i) => {
      if(Util.isObject(p) && p[Util.log.methodName])
        p = p[Util.log.methodName]();
      else if(Util.isObject(p) && p[Symbol.for('nodejs.util.inspect.custom')])
        p = p[Symbol.for('nodejs.util.inspect.custom')]();
      else if(typeof p != 'string') {
        if(Util.isObject(p) &&
          typeof p.toString == 'function' &&
          !Util.isNativeFunction(p.toString)
        )
          p = p.toString();
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
    this.filters = [...args].map(arg =>
      arg instanceof RegExp ? arg : new RegExp(arg)
    );
  };

  Util.log.getFilters = function() {
    return this.filters;
  };

  Util.msg = (strings, ...substitutions) => {
    let i,
      o = [];

    for(i = 0; i < Math.max(strings.length, substitutions.length); i++) {
      if(strings.i !== undefined) o.push(strings.i.trim());
      if(substitutions.i !== undefined) o.push(substitutions.i);
    }

    console.log(...o);
  };
  Util.logBase = Util.curry((base, n) => Math.log(n) / Math.log(base));

  Util.generalLog = function(n, x) {
    return Math.log(x) / Math.log(n);
  };

  Util.toSource = function(arg, opts = {}) {
    const { quote = "'", colors = false, multiline = false } = opts;
    const { c = Util.coloring(colors) } = opts;
    let o;
    const { print = (...args) => (o = c.concat(o, c.text(...args))) } = opts;

    if(Util.isArray(arg)) {
      print('[', 1, 36);

      for(let item of arg) {
        if(o.length > 0) print(', ');
        Util.toSource(item, { ...opts, c, print });
      }

      print(']', 1, 36);
    } else if(typeof arg == 'number' || arg === undefined || arg === null)
      print(arg, 1, 35);
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

        if(!m) print(prop, 1, 33);
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
      .map(arg =>
        typeof arg === 'object' ? JSON.toString(arg, removeCircular) : arg
      )
      .join(' ')
      .replaceAll('\n', '');
  };

  //console.log("STR: "+str);

  //console.log.call(console, str);

  //Util.log.apply(Util, args)

  Util.type = function({ type }) {
    return (type && String(type).split(new RegExp('[ ()]', 'g'))[1]) || '';
  };

  Util.functionName = function(fn) {
    const matches = /function\s*([^(]*)\(.*/g.exec(String(fn));
    if(matches && matches[1]) return matches[1];
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

    if(Util.isObject(proto) && 'constructor' in proto)
      return Util.fnName(proto.constructor);
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

  Util.bitGroups = function(num, bpp) {
    let m = Util.bitMask(bpp, 0);
    let n = Math.floor(64 / bpp);
    let r = [];

    for(let i = 0; i < n; i++) {
      r.push(num & m);
      num /= m + 1;
    }

    while(r.length > 0 && r[r.length - 1] == 0)
      /* && Util.mod(r.length *bpp, 8) > 0*/ r.pop();
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
    return Array.from(Object.assign({}, a, { length: 50 }), bit =>
      bit ? 1 : 0
    );
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

  Util.range = function(start, end) {
    if(start > end) {
      let ret = [];
      while(start >= end) ret.push(start--);
      return ret;
    }

    const r = Array.from({ length: end - start + 1 }, (v, k) => k + start);

    //console.log("Util.range ", r);

    return r;
  };

  Util.set = function(obj, prop, value) {
    const set =
      obj instanceof Map
        ? (prop, value) => obj.set(prop, value)
        : (prop, value) => (obj.prop = value);

    if(arguments.length == 1)
      return (prop, value) => {
        set(prop, value);
        return set;
      };

    if(arguments.length == 2) return value => set(prop, value);
    return set(prop, value);
  };
  Util.get = Util.curry((obj, prop) =>
    obj instanceof Map ? obj.get(prop) : obj.prop
  );

  Util.symbols = (() => {
    const {
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
    } = Symbol;
    return {
      inspect: Symbol.for('nodejs.util.inspect.custom'),
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
  Util.inspect = (obj, opts = {}) =>
    Util.inspect(obj, {
      toString: Util.symbols.inspect,
      colors: true,
      ...opts,
      multiline: true,
      newline: '\n'
    });

  /*
    const { indent = '  ', newline = '\n', depth = 2, spacing = ' ' } = typeof opts == 'object' ? opts : { indent: '', newline: '', depth: typeof opts == 'number' ? opts : 10, spacing: ' ' };
  
    return Util.formatAnnotatedObject(obj, { indent, newline, depth, spacing });
  };*/
  Util.bitArrayToNumbers = function(arr) {
    let numbers = [];

    for(let i = 0; i < arr.length; i++) {
      const number = i + 1;
      if(arr.i) numbers.push(number);
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
    let fnlist = pred.map(type =>
      Util.isConstructor(type) ? what instanceof type : this.is.type
    );

    //console.debug('fnlist:', fnlist);

    return fnlist.every(fn => fn(what));
  };

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

  Util.randomBits = function(r = ([1, 50], (n = 5))) {
    return Util.numbersToBits(Util.randomNumbers(r, n));
  };

  Util.padFn = function(len, char = (' ', (fn = (str, pad) => pad))) {
    return (s, n = len) => {
      let m = Util.stripAnsi(s).length;
      s = s ? s.toString() : '' + s;
      return fn(s, m < n ? char.repeat(n - m) : '');
    };
  };

  Util.pad = function(s, n, char = ' ') {
    return Util.padFn(n, char)(s);
  };

  Util.abbreviate = function(str, max = (40, (suffix = '...'))) {
    if(Util.isArray(str)) {
      return Array.prototype.slice
        .call(str, 0, Math.min(str.length, max))
        .concat([suffix]);
    }

    if(typeof str != 'string') return str;
    str = '' + str;

    if(str.length > max) {
      return str.substring(0, max - suffix.length) + suffix;
    }

    return str;
  };

  Util.trim = function(str, charset) {
    const r1 = RegExp(`^[${charset}]*`);
    const r2 = RegExp(`[${charset}]*$`);
    return str.replace(r1, '').replace(r2, '');
  };

  Util.trimRight = function(str, charset) {
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
      const [arg, overwrite = true] = args;
      let adecl = Object.getOwnPropertyDescriptors(arg);
      let odecl = {};

      for(let prop in adecl) {
        if(prop in obj) {
          if(!overwrite) continue;
          else delete obj.prop;
        }

        if(Object.getOwnPropertyDescriptor(obj, prop)) delete odecl.prop;
        else
          odecl.prop = {
            ...adecl.prop,
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
      const memoize = Util.memoize(methods.method);

      decls.method = {
        get() {
          return memoize.call(this);
        },
        enumerable: true,
        configurable: false
      };
    }

    return Object.defineProperties(obj, decls);
  };

  Util.copyWhole = (dst, ...args) => {
    let chain = [];
    for(let src of args)
      chain = chain.concat(Util.getPrototypeChain(src).reverse());

    //console.debug('chain:', ...chain);

    for(let obj of chain) Util.define(dst, obj);

    return dst;
  };

  Util.copyEntries = (obj, entries) => {
    for(let [k, v] of entries) obj.k = v;
    return obj;
  };

  Util.extend = (...args) => {
    let deep = false;
    if(typeof args[0] == 'boolean') deep = args.shift();
    let result = args[0];
    if(Util.isUnextendable(result))
      throw new Error('extendee must be an object');

    let extenders = args.slice(1);
    let len = extenders.length;

    for(let i = 0; i < len; i++) {
      let extender = extenders.i;

      for(let key in extender) {
        if(true || extender.hasOwnProperty(key)) {
          let value = extender.key;

          if(deep && Util.isCloneable(value)) {
            let base = Array.isArray(value) ? [] : {};
            result.key = Util.extend(true,
              result.hasOwnProperty(key) && !Util.isUnextendable(result.key)
                ? result.key
                : base,
              value
            );
          } else {
            result.key = value;
          }
        }
      }
    }

    return result;
  };
  Util.isCloneable = obj =>
    Array.isArray(obj) || {}.toString.call(obj) == '[object Object]';
  Util.isUnextendable = val =>
    !val || (typeof val != 'object' && typeof val != 'function');

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
      Util.tryPredicate(
        (key, depth) =>
          obj.key === undefined &&
          typeof functions.key == 'function' &&
          pred(key, depth, functions) && [key, value]
      )
    )) {
      const value = function(...args) {
        return fn.call(thisObj || obj, this, ...args);
      };

      try {
        obj.name = value;
      } catch(/*        Object.defineProperty(obj, name, { value, enumerable: false, configurable: false, writable: false });*/
        err
      ) {
        console.log('static:', err);
      }
    }

    return obj;
  };
  Util.defineGetter = (obj, key, fn, enumerable = false) =>
    obj.key === undefined &&
    Object.defineProperty(obj, key, {
      enumerable,
      configurable: true,
      get: fn
    });
  Util.defineGetterSetter = (obj, key, g, s, enumerable = false) =>
    obj.key === undefined &&
    Object.defineProperty(obj, key, { get: g, set: s, enumerable });

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

    Util.defineGetterSetter(arr,
      'tail',
      function() {
        return Util.tail(this);
      },
      function (value) {
        if(this.length == 0) this.push(value);
        else this[this.length - 1] = value;
      }
    );
  };

  /*Util.define(arr, 'inspect', function(opts = {}) {
      return Util.inspect(this, { depth: 100, ...opts });
    });*/
  Util.adapter = function(obj,
    getLength = (obj => obj.length,
    (getKey =
      ((obj, index) => obj.key(index),
      (getItem =
        ((obj, key) => obj.key,
        (setItem = (obj, index, value) => (obj.index = value)))))))
  ) {
    const adapter = obj && {
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

  Util.adapter.localStorage = function(s) {
    s = Util.tryCatch(() => !s && global.window,
      w => w.localStorage,
      () => s
    );
    return Util.adapter(s,
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
    Array.from(entries.map(([k, v]) => k),
      key => entries.find(([k, v]) => k === key)[1]
    );

  Util.toMap = function(hash = ({}, fn)) {
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
          yield entry.name !== undefined && entry.value !== undefined
            ? [entry.name, entry.value]
            : entry[0] !== undefined && entry[1] !== undefined
            ? entry
            : [entry, map.entry];
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
          ret.k = v;
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

  Util.keyOf = function(obj, prop) {
    const keys = Object.keys(obj);

    for(let k in keys) {
      if(obj.k === prop) return k;
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
      ret = [...ret, ...arr.i];
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
          if(typeof arg == 'function')
            args.unshift(arg(...args.splice(0, arg.length)));
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
  Util.mod = (a, b) =>
    typeof b == 'number' ? ((a % b) + b) % b : n => ((n % a) + a) % a;
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
          if(parent.key === f) return key;
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

  Util.findKey = function(obj, value) {
    let pred = typeof value == 'function' ? value : v => v === value;
    for(let k in obj) if(pred(obj.k, k)) return k;
  };

  Util.find = function(arr, value, prop = ('id', (acc = Util.array()))) {
    let pred;

    if(typeof value == 'function') pred = value;
    else if(prop && prop.length !== undefined) {
      pred = function(obj) {
        if(obj.prop == value) return true;
        return false;
      };
    } else pred = obj => obj.prop == value;

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

  Util.match = function(arg, pred) {
    let match = pred;

    if(pred instanceof RegExp) {
      const re = pred;
      match = (val, key) =>
        (val && val.tagName !== undefined && re.test(val.tagName)) ||
        (typeof key === 'string' && re.test(key)) ||
        (typeof val === 'string' && re.test(val));
    }

    if(Util.isArray(arg)) {
      if(!(arg instanceof Array)) arg = [...arg];

      return arg.reduce((acc, val, key) => {
        if(match(val, key, arg)) acc.push(val);
        return acc;
      }, []);
    } else if(Util.isMap(arg)) {
      //console.log('Util.match ', { arg });

      return [...arg.keys()].reduce((acc, key) =>
          match(arg.get(key), key, arg) ? acc.set(key, arg.get(key)) : acc,
        new Map()
      );
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
      if(obj.key === prop) return key;
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

  Util.greatestCommonDenominator = (a, b) =>
    b ? Util.greatestCommonDenominator(b, a % b) : a;

  Util.leastCommonMultiple = (n1, n2) => {
    //Find the gcd first

    let gcd = Util.greatestCommonDenominator(n1, n2);

    //then calculate the lcm

    return (n1 * n2) / gcd;
  };
  const inspectSymbol = Symbol.for('nodejs.util.inspect.custom');

  Util.matchAll = Util.curry(function* (re, str) {
    let match;
    re =
      re instanceof RegExp
        ? re
        : new RegExp(Util.isArray(re) ? '(' + re.join('|') + ')' : re, 'g');

    do {
      if((match = re.exec(str))) yield match;
    } while(match != null);
  });

  Util.inspect = (obj, opts = {}) => {
    const {
      quote = '"',
      multiline = true,
      toString = Symbol.toStringTag || 'toString',
      /*Util.symbols.toStringTag*/ stringFn = str => str,
      indent = '',
      colors = false,
      stringColor = [1, 36],
      spacing = '',
      newline = '\n',
      padding = ' ',
      separator = ',',
      colon = ': ',
      depth = 10
    } = { ...Util.inspect.defaultOpts, ...opts };

    /* if(depth < 0) {
        if(Util.isArray(obj)) return `[...${obj.length}...]`;
        if(Util.isObject(obj)) return `{ ..${Object.keys(obj).length}.. }`;
        return '' + obj;
      }*/
    let out;

    const { c = Util.coloring(colors) } = opts;
    const {
      print = (...args) => (out = c.concat(out, c.text(...args)))
    } = opts;
    const sep =
      multiline && depth > 0
        ? (space = false) => newline + indent + (space ? '  ' : '')
        : (space = false) => (space ? spacing : '');

    if(typeof obj == 'number') {
      print(obj + '', 1, 36);
    } else if(typeof obj == 'undefined' || obj === null) {
      print(obj + '', 1, 35);
    } else if(typeof obj == 'function') {
      /*|| obj instanceof Function || Util.className(obj) == 'Function'*/ obj =
        '' + obj;

      //  if(!multiline)

      obj = obj.split(/\n/g)[0].replace(/{\s*$/, '{}');

      print(obj);
    } else if(typeof obj == 'string') {
      print(`'${stringFn(obj)}'`, 1, 36);
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
        Util.inspect(obj.i, {
          ...opts,
          c,
          print,
          newline: newline + '  ',
          depth: depth - 1
        });
      }

      print((padding || '') + `]`, 1, 36);
    } else if(Util.isObject(obj)) {
      const inspect = toString ? obj.toString : null;

      if(typeof inspect == 'function' &&
        !Util.isNativeFunction(inspect) &&
        !/Util.inspect/.test(inspect + '')
      ) {
        //   if(Util.className(obj) != 'Range') console.debug('inspect:', Util.className(obj), inspect + '');

        let s = inspect.call(obj, depth, { ...opts });

        //  if(Util.className(obj) != 'Range')

        console.debug('s:', s);

        console.debug('inspect:', inspect + '');
        out += s;
      } else {
        let isMap = obj instanceof Map;
        let keys = isMap ? obj.keys() : Object.keys(obj);

        //let entries = isMap ? [...obj.entries()] : Object.entries(obj);

        // print('[object ' + Util.className(obj) + ']');

        if(Object.getPrototypeOf(obj) !== Object.prototype)
          print(Util.className(obj) + ' ', 1, 31);

        isMap
          ? print(`(${obj.size}) {${sep(true)}`, 1, 36)
          : print('{' + sep(true), 1, 36);
        let i = 0;
        let getFn = isMap ? key => obj.get(key) : key => obj.key;
        let propSep = isMap ? [' => ', 0] : [': ', 1, 36];

        for(let key of keys) {
          const value = getFn(key);
          if(i > 0) print(sep(true), 36);
          if(typeof key == 'symbol') print(key.toString(), 1, 32);
          else if(Util.isObject(key) && typeof key.toString == 'function')
            print(isMap ? `'${key.toString()}'` : key.toString(),
              1,
              isMap ? 36 : 33
            );
          else if(typeof key == 'string' ||
            (!isMap && Util.isObject(key) && typeof key.toString == 'function')
          )
            print(isMap ? `'${key}'` : key, 1, isMap ? 36 : 33);
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
          else if(typeof value == 'string' || value instanceof String)
            print(`'${value}'`, 1, 36);
          else if(typeof value == 'object')
            Util.inspect(value, {
              ...opts,
              print,
              multiline: isMap && !(value instanceof Map) ? false : multiline,
              newline: newline + '  ',
              depth: depth - 1
            });
          else print((value + '').replaceAll('\n', sep(true)));
          i++;
        }

        print(`${multiline ? newline : padding}}`, 1, 36);
      }
    }

    return out;
  };
  Util.inspect.defaultOpts = { spacing: ' ', padding: ' ' };

  Util.dump = function(name, props) {
    const args = [name];

    for(let key in props) {
      f;

      args.push(`
      \t${key}: `);

      args.push(props.key);
    }

    const w = Util.tryCatch(() => global.window,
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
    if(typeof str != 'string') str = String(str);
    return str.substring(0, 1).toUpperCase() + str.substring(1);
  };

  Util.lcfirst = function(str) {
    return str.substring(0, 1).toLowerCase() + str.substring(1);
  };

  Util.typeOf = function(v) {
    if(Util.isObject(v) && Object.getPrototypeOf(v) != Object.prototype)
      return `${Util.className(v)}`;
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

  Util.decamelize = function(str, separator = '-') {
    return /.[A-Z]/.test(str)
      ? str
          .replace(/([a-z\d])([A-Z])/g, `$1${separator}$2`)
          .replace(/([A-Z]+)([A-Z][a-z\d]+)/g, `$1${separator}$2`)
          .toLowerCase()
      : str;
  };

  Util.ifThenElse = function(pred = (value => !!value, (_then = (() => {}, (_else = () => {}))))
  ) {
    return function(value) {
      let result = pred(value);
      let ret = result ? _then(value) : _else(value);
      return ret;
    };
  };
  Util.if = (value, _then, _else, pred) =>
    Util.ifThenElse(pred || (v => !!v),
      _then || (() => value),
      _else || (() => value)
    )(value);
  Util.ifElse = (value, _else, pred) =>
    Util.ifThenElse(pred || (v => !!v),
      () => value,
      _else ? () => _else : () => value
    )(value);
  Util.ifThen = (value, _then, pred) =>
    Util.ifThenElse(pred || (v => !!v),
      _then ? () => _then : () => value,
      () => value
    )(value);

  Util.transform = Util.curry(function* (fn, arr) {
    for(let item of arr) yield fn(item);
  });

  Util.colorDump = (iterable, textFn) => {
    textFn =
      textFn || ((color, n) => ('   ' + (n + 1)).slice(-3) + ` ${color}`);
    let j = 0;
    const filters =
      'font-weight: bold; text-shadow: 0px 0px 1px rgba(0,0,0,0.8); filter: drop-shadow(30px 10px 4px #4444dd)';
    if(!Util.isArray(iterable)) iterable = [...iterable];

    for(let j = 0; j < iterable.length; j++) {
      const [i, color] = iterable.j.length == 2 ? iterable.j : [j, iterable.j];
      console.log(`  %c    %c ${color} %c ${textFn(color, i)}`,
        `background: ${color}; font-size: 18px; ${filters};`,
        `background: none; color: ${color}; min-width: 120px; ${filters}; `,
        `color: black; font-size: 12px;`
      );
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
                const v = arg.k;
                let a = map.k || [];
                if(typeof a.push == 'function') a.push(v);
                map.k = a;
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
          if(queue.length === 1) resolve();
        }

        //allow the generator to resume

        return this;
      },
      loop: generator(),
      async run() {
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
  Util.isNumeric = v =>
    /^[-+]?(0x|0b|0o|)[0-9]*\.?[0-9]+(|[Ee][-+]?[0-9]+)$/.test(v + '');
  Util.isUndefined = arg => arg === undefined;

  Util.isObject = (obj, ...protoOrPropNames) => {
    let isObj = arg =>
      ['object', 'function'].indexOf(typeof arg) != -1 && arg !== null;

    do {
      if(!isObj(obj)) return false;
      if(protoOrPropNames.length == 0) break;
      let p = protoOrPropNames.shift();

      if(Util.isFunction(p) || Util.isArrowFunction(p)) {
        obj = p(obj);
        continue;
      }

      if(!isObj(p)) {
        obj = obj.p;
        continue;
      }

      if(Object.getPrototypeOf(obj) !== p) return false;
    } while(true);

    let r = obj || false;
    if(!r)
      console.log('Util.isObject(',
        obj,
        ...protoOrPropNames,
        ...')',
        ...` = ${!!r}`
      );
    return r;
  };

  Util.isFunction = arg => {
    if(arg !== undefined)
      return (typeof arg == 'function' ||
        !!(arg && arg.constructor && arg.call && arg.apply)
      );
  };

  /*
    let fn = arg => Util.isFunction(arg);
    fn.inverse = arg => !Util.isFunction(arg);
    return fn;*/
  Util.not = fn =>
    function(...args) {
      return !fn(...args);
    };
  Util.isAsync = fn => typeof fn == 'function' && /async/.test(fn + '');
  /*|| fn() instanceof Promise*/ Util.isArrowFunction = fn =>
    (Util.isFunction(fn) && !('prototype' in fn)) ||
    /\ =>\ /.test(('' + fn).replace(/\n.*/g, ''));
  Util.isEmptyString = v => Util.isString(v) && (v == '' || v.length == 0);

  Util.isEmpty = function(v) {
    if(typeof v == 'object' &&
      !!v &&
      v.constructor == Object &&
      Object.keys(v).length == 0
    )
      return true;
    if(!v || v === null) return true;
    if(typeof v == 'object' && v.length !== undefined && v.length === 0)
      return true;
    return false;
  };
  Util.isNonEmpty = v => !Util.isEmpty(v);

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
    return (value.length > 7 &&
      new RegExp('^(?![d]+$)(?![a-zA-Z]+$)(?![!#$%^&*]+$)[da-zA-Z!#$ %^&*]'
      ).test(value) &&
      !/\s/.test(value)
    );
  };

  Util.clone = function(obj, proto) {
    if(Util.isArray(obj)) return obj.slice();
    else if(typeof obj == 'object')
      return Object.create(proto || obj.constructor.prototype || Object.getPrototypeOf(obj),
        Object.getOwnPropertyDescriptors(obj)
      );
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

        return object.key;
      }

      let value = Util.findVal(object.key, propName, maxDepth - 1);
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
        o.i = this.deepCloneObservable(data.i);
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
      e.pId === appId && arr.push(e);
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
      a.i.children = [];

      old.map((se, si) => {
        if(se.pId === a.i.id) {
          a.i.children = [...a.i.children, se];
          this.to3wei(a.i.children, old, id, pId);
        }
      });

      if(!a.i.children.length) {
        delete a.i.children;
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
    arr.i = arr.splice(j, 1, arr.i)[0];
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
    let arr = cookie.match(new RegExp(`(^| )${name}=([^;]*)(;|$)`));
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
      while(i < c.length && pred(c.i)) i++;
      let r = c.substring(start, i);
      return r;
    };

    do {
      let str = skip(char => char != '=' && char != ';');

      if(c.i == '=' && str != 'path') {
        i++;
        key = str;
        value = skip(char => char != ';');
      } else {
        i++;
        skip();
      }

      if(key != '') ret.key = value;
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
    });

  //console.log(`Setting cookie[${key}] = ${value}`);

  Util.clearCookies = function(c) {
    return Util.setCookies(Object.keys(Util.parseCookie(c)).reduce(
        (acc, name) =>
          Object.assign(acc, {
            [[name]]: `; max-age=0; expires=${new Date().toUTCString()}`
          }),
        {}
      )
    );
  };

  Util.deleteCookie = function(name) {
    const w = Util.tryCatch(() => global.window,
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

    return ((Number(s1.replace('.', '')) * Number(s2.replace('.', ''))) /
      Math.pow(10, m)
    );
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
    return numSplit[0]
      .replace(/\B(?=(\d{3})+(?!\d))/g, ',')
      .concat(`.${numSplit[1]}`);
  };

  Util.searchObject = function(object,
    matchCallback,
    currentPath,
    result,
    searched
  ) {
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

          if(property.indexOf('$') !== 0 &&
            typeof object.property !== 'function' &&
            !desc.get &&
            !desc.set
          ) {
            if(typeof object.property === 'object') {
              try {
                JSON.toString(object.property);
              } catch(err) {
                continue;
              }
            }

            //if (Object.prototype.hasOwnProperty.call(object, property)) {

            Util.searchObject(object.property,
              matchCallback,
              `${currentPath}.${property}`,
              result,
              searched
            );
          }
        }
      }
    } catch(//}

      e
    ) {}

    //console.log(object);

    //throw e;

    return result;
  };

  Util.getURL = Util.memoize((req = {}) =>
    Util.tryCatch(() => process.argv[1],
      () => 'file://' + Util.scriptDir(),
      () => {
        let proto =
          Util.tryCatch(() =>
            process.env.NODE_ENV === 'production' ? 'https' : null
          ) || 'http';
        let port =
          Util.tryCatch(() =>
            process.env.PORT
              ? parseInt(process.env.PORT)
              : process.env.NODE_ENV === 'production'
              ? 443
              : null
          ) || 3000;
        let host =
          Util.tryCatch(() => global.ip) ||
          Util.tryCatch(() => global.host) ||
          Util.tryCatch(() => window.location.host.replace(/:.*/g, '')) ||
          'localhost';
        if(req && req.headers && req.headers.host !== undefined)
          host = req.headers.host.replace(/:.*/, '');
        else
          Util.tryCatch(() => process.env.HOST !== undefined && (host = process.env.HOST)
          );
        if(req.url !== undefined) return req.url;
        const url = `${proto}://${host}:${port}`;
        return url;
      }
    )
  );

  Util.parseQuery = function(url = Util.getURL()) {
    let startIndex;
    let query = {};

    try {
      if((startIndex = url.indexOf('?')) != -1)
        url = url.substring(startIndex);
      const args = [...url.matchAll(/[?&]([^=&#]+)=?([^&#]*)/g)];

      if(args) {
        for(let i = 0; i < args.length; i++) {
          const k = args.i[1];
          query.k = decodeURIComponent(args.i[2]);
        }
      }

      return query;
    } catch(err) {
      return undefined;
    }
  };

  Util.encodeQuery = function(data) {
    const ret = [];
    for(let d in data)
      ret.push(`${encodeURIComponent(d)}=${encodeURIComponent(data.d)}`);
    return ret.join('&');
  };

  Util.parseURL = function(href = this.getURL()) {
    //console.debug('href:', href);

    const matches = new RegExp('^([^:]+://)?([^/:]*)(:[0-9]*)?(/?.*)?',
      'g'
    ).exec(href);

    const [all, proto, host, port, location = ''] = matches;

    //console.debug('matches:', matches);

    if(!matches) return null;

    const argstr =
      location.indexOf('?') != -1 ? location.replace(/^[^?]*\?/, '') : '';

    /* + "&test=1"*/
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

    return {
      protocol: proto,
      host,
      port: typeof port === 'string' ? parseInt(port.substring(1)) : 443,
      location: location.replace(/\?.*/, ''),
      query: params,
      href(override) {
        if(typeof override === 'object') Object.assign(this, override);
        const qstr = Util.encodeQuery(this.query);
        return ((this.protocol ? `${this.protocol}://` : '') +
          (this.host ? this.host : '') +
          (this.port ? `:${this.port}` : '') +
          `${this.location}` +
          (qstr != '' ? `?${qstr}` : '')
        );
      }
    };
  };

  Util.makeURL = function(...args) {
    let href = typeof args[0] == 'string' ? args.shift() : Util.getURL();
    let url = Util.parseURL(href);
    let obj = typeof args[0] == 'object' ? args.shift() : {};
    Object.assign(url, obj);
    return url.href();
  };

  /*
    let href = typeof args[0] === "string" ? args.shift() : this.getURL();
    let urlObj = null;
    urlObj = this.parseURL(href);
    return urlObj ? urlObj.href(args[0]) : null;*/
  Util.numberFromURL = function(url, fn) {
    const obj = typeof url === 'object' ? url : this.parseURL(url);
    const nr_match = RegExp('.*[^0-9]([0-9]+)$').exec(url.location);
    const nr_arg = nr_match ? nr_match[1] : undefined;
    const nr = nr_arg && parseInt(nr_arg);
    if(!isNaN(nr) && typeof fn === 'function') fn(nr);
    return nr;
  };
  Util.tryPromise = fn =>
    new Promise((resolve, reject) => Util.tryCatch(fn, resolve, reject));

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
  Util.tryCatch = (fn, resolve = a => a, reject = () => null, ...args) =>
    Util.tryFunction(fn, resolve, reject)(...args);

  Util.putError = err => {
    let s = Util.stack(err.stack);
    let e = Util.exception(err);
    (console.info || console.log)('Util.putError ', e);
    (console.error || console.log)('ERROR:\n' + err.message + '\nstack:\n' + s.toString()
    );
  };

  Util.putStack = (stack = new Error().stack) => {
    // (console.error || console.log)('STACK TRACE:', Util.className(stack), Util.className(stack[1]));

    stack = stack instanceof Util.stack ? stack : Util.stack(stack);

    (console.error || console.log)('STACK TRACE:', stack.toString());
  };

  Util.trap = (() => {
    Error.stackTraceLimit = 100;
    return fn =>
      /* prettier-ignore */ Util.tryFunction(fn, ((ret) => ret), Util.putError);
  })();
  Util.tryPredicate = (fn, defaultRet) =>
    Util.tryFunction(fn,
      ret => ret,
      () => defaultRet
    );

  Util.isBrowser = function() {
    let ret = false;
    Util.tryCatch(() => window,
      w => (Util.isObject(w) ? (ret = true) : undefined),
      () => {}
    );
    Util.tryCatch(() => document,
      w => (Util.isObject(w) ? (ret = true) : undefined),
      () => {}
    );
    return ret;
  };

  //return !!(global.window && global.window.document);

  Util.waitFor = msecs => {
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
  Util.timeout = async (msecs, promises, promiseClass = Promise) =>
    await promiseClass.race([Util.waitFor(msecs)].concat(
        Util.isArray(promises) ? promises : [promises]
      )
    );

  Util.isServer = function() {
    return !Util.isBrowser();
  };

  Util.isMobile = function() {
    return true;
  };
  Util.uniquePred = (cmp = null) =>
    cmp === null
      ? (el, i, arr) => arr.indexOf(el) === i
      : (el, i, arr) => arr.findIndex(item => cmp(el, item)) === i;
  Util.unique = (arr, cmp) => arr.filter(Util.uniquePred(cmp));

  Util.histogram = /* new Set()*/ (arr,
    t,
    out = false ? {} : new Map(),
    initVal = () => 0,
    setVal = v => v
  ) => {
    const set = /*Util.isObject(out) && typeof out.set == 'function' ? (k, v) => out.set(k, v) :*/ Util.setter(out
    );
    const get = Util.getOrCreate(out, initVal, set);
    let ctor =
      Object.getPrototypeOf(out) !== Object.prototype ? out.constructor : null;
    let tmp;
    const defKeyFunc = it => it;
    t = t || defKeyFunc;
    if(Util.isObject(arr) && typeof arr.entries == 'function')
      arr = arr.entries();
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
    }
  };

  /*   else  else {
        throw new Error("No such arg type:"+typeof(arg));
      }*/
  Util.distinct = function(arr) {
    return Array.prototype.filter.call(arr,
      (value, index, me) => me.indexOf(value) === index
    );
  };

  Util.rangeMinMax = function(arr, field) {
    const numbers = [...arr].map(obj => obj.field);
    return [Math.min(...numbers), Math.max(...numbers)];
  };

  Util.remap = (...args) => {
    const getR = () =>
      Util.isArray(args[0]) ? args.shift() : args.splice(0, 2);
    const _from = getR(),
      to = getR();
    const f = [to[1] - to[0], _from[1] - _from[0]];
    const factor = f[0] / f[1];
    const r = val => (val - _from[0]) * factor + to[0];
    return r;
  };

  Util.mergeLists = function(arr1, arr2, key = 'id') {
    let hash = {};
    for(let obj of arr1) hash[obj.key] = obj;
    for(let obj of arr2) hash[obj.key] = obj;
    return Object.values(hash);
  };

  /* let hash = arr1.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
    hash = arr2.reduce((acc, it) => Object.assign({ [it[key]]: it }, acc), {});
    let ret = [];
    for(let k in hash) {
      if(hash[k][key]) ret.push(hash[k]);
    }
    return ret;*/
  Util.throttle = function(fn, wait) {
    let time = Date.now();

    return function() {
      if(time + wait - Date.now() < 0) {
        fn();
        time = Date.now();
      }
    };
  };

  Util.foreach = function(o, fn) {
    for(let [k, v] of Util.entries(o)) {
      if(fn(v, k, o) === false) break;
    }
  };

  Util.all = function(obj, pred) {
    for(let k in obj) if(!pred(obj.k)) return false;
    return true;
  };

  Util.isGenerator = function(fn) {
    return ((typeof fn == 'function' && /^[^(]*\*/.test(fn.toString())) ||
      (['function', 'object'].indexOf(typeof fn) != -1 && fn.next !== undefined)
    );
  };
  Util.isIterator = obj => Util.isObject(obj) && typeof obj.next == 'function';

  Util.isIterable = obj => {
    try {
      for(let item of obj) return true;
    } catch(err) {}

    return false;
  };
  Util.isNativeFunction = x =>
    typeof x == 'function' && /\[(native\ code|[^\n]*)\]/.test(x + '');

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
    let fn = (k, v) => (ret.k = v);
    for(let [k, v] of Util.entries(a)) if(pred(v, k, a)) fn(k, v);
    return Object.setPrototypeOf(ret, Object.getPrototypeOf(a));
  };

  Util.reduce = (obj, fn, accu) => {
    if(Util.isGenerator(obj)) {
      let i = 0;
      for(let item of obj) accu = fn(accu, item, i++, obj);
      return accu;
    }

    for(let key in obj) accu = fn(accu, obj.key, key, obj);
    return accu;
  };

  Util.mapFunctional = fn =>
    function* (arg) {
      for(let item of arg) yield fn(item);
    };

  Util.map = (...args) => {
    const [obj, fn] = args;
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
    }

    //    ret = a => new ctor(a);

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
        let item = fn(key, obj.key, obj);
        if(item) ret[item[0]] = item[1];
      }
    }

    return ret;
  };

  //Object.setPrototypeOf(ret,Object.getPrototypeOf(obj));

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
      acc.k = v;
      return acc;
    }, {});
  };

  Util.isDate = function(d) {
    return (d instanceof Date ||
      (typeof d == 'string' &&
        /[0-9][0-9][0-9][0-9]-[0-9][0-9]-[0-9][0-9]T[0-9][0-9]:[0-9][0-9]:[0-9][0-9]/.test(d
        ))
    );
  };

  Util.parseDate = function(d) {
    if(Util.isDate(d)) {
      d = new Date(d);
    }

    return d;
  };

  //return /^[0-9]+$/.test(d) ? Util.fromUnixTime(d) : new Date(d);

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

  Util.formatTime = function(date = (new Date(), (format = 'HH:MM:SS'))) {
    let n;
    let out = '';
    if(typeof date == 'number') date = new Date(date);

    for(let i = 0; i < format.length; i += n) {
      n = 1;
      while(format.i == format[i + n]) n++;
      const fmt = format.substring(i, i + n);
      let num = fmt;
      if(fmt.startsWith('H')) num = `0${date.getHours()}`.substring(0, n);
      else if(fmt.startsWith('M'))
        num = `0${date.getMinutes()}`.substring(0, n);
      else if(fmt.startsWith('S'))
        num = `0${date.getSeconds()}`.substring(0, n);
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
    ret = `${('0' + hours).substring(0, 2)}:${('0' + minutes).substring(0,
      2
    )}:${('0' + seconds).substring(0, 2)}`;
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
    return Math.round(Util.randFloat(...range, ...rnd));
  };

  Util.randStr = (len, charset, rnd = Util.rng) => {
    let o = '';
    if(!charset)
      charset =
        '_0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz';

    while(--len >= 0) {
      o += charset[Math.round(rnd() * (charset.length - 1))];
    }

    return o;
  };

  Util.hex = function(num, numDigits = 0) {
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

  Util.roundTo = function(value, prec, digits, type = 'round') {
    if(!isFinite(value)) return value;
    const fn = Math.type;
    if(prec == 1) return fn(value);

    /*  const decimals = Math.log10(prec);
      const digits = Math.ceil(-decimals);
      console.log('digits:', digits);*/
    let ret = fn(value / prec) * prec;

    digits = digits || -Math.min(0, Math.floor(Math.log10(prec)));
    if(digits == 0) ret = Math.type(ret);
    else if(typeof digits == 'number' && digits >= 1)
      ret = +ret.toFixed(digits);
    return ret;
  };

  Util.base64 = (() => {
    const g = globalThis;
    return {
      encode: Util.tryFunction(utf8 => g.btoa(g.unescape(g.encodeURIComponent(utf8))),
        v => v,
        utf8 => Buffer.from(utf8).toString('base64')
      ),
      decode: Util.tryFunction(base64 => g.decodeURIComponent(g.escape(g.atob(base64))),
        v => v,
        string => Buffer.from(string, 'base64').toString('utf-8')
      )
    };
  })();

  Util.formatRecord = function(obj) {
    let ret = {};

    for(let key in obj) {
      let val = obj.key;
      if(val instanceof Array) val = val.map(item => Util.formatRecord(item));
      else if(/^-?[0-9]+$/.test(val)) val = parseInt(val);
      else if(/^-?[.0-9]+$/.test(val)) val = parseFloat(val);
      else if(val == 'true' || val == 'false') val = Boolean(val);
      ret.key = val;
    }

    return ret;
  };

  Util.isArray = function(obj) {
    return ((obj &&
        !Util.isGetter(obj, 'length') &&
        Util.isObject(obj) &&
        'length' in obj &&
        !(obj instanceof String) &&
        !(obj instanceof Function) &&
        typeof obj == 'function') ||
      obj instanceof Array
    );
  };

  Util.equals = function(a, b) {
    if(Util.isArray(a) && Util.isArray(b)) {
      return a.length == b.length && a.every((e, i) => b.i === e);
    } else if(Util.isObject(a) && Util.isObject(b)) {
      const size_a = Util.size(a);
      if(size_a != Util.size(b)) return false;
      for(let k in a) if(!Util.equals(a.k, b.k)) return false;
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
  Util.isBool = value => value === true || value === false;

  Util.size = function(obj) {
    if(Util.isObject(obj)) {
      if('length' in obj) return obj.length;
      return Object.keys(obj).length;
    }

    return undefined;
  };

  Util.isMap = function(obj) {
    return ((obj && obj.get !== undefined && obj.keys !== undefined) ||
      obj instanceof Map
    );
  };

  Util.effectiveDeviceWidth = function() {
    let deviceWidth =
      window.orientation == 0 ? window.screen.width : window.screen.height;

    //iOS returns available pixels, Android returns pixels / pixel ratio

    //http://www.quirksmode.org/blog/archives/2012/07/more_about_devi.html

    if(navigator.userAgent.indexOf('Android') >= 0 &&
      window.devicePixelRatio
    ) {
      deviceWidth = deviceWidth / window.devicePixelRatio;
    }

    return deviceWidth;
  };

  Util.getFormFields = function(initialState) {
    return Util.mergeObjects([
      initialState,
      [...document.forms].reduce((acc, { elements }) =>
          [...elements].reduce((acc2, { name, value }) =>
              name == '' || value == undefined || value == 'undefined'
                ? acc2
                : Object.assign(acc2, { [[name]]: value }),
            acc
          ),
        {}
      )
    ]);
  };

  Util.mergeObjects = function(objArr,
    predicate = (dst, src, key) => (src.key == '' ? undefined : src.key)
  ) {
    let args = objArr;
    let obj = {};

    for(let i = 0; i < args.length; i++) {
      for(let key in args.i) {
        const newVal = predicate(obj, args.i, key);
        if(newVal != undefined) obj.key = newVal;
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

  Util.lottoChances = function(numbers, draws) {
    const f = Util.factorial;
    return f(numbers) / (f(numbers - draws) * f(draws));
  };

  Util.increment = function(obj, key) {
    if(obj.key >= 1) obj.key == 0;
    obj.key++;
    return obj.key;
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

    for(let key in obj) if(pred(key, obj.key, obj)) ret.key = obj.key;

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
    const pred =
      typeof arr == 'function'
        ? (v, k, o) => arr(k, v, o)
        : arr instanceof RegExp
        ? (k, v) => arr.test(k)
        : /*|| arr.test(v)*/ key => arr.indexOf(key) != -1;
    return Util.filterOutMembers(obj, (v, k, o) => pred(k, v, o));
  };

  Util.getKeys = function(obj, arr) {
    let ret = {};
    for(let key of arr) ret.key = obj.key;
    return ret;
  };

  Util.numbersConvert = function(str) {
    return str
      .split('')
      .map((ch, i) =>
        new RegExp('[ :,./]').test(ch)
          ? ch
          : String.fromCharCode((str.charCodeAt(i) & 0x0f) + 0x30)
      )
      .join('');
  };

  Util.entries = function(arg) {
    if(Util.isArray(arg) || Util.isObject(arg)) {
      if(typeof arg.entries == 'function') return arg.entries();
      else if(Util.isIterable(arg))
        return (function* () {
          for(let key in arg) yield [key, arg.key];
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
              for(let key in arg) yield key;
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
              for(let key in arg) yield arg.key;
            };
    }

    if(ret) return ret.call(arg);
  };

  Util.removeEqual = function(a, b) {
    let c = {};

    for(let key of Util.keys(a)) {
      if(b.key === a.key) continue;

      //console.log(`removeEqual '${a[key]}' === '${b[key]}'`);

      c.key = a.key;
    }

    //console.log(`removeEqual`,c);

    return c;
  };

  Util.remove = function(arr, item) {
    let idx,
      count = 0;

    for(count = 0;
      (idx = arr.findIndex(other => other === item)) != -1;
      count++
    )
      arr.splice(idx, idx + 1);

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
    for(let key of rootPath) o = o.key;

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
    for(let key of p) o = o.key;
    return o;
  };
  Util.pushUnique = (arr, ...args) =>
    args.reduce((acc, item) =>
        arr.indexOf(item) == -1 ? (arr.push(item), acc + 1) : acc,
      0
    );

  Util.insertSorted = function(arr, item, cmp = (a, b) => b - a) {
    let i = 0,
      len = arr.length;

    while(i < len) {
      if(cmp(item, arr.i) >= 0) break;
      i++;
    }

    i < len ? arr.splice(i, 0, item) : arr.push(item);
    return i;
  };

  Util.inserter = (dest, next = (k, v) => {}) => {
    // if(typeof dest == 'function' && dest.map !== undefined) dest = dest.map;

    const insert =
      /*dest instanceof Map ||
        dest instanceof WeakMap ||*/ typeof dest.set ==
        'function' && dest.set.length >= 2
        ? (k, v) => dest.set(k, v)
        : Util.isArray(dest)
        ? (k, v) => dest.push([k, v])
        : (k, v) => (dest.k = v);

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

    if(typeof obj.entries == 'function' &&
      Util.isIterator((it = obj.entries()))
    ) {
      return it;
    } else if(Util.isArray(obj)) {
      return Array.prototype.entries.call(obj);
    } else if('length' in obj) {
      return (function* () {
        for(let key of Array.prototype[Symbol.iterator].call(obj))
          yield [key, obj.key];
      })();
    }
  };

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

    if(getSetFunction[Symbol.iterator])
      r.entries = getSetFunction[Symbol.iterator];
    else {
      let g = getSetFunction();
      if(Util.isIterable(g) || Util.isGenerator(g))
        r.entries = () => getSetFunction();
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

    fn = function(key, value) {
      if(value === undefined) return fn.get(key);
      return fn.set(key, value);
    };

    fn.map = (m => {
      while(Util.isFunction(m) && m.map !== undefined) m = m.map;
      return m;
    })(map);

    if(map instanceof Map ||
      (Util.isObject(map) &&
        typeof map.get == 'function' &&
        typeof map.set == 'function')
    ) {
      fn.set = (key, value) => (map.set(key, value), (k, v) => fn(k, v));
      fn.get = key => map.get(key);
    } else {
      fn.set = (key, value) => ((map.key = value), (k, v) => fn(k, v));
      fn.get = key => map.key;
    }

    fn.update = function(key, fn = (k, v) => v) {
      let oldValue = this.get(key);
      let newValue = fn(oldValue, key);

      if(oldValue != newValue) {
        if(newValue === undefined && typeof map.delete == 'function')
          map.delete(key);
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

      fn[Symbol.for('nodejs.util.inspect.custom')] = function() {
        return new Map(this.map(([key, value]) => [
            Util.isArray(key) ? key.join('.') : key,
            value
          ])
        );
      };
    }

    if(typeof fn.entries == 'function') {
      fn.filter = function(pred) {
        return Util.mapFunction(new Map(
            (function* () {
              let i = 0;
              for(let [key, value] of fn.entries())
                if(pred([key, value], i++)) yield [key, value];
            })()
          )
        );
      };

      fn.map = function(t) {
        return Util.mapFunction(new Map(
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
    if(typeof map.keys == 'function')
      fn.keys = () => [...map.keys()].map(fromKey);

    if(typeof map.entries == 'function')
      fn.entries = function* () {
        for(let [key, value] of map.entries()) yield [fromKey(key), value];
      };

    if(typeof map.values == 'function')
      fn.values = function* () {
        for(let value of map.values()) yield value;
      };

    if(typeof map.has == 'function') fn.has = key => map.has(toKey(key));
    if(typeof map.delete == 'function')
      fn.delete = key => map.delete(toKey(key));

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
    fn.get = forward.reduceRight(((a, m) => makeGetter(m, ((key) => a(key)))), ((a) => a));

    return fn;

    function makeGetter(map, next = a => a) {
      return key => (false && console.log('getter', { map, key }), next(map.get(key))
      );
    }
  };

  Util.predicate = fn_or_regex => {
    let fn;
    if(fn_or_regex instanceof RegExp)
      fn = (...args) => fn_or_regex.test(args + '');
    else fn = fn_or_regex;
    return fn;
  };

  Util.iterateMembers = function* (obj,
    predicate = ((name, depth, obj, proto) => true, (depth = 0))
  ) {
    let names = [];
    let pred = Util.predicate(predicate);
    const proto = Object.getPrototypeOf(obj);
    for(let name in obj) if(pred(name, depth, obj)) yield name;

    for(let name of Object.getOwnPropertyNames(obj)) {
      if(pred(name, depth, obj)) yield name;
    }

    for(let symbol of Object.getOwnPropertySymbols(obj))
      if(pred(symbol, depth, obj)) yield symbol;

    if(proto) yield* Util.iterateMembers(proto, predicate, depth + 1);
  };
  Util.and = (...predicates) => (...args) =>
    predicates.every(pred => pred(...args));
  Util.or = (...predicates) => (...args) =>
    predicates.some(pred => pred(...args));
  Util.members = Util.curry((pred, obj) =>
    Util.unique([...Util.iterateMembers(obj, Util.tryPredicate(pred))])
  );
  Util.memberNameFilter = (depth = (1, (start = 0))) =>
    Util.and((m, l, o) => start <= l && l < depth + start,
      (m, l, o) =>
        typeof m != 'string' ||
        ['caller', 'callee', 'constructor', 'arguments'].indexOf(m) == -1,
      (name, depth, obj, proto) => obj != Object.prototype
    );
  Util.getMemberNames = (obj, depth = Number.Infinity, start = 0) =>
    Util.members(Util.memberNameFilter(depth, start))(obj);
  Util.objectReducer = (filterFn,
    accFn = (a, m, o) => ({ ...a, [[m]]: o.m }),
    accu = {}
  ) => (obj, ...args) =>
    Util.members(filterFn(...args), obj).reduce(Util.tryFunction(
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

  Util.mapReducer = (setFn,
    filterFn = (key, value) => true,
    mapObj = new Map()
  ) => {
    setFn = setFn || Util.setter(mapObj);
    let fn;
    let next = Util.tryFunction(((acc, mem, idx) => (filterFn(mem, idx) ? (setFn(idx, mem), acc) : null),
      r => r,
      () => mapObj)
    );

    fn = function ReduceIntoMap(arg, acc = mapObj) {
      if(Util.isObject(arg) && typeof o.reduce == 'function')
        return arg.reduce((acc, arg) =>
            (Util.isArray(arg) ? arg : Util.members(arg)).reduce(reducer, acc),
          self.map
        );
      let c = Util.counter();
      for(let mem of arg) acc = next(acc, mem, c());
      return acc;
    };

    return Object.assign(fn, { setFn, filterFn, mapObj, next });
  };
  Util.getMembers = Util.objectReducer(Util.memberNameFilter);
  Util.getMemberDescriptors = Util.objectReducer(Util.memberNameFilter,
    (a, m, o) => ({
      ...a,
      [[m]]: Object.getOwnPropertyDescriptor(o, m)
    })
  );
  Util.methodNameFilter = (depth = (1, (start = 0))) =>
    Util.and((m, l, o) => typeof o.m == 'function',
      Util.memberNameFilter(depth, start)
    );
  Util.getMethodNames = (obj, depth = 1, start = 0) =>
    Util.members(Util.methodNameFilter(depth, start))(obj);
  Util.getMethods = Util.objectReducer(Util.methodNameFilter);
  Util.getMethodDescriptors = Util.objectReducer(Util.methodNameFilter,
    (a, m, o) => ({
      ...a,
      [[m]]: Object.getOwnPropertyDescriptor(o, m)
    })
  );

  Util.inherit = (dst, src, depth = 1) => {
    for(let k of Util.getMethodNames(src, depth)) dst.k = src.k;
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

  Util.bindMethods = function(methods, obj) {
    for(let name in methods) {
      methods.name = methods.name.bind(obj);
    }

    return methods;
  };

  Util.bindMethodsTo = function(dest, obj, methods) {
    for(let name in methods) {
      dest.name = methods.name.bind(obj);
    }

    return dest;
  };
  Util.getConstructor = obj =>
    obj.constructor || Object.getPrototypeOf(obj).constructor;

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
  Util.getConstructorChain = (ctor, fn = (c, p) => c) =>
    Util.getPrototypeChain(ctor, (p, o) => fn(o, p));

  Util.weakAssign = function(...args) {
    let obj = args.shift();

    args.forEach(other => {
      for(let key in other) {
        if(obj.key === undefined) obj.key = other.key;
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
      const { message, stack } = exc;
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

  Util.define(Util.exception.prototype, {
      toString(color = false) {
        const { message, stack, proto } = this;

        return `${Util.fnName((proto && proto.constructor) || this.constructor
        )}: ${message}
    Stack:${Util.stack.prototype.toString.call(stack,
      color,
      stack.columnWidths
    )}`;
      }, [Symbol.toStringTag]() {
        return this.toString(false);
      }, [Symbol.for('nodejs.util.inspect.custom')]() {
        return Util.exception.prototype.toString.call(this, true);
      }
    },
    true
  );

  Util.location = function Location(...args) {
    let ret =
      this instanceof Util.location
        ? this
        : Object.setPrototypeOf({}, Util.location.prototype);

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

  Util.location.palettes = [ [ [ 128, 128, 0 ], [ 255, 0, 255 ], [ 0, 255, 255 ] ], [ [ 9, 119, 18 ], [ 139, 0, 255 ], [ 0, 165, 255 ] ] ];

  Util.define(Util.location.prototype, {
    toString(color = false) {
      let { fileName, lineNumber, columnNumber, functionName } = this;
      fileName = fileName.replace(new RegExp(Util.getURL() + '/', 'g'), '');
      let text = /*color ? new this.colorCtor() : */ '';
      const c = /*color ? (t, color) => text.write(t, color) :*/ t =>
        (text += t);
      const palette = Util.location.palettes[Util.isBrowser() ? 1 : 0];
      if(functionName)
        c(functionName.replace(/\s*\[.*/g, '').replace(/^Function\./, '') + ' ',
          palette[1]
        );
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
      return this.fileName.replace(/:.*/g, '');
    },
    getLineNumber() {
      return this.lineNumber;
    },
    getColumnNumber() {
      return this.columnNumber;
    }
  });

  Util.stackFrame = function StackFrame(frame) {
    //console.debug('Util.stackFrame', frame);

    [
      'methodName',
      'functionName',
      'fileName',
      'lineNumber',
      'columnNumber',
      'typeName'
    ].forEach(prop => {
      let fn = 'get' + Util.ucfirst(prop);
      if(frame.prop === undefined && typeof frame.fn == 'function')
        frame.prop = frame.fn();
    });

    if(Util.colorCtor) frame.colorCtor = Util.colorCtor;
    return Object.setPrototypeOf(frame, Util.stackFrame.prototype);
  };
  Util.define(Util.stackFrame, {
    methodNames: [
      'getThis',
      'getTypeName',
      'getFunction',
      'getFunctionName',
      'getMethodName',
      'getFileName',
      'getLineNumber',
      'getColumnNumber',
      'getEvalOrigin',
      'isToplevel',
      'isEval',
      'isNative',
      'isConstructor',
      'isAsync',
      'isPromiseAll',
      'getPromiseIndex'
    ]
  });

  Util.memoizedProperties(Util.stackFrame, {
    propertyMap() {
      return this.methodNames.map(method => [
        method,
        Util.lcfirst(method.replace(/^[a-z]+/, ''))
      ]);
    }
  });

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
    },
    true
  );

  Util.define(Util.stackFrame.prototype, {
      colorCtor: null,
      get() {
        const { fileName, columnNumber, lineNumber } = this;
        return fileName ? `${fileName}:${lineNumber}:${columnNumber}` : null;
      },
      toString(color, opts = {}) {
        const { columnWidths = [0, 0, 0, 0], stripUrl } = opts;
        let text = color && this.colorCtor ? new this.colorCtor() : '';
        const c =
          color && this.colorCtor
            ? (t, color) => text.write(t, color)
            : t => (text += t);
        let fields = ['functionName', 'fileName', 'lineNumber', 'columnNumber'];
        const colors = [
          [0, 255, 0],
          [255, 255, 0],
          [0, 255, 255],
          [0, 255, 255]
        ];

        //const { functionName, fileName, columnNumber, lineNumber } = this;

        let columns = fields.map(fn => this.fn);

        columns = columns.map((f, i) =>
          (f + '')[i >= 2 ? 'padStart' : 'padEnd'](columnWidths.i || 0, ' ')
        );

        // columns = columns.map((fn, i) => c(fn, colors[i]));

        let [functionName, fileName, lineNumber, columnNumber] = columns;

        if(stripUrl) fileName = fileName.replace(/.*:\/\/[^\/]*\//, '');

        //console.log('stackFrame.toString', { color ,columnWidths, functionName, fileName, lineNumber, columnNumber});

        let colonList = [fileName, lineNumber, columnNumber]
          .map(p => ('' + p == 'undefined' ? undefined : p))
          .filter(p =>
              p !== undefined &&
              p != 'undefined' &&
              ['number', 'string'].indexOf(typeof p) != -1
          )
          .join(':');

        return `${functionName} ${colonList}` + c('', 0);
      },
      getLocation() {
        return new Util.location(this);
      },
      get location() {
        return this.getLocation();
      }, [Symbol.toStringTag]() {
        return this.toString(false);
      }, [Symbol.for('nodejs.util.inspect.custom')](...args) {
        return Util.stackFrame.prototype.toString.call(this,
          true,
          this.columnWidths
        );
      }
    },
    true
  );
  Util.scriptName = () =>
    Util.tryCatch(() => process.argv[1],
      script => script + '',
      () => Util.getURL()
    );

  Util.getFunctionName = () => {
    const frame = Util.getCallerStack(2)[0];
    return frame.getFunctionName() || frame.getMethodName();
  };
  Util.scriptDir = () =>
    Util.tryCatch(() => Util.scriptName(),
      script => (script + '').replace(new RegExp('\\/[^/]*$', 'g'), ''),
      () => Util.getURL()
    );

  Util.stack = function Stack(stack, offset) {
    //console.log('Util.stack (1)', stack);

    if(typeof stack == 'number')
      return Object.setPrototypeOf(new Array(stack), Util.stack.prototype);

    if(!stack) {
      const oldPrepareStackTrace = Error.prepareStackTrace;
      Error.prepareStackTrace = (_, stack) => stack;
      stack = new Error().stack;
      Error.prepareStackTrace = oldPrepareStackTrace;
      const { propertyMap } = Util.stackFrame;

      //console.debug('stack methods', propertyMap);

      stack = [...stack].map(frame =>
        propertyMap
          .filter(([m, p]) => frame.m() !== undefined)
          .reduce((acc, [method, property]) => ({
              ...acc,
              get [property]() {
                return frame.method();
              }
            }),
            {}
          )
      );
    } //console.debug('stack ctor:', [...stack]);

    //console.debug('stack frame[0]:', [...stack][0]);
    else if(!(typeof stack == 'string')) stack = stackToString(stack, 0);

    function stackToString(st, start = 0) {
      if(Util.isArray(st)) {
        st = [
          ...(function* () {
            for(let i = start; i < st.length; i++) yield st.i;
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
      stack = stack.split(/\n/g).slice(1);

      //console.log('Util.stack (2)', [...stack] /*.toString(true)*/);

      const re = new RegExp('.* at ([^ ][^ ]*) \\(([^)]*)\\)');

      stack = stack.map(frame =>
        typeof frame == 'string'
          ? frame
              .replace(/^\s*at\s+/, '')
              .split(/[()]+/g)
              .map(part => part.trim())
          : frame
      );
      stack = stack.map(frame =>
        Util.isArray(frame)
          ? (frame.length < 2 ? ['', ...frame] : frame).slice(0, 2)
          : frame
      );
      stack = stack.map(([func, file]) => [
        func,
        file
          .split(/:/g)
          .reverse()
          .map(n => (!isNaN(+n) ? +n : n))
      ]);
      stack = stack.map(([func, file]) => [
        func,
        file.length >= 3 ? file : ['', '', ...file]
      ]);
      stack = stack.map(([func, [columnNumber, lineNumber, ...file]]) => ({
        functionName: func.replace(/Function\.Util/, 'Util'),
        methodName: func.replace(/.*\./, ''),
        fileName: file.reverse().join(':'),
        lineNumber,
        columnNumber
      }));
      stack = stack.map(({
          methodName,
          functionName: func,
          fileName: file,
          columnNumber: column,
          lineNumber: line
        }) => ({
          functionName: func,
          methodName,
          fileName: file
            .replace(new RegExp(Util.getURL() + '/', 'g'), '')
            .replace(/:.*/g, ''),
          lineNumber: Util.ifThenElse(s => s != '',
            s => +s,
            () => undefined
          )(line + file.replace(/.*[^0-9]([0-9]*)$/g, '$1')),
          columnNumber: Util.ifThenElse(s => s != '',
            s => +s,
            () => undefined
          )(column)
        })
      );
    } else {
      stack = stack.map(frame => new Util.stackFrame(frame));
    }

    //Util.getCallers(1, Number.MAX_SAFE_INTEGER, () => true, stack);

    stack = stack.map(frame =>
      Object.setPrototypeOf(frame, Util.stackFrame.prototype)
    );

    if(offset > 0) stack = stack.slice(offset);
    stack = Object.setPrototypeOf(stack, Util.stack.prototype);

    //stack.forEach(frame => console.log("stack frame:",frame));

    return stack;
  };
  Util.stack.prototype = Object.assign(Util.stack.prototype,
    Util.getMethods(new Array(), 1, 1)
  );
  Object.defineProperty(Util.stack, Symbol.species, { get: () => Util.stack });

  Util.stack.prototype = Object.assign(Util.stack.prototype, {
    toString(color = false) {
      const { columnWidths } = this;
      let a = [...this].map(frame =>
        Util.stackFrame.prototype.toString.call(frame, color, { columnWidths })
      );
      let s = a.join('\n');
      return s + '\n';
    }, [Symbol.toStringTag]() {
      return Util.stack.prototype.toString.call(this);
    }, [Symbol.for('nodejs.util.inspect.custom')](...args) {
      const { columnWidths } = this;
      return ('\n' +
        this.map(f => f.toString(!Util.isBrowser(), { columnWidths })).join('\n'
        )
      );
    }
  });

  Object.defineProperties(Util.stack.prototype, {
    columnWidths: {
      get() {
        return this.reduce((a, f) =>
            ['functionName'].map((fn, i) => Math.max(a.i, (f.fn + '').length)),
          [0, 0, 0, 0]
        );
      }
    }
  });

  Util.getCallerStack = function(position = (2, (limit = (1000, stack)))) {
    Error.stackTraceLimit = position + limit;

    if(position >= Error.stackTraceLimit) {
      throw new TypeError(`getCallerFile(position) requires position be less then Error.stackTraceLimit but position was: '${position}' and Error.stackTraceLimit was: '${Error.stackTraceLimit}'`
      );
    }

    const oldPrepareStackTrace = Error.prepareStackTrace;
    Error.prepareStackTrace = (_, stack) => stack;
    stack = Util.stack(stack, position);
    return stack.slice(0, limit);
  };

  Util.getCallerFile = function(position = 2) {
    let stack = Util.getCallerStack();

    if(stack !== null && typeof stack === 'object') {
      const frame = stack.position;
      return frame
        ? `${frame.getFileName()}:${frame.getLineNumber()}`
        : undefined;
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
      return frame
        ? frame.getMethodName() || frame.getFunctionName()
        : undefined;
    }
  };

  Util.getCallerFunctionNames = function(position = 2) {
    let stack = Util.getCallerStack(position + 1);

    if(stack !== null && typeof stack === 'object') {
      let ret = [];

      for(let i = 0; stack.i; i++) {
        const frame = stack.i;
        const method = frame.getMethodName();
        ret.push(method ? frame.getFunction() + '.' + method : frame.getFunctionName()
        );
      }

      return ret;
    }
  };

  Util.getCaller = function(index = (1, stack)) {
    const methods = [
      'getThis',
      'getTypeName',
      'getFunction',
      'getFunctionName',
      'getMethodName',
      'getFileName',
      'getLineNumber',
      'getColumnNumber',
      'getEvalOrigin',
      'isToplevel',
      'isEval',
      'isNative',
      'isConstructor'
    ];
    stack = stack || Util.getCallerStack(2, 1 + index, stack);
    let thisIndex = stack.findIndex(f => f.functionName.endsWith('getCaller'));
    index += thisIndex + 1;
    const frame = stack.index;
    return frame;
  };

  Util.getCallers = function(index = (1, (num = (Number.MAX_SAFE_INTEGER, stack)))
  ) {
    const methods = [
      'getThis',
      'getTypeName',
      'getFunction',
      'getFunctionName',
      'getMethodName',
      'getFileName',
      'getLineNumber',
      'getColumnNumber',
      'getEvalOrigin',
      'isToplevel',
      'isEval',
      'isNative',
      'isConstructor'
    ];
    stack = stack || Util.getCallerStack(2, num + index, stack);
    let thisIndex = stack.findIndex(f =>
      ((f.functionName || f.methodName) + '').endsWith('getCaller')
    );
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
      if(Object.getPrototypeOf(frame) !== Util.stackFrame.prototype)
        frame = Util.stackFrame(frame);
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

  Util.hashString = function(string, bits = (32, (mask = 0xffffffff))) {
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
    if(typeof tree.children == 'object' &&
      tree.children !== null &&
      tree.children.length
    )
      for(let child of tree.children) Util.flatTree(child, addOutput);
    return ret;
  };

  Util.traverseTree = function(tree, fn, depth = (0, (parent = null))) {
    fn(tree, depth, parent);
    if(Util.isObject(tree.children) && tree.children.length > 0)
      for(let child of tree.children)
        Util.traverseTree(child, fn, depth + 1, tree);
  };

  Util.walkTree = function(node, pred, t, depth = (0, (parent = null))) {
    return (function* () {
      if(!pred) pred = i => true;

      if(!t)
        t = function(i) {
          i.depth = depth;
          return i;
        };

      if(pred(node, depth, parent)) {
        yield t(node);

        if(typeof node == 'object' &&
          node !== null &&
          typeof node.children == 'object' &&
          node.children.length
        ) {
          for(let child of [...node.children]) {
            yield* Util.walkTree(child, pred, t, depth + 1, node.parent_id);
          }
        }
      }
    })();
  };

  Util.isPromise = function(obj) {
    return ((Boolean(obj) && typeof obj.then === 'function') || obj instanceof Promise
    );
  };

  /* eslint-disable no-use-before-define */
  if(typeof setImmediate !== 'function')
    var setImmediate = fn => setTimeout(fn, 0);

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
      tooDark: (options.tooDark || 0.03) * 255 * 3, //How dark is too dark for a pixel

      tooLight: (options.tooLight || 0.97) * 255 * 3, //How light is too light for a pixel

      tooAlpha: (options.tooAlpha || 0.1) * 255
    };

    //How transparent is too transparent for a pixel

    const w = imageElement.width;

    let h = imageElement.height;

    //Setup canvas and draw image onto it

    const context = document.createElement('canvas').getContext('2d');

    context.drawImage(imageElement, 0, 0, w, h);

    //Extract the rgba data for the image from the canvas

    const subpixels = context.getImageData(0, 0, w, h).data;

    const pixels = { r: 0, g: 0, b: 0, a: 0 };
    let processedPixels = 0;
    const pixel = { r: 0, g: 0, b: 0, a: 0 };
    let luma = 0;

    //Having luma in the pixel object caused ~10% performance penalty for some reason

    //Loop through the rgba data

    for(let i = 0, l = w * h * 4; i < l; i += 4) {
      pixel.r = subpixels.i;
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

    let channels = { r: null, g: null, b: null, a: null };

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
      if((line.length ? line.length + 1 : 0) + tokens[0].length >
        max_linelen
      ) {
        lines.push(line);
        line = '';
      }

      if(line != '') line += ' ';
      line += tokens.shift();
    }

    if(line != '') lines.push(line);
    return lines;
  };

  /*Util.matchAll = Util.curry(function* (re, str) {
    re = new RegExp(re + '', 'g');
    let match;
    while((match = re.exec(str)) != null) yield match;
  });*/
  Util.decodeEscapes = function(text) {
    let matches = [...Util.matchAll(/([^\\]*)(\\u[0-9a-f]{4}|\\)/gi, text)];

    if(matches.length) {
      matches = matches
        .map(m => [...m].slice(1))
        .map(([s, t]) => s + String.fromCodePoint(parseInt(t.substring(2), 16))
        );
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
  Util.stripNonPrintable = text =>
    text.replace(/[^\x20-\x7f\x0a\x0d\x09]/g, '');

  Util.decodeHTMLEntities = function(text) {
    let entities = {
      amp: '&',
      apos: "'",
      ['#x27']: "'",
      ['#x2F']: '/',
      ['#39']: "'",
      ['#47']: '/',
      lt: '<',
      gt: '>',
      nbsp: ' ',
      quot: '"'
    };
    return text.replace(new RegExp('&([^;]+);', 'gm'),
      (match, entity) => entities.entity || match
    );
  };
  Util.encodeHTMLEntities = (str, charset = '\u00A0-\u9999<>&') =>
    str.replace(new RegExp(`[${charset}](?!#)`, 'gim'),
      i => '&#' + i.charCodeAt(0) + ';'
    );

  Util.stripAnsi = function(str) {
    return (str + '').replace(new RegExp('\x1b[[(?);]{0,2}(;?[0-9])*.', 'g'),
      ''
    );
  };

  Util.proxy = (obj = ({}, handler)) =>
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

  Util.propertyLookup = (obj = ({}, (handler = key => null))) =>
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
    const argsType =
      typeof args === 'object' && Util.isArray(args) ? 'array' : 'object';
    const errorText =
      argsType === 'array'
        ? "Error! You can't change elements of this array"
        : "Error! You can't change properties of this object";

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
            for(let n in p) ctor.prototype.n = p.n;
          }
    );

    let body = `class ${imName} extends ${name} {
      constructor(...args) {
        super(...args);
        if(new.target === ${imName})
          return Object.freeze(this);
      }
    };
    
    ${imName}.prototype.constructor = ${imName};
    
    return ${imName};`;

    for(let p of initialProto) p(orig);
    let ctor = new Function(name, body)(orig);

    //console.log('immutableClass', { initialProto, body }, orig);

    let species = ctor;

    /* prettier-ignore */ Util.defineGetterSetter(ctor, Symbol.species, (() => species), ((value) => {
      species = value;
    }));

    return ctor;
  };

  Util.partial = function partial(fn) /*, arg1, arg2 etc */ {
    let partialArgs = [].slice.call(arguments, 1);

    if(!partialArgs.length) {
      return fn;
    }

    return function() {
      let args = [].slice.call(arguments);
      let derivedArgs = [];

      for(let i = 0; i < partialArgs.length; i++) {
        let thisPartialArg = partialArgs.i;
        derivedArgs.i =
          thisPartialArg === undefined ? args.shift() : thisPartialArg;
      }

      return fn.apply(this, derivedArgs.concat(args));
    };
  };
  Util.clamp = Util.curry((min, max, value) =>
    Math.max(min, Math.min(max, value))
  );

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
          palette: [
            'rgb(0,0,0)',
            'rgb(80,0,0)',
            'rgb(0,80,0)',
            'rgb(80,80,0)',
            'rgb(0,0,80)',
            'rgb(80,0,80)',
            'rgb(0,80,80)',
            'rgb(80,80,80)',
            'rgb(0,0,0)',
            'rgb(160,0,0)',
            'rgb(0,160,0)',
            'rgb(160,160,0)',
            'rgb(0,0,160)',
            'rgb(160,0,160)',
            'rgb(0,160,160)',
            'rgb(160,160,160)'
          ],
          /*Util.range(0, 15).map(i =>
              `rgb(${Util.range(0, 2)
                .map(bitno => Util.getBit(i, bitno) * (i & 0x08 ? 160 : 80))
                .join(',')})`
          )*/ code(...args
          ) {
            let css = '';
            let bold = 0;

            for(let arg of args) {
              let c = (arg % 10) + bold;
              let rgb = this.palette.c;

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
              if(Util.isArray(arg) && typeof arg[0] == 'string')
                out[0] += arg.shift();
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

  Util.stripAnsi = str => {
    let o = '';

    for(let i = 0; i < str.length; i++) {
      if(str.i == '\x1b' && str[i + 1] == '[') {
        while(!/[A-Za-z]/.test(str.i)) i++;
        continue;
      }

      o += str.i;
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

      for(; p > 0 && str.p != ' '; p--) {}

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

    for(let i = 0; i < arr.length; i++)
      if(arr.i.length > width) arr.i = Util.wordWrap(arr.i, width, delimiter);

    return arr.join(delimiter);
  };

  Util.defineInspect = (proto, ...props) => {
    if(!Util.isBrowser()) {
      const c = Util.coloring();

      proto[Symbol.for('nodejs.util.inspect.custom')] = function() {
        const obj = this;

        return (c.text(Util.fnName(proto.constructor) + ' ', 1, 31) +
          Util.inspect(props.reduce((acc, key) => {
              acc.key = obj.key;
              return acc;
            }, {}),
            {
              multiline: false,
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
    if(props instanceof Array)
      props = Object.fromEntries(props.map(name => [name, name]));
    const [propMap, propNames] = Util.isArray(props)
      ? [props.reduce((acc, name) => ({ ...acc, [[name]]: name }), {}), props]
      : [props, Object.keys(props)];
    if(!gen)
      gen = p => v =>
        v === undefined ? target[propMap.p] : (target[propMap.p] = v);
    const propGetSet = propNames
      .map(k => [k, propMap.k])
      .reduce((a, [k, v]) => ({
          ...a,
          [[k]]: Util.isFunction(v)
            ? (...args) => v.call(target, k, ...args)
            : (gen && gen(k)) ||
              ((...args) => (args.length > 0 ? (target.k = args[0]) : target.k))
        }),
        {}
      );

    /*  console.log(`Util.bindProperties`, { proxy, target, props, gen });*/
    //console.log(`Util.bindProperties`, { propMap, propNames, propGetSet });

    Object.defineProperties(proxy,
      propNames.reduce(
        (a, k) => {
          const prop = props.k;
          const get_set = propGetSet.k;

          //typeof prop == 'function' ? prop : gen(prop);

          return {
            ...a,
            [[k]]: { get: get_set, set: get_set, enumerable: true }
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
    [true]: val => val === 'true' || val === true,
    [false]: val => val === 'false' || val === false
  });

  Util.assert = function Assert(act, message) {
    if(!act) throw new Error(format('ASSERTION_S', message));
  };
  Util.assignGlobal = () => Util.weakAssign(globalThis, Util);

  Util.weakMapper = function(createFn, map = new WeakMap()) {
    let self = function(obj, ...args) {
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
    self.get = k => map.get(k);
    self.map = map;
    return self;
  };

  Util.merge = function(...args) {
    return args.reduce((acc, arg) => ({ ...acc, ...arg }), {});
  };

  Util.transformer = (a, ...l) =>
    (l || []).reduce((c, f) =>
        function(...v) {
          return f.apply(this, [c.apply(this, v), ...v]);
        },
      a
    );

  Util.copyTextToClipboard = (i, t) => {
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
  Util.toPlainObject = (obj, t = (v, n) => v) =>
    [...Util.getMemberNames(obj)].reduce((acc, k) => ({ ...acc, [[k]]: t(obj.k, k) }),
      {}
    );

  Util.timer = msecs => {
    let ret,
      id,
      rej,
      createdTime,
      startTime,
      stopTime,
      endTime,
      res,
      delay,
      n,
      timer;
    createdTime = new Date();

    const remaining = () => {
      let r =
        startTime +
        msecs -
        (typeof stopTime == 'number' ? stopTime : new Date());
      return r >= 0 ? r : 0;
    };

    const finish = callback => {
      stopTime = new Date();
      if(stopTime.valueOf() > endTime.valueOf()) stopTime = endTime;
      if(typeof callback == 'function') callback(stopTime);
      res((n = remaining()));
    };

    const log = (method, ...args) =>
      console.log(`${Date.now() - createdTime.valueOf()} timer#${id}.${method}`,
        ...args.map(obj =>
          Util.toPlainObject(obj || {},
            v => v || (v instanceof Date ? `+${v.valueOf() - createdTime}` : v)
          )
        )
      );

    const timeout = (msecs, tmr = timer) => {
      let now = Date.now();
      if(!startTime) startTime = new Date(now);
      endTime = new Date(now + msecs);
      stopTime = undefined;

      id = setTimeout(() => {
        finish(typeof tmr.callback == 'function'
            ? (...args) => tmr.callback(...args)
            : () => {}
        );
        log(`finish`, tmr);
      }, msecs);

      log('start', tmr);
    };

    const add = (arr, ...items) => [...(arr ? arr : []), ...items];

    timer = {
      subscribers: [],
      get delay() {
        return delay;
      },
      get created() {
        return createdTime;
      },
      get start() {
        return startTime || new Date(endTime.valueOf() - delay);
      },
      get stop() {
        return stopTime instanceof Date ? stopTime : undefined;
      },
      get elapsed() {
        return delay + (stopTime || new Date()).valueOf() - endTime.valueOf();
      },
      get end() {
        return endTime;
      },
      get remain() {
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
    for(let name of Util.getMethodNames(obj, 1, 0))
      obj.name = Util.wrapGenerator(obj.name);
    return obj;
  };

  Util.decorateIterable = (proto, generators = false) => {
    const methods = {
      forEach(fn, thisArg) {
        for(let [i, item] of this.entries()) fn.call(thisArg, item, i, this);
      },
      *map(fn, thisArg) {
        for(let [i, item] of this.entries())
          yield fn.call(thisArg, item, i, this);
      },
      *filter(pred, thisArg) {
        for(let [i, item] of this.entries())
          if(pred.call(thisArg, item, i, this)) yield item;
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
        if(idx != -1)
          return typeof this.item == 'function' ? this.item(idx) : this.idx;
      },
      every(pred, thisArg) {
        for(let [i, item] of this.entries())
          if(!pred(item, i++, this)) return false;
        return true;
      },
      some(pred, thisArg) {
        for(let [i, item] of this.entries())
          if(pred(item, i, this)) return true;
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
        let gen = proto.name;
        proto.name = Util.wrapGenerator(gen);
      }
    }

    return proto;
  };
  Util.swap = (a, b) => [b, a];
  Util.swapArray = ([a, b]) => [b, a];

  Util.cacheAdapter = (st, defaultOpts = {}) => {
    if(typeof st == 'string')
      st = Util.tryCatch(() => window.caches,
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
        return (await (await st).keys()).index;
      },
      async keys(urls = (false, (t = a => a))) {
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
    let {
      cache = 'fetch',
      fetch = Util.getGlobalObject('fetch'),
      debug,
      print,
      ...opts
    } = allOpts;
    const storage = Util.cacheAdapter(cache);

    let self = async function CachedFetch(request, opts = {}) {
      let response;

      try {
        if(typeof request == 'string')
          request = new Request(request, { ...self.defaultOpts, ...opts });
        response = await storage.getItem(request, {
          ...self.defaultOpts,
          ...opts
        });

        if(response == undefined) {
          response = await /*self.*/ fetch(request, {
            ...self.defaultOpts,
            ...opts
          });
          if(response) storage.setItem(request, response.clone());
        } else {
          response.cached = true;
        }
      } catch(err) {
        throw new Error(`CachedFetch: ` + (request.url || request) + ' ' + err.message
        );
      }

      return response;
    };

    if(debug)
      self = Util.printReturnValue(self, {
        print: print ||
          ((returnValue, fn, ...args) =>
            console.debug(`cachedFetch[${cache}] (`,
              ...args,
              ...`) =`,
              ...returnValue
            ))
      });
    Util.define(self, { fetch, cache, storage, opts });
    return self;
  };

  Util.proxyObject = (root, handler) => {
    const ptr = path => path.reduce((a, i) => a.i, root);

    const nodes = Util.weakMapper((value, path) =>
        new Proxy(value, {
          get(target, key) {
            let prop = value.key;
            if(Util.isObject(prop) || Util.isArray(prop))
              return new node([...path, key]);
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
    return Util.tryCatch(() => new DOMParser(),
      parser => parser.parseFromString(xmlStr, 'application/xml')
    );
  };

  Util.weakAssoc = (fn = (value, ...args) => Object.assign(value, ...args)) => {
    let mapper = Util.tryCatch(() => new WeakMap(),
      map => Util.weakMapper((obj, ...args) => Util.merge(...args), map),
      () => (obj, ...args) => Util.define(obj, ...args)
    );

    return (obj, ...args) => {
      let value = mapper(obj, ...args);
      return fn(value, ...args);
    };
  };
  Util.getArgs = Util.memoize(() =>
    Util.tryCatch(() => process.argv,
      a => a.slice(2),
      () =>
        Util.tryCatch(() => scriptArgs,
          a => a.slice(1)
        )
    )
  );
  Util.getEnv = async varName =>
    Util.tryCatch(() => process.env,
      async e => e.varName,
      () =>
        Util.tryCatch(async () => await import('std').then(std => std.getenv(varName))
        )
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
      Error.stackTraceLimit = Infinity;

      exec = Util.tryFunction(exec, //async (...args) => { Error.stackTraceLimit=Infinity;  return await exec(...args); },

        a => a,
        err => {
          let { message, stack } = err;

          //console.debug('main stack:', [...err.stack].map((f) => f + ''));

          console.log('main stack:', err.stack);

          stack = Util.stack(err.stack);

          // console.log("main Stack:", Util.className(stack), stack.toString+'', Util.className(stack[0]), stack[0].toString)

          console.log('main Exception:',
            message,
            '\n' + stack.toString(true) + ''
          );
        }
      );
    }

    return exec;
  };
  Util.safeCall = async (fn, args = []) =>
    await Util.safeFunction(fn, true)(...args);
  Util.callMain = async (fn, trapExceptions) =>
    await Util.safeFunction(fn, trapExceptions)(...Util.getArgs());

  Util.printReturnValue = (fn, opts = {}) => {
    const {
      print = (returnValue, fn, ...args) => {
        let stack = Util.getCallerStack();
        (console.debug || console.log)('RETURN VAL:',
          Util.inspect(returnValue, { colors: false }),
          {
            fn,
            args,
            stack
          }
        );
      }
    } = opts;

    let self;

    self = (...args) => {
      fn = Util.tryFunction(fn, (returnValue, ...args) => {
        print.call(self, returnValue, fn, ...args);
        return returnValue;
      });

      return fn(...args);
    };

    Util.define(self, { fn, opts });
    return self;
  };

  Util.replaceAll = (needles, haystack) => {
    return Util.entries(needles)
      .map(([re, str]) => [
        typeof re == 'string' ? new RegExp(re, 'g') : re,
        str
      ])
      .reduce((acc, [match, replacement]) => acc.replace(match, replacement),
        haystack
      );
  };

  Util.escape = str => {
    let s = '';

    for(let i = 0; i < str.length; i++) {
      let code = str.codePointAt(i);
      if(code <= 128) s += str.i;
      else s += `\\u${('0000' + code.toString(16)).slice(-4)}`;
    }

    return s;
  };

  Util.consolePrinter = function ConsolePrinter(log = console.log) {
    let self;

    self = function(...args) {
      self.add(...args);
      self.print();
      self.clear();
    };

    delete self.length;
    Object.setPrototypeOf(self,
      Util.extend(
        Util.consolePrinter.prototype,
        Util.getMethods(Object.getPrototypeOf(self), 1, 0)
      )
    );
    self.splice(0, self.length, '');
    self.log = (...args) => log(...args);
    return self;
  };
  Object.assign(Util.consolePrinter.prototype,
    Util.getMethods(Array.prototype)
  );

  Util.consoleConcat = function(...args) {
    let self;

    self = function ConsoleConcat(...args) {
      return self.add(...args);
    };

    self.add = function(...args) {
      concat(this, args);
      return this;
    };

    self.call = function(thisObj, ...args) {
      return self(...args);
    };

    function concat(out, args) {
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
        } else if(Util.isArray(arg) &&
          typeof arg[0] == 'string' &&
          /%[cos]/.test(arg[0])
        ) {
          concat(out, arg);
        } else {
          out[0] += ' %o';
          out.push(arg);
        }
      }

      return out;
    }

    delete self.length;
    Object.setPrototypeOf(self,
      Util.extend(Util.consoleConcat.prototype, Object.getPrototypeOf(self))
    );
    self.push('');
    if(args.length) self(...args);
    return self;
  };

  Util.consoleConcat.prototype = Object.assign(Util.consoleConcat.prototype,
    Util.getMethods(Array.prototype, 1, 0),
    {
      [Symbol.for('nodejs.util.inspect.custom')]() {
        return [this, [...this]];
      }, [Symbol.iterator]() {
        return Array.prototype[Symbol.iterator].call(this);
      },
      clear() {
        return this.splice(0, this.length);
      },
      print(log = (...args) => console.info(...args)) {
        log(...this);
      }
    }
  );
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

        // console.debug('arg =', typeof arg, Util.className(arg), arg);

        if(i > 0) this[0] += ' ';

        if(typeof arg != 'string') {
          this[0] += '%o';
          this.push(arg);
        } else {
          this[0] += arg;

          if(/color:/.test(this[0])) {
            throw new Error(`this[0] is CSS: i=${i}
          this[0] = "${this[0]}"
          arg= ${typeof arg} "${(arg + '').replaceAll('\n', '\\n')}"`);
          }

          const matches = [...Util.matchAll(['%c', '%o'], arg)];
          console.debug('matches.length:',
            matches.length,
            ' args.length:',
            args.length
          );

          if(matches.length > 0) {
            const styles = args.splice(0, matches.length);
            this.splice(this.length, 0, ...styles);
          }
        }
      }
    }
  });

  /* --- concatenated 'lib/geom/graph.js' --- */
  Graph.Node = class {
    constructor(point, connections) {
      this.point = point;
    }

    equals(node) {
      return Point.equals(node.point, this.point);
    }
  };

  Graph.Connection = class {
    constructor(node1, node2) {
      this.node1 = node1;
      this.node2 = node2;
    }

    equals(connection) {
      return ((this.node1.equals(connection.node1) &&
          this.node2.equals(connection.node2)) ||
        (this.node2.equals(connection.node1) &&
          this.node1.equals(connection.node2))
      );
    }
  };

  /* --- concatenated 'lib/geom/point.js' --- */
  const SymSpecies = Util.tryCatch(() => Symbol,
    sym => sym.species
  );

  const CTOR = obj => {
    if(obj.SymSpecies) return obj.SymSpecies;
    let p = Object.getPrototypeOf(obj);
    if(p.SymSpecies) return p.SymSpecies;
    return p.constructor;
  };
  const getArgs = args => (console.debug('getArgs', ...args),
    typeof args[0] == 'number' ? [{ x: args[0], y: args[1] }] : args
  );

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
    return ((other.y - this.y) * (other.y - this.y) +
      (other.x - this.x) * (other.x - this.x)
    );
  };

  Point.prototype.distance = function(other = { x: 0, y: 0 }) {
    return Math.sqrt(Point.prototype.distanceSquared.call(this, other));
  };

  Point.prototype.equals = function(other) {
    let { x, y } = this;
    return +x == +other.x && +y == +other.y;
  };

  Point.prototype.round = function(precision = (0.001, (digits = (3, (type = 'round'))))
  ) {
    let { x, y } = this;
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
  */ Point.prototype.toString = function(opts = {}
  ) {
    const {
      precision = 0.001,
      unit = '',
      separator = ',',
      left = '',
      right = '',
      pad = 0
    } = opts;
    let x = Util.roundTo(this.x, precision);
    let y = Util.roundTo(this.y, precision);

    if(pad > 0) {
      x = x + '';
      y = y + '';
      if(y[0] != '-') y = ' ' + y;
      if(x[0] != '-') x = ' ' + x;
    }

    //console.debug("toString", {x,y}, {pad});

    return `${left}${(x + '').padStart(pad, ' ')}${unit}${separator}${(y + ''
    ).padEnd(pad, ' ')}${unit}${right}`;
  };

  Util.defineGetterSetter(Point.prototype,
    Symbol.toStringTag,
    function() {
      return `Point{ ${Point.prototype.toSource.call(this)}`;
    },
    () => {},
    false
  );

  Point.prototype.toSource = function(opts = {}) {
    const {
      asArray = false,
      plainObj = false,
      pad = a => a,
      /*a.padStart(4, ' ')*/ showNew = true
    } = opts;
    let x = pad(this.x + '');
    let y = pad(this.y + '');
    let c = t => t;
    if(typeof this != 'object' || this === null) return '';
    if(asArray) return `[${x},${y}]`;
    if(plainObj) return `{x:${x},y:${y}}`;
    return `${c(showNew ? 'new ' : '', 1, 31)}${c('Point', 1, 33)}${c('(',
      1,
      36
    )}${c(x, 1, 32)}${c(',', 1, 36)}${c(y, 1, 32)}${c(')', 1, 36)}`;
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

  Point.prototype.toCSS = function(precision = (0.001, (edges = ['left', 'top']))
  ) {
    return {
      [[edges[0]]]: Util.roundTo(this.x, precision) + 'px',
      [[edges[1]]]: Util.roundTo(this.y, precision) + 'px'
    };
  };

  Point.prototype.toFixed = function(digits) {
    return new Point(+this.x.toFixed(digits), +this.y.toFixed(digits));
  };

  Point.prototype.isNull = function() {
    return this.x == 0 && this.y == 0;
  };

  Point.prototype.inside = function(rect) {
    return (this.x >= rect.x &&
      this.x < rect.x + rect.width &&
      this.y >= rect.y &&
      this.y < rect.y + rect.height
    );
  };

  Point.prototype.transform = function(m) {
    if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
    if(Util.isObject(m) && typeof m.transform_point == 'function')
      return m.transform_point(this);
    const x = m[0] * this.x + m[1] * this.y + m[2];
    const y = m[3] * this.x + m[4] * this.y + m[5];
    this.x = x;
    this.y = y;
    return this;
  };

  Point.prototype.scaleTo = function(minmax) {
    return new Point({
      x: (this.x - minmax.x1) / (minmax.x2 - minmax.x1),
      y: (this.y - minmax.y1) / (minmax.y2 - minmax.y1)
    });
  };

  Point.prototype.normal = function() {
    let d = Point.prototype.distance.call(this);
    return new Point({ x: this.x / d, y: this.y / d });
  };
  Point.move = (point, x, y) => Point.prototype.move.call(point, x, y);
  Point.angle = (point, other, deg = false) =>
    Point.prototype.angle.call(point, other, deg);
  Point.inside = (point, rect) => Point.prototype.inside.call(point, rect);
  Point.sub = (point, other) => Point.prototype.sub.call(point, other);
  Point.prod = (a, b) => Point.prototype.prod.call(a, b);
  Point.quot = (a, b) => Point.prototype.quot.call(a, b);
  Point.equals = (a, b) => Point.prototype.equals.call(a, b);
  Point.round = (point, prec) => Point.prototype.round.call(point, prec);
  Point.fromAngle = (angle, f) =>
    Point.prototype.fromAngle.call(new Point(0, 0), angle, f);

  for(let name of [
    'clone',
    'comp',
    'neg',
    'sides',
    'dimension',
    'toString', //'toSource',

    'toCSS',
    'sub',
    'diff',
    'add',
    'sum',
    'distance'
  ]) {
    Point.name = (point, ...args) =>
      Point.prototype.name.call(point || new Point(point), ...args);
  }
  Point.toSource = (point, { space = ' ', padding = ' ', separator = ',' }) =>
    `{${padding}x:${space}${point.x}${separator}y:${space}${point.y}${padding}}`;
  const isPoint = o =>
    o &&
    ((o.x !== undefined && o.y !== undefined) ||
      ((o.left !== undefined || o.right !== undefined) &&
        (o.top !== undefined || o.bottom !== undefined)) ||
      o instanceof Point ||
      Object.getPrototypeOf(o).constructor === Point);
  Point.isPoint = isPoint;
  Util.defineInspect(Point.prototype, 'x', 'y');

  Point.bind = (...args) => {
    const keys = ['x', 'y'];
    const [o, p = keys] = args;
    const { x, y } =
      (Util.isArray(p) &&
        p.reduce((acc, name, i) => ({ ...acc, [[keys.i]]: name }), {})) ||
      p;

    //  console.debug('Point.bind', { o, x, y });

    return Object.setPrototypeOf(Util.bindProperties({}, o, { x, y }),
      Point.prototype
    );
  };

  Util.defineGetter(Point, Symbol.species, function() {
    return this;
  });
  const ImmutablePoint = Util.immutableClass(Point);
  Util.defineGetter(ImmutablePoint, Symbol.species, () => ImmutablePoint);

  /* --- concatenated 'lib/geom/line.js' --- */
  const isLine = obj =>
    (Util.isObject(obj) &&
      ['x1', 'y1', 'x2', 'y2'].every(prop => obj.prop !== undefined)) ||
    ['a', 'b'].every(prop => isPoint(obj.prop));

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
      y: (ma * mb * (other[0].x - this[0].x) +
          mb * this[0].y -
          ma * other[0].y) /
        (mb - ma)
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
      return { x: this.x2 - this.x1, y: this.y2 - this.y1 };
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
      if(intercept.axis) {
        let [c0, m] = intercept.axis;
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
      fns.x = new Function('y', `return ${x}`);
    }

    //y => x;

    return fns;
  };

  Line.prototype.angle = function() {
    return Point.prototype.angle.call(Line.prototype.getVector.call(this));
  };

  Line.prototype.getLength = function() {
    const { a, b } = this;
    const { x1, y1, x2, y2 } = this;

    //Util.log("a:",a, " b:",b);

    //Util.log('a:', a, ' b:', b);

    //Util.log('this:', this);

    return Point.prototype.distance.call(a, b);
  };

  Line.prototype.endpointDist = function(point) {
    return Math.min(point.distance(this.a), point.distance(this.b));
  };

  Line.prototype.matchEndpoints = function(arr) {
    const { a, b } = this;
    return [...arr.entries()].filter(([i, otherLine]) =>
        !Line.prototype.equals.call(this, otherLine) &&
        (Point.prototype.equals.call(a, otherLine.a) ||
          Point.prototype.equals.call(b, otherLine.b) ||
          Point.prototype.equals.call(b, otherLine.a) ||
          Point.prototype.equals.call(a, otherLine.b))
    );
  };

  Line.prototype.distanceToPointSquared = function(p) {
    const { a, b } = this;
    let l2 = Point.prototype.distanceSquared.call(a, b);
    if(l2 === 0) return Point.prototype.distanceSquared.call(p, a);
    let t = ((p.x - a.x) * (b.x - a.x) + (p.y - a.y) * (b.y - a.y)) / l2;
    t = Math.max(0, Math.min(1, t));
    return Point.prototype.distanceSquared.call(p,
      new Point(a.x + t * (b.x - a.x), a.y + t * (b.y - a.y))
    );
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
    return new Point(pos * (this.x2 - this.x1) + this.x1,
      pos * (this.y2 - this.y1) + this.y1
    );
  };

  Line.prototype.transform = function(m) {
    this.a = this.a.transform(m);
    this.b = this.b.transform(m);
    return this;
  };

  Line.prototype.bbox = function() {
    const { x1, y1, x2, y2 } = this;
    return new BBox(x1, y1, x2, y2);
  };

  Line.prototype.add = function(other) {
    const { x1, y1, x2, y2 } = Line(...arguments);
    this.x1 += x1;
    this.y1 += y1;
    this.x2 += x2;
    this.y2 += y2;
    return this;
  };

  Line.prototype.points = function() {
    const { a, b } = this;
    return [a, b];
  };

  Line.prototype.diff = function(other) {
    other = Line(...arguments);
    return new Line(Point.diff(this.a, other.a), Point.diff(this.b, other.b));
  };

  Line.prototype[Symbol.for('nodejs.util.inspect.custom')] = function(n,
    options = {}
  ) {
    const { x1, y1, x2, y2 } = this;
    return 'Line ' + Util.inspect({ x1, y1, x2, y2 }, options) + ' }';
  };

  Line.prototype.toString = function(opts = {}) {
    const { separator = ', ', brackets = s => `[ ${s} ]`, pad = 6 } = opts;
    const { x1, y1, x2, y2 } = this;
    return (brackets(
        Point.toString(this.a || Point(x1, y1), { ...opts, separator, pad })
      ) +
      separator +
      brackets(Point.toString(this.b || Point(x2, y2), { ...opts, separator, pad })
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
    return new ctor(x1, y1, x2, y2);
  };

  Line.prototype.round = function(precision = (0.001, digits)) {
    let { x1, y1, x2, y2 } = this;
    this.a.x = Util.roundTo(x1, precision, digits);
    this.a.y = Util.roundTo(y1, precision, digits);
    this.b.x = Util.roundTo(x2, precision, digits);
    this.b.y = Util.roundTo(y2, precision, digits);
    return this;
  };

  Line.prototype.some = function(pred) {
    return pred(this.a) || pred(this.b);
  };

  Line.prototype.every = function(pred) {
    return pred(this.a) && pred(this.b);
  };

  Line.prototype.includes = function(point) {
    return (Point.prototype.equals.call(this.a, point) ||
      Point.prototype.equals.call(this.b, point)
    );
  };

  Line.prototype.equals = function(other) {
    //Util.log('Line.equals', this, other);

    other = Line(other);

    if(Point.equals(this.a, other.a) && Point.equals(this.b, other.b))
      return 1;
    if(Point.equals(this.a, other.b) && Point.equals(this.b, other.a))
      return -1;
    return false;
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
    return new Line(this.b, this.a);
  };

  Line.prototype.toPoints = function(ctor = Array.of) {
    const { x1, y1, x2, y2 } = this;
    return ctor({ x: x1, y: y1 }, { x: x2, y: y2 });
  };

  Line.prototype[Symbol.iterator] = function* () {
    yield this.a;

    yield this.b;
  };

  for(let name of [
    'direction',
    'round',
    'slope',
    'angle',
    'bbox',
    'points',
    'inspect',
    'toString',
    'toObject',
    'toSource',
    'distanceToPointSquared',
    'distanceToPoint'
  ]) {
    Line.name = (line, ...args) =>
      Line.prototype.name.call(line || new Line(line), ...args);
  }
  Util.defineInspect(Line.prototype, 'x1', 'y1', 'x2', 'y2');
  Line.a = Util.memoize(line => Point.bind(line, ['x1', 'y1']), new WeakMap());
  Line.b = Util.memoize(line => Point.bind(line, ['x2', 'y2']), new WeakMap());

  Line.bind = (o, p, gen) => {
    const [x1, y1, x2, y2] = p || ['x1', 'y1', 'x2', 'y2'];
    if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
    let proxy = { a: Point.bind(o, [x1, y1]), b: Point.bind(o, [x2, y2]) };
    Util.bindProperties(proxy, o, { x1, y1, x2, y2 }, gen);
    return Object.setPrototypeOf(proxy, Line.prototype);
  };

  Util.defineGetter(Line, Symbol.species, function() {
    return this;
  });
  const ImmutableLine = Util.immutableClass(Line);
  Util.defineGetter(ImmutableLine, Symbol.species, () => ImmutableLine);

  /* --- concatenated 'lib/geom/size.js' --- */
  const getArgs = args =>
    /*console.debug('getArgs', ...args), */ typeof args[0] == 'number'
      ? [{ width: args[0], height: args[1] }]
      : args;
  Size.prototype.width = NaN;
  Size.prototype.height = NaN;
  Size.prototype.units = null;

  Size.prototype.convertUnits = function(w = 'window' in global ? window : null
  ) {
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
    units =
      typeof units == 'string'
        ? { width: units, height: units }
        : units || this.units || { width: 'px', height: 'px' };
    if(this.width !== undefined)
      ret.width = this.width + (units.width || 'px');
    if(this.height !== undefined)
      ret.height = this.height + (units.height || 'px');
    return ret;
  };

  Size.prototype.transform = function(m) {
    this.width = m.xx * this.width + m.yx * this.height;
    this.height = m.xy * this.width + m.yy * this.height;
    return this;
  };

  Size.prototype.isSquare = function() {
    return Math.abs(this.width - this.height) < 1;
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
    for(let other of getArgs(args)) {
      this.width += other.width;
      this.height += other.height;
    }

    return this;
  };

  Size.prototype.diff = function(other) {
    return new Size(this.width - other.width, this.height - other.height);
  };

  Size.prototype.sub = function(...args) {
    for(let other of getArgs(args)) {
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
    for(let f of getArgs(args)) {
      const o = isSize(f)
        ? f
        : isPoint(f)
        ? { width: f.x, height: f.y }
        : { width: f, height: f };
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
    for(let f of getArgs(args)) {
      this.width /= f;
      this.height /= f;
    }

    return this;
  };

  Size.prototype.round = function(precision = (1, digits)) {
    let { width, height } = this;
    this.width = Util.roundTo(width, precision, digits);
    this.height = Util.roundTo(height, precision, digits);
    return this;
  };

  Size.prototype.bounds = function(other) {
    let w = [
      Math.min(this.width, other.width),
      Math.max(this.width, other.width)
    ];
    let h = [
      Math.min(this.height, other.height),
      Math.max(this.height, other.height)
    ];
    let scale = h / this.height;
    this.mul(scale);
    return this;
  };

  Size.prototype.fit = function(size) {
    size = new Size(size);
    let factors = Size.prototype.fitFactors.call(this, size);
    let ret = [
      Size.prototype.prod.call(this, factors[0]),
      Size.prototype.prod.call(this, factors[1])
    ];
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
    return `${left}${width}${unit || units.width || ''}${separator}${height}${
      unit || units.height || ''
    }${right}`;
  };

  /*Size.prototype[Symbol.iterator] = function() {
      let [width,height]= this;
      return [width,height][Symbol.iterator]();
    }*/
  Size.area = sz => Size.prototype.area.call(sz);
  Size.aspect = sz => Size.prototype.aspect.call(sz);

  Size.bind = (...args) => {
    const o = args[0] instanceof Size ? args.shift() : new Size();
    const gen = Util.isFunction(args[args.length - 1]) && args.pop();
    const p = args.length > 1 ? args.pop() : ['width', 'height'];
    const t = args.pop();
    gen = gen || (k => v => (v === undefined ? t.k : (t.k = v)));

    // const [  p = ['width', 'height']  ] = args[0] instanceof Size ? args : [new Size(), ...args];

    console.debug('Size.bind', { args, o, t, p, gen });

    const { width, height } = Util.isArray(p)
      ? p.reduce((acc, name) => ({ ...acc, [[name]]: name }), {})
      : p;
    return Util.bindProperties(new Size(0, 0), t, { width, height }, gen);
  };
  for(let method of Util.getMethodNames(Size.prototype))
    Size.method = (size, ...args) =>
      Size.prototype.method.call(size || new Size(size), ...args);
  const isSize = o =>
    o &&
    ((o.width !== undefined && o.height !== undefined) ||
      (o.x !== undefined &&
        o.x2 !== undefined &&
        o.y !== undefined &&
        o.y2 !== undefined) ||
      (o.left !== undefined &&
        o.right !== undefined &&
        o.top !== undefined &&
        o.bottom !== undefined));

  for(let name of [
    'toCSS',
    'isSquare',
    'round',
    'sum',
    'add',
    'diff',
    'sub',
    'prod',
    'mul',
    'quot',
    'div'
  ]) {
    Size.name = (size, ...args) =>
      Size.prototype.name.call(size || new Size(size), ...args);
  }

  Util.defineGetter(Size, Symbol.species, function() {
    return this;
  });
  const ImmutableSize = Util.immutableClass(Size);
  Util.defineGetter(ImmutableSize, Symbol.species, () => ImmutableSize);

  /* --- concatenated 'lib/geom/rect.js' --- */
  Rect.prototype = { ...Size.prototype, ...Point.prototype, ...Rect.prototype };

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
    return c.concat(c.text('new', 1, 31),
      c.text('Rect', 1, 33),
      `(${x},${y},${width},${height})`
    );
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
      console.log('gen', { k });

      return v => {
        return v !== undefined ? (rect.k = v) : rect.k;
      };
    })
  );
  const getPoint = Util.memoize(rect =>
    Util.bindProperties(new Point(0, 0), rect, ['x', 'y'], k => v =>
      v !== undefined ? (rect.k = v) : rect.k
    )
  );

  Object.defineProperty(Rect.prototype, 'center', {
    get() {
      return Rect.center(this);
    }
  });
  Rect.prototype.getSize = Util.memoize;

  Object.defineProperty(Rect.prototype, 'size', {
    get() {
      let ret = getSize(this);
      console.log('getSize( ) =', ret);
      return ret;
    }
  });

  Object.defineProperty(Rect.prototype, 'point', {
    get() {
      let ret = getPoint(this);
      console.log('getPoint( ) =', ret);
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
    return (Point.prototype.equals.call(this, ...args) &&
      Size.prototype.equals.call(this, ...args)
    );
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
    if(typeof trbl == 'number')
      trbl = { top: trbl, right: trbl, bottom: trbl, left: trbl };
    this.x -= trbl.left;
    this.y -= trbl.top;
    this.width += trbl.left + trbl.right;
    this.height += trbl.top + trbl.bottom;
    return this;
  };

  Rect.prototype.inset = function(trbl) {
    if(typeof trbl == 'number') trbl = new TRBL(trbl, trbl, trbl, trbl);

    if(trbl.left + trbl.right < this.width &&
      trbl.top + trbl.bottom < this.height
    ) {
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

  Rect.prototype.fit = function(other,
    align = Align.CENTER | Align.MIDDLE | Rect.CONTAIN
  ) {
    let factors = Size.prototype.fitFactors
      .call(this, new Size(other))
      .sort((a, b) => a - b);

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
    let a =
      num == 2
        ? [new Point(x, y), new Point(x + width, y + height)]
        : [
            new Point(x, y),
            new Point(x + width, y),
            new Point(x + width, y + height),
            new Point(x, y + height)
          ];
    return ctor(a);
  };

  Rect.prototype.toLines = function(ctor = lines => Array.from(lines, points => new Line(...points))
  ) {
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

  Rect.prototype.round = function(precision = (0.001, (digits = (3, (type = 'round'))))
  ) {
    let { x1, y1, x2, y2 } = this.toObject(true);
    let a = new Point(-x1, -y1).round(precision, digits, type);
    let b = new Point(x2, y2).round(precision, digits, type);
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
    return this;
  };

  Rect.prototype[Symbol.iterator] = function* () {
    let { x, y, width, height } = this;
    for(let prop of [x, y, width, height]) yield prop;
  };
  Rect.round = rect => Rect.prototype.round.call(rect);
  Rect.align = (rect, align_to, a = 0) =>
    Rect.prototype.align.call(rect, align_to, a);
  Rect.toCSS = rect => Rect.prototype.toCSS.call(rect);
  Rect.inset = (rect, trbl) => Rect.prototype.inset.call(rect, trbl);
  Rect.outset = (rect, trbl) => Rect.prototype.outset.call(rect, trbl);
  Rect.center = rect =>
    new Point(rect.x + rect.width / 2, rect.y + rect.height / 2);

  Rect.bind = rect => {
    let obj = new Rect();
  };
  Rect.inside = (rect, point) =>
    point.x >= rect.x &&
    point.x <= rect.x + rect.width &&
    point.y >= rect.y &&
    point.y <= rect.y + rect.height;

  Rect.from = function(obj) {
    //const { x1,y1,x2,y2 } = obj;

    const fn = (v1, v2) => [Math.min(v1, v2), Math.max(v1, v2)];

    const h = fn(obj.x1, obj.x2);
    const v = fn(obj.y1, obj.y2);
    const [x1, x2, y1, y2] = [...h, ...v];
    return new Rect(x1, y1, x2 - x1, y2 - y1);
  };

  //h[0], v[0], h[1] - h[0], v[1] - v[0]);

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
    'toString', //'toSource',

    'points',
    'toCSS',
    'toTRBL',
    'toPoints',
    'equals'
  ]) {
    Rect.name = (rect, ...args) =>
      Rect.prototype.name.call(rect || new Rect(rect), ...args);
  }

  Rect.toSource = (rect, opts = {}) => {
    const { sep = ', ', inner = false, spc = ' ', colon = ':' } = opts;
    let props = `x${colon}${spc}${rect.x}${sep}y${colon}${spc}${rect.y}${sep}width${colon}${spc}${rect.width}${sep}height${colon}${spc}${rect.height}`;
    if(inner) return props;
    return `{${sep}${props}${sep}}`;
  };

  Rect.bind = (...args) => {
    const [o, p, gen = k => v => (v === undefined ? o.k : (o.k = v))] =
      args[0] instanceof Rect ? [new Rect(), ...args] : args;
    const [x, y, width, height] = p || ['x', 'y', 'width', 'height'];
    let pt = Point.bind(o, ['x', 'y'], gen);
    let sz = Size.bind(o, ['width', 'height'], gen);
    let proxy = new Rect(pt, sz);
    return proxy;
  };
  Rect.scale = Util.curry((rect, sx, sy) =>
    Matrix.scale(sx, sy).transform_rect(rect)
  );

  Rect.resize = Util.curry((rect, width, height) => {
    rect.width = width;
    rect.height = height;
    return rect;
  });
  Rect.translate = Util.curry((rect, x, y) =>
    Matrix.translate(f, f).transform_rect(rect)
  );

  for(let f of ['scale', 'resize', 'translate']) {
    Rect.prototype.f = function(...args) {
      Rect.f(this, ...args);
      return this;
    };
  }
  Util.defineInspect(Rect.prototype, 'x', 'y', 'width', 'height');
  const isRect = rect => isPoint(rect) && isSize(rect);

  Util.defineGetter(Rect, Symbol.species, function() {
    return this;
  });
  const ImmutableRect = Util.immutableClass(Rect);
  delete ImmutableRect[Symbol.species];
  Util.defineGetter(ImmutableRect, Symbol.species, () => ImmutableRect);

  Rect.prototype.toString = function(opts = {}) {
    if(typeof opts == 'string') opts = { separator: opts };
    const {
      precision = 0.001,
      unit = '',
      separator = ' ',
      left = '',
      right = ''
    } = opts;
    let { x, y, width, height } = this;
    let props = [x, y, width, height];
    return left + props.map(p => p + unit).join(' ') + right;
  };

  /* --- concatenated 'lib/geom/trbl.js' --- */
  TRBL.prototype.null = function() {
    return (this.top == 0 && this.right == 0 && this.bottom == 0 && this.left == 0
    );
  };
  TRBL.null = trbl => TRBL.prototype.null.call(trbl);
  TRBL.neg = (trbl = this) => ({
    top: -trbl.top,
    right: -trbl.right,
    bottom: -trbl.bottom,
    left: -trbl.left
  });

  TRBL.prototype.isNaN = function() {
    return (isNaN(this.top) ||
      isNaN(this.right) ||
      isNaN(this.bottom) ||
      isNaN(this.left)
    );
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
  TRBL.toRect = trbl =>
    new Rect(trbl.left,
      trbl.top,
      trbl.right - trbl.left,
      trbl.bottom - trbl.top
    );

  TRBL.prototype.toString = function(unit = 'px') {
    return ('' +
      this.top +
      '' +
      unit +
      ' ' +
      this.right +
      '' +
      unit +
      ' ' +
      this.bottom +
      '' +
      unit +
      ' ' +
      this.left +
      unit
    );
  };

  TRBL.prototype.toSource = function() {
    return ('{top:' +
      this.top +
      ',right:' +
      this.right +
      ',bottom:' +
      this.bottom +
      ',left:' +
      this.left +
      '}'
    );
  };

  for(let name of ['null', 'isNaN', 'outset', 'toRect', 'toSource']) {
    TRBL.name = points => TRBL.prototype.name.call(points);
  }

  Util.defineGetter(TRBL, Symbol.species, function() {
    return this;
  });
  const ImmutableTRBL = Util.immutableClass(TRBL);
  Util.defineGetter(ImmutableTRBL, Symbol.species, () => ImmutableTRBL);

  /* --- concatenated 'lib/dom/iterator.js' --- */
  //Example usage:

  //

  //void async function() {

  //let [clicks, onclick] = iterator()

  //document.querySelector('button').addEventListener('click', onclick)

  //for await (let click of clicks) console.log(click)

  //}()

  /* --- concatenated 'lib/geom/pointList.js' --- */
  PointList.prototype[Symbol.toStringTag] = function() {
    return PointList.prototype.toString.apply(this, arguments);
  };
  Util.defineGetter(PointList, Symbol.species, () => PointList);
  PointList.prototype[Symbol.isConcatSpreadable] = true;

  PointList.prototype.rotateRight = function(n) {
    this.unshift(...this.splice(n % this.length, this.length));
    return this;
  };

  PointList.prototype.rotateLeft = function(n) {
    return this.rotateRight(this.length - (n % this.length));
  };

  PointList.prototype.rotate = function(n) {
    if(n < 0) return this.rotateLeft(-n);
    if(n > 0) return this.rotateRight(n);
    return this;
  };

  PointList.prototype.push = function(...args) {
    while(args.length > 0) Array.prototype.push.call(this, new Point(args));
    return this;
  };

  PointList.prototype.unshift = function(...args) {
    let points = [];
    while(args.length > 0) points.push(new Point(args));
    Array.prototype.splice.call(this, 0, 0, ...points);
    return this;
  };
  PointList.prototype.length = 0;

  PointList.prototype.getLength = function() {
    return this.length;
  };

  Object.defineProperty(PointList.prototype, 'size', {
    get() {
      return PointList.prototype.getLength.call(this);
    }
  });

  PointList.prototype.at = function(index) {
    return this[+index];
  };

  PointList.prototype.splice = function() {
    let args = [...arguments];
    const start = args.shift();
    const remove = args.shift();
    return Array.prototype.splice.apply(this, [
      start,
      remove,
      ...args.map(arg => (arg instanceof Point ? arg : new Point(arg)))
    ]);
  };
  PointList.prototype.slice = Array.prototype.slice;

  PointList.prototype.removeSegment = function(index) {
    let indexes = [
      PointList.prototype.getLineIndex.call(this, index - 1),
      PointList.prototype.getLineIndex.call(this, index),
      PointList.prototype.getLineIndex.call(this, index + 1)
    ];
    let lines = indexes.map(i => PointList.prototype.getLine.call(this, i));
    let point = Line.intersect(lines[0], lines[2]);

    if(point) {
      PointList.prototype.splice.call(this, 0, 2, new Point(point));
    }
  };

  /*PointList.prototype.toPath = function(options = {}) {
    const { relative = false, close = false, precision = 0.001 } = options;
    let out = '';
    const point = relative ? (i) => (i > 0 ? Point.diff(PointList.prototype.at.call(this, i), PointList.prototype.at.call(this, i - 1)) : PointList.prototype.at.call(this, i)) : (i) => PointList.prototype.at.call(this, i);
    const cmd = (i) => (i == 0 ? 'M' : 'L'[relative ? 'toLowerCase' : 'toUpperCase']());
    const len = PointList.prototype.getLength.call(this);
    for(let i = 0; i < len; i++) {
      out += cmd(i) + Util.roundTo(point(i).x, precision) + ',' + Util.roundTo(point(i).y, precision) + ' ';
    }
    if(close) out += 'Z';
    return out;
  };*/
  PointList.prototype.clone = function() {
    const ctor = this.constructor[Symbol.species];
    let points = PointList.prototype.map.call(this, p =>
      Point.prototype.clone.call(p)
    );
    return new ctor(points);
  };

  PointList.prototype.toPolar = function(tfn) {
    let ret = new PointList();
    let t =
      typeof tfn == 'function'
        ? tfn
        : (x, y) => ({ x, /*: (x * 180) / Math.PI*/ y });

    ret.splice.apply(ret, [
      0,
      ret.length,
      ...PointList.prototype.map.call(this, p => {
        const angle = Point.prototype.toAngle.call(p);
        return t(angle, Point.prototype.distance.call(p));
      })
    ]);

    return ret;
  };

  PointList.prototype.fromPolar = function(tfn) {
    let ret = new PointList();
    let t =
      typeof tfn == 'function'
        ? tfn
        : (x, y) => ({ x, /*: (x * Math.PI) / 180*/ y });

    ret.splice.apply(ret, [
      0,
      ret.length,
      ...PointList.prototype.map.call(this, p => {
        let r = t(p.x, p.y);
        return Point.prototype.fromAngle.call(r.x, r.y);
      })
    ]);

    return ret;
  };

  PointList.prototype.draw = function(ctx, close = false) {
    const first = PointList.prototype.at.call(this, 0);
    const len = PointList.prototype.getLength.call(this);
    ctx.to(first.x, first.y);

    for(let i = 1; i < len; i++) {
      const { x, y } = PointList.prototype.at.call(this, i);
      ctx.line(x, y);
    }

    if(close) ctx.line(first.x, first.y);
    return this;
  };

  PointList.prototype.area = function() {
    let area = 0;
    let i;
    let j;
    let point1;
    let point2;
    const len = PointList.prototype.getLength.call(this);

    for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
      point1 = PointList.prototype.at.call(this, i);
      point2 = PointList.prototype.at.call(this, j);
      area += point1.x * point2.y;
      area -= point1.y * point2.x;
    }

    area /= 2;
    return area;
  };

  PointList.prototype.centroid = function() {
    let x = 0;
    let y = 0;
    let i;
    let j;
    let f;
    let point1;
    let point2;
    const len = PointList.prototype.getLength.call(this);

    for(i = (0, (j = len - 1)); i < len; j = (i, (i += 1))) {
      point1 = PointList.prototype.at.call(this, i);
      point2 = PointList.prototype.at.call(this, j);
      f = point1.x * point2.y - point2.x * point1.y;
      x += (point1.x + point2.x) * f;
      y += (point1.y + point2.y) * f;
    }

    f = PointList.prototype.area.call(this) * 6;
    return new Point(x / f, y / f);
  };

  PointList.prototype.avg = function() {
    let ret = PointList.prototype.reduce.call(this,
      (acc, p) => acc.add(p),
      new Point()
    );
    return ret.div(PointList.prototype.getLength.call(this));
  };

  PointList.prototype.bbox = function() {
    const len = PointList.prototype.getLength.call(this);
    if(!len) return {};
    const first = PointList.prototype.at.call(this, 0);

    let ret = {
      x1: first.x,
      x2: first.x,
      y1: first.y,
      y2: first.y,
      toString() {
        return `{x1:${(this.x1 + '').padStart(4, ' ')},x2:${(this.x2 + ''
        ).padStart(4, ' ')},y1:${(this.y1 + '').padStart(4, ' ')},y2:${(this.y2 + ''
        ).padStart(4, ' ')}}`;
      }
    };

    for(let i = 1; i < len; i++) {
      const { x, y } = PointList.prototype.at.call(this, i);
      if(x < ret.x1) ret.x1 = x;
      if(x > ret.x2) ret.x2 = x;
      if(y < ret.y1) ret.y1 = y;
      if(y > ret.y2) ret.y2 = y;
    }

    return ret;
  };

  PointList.prototype.rect = function() {
    return new Rect(PointList.prototype.bbox.call(this));
  };

  PointList.prototype.xrange = function() {
    const bbox = PointList.prototype.bbox.call(this);
    return [bbox.x1, bbox.x2];
  };

  PointList.prototype.normalizeX = function(newVal = x => x) {
    const xrange = PointList.prototype.xrange.call(this);
    const xdiff = xrange[1] - xrange[0];

    PointList.prototype.forEach.call(this, (p, i, l) => {
      l.i.x = newVal((l.i.x - xrange[0]) / xdiff);
    });

    return this;
  };

  PointList.prototype.yrange = function() {
    const bbox = PointList.prototype.bbox.call(this);
    return [bbox.y1, bbox.y2];
  };

  PointList.prototype.normalizeY = function(newVal = y => y) {
    const yrange = PointList.prototype.yrange.call(this);
    const ydiff = yrange[1] - yrange[0];

    PointList.prototype.forEach.call(this, (p, i, l) => {
      l.i.y = newVal((l.i.y - yrange[0]) / ydiff);
    });

    return this;
  };

  PointList.prototype.boundingRect = function() {
    return new Rect(PointList.prototype.bbox.call(this));
  };

  PointList.prototype.translate = function(x, y) {
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.move.call(it, x, y)
    );
    return this;
  };

  PointList.prototype.transform = function(m) {
    if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();

    if(Util.isObject(m) && typeof m.transform_point == 'function') {
      this.forEach(p => m.transform_point(p));
      return this;
    }

    for(let i = 0; i < this.length; i++)
      Point.prototype.transform.call(this.i, m);

    return this;
  };

  PointList.prototype.filter = function(pred) {
    let ret = new PointList();
    PointList.prototype.forEach.call(this,
      (p, i, l) => pred(p, i, l) && ret.push(new Point(l.i))
    );
    return ret;
  };

  PointList.prototype.getLineIndex = function(index) {
    const len = PointList.prototype.getLength.call(this);
    return (index < 0 ? len + index : index) % len;
  };

  PointList.prototype.getLine = function(index) {
    let a = PointList.prototype.getLineIndex.call(this, index);
    let b = PointList.prototype.getLineIndex.call(this, index + 1);
    return [
      PointList.prototype.at.call(this, a),
      PointList.prototype.at.call(this, b)
    ];
  };

  PointList.prototype.lines = function(closed = false) {
    const points = this;
    const n = points.length - (closed ? 0 : 1);

    const iterableObj = {
      [Symbol.iterator]() {
        let step = 0;

        return {
          next() {
            let value;
            let done = step >= n;

            if(!done) {
              value = new Line(points.step, points[(step + 1) % points.length]);
              step++;
            }

            return { value, done };
          }
        };
      }
    };

    return iterableObj;
  };

  PointList.prototype.sort = function(pred) {
    return Array.prototype.sort.call(this,
      pred ||
        ((a, b) =>
          Point.prototype.valueOf.call(a) - Point.prototype.valueOf.call(b))
    );
  };

  PointList.prototype.toString = function(sep = (',', prec)) {
    return Array.prototype.map
      .call(this, point =>
        Point.prototype.toString
          ? Point.prototype.toString.call(point, prec, sep)
          : point + ''
      )
      .join(' ');
  };

  PointList.prototype[Symbol.toStringTag] = function(sep = (',', prec)) {
    return Array.prototype.map
      .call(this, point => point.round(prec))
      .map(point => `${point.x}${sep}${point.y}`)
      .join(' ');
  };

  PointList.prototype.toPath = function() {
    return Array.prototype.map
      .call(this, (point, i) => `${i > 0 ? 'L' : 'M'}${point}`)
      .join(' ');
    return Array.prototype.reduce.call(this,
      (acc, point, i) => (acc ? acc + ' ' : '') + `${acc ? 'L' : 'M'}${point}`
    );
  };

  PointList.prototype.toSource = function(opts = {}) {
    if(opts.asString) return `new PointList("${this.toString(opts)}")`;
    let fn = opts.asArray
      ? p => `[${p.x},${p.y}]`
      : opts.plainObj
      ? p => Point.toSource(p, { space: '', padding: ' ', separator: ',' })
      : point =>
          Point.prototype.toSource.call(point, { ...opts, plainObj: true });
    return ('new PointList([' +
      PointList.prototype.map.call(this, fn).join(',') +
      '])'
    );
  };

  PointList.prototype.add = function(pt) {
    if(!(pt instanceof Point)) pt = new Point(...arguments);
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.add.call(it, pt)
    );
    return this;
  };

  PointList.prototype.sum = function(pt) {
    let ret = PointList.prototype.clone.call(this);
    return PointList.prototype.add.apply(ret, arguments);
  };

  PointList.prototype.sub = function(pt) {
    if(!(pt instanceof Point)) pt = new Point(...arguments);
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.sub.call(it, pt)
    );
    return this;
  };

  PointList.prototype.diff = function(pt) {
    let ret = PointList.prototype.clone.call(this);
    return PointList.prototype.sub.apply(ret, arguments);
  };

  PointList.prototype.mul = function(pt) {
    if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
    if(!(pt instanceof Point)) pt = new Point(...arguments);
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.mul.call(it, pt)
    );
    return this;
  };

  PointList.prototype.prod = function(pt) {
    let ret = PointList.prototype.clone.call(this);
    return PointList.prototype.mul.apply(ret, arguments);
  };

  PointList.prototype.div = function(pt) {
    if(typeof pt == 'number') pt = new Point({ x: pt, y: pt });
    if(!(pt instanceof Point)) pt = new Point(...arguments);
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.div.call(it, pt)
    );
    return this;
  };

  PointList.prototype.quot = function(pt) {
    let ret = PointList.prototype.clone.call(this);
    return PointList.prototype.div.apply(ret, arguments);
  };

  PointList.prototype.round = function(prec) {
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.round.call(it, prec)
    );
    return this;
  };

  PointList.prototype.ceil = function(prec) {
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.ceil.call(it, prec)
    );
    return this;
  };

  PointList.prototype.floor = function(prec) {
    PointList.prototype.forEach.call(this, it =>
      Point.prototype.floor.call(it, prec)
    );
    return this;
  };

  PointList.prototype.toMatrix = function() {
    return Array.prototype.map.call(this, ({ x, y }) => Object.freeze([x, y]));
  };

  if(!Util.isBrowser()) {
    let c = Util.coloring();
    let sym = Symbol.for('nodejs.util.inspect.custom');

    PointList.prototype.sym = function() {
      return `${c.text('PointList', 1, 31)}${c.text('(', 1, 36)}${
        c.text(this.getLength(), 1, 35) + c.code(1, 36)
      }) [
        ${this.map(({ x, y }) =>
          Util.inspect({ x, y }, { multiline: false, spacing: ' ' })
        ).join(///*Point.prototype.toSource.call(point, { plainObj: true, colors: true })  ||*/ Util.toSource(point, {colors: true }) || point[sym]() ||

          ',\n  '
        )}
      ${c.text(']', 1, 36)}`;
    };
  }

  for(let name of [
    'push',
    'splice',
    'clone',
    'area',
    'centroid',
    'avg',
    'bbox',
    'rect',
    'xrange',
    'yrange',
    'boundingRect'
  ]) {
    PointList.name = points => PointList.prototype.name.call(points);
  }
  Polyline.prototype = new PointList();

  Polyline.prototype.toSVG = function(factory,
    attrs = ({}, (parent = (null, prec)))
  ) {
    return factory('polyline',
      { points: PointList.prototype.toString.call(this), ...attrs },
      parent,
      prec
    );
  };

  Polyline.prototype.push = function(...args) {
    const last = this[this.length - 1];

    for(let arg of args) {
      if(last && Point.equals(arg, last)) continue;
      PointList.prototype.push.call(this, arg);
    }

    return this;
  };

  Polyline.prototype.inside = function(point) {
    let i,
      j,
      c = false,
      nvert = this.length;

    for(i = (0, (j = nvert - 1)); i < nvert; j = i++) {
      if(this.i.y > point.y !== this.j.y > point.y &&
        point.x <
          ((this.j.x - this.i.x) * (point.y - this.i.y)) /
            (this.j.y - this.i.y) +
            this.i.x
      ) {
        c = !c;
      }
    }

    return c;
  };

  Polyline.inside = function(a, b) {
    return a.every(point => b.inside(point));
  };

  Polyline.prototype.isClockwise = function() {
    let sum = 0;

    for(let i = 0; i < this.length - 1; i++) {
      let cur = this.i,
        next = this[i + 1];
      sum += (next.x - cur.x) * (next.y + cur.y);
    }

    return sum > 0;
  };

  Util.defineGetter(Polyline.prototype, 'clockwise', function() {
    let ret = new (this[Symbol.species] || this.constructor)().concat(this);
    return Polyline.prototype.isClockwise.call(this) ? ret : ret.reverse();
  });

  Util.defineGetter(Polyline.prototype, 'counterClockwise', function() {
    let ret = new (this[Symbol.species] || this.constructor)().concat(this);
    return Polyline.prototype.isClockwise.call(this) ? ret.reverse() : ret;
  });

  Polyline.isClockwise = function isClockwise(poly) {
    let sum = 0;

    for(let i = 0; i < poly.length - 1; i++) {
      let cur = poly.i,
        next = poly[i + 1];
      sum += (next.x - cur.x) * (next.y + cur.y);
    }

    return sum > 0;
  };

  Util.define(PointList, {
    get [Symbol.species]() {
      return PointList;
    }
  });
  const ImmutablePointList = Util.immutableClass(PointList);
  Util.defineGetter(ImmutablePointList,
    Symbol.species,
    () => ImmutablePointList
  );

  /* --- concatenated 'lib/geom/lineList.js' --- */
  /**
   *
   * @param [[[x, y], [x, y]], ...] lines
   */
  LineList.toPolygons = (lines,
    createfn = points => Object.setPrototypeOf(points, PointList.prototype)
  ) => {
    const polygons = [];

    for(var i = 0; i < lines.length; i++) {
      // Récupération et suppression du tableau du premier élément

      const firstLine = lines.splice(i--, 1)[0];

      // create a new polygon array

      const polygon = [];

      // init current start point and current end point

      let currentStartPoint = firstLine.a;

      let currentEndPoint = firstLine.b;

      // put the 2 points of the first line on the polygon array

      polygon.push(currentStartPoint, currentEndPoint);

      let j = 0;

      // init the linesLength

      let linesLength = lines.length;

      while(lines.length &&
        (j < lines.length || linesLength != lines.length)
      ) {
        // if j == lines.length, we have to return to the first index and redefine linesLength to the new lines.length

        if(j == lines.length) {
          j = 0;
          linesLength = lines.length;
        }

        // The nextLine in the array

        const nextLine = lines[j++];

        // min 3 lines to have a closed polygon

        // check if the polygon is closed (the nextLine start point is one of the current start or end point and the nextLine end point is one of the current start or end point)

        if(polygon.length >= 3 &&
          ((currentEndPoint.x === nextLine.x1 &&
            currentEndPoint.y === nextLine.y1 &&
            currentStartPoint.x === nextLine.x2 &&
            currentStartPoint.y === nextLine.y2) ||
            (currentStartPoint.x === nextLine.x1 &&
              currentStartPoint.y === nextLine.y1 &&
              currentEndPoint.x === nextLine.x2 &&
              currentEndPoint.y === nextLine.y2))
        ) {
          polygons.push(polygon);
          break;
        }

        if(currentEndPoint.x === nextLine.x1 &&
          currentEndPoint.y === nextLine.y1
        ) {
          // end point of the current line equals to start point of the next line

          polygon.push(nextLine.b);

          // update current end point

          currentEndPoint = nextLine.b;

          // Suppression de la ligne dans le tableau

          lines.splice(--j, 1);
        } else if(currentStartPoint.x === nextLine.x1 &&
          currentStartPoint.y === nextLine.y1
        ) {
          // start point of the current line equals to start point of the next line

          polygon.unshift(nextLine.b);

          // update current start point

          currentStartPoint = nextLine.b;

          lines.splice(--j, 1);
        } else if(currentEndPoint.x === nextLine.x2 &&
          currentEndPoint.y === nextLine.y2
        ) {
          // end point of the current line equals to end point of the next line

          polygon.push(nextLine.a);

          // update current end point

          currentEndPoint = nextLine.a;

          lines.splice(--j, 1);
        } else if(currentStartPoint.x == nextLine.x2 &&
          currentStartPoint.y == nextLine.y2
        ) {
          // start point of the current line equals to end point of the next line

          polygon.unshift(nextLine.a);

          // update current start point

          currentStartPoint = nextLine.a;

          lines.splice(--j, 1);
        }
      }
    }

    return polygons.map(points => createfn(points));
  };

  /*
  if(!Util.isBrowser()) {
    let c = Util.coloring();
    const sym = Symbol.for('nodejs.util.inspect.custom');
    LineList.prototype[sym] = function() {
      return `${c.text('LineList', 1, 31)}${c.text('(', 1, 36)}${c.text(this.length, 1, 35) + c.code(1, 36)}) [\n  ${this.map((line) => line[sym]()  ).join(',\n  ')}\n${c.text(']', 1, 36)}`;
    };
  }*/

  Util.defineGetter(LineList, Symbol.species, function() {
    return this;
  });
  const ImmutableLineList = Util.immutableClass(LineList);
  Util.defineGetter(ImmutableLineList, Symbol.species, () => ImmutableLineList);

  /* --- concatenated 'lib/geom/matrix.js' --- */
  /*
  Object.assign(Matrix.prototype, Array.prototype);
  
  Matrix.prototype.splice = Array.prototype.splice;
  Matrix.prototype.slice = Array.prototype.slice;
  */

  Object.defineProperty(Matrix, Symbol.species, {
    get() {
      return Matrix;
    }
  });

  Matrix.prototype[Symbol.toStringTag] = function() {
    return Matrix.prototype.toString.apply(this, arguments);
  };
  Matrix.prototype[Symbol.isConcatSpreadable] = false;
  Object.defineProperty(Matrix.prototype, 'length', {
    value: 6,
    enumerable: false,
    writable: true,
    configurable: false
  });
  Matrix.prototype.keys = ['xx', 'xy', 'x0', 'yx', 'yy', 'y0'];
  Matrix.prototype.keySeq = ['xx', 'yx', 'xy', 'yy', 'x0', 'y0'];
  const keyIndexes = {
    xx: 0,
    a: 0,
    xy: 1,
    c: 1,
    x0: 2,
    tx: 2,
    e: 2,
    yx: 3,
    b: 3,
    yy: 4,
    d: 4,
    y0: 5,
    ty: 5,
    f: 5
  };

  Matrix.prototype.at = function(col, row = 0) {
    return this[row * 3 + col];
  };

  Matrix.prototype.get = function(field) {
    if(typeof field == 'number' && field < this.length) return this.field;
    if((field = keyIndexes.field)) return this.field;
  };

  const MatrixProps = (obj = {}) =>
    Object.entries(keyIndexes).reduce((acc, [k, i]) => ({
        ...acc,
        [[k]]: {
          get() {
            return this.i;
          },
          set(v) {
            this.i = v;
          },
          enumerable: true
        }
      }),
      obj
    );
  Object.defineProperties(Matrix.prototype, MatrixProps());

  //prettier-ignore

  /*Object.defineProperties(Matrix.prototype, {
    xx: {get: function() { return this[0]; }, set: function(v) {this[0] = v; }, enumerable: true },
    xy: {get: function() { return this[1]; }, set: function(v) {this[1] = v; }, enumerable: true },
    x0: {get: function() { return this[2]; }, set: function(v) {this[2] = v; }, enumerable: true },
    yx: {get: function() { return this[3]; }, set: function(v) {this[3] = v; }, enumerable: true },
    yy: {get: function() { return this[4]; }, set: function(v) {this[4] = v; }, enumerable: true },
    y0: {get: function() { return this[5]; }, set: function(v) {this[5] = v; }, enumerable: true }
  });*/

  //Object.defineProperties(Matrix.prototype,['a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i'].reduce((acc,prop,i) => ({ ...acc, [prop]: { get: function() { return this[i]; }, set: function(v) { this[i] = v; } } }), {}));

  Matrix.propDescriptors = MatrixProps;

  Matrix.prototype.init = function(...args) {
    if(args.length == 1) args = args[0];
    if(args.length < 9)
      args = args.concat(Array.prototype.slice.call(Matrix.IDENTITY, args.length)
      );
    Array.prototype.splice.call(this, 0, this.length, ...args);
    return this;
  };

  Matrix.prototype.set_row = function(...args) {
    const start = args.shift() * 3;
    const end = Math.max(3, args.length);

    for(let i = 0; i < end; i++) this[start + i] = args.i;

    return this;
  };

  Matrix.prototype.multiply = function(...args) {
    return this.clone().multiplySelf(...args);
  };

  Matrix.prototype.multiplySelf = function(...args) {
    for(let arg of args) {
      if(!(arg instanceof Matrix))
        throw new Error('Not a Matrix: ' + arg.constructor);

      this.init([
        this[0] * arg[0] + this[1] * arg[3],
        this[0] * arg[1] + this[1] * arg[4],
        this[0] * arg[2] + this[1] * arg[5] + this[2],
        this[3] * arg[0] + this[4] * arg[3],
        this[3] * arg[1] + this[4] * arg[4],
        this[3] * arg[2] + this[4] * arg[5] + this[5]
      ]);
    }

    return this;
  };

  Matrix.prototype.multiply_self = function(...args) {
    for(let m of args) {
      if(!(m instanceof Matrix)) m = new Matrix(m);
      Matrix.prototype.init.call(this,
        this[0] * m[0] + this[1] * m[3],
        this[0] * m[1] + this[1] * m[4],
        this[0] * m[2] + this[1] * m[5] + this[2],
        this[3] * m[0] + this[4] * m[3],
        this[3] * m[1] + this[4] * m[4],
        this[3] * m[2] + this[4] * m[5] + this[5]
      );
    }

    return this;
  };

  Matrix.prototype.toObject = function() {
    const { xx, xy, x0, yx, yy, y0 } = this;
    return { xx, xy, x0, yx, yy, y0 };
  };

  Matrix.prototype.entries = function() {
    return Object.entries(Matrix.prototype.toObject.call(this));
  };

  Matrix.prototype.clone = function() {
    const ctor = this.constructor[Symbol.species];
    return new ctor(Array.from(this));
  };

  Matrix.prototype.row = function(row) {
    let i = row * 3;
    return Array.prototype.slice.call(this, i, i + 3);
  };

  Matrix.prototype.rows = function() {
    let ret = [];

    for(let i = 0; i < 9; i += 3)
      ret.push([this[i + 0], this[i + 1], this[i + 2]]);

    return ret;
  };

  Matrix.prototype.toArray = function() {
    return Array.from(this);
  };

  Matrix.prototype.isIdentity = function() {
    return Util.equals(this, Matrix.IDENTITY);
  };

  Matrix.prototype.determinant = function() {
    return (this[0] * (this[4] * this[8] - this[5] * this[7]) +
      this[1] * (this[5] * this[6] - this[3] * this[8]) +
      this[2] * (this[3] * this[7] - this[4] * this[6])
    );
  };

  Matrix.prototype.invert = function() {
    const det = Matrix.prototype.determinant.call(this);
    return new Matrix([
      (this[4] * this[8] - this[5] * this[7]) / det,
      (this[2] * this[7] - this[1] * this[8]) / det,
      (this[1] * this[5] - this[2] * this[4]) / det,
      (this[5] * this[6] - this[3] * this[8]) / det,
      (this[0] * this[8] - this[2] * this[6]) / det,
      (this[2] * this[3] - this[0] * this[5]) / det,
      (this[3] * this[7] - this[4] * this[6]) / det,
      (this[6] * this[1] - this[0] * this[7]) / det,
      (this[0] * this[4] - this[1] * this[3]) / det
    ]);
  };

  Matrix.prototype.scalar_product = function(f) {
    return new Matrix({
      xx: this[0] * f,
      xy: this[1] * f,
      x0: this[2] * f,
      yx: this[3] * f,
      yy: this[4] * f,
      y0: this[5] * f
    });
  };

  Matrix.prototype.toSource = function(construct = (false, (multiline = true))
  ) {
    const nl = multiline ? '\n' : '';
    const rows = Matrix.prototype.rows.call(this);
    const src = `${rows
      .map(row => row.join(','))
      .join(multiline ? ',\n ' : ',')}`;
    return construct ? `new Matrix([${nl}${src}${nl}])` : `[${src}]`;
  };

  Matrix.prototype.toString = function(separator = ' ') {
    let rows = Matrix.prototype.rows.call(this);
    let name = rows[0].length == 3 ? 'matrix' : 'matrix3d';

    if(rows[0].length == 3) {
      rows = [['a', 'b', 'c', 'd', 'e', 'f'].map(k => this[keyIndexes.k])];
    }

    return (`${name}(` +
      rows.map(row => row.join(',' + separator)).join(',' + separator) +
      ')'
    );
  };

  Matrix.prototype.toSVG = function() {
    return ('matrix(' +
      ['a', 'b', 'c', 'd', 'e', 'f'].map(k => this[keyIndexes.k]).join(',') +
      ')'
    );
  };

  Matrix.prototype.toDOM = function(ctor = DOMMatrix) {
    const rows = Matrix.prototype.rows.call(this);
    const [a, c, e] = rows[0];
    const [b, d, f] = rows[1];
    return new ctor([a, b, c, d, e, f]);
  };

  Matrix.prototype.toJSON = function() {
    const rows = Matrix.prototype.rows.call(this);
    const [a, c, e] = rows[0];
    const [b, d, f] = rows[1];
    return { a, b, c, d, e, f };
  };
  Matrix.fromJSON = obj => new Matrix(obj);

  Matrix.fromDOM = matrix => {
    const { a, b, c, d, e, f } = matrix;
    return new Matrix([a, c, e, b, d, f]);
  };

  Matrix.prototype.equals = function(other) {
    return Array.prototype.every.call((n, i) => other.i == n);
  };

  Matrix.prototype.transform_distance = function(d) {
    const k =
      'x' in d && 'y' in d
        ? ['x', 'y']
        : 'width' in d && 'height' in d
        ? ['width', 'height']
        : [0, 1];
    const x = this[0] * d[k[0]] + this[2] * d[k[1]];
    const y = this[1] * d[k[0]] + this[3] * d[k[1]];
    d[k[0]] = x;
    d[k[1]] = y;
    return d;
  };

  Matrix.prototype.transform_xy = function(x, y) {
    const m0 = this.row(0);
    const m1 = this.row(1);
    return [m0[0] * x + m0[1] * y + m0[2], m1[0] * x + m1[1] * y + m0[2]];
  };

  Matrix.prototype.transform_point = function(p) {
    const k = 'x' in p && 'y' in p ? ['x', 'y'] : [0, 1];
    const m0 = this.row(0);
    const m1 = this.row(1);
    const x = m0[0] * p[k[0]] + m0[1] * p[k[1]] + m0[2];
    const y = m1[0] * p[k[0]] + m1[1] * p[k[1]] + m1[2];
    p[k[0]] = x;
    p[k[1]] = y;
    return p;
  };

  Matrix.prototype.transformGenerator = function(what = 'point') {
    const matrix = Object.freeze(this.clone());

    return function* (list) {
      const method =
        Matrix.prototype['transform_' + what] ||
        (typeof what == 'function' && what) ||
        Matrix.prototype.transform_xy;
      for(let item of list)
        yield item instanceof Array
          ? method.apply(matrix, [...item])
          : method.call(matrix, { ...item });
    };
  };

  Matrix.prototype.transform_points = function* (list) {
    for(let i = 0; i < list.length; i++)
      yield Matrix.prototype.transform_point.call(this, { ...list.i });
  };

  Matrix.prototype.transform_wh = function(width, height) {
    const w = this[0] * width + this[1] * height;
    const h = this[3] * width + this[4] * height;
    return [w, h];
  };

  Matrix.prototype.transform_size = function(s) {
    const w = this[0] * s.width + this[1] * s.height;
    const h = this[3] * s.width + this[4] * s.height;
    s.width = w;
    s.height = h;
    return s;
  };

  Matrix.prototype.transform_xywh = function(x, y, width, height) {
    return [
      ...Matrix.prototype.transform_xy.call(this, x, y),
      ...Matrix.prototype.transform_wh.call(this, width, height)
    ];
  };

  Matrix.prototype.transform_rect = function(rect) {
    let { x1, y1, x2, y2 } = rect;
    [x1, y1] = Matrix.prototype.transform_xy.call(this, x1, y1);
    [x2, y2] = Matrix.prototype.transform_xy.call(this, x2, y2);
    let xrange = [x1, x2];
    let yrange = [y1, y2];
    [x1, x2] = [Math.min, Math.max].map(fn => fn(...xrange));
    [y1, y2] = [Math.min, Math.max].map(fn => fn(...yrange));
    Object.assign(rect, { x1, x2, y1, y2 });
    return rect;
  };

  Matrix.prototype.point_transformer = function() {
    const matrix = this;
    return point => matrix.transform_point(point);
  };

  Matrix.prototype.transformer = function() {
    const matrix = this;
    return {
      point: point => matrix.transform_point(point),
      xy: (x, y) => matrix.transform_xy(x, y),
      size: s => matrix.transform_size(s),
      wh: (w, h) => matrix.transform_wh(w, h),
      rect: rect => matrix.transform_rect(rect),
      points: list => matrix.transform_points(list),
      distance: d => matrix.transform_distance(d)
    };
  };

  Matrix.prototype.scale_sign = function() {
    return this[0] * this[4] < 0 || this[1] * this[3] > 0 ? -1 : 1;
  };

  Matrix.prototype.affine_transform = function(a, b) {
    let xx, yx, xy, yy, tx, ty;
    if(typeof a == 'object' && a.toPoints !== undefined) a = a.toPoints();
    if(typeof b == 'object' && b.toPoints !== undefined) b = b.toPoints();
    xx =
      (b[0].x * a[1].y +
        b[1].x * a[2].y +
        b[2].x * a[0].y -
        b[0].x * a[2].y -
        b[1].x * a[0].y -
        b[2].x * a[1].y) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    yx =
      (b[0].y * a[1].y +
        b[1].y * a[2].y +
        b[2].y * a[0].y -
        b[0].y * a[2].y -
        b[1].y * a[0].y -
        b[2].y * a[1].y) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    xy =
      (a[0].x * b[1].x +
        a[1].x * b[2].x +
        a[2].x * b[0].x -
        a[0].x * b[2].x -
        a[1].x * b[0].x -
        a[2].x * b[1].x) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    yy =
      (a[0].x * b[1].y +
        a[1].x * b[2].y +
        a[2].x * b[0].y -
        a[0].x * b[2].y -
        a[1].x * b[0].y -
        a[2].x * b[1].y) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    tx =
      (a[0].x * a[1].y * b[2].x +
        a[1].x * a[2].y * b[0].x +
        a[2].x * a[0].y * b[1].x -
        a[0].x * a[2].y * b[1].x -
        a[1].x * a[0].y * b[2].x -
        a[2].x * a[1].y * b[0].x) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    ty =
      (a[0].x * a[1].y * b[2].y +
        a[1].x * a[2].y * b[0].y +
        a[2].x * a[0].y * b[1].y -
        a[0].x * a[2].y * b[1].y -
        a[1].x * a[0].y * b[2].y -
        a[2].x * a[1].y * b[0].y) /
      (a[0].x * a[1].y +
        a[1].x * a[2].y +
        a[2].x * a[0].y -
        a[0].x * a[2].y -
        a[1].x * a[0].y -
        a[2].x * a[1].y);
    this.set_row.call(this, 0, xx, xy, tx);
    this.set_row.call(this, 1, yx, yy, ty);
    this.set_row.call(this, 2, 0, 0, 1);
    return this;
  };

  Matrix.getAffineTransform = (a, b) => {
    let matrix = new Matrix();
    matrix.affine_transform(a, b);
    return matrix;
  };

  Matrix.prototype.decompose = function(degrees = (false, (useLU = true))) {
    let a = this[0],
      b = this[3],
      c = this[1],
      d = this[4];
    let translate = { x: this[2], y: this[5] },
      rotation = 0,
      scale = { x: 1, y: 1 },
      skew = { x: 0, y: 0 };
    let determ = a * d - b * c,
      r,
      s;

    const calcFromValues = (r1, m1, r2, m2) => {
      if(!isFinite(r1)) return r2;
      else if(!isFinite(r2)) return r1;
      (m1 = Math.abs(m1)), (m2 = Math.abs(m2));
      return Util.roundTo((m1 * r1 + m2 * r2) / (m1 + m2), 0.0001);
    };

    //if(useLU) {

    let sign = Matrix.prototype.scale_sign.call(this);

    rotation =
      (Math.atan2(this[3], this[4]) +
        Math.atan2(-sign * this[1], sign * this[0])) /
      2;
    const cos = Math.cos(rotation),
      sin = Math.sin(rotation);
    scale = {
      x: calcFromValues(this[0] / cos, cos, -this[1] / sin, sin),
      y: calcFromValues(this[4] / cos, cos, this[3] / sin, sin)
    };

    /*  } else if(a) {
          skew = { x: Math.atan(c / a), y: Math.atan(b / a) };
          scale = { x: a, y: determ / a };
        } else {
          scale = { x: c, y: d };
          skew.x = Math.PI * 0.25;
        }*/
    /* } else {
        if(a || b) {
          r = Math.sqrt(a * a + b * b);
          rotation = b > 0 ? Math.acos(a / r) : -Math.acos(a / r);
          scale = { x: r, y: determ / r };
          skew.x = Math.atan((a * c + b * d) / (r * r));
        } else if(c || d) {
          s = Math.sqrt(c * c + d * d);
          rotation = Math.PI * 0.5 - (d > 0 ? Math.acos(-c / s) : -Math.acos(c / s));
          scale = { x: determ / s, y: s };
          skew.y = Math.atan((a * c + b * d) / (s * s));
        } else {
          scale = { x: 0, y: 0 };
        }
      }*/

    return {
      translate,
      rotate: degrees === true
          ? Util.roundTo(Matrix.rad2deg(rotation), 0.1)
          : rotation,
      scale,
      skew
    };
  };

  Matrix.prototype.init_identity = function() {
    return Matrix.prototype.init.call(this, 1, 0, 0, 0, 1, 0, 0, 0, 1);
  };

  Matrix.prototype.is_identity = function() {
    return Matrix.prototype.equals.call(this, [1, 0, 0, 0, 1, 0, 0, 0, 1]);
  };

  Matrix.prototype.init_translate = function(tx, ty) {
    return Matrix.prototype.init.call(this, 1, 0, tx, 0, 1, ty);
  };

  Matrix.prototype.init_scale = function(sx, sy) {
    if(sy === undefined) sy = sx;
    return Matrix.prototype.init.call(this, sx, 0, 0, 0, sy, 0);
  };

  Matrix.prototype.init_rotate = function(angle, deg = false) {
    const rad = deg ? Matrix.deg2rad(angle) : angle;
    const s = Math.sin(rad);
    const c = Math.cos(rad);
    return Matrix.prototype.init.call(this, c, -s, 0, s, c, 0);
  };

  Matrix.prototype.init_skew = function(x, y, deg = false) {
    const ax = Math.tan(deg ? Matrix.deg2rad(x) : x);
    const ay = Math.tan(deg ? Matrix.deg2rad(y) : y);
    return Matrix.prototype.init.call(this, 1, ax, 0, ay, 1, 0);
  };
  Matrix.identity = () => new Matrix([1, 0, 0, 0, 1, 0, 0, 0, 1]);
  Matrix.IDENTITY = Object.freeze(Matrix.identity());
  Matrix.rad2deg = radians => (radians * 180) / Math.PI;
  Matrix.deg2rad = degrees => (degrees * Math.PI) / 180;

  for(let name of [
    'toObject',
    'init',
    'toArray',
    'isIdentity',
    'determinant',
    'invert',
    'multiply',
    'scalar_product',
    'toSource',
    'toString',
    'toSVG',
    'equals',
    'init_identity',
    'is_identity',
    'init_translate',
    'init_scale',
    'init_rotate',
    'scale_sign',
    'decompose',
    'transformer'
  ]) {
    Matrix.name = (matrix, ...args) =>
      Matrix.prototype.name.call(matrix || new Matrix(matrix), ...args);
  }

  for(let name of ['translate', 'scale', 'rotate', 'skew']) {
    Matrix.name = (...args) =>
      Matrix.prototype['init_' + name].call(new Matrix(), ...args);
  }

  for(let name of ['translate', 'scale', 'rotate', 'skew']) {
    Matrix.prototype.name = function(...args) {
      return Matrix.prototype.multiply.call(this,
        new Matrix()['init_' + name](...args)
      );
    };

    Matrix.prototype[name + '_self'] = function(...args) {
      return Matrix.prototype.multiply_self.call(this,
        new Matrix()['init_' + name](...args)
      );
    };
  }

  for(let name of [
    'transform_distance',
    'transform_xy',
    'transform_point',
    'transform_points',
    'transform_wh',
    'transform_size',
    'transform_rect',
    'affine_transform'
  ]) {
    const method = Matrix.prototype.name;

    if(method.length == 2) {
      Matrix.name = Util.curry((m, a, b) =>
        Matrix.prototype.name.call(m || new Matrix(m), a, b)
      );
    } else if(method.length == 1) {
      Matrix.name = Util.curry((m, a) =>
        Matrix.prototype.name.call(m || new Matrix(m), a)
      );
    }
  }

  Util.defineGetter(Matrix, Symbol.species, function() {
    return this;
  });
  const isMatrix = m =>
    Util.isObject(m) &&
    (m instanceof Matrix ||
      (m.length !== undefined &&
        (m.length == 6 || m.length == 9) &&
        m.every(el => typeof el == 'number')));
  const ImmutableMatrix = Util.immutableClass(Matrix);
  Util.defineGetter(ImmutableMatrix, Symbol.species, () => ImmutableMatrix);

  /* --- concatenated 'lib/geom/circle.js' --- */
  const isCircle = obj =>
    ['x', 'y', 'radius'].every(prop => obj.prop !== undefined);
  Object.defineProperty(Circle.prototype, 'x', {
    value: 0,
    enumerable: true,
    writable: true
  });
  Object.defineProperty(Circle.prototype, 'y', {
    value: 0,
    enumerable: true,
    writable: true
  });
  Object.defineProperty(Circle.prototype, 'radius', {
    value: 0,
    enumerable: true,
    writable: true
  });

  Object.defineProperty(Circle.prototype, 'center', {
    get() {
      return Point.bind(this, null, value => {
        if(value === undefined) return new Point(this.x, this.y);
        this.x = value.x;
        this.y = value.y;
      });
    }
  });

  Circle.prototype.bbox = function(width = 0) {
    const { x, y, radius } = this;
    let distance = radius + width;
    return new Rect({
      x1: x - distance,
      x2: x + distance,
      y1: y - distance,
      y2: y + distance
    });
  };

  Circle.prototype.transform = function(m) {
    if(Util.isObject(m) && typeof m.toMatrix == 'function') m = m.toMatrix();
    Matrix.prototype.transform_point.call(m, this);
    this.radius = Matrix.prototype.transform_wh.call(m,
      this.radius,
      this.radius
    )[0];
    return this;
  };
  Util.defineInspect(Circle.prototype, 'x', 'y', 'radius');

  Circle.bind = (o, p, gen) => {
    const [x, y, radius] = p || ['x', 'y', 'radius'];
    if(!gen) gen = k => v => (v === undefined ? o.k : (o.k = v));
    return Util.bindProperties(new Circle(0, 0, 0), o, { x, y, radius }, gen);
  };

  /* --- concatenated 'lib/geom/polygon.js' --- */
  const Polygon = function Polygon() {};

  Polygon.area = polygon => {
    let area = 0;
    let j = polygon.length - 1;
    let p1;
    let p2;

    for(let k = 0; k < polygon.length; j = k++) {
      p1 = polygon.k;
      p2 = polygon.j;

      if(p1.x !== undefined && p2.x !== undefined) {
        area += p1.x * p2.y;
        area -= p1.y * p2.x;
      } else {
        area += p1[0] * p2[1];
        area -= p1[1] * p2[0];
      }
    }

    area = area / 2;
    return area;
  };

  Polygon.center = polygon => {
    let x = 0;
    let y = 0;
    let f;
    let j = polygon.length - 1;
    let p1;
    let p2;

    for(let k = 0; k < polygon.length; j = k++) {
      p1 = polygon.k;
      p2 = polygon.j;

      if(p1.x !== undefined && p2.x !== undefined) {
        f = p1.x * p2.y - p2.x * p1.y;
        x += (p1.x + p2.x) * f;
        y += (p1.y + p2.y) * f;
      } else {
        f = p1[0] * p2[1] - p2[0] * p1[1];
        x += (p1[0] + p2[0]) * f;
        y += (p1[1] + p2[1]) * f;
      }
    }

    f = area(polygon) * 6;
    return [x / f, y / f];
  };

  Polygon.approxCircle = (radius, npoints) => {
    let ret = [];

    for(let k = 0; k < npoints; k++) {
      let theta = (Math.PI * 2 * k) / npoints;
      let x = Math.sin(theta) * radius;
      let y = Math.cos(theta) * radius;
      ret.push({ x, y });
    }

    return ret;
  };

  Polygon.toPath = (polygon, relative = true) => {
    let prevx = 0;
    let prevy = 0;
    let path = '';

    for(let k = 0; k < polygon.length; k++) {
      let x = polygon.k.x !== undefined ? polygon.k.x : polygon.k[0];
      let y = polygon.k.y !== undefined ? polygon.k.y : polygon.k[1];

      if(relative) {
        x -= prevx;
        y -= prevy;
      }

      let cmd = k == 0 ? 'M' : 'L';
      if(relative) cmd = cmd.toLowerCase();
      path += `${cmd}${x},${y}`;
    }

    path += 'z';
    return path;
  };

  Polygon.fromLine = (arg, offset, steps = 3) => {
    let line = new Line(arg);
    const PI2 = Math.PI * 0.5;
    const step = Util.range(0, steps - 1).map(i => (i * Math.PI) / (steps - 1));
    const a = line.angle();
    let vl = new PointList();

    //Util.log('step:', step);

    vl = vl.concat(step.map(va => Point.fromAngle(a - PI2 - va, offset).sum(line.a))
    );

    vl = vl.concat(step.map(va => Point.fromAngle(a + PI2 - va, offset).sum(line.b))
    );
    return vl;
  };

  /* --- concatenated 'lib/geom/sweepLine.js' --- */
  SweepLineClass._LO = false;
  SweepLineClass._HI = true;

  SweepLineClass.NodeClass = class {
    constructor(sweepLine, object, loHi, x) {
      this.object = object;
      this.parent = sweepLine;
      this.loHi = loHi;
      this.x = x;
      this.prev = null;
      this.next = null;
      if(sweepLine.queueHead) sweepLine.queueHead.prev = this;
      this.next = sweepLine.queueHead;
      sweepLine.queueHead = this;
      sweepLine.sortNode(this);
    }
  };

  /* --- concatenated 'lib/geom/transformation.js' --- */
  const RAD2DEG = 180 / Math.PI;
  const DEG2RAD = Math.PI / 180;

  Object.defineProperty(Transformation, Symbol.hasInstance, {
    value(inst) {
      return [
        Transformation,
        MatrixTransformation,
        Rotation,
        Translation,
        Scaling,
        TransformationList
      ].some(ctor => Object.getPrototypeOf(inst) == ctor.prototype);
    }
  });
  const ImmutableTransformation = Util.immutableClass(Transformation);
  const ImmutableRotation = Util.immutableClass(Rotation);
  const ImmutableTranslation = Util.immutableClass(Translation);
  const ImmutableScaling = Util.immutableClass(Scaling);
  const ImmutableMatrixTransformation = Util.immutableClass(MatrixTransformation
  );
  const {
    concat,
    copyWithin,
    find,
    findIndex,
    lastIndexOf,
    pop,
    push,
    shift,
    unshift,
    slice,
    splice,
    includes,
    indexOf,
    entries,
    filter,
    map,
    every,
    some,
    reduce,
    reduceRight
  } = Array.prototype;

  Util.inherit(TransformationList.prototype, {
      // concat,

      copyWithin,
      find,
      findIndex,
      lastIndexOf,
      pop,
      shift, //   slice,

      //splice,

      includes,
      indexOf,
      entries, //  filter,

      //  map,

      every,
      some,
      reduce,
      reduceRight
    },
    {
      [Symbol.iterator]() {
        return Array.prototype[Symbol.iterator];
      }, [Symbol.isConcatSpreadable]() {
        return true;
      }
    }
  );

  //Object.setPrototypeOf(TransformationList.prototype, Transformation.prototype);

  const ImmutableTransformationList = Util.immutableClass(TransformationList);
  Util.defineGetter(ImmutableTransformationList,
    Symbol.species,
    () => ImmutableTransformationList
  );

  /* --- concatenated 'lib/geom/simplify.js' --- */
  /**
   * square distance between 2 points
   * @param {[number, number]} p1
   * @param {[number, number]} p2
   * @returns {number}
   */
  function getSqDist(p1, p2) {
    let dx = p1[0] - p2[0],
      dy = p1[1] - p2[1];
    return dx * dx + dy * dy;
  }

  /**
   * square distance from a point to a segment
   *
   * @param {[number, number]} p
   * @param {[number, number]} p1
   * @param {[number, number]} p2
   * @returns {number}
   */
  function getSqSegDist(p, p1, p2) {
    let x = p1[0],
      y = p1[1],
      dx = p2[0] - x,
      dy = p2[1] - y,
      t;

    if(dx !== 0 || dy !== 0) {
      t = ((p[0] - x) * dx + (p[1] - y) * dy) / (dx * dx + dy * dy);

      if(t > 1) {
        x = p2[0];
        y = p2[1];
      } else if(t > 0) {
        x += dx * t;
        y += dy * t;
      }
    }

    dx = p[0] - x;
    dy = p[1] - y;
    return dx * dx + dy * dy;
  }

  Util.weakAssign(globalObj, { Align });
  Util.weakAssign(globalObj, { Util });
  Util.weakAssign(globalObj, { BBox });
  Util.weakAssign(globalObj, { Graph });
  Util.weakAssign(globalObj, { Point });
  Util.weakAssign(globalObj, { Line });
  Util.weakAssign(globalObj, { Intersection });
  Util.weakAssign(globalObj, { Node });
  Util.weakAssign(globalObj, { Size });
  Util.weakAssign(globalObj, { Rect });
  Util.weakAssign(globalObj, { TRBL, isTRBL });
  Util.weakAssign(globalObj, { iterator, eventIterator });
  Util.weakAssign(globalObj, { Element, isElement });
  Util.weakAssign(globalObj, { PointList, Polyline });
  Util.weakAssign(globalObj, { LineList });
  Util.weakAssign(globalObj, { Matrix });
  Util.weakAssign(globalObj, { Circle });
  Util.weakAssign(globalObj, { PolygonFinder });
  Util.weakAssign(globalObj, { Polygon });
  Util.weakAssign(globalObj, { SweepLineClass });
  Util.weakAssign(globalObj, {
    Transformation,
    Rotation,
    Translation,
    Scaling,
    MatrixTransformation,
    TransformationList
  });
  Util.weakAssign(globalObj, { Vector });
  Util.weakAssign(globalObj, {
    simplifyRadialDist,
    simplifyDPStep,
    simplifyDouglasPeucker,
    simplify
  });
  Util.weakAssign(globalObj, {});
})(window);

/* jshint ignore:end */
