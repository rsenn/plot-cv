import { define, mod, roundTo } from 'util';
import { EventEmitter } from './quickjs/qjs-modules/lib/events.js';
import * as cv from 'opencv';

const MinMax = (min, max) => value => Math.max(min, Math.min(max, value));

export class Param extends EventEmitter {
  valueOf() {
    if(typeof this.callback == "function") this.callback.call(this, this);

    return this.get();
  }

  /*[Symbol.toStringTag]() {
    return this.toString();
  }*/

  toString() {
    return "" + this.valueOf();
  }

  createTrackbar(name, win) {
    cv.createTrackbar(name, win + "", this.value, this.max, value => {
      let old = this.get();
      this.set(value);
      if(value != old) this.emit("change", value, old);
    });
  }
}

define(Param.prototype, { get [Symbol.toStringTag]() { return this.toString(); } });

export class NumericParam extends Param {
  constructor(value = 0, min = 0, max = 1, step = 1) {
    super();
    const clamp = MinMax(min, max);
    this.value = clamp(value);
    Object.assign(this, { default: value, min, max });
    define(this, { step, clamp });
  }

  get() {
    return this.value;
  }

  set(value) {
    const { clamp, min, step } = this;
    let newValue = this.clamp(min + roundTo(value - min, step, null, "floor"));
    //console.log(`Param.set oldValue=${this.value} new=${newValue}`);
    this.value = newValue;
  }

  /* prettier-ignore */ get alpha() {
    const { value, min, max } = this;
    return (value - min) / (max - min);
  }

  /* prettier-ignore */ set alpha(a) {
    const { min, max, step, clamp } = this;
    this.value = this.clamp(min + roundTo((max - min) * a, step, null, 'floor'));
  }

  /* prettier-ignore */ get range() {
    const { min, max } = this;
    let r = [min, max];
    r.valueOf = function() {
      return this[1] - this[0];
    };
    return r;
  }

  increment() {
    let value = NumericParam.prototype.get.call(this);

    NumericParam.prototype.set.call(this, value + this.step);
  }

  decrement() {
    let value = NumericParam.prototype.get.call(this);
    NumericParam.prototype.set.call(this, value - this.step);
  }

  reset() {
    NumericParam.prototype.set.call(this, this.default);
  }
}

export class EnumParam extends NumericParam {
  constructor(...args) {
    let values, init;
    if(Array.isArray(args[0])) {
      values = args[0];
      init = args[1] || 0;
    } else {
      init = args.shift();
      values = Array.isArray(args[0]) ? args[0] : args;
    }
    super(init, 0, values.length - 1);
    define(this, { values });
  }

  get() {
    return this.values[Math.floor(super.get())];
  }

  set(newVal) {
    let i;
    if(typeof newVal == "number") i = newVal;
    else if((i = this.values.indexOf(newVal)) == -1) throw new Error(`No such value '${newVal}' in [${this.values}]`);
    super.set(i);
  }
}

export function ParamNavigator(map, index = 0) {
  if(!new.target) return new ParamNavigator(map);

  console.log("ParamNavigator", map);
  if(!(map instanceof Map)) map = new Map(Object.entries(map));

  // console.log('map:', map);

  define(this, {
    map,
    index,
    next() {
      this.index = mod(this.index + 1, map.size);
      // console.log('ParamNavigator index =', this.index);
    },
    prev() {
      this.index = mod(this.index - 1, map.size);
      // console.log('ParamNavigator index =', this.index);
    },
    setCallback(fn, thisObj) {
      const { map } = this;
      thisObj ??= this;

      for(let [name, param] of map) param.callback = () => fn.call(thisObj, name, param);
      return fn;
    }
  });
}

define(ParamNavigator.prototype, {
  nameOf(param) {
    for(let [name, value] of this.map) if(value === param) return name;
  }
});

define(ParamNavigator.prototype, {
  at(index) {
    const { map } = this;
    return [...map.entries()][index];
  },
  indexOf(param) {
    const { map } = this;
    let i = 0;
    for(let [name, value] of map) {
      if(param === name || param === value) return i;
      i++;
    }
    return -1;
  },
  nameOf(param) {
    const { map } = this;
    let i = 0;
    for(let [name, value] of map) {
      if(param === i || param === value) return name;
      i++;
    }
    return -1;
  },
  get(name) {
    const { map } = this;
    return map.get(name);
  },
  /* prettier-ignore */ get current() {
    return this.at(this.index);
  },
  /* prettier-ignore */ set current(index_or_name) {
if(typeof index_or_name != 'number')
  index_or_name = this.indexOf(index_or_name);
if(index_or_name >= 0 && index_or_name < this.map.size)
  this.index = index_or_name;
  },
  /* prettier-ignore */ get name() {
    const { map, index } = this;
    return [...map.keys()][index];
  },
  /* prettier-ignore */ get param() {
    const { map, index } = this;
    return [...map.values()][index];
  },
  /* prettier-ignore */ get size() {
    const { map } = this;
    return map.size;
  }
});