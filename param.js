import Util from './lib/util.js';
import * as cv from 'opencv';

const MinMax = (min, max) => value => Math.max(min, Math.min(max, value));

export class Param {
  valueOf() {
    if(typeof this.callback == 'function') this.callback.call(this, this);

    return this.get();
  }

  [Symbol.toStringTag]() {
    return this.toString();
  }

  toString() {
    return '' + this.valueOf();
  }

  createTrackbar(name, win) {
    cv.createTrackbar(name, win + '', this.value, this.max, value => this.set(value));
  }
}

export class NumericParam extends Param {
  constructor(value = 0, min = 0, max = 1, step = 1) {
    super();
    const clamp = MinMax(min, max);
    this.value = clamp(value);
    Object.assign(this, { default: value, min, max });
    Util.define(this, { step, clamp });
  }

  get() {
    return this.value;
  }

  set(value) {
    const { clamp, min, step } = this;
    let newValue = this.clamp(min + Util.roundTo(value - min, step, null, 'floor'));
    console.log(`Param.set oldValue=${this.value} new=${newValue}`);
    this.value = newValue;
  }

  get alpha() {
    const { value, min, max } = this;
    return (value - min) / (max - min);
  }

  set alpha(a) {
    const { min, max, step, clamp } = this;
    this.value = this.clamp(min + Util.roundTo((max - min) * a, step, null, 'floor'));
  }

  get range() {
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
    if(Util.isArray(args[0])) {
      values = args[0];
      init = args[1] || 0;
    } else {
      init = args.shift();
      values = Util.isArray(args[0]) ? args[0] : args;
    }
    super(init, 0, values.length - 1);
    Util.define(this, { values });
  }

  get() {
    return this.values[Math.floor(super.get())];
  }

  set(newVal) {
    let i;
    if(typeof newVal == 'number') i = newVal;
    else if((i = this.values.indexOf(newVal)) == -1)
      throw new Error(`No such value '${newVal}' in [${this.values}]`);
    super.set(i);
  }
}

export function ParamNavigator(map, index = 0) {
  if(!new.target) return new ParamNavigator(map);

  if(!(map instanceof Map)) map = Util.toMap(map);

  const mod = Util.mod(map.size);

  // console.log('map:', map);

  Util.define(this, {
    map,
    index,
    next() {
      this.index = mod(this.index + 1);
      // console.log('ParamNavigator index =', this.index);
    },
    prev() {
      this.index = mod(this.index - 1);
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
Util.define(ParamNavigator.prototype, {
  nameOf(param) {
    for(let [name, value] of this.map) if(value === param) return name;
  }
});

Util.define(ParamNavigator.prototype, {
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
  get current() {
    return this.at(this.index);
  },
  set current(index_or_name) {
if(typeof index_or_name != 'number')
  index_or_name = this.indexOf(index_or_name);
if(index_or_name >= 0 && index_or_name < this.map.size)
  this.index = index_or_name;
  },
  get name() {
    const { map, index } = this;
    return [...map.keys()][index];
  },
  get param() {
    const { map, index } = this;
    return [...map.values()][index];
  },
  get size() {
    const { map } = this;
    return map.size;
  }
});
