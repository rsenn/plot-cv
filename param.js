import Util from './lib/util.js';
import { Repeater } from './lib/repeater/repeater.js';

const MinMax = (min, max) => value => Math.max(min, Math.min(max, value));

export class Param {
  /*[Symbol.toPrimitive](hint) {
  //console.log(`Param[Symbol.toPrimitive](${hint})`);
    if(hint == 'number') return NumericParam.prototype.get.call(this);
    else if(hint == 'string') return Param.prototype.valueOf.call(this) + '';
    else return Param.prototype.valueOf.call(this);
  }*/

  valueOf() {
    return this.get(); //NumericParam.prototype.get.call(this);
  }

  /*  toPrimitive() {
    return this.valueOf();
  }*/

  [Symbol.toStringTag]() {
    return this.toString();
  }

  toString() {
    return '' + this.valueOf();
  }

  async createTrackbar(name, win) {
    const cv = await import('cv.so');

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

  //console.log('map:', map);

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
  get current() {
    return this.at(this.index);
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
