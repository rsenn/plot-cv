import Util from './lib/util.js';

export class Param {
  [Symbol.toPrimitive](hint) {
    //console.log(`Param[Symbol.toPrimitive](${hint})`);
    if(hint == 'number') return NumericParam.prototype.get.call(this);
    else if(hint == 'string') return this.valueOf() + '';
    else return this.valueOf();
  }

  valueOf() {
    return NumericParam.prototype.get.call(this);
  }

  [Symbol.toStringTag]() {
    return this.toString();
  }

  toString() {
    return '' + this.valueOf();
  }
}

export class NumericParam extends Param {
  constructor(value = 0, min = 0, max = 1, step = 1) {
    super();
    Object.assign(this, { min, max, step });
    Util.define(this, { value /*alpha: (value - min) / (max - min)*/ });
  }

  get() {
    return this.value;
    /*const { min, max, alpha } = this;
    return min + alpha * (max - min);*/
  }

  set(value) {
    //const { min, max } = this;
    this.value = Util.roundTo(value, this.step);
  }

  get alpha() {
    const { value, min, max } = this;
    return (value - min) / (max - min);
  }

  set alpha(a) {
    const { min, max, step } = this;
    this.value = Util.roundTo(min + (max - min) * a, step);
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
}

export class EnumParam extends NumericParam {
  constructor(...args) {
    let values, init;
    if(Util.isArray(args[0])) {
      values = args[0];
      init = args[1] || 0;
    } else {
      values = args;
      init = 0;
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
    else if((i = this.values.indexOf(newVal)) == -1) throw new Error(`No such value '${newVal}' in [${this.values}]`);
    super.set(i);
  }
}

export function ParamNavigator(map) {
  if(!new.target) return new ParamNavigator(map);

  if(!(map instanceof Map)) map = Util.toMap(map);

  const mod = Util.mod(map.size);

  console.log('map:', map);

  Util.define(this, {
    map,
    index: 0,
    next() {
      this.index = mod(this.index + 1);
      console.log('ParamNavigator index =', this.index);
    },
    prev() {
      this.index = mod(this.index - 1);
      console.log('ParamNavigator index =', this.index);
    }
  });
}

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
  } /*,
  get(name) {
    return this.map.get(name).get();
  },
  set(name, value) {
    return this.map.get(name).set(value);
  },
  get names() {
    return [...this.map.keys()];
  },
  get values() {
    let ret = {};
    for(let [name, param] of this.map) {
      ret[name] = +param;
    }
    return ret;
  }*/
});
