import Util from './lib/util.js';

export class Param {
  [Symbol.toPrimitive](hint) {
    //console.log(`Param[Symbol.toPrimitive](${hint})`);
    if(hint == 'number') return NumericParam.prototype.get.call(this);
    else if(hint == 'string') return this.valueOf() + '';
    else return this.valueOf();
  }

  valueOf() {
    return this.get();
  }

  [Symbol.toStringTag]() {
    return this.toString();
  }

  toString() {
    return '' + this.valueOf();
  }
}

export class NumericParam extends Param {
  constructor(value = 0, min = 0, max = 1) {
    super();
    Object.assign(this, { min, max });
    Util.define(this, { alpha: (value - min) / (max - min) });
  }

  get() {
    const { min, max, alpha } = this;
    return min + alpha * (max - min);
  }

  set(num) {
    const { min, max } = this;
    Util.define(this, { alpha: (num - min) / (max - min) });
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
