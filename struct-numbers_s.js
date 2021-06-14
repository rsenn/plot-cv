class numbers_s extends ArrayBuffer {
  constructor(obj = {}) {
    super(16);
    Object.assign(this, obj);
  }
  /* prettier-ignore */ get [Symbol.toStringTag]() { return `[struct numbers_s @ ${this} ]`; }

  /* 0: float fl@4 */
  /* prettier-ignore */ set fl(value) { new Float32Array(this, 0)[0] = value; }
  /* prettier-ignore */ get fl() { return new Float32Array(this, 0)[0]; }

  /* 4: short sh@2 */
  /* prettier-ignore */ set sh(value) { new Int16Array(this, 4)[0] = value; }
  /* prettier-ignore */ get sh() { return new Int16Array(this, 4)[0]; }

  /* 8: double db@8 */
  /* prettier-ignore */ set db(value) { new Float64Array(this, 8)[0] = value; }
  /* prettier-ignore */ get db() { return new Float64Array(this, 8)[0]; }

  toString() {
    const { fl, sh, db } = this;
    return `struct numbers_s {\n\t.fl = ${fl},\n\t.sh = ${sh},\n\t.db = ${db}\n}`;
  }
}
