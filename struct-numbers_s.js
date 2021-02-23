class numbers_s extends ArrayBuffer {
  constructor(obj = {}) {
    super(16);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() { return `[struct numbers_s @ ${this} ]`; }

  /* 0: float fl@4 */
  set fl(value) { new Float32Array(this, 0)[0] = value; }
  get fl() { return new Float32Array(this, 0)[0]; }

  /* 4: short sh@2 */
  set sh(value) { new Int16Array(this, 4)[0] = value; }
  get sh() { return new Int16Array(this, 4)[0]; }

  /* 8: double db@8 */
  set db(value) { new Float64Array(this, 8)[0] = value; }
  get db() { return new Float64Array(this, 8)[0]; }

  toString() {
    const { fl, sh, db } = this;
    return `struct numbers_s {\n\t.fl = ${fl},\n\t.sh = ${sh},\n\t.db = ${db}\n}`;
  }
}
