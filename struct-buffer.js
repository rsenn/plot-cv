class buffer extends ArrayBuffer {
  constructor(obj = {}) {
    super(64);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() {
    return `[struct buffer @ ${this} ]`;
  }

  /* 0: char* x */
  /* undefined 1 true */
  set x(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 0)[0] = BigInt(value);
  }
  get x() {
    return '0x' + new BigInt64Array(this, 0)[0].toString(16);
  }

  /* 8: size_t (unsigned long) p */
  set p(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }
  get p() {
    return new BigInt64Array(this, 8)[0];
  }

  /* 16: size_t (unsigned long) n */
  set n(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }
  get n() {
    return new BigInt64Array(this, 16)[0];
  }

  /* 24: size_t (unsigned long) a */
  set a(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 24)[0] = BigInt(value);
  }
  get a() {
    return new BigInt64Array(this, 24)[0];
  }

  /* 32: buffer_op_proto* op */
  /* buffer_op_proto 8 true */
  set op(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 32)[0] = BigInt(value);
  }
  get op() {
    return '0x' + new BigInt64Array(this, 32)[0].toString(16);
  }

  /* 40: void* cookie */
  /* undefined 0 true */
  set cookie(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 40)[0] = BigInt(value);
  }
  get cookie() {
    return '0x' + new BigInt64Array(this, 40)[0].toString(16);
  }

  /* 48: void (*)() deinit */
  set deinit(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 48)[0] = BigInt(value);
  }
  get deinit() {
    return '0x' + new BigInt64Array(this, 48)[0].toString(16);
  }

  /* 56: fd_t (int) fd */
  set fd(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 56)[0] = value;
  }
  get fd() {
    return new Int32Array(this, 56)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(address, 60);
    return Object.setPrototypeOf(ret, buffer.prototype);
  }

  toString() {
    const { x, p, n, a, op, cookie, deinit, fd } = this;
    return `struct buffer {\n\t.x = 0x${x.toString(16)},\n\t.p = ${p},\n\t.n = ${n},\n\t.a = ${a},\n\t.op = 0x${op.toString(16)},\n\t.cookie = 0x${cookie.toString(16)},\n\t.deinit = 0x${deinit.toString(16)},\n\t.fd = ${fd}\n}`;
  }
}
