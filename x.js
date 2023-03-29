class PmDeviceInfo_S extends ArrayBuffer {
  constructor(obj = {}) {
    super(40);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() {
    return `[PmDeviceInfo_S @ ${this} ]`;
  }

  /* 0: int structVersion */
  set structVersion(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 0)[0] = value;
  }
  get structVersion() {
    return new Int32Array(this, 0)[0];
  }

  /* 8: const char* interf */
  set interf(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }
  get interf() {
    return '0x' + new BigInt64Array(this, 8)[0].toString(16);
  }

  /* 16: char* name */
  /* undefined 1 true */
  set name(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }
  get name() {
    return '0x' + new BigInt64Array(this, 16)[0].toString(16);
  }

  /* 24: int input */
  set input(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 24)[0] = value;
  }
  get input() {
    return new Int32Array(this, 24)[0];
  }

  /* 28: int output */
  set output(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 28)[0] = value;
  }
  get output() {
    return new Int32Array(this, 28)[0];
  }

  /* 32: int opened */
  set opened(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 32)[0] = value;
  }
  get opened() {
    return new Int32Array(this, 32)[0];
  }

  /* 36: int is_virtual */
  set is_virtual(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 36)[0] = value;
  }
  get is_virtual() {
    return new Int32Array(this, 36)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(address, 40);
    return Object.setPrototypeOf(ret, PmDeviceInfo_S.prototype);
  }

  toString() {
    const { structVersion, interf, name, input, output, opened, is_virtual } = this;
    return `PmDeviceInfo_S {\n\t.structVersion = ${structVersion},\n\t.interf = 0x${interf.toString(16)},\n\t.name = 0x${name.toString(
      16
    )},\n\t.input = ${input},\n\t.output = ${output},\n\t.opened = ${opened},\n\t.is_virtual = ${is_virtual}\n}`;
  }
}
