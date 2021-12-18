class inotify_event extends ArrayBuffer {
  constructor(obj = {}) {
    super(24);
    Object.assign(this, obj);
  }

  /* 0: int wd */
  set wd(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 0)[0] = value;
  }
  get wd() {
    return new Int32Array(this, 0)[0];
  }

  /* 4: uint32_t (unsigned int) mask */
  set mask(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Uint32Array(this, 4)[0] = value;
  }
  get mask() {
    return new Uint32Array(this, 4)[0];
  }

  /* 8: uint32_t (unsigned int) cookie */
  set cookie(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Uint32Array(this, 8)[0] = value;
  }
  get cookie() {
    return new Uint32Array(this, 8)[0];
  }

  /* 12: uint32_t (unsigned int) len */
  set len(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Uint32Array(this, 12)[0] = value;
  }
  get len() {
    return new Uint32Array(this, 12)[0];
  }

  /* 16: char [] name */
  set name(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int8Array(this, 16)[0] = value;
  }
  get name() {
    return new Int8Array(this, 16)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(address, 20);
    return Object.setPrototypeOf(ret, inotify_event.prototype);
  }

  toString() {
    const { wd, mask, cookie, len, name } = this;
    return `inotify_event {\n\t.wd = ${wd},\n\t.mask = ${mask},\n\t.cookie = ${cookie},\n\t.len = ${len},\n\t.name = ${name}\n}`;
  }
}

inotify_event.prototype[Symbol.toStringTag] = 'inotify_event';
