class Elf64_Ehdr extends ArrayBuffer {
  constructor(obj = {}) {
    super(72);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() {
    return `[Elf64_Ehdr @ ${this} ]`;
  }

  /* 0: unsigned char [16] e_ident */
  set e_ident(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Uint8Array(this, 0)[0] = value;
  }
  get e_ident() {
    return new Uint8Array(this, 0)[0];
  }

  /* 4: Elf64_Half (unsigned short) e_type */
  set e_type(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 4)[0] = value;
  }
  get e_type() {
    return new Int16Array(this, 4)[0];
  }

  /* 8: Elf64_Half (unsigned short) e_machine */
  set e_machine(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 8)[0] = value;
  }
  get e_machine() {
    return new Int16Array(this, 8)[0];
  }

  /* 12: Elf64_Word (unsigned int) e_version */
  set e_version(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int32Array(this, 12)[0] = value;
  }
  get e_version() {
    return new Int32Array(this, 12)[0];
  }

  /* 16: Elf64_Addr (unsigned long) e_entry */
  set e_entry(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }
  get e_entry() {
    return new BigInt64Array(this, 16)[0];
  }

  /* 24: Elf64_Off (unsigned long) e_phoff */
  set e_phoff(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new BigInt64Array(this, 24)[0] = BigInt(value);
  }
  get e_phoff() {
    return new BigInt64Array(this, 24)[0];
  }

  /* 32: Elf64_Off (unsigned long) e_shoff */
  set e_shoff(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new BigInt64Array(this, 32)[0] = BigInt(value);
  }
  get e_shoff() {
    return new BigInt64Array(this, 32)[0];
  }

  /* 40: Elf64_Word (unsigned int) e_flags */
  set e_flags(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int32Array(this, 40)[0] = value;
  }
  get e_flags() {
    return new Int32Array(this, 40)[0];
  }

  /* 44: Elf64_Half (unsigned short) e_ehsize */
  set e_ehsize(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 44)[0] = value;
  }
  get e_ehsize() {
    return new Int16Array(this, 44)[0];
  }

  /* 48: Elf64_Half (unsigned short) e_phentsize */
  set e_phentsize(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 48)[0] = value;
  }
  get e_phentsize() {
    return new Int16Array(this, 48)[0];
  }

  /* 52: Elf64_Half (unsigned short) e_phnum */
  set e_phnum(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 52)[0] = value;
  }
  get e_phnum() {
    return new Int16Array(this, 52)[0];
  }

  /* 56: Elf64_Half (unsigned short) e_shentsize */
  set e_shentsize(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 56)[0] = value;
  }
  get e_shentsize() {
    return new Int16Array(this, 56)[0];
  }

  /* 60: Elf64_Half (unsigned short) e_shnum */
  set e_shnum(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 60)[0] = value;
  }
  get e_shnum() {
    return new Int16Array(this, 60)[0];
  }

  /* 64: Elf64_Half (unsigned short) e_shstrndx */
  set e_shstrndx(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer)
      value = toPointer(value);
    new Int16Array(this, 64)[0] = value;
  }
  get e_shstrndx() {
    return new Int16Array(this, 64)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(address, 68);
    return Object.setPrototypeOf(ret, Elf64_Ehdr.prototype);
  }

  toString() {
    const {
      e_ident,
      e_type,
      e_machine,
      e_version,
      e_entry,
      e_phoff,
      e_shoff,
      e_flags,
      e_ehsize,
      e_phentsize,
      e_phnum,
      e_shentsize,
      e_shnum,
      e_shstrndx
    } = this;
    return `Elf64_Ehdr {\n\t.e_ident = ${e_ident},\n\t.e_type = ${e_type},\n\t.e_machine = ${e_machine},\n\t.e_version = ${e_version},\n\t.e_entry = ${e_entry},\n\t.e_phoff = ${e_phoff},\n\t.e_shoff = ${e_shoff},\n\t.e_flags = ${e_flags},\n\t.e_ehsize = ${e_ehsize},\n\t.e_phentsize = ${e_phentsize},\n\t.e_phnum = ${e_phnum},\n\t.e_shentsize = ${e_shentsize},\n\t.e_shnum = ${e_shnum},\n\t.e_shstrndx = ${e_shstrndx}\n}`;
  }
}
