import { dlopen, define, dlerror, dlclose, dlsym, call, errno, RTLD_NOW } from 'ffi';

class PaStreamParameters extends ArrayBuffer {
  constructor(obj = {}) {
    super(32);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() {
    return `[PaStreamParameters @ ${this} ]`;
  }

  /* 0: PaDeviceIndex (int) device */
  set device(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 0)[0] = value;
  }
  get device() {
    return new Int32Array(this, 0)[0];
  }

  /* 4: int channelCount */
  set channelCount(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 4)[0] = value;
  }
  get channelCount() {
    return new Int32Array(this, 4)[0];
  }

  /* 8: PaSampleFormat (unsigned long) sampleFormat */
  set sampleFormat(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }
  get sampleFormat() {
    return new BigInt64Array(this, 8)[0];
  }

  /* 16: PaTime (double) suggestedLatency */
  set suggestedLatency(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }
  get suggestedLatency() {
    return new BigInt64Array(this, 16)[0];
  }

  /* 24: void* hostApiSpecificStreamInfo */
  /* undefined 0 true */
  set hostApiSpecificStreamInfo(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 24)[0] = BigInt(value);
  }
  get hostApiSpecificStreamInfo() {
    return '0x' + new BigInt64Array(this, 24)[0].toString(16);
  }

  static from(address) {
    let ret = toArrayBuffer(address, 32);
    return Object.setPrototypeOf(ret, PaStreamParameters.prototype);
  }

  toString() {
    const { device, channelCount, sampleFormat, suggestedLatency, hostApiSpecificStreamInfo } = this;
    return `PaStreamParameters {\n\t.device = ${device},\n\t.channelCount = ${channelCount},\n\t.sampleFormat = ${sampleFormat},\n\t.suggestedLatency = ${suggestedLatency},\n\t.hostApiSpecificStreamInfo = 0x${hostApiSpecificStreamInfo.toString(
      16
    )}\n}`;
  }
}
