import { dlopen, dlsym, RTLD_NOW, define, call, toArrayBuffer, toString, toPointer } from 'ffi';
export { dlopen, dlsym, RTLD_NOW, define, call, toArrayBuffer, toString, toPointer } from 'ffi';

const libportmidi = dlopen('libportmidi.so.0', RTLD_NOW);

export function PmError(n) {
  return { [0]: 'pmNoError', [1]: 'pmGotData', [-10000]: 'pmHostError', [-9999]: 'pmInvalidDeviceId', [-9998]: 'pmInsufficientMemory', [-9997]: 'pmBufferTooSmall', [-9996]: 'pmBufferOverflow', [-9995]: 'pmBadPtr', [-9994]: 'pmBadData', [-9993]: 'pmInternalError', [-9992]: 'pmBufferMaxSize', [-9991]: 'pmNotImplemented', [-9990]: 'pmInterfaceNotSupported', [-9989]: 'pmNameConflict' }[n];
}

export function Pm_Message(status, data1, data2) {
  return ((data2 << 16) & 0xff0000) | ((data1 << 8) & 0xff00) | (status & 0xff);
}

Object.assign(PmError, {
  pmNoError: 0,
  pmNoData: 0,
  pmGotData: 1,
  pmHostError: -10000,
  pmInvalidDeviceId: -9999,
  pmInsufficientMemory: -9998,
  pmBufferTooSmall: -9997,
  pmBufferOverflow: -9996,
  pmBadPtr: -9995,
  pmBadData: -9994,
  pmInternalError: -9993,
  pmBufferMaxSize: -9992,
  pmNotImplemented: -9991,
  pmInterfaceNotSupported: -9990,
  pmNameConflict: -9989
});

export class PmDeviceInfo extends ArrayBuffer {
  constructor(obj = {}) {
    super(40);
    Object.assign(this, obj);
  }

  /* 0: int structVersion */
  set structVersion(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 0)[0] = value;
  }
  get structVersion() {
    return '0x' + new Uint32Array(this, 0)[0].toString(16);
  }

  /* 8: const char* interf */
  /* set interf(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }*/
  get interf() {
    return toString('0x' + new BigInt64Array(this, 8)[0].toString(16));
  }

  /* 16: char* name */
  /* undefined 1 true */
  /*set name(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }*/
  get name() {
    return toString('0x' + new BigInt64Array(this, 16)[0].toString(16));
  }

  /* 24: int input */
  set input(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 24)[0] = value;
  }
  get input() {
    return Boolean(new Int32Array(this, 24)[0]);
  }

  /* 28: int output */
  set output(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 28)[0] = value;
  }
  get output() {
    return Boolean(new Int32Array(this, 28)[0]);
  }

  /* 32: int opened */
  set opened(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 32)[0] = value;
  }
  get opened() {
    return Boolean(new Int32Array(this, 32)[0]);
  }

  /* 36: int is_virtual */
  set is_virtual(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new Int32Array(this, 36)[0] = value;
  }
  get is_virtual() {
    return Boolean(new Int32Array(this, 36)[0]);
  }

  static from(address) {
    let ret = toArrayBuffer(address, 40);
    return Object.setPrototypeOf(ret, PmDeviceInfo.prototype);
  }

  toString() {
    const { structVersion, interf, name, input, output, opened, is_virtual } = this;
    return `PmDeviceInfo {\n\t.structVersion = ${structVersion},\n\t.interf = 0x${interf.toString(16)},\n\t.name = 0x${name.toString(16)},\n\t.input = ${input},\n\t.output = ${output},\n\t.opened = ${opened},\n\t.is_virtual = ${is_virtual}\n}`;
  }

  [Symbol.inspect]() {
    const { structVersion, interf, name, input, output, opened } = this;
    return `PmDeviceInfo ` + inspect({ structVersion, interf, name, input, output, opened });
  }
}

//delete PmDeviceInfo.prototype[Symbol.toStringTag];
//PmDeviceInfo.prototype[Symbol.toStringTag] = 'PmDeviceInfo';

/**
 * @function Pm_Initialize
 *
 * @return   {Number}
 */
define('Pm_Initialize', dlsym(libportmidi, 'Pm_Initialize'), null, 'int');
export function Pm_Initialize() {
  return call('Pm_Initialize');
}

/**
 * @function Pm_Terminate
 *
 * @return   {Number}
 */
define('Pm_Terminate', dlsym(libportmidi, 'Pm_Terminate'), null, 'int');
export function Pm_Terminate() {
  return call('Pm_Terminate');
}

/**
 * @function Pm_HasHostError
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pm_HasHostError', dlsym(libportmidi, 'Pm_HasHostError'), null, 'int', 'void *');
export function Pm_HasHostError(stream) {
  return call('Pm_HasHostError', stream);
}

/**
 * @function Pm_GetErrorText
 *
 * @param    {Number}        errnum
 *
 * @return   {String}
 */
define('Pm_GetErrorText', dlsym(libportmidi, 'Pm_GetErrorText'), null, 'char *', 'int');
export function Pm_GetErrorText(errnum) {
  return call('Pm_GetErrorText', errnum);
}

/**
 * @function Pm_GetHostErrorText
 *
 * @param    {String}        msg
 * @param    {Number}        len
 */
define('Pm_GetHostErrorText', dlsym(libportmidi, 'Pm_GetHostErrorText'), null, 'void', 'char *', 'unsigned int');
export function Pm_GetHostErrorText(msg, len) {
  call('Pm_GetHostErrorText', msg, len);
}

/**
 * @function Pm_CountDevices
 *
 * @return   {Number}
 */
define('Pm_CountDevices', dlsym(libportmidi, 'Pm_CountDevices'), null, 'int');
export function Pm_CountDevices() {
  return call('Pm_CountDevices');
}

/**
 * @function Pm_GetDefaultInputDeviceID
 *
 * @return   {Number}
 */
define('Pm_GetDefaultInputDeviceID', dlsym(libportmidi, 'Pm_GetDefaultInputDeviceID'), null, 'int');
export function Pm_GetDefaultInputDeviceID() {
  return call('Pm_GetDefaultInputDeviceID');
}

/**
 * @function Pm_GetDefaultOutputDeviceID
 *
 * @return   {Number}
 */
define('Pm_GetDefaultOutputDeviceID', dlsym(libportmidi, 'Pm_GetDefaultOutputDeviceID'), null, 'int');
export function Pm_GetDefaultOutputDeviceID() {
  return call('Pm_GetDefaultOutputDeviceID');
}

/**
 * @function Pm_GetDeviceInfo
 *
 * @param    {Number}        id
 *
 * @return   {Number}
 */
define('Pm_GetDeviceInfo', dlsym(libportmidi, 'Pm_GetDeviceInfo'), null, 'buffer', 'int');
export function Pm_GetDeviceInfo(id) {
  const ptr = call('Pm_GetDeviceInfo', id);

  return Object.setPrototypeOf(toArrayBuffer(ptr, 40), PmDeviceInfo.prototype);
}

/**
 * @function Pm_OpenInput
 *
 * @param    {Number}        stream
 * @param    {Number}        inputDevice
 * @param    {Number}        inputDriverInfo
 * @param    {Number}        bufferSize
 * @param    {Number}        time_proc
 * @param    {Number}        time_info
 *
 * @return   {Number}
 */
define('Pm_OpenInput', dlsym(libportmidi, 'Pm_OpenInput'), null, 'int', 'void *', 'int', 'void *', 'int', 'int', 'void *');
export function Pm_OpenInput(stream, inputDevice, inputDriverInfo = null, bufferSize = 0, time_proc = null, time_info = null) {
  let streamPtr = new BigUint64Array(1);
  let ret = call('Pm_OpenInput', streamPtr.buffer, inputDevice, inputDriverInfo, bufferSize, time_proc, time_info);
  let ptr = Number(streamPtr[0]);
  let buf = toArrayBuffer(ptr, 48);
  if(typeof stream == 'function') stream(buf, ptr);
  else if('length' in stream) stream.splice(0, stream.length, buf, ptr);
  else throw new TypeError(`Pm_OpenInput argument 1 must be function or array`);

  return ret;
}

/**
 * @function Pm_OpenOutput
 *
 * @param    {Number}        stream
 * @param    {Number}        outputDevice
 * @param    {Number}        outputDriverInfo
 * @param    {Number}        bufferSize
 * @param    {Number}        time_proc
 * @param    {Number}        time_info
 * @param    {Number}        latency
 *
 * @return   {Number}
 */
define('Pm_OpenOutput', dlsym(libportmidi, 'Pm_OpenOutput'), null, 'int', 'void *', 'int', 'void *', 'int', 'int', 'void *', 'int');
export function Pm_OpenOutput(stream, outputDevice, outputDriverInfo = null, bufferSize = 0, time_proc = null, time_info = null, latency = 0) {
  let streamPtr = new BigUint64Array(1);
  let ret = call('Pm_OpenOutput', streamPtr.buffer, outputDevice, outputDriverInfo, bufferSize, time_proc, time_info, latency);
  let ptr = Number(streamPtr[0]);
  let buf = toArrayBuffer(ptr, 48);
  if(typeof stream == 'function') stream(buf, ptr);
  else if('length' in stream) stream.splice(0, stream.length, buf, ptr);
  else throw new TypeError(`Pm_OpenOutput argument 1 must be function or array`);
  return ret;
}

/**
 * @function Pm_CreateVirtualInput
 *
 * @param    {String}        name
 * @param    {String}        interf
 *
 * @return   {Number}
 */
define('Pm_CreateVirtualInput', dlsym(libportmidi, 'Pm_CreateVirtualInput'), null, 'int', 'string', 'string', 'void *');
export function Pm_CreateVirtualInput(name, interf) {
  return call('Pm_CreateVirtualInput', name, interf, null);
}

/**
 * @function Pm_CreateVirtualOutput
 *
 * @param    {String}        name
 * @param    {String}        interf
 *
 * @return   {Number}
 */
define('Pm_CreateVirtualOutput', dlsym(libportmidi, 'Pm_CreateVirtualOutput'), null, 'int', 'string', 'string', 'void *');
export function Pm_CreateVirtualOutput(name, interf) {
  return call('Pm_CreateVirtualOutput', name, interf, null);
}

/**
 * @function Pm_SetFilter
 *
 * @param    {Number}        stream
 * @param    {Number}        filters
 *
 * @return   {Number}
 */
define('Pm_SetFilter', dlsym(libportmidi, 'Pm_SetFilter'), null, 'int', 'void *', 'int');
export function Pm_SetFilter(stream, filters) {
  return call('Pm_SetFilter', stream, filters);
}

/**
 * @function Pm_SetChannelMask
 *
 * @param    {Number}        stream
 * @param    {Number}        mask
 *
 * @return   {Number}
 */
define('Pm_SetChannelMask', dlsym(libportmidi, 'Pm_SetChannelMask'), null, 'int', 'void *', 'int');
export function Pm_SetChannelMask(stream, mask) {
  return call('Pm_SetChannelMask', stream, mask);
}

/**
 * @function Pm_Abort
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pm_Abort', dlsym(libportmidi, 'Pm_Abort'), null, 'int', 'void *');
export function Pm_Abort(stream) {
  return call('Pm_Abort', stream);
}

/**
 * @function Pm_Close
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pm_Close', dlsym(libportmidi, 'Pm_Close'), null, 'int', 'void *');
export function Pm_Close(stream) {
  return call('Pm_Close', stream);
}

/**
 * @function Pm_Synchronize
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pm_Synchronize', dlsym(libportmidi, 'Pm_Synchronize'), null, 'int', 'void *');
export function Pm_Synchronize(stream) {
  return call('Pm_Synchronize', stream);
}

/**
 * @function Pm_Read
 *
 * @param    {Number}        stream
 * @param    {Number}        buffer
 * @param    {Number}        length
 *
 * @return   {Number}
 */
define('Pm_Read', dlsym(libportmidi, 'Pm_Read'), null, 'int', 'void *', 'void *', 'int');
export function Pm_Read(stream, buffer, length) {
  return call('Pm_Read', stream, buffer, length);
}

/**
 * @function Pm_Poll
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pm_Poll', dlsym(libportmidi, 'Pm_Poll'), null, 'int', 'void *');
export function Pm_Poll(stream) {
  return call('Pm_Poll', stream);
}

/**
 * @function Pm_Write
 *
 * @param    {Number}        stream
 * @param    {Number}        buffer
 * @param    {Number}        length
 *
 * @return   {Number}
 */
define('Pm_Write', dlsym(libportmidi, 'Pm_Write'), null, 'int', 'void *', 'void *', 'int');
export function Pm_Write(stream, buffer, length) {
  length ??= buffer.byteLength >> 3;
  return call('Pm_Write', stream, buffer, length);
}

/**
 * @function Pm_WriteShort
 *
 * @param    {Number}        stream
 * @param    {Number}        when
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_WriteShort', dlsym(libportmidi, 'Pm_WriteShort'), null, 'int', 'void *', 'int', 'int');
export function Pm_WriteShort(stream, when, msg) {
  return call('Pm_WriteShort', stream, when, msg);
}

/**
 * @function Pm_WriteSysEx
 *
 * @param    {Number}        stream
 * @param    {Number}        when
 * @param    {Number}        msg
 *
 * @return   {Number}
 */
define('Pm_WriteSysEx', dlsym(libportmidi, 'Pm_WriteSysEx'), null, 'int', 'void *', 'int', 'void *');
export function Pm_WriteSysEx(stream, when, msg) {
  return call('Pm_WriteSysEx', stream, when, msg);
}
