import { dlopen, define, dlerror, dlclose, dlsym, call, errno, RTLD_NOW, toString } from 'ffi';
import { toPointer, toArrayBuffer, isArrayBuffer, isObject, define as defineObj } from 'util';

const libportaudio = dlopen('libportaudio.so.2', RTLD_NOW);

export const paFloat32 = 0x00000001; /**< @see PaSampleFormat */
export const paInt32 = 0x00000002; /**< @see PaSampleFormat */
export const paInt24 = 0x00000004; /**< Packed 24 bit format. @see PaSampleFormat */
export const paInt16 = 0x00000008; /**< @see PaSampleFormat */
export const paInt8 = 0x00000010; /**< @see PaSampleFormat */
export const paUInt8 = 0x00000020; /**< @see PaSampleFormat */
export const paCustomFormat = 0x00010000; /**< @see PaSampleFormat */
export const paNonInterleaved = 0x80000000; /**< @see PaSampleFormat */

export const paInDevelopment = 0 /* use while developing support for a new host API */,
  paDirectSound = 1,
  paMME = 2,
  paASIO = 3,
  paSoundManager = 4,
  paCoreAudio = 5,
  paOSS = 7,
  paALSA = 8,
  paAL = 9,
  paBeOS = 10,
  paWDMKS = 11,
  paJACK = 12,
  paWASAPI = 13,
  paAudioScienceHPI = 14;

export const paContinue = 0 /**< Signal that the stream should continue invoking the callback and processing audio. */,
  paComplete = 1 /**< Signal that the stream should stop invoking the callback and finish once all output samples have played. */,
  paAbort = 2; /**< Signal that the stream should stop invoking the callback and finish as soon as possible. */

export const paNoError = 0,
  paNotInitialized = -10000,
  paUnanticipatedHostError = -9999,
  paInvalidChannelCount = -9998,
  paInvalidSampleRate = -9997,
  paInvalidDevice = -9996,
  paInvalidFlag = -9995,
  paSampleFormatNotSupported = -9994,
  paBadIODeviceCombination = -9993,
  paInsufficientMemory = -9992,
  paBufferTooBig = -9991,
  paBufferTooSmall = -9990,
  paNullCallback = -9989,
  paBadStreamPtr = -9988,
  paTimedOut = -9987,
  paInternalError = -9986,
  paDeviceUnavailable = -9985,
  paIncompatibleHostApiSpecificStreamInfo = -9984,
  paStreamIsStopped = -9983,
  paStreamIsNotStopped = -9982,
  paInputOverflowed = -9981,
  paOutputUnderflowed = -9980,
  paHostApiNotFound = -9979,
  paInvalidHostApi = -9978,
  paCanNotReadFromACallbackStream = -9977,
  paCanNotWriteToACallbackStream = -9976,
  paCanNotReadFromAnOutputOnlyStream = -9975,
  paCanNotWriteToAnInputOnlyStream = -9974,
  paIncompatibleStreamHostApi = -9973,
  paBadBufferPtr = -9972;

export let Pa_Result;

const nameKey = /*'name' ??*/ Symbol.toStringTag;
const inspectKey = Symbol.inspect;

export function HexString(num) {
  return '0x' + num.toString(16);
}

export function DereferencePointer(ptr) {
  return new BigInt64Array(ptr)[0];
}

export function GetStream(stream_ptr) {
  return toArrayBuffer(typeof stream_ptr == 'string' ? stream_ptr : HexString(DereferencePointer(stream_ptr)), 16);
}

export function PassPointer(arg) {
  if(!arg) arg = null;

  if(arg) {
    if(typeof arg == 'string') arg = toArrayBuffer('0x' + BigInt(arg).toString(16), 16);
    if(typeof arg == 'object' && !(arg instanceof ArrayBuffer)) arg = arg.buffer;

    if(!isArrayBuffer(arg)) throw new Error(`PassPointer() argument is not array buffer: ${arg}`);
  }

  console.log('PassPointer()', `arg=`, isArrayBuffer(arg) ? toPointer(arg) : arg);

  return arg;
}

export class PaVersionInfo extends ArrayBuffer {
  constructor(obj) {
    super(32);

    if(isObject(obj)) {
      const { versionMajor, versionMinor, versionSubMinor, versionControlRevision, versionText } = obj;
      Object.assign(this, { versionMajor, versionMinor, versionSubMinor, versionControlRevision, versionText });
    }
  }

  /* 0: int versionMajor */
  set versionMajor(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get versionMajor() {
    return new Int32Array(this, 0)[0];
  }

  /* 4: int versionMinor */
  set versionMinor(value) {
    new Int32Array(this, 4)[0] = value;
  }

  get versionMinor() {
    return new Int32Array(this, 4)[0];
  }

  /* 8: int versionSubMinor */
  set versionSubMinor(value) {
    new Int32Array(this, 8)[0] = value;
  }

  get versionSubMinor() {
    return new Int32Array(this, 8)[0];
  }

  /*set versionControlRevision(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }*/

  get versionControlRevision() {
    const a = new BigInt64Array(this, 16);
    if(a[0] == BigInt(0)) return null;
    return toString('0x' + a[0].toString(16));
  }

  /*set versionText(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 24)[0] = BigInt(value);
  }*/

  get versionText() {
    const a = new BigInt64Array(this, 24);
    if(a[0] == BigInt(0)) return null;
    return toString('0x' + a[0].toString(16));
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 32);
    return Object.setPrototypeOf(ret, PaVersionInfo.prototype);
  }

  [inspectKey]() {
    const { versionMajor, versionMinor, versionSubMinor, versionControlRevision, versionText } = this;
    return (
      `\x1b[1;31mPaVersionInfo\x1b[0m ` +
      inspect({ versionMajor, versionMinor, versionSubMinor, versionControlRevision, versionText }, { colors: true })
    );
  }
}

defineObj(PaVersionInfo.prototype, { [nameKey]: 'PaVersionInfo' });

export class PaHostApiInfo extends ArrayBuffer {
  constructor(obj) {
    super(32);

    if(isObject(obj)) {
      const { structVersion, type, name, deviceCount, defaultInputDevice, defaultOutputDevice } = obj;
      Object.assign(this, { structVersion, type, name, deviceCount, defaultInputDevice, defaultOutputDevice });
    }
  }

  /* 0: int structVersion */
  set structVersion(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get structVersion() {
    return new Int32Array(this, 0)[0];
  }

  /* 4: PaHostApiTypeId (int) type */
  set type(value) {
    new Int32Array(this, 4)[0] = value;
  }

  get type() {
    return new Int32Array(this, 4)[0];
  }

  /*set name(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }*/

  get name() {
    const a = new BigInt64Array(this, 8, 1);
    if(a[0] == BigInt(0)) return null;
    return toString('0x' + a[0].toString(16));
  }

  /* 16: int deviceCount */
  set deviceCount(value) {
    new Int32Array(this, 16)[0] = value;
  }

  get deviceCount() {
    return new Int32Array(this, 16)[0];
  }

  /* 20: PaDeviceIndex (int) defaultInputDevice */
  set defaultInputDevice(value) {
    new Int32Array(this, 20)[0] = value;
  }

  get defaultInputDevice() {
    return new Int32Array(this, 20)[0];
  }

  /* 24: PaDeviceIndex (int) defaultOutputDevice */
  set defaultOutputDevice(value) {
    new Int32Array(this, 24)[0] = value;
  }

  get defaultOutputDevice() {
    return new Int32Array(this, 24)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 28);
    return Object.setPrototypeOf(ret, PaHostApiInfo.prototype);
  }

  [inspectKey]() {
    const { type, name, deviceCount, defaultInputDevice, defaultOutputDevice } = this;

    return (
      `\x1b[1;31mPaHostApiInfo\x1b[0m ` +
      inspect({ type, name, deviceCount, defaultInputDevice, defaultOutputDevice }, { colors: true })
    );
  }
}

defineObj(PaHostApiInfo.prototype, { [nameKey]: 'PaHostApiInfo' });

export class PaHostErrorInfo extends ArrayBuffer {
  constructor(obj) {
    super(24);

    if(isObject(obj)) {
      const { hostApiType, errorCode, errorText } = obj;
      Object.assign(this, { hostApiType, errorCode, errorText });
    }
  }

  /* 0: PaHostApiTypeId (int) hostApiType */
  set hostApiType(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get hostApiType() {
    return new Int32Array(this, 0)[0];
  }

  /* 8: long errorCode */
  set errorCode(value) {
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }

  get errorCode() {
    return Number(new BigInt64Array(this, 8)[0]);
  }

  /*set errorText(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 16)[0] = BigInt(value);
  }*/

  get errorText() {
    const a = new BigInt64Array(this, 16);
    if(a[0] == BigInt(0)) return null;
    return toString('0x' + a[0].toString(16));
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 24);
    return Object.setPrototypeOf(ret, PaHostErrorInfo.prototype);
  }

  [inspectKey]() {
    const { hostApiType, errorCode, errorText } = this;

    return `\x1b[1;31mPaHostErrorInfo\x1b[0m ` + inspect({ hostApiType, errorCode, errorText }, { colors: true });
  }
}

defineObj(PaHostErrorInfo.prototype, { [nameKey]: 'PaHostErrorInfo' });

export class PaDeviceInfo extends ArrayBuffer {
  constructor(obj) {
    super(72);

    if(isObject(obj)) {
      /* prettier-ignore */ const { structVersion, name, hostApi, maxInputChannels, maxOutputChannels, defaultLowInputLatency, defaultLowOutputLatency, defaultHighInputLatency, defaultHighOutputLatency, defaultSampleRate } = obj;
      /* prettier-ignore */ Object.assign(this, { structVersion, name, hostApi, maxInputChannels, maxOutputChannels, defaultLowInputLatency, defaultLowOutputLatency, defaultHighInputLatency, defaultHighOutputLatency, defaultSampleRate });
    }
  }

  /* 0: int structVersion */
  set structVersion(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get structVersion() {
    return new Int32Array(this, 0)[0];
  }

  /*set name(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }*/

  get name() {
    const a = new BigInt64Array(this, 8);
    if(a[0] == BigInt(0)) return null;
    return toString('0x' + a[0].toString(16));
  }

  /* 16: PaHostApiIndex (int) hostApi */
  set hostApi(value) {
    new Int32Array(this, 16)[0] = value;
  }

  get hostApi() {
    return new Int32Array(this, 16)[0];
  }

  /* 20: int maxInputChannels */
  set maxInputChannels(value) {
    new Int32Array(this, 20)[0] = value;
  }

  get maxInputChannels() {
    return new Int32Array(this, 20)[0];
  }

  /* 24: int maxOutputChannels */
  set maxOutputChannels(value) {
    new Int32Array(this, 24)[0] = value;
  }

  get maxOutputChannels() {
    return new Int32Array(this, 24)[0];
  }

  /* 32: PaTime (double) defaultLowInputLatency */
  set defaultLowInputLatency(value) {
    new Float64Array(this, 32)[0] = value;
  }

  get defaultLowInputLatency() {
    return new Float64Array(this, 32)[0];
  }

  /* 40: PaTime (double) defaultLowOutputLatency */
  set defaultLowOutputLatency(value) {
    new Float64Array(this, 40)[0] = value;
  }

  get defaultLowOutputLatency() {
    return new Float64Array(this, 40)[0];
  }

  /* 48: PaTime (double) defaultHighInputLatency */
  set defaultHighInputLatency(value) {
    new Float64Array(this, 48)[0] = value;
  }

  get defaultHighInputLatency() {
    return new Float64Array(this, 48)[0];
  }

  /* 56: PaTime (double) defaultHighOutputLatency */
  set defaultHighOutputLatency(value) {
    new Float64Array(this, 56)[0] = value;
  }

  get defaultHighOutputLatency() {
    return new Float64Array(this, 56)[0];
  }

  /* 64: double defaultSampleRate */
  set defaultSampleRate(value) {
    new Float64Array(this, 64)[0] = value;
  }

  get defaultSampleRate() {
    return new Float64Array(this, 64)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 72);
    return Object.setPrototypeOf(ret, PaDeviceInfo.prototype);
  }

  [inspectKey]() {
    const {
      name,
      hostApi,
      maxInputChannels,
      maxOutputChannels,
      defaultLowInputLatency,
      defaultLowOutputLatency,
      defaultHighInputLatency,
      defaultHighOutputLatency,
      defaultSampleRate,
    } = this;
    return (
      `\x1b[1;31mPaDeviceInfo\x1b[0m ` +
      inspect(
        {
          name,
          hostApi,
          maxInputChannels,
          maxOutputChannels,
          defaultLowInputLatency,
          defaultLowOutputLatency,
          defaultHighInputLatency,
          defaultHighOutputLatency,
          defaultSampleRate,
        },
        { colors: true },
      )
    );
  }
}

defineObj(PaDeviceInfo.prototype, { [nameKey]: 'PaDeviceInfo' });

export class PaStreamParameters extends ArrayBuffer {
  constructor(obj) {
    super(32);

    if(isObject(obj)) {
      const { device, channelCount, sampleFormat, suggestedLatency, hostApiSpecificStreamInfo } = obj;
      Object.assign(this, { device, channelCount, sampleFormat, suggestedLatency, hostApiSpecificStreamInfo });
    }
  }

  /* 0: PaDeviceIndex (int) device */
  set device(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get device() {
    return new Int32Array(this, 0)[0];
  }

  /* 4: int channelCount */
  set channelCount(value) {
    new Int32Array(this, 4)[0] = value;
  }

  get channelCount() {
    return new Int32Array(this, 4)[0];
  }

  /* 8: PaSampleFormat (unsigned long) sampleFormat */
  set sampleFormat(value) {
    new BigInt64Array(this, 8)[0] = BigInt(value);
  }

  get sampleFormat() {
    return Number(new BigInt64Array(this, 8)[0]);
  }

  /* 16: PaTime (double) suggestedLatency */
  set suggestedLatency(value) {
    new Float64Array(this, 16)[0] = value;
  }

  get suggestedLatency() {
    return new Float64Array(this, 16)[0];
  }

  /* 24: void* hostApiSpecificStreamInfo */
  set hostApiSpecificStreamInfo(value) {
    if(typeof value == 'object' && value != null && value instanceof ArrayBuffer) value = toPointer(value);
    try {
      new BigInt64Array(this, 24)[0] = BigInt(value);
    } catch(e) {
      console.log('set hostApiSpecificStreamInfo', value);
    }
  }

  get hostApiSpecificStreamInfo() {
    return '0x' + new BigInt64Array(this, 24)[0].toString(16);
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 32);
    return Object.setPrototypeOf(ret, PaStreamParameters.prototype);
  }

  [inspectKey]() {
    const { device, channelCount, sampleFormat, suggestedLatency, hostApiSpecificStreamInfo } = this;
    return (
      `\x1b[1;31mPaStreamParameters\x1b[0m ` +
      inspect({ device, channelCount, sampleFormat, suggestedLatency, hostApiSpecificStreamInfo }, { colors: true })
    );
  }
}

defineObj(PaStreamParameters.prototype, { [nameKey]: 'PaStreamParameters' });

export class PaStreamInfo extends ArrayBuffer {
  constructor(obj) {
    super(32);

    if(isObject(obj)) {
      const { structVersion, inputLatency, outputLatency, sampleRate } = obj;

      Object.assign(this, { structVersion, inputLatency, outputLatency, sampleRate });
    }
  }

  /* 0: int structVersion */
  set structVersion(value) {
    new Int32Array(this, 0)[0] = value;
  }

  get structVersion() {
    return new Int32Array(this, 0)[0];
  }

  /* 8: PaTime (double) inputLatency */
  set inputLatency(value) {
    new Float64Array(this, 8)[0] = value;
  }

  get inputLatency() {
    return new Float64Array(this, 8)[0];
  }

  /* 16: PaTime (double) outputLatency */
  set outputLatency(value) {
    new Float64Array(this, 16)[0] = value;
  }

  get outputLatency() {
    return new Float64Array(this, 16)[0];
  }

  /* 24: double sampleRate */
  set sampleRate(value) {
    new Float64Array(this, 24)[0] = value;
  }

  get sampleRate() {
    return new Float64Array(this, 24)[0];
  }

  static from(address) {
    let ret = toArrayBuffer(isArrayBuffer(address) ? toPointer(address) : address, 32);
    return Object.setPrototypeOf(ret, PaStreamInfo.prototype);
  }

  [inspectKey]() {
    const { inputLatency, outputLatency, sampleRate } = this;
    return `\x1b[1;31mPaStreamInfo\x1b[0m ` + inspect({ inputLatency, outputLatency, sampleRate }, { colors: true });
  }
}

defineObj(PaStreamInfo.prototype, { [nameKey]: 'PaStreamInfo' });

/**
 * @function Pa_GetVersion
 *
 * @return   {Number}
 */
define('Pa_GetVersion', dlsym(libportaudio, 'Pa_GetVersion'), null, 'int');

export function Pa_GetVersion() {
  return call('Pa_GetVersion');
}

/**
 * @function Pa_GetVersionText
 *
 * @return   {String}
 */
define('Pa_GetVersionText', dlsym(libportaudio, 'Pa_GetVersionText'), null, 'char *');

export function Pa_GetVersionText() {
  return call('Pa_GetVersionText');
}

/**
 * @function Pa_GetVersionInfo
 *
 * @return   {Number}
 */
define('Pa_GetVersionInfo', dlsym(libportaudio, 'Pa_GetVersionInfo'), null, 'void *');

export function Pa_GetVersionInfo() {
  const result = call('Pa_GetVersionInfo');

  return result ? PaVersionInfo.from(result) : null;
}

/**
 * @function Pa_GetErrorText
 *
 * @param    {Number}        errorCode
 *
 * @return   {String}        The error text
 */
define('Pa_GetErrorText', dlsym(libportaudio, 'Pa_GetErrorText'), null, 'char *', 'int');
export function Pa_GetErrorText(errorCode = Pa_Result) {
  return call('Pa_GetErrorText', errorCode);
}

/**
 * @function Pa_Initialize
 *
 * @return   {Number}          PaError
 */
define('Pa_Initialize', dlsym(libportaudio, 'Pa_Initialize'), null, 'int');
export function Pa_Initialize() {
  return call('Pa_Initialize');
}

/**
 * @function Pa_Terminate
 *
 * @return   {Number}          PaError
 */
define('Pa_Terminate', dlsym(libportaudio, 'Pa_Terminate'), null, 'int');
export function Pa_Terminate() {
  return call('Pa_Terminate');
}

/**
 * @function Pa_GetHostApiCount
 *
 * @return   {Number}         Number of host APIs
 */
define('Pa_GetHostApiCount', dlsym(libportaudio, 'Pa_GetHostApiCount'), null, 'int');
export function Pa_GetHostApiCount() {
  return call('Pa_GetHostApiCount');
}

/**
 * @function Pa_GetDefaultHostApi
 *
 * @return   {Number}         Index of the default host API
 */
define('Pa_GetDefaultHostApi', dlsym(libportaudio, 'Pa_GetDefaultHostApi'), null, 'int');
export function Pa_GetDefaultHostApi() {
  return call('Pa_GetDefaultHostApi');
}

/**
 * @function Pa_GetHostApiInfo
 *
 * @param    {Number}        A valid host API index ranging from 0 to (Pa_GetHostApiCount()-1)
 *
 * @return   {PaHostApiInfo|null}   A pointer to an immutable PaHostApiInfo structure describing a specific host API. If the hostApi parameter is out of range or an error is encountered, the function returns NULL.
 */
define('Pa_GetHostApiInfo', dlsym(libportaudio, 'Pa_GetHostApiInfo'), null, 'void *', 'int');
export function Pa_GetHostApiInfo(hostApi) {
  const result = call('Pa_GetHostApiInfo', hostApi);
  return result ? PaHostApiInfo.from(result) : null;
}

/**
 * @function Pa_HostApiTypeIdToHostApiIndex
 *
 * @param    {Number}        type     A unique host API identifier belonging to the PaHostApiTypeId
 enumeration.

 *
 * @return   {Number}        A valid PaHostApiIndex ranging from 0 to (Pa_GetHostApiCount()-1) or, a PaErrorCode (which are always negative) if PortAudio is not initialized or an error is encountered.
 */
define('Pa_HostApiTypeIdToHostApiIndex', dlsym(libportaudio, 'Pa_HostApiTypeIdToHostApiIndex'), null, 'int', 'int');
export function Pa_HostApiTypeIdToHostApiIndex(type) {
  return call('Pa_HostApiTypeIdToHostApiIndex', type);
}

/**
 * @function Pa_HostApiDeviceIndexToDeviceIndex
 *
 * @param    {Number}     hostApi              Host API index
 * @param    {Number}     hostApiDeviceIndex   host-API-specific device index
 *
 * @return   {Number}        A non-negative PaDeviceIndex ranging from 0 to (Pa_GetDeviceCount()-1) or, a PaErrorCode (which are always negative) if PortAudio is not initialized or an error is encountered.
 */
define(
  'Pa_HostApiDeviceIndexToDeviceIndex',
  dlsym(libportaudio, 'Pa_HostApiDeviceIndexToDeviceIndex'),
  null,
  'int',
  'int',
  'int',
);
export function Pa_HostApiDeviceIndexToDeviceIndex(hostApi, hostApiDeviceIndex) {
  return call('Pa_HostApiDeviceIndexToDeviceIndex', hostApi, hostApiDeviceIndex);
}

/**
 * @function Pa_GetLastHostErrorInfo
 *
 * @return   {Number}
 */
define('Pa_GetLastHostErrorInfo', dlsym(libportaudio, 'Pa_GetLastHostErrorInfo'), null, 'void *');
export function Pa_GetLastHostErrorInfo() {
  const result = call('Pa_GetLastHostErrorInfo');
  return result ? PaHostErrorInfo.from(result) : null;
}

/**
 * @function Pa_GetDeviceCount
 *
 * @return   {Number}
 */
define('Pa_GetDeviceCount', dlsym(libportaudio, 'Pa_GetDeviceCount'), null, 'int');
export function Pa_GetDeviceCount() {
  return call('Pa_GetDeviceCount');
}

/**
 * @function Pa_GetDefaultInputDevice
 *
 * @return   {Number}
 */
define('Pa_GetDefaultInputDevice', dlsym(libportaudio, 'Pa_GetDefaultInputDevice'), null, 'int');
export function Pa_GetDefaultInputDevice() {
  return call('Pa_GetDefaultInputDevice');
}

/**
 * @function Pa_GetDefaultOutputDevice
 *
 * @return   {Number}
 */
define('Pa_GetDefaultOutputDevice', dlsym(libportaudio, 'Pa_GetDefaultOutputDevice'), null, 'int');
export function Pa_GetDefaultOutputDevice() {
  return call('Pa_GetDefaultOutputDevice');
}

/**
 * @function Pa_GetDeviceInfo
 *
 * @param    {Number}        device
 *
 * @return   {Number}
 */
define('Pa_GetDeviceInfo', dlsym(libportaudio, 'Pa_GetDeviceInfo'), null, 'void *', 'int');
export function Pa_GetDeviceInfo(device) {
  const result = call('Pa_GetDeviceInfo', device);
  return result ? PaDeviceInfo.from(result) : null;
}

/**
 * @function Pa_IsFormatSupported
 *
 * @param    {Number}        inputParameters
 * @param    {Number}        outputParameters
 * @param    {Number}        sampleRate
 *
 * @return   {Number}
 */
define('Pa_IsFormatSupported', dlsym(libportaudio, 'Pa_IsFormatSupported'), null, 'int', 'void *', 'void *', 'double');
export function Pa_IsFormatSupported(inputParameters, outputParameters, sampleRate) {
  return call('Pa_IsFormatSupported', inputParameters, outputParameters, sampleRate);
}

/**
 * @function Pa_OpenStream
 *
 * @param    {Number}        stream
 * @param    {Number}        inputParameters
 * @param    {Number}        outputParameters
 * @param    {Number}        sampleRate
 * @param    {Number}        framesPerBuffer
 * @param    {Number}        streamFlags
 * @param    {Number}        streamCallback
 * @param    {Number}        userData
 *
 * @return   {Number}
 */
define(
  'Pa_OpenStream',
  dlsym(libportaudio, 'Pa_OpenStream'),
  null,
  'int',
  'void *',
  'void *',
  'void *',
  'double',
  'unsigned long',
  'unsigned long',
  'void *',
  'void *',
);
export function Pa_OpenStream(
  stream,
  inputParameters,
  outputParameters,
  sampleRate,
  framesPerBuffer,
  streamFlags,
  streamCallback,
  userData,
) {
  let cb;

  typeof stream == 'function' && ((cb = stream), (stream = new ArrayBuffer(16)));

  Pa_Result = call(
    'Pa_OpenStream',
    stream,
    inputParameters,
    outputParameters,
    sampleRate,
    framesPerBuffer,
    streamFlags,
    streamCallback,
    userData,
  );

  cb && cb(GetStream());

  return Pa_Result;
}

/**
 * @function Pa_OpenDefaultStream
 *
 * @param    {Number}        stream
 * @param    {Number}        numInputChannels
 * @param    {Number}        numOutputChannels
 * @param    {Number}        sampleFormat
 * @param    {Number}        sampleRate
 * @param    {Number}        framesPerBuffer
 * @param    {Number}        streamCallback
 * @param    {Number}        userData
 *
 * @return   {Number}
 */
define(
  'Pa_OpenDefaultStream',
  dlsym(libportaudio, 'Pa_OpenDefaultStream'),
  null,
  'int',
  'void *',
  'int',
  'int',
  'unsigned long',
  'double',
  'unsigned long',
  'void *',
  'void *',
);
export function Pa_OpenDefaultStream(
  stream,
  numInputChannels,
  numOutputChannels,
  sampleFormat,
  sampleRate,
  framesPerBuffer,
  streamCallback,
  userData,
) {
  return call(
    'Pa_OpenDefaultStream',
    stream,
    numInputChannels,
    numOutputChannels,
    sampleFormat,
    sampleRate,
    framesPerBuffer,
    streamCallback,
    userData,
  );
}

/**
 * @function Pa_CloseStream
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_CloseStream', dlsym(libportaudio, 'Pa_CloseStream'), null, 'int', 'void *');
export function Pa_CloseStream(stream) {
  return call('Pa_CloseStream', GetStream(stream));
}

/**
 * @function Pa_SetStreamFinishedCallback
 *
 * @param    {Number}        stream
 * @param    {Number}        streamFinishedCallback
 *
 * @return   {Number}
 */
define(
  'Pa_SetStreamFinishedCallback',
  dlsym(libportaudio, 'Pa_SetStreamFinishedCallback'),
  null,
  'int',
  'void *',
  'void *',
);
export function Pa_SetStreamFinishedCallback(stream, streamFinishedCallback) {
  return call('Pa_SetStreamFinishedCallback', GetStream(stream), streamFinishedCallback);
}

/**
 * @function Pa_StartStream
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_StartStream', dlsym(libportaudio, 'Pa_StartStream'), null, 'int', 'void *');
export function Pa_StartStream(stream) {
  return call('Pa_StartStream', GetStream(stream));
}

/**
 * @function Pa_StopStream
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_StopStream', dlsym(libportaudio, 'Pa_StopStream'), null, 'int', 'void *');
export function Pa_StopStream(stream) {
  return call('Pa_StopStream', GetStream(stream));
}

/**
 * @function Pa_AbortStream
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_AbortStream', dlsym(libportaudio, 'Pa_AbortStream'), null, 'int', 'void *');
export function Pa_AbortStream(stream) {
  return call('Pa_AbortStream', GetStream(stream));
}

/**
 * @function Pa_IsStreamStopped
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_IsStreamStopped', dlsym(libportaudio, 'Pa_IsStreamStopped'), null, 'int', 'void *');
export function Pa_IsStreamStopped(stream) {
  return call('Pa_IsStreamStopped', GetStream(stream));
}

/**
 * @function Pa_IsStreamActive
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_IsStreamActive', dlsym(libportaudio, 'Pa_IsStreamActive'), null, 'int', 'void *');
export function Pa_IsStreamActive(stream) {
  return call('Pa_IsStreamActive', GetStream(stream));
}

/**
 * @function Pa_GetStreamInfo
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_GetStreamInfo', dlsym(libportaudio, 'Pa_GetStreamInfo'), null, 'void *', 'void *');
export function Pa_GetStreamInfo(stream) {
  const result = call('Pa_GetStreamInfo', GetStream(stream));

  return result ? PaStreamInfo.from(result) : null;
}

/**
 * @function Pa_GetStreamTime
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_GetStreamTime', dlsym(libportaudio, 'Pa_GetStreamTime'), null, 'double', 'void *');
export function Pa_GetStreamTime(stream) {
  return call('Pa_GetStreamTime', GetStream(stream));
}

/**
 * @function Pa_GetStreamCpuLoad
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_GetStreamCpuLoad', dlsym(libportaudio, 'Pa_GetStreamCpuLoad'), null, 'double', 'void *');
export function Pa_GetStreamCpuLoad(stream) {
  return call('Pa_GetStreamCpuLoad', GetStream(stream));
}

/**
 * @function Pa_ReadStream
 *
 * @param    {Number}        stream
 * @param    {Number}        buffer
 * @param    {Number}        frames
 *
 * @return   {Number}
 */
define('Pa_ReadStream', dlsym(libportaudio, 'Pa_ReadStream'), null, 'int', 'void *', 'void *', 'unsigned long');
export function Pa_ReadStream(stream, buffer, frames) {
  return (Pa_Result = call('Pa_ReadStream', GetStream(stream), PassPointer(buffer), frames));
}

/**
 * @function Pa_WriteStream
 *
 * @param    {Number}        stream
 * @param    {Number}        buffer
 * @param    {Number}        frames
 *
 * @return   {Number}
 */
define('Pa_WriteStream', dlsym(libportaudio, 'Pa_WriteStream'), null, 'int', 'void *', 'void *', 'unsigned long');
export function Pa_WriteStream(stream, buffer, frames) {
  return (Pa_Result = call('Pa_WriteStream', GetStream(stream), PassPointer(buffer), frames));
}

/**
 * @function Pa_GetStreamReadAvailable
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_GetStreamReadAvailable', dlsym(libportaudio, 'Pa_GetStreamReadAvailable'), null, 'long', 'void *');
export function Pa_GetStreamReadAvailable(stream) {
  return call('Pa_GetStreamReadAvailable', GetStream(stream));
}

/**
 * @function Pa_GetStreamWriteAvailable
 *
 * @param    {Number}        stream
 *
 * @return   {Number}
 */
define('Pa_GetStreamWriteAvailable', dlsym(libportaudio, 'Pa_GetStreamWriteAvailable'), null, 'long', 'void *');
export function Pa_GetStreamWriteAvailable(stream) {
  return call('Pa_GetStreamWriteAvailable', GetStream(stream));
}

/**
 * @function Pa_GetStreamHostApiType
 *
 * @param    {Number}        stream
 *
 * @return   {undefined}
 */
define('Pa_GetStreamHostApiType', dlsym(libportaudio, 'Pa_GetStreamHostApiType'), null, 'long', 'void *');
export function Pa_GetStreamHostApiType(stream) {
  return call('Pa_GetStreamHostApiType', GetStream(stream));
}

/**
 * @function Pa_GetSampleSize
 *
 * @param    {Number}        format
 *
 * @return   {Number}
 */
define('Pa_GetSampleSize', dlsym(libportaudio, 'Pa_GetSampleSize'), null, 'int', 'unsigned long');
export function Pa_GetSampleSize(format) {
  return call('Pa_GetSampleSize', format);
}

/**
 * @function Pa_Sleep
 *
 * @param    {Number}        msec
 */
define('Pa_Sleep', dlsym(libportaudio, 'Pa_Sleep'), null, 'void', 'long');
export function Pa_Sleep(msec) {
  call('Pa_Sleep', msec);
}

export function Pa_GetDevices() {
  return Array.from({ length: Pa_GetDeviceCount() }).map((d, i) => Pa_GetDeviceInfo(i));
}