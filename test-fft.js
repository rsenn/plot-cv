import Util from './lib/util.js';
import { dlopen, dlsym, RTLD_NOW, RTLD_DEFAULT, toPointer } from 'ffi';
import * as ffi from 'ffi';
import * as std from 'std';
import * as dsp from './lib/dsp/util.js';
import FFT from './lib/dsp/fft.js';
import ConsoleSetup from './lib/consoleSetup.js';

export function define(so, name, rtype, ...args) {
  if(so == null || so == undefined) so = ffi.RTLD_DEFAULT;
  var p = ffi.dlsym(so, name);
  if(p == null) {
    console.log(name, 'not in so');
    std.exit(1);
  }
  if(!ffi.define(name, p, null, rtype, ...args)) {
    console.log('define failed');
    std.exit(1);
  }
  return function(...a) {
    return ffi.call(name, ...a);
  };
}

const SFM_READ = 0x10;
const SFM_WRITE = 0x20;
const SFM_RDWR = 0x30;

class sf_info_t extends ArrayBuffer {
  constructor(obj = {}) {
    super(32);
    Object.assign(this, obj);
  }
  get [Symbol.toStringTag]() { return `[struct sf_info_t @ ${toPointer(this)} ]`; }

  set frames(v) { new BigUint64Array(this, 0)[0] = BigInt(v); }
  get frames() { return Number(new BigUint64Array(this, 0)[0]); }

  set samplerate(v) { new Uint32Array(this, 8)[0] = v; }
  get samplerate() { return new Uint32Array(this, 8)[0]; }

  set channels(v) { new Uint32Array(this, 12)[0] = v; }
  get channels() { return new Uint32Array(this, 12)[0]; }

  set format(v) { new Uint32Array(this, 16)[0] = v; }
  get format() { return new Uint32Array(this, 16)[0]; }

  set sections(v) { new Uint32Array(this, 20)[0] = v; }
  get sections() { return new Uint32Array(this, 20)[0]; }

  set seekable(v) { new Uint32Array(this, 24)[0] = v; }
  get seekable() { return new Uint32Array(this, 24)[0]; }

  toString() {
    const { frames, samplerate, channels, format, sections, seekable } = this;
    return `struct sf_info_t {\n\t.frames = ${frames},\n\t.samplerate = ${samplerate},\n\t.channels = ${channels},\n\t.format = ${format},\n\t.sections = ${sections},\n\t.seekable = ${seekable}\n}`;
  }
}

const libsndfile = dlopen('libsndfile.so.1', RTLD_NOW);

const sf_open = define(libsndfile, 'sf_open', 'void *', 'char *', 'int', 'void *');
const sf_readf_float = define(libsndfile, 'sf_readf_float', 'int', 'void *', 'ulong');

function main(...args) {
  ConsoleSetup({ depth: 20, colors: true, breakLength: 80 });
  console.log('test');

  let info = new sf_info_t();

  console.log('info:', info);
  console.log('args:', args);

  let ptr = sf_open(args[0] || '/opt/games/endless-sky/sounds/hyperdrive.wav',
    SFM_READ,
    info
  );
  console.log('info: ' + info.toString());

  let rate = info.samplerate;
  let sample = new Float32Array(info.frames);

  let ret = sf_readf_float(ptr, sample.buffer, info.frames);
  console.log('ret:', ret);
  console.log('sample:', sample.slice(0, 100));
  const size = 2048;
  const fft = new FFT(size);

  let re = new Float32Array(size);
  let im = new Float32Array(size);

  let input = dsp.window(sample.slice(0, size), size, 1, 0, dsp.hamming_window);

  fft.forward(sample.slice(0, size), 1, 0, re, im);
  console.log('re:', re.slice(0, 100));
  console.log('im:', im.slice(0, 100));

  const bin2frequency = (() => {
    const top_freq = rate / 2 - rate / size;

    return bin => (bin * top_freq) / (size - 1);
  })();

  console.log(`bin2freq(${size - 1}) =`, bin2frequency(size - 1));

  let mag = re.map(m => Math.abs(m));
  let max = Math.max(...mag);
  let ind = [...mag].map((m, i) => [i, m]).filter(([i, m]) => m === max);
  console.log(`dominant frequencies:`,
    ind.map(([i, m]) => bin2frequency(i))
  );
}

Util.callMain(main, true);
