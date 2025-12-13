import FFT from './lib/dsp/fft.js';
import * as dsp from './lib/dsp/util.js';
import * as ffi from 'ffi';
import * as std from 'std';

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
  /* prettier-ignore */ get [Symbol.toStringTag]() { return `[struct sf_info_t @ ${toPointer(this)} ]`; }

  /* prettier-ignore */ set frames(v) { new BigUint64Array(this, 0)[0] = BigInt(v); }
  /* prettier-ignore */ get frames() { return Number(new BigUint64Array(this, 0)[0]); }

  /* prettier-ignore */ set samplerate(v) { new Uint32Array(this, 8)[0] = v; }
  /* prettier-ignore */ get samplerate() { return new Uint32Array(this, 8)[0]; }

  /* prettier-ignore */ set channels(v) { new Uint32Array(this, 12)[0] = v; }
  /* prettier-ignore */ get channels() { return new Uint32Array(this, 12)[0]; }

  /* prettier-ignore */ set format(v) { new Uint32Array(this, 16)[0] = v; }
  /* prettier-ignore */ get format() { return new Uint32Array(this, 16)[0]; }

  /* prettier-ignore */ set sections(v) { new Uint32Array(this, 20)[0] = v; }
  /* prettier-ignore */ get sections() { return new Uint32Array(this, 20)[0]; }

  /* prettier-ignore */ set seekable(v) { new Uint32Array(this, 24)[0] = v; }
  /* prettier-ignore */ get seekable() { return new Uint32Array(this, 24)[0]; }

  toString() {
    const { frames, samplerate, channels, format, sections, seekable } = this;
    return `struct sf_info_t {\n\t.frames = ${frames},\n\t.samplerate = ${samplerate},\n\t.channels = ${channels},\n\t.format = ${format},\n\t.sections = ${sections},\n\t.seekable = ${seekable}\n}`;
  }
}

const libsndfile = dlopen('libsndfile.so.1', RTLD_NOW);

const sf_open = define(libsndfile, 'sf_open', 'void *', 'char *', 'int', 'void *');
const sf_readf_float = define(libsndfile, 'sf_readf_float', 'int', 'void *', 'ulong');

function main(...args) {
  console.log('test');

  let info = new sf_info_t();

  console.log('info:', info);
  console.log('args:', args);

  let ptr = sf_open(args[0] || '/opt/games/endless-sky/sounds/hyperdrive.wav', SFM_READ, info);
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
  console.log(
    `dominant frequencies:`,
    ind.map(([i, m]) => bin2frequency(i))
  );
}

main(...scriptArgs.slice(1));