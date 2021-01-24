const BPG_OUTPUT_FORMAT_RGB24 = 1;
const BPG_OUTPUT_FORMAT_RGBA32 = 2;
const BPG_OUTPUT_FORMAT_RGB48 = 3;
const BPG_OUTPUT_FORMAT_RGBA64 = 4;
const BPG_OUTPUT_FORMAT_CMYK32 =5;
const BPG_OUTPUT_FORMAT_CMYK64 = 6;

   class BPGImageInfo extends ArrayBuffer {
  constructor(obj = {}) {
    super(24);
    Object.assign(this, obj);
  }

  static from(arrayBuf) {
    return Object.setPrototypeOf(arrayBuf.slice(), BPGImageInfo.prototype);
  }
  get [Symbol.toStringTag]() { return `[struct BPGImageInfo @ ${this} ]`; }

  /* width@0 uint32_t 4 */
  set width(v) { new Uint32Array(this, 0)[0] = v; }
  get width() { return new Uint32Array(this, 0)[0]; }

  /* height@4 uint32_t 4 */
  set height(v) { new Uint32Array(this, 4, 1)[0] = v; }
  get height() { return new Uint32Array(this, 4, 1)[0]; }

  /* format@8 uint8_t 1 */
  set format(v) { new Uint8Array(this, 8)[0] = v; }
  get format() { return new Uint8Array(this, 8)[0]; }

  /* has_alpha@9 uint8_t 1 */
  set has_alpha(v) { new Uint8Array(this, 9)[0] = v; }
  get has_alpha() { return new Uint8Array(this, 9)[0]; }

  /* color_space@10 uint8_t 1 */
  set color_space(v) { new Uint8Array(this, 10)[0] = v; }
  get color_space() { return new Uint8Array(this, 10)[0]; }

  /* bit_depth@11 uint8_t 1 */
  set bit_depth(v) { new Uint8Array(this, 11)[0] = v; }
  get bit_depth() { return new Uint8Array(this, 11)[0]; }

  /* premultiplied_alpha@12 uint8_t 1 */
  set premultiplied_alpha(v) { new Uint8Array(this, 12)[0] = v; }
  get premultiplied_alpha() { return new Uint8Array(this, 12)[0]; }

  /* has_w_plane@13 uint8_t 1 */
  set has_w_plane(v) { new Uint8Array(this, 13)[0] = v; }
  get has_w_plane() { return new Uint8Array(this, 13)[0]; }

  /* limited_range@14 uint8_t 1 */
  set limited_range(v) { new Uint8Array(this, 14)[0] = v; }
  get limited_range() { return new Uint8Array(this, 14)[0]; }

  /* has_animation@15 uint8_t 1 */
  set has_animation(v) { new Uint8Array(this, 15)[0] = v; }
  get has_animation() { return new Uint8Array(this, 15)[0]; }

  /* loop_count@16 uint16_t NaN */
  set loop_count(v) { new Uint8Array(this, 16)[0] = v; }
  get loop_count() { return new Uint8Array(this, 16)[0]; }

  toString() {
    const { width, height, format, has_alpha, color_space, bit_depth, premultiplied_alpha, has_w_plane, limited_range, has_animation, loop_count } = this;
    return `struct BPGImageInfo {\n\t.width = ${width},\n\t.height = ${height},\n\t.format = ${format},\n\t.has_alpha = ${has_alpha},\n\t.color_space = ${color_space},\n\t.bit_depth = ${bit_depth},\n\t.premultiplied_alpha = ${premultiplied_alpha},\n\t.has_w_plane = ${has_w_plane},\n\t.limited_range = ${limited_range},\n\t.has_animation = ${has_animation},\n\t.loop_count = ${loop_count}\n}`;
  }
}


var importObject = {
  //c: { imported_func: arg => console.log(arg) },
  a(arg) {
    console.log(arg);
  },
  b: function (arg) {
    console.log(arg);
  },
  a: {
    a: arg => {
      console.log(arg);
      return () => {};
    },
    b: arg => {
      console.log(arg);
      return () => {};
    },
    c: arg => {
      console.log(arg);
      return () => {};
    }
  }
};

let functionNames = {
  d: 'memory',
  e: 'tables',
  g: '_malloc',
  h: '_free',
  i: '_bpg_decoder_get_info',
  j: '_bpg_decoder_start',
  k: '_bpg_decoder_get_frame_duration',
  l: '_bpg_decoder_get_line',
  m: '_bpg_decoder_open',
  n: '_bpg_decoder_decode',
  o: '_bpg_decoder_close',
  p: 'stackSave',
  q: 'stackRestore',
  r: 'stackAlloc'
};

 
function loadWebAssembly(fileName, impObj = {}) {
  return fetch(fileName)
    .then(response => response.arrayBuffer())
    .then(buffer => convertBufferStringToInstance(buffer, impObj)); 
}

let bpgdec={};


async function convertBufferStringToInstance(bufferArrayString, impObj = {}) {
  const buffer = new Uint8Array(typeof bufferArrayString == 'string' ? bufferArrayString.split(',') : bufferArrayString
  );
  const module = await WebAssembly.compile(buffer);
  console.log('convertBufferStringToInstance', { module, impObj });
  return WebAssembly.instantiate(buffer, impObj);
}
Object.assign(window, { loadWebAssembly, importObject, convertBufferStringToInstance, bpgdec, BPG_OUTPUT_FORMAT_RGB24, BPG_OUTPUT_FORMAT_RGBA32, BPG_OUTPUT_FORMAT_RGB48, BPG_OUTPUT_FORMAT_RGBA64, BPG_OUTPUT_FORMAT_CMYK32, BPG_OUTPUT_FORMAT_CMYK64
 });

window.addEventListener('load', async event => {
  console.log('onload');

  let wasm = await loadWebAssembly('/static/wasm/bpgdec8a.wasm', importObject).then(instance => {
    console.log('instance:', instance);
    console.log('instance.exports:', instance.exports);
    // bpgdec = instance.exports._Z7squareri;
    console.log('Finished compiling! Ready when you are...');
    return instance;
  });

  window.wasm = wasm;

  console.log('wasm:', wasm);
  console.log('wasm.instance.exports:', wasm.instance.exports);

  const {
    free,
    malloc,
    __wasm_call_ctors,
    stackAlloc,
    stackRestore,
    stackSave,
    bpg_decoder_close,
    bpg_decoder_decode,
    bpg_decoder_open,
    bpg_decoder_get_line,
    bpg_decoder_get_frame_duration,
    bpg_decoder_start,
    bpg_decoder_get_info
  } = wasm.instance.exports;

  /*  let bpgdec = {free, g, f, stackAlloc, stackRestore, stackSave, bpg_decoder_close, bpg_decoder_decode, bpg_decoder_open, bpg_decoder_get_line, bpg_decoder_get_frame_duration, bpg_decoder_start, bpg_decoder_get_info, e, d } || Object.fromEntries(Object.keys(functionNames).map(name => [functionNames[name], wasm.instance.exports[name]]));*/
   window.bpg = Object.assign(bpgdec, {
    free,
    malloc,
    __wasm_call_ctors,
    stackAlloc,
    stackRestore,
    stackSave,
    bpg_decoder_close,
    bpg_decoder_decode,
    bpg_decoder_open,
    bpg_decoder_get_line,
    bpg_decoder_get_frame_duration,
    bpg_decoder_start,
    bpg_decoder_get_info 
  });
  console.log(' window.bpg:', window.bpg);

  console.log('__wasm_call_ctors', __wasm_call_ctors());
});


async  function BPGDecode(buffer, outputFmt = BPG_OUTPUT_FORMAT_RGB24) {
if(typeof buffer == 'string') {
  buffer = await fetch(buffer).then(response => response.arrayBuffer())
}

  let decoder = bpg.bpg_decoder_open();
   console.log("decoder:", decoder);
 let info = new BPGImageInfo();
  let output;

  let ret;
    if((ret = bpg.bpg_decoder_get_info(decoder, info)) < 0)
    throw new Error(`bpg_decoder_get_info: ${ret}`);

  console.log("info:", info);

  output = new Array(info.height).map(() => new ArrayBuffer(info.width));
if((ret = bpg.bpg_decoder_start(decoder, outputFmt)) < 0)
    throw new Error(`bpg_decoder_start: ${ret}`);

  if((ret = bpg_decoder_decode(decoder, buffer, buffer.byteLength)) < 0)
    throw new Error(`bpg_decoder_decode: ${ret}`);


}

window.BPGDecode = BPGDecode;