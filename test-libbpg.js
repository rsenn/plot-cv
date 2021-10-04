//import Util from './lib/util.js';

const BPG_OUTPUT_FORMAT_RGB24 = 1;
const BPG_OUTPUT_FORMAT_RGBA32 = 2;
const BPG_OUTPUT_FORMAT_RGB48 = 3;
const BPG_OUTPUT_FORMAT_RGBA64 = 4;
const BPG_OUTPUT_FORMAT_CMYK32 = 5;
const BPG_OUTPUT_FORMAT_CMYK64 = 6;

class BPGImageInfo extends ArrayBuffer {
  constructor(obj = {}) {
    super(24);
    Object.assign(this, obj);
  }

  static from(arrayBuf) {
    if(typeof arrayBuf.buffer == 'object') arrayBuf = arrayBuf.buffer;

    return Object.setPrototypeOf(arrayBuf.slice(), BPGImageInfo.prototype);
  }
  /* prettier-ignore */ get [Symbol.toStringTag]() { return `[struct BPGImageInfo @ ${this} ]`; }

  /* width@0 uint32_t 4 */
  /* prettier-ignore */ set width(v) { new Uint32Array(this, 0)[0] = v; }
  /* prettier-ignore */ get width() { return new Uint32Array(this, 0)[0]; }

  /* height@4 uint32_t 4 */
  /* prettier-ignore */ set height(v) { new Uint32Array(this, 4, 1)[0] = v; }
  /* prettier-ignore */ get height() { return new Uint32Array(this, 4, 1)[0]; }

  /* format@8 uint8_t 1 */
  /* prettier-ignore */ set format(v) { new Uint8Array(this, 8, 1)[0] = v; }
  /* prettier-ignore */ get format() { return new Uint8Array(this, 8, 1)[0]; }

  /* has_alpha@9 uint8_t 1 */
  /* prettier-ignore */ set has_alpha(v) { new Uint8Array(this, 9, 1)[0] = v; }
  /* prettier-ignore */ get has_alpha() { return new Uint8Array(this, 9, 1)[0]; }

  /* color_space@10 uint8_t 1 */
  /* prettier-ignore */ set color_space(v) { new Uint8Array(this, 10, 1)[0] = v; }
  /* prettier-ignore */ get color_space() { return new Uint8Array(this, 10, 1)[0]; }

  /* bit_depth@11 uint8_t 1 */
  /* prettier-ignore */ set bit_depth(v) { new Uint8Array(this, 11, 1)[0] = v; }
  /* prettier-ignore */ get bit_depth() { return new Uint8Array(this, 11, 1)[0]; }

  /* premultiplied_alpha@12 uint8_t 1 */
  /* prettier-ignore */ set premultiplied_alpha(v) { new Uint8Array(this, 12, 1)[0] = v; }
  /* prettier-ignore */ get premultiplied_alpha() { return new Uint8Array(this, 12, 1)[0]; }

  /* has_w_plane@13 uint8_t 1 */
  /* prettier-ignore */ set has_w_plane(v) { new Uint8Array(this, 13, 1)[0] = v; }
  /* prettier-ignore */ get has_w_plane() { return new Uint8Array(this, 13, 1)[0]; }

  /* limited_range@14 uint8_t 1 */
  /* prettier-ignore */ set limited_range(v) { new Uint8Array(this, 14, 1)[0] = v; }
  /* prettier-ignore */ get limited_range() { return new Uint8Array(this, 14, 1)[0]; }

  /* has_animation@15 uint8_t 1 */
  /* prettier-ignore */ set has_animation(v) { new Uint8Array(this, 15, 1)[0] = v; }
  /* prettier-ignore */ get has_animation() { return new Uint8Array(this, 15, 1)[0]; }

  /* loop_count@16 uint16_t NaN */
  /* prettier-ignore */ set loop_count(v) { new Uint8Array(this, 16, 1)[0] = v; }
  /* prettier-ignore */ get loop_count() { return new Uint8Array(this, 16, 1)[0]; }

  toString() {
    const {
      width,
      height,
      format,
      has_alpha,
      color_space,
      bit_depth,
      premultiplied_alpha,
      has_w_plane,
      limited_range,
      has_animation,
      loop_count
    } = this;
    return `struct BPGImageInfo {\n\t.width = ${width},\n\t.height = ${height},\n\t.format = ${format},\n\t.has_alpha = ${has_alpha},\n\t.color_space = ${color_space},\n\t.bit_depth = ${bit_depth},\n\t.premultiplied_alpha = ${premultiplied_alpha},\n\t.has_w_plane = ${has_w_plane},\n\t.limited_range = ${limited_range},\n\t.has_animation = ${has_animation},\n\t.loop_count = ${loop_count}\n}`;
  }
}

Object.assign(window, {
  BPG_OUTPUT_FORMAT_RGB24,
  BPG_OUTPUT_FORMAT_RGBA32,
  BPG_OUTPUT_FORMAT_RGB48,
  BPG_OUTPUT_FORMAT_RGBA64,
  BPG_OUTPUT_FORMAT_CMYK32,
  BPG_OUTPUT_FORMAT_CMYK64
});

const BPG_OUTPUT_FORMAT_NAMES = {
  [BPG_OUTPUT_FORMAT_RGB24]: 'BPG_OUTPUT_FORMAT_RGB24',
  [BPG_OUTPUT_FORMAT_RGBA32]: 'BPG_OUTPUT_FORMAT_RGBA32',
  [BPG_OUTPUT_FORMAT_RGB48]: 'BPG_OUTPUT_FORMAT_RGB48',
  [BPG_OUTPUT_FORMAT_RGBA64]: 'BPG_OUTPUT_FORMAT_RGBA64',
  [BPG_OUTPUT_FORMAT_CMYK32]: 'BPG_OUTPUT_FORMAT_CMYK32',
  [BPG_OUTPUT_FORMAT_CMYK64]: 'BPG_OUTPUT_FORMAT_CMYK64'
};

const BPG_OUTPUT_FORMAT_PIXELSIZE = {
  [BPG_OUTPUT_FORMAT_RGB24]: 4 || 24 / 8,
  [BPG_OUTPUT_FORMAT_RGBA32]: 32 / 8,
  [BPG_OUTPUT_FORMAT_RGB48]: 48 / 8,
  [BPG_OUTPUT_FORMAT_RGBA64]: 64 / 8,
  [BPG_OUTPUT_FORMAT_CMYK32]: 32 / 8,
  [BPG_OUTPUT_FORMAT_CMYK64]: 64 / 8
};

function isBPG(buf) {
  if(buf instanceof ArrayBuffer) buf = new Uint8Array(buf);

  if(!buf || buf.length < 4) return false;

  // file_magic: 0x425047fb
  return buf[0] === 66 && buf[1] === 80 && buf[2] === 71 && buf[3] === 251;
}

class BPGLoader extends BPGDecoder {
  static Module = BPGDecoder.Module;
  static HeapU8 = null;

  constructor() {
    super();
  }

  open() {
    if(!this.open) {
      const { bpg_decoder_open } = this;
      this.handle = this.bpg_decoder_open();
      this.open = true;
    }
  }

  close() {
    if(this.open) {
      const { bpg_decoder_close } = this;
      this.bpg_decoder_close(this.handle);
      this.handle = null;
      this.open = false;
    }
  }

  async load(buffer) {
    if(typeof buffer == 'string')
      buffer = await fetch(buffer).then(response => response.arrayBuffer());

    if(!isBPG(buffer)) {
      const magic = new Uint8Array(buffer, 0, 4);
      throw new Error(
        `BPGLoader.load: is not a BPG (${magic.join(', ')}) '${[...magic]
          .map(code =>
            code < 32 || code >= 127
              ? `\\x${('0' + code.toString(16)).slice(-2)}`
              : String.fromCharCode(code)
          )
          .join('')}'`
      );
    }

    return (this.buffer = buffer).byteLength;
  }

  decode() {
    const { handle, bpg_decoder_decode } = this;
    this.open();
    let array = new Uint8Array(this.buffer);
    let ret = bpg_decoder_decode(handle, array, array.length);
    if(ret < 0) throw new Error(`bpc_decoder_decode ret=${ret}`);
    return ret;
  }

  /* prettier-ignore */ get info() {
    const { handle, bpg_decoder_get_info } = this;
    let info = this.malloc(5 * 4);
    let ret = bpg_decoder_get_info(handle, info);
    if(ret < 0) throw new Error(`bpg_decoder_get_info ret=${ret}`);
    let arrayBuf = BPGDecoder.Module.HEAPU8.buffer.slice(info);
    this.free(info);
    ret = BPGImageInfo.from(arrayBuf);
    return ret;
   }

  start(format) {
    const { handle, bpg_decoder_start, info } = this;
    if(format == undefined) format = info.format; //info.has_alpha ? BPG_OUTPUT_FORMAT_RGBA32 : BPG_OUTPUT_FORMAT_RGB24;
    this.format = format;
    this.pixelSize = BPG_OUTPUT_FORMAT_PIXELSIZE[format];
    this.lineSize = this.pixelSize * info.width;
    const { pixelSize, lineSize } = this;
    console.log(`bpg_decoder_start(${format})`, { pixelSize, lineSize });
    let ret = bpg_decoder_start(handle, format);
    if(ret < 0) throw new Error(`bpg_decoder_start(${format}) ret=${ret}`);
    return ret;
  }

  get_line() {
    const { handle, bpg_decoder_get_line, lineSize } = this;
    // console.log(`get_line()`, {  lineSize  });
    let data = this.malloc(lineSize);
    let ret = bpg_decoder_get_line(handle, data);
    if(ret < 0) throw new Error(`bpg_decoder_get_line ret=${ret}`);
    if(BPGLoader.HeapU8 == null) BPGLoader.HeapU8 = BPGLoader.Module.HEAPU8.buffer;
    let array = new Uint8Array(BPGLoader.HeapU8, data, lineSize);
    this.free(data);
    return array;
  }

  *[Symbol.iterator]() {
    const { info } = this;
    const { width, height } = info;
    for(let i = 0; i < height; i++) {
      let line = this.get_line();
      yield line;
    }
  }

  getImageData() {
    const { info } = this;
    const { width, height, has_alpha } = info;
    let ret = new ImageData(width, height);
    let { data } = ret;
    this.start();
    const { pixelSize } = this;
    let p = 0;
    for(let line of this) {
      for(let x = 0; x < line.length; x += pixelSize) {
        let r = line[x];
        let g = line[x + 1];
        let b = line[x + 2];
        let a = has_alpha ? line[x + 3] : 0xff;
        data[p++] = r;
        data[p++] = g;
        data[p++] = b;
        data[p++] = a;
      }
    }
    console.log(`p = ${p}`);
    console.log(`width * height * 4 = ${width * height * 4}`);
    return ret;
  }
}

window.addEventListener('load', async event => {
  let decoder = new BPGLoader();
  window.decoder = decoder;
  console.log('window onload', { decoder });
  let result = await decoder.load('data/me.bpg');
  console.log('decode', decoder.decode());
  console.log('info', decoder.info + '');
  let y = 0;
  let { width, height } = decoder.info;
  let { pixelSize } = decoder;

  let canvas = document.querySelector('#output');

  canvas.setAttribute('width', width);
  canvas.setAttribute('height', height);

  let ctx = canvas.getContext('2d');

  let imageData = (window.imageData = decoder.getImageData());

  ctx.putImageData(imageData, 0, 0);
});
