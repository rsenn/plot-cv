import Alea from './lib/alea.js';
import * as path from './lib/path.js';
import tXml from './lib/tXml.js';
import * as zlib from './quickjs/qjs-ffi/examples/zlib.js';
import * as ffi from 'ffi';
let prng = new Alea().seed(Date.now());

/*class Uint64 {
  constructor(buffer, offset) {
    Util.define(this, { buffer, offset });
  }

  set lo(v) {
    new Uint32Array(this.buffer, this.offset, 2)[0] = v;
  }
  get lo() {
    return new Uint32Array(this.buffer, this.offset, 2)[0];
  }
  set hi(v) {
    new Uint32Array(this.buffer, this.offset, 2)[1] = v;
  }
  get hi() {
    return new Uint32Array(this.buffer, this.offset, 2)[1];
  }

  set big(v) {
    let b = BigInt(v);
    this.lo = b & 0xffffffffn;
    this.hi = (b >> 32n) & 0xffffffffn;
  }
  get big() {
    return (BigInt(this.hi) << 32n) | BigInt(this.lo);
  }

  set pointer(v) {
    if(typeof(v) == 'object' && v != null && v instanceof ArrayBuffer)
      v = ffi.toPointer(v);    
    this.big = v;
  }

  get pointer() {
    return '0x' + ('0000000000000000' + this.big.toString(16)).slice(-16);
  }

  [Symbol.toPrimitive](hint) {
    return '0x' + ('0000000000000000' + this.big.toString(16)).slice(-16);
  }
}*/

async function readBJSON(filename) {
  let data = filesystem.readFileSync(filename, null);
  let obj = await import('bjson.so')
    .then(({ read }) => read(data, 0, data.byteLength))
    .catch(err => console.log(err));
  return obj;
}

function deflate(buffer, level = 9) {
  let zstr = new zlib.z_stream();
  let ret = zlib.deflateInit2(zstr, level, zlib.Z_DEFLATED, 15 + 16, 8, zlib.Z_DEFAULT_STRATEGY);
  console.log('zlib.deflateInit2() =', ret);
  console.log('ffi.toPointer(buffer) =', ffi.toPointer(buffer));
  //console.log('ffi.toString(buffer) =', ffi.toString(ffi.toPointer(buffer)));
  const p = ffi.toPointer(buffer);
  const s = ffi.toString(p);
  zstr.next_in = p;
  zstr.avail_in = buffer.byteLength;

  let hdr = new zlib.gz_header({
    text: 0,
    time: Math.round(Date.now() / 1000),
    os: 1,
    name: 'test.c',
    name_max: 'test.c'.length + 1
  });
  console.log('hdr =', hdr.toString());
  zlib.deflateSetHeader(zstr, hdr);
  let out = new ArrayBuffer(buffer.byteLength * 2);

  zstr.next_out = out;
  zstr.avail_out = out.byteLength;
  console.log('zstr =', zstr.toString()); //  .next_in, zstr.avail_in, zstr.next_out, zstr.avail_out);

  ret = zlib.deflate(zstr, 0);
  console.log('zlib.deflate() =', ret);
  return out;
}

function readXML(filename) {
  //console.log('readXML', filename);
  let data = filesystem.readFileSync(filename);
  let xml = tXml(data);
  //console.log('xml:', xml);
  return xml;
}

function WriteFile(name, data) {
  console.log('WriteFile', { name });
  if(typeof data == 'string' && !data.endsWith('\n')) data += '\n';

  let raw = filesystem.bufferFrom(data);
  console.log('raw =', raw && raw.byteLength);
  let compressed = deflate(raw);

  console.log('compressed =', compressed);
  console.log('compressed.length =', compressed && compressed.byteLength);

  if(name == '-' || typeof name != 'string') {
    //  let stdout = filesystem.fdopen(1, 'r');
    let buffer = data instanceof ArrayBuffer ? data : filesystem.bufferFrom(data);
    filesystem.write(1, buffer, 0, buffer.byteLength);
    return;
  }

  if(Array.isArray(data)) data = data.join('\n');
  // if(typeof data != 'string') data = '' + data;

  //
  filesystem.writeFile(name, data);
  console.log(`Wrote ${name}: ${data.length} bytes`);
}

const push_back = (arr, ...items) => [...(arr || []), ...items];

const push_front = (arr, ...items) => [...items, ...(arr || [])];

const tail = arr => arr[arr.length - 1];

async function main(...args) {
  let params = Util.getOpt(
    {
      output: [true, null, 'o'],
      input: [true, null, 'i'],
      xml: [true, null, 'x'],
      json: [true, null, 'j'],
      tag: [true, null, 't'],
      include: [true, null, 'I'],
      exclude: [true, null, 'X'],
      'no-remove-empty': [false, null, 'E'],
      '@': 'input,output,xml'
    },
    scriptArgs.slice(1)
  );
  console.log('main', args, params);
  if(params['@'].length == 0 && !params.input) {
    console.log(`Usage: ${scriptArgs[0]} <...files>`);
    return 1;
  }

  let { input: filename = '-' } = params;

  let basename = path.basename(filename, /\.[^.]+$/g);
  let outfile = params.output || '-'; /* ||   basename + '.out.xml'*/
  let xmlfile = params.xml || basename + '.out.xml';
  let jsonfile = params.json || basename + '.out.json';
  let tagkey = params.tag || 'type';
  let { include, exclude } = params;

  let cmds = args;
  let newObj = {};
  let xmlData;

  try {
    let js = await readBJSON(filename);
    let json = JSON.stringify(js, null, '  ');

    //   await import('bjson.so').then(({ read, write }) => json = write(xml)).catch(err => console.error(err));

    WriteFile(outfile, json);

    // WriteFile(xmlfile, toXML(xmlData));
  } catch(err) {
    console.log('error:', { err });
    throw err;
  }
}

main(...scriptArgs.slice(1));