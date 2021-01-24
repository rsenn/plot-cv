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

window.importObject = importObject;

function loadWebAssembly(fileName, impObj = {}) {
  return fetch(fileName)
    .then(response => response.arrayBuffer())
    .then(buffer => convertBufferStringToInstance(buffer, impObj));
  /* .then(buffer => WebAssembly.compile(buffer))
    .then(module => {return new WebAssembly.Instance(module, impObj.imports) })*/
}

let bpgdec;

async function convertBufferStringToInstance(bufferArrayString, impObj = {}) {
  const buffer = new Uint8Array(typeof bufferArrayString == 'string' ? bufferArrayString.split(',') : bufferArrayString
  );
  const module = await WebAssembly.compile(buffer);
  console.log('convertBufferStringToInstance', { module, impObj });
  return WebAssembly.instantiate(buffer, impObj);
}

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
    g,
    f,
    stackAlloc,
    stackRestore,
    stackSave,
    bpg_decoder_close,
    bpg_decoder_decode,
    bpg_decoder_open,
    bpg_decoder_get_line,
    bpg_decoder_get_frame_duration,
    bpg_decoder_start,
    bpg_decoder_get_info,
    e,
    d
  } = wasm.instance.exports;

  /*  let bpgdec = {free, g, f, stackAlloc, stackRestore, stackSave, bpg_decoder_close, bpg_decoder_decode, bpg_decoder_open, bpg_decoder_get_line, bpg_decoder_get_frame_duration, bpg_decoder_start, bpg_decoder_get_info, e, d } || Object.fromEntries(Object.keys(functionNames).map(name => [functionNames[name], wasm.instance.exports[name]]));*/
   window.bpg = {
    free,
    g,
    f,
    stackAlloc,
    stackRestore,
    stackSave,
    bpg_decoder_close,
    bpg_decoder_decode,
    bpg_decoder_open,
    bpg_decoder_get_line,
    bpg_decoder_get_frame_duration,
    bpg_decoder_start,
    bpg_decoder_get_info,
    e,
    d
  };
  console.log(' window.bpg:', window.bpg);
});
