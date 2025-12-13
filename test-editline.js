import * as el from './editline.js';
import * as ffi from 'ffi';
function hex(num) {
  return '0x' + ('0000000' + num.toString(16)).slice(-16);
}

Number.prototype.toHex = function() {
  return hex(this);
};

async function main(...args) {
  console.log('el.handle', el.handle);

  let rl_meta_chars = ffi.dlsym(el.handle, 'rl_meta_chars');
  console.log('rl_meta_chars', rl_meta_chars.toHex());

  let b = ffi.toArrayBuffer(rl_meta_chars, 4);
  console.log('b', b);
}

main(...scriptArgs.slice(1));