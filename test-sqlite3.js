import { sqlite3_open } from './sqlite3.js';
function main(...args) {
  let ret,
    a = new Uint32Array(2);

  ret = sqlite3_open('/home/roman/.config/google-chrome/Default/History', a.buffer);

  console.log('ret', ret);
}

main(...scriptArgs.slice(1));