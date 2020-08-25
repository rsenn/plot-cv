import { open, SEEK_END, SEEK_SET } from 'std';
import { realpath as OS_realpath } from 'os';

function ArrayBufToString(buf, bytes = 1) {
  const ctor = bytes == 1 ? Uint8Array : bytes == 2 ? Uint16Array : Uint32Array;
  return String.fromCharCode.apply(null, new ctor(buf));
}
function StringToArrayBuf(str, bytes = 1) {
  const ctor = bytes == 1 ? Uint8Array : bytes == 2 ? Uint16Array : Uint32Array;
  const buf = new ArrayBuffer(str.length * bytes);
  const bufView = new ctor(buf);
  for(var i = 0, strLen = str.length; i < strLen; i++) bufView[i] = str.charCodeAt(i);
  return buf;
}

const filesystem = {
  readFile(filename) {
    let errorObj = { errno: 0 };
    let file = open(filename, 'r', errorObj);
    let size, b;
    if(!errorObj.errno) {
      file.seek(0, SEEK_END);
      size = file.tell();
      file.seek(0, SEEK_SET);
      b = new ArrayBuffer(size);
      file.read(b, 0, size);
      //b = file.readAsString(/*size*/);
      file.close();
      return ArrayBufToString(b);
    }
    return errorObj.errno;
  },
  writeFile(filename, data, overwrite = true) {
    let errorObj = { errno: 0 };
    let file = open(filename, overwrite ? 'w' : 'wx', errorObj);
    if(!errorObj.errno) {
      let b = typeof data == 'string' ? StringToArrayBuf(data) : data;
      file.write(b, 0, b.byteLength);
      file.flush();
      let r = file.close();
      if(r < 0) errorObj.errno = -r;
    }
    return errorObj.errno;
  },
  exists(filename) {
    let errorObj = { errno: 0 };
    let file = open(filename, 'r', errorObj);
    if(!errorObj.errno) {
      file.close();
      return true;
    }
    return false;
  },
  realpath(filename) {
    let [str, err] = OS_realpath(filename);
    if(!err) return str;
    return err;
  }
};

function main() {
  filesystem.writeFile('test.txt', 'BLAH\nthis is a test!\n\n');
  let str = filesystem.readFile('test-filesystem.js');
  console.log('str:', str);
  console.log('str.length:', str.length);
  console.log('realpath:', filesystem.realpath('/opt/sublime_text_3'));
}

main();
