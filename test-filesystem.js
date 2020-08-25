import Util from './lib/util.js';

function QuickJSFileSystem(std, os) {
  console.log(`std.open:`, std.open);
  console.log(`os.realpath:`, os.realpath);

  const CharWidth = {
    1: Uint8Array,
    2: Uint16Array,
    4: Uint32Array
  };

  function ArrayBufToString(buf, bytes = 1) {
    return String.fromCharCode.apply(null, new CharWidth[bytes](buf));
  }

  function StringToArrayBuf(str, bytes = 1) {
    const buf = new ArrayBuffer(str.length * bytes);
    const view = new CharWidth[bytes](buf);
    for(let i = 0, strLen = str.length; i < strLen; i++) view[i] = str.charCodeAt(i);
    return buf;
  }

  return {
    readFile(filename) {
      let errorObj = { errno: 0 };
      let file = std.open(filename, 'r', errorObj);
      let size, b;
      if(!errorObj.errno) {
        file.seek(0, std.SEEK_END);
        size = file.tell();
        file.seek(0, std.SEEK_SET);
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
      let file = std.open(filename, overwrite ? 'w' : 'wx', errorObj);
      let b, n, r;
      console.log('writeFile', filename, data.length, file, errorObj.errno);
      if(!errorObj.errno) {
        b = typeof data == 'string' ? StringToArrayBuf(data) : data;
        n = file.write(b, 0, b.byteLength);
        file.flush();
        r = file.close();
        if(r < 0) errorObj.errno = -r;
        else return n;
      }
      return -errorObj.errno;
    },
    exists(filename) {
      let errorObj = { errno: 0 };
      let file = std.open(filename, 'r', errorObj);
      if(!errorObj.errno) {
        file.close();
        return true;
      }
      return false;
    },
    size(filename) {
      let errorObj = { errno: 0 };
      let file = std.open(filename, 'r', errorObj);
      let size;
      if(!errorObj.errno) {
        file.seek(0, std.SEEK_END);
        size = file.tell();

        file.close();
        return size;
      }
      return -errorObj.errno;
    },
    realpath(filename) {
      let [str, err] = os.realpath(filename);
      if(!err) return str;
      return err;
    }
  };
}

function NodeJSFileSystem(fs) {
  console.log(`fs.readFileSync:`, fs.readFileSync);

  return {
    readFile(filename) {
      let data = fs.readFileSync(filename);
      return typeof data.toString == 'function' ? data.toString() : data;
    },
    writeFile(filename, data, overwrite = true) {
      let fd = fs.openSync(filename, overwrite ? 'w' : 'wx');
      let r = fs.writeSync(fd, data);
      fs.closeSync(fd);
      return r;
    },
    exists(filename) {
      return fs.existsSync(filename);
    },
    realpath(filename) {
      return fs.realpathSync(filename);
    },
    size(filename) {
      let st = fs.statSync(filename);
      return st.size;
    }
  };
}
async function main() {
  let outputFile = 'test.txt';
  let filesystem;
  let err;
  try {
    filesystem = QuickJSFileSystem(await import('std'), await import('os'));
  } catch(error) {
    err = error;
  }
  if(!filesystem) {
    err = null;
    try {
      filesystem = NodeJSFileSystem(await import('fs'));
    } catch(error) {
      err = error;
    }
  }
  if(err) {
    console.log(`error:`, err);
    return;
  }
  err = null;

  try {
    console.log(`filesystem :`, filesystem);
    console.log(`filesystem.writeFile('${outputFile}', ...):`, filesystem.writeFile(outputFile, 'BLAH\nthis is a test!\n\n'));
    console.log(`filesystem.readFile('test-filesystem.js'):`, Util.abbreviate(filesystem.readFile('test-filesystem.js')));
    console.log(`filesystem.realpath('/proc/self'):`, filesystem.realpath('/proc/self'));
    console.log(`filesystem.exists('${outputFile}'):`, filesystem.exists(outputFile));
    console.log(`filesystem.size('${outputFile}'):`, filesystem.size(outputFile));
    console.log(`filesystem.exists('blah.txt'):`, filesystem.exists('blah.txt'));
  } catch(error) {
    err = error;
  }
  if(err) console.log(`error:`, err);
}

main();
