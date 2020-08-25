import Util from './lib/util.js';

function QuickJSFileSystem(std, os) {
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
      let buf, size, res = { errno: 0 };
      let file = std.open(filename, 'r', res);
      if(!res.errno) {
        file.seek(0, std.SEEK_END);
        size = file.tell();
        file.seek(0, std.SEEK_SET);
        buf = new ArrayBuffer(size);
        file.read(buf, 0, size);
        //buf = file.readAsString(/*size*/);
        file.close();
        return ArrayBufToString(buf);
      }
      return res.errno;
    },
    writeFile(filename, data, overwrite = true) {
      let buf, bytes, res = { errno: 0 };
      let file = std.open(filename, overwrite ? 'w' : 'wx', res);
      // console.log('writeFile', filename, data.length, file, res.errno);
      if(!res.errno) {
        buf = typeof data == 'string' ? StringToArrayBuf(data) : data;
        bytes = file.write(buf, 0, buf.byteLength);
        file.flush();
        res = file.close();
        if(res < 0) return res;
        return bytes;
      }
      return -res.errno;
    },
    exists(filename) {
      let file = std.open(filename, 'r');
      let res = file != null;
      if(file) file.close();
      return res;
    },
    size(filename) {
      let bytes, res = { errno: 0 };
      let file = std.open(filename, 'r', res);
      if(!res.errno) {
        file.seek(0, std.SEEK_END);
        bytes = file.tell();
        res = file.close();
        if(res < 0) return res;
        return bytes;
      }
      return -res.errno;
    },
    realpath(filename) {
      let [str, err] = os.realpath(filename);
      if(!err) return str;
      return err;
    }
  };
}

function NodeJSFileSystem(fs) {
  return {
    readFile(filename) {
      return fs.readFileSync(filename, { encoding: 'utf-8' });
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

async function PortableFileSystem() {
  let fs, err;

  try {
    fs = QuickJSFileSystem(...(await Promise.all([import('std'), import('os')])));
  } catch(error) {
    err = error;
  }

  if(fs && !err) return fs;
  err = null;

  try {
    fs = NodeJSFileSystem(await import('fs'));
  } catch(error) {
    err = error;
  }

  if(fs && !err) return fs;
}

async function main() {
  let outputFile = 'test.txt';
  let filesystem = await PortableFileSystem();
  let err;

  try {
    // console.log(`filesystem :`, filesystem);
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
