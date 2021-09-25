import fs from 'fs';
import * as os from 'os';
import { Console } from 'console';
import { mmap, munmap, PROT_READ, PROT_WRITE, MAP_PRIVATE, MAP_SHARED, MAP_ANONYMOUS } from 'mmap';
import { extendArray, ArrayExtensions, SyscallError, arrayToBitfield, atob, atomToString, atomToValue, bitfieldToArray, btoa, compileScript, concatArrayBuffer, dupArrayBuffer, evalBinary, getByteCode, getClassAtom, getClassConstructor, getClassCount, getClassID, getClassName, getClassProto, getCommandLine, getCurrentWorkingDirectory, getExecutable, getFileDescriptor, getOpCodes, getPerformanceCounter, getProcMaps, getProcMounts, getProcStat, getPrototypeChain, getRootDirectory, getegid, geteuid, getgid, getpid, getppid, getsid, getuid, hrtime, readObject, resizeArrayBuffer, setegid, seteuid, setgid, setuid, toArrayBuffer, toPointer, toString, uname, valuePtr, valueTag, valueToAtom, valueType, writeObject, inspect, errors, types, isObject, hasBuiltIn, format, formatWithOptions, assert, setInterval, clearInterval, memoize, once, waitFor, define, weakAssign, getConstructorChain, hasPrototype, filter, randInt, ansiStyles } from 'util';

function PrintSlice(arr, start, end, linelen) {
  let i, j;
  let offset = start % linelen;

  for(i = start - offset; i < end; i += linelen) {
    let address = i.toString(16).padStart(8, '0');
    let data = '';
    let chars = '';

    for(j = 0; j < linelen; j++) {
      const pos = i + j;
      const byte = arr[pos];

      if(j % 8 == 0) data += ' ';

      if(pos < start || pos >= end) {
        data += '   ';
        chars += ' ';
      } else {
        data += ' ' + byte.toString(16).padStart(2, '0');
        chars += byte >= 0x20 && byte < 0x80 ? String.fromCharCode(byte) : '.';
      }
    }
    console.log(address, data, ` |${chars}|`);
  }
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: 1,
      compact: 1,
      customInspect: true,
      maxStringLength: 100,
      maxArrayLength: 64
    }
  });

  let fd = os.open(args[0], os.O_RDONLY);
  console.log('fd', fd);
  let [st, err] = os.stat(args[0]);
  console.log('st', st);
  const { size } = st;
  console.log('size', size);

  console.log('typeof 1n', typeof 1n);

  let map = mmap(0, size, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);

  console.log('map', map);

  // console.log('array', array);
  let array = new Uint8Array(map);

  //
  //  let string = toString(map);
  /*  let string = array.reduce((acc,byte) => acc+String.fromCodePoint(byte), '');

    console.log('string', string.length);*/

  for(let [offset, length, patch] of [
    [0x3653ce, 31, [0x48, 0x31, 0xc0, 0xc3]],
    [0x34f5f0, 23, [0x48, 0x31, 0xc0, 0xc3]],
    [0x366d5e, 14, [0x48, 0x31, 0xc0, 0xc3]],
    [0x35bccb, 13, [0x90, 0x90, 0x90, 0x90, 0x90]],
    [0x35bce6, 17, [0x90, 0x90, 0x90, 0x90, 0x90]],
    [0x367171, 12, [0x48, 0x31, 0xc0, 0x48, 0xff, 0xc0, 0xc3]]
  ]) {
    let range = new Uint8Array(map, offset, length);
    let bin = map.slice(offset, offset + length);
    console.log(bin);

    let outFd = os.open(`0x${offset.toString(16).padStart(8, '0')}-${length}.bin`, os.O_TRUNC | os.O_CREAT | os.O_WRONLY, 0o644);

    os.write(outFd, bin, 0, length);
    os.close(outFd);

    console.log(`\n${length} bytes from ${offset}:\n`);
    PrintSlice(array, offset, offset + length, 16);

    for(let i = 0; i < patch.length; i++) {
      range[i] = patch[i];
    }
    console.log(`\nPatched:\n`);
    PrintSlice(array, offset, offset + length, 16);
  }

  let outFd = os.open(`sublime_text`, os.O_TRUNC | os.O_CREAT | os.O_WRONLY, 0o755);
  os.write(outFd, map, 0, size);
  os.close(outFd);

  munmap(map);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
}
