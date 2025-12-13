import { spawn } from 'child_process';
import * as os from 'os';
import { dupArrayBuffer, memcpy, searchArrayBuffer, toPointer, toString } from './lib/misc.js';
import { Console } from 'console';
import { MAP_PRIVATE, mmap, mprotect, munmap, PROT_READ, PROT_WRITE } from 'mmap';

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

function ExecTool(cmd, ...args) {
  let child = spawn(cmd, args, { stdio: [0, 'pipe', 2] });
  let [stdin, stdout, stderr] = child.stdio;
  let r;
  let b = new ArrayBuffer(1024);
  r = child.wait();
  // console.log('ExecTool', { args, child });

  r = os.read(stdout, b, 0, 1024);
  let data = b.slice(0, r);
  let str = toString(data);

  return parseInt(str);
}

function RVA2Offset(file, rva) {
  return ExecTool('elflist', file, '-a', rva);
}

function Offset2RVA(file, offset) {
  return ExecTool('elflist', file, '-o', offset);
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: 2,
      compact: 2,
      customInspect: false,
      maxStringLength: 100,
      maxArrayLength: 64,
      numberBase: 16
    }
  });

  let fd = os.open(args[0], os.O_RDONLY);
  console.log('fd', fd);
  console.log('args[0]', args[0]);
  let [st, err] = os.stat(args[0]);
  console.log('st', st);

  const { size } = st;
  console.log('size', size);

  let map = mmap(0, size, PROT_READ | PROT_WRITE, MAP_PRIVATE, fd, 0);

  console.log('map', toPointer(map));
  console.log('map', map);

  // console.log('array', array);
  let array = new Uint8Array(map);

  const patterns = [
    ['e8 ? ? ? ? 49 8b bf ? ? ? ? 85 c0', 'License Validity Checking (Aka IsValidLicense)', '48 31 c0 c3'],
    ['e8 ? ? ? ? 48 89 5c 24 ? 48 8b b3', 'Invalidation/Validation Functions - Pattern 1', '90 90 90 90 90'],
    ['e8 ? ? ? ? bf ? ? ? ? e8 ? ? ? ? 83 25', 'Invalidation/Validation Functions - Pattern 2', '90 90 90 90 90'],
    ['55 41 56 53 41 89 f6 48 89 fd 6a 28', 'Server Validation Thread', '48 31 c0 48 ff c0 c3'],
    ['e8 ? ? ? ? 3d ? ? ? ? 75 12', 'License Validity Checking', '48 31 c0 c3' /*'48 c7 c0 19 01 00 00'*/],
    ['41 57 41 56 56 57 55 53 b8 28 21 00 00', 'RSA Key Patch (allows any key in right format to work)', '33 c0 fe c0 c3 57 55 53 b8 28 21 00 00'],
    ['6c 69 63 65 6e 73 65 2e 73 75 62 6c 69 6d 65 68 71 2e 63 6f 6d', 'Disable License Check', '73 75 62 6c 69 6d 65 68 71 2e 6c 6f 63 61 6c 68 6f 73 74 00 00'],
    [
      '48 89 e7 be ? ? ? ? ba ? ? ? ? e8 ? ? ? ? 45 84 e4 74 12',
      'Upgrade required',
      (map, offset, length) => {
        let buf = dupArrayBuffer(map, offset, length);
        /*console.log('map', toPointer(map));
        console.log('buf', toPointer(buf) - toPointer(map));
        console.log('buf', buf);
        console.log('offset', offset);
        console.log('length', length);*/

        let arr = new Uint32Array(buf, 4, 1);

        //console.log('arr[0]', arr[0]);

        //++arr[0];
        //console.log('arr[0]', arr[0]);
      }
    ],
    ['85 db 74 29 0f be b3 0d cc 29', 'Upgrade', '85 db eb'],
    [
      '50 be fc d8 20 00 ba ab 10 21 00 31 ff e8 bd 79 18 00 84 c0 74 16 48 8b 05 b9 ed 4a 00 be a7 ae 21 00 31 ff 31 d2 31 c9 41 58 ff e0 58',
      'Patch-Sublime-License-Message',
      '90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90 90'
    ],
    ['41 57 41 56 56 57 55 53 B8 28 21 00 00', 'RSA Key Patch (allows any key in right format to work)', '33 C0 FE C0 C3 57 55 53 B8 28 21 00 00']
    //['c3', "Ret"]
  ];
  const replacements = patterns.map(([s, m, r]) => r);

  function Pattern(str) {
    return new Uint8Array(str.split(' ').map(s => (s == '?' ? 0 : +(`0x` + s))));
  }

  function Mask(str) {
    return new Uint8Array(str.split(' ').map(s => (s == '?' ? 0 : 0xff)));
  }

  function searchPattern(str) {
    const results = searchAll(str);

    if(results.length > 1) throw new Error(`Multiple results for pattern '${str}': ${results.map(r => '0x' + r.toString(16)).join(', ')}`);
    return results[0];
  }
  function searchAll(str) {
    const needle = Pattern(str);
    const mask = Mask(str);
    /*console.log('needle',  needle);
    console.log('mask', mask);
*/
    let pos,
      base = 0,
      offsets = [],
      range = map;
    for(;;) {
      pos = searchArrayBuffer(range, needle.buffer, mask.buffer);
      if(pos === null) break;
      offsets.push(base + pos);

      base += pos + 1;
      range = range.slice(pos + 1);
    }
    return offsets;
  }
  const results = patterns.map(([pattern, description], i) => {
    console.log(`[${i}] Searching ${description} [ ${pattern} ]`);
    let results = searchAll(pattern).map(offset => ({
      offset,
      rva: Offset2RVA(args[0], offset)
    }));

    //console.log(`results[${results.length}]`, results);
    return [description, results];
  });

  console.log('results', console.config({ compact: 3, depth: Infinity }), results);

  let offsets = results.map(([desc, r]) => {
    const { offset, rva } = r[0] ?? {};
    if(offset == 'number') {
      console.log(`RVA`, rva);
    }
    return [desc, offset];
  });

  //offsets = [offsets[0], offsets[3]]
  offsets = offsets.map((a, i) => [i, a]);
  console.log(
    'offsets',
    console.config({ /*numberBase: 10, */ compact: 1 }),
    offsets.filter(([i, [name, offset]]) => offset !== undefined).reduce((acc, [i, a]) => ({ ...acc, [i]: a }), {})
  );

  offsets = offsets.filter(([i, [name, offset]]) => offset !== undefined);

  console.log('offsets', offsets);

  // offsets = offsets.slice(0, 4);
  offsets.forEach(([i, [desc, offset]]) => {
    const rep = replacements[i];

    if(typeof rep == 'function') {
      console.log('offset', offset);
      console.log('patterns[i]', patterns[i]);

      rep(map, offset, Pattern(patterns[i][0]).length);
    }
    if(typeof rep == 'string') {
      const { buffer } = new Uint8Array(replacements[i].split(' ').map(s => (s == '?' ? 0 : +(`0x` + s))));
      if(offset !== null) {
        const dst = dupArrayBuffer(map, offset, buffer.byteLength);
        const diff = toPointer(dst) - toPointer(map);
        console.log(`patch[${i}]`, {
          map: +toPointer(map),
          dst: +toPointer(dst),
          offset,
          diff
        });

        mprotect(dst, dst.byteLength, PROT_WRITE);
        console.log(`replacement`, new Uint8Array(buffer));

        memcpy(dst, buffer);
        console.log(`dst[${i}]`, dst);
      }
    }
  });
  //  let string = toString(map);
  /*  let string = array.reduce((acc,byte) => acc+String.fromCodePoint(byte), '');

    console.log('string', string.length);*/

  /*  for(let [offset, length, patch] of [
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
  }*/

  let outFd = os.open(`sublime_text`, os.O_TRUNC | os.O_CREAT | os.O_WRONLY, 0o755);

  mprotect(map, size, PROT_READ);

  let r = os.write(outFd, map, 0, size);
  /*console.log('map', toPointer(map));
  console.log('map', map);
  console.log('size', size);*/

  console.log(`Wrote ${r} bytes`);
  os.close(outFd);

  //msync(map, null, MS_SYNC);
  munmap(map);
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
}