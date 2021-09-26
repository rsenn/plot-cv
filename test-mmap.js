import * as os from 'os';
import * as fs from 'fs';
import { exec,spawn } from 'child_process';
import { Console } from 'console';
import { mmap, munmap, PROT_READ, PROT_WRITE, MAP_PRIVATE, msync, MS_SYNC } from 'mmap';
import { searchArrayBuffer, dupArrayBuffer, copyArrayBuffer, toString, toPointer, format } from 'util';

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

function ExecTool(cmd,...args) {
let child = spawn(cmd, args, { stdio: [0,'pipe', 2] });
     console.log('ExecTool',{args,child});
 let [stdin, stdout, stderr] = child.stdio;
let r;
let b = new ArrayBuffer(1024);
r=child.wait();
    //console.log('r',r);

r  = os.read(stdout, b, 0, 1024);
let data = b.slice(0,r);
let str = toString(data);
    //console.log('r',{data,str});

return parseInt(str);
}

function RVA2Offset(file, rva) {
return   ExecTool('elflist', file, '-a', rva);
}

function Offset2RVA(file, offset) {
return   ExecTool('elflist', file, '-o', offset);
}

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: 1,
      compact: 2,
      customInspect: false,
      maxStringLength: 100,
      maxArrayLength: 64,
      numberBase: 16
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

  const patterns = [
    ['e8 ? ? ? ? 49 8b bf ? ? ? ? 85 c0', 'License Validity Checking (Aka IsValidLicense)', '48 31 c0 c3'],
    ['e8 ? ? ? ? 48 89 5c 24 ? 48 8b b3', 'Invalidation/Validation Functions - Pattern 1', '90 90 90 90 90'],
    ['e8 ? ? ? ? bf ? ? ? ? e8 ? ? ? ? 83 25', 'Invalidation/Validation Functions - Pattern 2', '90 90 90 90 90'],
    ['55 41 56 53 41 89 f6 48 89 fd 6a 28', 'Server Validation Thread', '48 31 c0 48 ff c0 c3'],
    ['e8 ? ? ? ? 3d ? ? ? ? 75 12', 'License Validity Checking', '48 31 c0 c3' /*'48 c7 c0 19 01 00 00'*/],
    [
      '41 57 41 56 56 57 55 53 b8 28 21 00 00',
      'RSA Key Patch (allows any key in right format to work)',
      '33 c0 fe c0 c3 57 55 53 b8 28 21 00 00'
    ],
    [
      '6c 69 63 65 6e 73 65 2e 73 75 62 6c 69 6d 65 68 71 2e 63 6f 6d',
      'Disable License Check',
      '73 75 62 6c 69 6d 65 68 71 2e 6c 6f 63 61 6c 68 6f 73 74 00 00'
    ],
    [
      '48 89 e7 be ? ? ? ? ba ? ? ? ? e8 ? ? ? ? 45 84 e4 74 12',
      'Upgrade required',
      (map, offset, length) => {
        let buf = dupArrayBuffer(map, offset, length);
      }
    ]
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

   if(results.length > 1) 
    throw new Error(`Multiple results for pattern '${str}': ${results.map(r => '0x'+r.toString(16)).join(', ')}`);
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
      console.log('base + pos', base + pos);
      offsets.push(base + pos);

      base += pos + 1;
      range = range.slice(pos + 1);
    }
    return offsets;
  }
  const results = patterns.map(([pattern, description], i) => {
    console.log(`Searching [${i}] ${description}`);
    return searchAll(pattern);
  });
 
  const offsets = results.map(r => {
if(typeof r[0] == 'number') {
  let rva=Offset2RVA(args[0], r[0]);
  console.log(`RVA`,rva);
}
    return r[0];
  });
  console.log('results', { ...results });

  offsets.forEach((offset, i) => {
    const rep = replacements[i];

    if(typeof rep == 'function') {
      rep(map, offset, Pattern(patterns[i][0]).length);
    }
    if(typeof rep == 'string') {
      const { buffer } = new Uint8Array(replacements[i].split(' ').map(s => (s == '?' ? 0 : +(`0x` + s))));
      if(offset !== null) {
     //   console.log('patch', { map, offset, buffer });
        const dst = dupArrayBuffer(map, offset, buffer.byteLength);
        console.log(`dst[${i}]`, dst);
        copyArrayBuffer(dst, buffer);
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
  let r = os.write(outFd, map, 0, size);
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
