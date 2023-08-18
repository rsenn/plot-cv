import { popenAndRead, stringToArrayBuffer } from './popen.so';
import * as std from 'std';

const input = std.in.getline();
const command = input.replace(/^['"`]?(.*)['"`]?$/, '$1');

console.log(new Uint8Array(stringToArrayBuffer('[ 1234, 0 ]')));

//std.err.puts(`Executing: ${command}\n`);

popenAndRead(command, data => {
  const array = new Uint8Array(data);

  std.out.puts(`[ ${array.map(n => `${n}`.padStart(3, ' ')).join(', ')} ]\n`);
  std.out.flush();
});