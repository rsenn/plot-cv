import { Util } from './lib/util.js';

console.log(Util.escape(read('/proc/self/cmdline')));
console.log(Util.escape(read('/etc/hosts')));
console.log(globalThis.options());
console.log(Util.getArgv());
