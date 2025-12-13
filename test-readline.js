import * as rl from './readline.js';
import { Console } from 'console';
function hex(num) {
  return '0x' + ('0000000' + num.toString(16)).slice(-16);
}

Number.prototype.toHex = function() {
  return hex(this);
};

function main(...args) {
  globalThis.console = new Console({
    inspectOptions: {
      colors: true,
      depth: Infinity,
      maxArrayLength: 100,
      breakLength: 10000,
      compact: 2 /*,
      customInspect: true*/
    }
  });
  let line = rl.readline('test-readline> ');
}

try {
  main(...scriptArgs.slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
  std.exit(1);
} finally {
  console.log('SUCCESS');
}