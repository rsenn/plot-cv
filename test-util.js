import { Util } from './lib/util.js';
async function main(...args) {
  function test(arg) {
    console.log('function test', { arg });
    return -1;
  }

  let trace = Util.trace(test);

  trace('testarg');

  console.log('Util.getPlatform():', Util.getPlatform());
  console.log('scriptArgs:', scriptArgs);
  console.log('process.argv:', process.argv);
  console.log('Util.scriptName():', Util.scriptName());
  //  console.log('Util.now:', await Util.now);
  console.log('Util.now:', Util.now);
  console.log('Util.waitFor:', Util.waitFor);
  for(let i = 0; i < 100; i += 10) console.log('Util.waitFor(10):', await Util.waitFor(10));
  let now;
  console.log('Util.now:', (now = Util.now));
  console.log('Util.getNow():', Util.getNow());
  console.log('Util.isAsync(Util.now):', Util.isAsync(Util.now));
  console.log('now:', now);
  //console.log('Util.now():', await now());

  let obj = JSON.parse('{"a":1,"b":2}');

  console.log('obj:', obj);
  console.log(`obj=${obj}`);
  console.log(`{a:1,b:2}: ${{ a: 1, b: 2 }}`);
  console.log(`await import('os'):`, Object.keys(await import('os')));
  console.log(`await import('std'):`, await import('std').catch(err => (console.log(err), err)));
  //console.log(`await import('ffi.so'):`, await import('ffi.so'));
  const { O_RDONLY, O_WRONLY, O_RDWR, O_APPEND, O_CREAT, O_EXCL, O_TRUNC, O_TEXT } = filesystem;
  let f = filesystem.openSync('test.txt', O_WRONLY | O_CREAT);
  console.log('write:', filesystem.writeSync(f, 'test file\n'));
  filesystem.closeSync(f);

  f = filesystem.openSync('test.txt', O_WRONLY | O_CREAT | O_EXCL);
  console.log('write:', filesystem.writeSync(f, 'overwritten test file\n'));
  filesystem.closeSync(f);

  process.exit(0);
}

try {
  await main(...scriptArgs.slice(1)).catch(error => console.log('ERROR', error ? error.message + '\n' + error.stack : error));
} catch(error) {
  console.log('ERROR', error ? error.message + '\n' + error.stack : error);
}