import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

async function main(...args) {
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200
  });

  console.log(
    `console`,
    inspect(console, { customInspect: true, showHidden: true, compact: false })
  );

  let c = console;
  console.log('console', c.log);
  console.log('globalThis["console"]', globalThis['console']);
  console.log('globalThis["console"]', typeof globalThis['console']);
  console.log('globalThis["console"] == undefined', globalThis['console'] == undefined);
  console.log('console.log', console.log);

  if(1) {
    let args = Util.getArgs();
    let path = args[0];
    let fn = function(...args) {
      return args;
    };
    let boundFn = fn.bind(fn);
    let map = new Map([
      ['a', 1],
      ['b', 2],
      ['c', 3]
    ]);

    boundFn.test = 'TEST';
    boundFn.num = 1234;

    let obj = Object.assign(Object.create(null), {
      null: null,
      0: 0,
      1: 1,
      false: false,
      true: true
    });
    let obj2 = Util.define({}, { test: 'ABC', number: 1234 });
    let weakMap = new WeakMap([
      [obj, 1],
      [obj2, 2],
      [map, 3]
    ]);

    console.log(`args`, args);
    console.log(`obj`, obj);
    console.log(`obj2`, obj2);
    console.log(`fn`, fn);
    console.log(`boundFn`, boundFn);
    console.log(`path`, inspect(path));
    console.log(
      `console`,
      console.config({ colors: true, compact: 1, breakLength: Infinity }),
      console
    );
    console.log(`map`, map);
    console.log(`weakMap`, weakMap);
  }

  return 'done';
}
Util.callMain(main, true);
/*
console.log('TEST\n');
print('TEST\n');
let retVal;
//retVal =  main().catch(err => console.log("Error:", err, err.stack)).then(ret => (console.log("Resolved:", ret),ret));
retVal = main();
console.log('retVal:', retVal);
retVal;
1;
//Util.callMain(main, true);
*/
