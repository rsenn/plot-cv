import Util from './lib/util.js';

//import inspect from 'inspect.so';

async function main(...args) {
  const options = {
    colors: true,
    showHidden: false,
    customInspect: true,
    showProxy: false,
    getters: false,
    depth: 3 /*|| Infinity*/,

    maxArrayLength: 10,
    maxStringLength: 200,
    breakLength: (await Util.ttyGetWinSize(1))[0] || 80,
    compact: 1
  };

  await import(Util.getPlatform() == 'quickjs' ? 'inspect.so' : 'util').then(module => (globalThis.inspect = module.inspect)
  );

  console.log('main', args);
  let value = 0;
  let str =
    '2.6 Test262 (ECMAScript Test Suite)\nA test262 runner is included in the QuickJS archive. The test262 tests can be installed in the\nQuickJS source directory with:\ngit clone https://github.com/tc39/test262.git test262\ncd test262\npatch -p1 < ../tests/test262.patch\ncd ..\nThe patch adds the implementation specific harness functions and optimizes the inefficient\nRegExp character classes and Unicode property escapes tests (the tests themselves are not\nmodified, only a slow string initialization function is optimized).\nThe tests can be run with\nmake test2\nThe configuration files test262.conf (resp. test262o.conf for the old ES5.1 tests1 )) contain\nthe options to run the various tests. Tests can be excluded based on features or filename.\nThe file test262_errors.txt contains the current list of errors. The runner displays a\nmessage when a new error appears or when an existing error is corrected or modified. Use the\n-u option to update the current list of errors (or make test2-update).\nThe file test262_report.txt contains the logs of all the tests. It is useful to have a clearer\nanalysis of a particular error. In case of crash, the last line corresponds to the failing test.\nUse the syntax ./run-test262 -c test262.conf -f filename.js to run a single test. Use\nthe syntax ./run-test262 -c test262.conf N to start testing at test number N.\nFor more information, run ./run-test262 to see the command line options of the test262\nrunner.\nrun-test262 accepts the -N option to be invoked from test262-harness2 thru eshost.\nUnless you want to compare QuickJS with other engines under the same conditions, we do not\nrecommend to run the tests this way as it is much slower (typically half an hour instead of about\n100 seconds).\n';

  let obj = {
    '@nan': NaN,
    '-inf': Number.NEGATIVE_INFINITY,
    '+inf': Number.POSITIVE_INFINITY,
    1337: -0,
    array: [...Util.range(1, 10), 'last item'],
    string: str,
    boolean: true,
    null: null,
    symbol: Symbol.for('inspect'),
    number: 1234,
    undef: undefined,
    fn: function test() {},
    bigint: 314159265358979323846264n,
    object: {
      a: 1,
      b: true,
      c: 'prop',
      d: [...Util.range(1, 100), { x: 99, y: Infinity, z: NaN }]
    },
    get value() { return value; },
    set value(v) { value = v; },
    set v(v) { value = v; }
  };

  console.log('inspect(NaN)', inspect(NaN, options));
  // for(let value of Object.values(obj)) console.log('inspect', inspect(value, options));

  console.log('inspect', inspect(obj, options));
}

Util.callMain(main, true);
