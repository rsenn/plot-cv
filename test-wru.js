import wru from './lib/wru.js';
wru.test([
  {
    name: 'basic',
    test: () => wru.assert('works', 1)
  }
]);