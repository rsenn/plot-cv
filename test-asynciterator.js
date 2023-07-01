import { fail, assert, assertEquals, eq, assertStrictEquals, tests } from './lib/tinytest.js';

import('console').then( ({ Console }) => (globalThis.console = new Console({ inspectOptions: { compact: 2 } })) );

tests({
  async 'next() value'() {
    const gen = (async function* () {
      yield yield undefined;
    })();

    await gen.next();
    eq((await gen.next('value#1')).value, 'value#1');
  },
  async 'await yield'() {
    let step = 0;
    const gen = (async function* () {
      yield step;
      step++;
      yield step;
      step++;
      yield step;
    })();

    eq((await gen.next('value#1')).value, 0);
    eq(step, 0);
    eq((await gen.next('value#2')).value, 1);
    eq(step, 1);
    eq((await gen.next('value#3')).value, 2);
    eq(step, 2);
  },
  async 'return()'() {
    const gen = (async function* () {
      for(let i = 0; i < 10; i++) yield i;
    })();

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);
    eq((await gen.return('value#3')).value, 'value#3');

    eq((await gen.next()).done, true);
    eq((await gen.next()).value, undefined);
  },
  async 'throw()+rethrow'() {
    const gen = (async function* () {
      try {
        for(let i = 0; i < 10; i++) yield i;
      } catch(error) {
        throw new Error('throw+' + error.message);
      }
    })();

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);

    try {
      let pr = gen.throw(new Error('value#3'));
      console.log('pr', pr);
      await pr;
    } catch(err) {
      eq(err.message, 'throw+value#3');

      eq((await gen.next()).done, true);
      eq((await gen.next()).value, undefined);
    }
  }
});
