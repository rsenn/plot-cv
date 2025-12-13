import { Repeater } from './lib/repeater/repeater.js';
import { eq, tests } from './lib/tinytest.js';

//import('console').then(({ Console }) => (globalThis.console = new Console({ inspectOptions: { compact: 2 } })));

tests({
  async 'next() value'() {
    const gen = new Repeater(async (push, stop) => push(await push(undefined)));

    throw new Error('test');

    await gen.next();
    eq((await gen.next('value#1')).value, 'value#1');
  },
  async 'await push'() {
    let step = 0;
    const gen = new Repeater(async (push, stop) => {
      await push(step);
      step++;
      await push(step);
      step++;
      await push(step);
    });

    eq((await gen.next('value#1')).value, 0);
    eq(step, 0);
    eq((await gen.next('value#2')).value, 1);
    eq(step, 1);
    eq((await gen.next('value#3')).value, 2);
    eq(step, 2);
  },
  async 'return()'() {
    const gen = new Repeater(async (push, stop) => {
      for(let i = 0; i < 10; i++) await push(i);
    });

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);
    eq((await gen.return('value#3')).value, 'value#3');

    eq((await gen.next()).done, true);
    eq((await gen.next()).value, undefined);
  },
  async 'throw()+rethrow'() {
    const gen = new Repeater(async (push, stop) => {
      for(let i = 0; i < 10; i++) await push(i);
    });

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);

    try {
      let pr = gen.throw(new Error('value#3'));
      console.log('pr', pr);
      pr = await pr;
      console.log('pr', pr);
    } catch(err) {
      eq(err.message, 'value#3');

      eq((await gen.next()).done, true);
      eq((await gen.next()).value, undefined);
    }
  },
  async 'throw()+catch/stop()'() {
    const gen = new Repeater(async (push, stop) => {
      try {
        for(let i = 0; i < 10; i++) await push(i);
      } catch(e) {
        stop(new Error('stop+' + e.message));
      }
    });

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);

    try {
      let pr = gen.throw(new Error('value#3'));
      console.log('pr', pr);
      pr = await pr;
      console.log('pr', pr);
    } catch(err) {
      eq(err.message, 'stop+value#3');

      eq((await gen.next()).done, true);
      eq((await gen.next()).value, undefined);
    }
  },
  async 'throw()+handled'() {
    const gen = new Repeater(async (push, stop) => {
      try {
        for(let i = 0; i < 10; i++) await push(i);
      } catch(e) {
        push(Infinity);
        stop();
      }
      stop('end');
    });

    eq((await gen.next('value#1')).value, 0);
    eq((await gen.next('value#2')).value, 1);

    let r = await gen.throw(new Error('value#3'));
    eq(r.done, false);
    eq(r.value, Infinity);

    r = await gen.next();
    eq(r.done, true);
    eq(r.value, undefined);
  }
});