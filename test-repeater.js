import * as std from 'std';
import * as os from 'os';

import { Repeater } from './lib/repeater/repeater.js';
import { delay } from './lib/repeater/timers.js';
import { Console } from 'console';
import { waitFor } from './quickjs/qjs-modules/lib/util.js';
import Util from './lib/util.js';

const { setTimeout, clearTimeout } = os;
Object.assign(globalThis, { setTimeout, clearTimeout });

function main() {
  globalThis.console = new Console(std.err, {
    inspectOptions: {
      colors: true,
      depth: 1,
      maxArrayLength: 30,
      compact: 3
    }
  });
  const scale = 1;

  (async function() {
    let pushEvent, stopEvent;
    console.log('Repeater', Repeater);
    console.log('waitFor', waitFor);

    // await waitFor(0);
    console.log('setTimeout', globalThis.setTimeout);
    console.log('clearTimeout', globalThis.clearTimeout);

    let a = new Repeater(async (push, stop) => {
      pushEvent = push;
      stopEvent = stop;
      await stop;
    });

    let b = new Repeater(async (push, stop) => {
      for(let num of Util.range(1, 20)) {
        await waitFor(5 * scale);
        push(`b #${num}`);
      }
      stop();
    });

    let c = new Repeater(async (push, stop) => {
      for(let num of Util.range(1, 5)) push(`c #${num}`);
      stop();
    });

    //a.next();
    let repeat = Repeater.merge([a, b, Repeater.zip([c, delay(20 * scale)])]);

    let loop = (async () => {
      for await(let value of repeat) {
        console.log('value:', value);
      }
      console.log('stopped');
    })();

    for(let num of Util.range(1, 10)) {
      await waitFor(10 * scale);
      pushEvent(`a #${num}`);
    }
    stopEvent();

    await loop;

    let x = [
      new Repeater(genRepFunc('x', 20, 100 * scale)),
      new Repeater(genRepFunc('y', 20, 100 * scale)),
      new Repeater(genRepFunc('z', 20, 100 * scale))
    ];
    let latest = Repeater.latest(x);

    await (async () => {
      try {
        for await(let tuple of latest) {
          console.log('tuple:', console.config({ compact: 2 }), tuple);
        }
      } catch(error) {
        console.log('error:', error);
      }
      console.log('stopped');
    })();
  })();

  function genRepFunc(name, num, ms) {
    return async (push, stop) => {
      for(let n of Util.range(1, num)) {
        await waitFor(Util.randInt(ms));
        push(`${name} #${n}`);
      }
      stop();
    };
  }
}

main();
