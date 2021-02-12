import { Repeater } from './lib/repeater/repeater.js';
import { delay } from './lib/repeater/timers.js';
import ConsoleSetup from './lib/consoleSetup.js';

async function main() {
  await ConsoleSetup({ compact: 2 });
  let pushEvent, stopEvent;

  await Util.waitFor(0);
  console.log('setTimeout', globalThis.setTimeout);
  console.log('clearTimeout', globalThis.clearTimeout);

  let a = new Repeater(async (push, stop) => {
     pushEvent = push;
    stopEvent = stop;
    await stop;
  });

  let b = new Repeater(async (push, stop) => {
    for(let num of Util.range(1, 20)) {
      await Util.waitFor(50);
      push(`b #${num}`);
    }
    stop();
  });

  let c = new Repeater(async (push, stop) => {
    for(let num of Util.range(1, 5))
      push(`c #${num}`);
    stop();
  });        


  function genRepFunc(name, num, ms) {
    return async(push,stop) => {
        for(let n of Util.range(1, num)) {
          await Util.waitFor(Util.randInt(ms));
             push(`${name} #${n}`);
           }
    stop();
    };
  }




  //a.next();
  let repeat = Repeater.merge([a, b, Repeater.zip([c, delay(200)])]);

  let loop = (async () => {
    for await(let value of repeat) {
      console.log('value:', value);
    }
    console.log('stopped');
  })();

  for(let num of Util.range(1, 10)) {
    await Util.waitFor(100);
    pushEvent(`a #${num}`);
  }
  stopEvent();

  await loop;

    let x = [ new Repeater(genRepFunc('x', 20,1000)) ,new Repeater(genRepFunc('y', 20,1000))  ,new Repeater(genRepFunc('z', 20,1000)) ];
  let latest = Repeater.latest(x);

 
   await (async () => {
    for await(let tuple of latest) {
      console.log('tuple:', tuple);
    }
    console.log('stopped');
  })();
}

Util.callMain(main, true);
