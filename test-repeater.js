import { Repeater } from './lib/repeater/repeater.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { InMemoryPubSub } from './lib/repeater/pubsub.js';

async function main() {
  await ConsoleSetup();
  let pushEvent, stopEvent;

  let r = new Repeater(async (push, stop) => {
    //push(null);
    pushEvent = push;
    stopEvent = stop;
    await stop;
  });

  //r.next();

  let loop = (async () => {
    for await(let value of r) {
      console.log('value:', value);
    }
    console.log('stopped');
  })();

  for(let num of Util.range(1, 10)) {
    await Util.waitFor(100);
    pushEvent(`new value #${num}`);
  }
  stopEvent();

  await loop;
}

Util.callMain(main, true);
