import { Repeater } from './lib/repeater/repeater.js';
import { useValue, useResult, useAsyncIter, useRepeater } from './lib/repeater/react-hooks.js';
import { InMemoryPubSub } from './lib/repeater/pubsub.js';
import { semaphore, throttler } from './lib/repeater/limiters.js';
import { DroppingBuffer, FixedBuffer, SlidingBuffer } from './lib/repeater/buffers.js';

async function main() {
  let pushEvent;
  let r = new Repeater(async (push, stop) => {
    push(null);
    pushEvent = push;
    await stop;
  });

  r.next();

  pushEvent('new value');

  for await (let value of r) {
    console.log('value:', value);
  }
  /*
  let p = new InMemoryPubSub();

  let subscriber = await p.subscribe('test' );
  // let publish = await p.publish('test');

  //  p.publishers['test'].push(1234);
  await p.publish('test', 1234);

  console.log('subscriber:', subscriber);

  console.log('p:', p);

  for await (let value of subscriber) {
    console.log('value:', value);
  }*/
}
try {
  main(process.argv.slice(2));
} catch(err) {
  console.error('error:', err);
}
