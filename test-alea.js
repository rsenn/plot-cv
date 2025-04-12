import Alea from './lib/alea.js';
import { repeat } from './lib/misc.js';
import { eq, tests } from './lib/tinytest.js';

function log(arg) {
  console.log('result:', [...arg]);
}

const x = {
  alea() {
    let rng = new Alea(1337);

    log(repeat(10, rng));
    log(repeat(10, rng.fract53));
    log(repeat(10, rng.uint32));
    log(repeat(10, rng.int32));
    log(repeat(10, rng.signed));
    log(repeat(10, rng.color));
  }
};

x.alea();
