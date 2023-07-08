import Alea from './lib/alea.js';
import { repeat } from './lib/misc.js';
let rng = new Alea(1337);

console.log(repeat(10, rng));
console.log(repeat(10, rng.fract53));
console.log(repeat(10, rng.uint32));
console.log(repeat(10, rng.int32));
console.log(repeat(10, rng.signed));
console.log(repeat(10, rng.color));