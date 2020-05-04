const Alea = require("./lib/alea.js");
const Util = require("./lib/util.es5.js").Util;

let rng = new Alea(1337);

console.log(Util.repeat(10, rng));
console.log(Util.repeat(10, rng.fract53));
console.log(Util.repeat(10, rng.uint32));
console.log(Util.repeat(10, rng.int32));
console.log(Util.repeat(10, rng.signed));
console.log(Util.repeat(10, rng.color));
