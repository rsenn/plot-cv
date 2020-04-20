const Util = require("./lib/util.es5.js");

let d = Util.parseDate("2018-11-01T23:31:46.000Z");

console.log(Util.toUnixTime(d));
