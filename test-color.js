const { RGBA, HSLA } = require("./lib/dom.es5.js");

const { Console } = require("console");

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stdout,
  inspectOptions: { depth: 10 }
});

let args = [...process.argv].slice(2);

let colorStr = args.shift() || "#00f";
let color = new RGBA(colorStr);
let hsla = color.toHSLA();

console.log(color);
console.log(hsla.toString());
