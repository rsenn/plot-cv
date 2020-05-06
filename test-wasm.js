/*const fs = require('fs');
//#const bpgdec = require("./static/wasm/bpgdec.wasm.js");
const bpgdec = require("./static/wasm/bpgdec.js");

console.error("bpgdec: ", bpgdec);

*/
"use strict";
const fs = require("fs");
const bytes = fs.readFileSync("./static/wasm/bpgdec.wasm");
console.log("bytes: ", bytes);

var importObject = {
  /* imports: { imported_func: arg => console.log(arg) },
module: {},
 instance: {},
 env: {
   memory: new WebAssembly.Memory({ initial: 256 }),
   table: new WebAssembly.Table({ initial: 0, element: "anyfunc" }),
   abort: err => console.log("Abort:",err)
 }*/
  module: {},
  env: {
    memory: new WebAssembly.Memory({
      initial: 256
    }),
    table: new WebAssembly.Table({
      initial: 0,
      element: "anyfunc"
    }),
    abort(_msg, _file, line, column) {
      console.error("abort called at main.ts:" + line + ":" + column);
    },
    emscripten_resize_heap(b) {
      console.error("Cannot enlarge memory arrays to size " + b + " bytes (OOM). Either (1) compile with  -s TOTAL_MEMORY=X  with X higher than the current value " + C.length + ", (2) compile with  -s ALLOW_MEMORY_GROWTH=1  which allows increasing the size at runtime, or (3) if you want malloc to return NULL (0) instead of this abort, compile with  -s ABORTING_MALLOC=0 ");
    }
  }
};

/*(async () => {
 // Explicitly compile and then instantiate the wasm module.
 const module = await WebAssembly.compile(bytes);
 console.log("module: ", module);
 const instance = await WebAssembly.instantiate(module, importObject);

 //console.log(instance.exports.addTwo(8, 5));
})();*/

(async () => {
  // Instantiate the Wasm module with an implicit compilation step.
  const { instance, module } = await WebAssembly.instantiate(bytes, importObject);

  // console.log(instance.exports.addTwo(8, 5));
})();
