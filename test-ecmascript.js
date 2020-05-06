import Parser from "./lib/ecmascript/parser.js";
import Lexer from "./lib/ecmascript/lexer.js";
import Printer from "./lib/ecmascript/printer.js";
import Util from "./lib/util.js";
import fs from "fs";
import util from "util";
import { Console } from "console";
import { estree, Factory } from "./lib/ecmascript/estree.js";

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 20, colors: true }
});

const testfn = () => true;
const testtmpl = `this is
a test`;
let args = process.argv.slice(2);
if(args.length == 0) args.push("-");
/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
const dumpFile = (name, data) => {
  if(typeof data != "string") {
    if(Util.isArray(data)) data = data.join("\n");
  }
  fs.writeFileSync(name, data + "\n");
  console.log(`Wrote '${name}': ${data.length} bytes`);
};

Error.stackTraceLimit = 100;

(async arg => {
  let data, b, ret;
  if(arg == "-") arg = "/dev/stdin";

  arg = arg || "./lib/ecmascript/parser.js";
  data = await fs.promises.readFile(arg);
  console.log("opened:", data);

  let ast;
  console.log(estree.ArrayBindig);

  console.log(Util.getCallerStack());

  try {
    ast = await Parser.parse(data.toString());
  } catch(err) {
    console.log("ERROR: " + err.toString());
    console.log("Stack:", err.stack);
    let lexer = Parser.instance.lexer;
    let t = [];

    // console.log("currentLine:", lexer.currentLine());

    dumpFile("trace.log", Parser.trace);
  }

  let printer = new Printer({ indent: 4 });

  console.log("output:\n" + printer.print(ast));
})(...args);
