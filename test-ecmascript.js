import { ECMAScriptParser } from "./lib/ecmascript/parser.js";
import Lexer, { Stack, PathReplacer } from "./lib/ecmascript/lexer.js";
import Printer from "./lib/ecmascript/printer.js";
import Util from "./lib/util.js";
import fs from "fs";
import util from "util";
import { Console } from "console";
import { estree, Factory } from "./lib/ecmascript/estree.js";
//import process from 'process';

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

let files = args.reduce((acc, file) => ({ ...acc, [file]: undefined }), {});

process.on("uncaughtException", (err, origin) => {
  fs.writeSync(
    process.stderr.fd,
    `Caught exception: ${err}\n
Exception origin: ${origin}\n
Stack: ${err.stack}`
  );
  process.exit();
});

process.on("SIGINT", () => {
  //fs.writeSync(process.stderr.fd, "\nSIGINT - Exit\n");
  console.log("\nSIGINT - Exit\n");

  finish();
  process.exit(3);
});

process.on("exit", () => {
  //fs.writeSync(process.stderr.fd, "\nexited\n");
  console.log("\nexited\n");
  process.exit();
});

main(args);

/*
const LoginIcon = ({ style }) => (<svg style={style} height="56" width="34" viewBox="0 0 8.996 14.817" xmlns="http://www.w3.org/2000/svg">
    <defs />

*/
function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join("\n");
  if(typeof data != "string") data = "" + data;

  fs.writeFileSync(name, data + "\n");

  console.log(`Wrote '${name}': ${data.length} bytes`);
}

function printAst(ast) {
  let printer = new Printer({ indent: 4 });

  return printer.print(ast);
}

Error.stackTraceLimit = 100;

global.parser = null;

function main(args) {
  if(args.length == 0) args.push("./lib/ecmascript/parser.js");
  for(let file of args) {
    let data, b, ret;
    if(file == "-") file = "/dev/stdin";
    console.log("file:", file);
    data = fs.readFileSync(file);
    console.log("opened:", data);
    let ast, error;

    global.parser = new ECMAScriptParser(data.toString(), file);
    try {
      ast = parser.parseProgram();

    //  console.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
    } catch(err) {
      error = err;
    }
    files[file] = finish(error);

    if(!error) {
      const output_file = file.replace(/.*\/(.*)\.([^.]*)$/, "$1.es");

          console.log("saving to:", output_file);
      dumpFile(output_file, printAst(ast));
    }

    console.log("files:", files);
  }
  var success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}

function finish(err) {
  if(err) {
    console.log("ERROR: " + Util.className(err) + ": " + err.toString());
  }
  let fail = !!err;
  if(fail) {
    let stack = PathReplacer()("" + err.stack)
      .split(/\n/g)
      .slice(0, 10)
      .join("\n");
    console.log("STACK:\n" + stack);
  }
  let lexer = parser.lexer;
  let t = [];
  //console.log(parser.trace() );
  dumpFile("trace.log", parser.trace());
  if(fail) {
    console.log("\nerror:", err.msg, "\n", parser.lexer.currentLine());
  }
  console.log("finish: " + (fail ? "error" : "success"));
  return !fail;
}
