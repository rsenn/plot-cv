import Parser from "./lib/ecmascript/parser.js";
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

let files = args.reduce((acc, arg) => ({ ...acc, [arg]: undefined }), {});


process.on('uncaughtException', (err, origin) => {
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
    console.log( "\nSIGINT - Exit\n");

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
};

Error.stackTraceLimit = 100;

global.parser = null;



function main(args) {
  if(args.length == 0) args.push("./lib/ecmascript/parser.js");

  for(let arg of args) {
    let data, b, ret;
    if(arg == "-") arg = "/dev/stdin";

    console.log("file:", arg);

    data = fs.readFileSync(arg);
    console.log("opened:", data);

    let ast, error;
    console.log(estree.ArrayBindig);

    console.log(Util.getCallerStack());

    global.parser = new Parser(data.toString(), arg);

    try {
      ast = parser.parseProgram();
    } catch(err) {
      error = err;
    }

    files[arg] = finish(error);

    console.log("files:", files);

    //  process.exit(Number(files.length == 0));
  }
}

function finish(err) {

  if(err) {
    console.log("ERROR: "+Util.className(err)+": "+err.toString());
  }

  let fail = !!err;

//  let stack =/* (err instanceof SyntaxError  && err.stack.length) ? err.stack : */
 if(fail) {
 let stack = PathReplacer()(''+err.stack);
  console.log("STACK:\n"+stack);
}

  let lexer = parser.lexer;
  let t = [];

  console.log(parser.trace() );

  // console.log("currentLine:", lexer.currentLine());

  dumpFile("trace.log", parser.trace());

 

//  let printer = new Printer({ indent: 4 });

  if(fail) { console.log("\nerror:", err.msg, "\n", parser.lexer.currentLine());
  } else {
  /*  let out = printer.print(ast);
    console.log("\nAST:\n   " + out.replace(/\n/g, "\n  "));*/
  }

  console.log("finish: " + (fail ? "error" : "success"));
  return !fail;
}