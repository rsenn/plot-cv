import { ECMAScriptParser } from "./lib/ecmascript/parser.js";
import Lexer, { Stack, PathReplacer } from "./lib/ecmascript/lexer.js";
import Printer from "./lib/ecmascript/printer.js";
import { estree, Factory, ESNode } from "./lib/ecmascript/estree.js";

import Util from "./lib/util.js";
import fs from "fs";
import util from "util";
import { Console } from "console";
import deep from "./lib/deep.js";
import { SortedMap } from "./lib/container/sortedMap.js";

//import process from 'process';
Error.stackTraceLimit = 1000;

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

function printAst(ast, comments) {
  let printer = new Printer({ indent: 4 }, comments);

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

      let flat = deep.flatten(
        deep.iterate(ast, node => node instanceof ESNode),
        new SortedMap()
      );
      let posMap = new SortedMap(
        [...flat].map(([key, value]) => [value.position ? value.position.pos * 10 : -1, value]),
        (a, b) => a - b
      );

      let commentMap = new SortedMap(
        [...parser.comments].map(({ comment, node, pos, len, ...item }) => [pos * 10 - 1, { comment, pos, len, node: posMap.keyOf(node) }]),
        (a, b) => a - b
      );

      posMap = new SortedMap([/*...posMap,*/ ...commentMap], (a, b) => a - b);

      //  console.log("ast:", [...posMap.keys()]);$
      // console.log("ast:", Util.className(flat));
      console.log("commentMap:", commentMap);

      //  console.log("nodes:", parser.nodes.map(n =>  [Util.className(n), n.position.toString()]));
    } catch(err) {
      error = err;
    }
    files[file] = finish(error);

    if(!error) {
      const output_file = file.replace(/.*\//, "").replace(/\.[^.]*$/, "") + ".es";

      //  console.log("ast:", ast);
      console.log("saving to:", output_file);
      dumpFile(output_file, printAst(ast, parser.comments));
    }

    console.log("files:", files);
  }
  var success = Object.entries(files).filter(([k, v]) => !!v).length != 0;
  process.exit(Number(files.length == 0));
}

function finish(err) {
  let fail = !!err;
  if(fail) {
    err.stack = PathReplacer()("" + err.stack)
      .split(/\n/g)
      .filter(s => !/esfactory/.test(s))
      .join("\n");
  }

  if(err) {
    console.log(parser.lexer.currentLine());
    console.log(Util.className(err) + ": " + (err.msg || err) + "\n" + err.stack);
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
