import { parseModule } from 'esprima';
import Util from './lib/util.js';
import { Console } from 'console';
import * as fs from 'fs';
import * as path from 'path';

function main(...args) {
  globalThis.console = new Console({
    stdout: process.stdout,
    inspectOptions: {
      maxStringLength: 200,
      maxArrayLength: 10,
      breakLength: 100,
      compact: 1,
      depth: 10
    }
  });

  for(let arg of args) {
    let error;
    let ast;
    try {
      let code = fs.readFileSync(arg, 'utf8');
      ast = parseModule(code, {
        module: true,
        next: true,
        ranges: false,
        webcompat: false,
        loc: true,
        raw: false,
        directives: true,
        globalReturn: false,
        impliedStrict: false,
        onComment: [],
        onToken: [],
        preserveParens: true,
        lexical: false,
        source: false,
        identifierPattern: false,
        jsx: false,
        specDeviation: false
      });
    } catch(e) {
      error = e;
    }

    if(!error) {
      let outputFile = path.basename(arg, path.extname(arg)) + '.ast.json';
      console.log(`Parsing '${arg}' succeeded, writing AST to '${outputFile}'`);
      fs.writeFileSync(outputFile, JSON.stringify(ast, null, 2));
    } else {
      throw error;
    }
  }
}
let error;
try {
  main(...Util.getArgv().slice(1));
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    console.log('FAIL');
    Util.exit(1);
  } else {
    console.log('SUCCESS');
  }
}
