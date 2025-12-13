import * as path from 'path';
import { ReadFile, WriteFile } from './io-helpers.js';
import { Console } from 'console';
import { parseScript } from 'meriyah';

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
      let code = ReadFile(arg, 'utf8');
      ast = parseScript(code, {
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
        preserveParens: false,
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
      WriteFile(outputFile, JSON.stringify(ast, null, 2));
    } else {
      throw error;
    }
  }
}

let error;
try {
  main(...process.argv.slice(1));
} catch(e) {
  error = e;
} finally {
  if(error) {
    console.log(`FAIL: ${error.message}\n${error.stack}`);
    console.log('FAIL');
    process.exit(1);
  } else {
    console.log('SUCCESS');
  }
}