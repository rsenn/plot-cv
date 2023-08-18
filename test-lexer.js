import { ReadFile } from './io-helpers.js';
import Lexer from './lib/ecmascript/lexer.js';

const testfn = () => true;
const testtmpl = `this is
a test`;

const Code = `
       const re = /.*[\.\/\\\\]/;

       for(let [value, path] of deep.iterate(x, (v, k) => /data-/.test(k[k.length - 1]))) deep.unset(x, path);
  
 `;

function main(arg) {
  let file = arg || './lib/ecmascript/parser.js';
  let data = ReadFile(file);
  console.log('data:', data);

  let lexer = new Lexer(data.toString(), file);
  let token;
  try {
    do {
      token = lexer.lex();

      console.log(`tok(${token.id}):`, tokenColor(token));
    } while(token.type != 'eof');
  } catch(err) {
    console.log('ERROR:', err.message, err.stack);
  }
}

try {
  main(...getArgs());
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
} finally {
  console.log('SUCCESS');
}

function tokenColor(tok) {
  const { type, lexeme, loc, pos } = tok;

  const colors = {
    templateLiteral: '\x1b[1;35m',
    booleanLiteral: '\x1b[1;31m',

    identifier: '\x1b[1;33m',
    punctuator: '\x1b[1;36m',
    numericLiteral: '\x1b[1;36m',
    stringLiteral: '\x1b[1;36m',
    keyword: '\x1b[1;31m',
    nullLiteral: '\x1b[1;31m',
    regexpLiteral: '\x1b[1;35m'
  };

  return `${colors[type]}${lexeme}\x1b[0m`;
}