import { ReadFile } from './io-helpers.js';
import Lexer from './lib/jslexer.js';
import { escape } from './lib/misc.js';

const testfn = () => true;
const testtmpl = `this is
a test`;

const Code = `
       const re = /.*[\.\/\\\\]/;

       for(let [value, path] of deep.iterate(x, (v, k) => /data-/.test(k[k.length - 1]))) deep.unset(x, path);
  
 `;

function main(arg) {
  let file = arg || './lib/misc.js';
  let data = ReadFile(file, 'utf-8');
  //console.log('data:', data);

  let lexer = new Lexer();
  lexer.setInput(data.toString(), file);
  let token;

  try {
    do {
      let loc = lexer.loc.clone();

      token = lexer.lex();
      if(!token) break;
      const { lexeme, id } = token;

      if(lexer.state == 2 && token.lexeme == '}') lexer.popState();
      console.log(`token`, (id + '').padStart(2), ` '${escape(lexeme)}'`.padEnd(30), loc);
      //   console.log(`tok(${ id}): ${tokenColor(lexer.tokens[token.id-1])}${lexeme}\x1b[0m`);
    } while(token.type != 'eof');
  } catch(err) {
    console.log('ERROR:', err.message, err.stack);
  }
}

try {
  main(...getArgs().slice(1));
} catch(error) {
  console.log(`FAIL: ${error.message}\n${error.stack}`);
} finally {
  console.log('SUCCESS');
}

function tokenColor(type, lexeme) {
  //  const { type, lexeme, loc, pos } = tok;

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

  return colors[type];
}