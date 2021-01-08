import Lexer from './lib/ecmascript/lexer.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { h, Component } from './lib/dom/preactComponent.js';
import Util from './lib/util.js';
const testfn = () => true;
const testtmpl = `this is
a test`;

const Code = `
       const re = /.*[\.\/\\\\]/;

       for(let [value, path] of deep.iterate(x, (v, k) => /data-/.test(k[k.length - 1]))) deep.unset(x, path);
  
 `;

(async (arg) => {
  await ConsoleSetup();
  let data = Code || (await fs.readFile(arg || './lib/ecmascript/parser.js'));
  console.log('data:', data);

  let lexer = new Lexer(data.toString());
  let token;
  try {
    do {
      token = lexer.lex();

      console.info('tok:', tokenColor(token));
    } while (token.type != 'eof');
  } catch (err) {
    console.log('ERROR:', err);
  }
})(...Util.getArgs());

function tokenColor(tok) {
  const { type, value, position, offset } = tok;

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

  return `${colors[type]}${value}\x1b[0m`;
}
