import { Lexer } from './lib/parse/lexer.js';
import { Grammar } from './lib/parse/grammar.js';
import { Parser } from './lib/parse/parser.js';
import Ebnf2Parser from './lib/parse/ebnf2.js';
//import CGrammar from './test-grammar.json';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import path from './lib/path.js';
import Cowbird from './lib/parse/cowbird.js';
import deep from './lib/deep.js';

let filesystem;

function WriteFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: 4 });

  let [filename = './lib/grammars/INI.g4'] = args;
  let basename = path.basename(filename, path.extname(filename));

  let src = filesystem.readFile(filename).toString();

  let grammar = new Grammar(src, filename);

  grammar.parse();
  console.log('grammar:', grammar);

  WriteFile(`grammar-${basename}.js`, grammar.generate('./lib/parse/'));

  let a = [];
  for(let [name, rule] of grammar.rules) {
    a.push(rule.toCowbird(a, name));
  }

  let cowbirdGrammar = grammar.toCowbird();
  console.log('cowbird:', cowbirdGrammar);

  let data = filesystem.readFile('../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp');
  console.log('data:', Util.abbreviate(data, 100));
  let parser = new Cowbird(cowbirdGrammar, 'ini', true);
  console.log('parser:', parser);

  let result = parser.parse(data);
  console.log('result:', result);

  return;
}

Util.callMain(main, true);
