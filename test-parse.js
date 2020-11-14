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

function dumpFile(name, data) {
  if(Util.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

async function main(...args) {
  await PortableFileSystem(fs => (filesystem = fs));
  await ConsoleSetup({ depth: 4 });

  let [filename = './lib/grammars/INI.g4'] = args;
  let basename = path.basename(filename, '.g4');

  let src = filesystem.readFile(filename).toString();

  //let lex = new Lexer(src, filename);
  let grammar = new Grammar(src, filename);

  grammar.parse();
  //console.log('grammar:', grammar);
  //console.log('grammar:', grammar.generate());
  dumpFile(`grammar-${basename}.js`, grammar.generate('./lib/parse/'));
  //grammar.resolveRules();

  //console.log("CGrammar", CGrammar);
  //console.log('CGrammar.compilationUnit', CGrammar.compilationUnit);

  //filename = './seek_set.c';
  //filename = '../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp';
  //src = filesystem.readFile(filename).toString();
  //let result = CGrammar.compilationUnit(`int seek_set(int fd, seek_pos pos) {if(lseek(fd, (off_t)pos, SET) == -1) return -1; return 0; } `, 0);
  //let result = CGrammar.ini(src, 0);
  //console.log('parsed:', result);
  let a = [];
  for(let [name, rule] of grammar.rules) {
    a.push(rule.toCowbird(a, name));
  }

  /* let flat = deep.flatten(grammar.rules, new Map(), it => typeof it == 'object' || true);

  console.log('flat:', flat);
  let aMatch = grammar.rules.get('ini').productions[0];
  console.log('aMatch:', aMatch);
  console.log('aMatch:', aMatch.clone());*/

  let cowbirdGrammar = grammar.toCowbird();
  console.log('cowbird:', cowbirdGrammar);

  let data = filesystem.readFile('../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp');
  console.log('data:', Util.abbreviate(data, 100));
  let parser = new Cowbird(cowbirdGrammar, 'ini', true);
  console.log('parser:', parser);

  let result = parser.parse(data);
  console.log('result:', result);

  return;

  /*let clex = new Lexer(src, filename);
  let cparse = new Parser(clex);

  let rule = grammar.getRule('typeSpecifier');
  let buffer = filesystem.readFile('./lib/ecmascript/es6.ebnf');
  process.exit(0);
  let parser = new Ebnf2Parser(buffer.toString());
  grammar = parser.parseGrammar();
  if(grammar != null) parser.state.advance(grammar.nodeLength());
  if(parser.state.current == '') {
    grammar.print(0);
    process.exit(0);
  } else {
    process.exit(-1);
  }*/
}

Util.callMain(main, true);
