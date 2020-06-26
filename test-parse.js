import { Lexer } from './lib/parse/lexer.js';
import { Grammar } from './lib/parse/grammar.js';
import { Parser } from './lib/parse/parser.js';
import Ebnf2Parser from './lib/parse/ebnf2.js';
import fs from 'fs';
import { literal, optional, seq, or, param, exhaustive } from './lib/parse/expr.js';
import CGrammar from './test-grammar.js';
import { Console } from 'console';
global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 20, colors: true }
});

let filename = './lib/grammars/INI.g4';
let src = fs.readFileSync(filename).toString();

//let lex = new Lexer(src, filename);
let grammar = new Grammar(src, filename);

grammar.parse();
//console.log('grammar:', grammar);
//console.log('grammar:', grammar.generate());
fs.writeFileSync('test-grammar.js', grammar.generate());
//grammar.resolveRules();

//console.log("CGrammar", CGrammar);
console.log('CGrammar.compilationUnit', CGrammar.compilationUnit);

filename = './seek_set.c';
filename = '../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp';
src = fs.readFileSync(filename).toString();
//let result = CGrammar.compilationUnit(`int seek_set(int fd, seek_pos pos) {if(lseek(fd, (off_t)pos, SET) == -1) return -1; return 0; } `, 0);
let result = CGrammar.ini(src, 0);
console.log('parsed:', result);

let clex = new Lexer(src, filename);
let cparse = new Parser(clex);

for(let [name, rule] of grammar.rules.entries()) {
  let ok = rule.match(cparse);

  if(ok != -1 && ok) {
    console.log('ok:', ok);
    console.log(`rule[${ok}]:`, rule[ok]);

    console.log(`${clex.line}:${clex.column} rule ${name}:`, rule.toString());
  }
}

console.log('cparse:', cparse);

/*
for(let { tok, str } of lex) {
  tok = (tok + '').replace(/([\n\r\t])/g, '\\$1');
  console.log(`token: ${lex.position} ${tok} ${str}`);
}
*/
let rule = grammar.getRule('typeSpecifier');
console.log('rule:', rule);

let buffer = fs.readFileSync('./lib/ecmascript/es6.ebnf');

process.exit(0);
let parser = new Ebnf2Parser(buffer.toString());

grammar = parser.parseGrammar();
console.log('grammar:', grammar);
//console.log('grammar.nodeLength():', grammar.nodeLength());

if(grammar != null) parser.state.advance(grammar.nodeLength());
if(parser.state.current == '') {
  grammar.print(0);
  process.exit(0);
} else {
  console.log(
    'incomplete parse: lineNumber=' +
      parser.state.lineNumber +
      ' input=' +
      parser.state.buffer.substring(parser.state.offset, parser.state.offset + 50)
  );
  process.exit(-1);
}
