import { Lexer } from './lib/parse/lexer.js';
import { Grammar, Parser } from './lib/parse/grammar.js';
import fs from 'fs';
import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 10, colors: true }
});
let filename = './lib/grammars/C.g4';
let src = fs.readFileSync(filename).toString();

//let lex = new Lexer(src, filename);
let grammar = new Grammar(src, filename);

grammar.parse();
//grammar.resolveRules();

filename = './seek_set.c';
src = fs.readFileSync(filename).toString();
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
