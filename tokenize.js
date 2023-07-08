import { Lexer } from './lib/lexer.js';
import tokenRules from './token-rules.js';
function makeLexer(...args) {
  var lexer = new Lexer(...args);

  AddRule('singleLineComment', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'COMMENT', keyword: false }));
  AddRule('multiLineComment', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'COMMENT', keyword: false }));
  AddRule('while', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'WHILE', keyword: true }));
  AddRule('volatile', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'VOLATILE', keyword: true }));
  AddRule('void', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'VOID', keyword: true }));
  AddRule('unsigned', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'UNSIGNED', keyword: true }));
  AddRule('union', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'UNION', keyword: true }));
  AddRule('typedef', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'TYPEDEF', keyword: true }));
  AddRule('switch', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'SWITCH', keyword: true }));
  AddRule('struct', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'STRUCT', keyword: true }));
  AddRule('static', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'STATIC', keyword: true }));
  AddRule('sizeof', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'SIZEOF', keyword: true }));
  AddRule('signed', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'SIGNED', keyword: true }));
  AddRule('short', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'SHORT', keyword: true }));
  AddRule('return', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'RETURN', keyword: true }));
  AddRule('register', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'REGISTER', keyword: true }));
  AddRule('long', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'LONG', keyword: true }));
  AddRule('int', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'INT', keyword: true }));
  AddRule('if', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'IF', keyword: true }));
  AddRule('goto', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'GOTO', keyword: true }));
  AddRule('for', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'FOR', keyword: true }));
  AddRule('float', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'FLOAT', keyword: true }));
  AddRule('extern', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'EXTERN', keyword: true }));
  AddRule('enum', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'ENUM', keyword: true }));
  AddRule('else', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'ELSE', keyword: true }));
  AddRule('double', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'DOUBLE', keyword: true }));
  AddRule('do', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'DO', keyword: true }));
  AddRule('default', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'DEFAULT', keyword: true }));
  AddRule('continue', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONTINUE', keyword: true }));
  AddRule('const', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONST', keyword: true }));
  AddRule('char', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CHAR', keyword: true }));
  AddRule('case', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CASE', keyword: true }));
  AddRule('break', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'BREAK', keyword: true }));
  AddRule('auto', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'AUTO', keyword: true }));
  AddRule('bool', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'BOOL', keyword: true }));
  AddRule('complex', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'COMPLEX', keyword: true }));
  AddRule('imaginary', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'IMAGINARY', keyword: true }));
  AddRule('inline', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'INLINE', keyword: true }));
  AddRule('restrict', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'RESTRICT', keyword: true }));
  AddRule('identifier', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'IDENTIFIER' }));
  AddRule('hexadecimal', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('octal', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('decimal', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('char_literal', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('floatWithoutPoint', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('floatWithNothingBeforePoint', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('floatWithNothingAfterPoint', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' }));
  AddRule('string_literal', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'STRING_LITERAL' }));
  AddRule('ellipsis', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'ELLIPSIS' }));
  AddRule('right_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'RIGHT_ASSIGN' }));
  AddRule('left_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'LEFT_ASSIGN' }));
  AddRule('add_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'ADD_ASSIGN' }));
  AddRule('sub_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'SUB_ASSIGN' }));
  AddRule('mul_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'MUL_ASSIGN' }));
  AddRule('div_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'DIV_ASSIGN' }));
  AddRule('mod_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'MOD_ASSIGN' }));
  AddRule('and_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'AND_ASSIGN' }));
  AddRule('xor_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'XOR_ASSIGN' }));
  AddRule('or_assign', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'OR_ASSIGN' }));
  AddRule('right_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'RIGHT_OP' }));
  AddRule('left_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'LEFT_OP' }));
  AddRule('inc_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'INC_OP' }));
  AddRule('dec_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'DEC_OP' }));
  AddRule('ptr_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'PTR_OP' }));
  AddRule('and_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'AND_OP' }));
  AddRule('or_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'OR_OP' }));
  AddRule('le_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'LE_OP' }));
  AddRule('ge_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'GE_OP' }));
  AddRule('eq_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'EQ_OP' }));
  AddRule('ne_op', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'NE_OP' }));
  AddRule(';', lexeme => token({ lexeme, loc: { line, column }, tokenClass: ';' }));
  AddRule('{', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '{' }));
  AddRule('}', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '}' }));
  AddRule(',', lexeme => token({ lexeme, loc: { line, column }, tokenClass: ',' }));
  AddRule(':', lexeme => token({ lexeme, loc: { line, column }, tokenClass: ':' }));
  AddRule('=', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '=' }));
  AddRule('(', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '(' }));
  AddRule(')', lexeme => token({ lexeme, loc: { line, column }, tokenClass: ')' }));
  AddRule('[', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '[' }));
  AddRule(']', lexeme => token({ lexeme, loc: { line, column }, tokenClass: ']' }));
  AddRule('.', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '.' }));
  AddRule('&', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '&' }));
  AddRule('!', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '!' }));
  AddRule('~', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '~' }));
  AddRule('-', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '-' }));
  AddRule('+', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '+' }));
  AddRule('*', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '*' }));
  AddRule('/', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '/' }));
  AddRule('%', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '%' }));
  AddRule('<', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '<' }));
  AddRule('>', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '>' }));
  AddRule('^', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '^' }));
  AddRule('|', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '|' }));
  AddRule('?', lexeme => token({ lexeme, loc: { line, column }, tokenClass: '?' }));
  AddRule('whitespace', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'WHITESPACE' }));
  AddRule('unmatched', lexeme => token({ lexeme, loc: { line, column }, tokenClass: 'UNMATCHED' }));

  function token(tok) {
    count(tok.lexeme);
    return tok;
  }
  function AddRule(name, re, ...rest) {
    lexer.addRule(name, tokenRules[name], ...rest);
  }

  return lexer;
}
var line = 1,
  column = 1;

export function count(lexeme) {
  for(var i = 0; i < lexeme.length; i++) {
    if(lexeme[i] == '\n') {
      line = line + 1;
      column = 1;
    } else if(lexeme[i] == '\t') {
      column = column + (4 - (column % 4));
    } else {
      column = column + 1;
    }
  }
}

export function* tokenize(streamOfText) {
  line = 1;
  column = 1;

  let lexer = makeLexer(streamOfText);

  var x = lexer.lex();
  while(x != undefined) {
    if(x.tokenClass != 'UNMATCHED' && x.tokenClass != 'WHITESPACE' && x.tokenClass != 'COMMENT') {
      yield x;
    }
    x = lexer.lex();
  }
}

export default tokenize;