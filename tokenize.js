import tokenRules from './token-rules.js';
import Lexer from './lib/lexer.js';
var lexer = new Lexer();

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

function token(tok) {
  count(tok.lexeme);
  return tok;
}

export async function* tokenize(streamOfText) {
  line = 1;
  column = 1;
  lexer.addRule(tokenRules['singleLineComment'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'COMMENT', keyword: false })
  );
  lexer.addRule(tokenRules['multiLineComment'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'COMMENT', keyword: false })
  );
  lexer.addRule(tokenRules['while'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'WHILE', keyword: true })
  );
  lexer.addRule(tokenRules['while'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'WHILE', keyword: true })
  );
  lexer.addRule(tokenRules['volatile'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'VOLATILE', keyword: true })
  );
  lexer.addRule(tokenRules['void'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'VOID', keyword: true })
  );
  lexer.addRule(tokenRules['unsigned'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'UNSIGNED', keyword: true })
  );
  lexer.addRule(tokenRules['union'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'UNION', keyword: true })
  );
  lexer.addRule(tokenRules['typedef'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'TYPEDEF', keyword: true })
  );
  lexer.addRule(tokenRules['switch'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'SWITCH', keyword: true })
  );
  lexer.addRule(tokenRules['struct'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'STRUCT', keyword: true })
  );
  lexer.addRule(tokenRules['static'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'STATIC', keyword: true })
  );
  lexer.addRule(tokenRules['sizeof'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'SIZEOF', keyword: true })
  );
  lexer.addRule(tokenRules['signed'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'SIGNED', keyword: true })
  );
  lexer.addRule(tokenRules['short'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'SHORT', keyword: true })
  );
  lexer.addRule(tokenRules['return'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'RETURN', keyword: true })
  );
  lexer.addRule(tokenRules['register'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'REGISTER', keyword: true })
  );
  lexer.addRule(tokenRules['long'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'LONG', keyword: true })
  );
  lexer.addRule(tokenRules['int'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'INT', keyword: true })
  );
  lexer.addRule(tokenRules['if'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'IF', keyword: true })
  );
  lexer.addRule(tokenRules['goto'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'GOTO', keyword: true })
  );
  lexer.addRule(tokenRules['for'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'FOR', keyword: true })
  );
  lexer.addRule(tokenRules['float'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'FLOAT', keyword: true })
  );
  lexer.addRule(tokenRules['extern'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'EXTERN', keyword: true })
  );
  lexer.addRule(tokenRules['enum'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'ENUM', keyword: true })
  );
  lexer.addRule(tokenRules['else'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'ELSE', keyword: true })
  );
  lexer.addRule(tokenRules['double'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'DOUBLE', keyword: true })
  );
  lexer.addRule(tokenRules['do'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'DO', keyword: true })
  );
  lexer.addRule(tokenRules['default'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'DEFAULT', keyword: true })
  );
  lexer.addRule(tokenRules['continue'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONTINUE', keyword: true })
  );
  lexer.addRule(tokenRules['const'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONST', keyword: true })
  );
  lexer.addRule(tokenRules['char'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CHAR', keyword: true })
  );
  lexer.addRule(tokenRules['case'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CASE', keyword: true })
  );
  lexer.addRule(tokenRules['break'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'BREAK', keyword: true })
  );
  lexer.addRule(tokenRules['auto'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'AUTO', keyword: true })
  );
  lexer.addRule(tokenRules['bool'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'BOOL', keyword: true })
  );
  lexer.addRule(tokenRules['complex'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'COMPLEX', keyword: true })
  );
  lexer.addRule(tokenRules['imaginary'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'IMAGINARY', keyword: true })
  );
  lexer.addRule(tokenRules['inline'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'INLINE', keyword: true })
  );
  lexer.addRule(tokenRules['restrict'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'RESTRICT', keyword: true })
  );
  lexer.addRule(tokenRules['identifier'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'IDENTIFIER' })
  );
  lexer.addRule(tokenRules['hexadecimal'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['octal'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['decimal'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['char_literal'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['floatWithoutPoint'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['floatWithNothingBeforePoint'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['floatWithNothingAfterPoint'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'CONSTANT' })
  );
  lexer.addRule(tokenRules['string_literal'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'STRING_LITERAL' })
  );
  lexer.addRule(tokenRules['ellipsis'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'ELLIPSIS' })
  );
  lexer.addRule(tokenRules['right_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'RIGHT_ASSIGN' })
  );
  lexer.addRule(tokenRules['left_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'LEFT_ASSIGN' })
  );
  lexer.addRule(tokenRules['add_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'ADD_ASSIGN' })
  );
  lexer.addRule(tokenRules['sub_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'SUB_ASSIGN' })
  );
  lexer.addRule(tokenRules['mul_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'MUL_ASSIGN' })
  );
  lexer.addRule(tokenRules['div_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'DIV_ASSIGN' })
  );
  lexer.addRule(tokenRules['mod_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'MOD_ASSIGN' })
  );
  lexer.addRule(tokenRules['and_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'AND_ASSIGN' })
  );
  lexer.addRule(tokenRules['xor_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'XOR_ASSIGN' })
  );
  lexer.addRule(tokenRules['or_assign'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'OR_ASSIGN' })
  );
  lexer.addRule(tokenRules['right_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'RIGHT_OP' })
  );
  lexer.addRule(tokenRules['left_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'LEFT_OP' })
  );
  lexer.addRule(tokenRules['inc_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'INC_OP' })
  );
  lexer.addRule(tokenRules['dec_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'DEC_OP' })
  );
  lexer.addRule(tokenRules['ptr_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'PTR_OP' })
  );
  lexer.addRule(tokenRules['and_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'AND_OP' })
  );
  lexer.addRule(tokenRules['or_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'OR_OP' })
  );
  lexer.addRule(tokenRules['le_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'LE_OP' })
  );
  lexer.addRule(tokenRules['ge_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'GE_OP' })
  );
  lexer.addRule(tokenRules['eq_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'EQ_OP' })
  );
  lexer.addRule(tokenRules['ne_op'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'NE_OP' })
  );
  lexer.addRule(tokenRules[';'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: ';' })
  );
  lexer.addRule(tokenRules['{'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '{' })
  );
  lexer.addRule(tokenRules['}'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '}' })
  );
  lexer.addRule(tokenRules[','], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: ',' })
  );
  lexer.addRule(tokenRules[':'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: ':' })
  );
  lexer.addRule(tokenRules['='], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '=' })
  );
  lexer.addRule(tokenRules['('], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '(' })
  );
  lexer.addRule(tokenRules[')'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: ')' })
  );
  lexer.addRule(tokenRules['['], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '[' })
  );
  lexer.addRule(tokenRules[']'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: ']' })
  );
  lexer.addRule(tokenRules['.'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '.' })
  );
  lexer.addRule(tokenRules['&'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '&' })
  );
  lexer.addRule(tokenRules['!'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '!' })
  );
  lexer.addRule(tokenRules['~'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '~' })
  );
  lexer.addRule(tokenRules['-'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '-' })
  );
  lexer.addRule(tokenRules['+'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '+' })
  );
  lexer.addRule(tokenRules['*'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '*' })
  );
  lexer.addRule(tokenRules['/'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '/' })
  );
  lexer.addRule(tokenRules['%'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '%' })
  );
  lexer.addRule(tokenRules['<'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '<' })
  );
  lexer.addRule(tokenRules['>'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '>' })
  );
  lexer.addRule(tokenRules['^'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '^' })
  );
  lexer.addRule(tokenRules['|'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '|' })
  );
  lexer.addRule(tokenRules['?'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: '?' })
  );
  lexer.addRule(tokenRules['whitespace'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'WHITESPACE' })
  );
  lexer.addRule(tokenRules['unmatched'], lexeme =>
    token({ lexeme, loc: { line, column }, tokenClass: 'UNMATCHED' })
  );

  lexer.setInput(streamOfText);
  var x = lexer.lex();
  while(x != undefined) {
    if(x.tokenClass != 'UNMATCHED' && x.tokenClass != 'WHITESPACE' && x.tokenClass != 'COMMENT') {
      yield x;
    }
    x = lexer.lex();
  }
}

export default tokenize;
