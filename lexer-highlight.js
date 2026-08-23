/**
 * Syntax highlighter for project-server.js, built on the real tokenizers
 * in quickjs/qjs-modules/lib/lexer/{c,ecmascript}.js (a rule-based Lexer,
 * not a regex scanner) instead of a hand-rolled regex highlighter.
 */

import CLexer from './quickjs/qjs-modules/lib/lexer/c.js';
import ECMAScriptLexer from './quickjs/qjs-modules/lib/lexer/ecmascript.js';
import CMakeLexer from './quickjs/qjs-modules/lib/lexer/cmake.js';

const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

const ALIASES = { javascript: 'js', jsx: 'js', mjs: 'js', cjs: 'js', ts: 'js', tsx: 'js', h: 'c', cpp: 'c', cc: 'c', hpp: 'c', cxx: 'c' };

/* -------------------------------------------------------------------- C */

const C_KEYWORD = new Set([
  'while', 'volatile', 'union', 'typedef', 'switch', 'struct', 'static', 'sizeof', 'signed', 'short',
  'return', 'register', 'long', 'if', 'goto', 'for', 'extern', 'enum', 'else', 'do', 'default',
  'continue', 'const', 'case', 'break', 'auto', 'inline', 'restrict', 'unsigned',
]);
const C_TYPE = new Set(['void', 'int', 'float', 'double', 'char', 'bool', 'complex', 'imaginary']);
const C_COMMENT = new Set(['singleLineComment', 'multiLineComment']);
const C_STRING = new Set(['char_literal', 'string_literal']);
const C_NUMBER = new Set(['hexadecimal', 'octal', 'decimal', 'floatWithoutPoint', 'floatWithNothingBeforePoint', 'floatWithNothingAfterPoint']);
const C_OP = new Set([
  'ellipsis', 'right_assign', 'left_assign', 'add_assign', 'sub_assign', 'mul_assign', 'div_assign',
  'mod_assign', 'and_assign', 'xor_assign', 'or_assign', 'right_op', 'left_op', 'inc_op', 'dec_op',
  'ptr_op', 'and_op', 'or_op', 'le_op', 'ge_op', 'eq_op', 'ne_op', 'equal', 'amp', 'bang', 'tilde',
  'minus', 'plus', 'star', 'slash', 'percent', 'lt', 'gt', 'caret', 'pipe', 'question',
]);

function classifyC(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'preprocessor': return 'pre';
    case 'whitespace': return null;
    case 'identifier': {
      let j = i + 1;
      while(tokens[j] && tokens[j].type === 'whitespace') j++;
      if(tokens[j] && tokens[j].type === 'lparen') return 'fn';
      if(/_t$/.test(tok.lexeme) || /^[A-Z]/.test(tok.lexeme)) return 'type';
      return 'id';
    }
    default:
      if(C_COMMENT.has(tok.type)) return 'cm';
      if(C_STRING.has(tok.type)) return 'str';
      if(C_NUMBER.has(tok.type)) return 'num';
      if(C_TYPE.has(tok.type)) return 'type';
      if(C_KEYWORD.has(tok.type)) return 'kw';
      if(C_OP.has(tok.type)) return 'op';
      return null; // braces, parens, brackets, comma, semi, dot, colon: plain
  }
}

/* ------------------------------------------------------------------- JS */

const JS_STRING = new Set(['stringLiteral', 'regexpLiteral', 'templateLiteral', 'templateLiteralHead', 'templateLiteralPart', 'templateLiteralTail']);
const JS_PLAIN_PUNCT = new Set(['{', '}', '(', ')', '[', ']', ',', ';', '.', ':']);

function classifyJs(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'comment': return 'cm';
    case 'numericLiteral': return 'num';
    case 'booleanLiteral': case 'nullLiteral': return 'atom';
    case 'keyword': return 'kw';
    case 'privateIdentifier': return 'id';
    case 'whitespace': case 'shebang': case 'nestedEnd': return null;
    case 'punctuator': return JS_PLAIN_PUNCT.has(tok.lexeme) ? null : 'op';
    case 'identifier': {
      let j = i + 1;
      while(tokens[j] && tokens[j].type === 'whitespace') j++;
      if(tokens[j] && tokens[j].type === 'punctuator' && tokens[j].lexeme === '(') return 'fn';
      if(/^[A-Z]/.test(tok.lexeme)) return 'type';
      return 'id';
    }
    default:
      if(JS_STRING.has(tok.type)) return 'str';
      return null;
  }
}

/* ---------------------------------------------------------------- CMake */

const CMAKE_KEYWORD = new Set([
  'if', 'elseif', 'else', 'endif', 'foreach', 'endforeach', 'while', 'endwhile',
  'function', 'endfunction', 'macro', 'endmacro', 'break', 'continue', 'return',
]);
const CMAKE_ATOM = new Set(['TRUE', 'FALSE', 'ON', 'OFF', 'YES', 'NO']);

function classifyCMake(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'bracketComment': case 'lineComment': return 'cm';
    case 'bracketArgument': case 'quotedArgument': return 'str';
    case 'variableRef': return 'var';
    case 'whitespace': case 'newline': return null;
    case 'identifier': {
      let j = i + 1;
      while(tokens[j] && tokens[j].type === 'whitespace') j++;
      if(tokens[j] && tokens[j].type === 'lparen') return 'fn';
      if(CMAKE_KEYWORD.has(tok.lexeme.toLowerCase())) return 'kw';
      return 'id';
    }
    case 'unquotedArgument': return CMAKE_ATOM.has(tok.lexeme) ? 'atom' : null;
    default:
      return null; // lparen, rparen, dollar: plain
  }
}

/* ------------------------------------------------------------------- api */

const LEXERS = {
  c: [src => new CLexer(src, undefined, 'source'), classifyC],
  js: [src => new ECMAScriptLexer(src, 'source'), classifyJs],
  cmake: [src => new CMakeLexer(src, undefined, 'source'), classifyCMake],
};

/**
 * @param {string} code raw source text
 * @param {string} lang 'c', 'js', or an alias; unknown languages are escaped as-is
 * @returns {string} HTML with <span class="t-*"> token markup
 */
export function highlight(code, lang) {
  const key = ALIASES[lang] || lang;
  const entry = LEXERS[key];
  if(!entry) return esc(code);
  const [makeLexer, classify] = entry;

  let tokens;
  try {
    tokens = [...makeLexer(code)];
  } catch(e) {
    return esc(code); // malformed/unsupported input: fall back to plain text
  }

  let out = '';
  for(let i = 0; i < tokens.length; i++) {
    const tok = tokens[i];
    const cls = classify(tokens, i);
    out += cls ? '<span class="t-' + cls + '">' + esc(tok.lexeme) + '</span>' : esc(tok.lexeme);
  }
  return out;
}
