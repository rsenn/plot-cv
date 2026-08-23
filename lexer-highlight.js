/**
 * Syntax highlighter for project-server.js, built on the real tokenizers
 * in quickjs/qjs-modules/lib/lexer/{c,ecmascript}.js (a rule-based Lexer,
 * not a regex scanner) instead of a hand-rolled regex highlighter.
 */

import CLexer from './quickjs/qjs-modules/lib/lexer/c.js';
import ECMAScriptLexer from './quickjs/qjs-modules/lib/lexer/ecmascript.js';
import CMakeLexer from './quickjs/qjs-modules/lib/lexer/cmake.js';
import ShellLexer from './quickjs/qjs-modules/lib/lexer/shell.js';
import XMLLexer from './quickjs/qjs-modules/lib/lexer/xml.js';
import { GNUMakeLexer } from './quickjs/qjs-modules/lib/lexer/make.js';
import IniLexer from './quickjs/qjs-modules/lib/lexer/ini.js';
import CSVLexer from './quickjs/qjs-modules/lib/lexer/csv.js';
import BNFLexer from './quickjs/qjs-modules/lib/lexer/bnf.js';

const esc = s => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;');

const ALIASES = {
  javascript: 'js', jsx: 'js', mjs: 'js', cjs: 'js', ts: 'js', tsx: 'js', h: 'c', cpp: 'c', cc: 'c', hpp: 'c', cxx: 'c',
  bash: 'sh', shell: 'sh', html: 'xml', htm: 'xml', svg: 'xml', makefile: 'make', mk: 'make', yacc: 'bnf', lex: 'bnf',
};

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

/* ----------------------------------------------------------------- Shell */

const SH_KEYWORD = new Set(['bang', 'case', 'do', 'done', 'elif', 'else', 'esac', 'fi', 'for', 'if', 'in', 'then', 'until', 'while']);
const SH_OP = new Set([
  'dsemi', 'semi', 'and_if', 'backgnd', 'or_if', 'pipe', 'bq', 'dlessdash', 'dless', 'lessand',
  'lessgreat', 'less', 'dgreat', 'greatand', 'clobber', 'great',
]);

function classifyShell(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'comment': return 'cm';
    case 'newline': case 'whitespace': return null;
    case 'io_number': return 'num';
    case 'name': return 'id';
    default:
      if(SH_KEYWORD.has(tok.type)) return 'kw';
      if(SH_OP.has(tok.type)) return 'op';
      return null; // lparen, rparen, lbrace, rbrace, word/assign/redir placeholders: plain
  }
}

/* ------------------------------------------------------------------- XML */

function classifyXml(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'comment': case 'bangTag': return 'cm';
    case 'tagStart': case 'closeTagStart': case 'gt': case 'slash': return 'pre';
    case 'tagName': return 'type';
    case 'attrName': return 'id';
    case 'eq': return 'op';
    case 'quoted': case 'quotedSingle': return 'str';
    default:
      return null; // text, ws: plain
  }
}

/* ------------------------------------------------------------------- Make */

const MAKE_KEYWORD = new Set([
  'bangDirective', 'ifeq', 'ifneq', 'ifdef', 'ifndef', 'else', 'endif', 'includeIgnore', 'sinclude',
  'include', 'override', 'export', 'unexport', 'undefine', 'vpath', 'private', 'define', 'endef',
]);
const MAKE_OP = new Set(['dcolon', 'colon', 'assign', 'assignImmediate', 'assignSimple', 'assignAppend', 'assignConditional', 'assignShell']);
const MAKE_STR = new Set(['string', 'stringSingle']);
const MAKE_VAR = new Set(['varOpen', 'varClose', 'automaticVar', 'varText']);

function classifyMake(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'comment': return 'cm';
    case 'name': return 'id';
    case 'newline': case 'defineNewline': case 'whitespace': case 'lineContinuation':
    case 'defineBackslash': case 'recipePrefix': case 'recipeBackslash': return null;
    default:
      if(MAKE_KEYWORD.has(tok.type)) return 'kw';
      if(MAKE_OP.has(tok.type)) return 'op';
      if(MAKE_STR.has(tok.type)) return 'str';
      if(MAKE_VAR.has(tok.type)) return 'var';
      return null; // recipeText, defineBody, punctuation: plain
  }
}

/* -------------------------------------------------------------------- Ini */

function classifyIni(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'comment': return 'cm';
    case 'section': return 'type';
    case 'string': case 'stringSingle': return 'str';
    case 'equals': return 'op';
    default: return null; // text, newline, whitespace: plain
  }
}

/* -------------------------------------------------------------------- CSV */

function classifyCsv(tokens, i) {
  const tok = tokens[i];
  switch(tok.type) {
    case 'separator': return 'op';
    case 'field': return tok.lexeme.startsWith('"') ? 'str' : null;
    default: return null; // nl: plain
  }
}

/* -------------------------------------------------------------------- BNF */

const BNF_KEYWORD = new Set(['keyword', 'lexstart', 'directive', 'd_name', 'p_state']);
const BNF_STR = new Set(['char', 'chars', 'literal', 'd_string', 'p_literal', 'char_class', 'p_class']);
const BNF_COMMENT = new Set(['multiline_comment', 'singleline_comment']);
const BNF_OP = new Set(['bar', 'r_pipe', 'p_bar', 'dotdot', 'arrow', 'equals', 'tilde', 'asterisk', 'plus', 'question', 'p_postfix']);

function classifyBnf(tokens, i) {
  const tok = tokens[i];
  // Embedded C ("<C>..." rules) and JS ("<JS>..." rules) action code: keep
  // plain rather than reimplementing classifyC/classifyJs's lookahead logic
  // against these differently-prefixed token types.
  if(tok.type.startsWith('c_') || tok.type.startsWith('js_')) return null;
  switch(tok.type) {
    case 'identifier': case 'r_identifier': case 'l_identifier': case 'd_identifier': return 'id';
    default:
      if(BNF_COMMENT.has(tok.type)) return 'cm';
      if(BNF_STR.has(tok.type)) return 'str';
      if(BNF_KEYWORD.has(tok.type)) return 'kw';
      if(BNF_OP.has(tok.type)) return 'op';
      return null;
  }
}

/* ------------------------------------------------------------------- api */

const LEXERS = {
  c: [src => new CLexer(src, undefined, 'source'), classifyC],
  js: [src => new ECMAScriptLexer(src, 'source'), classifyJs],
  cmake: [src => new CMakeLexer(src, undefined, 'source'), classifyCMake],
  sh: [src => new ShellLexer(src, undefined, 'source'), classifyShell],
  xml: [src => new XMLLexer(src, 'source'), classifyXml],
  make: [src => new GNUMakeLexer(src, undefined, 'source'), classifyMake],
  ini: [src => new IniLexer(src, undefined, 'source'), classifyIni],
  csv: [src => new CSVLexer(src, 'source'), classifyCsv],
  bnf: [src => new BNFLexer(src, 'source'), classifyBnf],
};

/**
 * @param {string} code raw source text
 * @param {string} lang 'c', 'js', 'cmake', 'sh', 'xml', or an alias; unknown
 *   languages are escaped as-is
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

  // Some lexers (shell, xml) silently discard characters they don't have a
  // rule for (e.g. via a skip() handler) rather than erroring or tokenizing
  // them - walk token.charRange gaps and emit that source verbatim so no
  // text is ever lost from the rendered output.
  let out = '';
  let pos = 0;
  for(let i = 0; i < tokens.length; i++) {
    const tok = tokens[i];
    const [start, end] = tok.charRange;
    if(start > pos) out += esc(code.slice(pos, start));
    const cls = classify(tokens, i);
    out += cls ? '<span class="t-' + cls + '">' + esc(tok.lexeme) + '</span>' : esc(tok.lexeme);
    pos = end;
  }
  if(pos < code.length) out += esc(code.slice(pos));
  return out;
}
