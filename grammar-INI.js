import { any, choice, eof, ignore, many, option, regex, seq, token } from './lib/parse/fn.js';

function wrap(parser, name) {
  return (str, pos) => {
    let r = parser(str, pos);
    if(r[0] || name.startsWith('direct')) console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");
    return r;
  };
}

function ini(...args) {
  return wrap(seq(any(choice(LINE_COMMENT, section)), eof()), 'ini')(...args);
}

function section(...args) {
  return wrap(seq(section_header, option(WS), any(key_value)), 'section')(...args);
}

function section_header(...args) {
  return wrap(seq(LBRACK, section_header_title, RBRACK), 'section_header')(...args);
}

function section_header_title(...args) {
  return wrap(regex(/[A-Za-z0-9_]+/g), 'section_header_title')(...args);
}

function key_values(...args) {
  return wrap(many(key_value), 'key_values')(...args);
}

function key_value(...args) {
  return wrap(seq(key, EQUALS, value, option(WS)), 'key_value')(...args);
}

function key(...args) {
  return wrap(regex(/[-A-Za-z0-9_{}]+/g), 'key')(...args);
}

function value(...args) {
  return wrap(regex(/[^\n\r]*/g), 'value')(...args);
}

function text(...args) {
  return wrap(TEXT, 'text')(...args);
}

function TEXT(...args) {
  return wrap(many(regex(/[a-zA-Z_0-9\/\\:\*\.,@ ]/g)), 'TEXT')(...args);
}

function EQUALS(...args) {
  return wrap(ignore(regex(/[=]/g)), 'EQUALS')(...args);
}

function LBRACK(...args) {
  return wrap(ignore(token('[')), 'LBRACK')(...args);
}

function RBRACK(...args) {
  return wrap(ignore(token(']')), 'RBRACK')(...args);
}

function LINE_COMMENT(...args) {
  return wrap(seq(token(';'), any(regex(/[^\n\r]/g))), 'LINE_COMMENT')(...args);
}

function WS(...args) {
  return wrap(ignore(regex(/[\r\n\t\s]+/g)), 'WS')(...args);
}

export default {
  ini,
  section,
  section_header,
  section_header_title,
  key_values,
  key_value,
  key,
  value,
  text,
  TEXT,
  EQUALS,
  LBRACK,
  RBRACK,
  LINE_COMMENT,
  WS
};