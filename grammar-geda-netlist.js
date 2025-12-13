import { any, choice, eof, ignore, regex, seq, token } from './lib/parse/fn.js';

function wrap(parser, name) {
  return (str, pos) => {
    let r = parser(str, pos);
    /* if(r[0] || name.startsWith('direct'))
      console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");*/
    return r;
  };
}

function geda_netlist(...args) {
  return wrap(seq(components, nets, eof()), 'geda_netlist')(...args);
}

function value(...args) {
  return wrap(regex(/[^\r\n()\x5b\x5d]*/g), 'value')(...args);
}

function values(...args) {
  return wrap(any(seq(value, NL)), 'values')(...args);
}

function components(...args) {
  return wrap(any(choice(LINE_COMMENT, component)), 'components')(...args);
}

function nets(...args) {
  return wrap(any(choice(LINE_COMMENT, net)), 'nets')(...args);
}

function component(...args) {
  return wrap(seq(LBRACK, values, RBRACK), 'component')(...args);
}

function net(...args) {
  return wrap(seq(LPAREN, values, RPAREN), 'net')(...args);
}

function DELIM(...args) {
  return wrap(regex(/[\n\r\(\)\[\]]/g), 'DELIM')(...args);
}

function TEXT(...args) {
  return wrap(regex(/[^\n\r\(\)\[\]]/g), 'TEXT')(...args);
}

function LBRACK(...args) {
  return wrap(ignore(token('[')), 'LBRACK')(...args);
}

function RBRACK(...args) {
  return wrap(ignore(token(']')), 'RBRACK')(...args);
}

function LPAREN(...args) {
  return wrap(ignore(token('(')), 'LPAREN')(...args);
}

function RPAREN(...args) {
  return wrap(ignore(token(')')), 'RPAREN')(...args);
}

function LINE_COMMENT(...args) {
  return wrap(seq(token(';'), any(regex(/[^\n\r]/g))), 'LINE_COMMENT')(...args);
}

function NL(...args) {
  return wrap(ignore(regex(/[\r\n]/g)), 'NL')(...args);
}

export default {
  geda_netlist,
  value,
  values,
  components,
  nets,
  component,
  net,
  DELIM,
  TEXT,
  LBRACK,
  RBRACK,
  LPAREN,
  RPAREN,
  LINE_COMMENT,
  NL
};