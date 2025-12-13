import { choice, option, seq, token } from './lib/parse/fn.js';

function wrap(parser, name) {
  return (str, pos) => {
    let r = parser(str, pos);
    if(r[0] || name.startsWith('direct')) console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");
    return r;
  };
}

function primaryExpression(...args) {
  return wrap(choice(identifier, constant, stringLiteral, seq(token('('), expression, token(')'))), 'primaryExpression')(...args);
}

function postfixExpression(...args) {
  return wrap(seq(primaryExpression, option()), 'postfixExpression')(...args);
}

function argumentExpressionList(...args) {
  return wrap(seq(assignmentExpression, option()), 'argumentExpressionList')(...args);
}

function unaryExpression(...args) {
  return wrap(choice(postfixExpression, seq(token('++'), unaryExpression), seq(token('--'), unaryExpression), seq(unaryOperator, castExpression)), 'unaryExpression')(...args);
}

function unaryOperator(...args) {
  return wrap(choice(token('&'), token('*'), token('+'), token('-'), token('~'), token('!')), 'unaryOperator')(...args);
}

function castExpression(...args) {
  return wrap(unaryExpression, 'castExpression')(...args);
}

function multiplicativeExpression(...args) {
  return wrap(seq(castExpression, option()), 'multiplicativeExpression')(...args);
}

function additiveExpression(...args) {
  return wrap(seq(multiplicativeExpression, option()), 'additiveExpression')(...args);
}

function shiftExpression(...args) {
  return wrap(seq(additiveExpression, option()), 'shiftExpression')(...args);
}

function relationalExpression(...args) {
  return wrap(seq(shiftExpression, option()), 'relationalExpression')(...args);
}

function equalityExpression(...args) {
  return wrap(seq(relationalExpression, option()), 'equalityExpression')(...args);
}

function andExpression(...args) {
  return wrap(seq(equalityExpression, option()), 'andExpression')(...args);
}

function exclusiveOrExpression(...args) {
  return wrap(seq(andExpression, option()), 'exclusiveOrExpression')(...args);
}

function inclusiveOrExpression(...args) {
  return wrap(seq(exclusiveOrExpression, option()), 'inclusiveOrExpression')(...args);
}

function logicalAndExpression(...args) {
  return wrap(seq(inclusiveOrExpression, option()), 'logicalAndExpression')(...args);
}

function logicalOrExpression(...args) {
  return wrap(seq(logicalAndExpression, option()), 'logicalOrExpression')(...args);
}

function conditionalExpression(...args) {
  return wrap(choice(logicalOrExpression, seq(logicalOrExpression, token('?'), expression, token(':'), conditionalExpression)), 'conditionalExpression')(...args);
}

function assignmentExpression(...args) {
  return wrap(choice(conditionalExpression, seq(unaryExpression, assignmentOperator, assignmentExpression)), 'assignmentExpression')(...args);
}

function assignmentOperator(...args) {
  return wrap(choice(token('='), token('*='), token('/='), token('%='), token('+='), token('-='), token('<<='), token('>>='), token('&='), token('^='), token('|=')), 'assignmentOperator')(...args);
}

function expression(...args) {
  return wrap(seq(assignmentExpression, option()), 'expression')(...args);
}

export default {
  primaryExpression,
  postfixExpression,
  argumentExpressionList,
  unaryExpression,
  unaryOperator,
  castExpression,
  multiplicativeExpression,
  additiveExpression,
  shiftExpression,
  relationalExpression,
  equalityExpression,
  andExpression,
  exclusiveOrExpression,
  inclusiveOrExpression,
  logicalAndExpression,
  logicalOrExpression,
  conditionalExpression,
  assignmentExpression,
  assignmentOperator,
  expression
};