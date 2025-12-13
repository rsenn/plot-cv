import { any, char, choice, invert, many, option, regex, seq, token } from './lib/parse/fn.js';

function wrap(parser, name) {
  return (str, pos) => {
    console.log(`${name}(#${str.length}, ${pos})`);
    let r; /*= parser(str, pos);
    if(r[0] || name.startsWith('direct')) console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");*/
    return r;
  };
}

export function stylesheet(...args) {
  return wrap(
    seq(
      ws,
      any(seq(charset, any(choice(Comment, Space, Cdo, Cdc)))),
      any(seq(imports, any(choice(Comment, Space, Cdo, Cdc)))),
      any(seq(namespace, any(choice(Comment, Space, Cdo, Cdc)))),
      any(seq(nestedStatement, any(choice(Comment, Space, Cdo, Cdc))))
    ),
    'stylesheet'
  )(...args);
}

export function charset(...args) {
  return wrap(choice(seq(Charset, ws, String, ws, token(';'), ws), seq(Charset, ws, String, ws)), 'charset')(...args);
}

export function imports(...args) {
  return wrap(
    choice(
      seq(Import, ws, choice(String, Uri), ws, mediaQueryList, token(';'), ws),
      seq(Import, ws, choice(String, Uri), ws, token(';'), ws),
      seq(Import, ws, choice(String, Uri), ws, mediaQueryList),
      seq(Import, ws, choice(String, Uri), ws)
    ),
    'imports'
  )(...args);
}

export function namespace(...args) {
  return wrap(
    choice(seq(Namespace, ws, option(seq(namespacePrefix, ws)), choice(String, Uri), ws, token(';'), ws), seq(Namespace, ws, option(seq(namespacePrefix, ws)), choice(String, Uri), ws)),
    'namespace'
  )(...args);
}

export function namespacePrefix(...args) {
  return wrap(ident, 'namespacePrefix')(...args);
}

export function media(...args) {
  return wrap(seq(Media, ws, mediaQueryList, groupRuleBody, ws), 'media')(...args);
}

export function mediaQueryList(...args) {
  return wrap(seq(option(seq(mediaQuery, any(seq(Comma, ws, mediaQuery)))), ws), 'mediaQueryList')(...args);
}

export function mediaQuery(...args) {
  return wrap(choice(seq(option(choice(MediaOnly, Not)), ws, mediaType, ws, any(seq(And, ws, mediaExpression))), seq(mediaExpression, any(seq(And, ws, mediaExpression)))), 'mediaQuery')(...args);
}

export function mediaType(...args) {
  return wrap(ident, 'mediaType')(...args);
}

export function mediaExpression(...args) {
  return wrap(seq(token('('), ws, mediaFeature, option(seq(token(':'), ws, expr)), token(')'), ws), 'mediaExpression')(...args);
}

export function mediaFeature(...args) {
  return wrap(seq(ident, ws), 'mediaFeature')(...args);
}

export function page(...args) {
  return wrap(seq(Page, ws, option(pseudoPage), token('{'), ws, option(declaration), any(seq(token(';'), ws, option(declaration))), token('}'), ws), 'page')(...args);
}

export function pseudoPage(...args) {
  return wrap(seq(token(':'), ident, ws), 'pseudoPage')(...args);
}

export function selectorGroup(...args) {
  return wrap(seq(selector, any(seq(Comma, ws, selector))), 'selectorGroup')(...args);
}

export function selector(...args) {
  return wrap(seq(simpleSelectorSequence, ws, any(seq(combinator, simpleSelectorSequence, ws))), 'selector')(...args);
}

export function combinator(...args) {
  return wrap(choice(seq(Plus, ws), seq(Greater, ws), seq(Tilde, ws), seq(Space, ws)), 'combinator')(...args);
}

export function simpleSelectorSequence(...args) {
  return wrap(
    choice(seq(choice(typeSelector, universal), any(choice(Hash, className, attrib, pseudo, negation))), many(choice(Hash, className, attrib, pseudo, negation))),
    'simpleSelectorSequence'
  )(...args);
}

export function typeSelector(...args) {
  return wrap(seq(option(typeNamespacePrefix), elementName), 'typeSelector')(...args);
}

export function typeNamespacePrefix(...args) {
  return wrap(seq(option(choice(ident, token('*'))), token('|')), 'typeNamespacePrefix')(...args);
}

export function elementName(...args) {
  return wrap(ident, 'elementName')(...args);
}

export function universal(...args) {
  return wrap(seq(option(typeNamespacePrefix), token('*')), 'universal')(...args);
}

export function className(...args) {
  return wrap(seq(token('.'), ident), 'className')(...args);
}

export function attrib(...args) {
  return wrap(
    seq(
      token('['),
      ws,
      option(typeNamespacePrefix),
      ident,
      ws,
      option(seq(choice(PrefixMatch, SuffixMatch, SubstringMatch, token('='), Includes, DashMatch), ws, choice(ident, String), ws)),
      token(']')
    ),
    'attrib'
  )(...args);
}

export function pseudo(...args) {
  return wrap(seq(token(':'), option(token(':')), choice(ident, functionalPseudo)), 'pseudo')(...args);
}

export function functionalPseudo(...args) {
  return wrap(seq(Function, ws, expression, token(')')), 'functionalPseudo')(...args);
}

export function expression(...args) {
  return wrap(many(seq(choice(Plus, Minus, Dimension, UnknownDimension, Number, String, ident), ws)), 'expression')(...args);
}

export function negation(...args) {
  return wrap(seq(PseudoNot, ws, negationArg, ws, token(')')), 'negation')(...args);
}

export function negationArg(...args) {
  return wrap(choice(typeSelector, universal, Hash, className, attrib, pseudo), 'negationArg')(...args);
}

export function operator(...args) {
  return wrap(choice(seq(token('/'), ws), seq(Comma, ws), seq(Space, ws), seq(token('='), ws)), 'operator')(...args);
}

export function property(...args) {
  return wrap(choice(seq(ident, ws), seq(Variable, ws), seq(token('*'), ident), seq(token('_'), ident)), 'property')(...args);
}

export function ruleset(...args) {
  return wrap(choice(seq(selectorGroup, token('{'), ws, option(declarationList), token('}'), ws), seq(any(any), token('{'), ws, option(declarationList), token('}'), ws)), 'ruleset')(...args);
}

export function declarationList(...args) {
  return wrap(seq(any(seq(token(';'), ws)), declaration, ws, any(seq(token(';'), ws, option(declaration)))), 'declarationList')(...args);
}

export function declaration(...args) {
  return wrap(choice(seq(property, token(':'), ws, expr, option(prio)), seq(property, token(':'), ws, value)), 'declaration')(...args);
}

export function prio(...args) {
  return wrap(seq(Important, ws), 'prio')(...args);
}

export function value(...args) {
  return wrap(many(choice(any, block, seq(atKeyword, ws))), 'value')(...args);
}

export function expr(...args) {
  return wrap(seq(term, any(seq(option(operator), term))), 'expr')(...args);
}

export function term(...args) {
  return wrap(
    choice(
      seq(number, ws),
      seq(percentage, ws),
      seq(dimension, ws),
      seq(String, ws),
      seq(UnicodeRange, ws),
      seq(ident, ws),
      variable,
      seq(Uri, ws),
      hexcolor,
      calc,
      func,
      seq(unknownDimension, ws),
      dxImageTransform
    ),
    'term'
  )(...args);
}

export function func(...args) {
  return wrap(seq(Function, ws, expr, token(')'), ws), 'func')(...args);
}

export function dxImageTransform(...args) {
  return wrap(seq(DxImageTransform, ws, expr, token(')'), ws), 'dxImageTransform')(...args);
}

export function hexcolor(...args) {
  return wrap(seq(Hash, ws), 'hexcolor')(...args);
}

export function number(...args) {
  return wrap(seq(option(choice(Plus, Minus)), Number), 'number')(...args);
}

export function percentage(...args) {
  return wrap(seq(option(choice(Plus, Minus)), Percentage), 'percentage')(...args);
}

export function dimension(...args) {
  return wrap(seq(option(choice(Plus, Minus)), Dimension), 'dimension')(...args);
}

export function unknownDimension(...args) {
  return wrap(seq(option(choice(Plus, Minus)), UnknownDimension), 'unknownDimension')(...args);
}

export function any(...args) {
  return wrap(
    choice(
      seq(ident, ws),
      seq(number, ws),
      seq(percentage, ws),
      seq(dimension, ws),
      seq(unknownDimension, ws),
      seq(String, ws),
      seq(Uri, ws),
      seq(Hash, ws),
      seq(UnicodeRange, ws),
      seq(Includes, ws),
      seq(DashMatch, ws),
      seq(token(':'), ws),
      seq(Function, ws, any(choice(any, unused)), token(')'), ws),
      seq(token('('), ws, any(choice(any, unused)), token(')'), ws),
      seq(token('['), ws, any(choice(any, unused)), token(']'), ws)
    ),
    'any'
  )(...args);
}

export function atRule(...args) {
  return wrap(seq(atKeyword, ws, any(any), choice(block, seq(token(';'), ws))), 'atRule')(...args);
}

export function atKeyword(...args) {
  return wrap(seq(token('@'), ident), 'atKeyword')(...args);
}

export function unused(...args) {
  return wrap(choice(block, seq(atKeyword, ws), seq(token(';'), ws), seq(Cdo, ws), seq(Cdc, ws)), 'unused')(...args);
}

export function block(...args) {
  return wrap(seq(token('{'), ws, any(choice(declarationList, nestedStatement, any, block, seq(atKeyword, ws), seq(token(';'), ws))), token('}'), ws), 'block')(...args);
}

export function nestedStatement(...args) {
  return wrap(choice(ruleset, media, page, fontFaceRule, keyframesRule, supportsRule, viewport, counterStyle, fontFeatureValuesRule, atRule), 'nestedStatement')(...args);
}

export function groupRuleBody(...args) {
  return wrap(seq(token('{'), ws, any(nestedStatement), token('}'), ws), 'groupRuleBody')(...args);
}

export function supportsRule(...args) {
  return wrap(seq(Supports, ws, supportsCondition, ws, groupRuleBody), 'supportsRule')(...args);
}

export function supportsCondition(...args) {
  return wrap(choice(supportsNegation, supportsConjunction, supportsDisjunction, supportsConditionInParens), 'supportsCondition')(...args);
}

export function supportsConditionInParens(...args) {
  return wrap(choice(seq(token('('), ws, supportsCondition, ws, token(')')), supportsDeclarationCondition, generalEnclosed), 'supportsConditionInParens')(...args);
}

export function supportsNegation(...args) {
  return wrap(seq(Not, ws, Space, ws, supportsConditionInParens), 'supportsNegation')(...args);
}

export function supportsConjunction(...args) {
  return wrap(seq(supportsConditionInParens, many(seq(ws, Space, ws, And, ws, Space, ws, supportsConditionInParens))), 'supportsConjunction')(...args);
}

export function supportsDisjunction(...args) {
  return wrap(seq(supportsConditionInParens, many(seq(ws, Space, ws, Or, ws, Space, ws, supportsConditionInParens))), 'supportsDisjunction')(...args);
}

export function supportsDeclarationCondition(...args) {
  return wrap(seq(token('('), ws, declaration, token(')')), 'supportsDeclarationCondition')(...args);
}

export function generalEnclosed(...args) {
  return wrap(seq(choice(Function, token('(')), any(choice(any, unused)), token(')')), 'generalEnclosed')(...args);
}

export function variable(...args) {
  return wrap(seq(Var, ws, Variable, ws, token(')'), ws), 'variable')(...args);
}

export function calc(...args) {
  return wrap(seq(Calc, ws, calcSum, token(')'), ws), 'calc')(...args);
}

export function calcSum(...args) {
  return wrap(seq(calcProduct, any(seq(Space, ws, choice(Plus, Minus), ws, Space, ws, calcProduct))), 'calcSum')(...args);
}

export function calcProduct(...args) {
  return wrap(seq(calcValue, any(choice(seq(token('*'), ws, calcValue), seq(token('/'), ws, number, ws)))), 'calcProduct')(...args);
}

export function calcValue(...args) {
  return wrap(choice(seq(number, ws), seq(dimension, ws), seq(unknownDimension, ws), seq(percentage, ws), seq(token('('), ws, calcSum, token(')'), ws)), 'calcValue')(...args);
}

export function fontFaceRule(...args) {
  return wrap(seq(FontFace, ws, token('{'), ws, option(fontFaceDeclaration), any(seq(token(';'), ws, option(fontFaceDeclaration))), token('}'), ws), 'fontFaceRule')(...args);
}

export function fontFaceDeclaration(...args) {
  return wrap(choice(seq(property, token(':'), ws, expr), seq(property, token(':'), ws, value)), 'fontFaceDeclaration')(...args);
}

export function keyframesRule(...args) {
  return wrap(seq(Keyframes, ws, Space, ws, ident, ws, token('{'), ws, keyframesBlocks, token('}'), ws), 'keyframesRule')(...args);
}

export function keyframesBlocks(...args) {
  return wrap(any(seq(keyframeSelector, token('{'), ws, option(declarationList), token('}'), ws)), 'keyframesBlocks')(...args);
}

export function keyframeSelector(...args) {
  return wrap(seq(choice(From, To, Percentage), ws, any(seq(Comma, ws, choice(From, To, Percentage), ws))), 'keyframeSelector')(...args);
}

export function viewport(...args) {
  return wrap(seq(Viewport, ws, token('{'), ws, option(declarationList), token('}'), ws), 'viewport')(...args);
}

export function counterStyle(...args) {
  return wrap(seq(CounterStyle, ws, ident, ws, token('{'), ws, option(declarationList), token('}'), ws), 'counterStyle')(...args);
}

export function fontFeatureValuesRule(...args) {
  return wrap(seq(FontFeatureValues, ws, fontFamilyNameList, ws, token('{'), ws, any(featureValueBlock), token('}'), ws), 'fontFeatureValuesRule')(...args);
}

export function fontFamilyNameList(...args) {
  return wrap(seq(fontFamilyName, any(seq(ws, Comma, ws, fontFamilyName))), 'fontFamilyNameList')(...args);
}

export function fontFamilyName(...args) {
  return wrap(choice(String, seq(ident, any(seq(ws, ident)))), 'fontFamilyName')(...args);
}

export function featureValueBlock(...args) {
  return wrap(seq(featureType, ws, token('{'), ws, option(featureValueDefinition), any(seq(ws, token(';'), ws, option(featureValueDefinition))), token('}'), ws), 'featureValueBlock')(...args);
}

export function featureType(...args) {
  return wrap(atKeyword, 'featureType')(...args);
}

export function featureValueDefinition(...args) {
  return wrap(seq(ident, ws, token(':'), ws, number, any(seq(ws, number))), 'featureValueDefinition')(...args);
}

export function ident(...args) {
  return wrap(choice(Ident, MediaOnly, Not, And, Or, From, To), 'ident')(...args);
}

export function ws(...args) {
  return wrap(any(choice(Comment, Space)), 'ws')(...args);
}

export function Hex(...args) {
  return wrap(regex(/[0-9a-fA-F]/g), 'Hex')(...args);
}

export function NewlineOrSpace(...args) {
  return wrap(choice(token('\r\n'), regex(/[ \t\r\n\f]/g), empty()), 'NewlineOrSpace')(...args);
}

export function Unicode(...args) {
  return wrap(seq(token('\\'), Hex, option(Hex), option(Hex), option(Hex), option(Hex), option(Hex), NewlineOrSpace), 'Unicode')(...args);
}

export function Escape(...args) {
  return wrap(choice(Unicode, seq(token('\\'), invert(regex(/[\r\n\f0-9a-fA-F]/g)))), 'Escape')(...args);
}

export function Nmstart(...args) {
  return wrap(choice(regex(/[_a-zA-Z]/g), Nonascii, Escape), 'Nmstart')(...args);
}

export function Nmchar(...args) {
  return wrap(choice(regex(/[_a-zA-Z0-9\-]/g), Nonascii, Escape), 'Nmchar')(...args);
}

export function Comment(...args) {
  return wrap(seq(token('/*'), invert(any(token('*'))), many(token('*')), any(seq(invert(regex(/[\/*]/g)), invert(any(token('*'))), many(token('*')))), token('/')), 'Comment')(...args);
}

export function Name(...args) {
  return wrap(many(Nmchar), 'Name')(...args);
}

export function Url(...args) {
  return wrap(any(choice(regex(/[!#$%&*-~]/g), Nonascii, Escape)), 'Url')(...args);
}

export function Space(...args) {
  return wrap(regex(/[ \t\r\n\f]+/g), 'Space')(...args);
}

export function Whitespace(...args) {
  return wrap(choice(Space, empty()), 'Whitespace')(...args);
}

export function Newline(...args) {
  return wrap(choice(char('\n'), token('\r\n'), char('\r'), token('\f')), 'Newline')(...args);
}

export function ZeroToFourZeros(...args) {
  return wrap(seq(option(token('0')), option(token('0')), option(token('0')), option(token('0'))), 'ZeroToFourZeros')(...args);
}

export function A(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('41'), token('61')), NewlineOrSpace), option()), 'A')(...args);
}

export function B(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('42'), token('62')), NewlineOrSpace), option()), 'B')(...args);
}

export function C(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('43'), token('63')), NewlineOrSpace), option()), 'C')(...args);
}

export function D(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('44'), token('64')), NewlineOrSpace), option()), 'D')(...args);
}

export function E(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('45'), token('65')), NewlineOrSpace), option()), 'E')(...args);
}

export function F(...args) {
  return wrap(seq(seq(token('\\'), ZeroToFourZeros, choice(token('46'), token('66')), NewlineOrSpace), option()), 'F')(...args);
}

export function G(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('47'), token('67')), NewlineOrSpace), token('\\g'), token('\\G')), option()), 'G')(...args);
}

export function H(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('48'), token('68')), NewlineOrSpace), token('\\h'), token('\\H')), option()), 'H')(...args);
}

export function I(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('49'), token('69')), NewlineOrSpace), token('\\i'), token('\\I')), option()), 'I')(...args);
}

export function K(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('4b'), token('6b')), NewlineOrSpace), token('\\k'), token('\\K')), option()), 'K')(...args);
}

export function L(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('4c'), token('6c')), NewlineOrSpace), token('\\l'), token('\\L')), option()), 'L')(...args);
}

export function M(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('4d'), token('6d')), NewlineOrSpace), token('\\m'), token('\\M')), option()), 'M')(...args);
}

export function N(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('4e'), token('6e')), NewlineOrSpace), token('\\n'), token('\\N')), option()), 'N')(...args);
}

export function O(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('4f'), token('6f')), NewlineOrSpace), token('\\o'), token('\\O')), option()), 'O')(...args);
}

export function P(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('50'), token('70')), NewlineOrSpace), token('\\p'), token('\\P')), option()), 'P')(...args);
}

export function Q(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('51'), token('71')), NewlineOrSpace), token('\\q'), token('\\Q')), option()), 'Q')(...args);
}

export function R(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('52'), token('72')), NewlineOrSpace), token('\\r'), token('\\R')), option()), 'R')(...args);
}

export function S(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('53'), token('73')), NewlineOrSpace), token('\\s'), token('\\S')), option()), 'S')(...args);
}

export function T(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('54'), token('74')), NewlineOrSpace), token('\\t'), token('\\T')), option()), 'T')(...args);
}

export function U(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('55'), token('75')), NewlineOrSpace), token('\\u'), token('\\U')), option()), 'U')(...args);
}

export function V(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('56'), token('76')), NewlineOrSpace), token('\\v'), token('\\V')), option()), 'V')(...args);
}

export function W(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('57'), token('77')), NewlineOrSpace), token('\\w'), token('\\W')), option()), 'W')(...args);
}

export function X(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('58'), token('78')), NewlineOrSpace), token('\\x'), token('\\X')), option()), 'X')(...args);
}

export function Y(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('59'), token('79')), NewlineOrSpace), token('\\y'), token('\\Y')), option()), 'Y')(...args);
}

export function Z(...args) {
  return wrap(seq(choice(seq(token('\\'), ZeroToFourZeros, choice(token('5a'), token('7a')), NewlineOrSpace), token('\\z'), token('\\Z')), option()), 'Z')(...args);
}

export function DashChar(...args) {
  return wrap(choice(token('-'), seq(token('\\'), ZeroToFourZeros, token('2d'), NewlineOrSpace)), 'DashChar')(...args);
}

export function Cdo(...args) {
  return wrap(token('<!--'), 'Cdo')(...args);
}

export function Cdc(...args) {
  return wrap(token('-->'), 'Cdc')(...args);
}

export function Includes(...args) {
  return wrap(token('~='), 'Includes')(...args);
}

export function DashMatch(...args) {
  return wrap(token('|='), 'DashMatch')(...args);
}

export function Hash(...args) {
  return wrap(seq(token('#'), Name), 'Hash')(...args);
}

export function Import(...args) {
  return wrap(seq(token('@'), I, M, P, O, R, T), 'Import')(...args);
}

export function Page(...args) {
  return wrap(seq(token('@'), P, A, G, E), 'Page')(...args);
}

export function Media(...args) {
  return wrap(seq(token('@'), M, E, D, I, A), 'Media')(...args);
}

export function Namespace(...args) {
  return wrap(seq(token('@'), N, A, M, E, S, P, A, C, E), 'Namespace')(...args);
}

export function AtKeyword(...args) {
  return wrap(seq(token('@'), Ident), 'AtKeyword')(...args);
}

export function Charset(...args) {
  return wrap(char('@charset '), 'Charset')(...args);
}

export function Important(...args) {
  return wrap(seq(token('!'), any(choice(Space, Comment)), I, M, P, O, R, T, A, N, T), 'Important')(...args);
}

export function FontRelative(...args) {
  return wrap(choice(seq(Number, E, M), seq(Number, E, X), seq(Number, C, H), seq(Number, R, E, M)), 'FontRelative')(...args);
}

export function ViewportRelative(...args) {
  return wrap(choice(seq(Number, V, W), seq(Number, V, H), seq(Number, V, M, I, N), seq(Number, V, M, A, X)), 'ViewportRelative')(...args);
}

export function AbsLength(...args) {
  return wrap(choice(seq(Number, P, X), seq(Number, C, M), seq(Number, M, M), seq(Number, I, N), seq(Number, P, T), seq(Number, P, C), seq(Number, Q)), 'AbsLength')(...args);
}

export function Angle(...args) {
  return wrap(choice(seq(Number, D, E, G), seq(Number, R, A, D), seq(Number, G, R, A, D), seq(Number, T, U, R, N)), 'Angle')(...args);
}

export function Time(...args) {
  return wrap(choice(seq(Number, M, S), seq(Number, S)), 'Time')(...args);
}

export function Freq(...args) {
  return wrap(choice(seq(Number, H, Z), seq(Number, K, H, Z)), 'Freq')(...args);
}

export function Percentage(...args) {
  return wrap(seq(Number, token('%')), 'Percentage')(...args);
}

export function Uri(...args) {
  return wrap(choice(seq(U, R, L, token('('), Whitespace, String, Whitespace, token(')')), seq(U, R, L, token('('), Whitespace, Url, Whitespace, token(')'))), 'Uri')(...args);
}

export function UnicodeRange(...args) {
  return wrap(
    choice(
      seq(regex(/[u|U]/g), token('+?'), option(token('?')), option(token('?')), option(token('?')), option(token('?')), option(token('?'))),
      seq(regex(/[u|U]/g), token('+'), Hex, option(token('?')), option(token('?')), option(token('?')), option(token('?')), option(token('?'))),
      seq(regex(/[u|U]/g), token('+'), Hex, Hex, option(token('?')), option(token('?')), option(token('?')), option(token('?'))),
      seq(regex(/[u|U]/g), token('+'), Hex, Hex, Hex, option(token('?')), option(token('?')), option(token('?'))),
      seq(regex(/[u|U]/g), token('+'), Hex, Hex, Hex, Hex, option(token('?')), option(token('?'))),
      seq(regex(/[u|U]/g), token('+'), Hex, Hex, Hex, Hex, Hex, option(token('?')))
    ),
    'UnicodeRange'
  )(...args);
}

export function MediaOnly(...args) {
  return wrap(seq(O, N, L, Y), 'MediaOnly')(...args);
}

export function Not(...args) {
  return wrap(seq(N, O, T), 'Not')(...args);
}

export function And(...args) {
  return wrap(seq(A, N, D), 'And')(...args);
}

export function Resolution(...args) {
  return wrap(choice(seq(Number, D, P, I), seq(Number, D, P, C, M), seq(Number, D, P, P, X)), 'Resolution')(...args);
}

export function Length(...args) {
  return wrap(choice(AbsLength, FontRelative, ViewportRelative), 'Length')(...args);
}

export function Dimension(...args) {
  return wrap(choice(Length, Time, Freq, Resolution, Angle), 'Dimension')(...args);
}

export function UnknownDimension(...args) {
  return wrap(seq(Number, Ident), 'UnknownDimension')(...args);
}

export function Nonascii(...args) {
  return wrap(invert(regex(/[\u0000-\u007f]/g)), 'Nonascii')(...args);
}

export function Plus(...args) {
  return wrap(token('+'), 'Plus')(...args);
}

export function Minus(...args) {
  return wrap(token('-'), 'Minus')(...args);
}

export function Greater(...args) {
  return wrap(token('>'), 'Greater')(...args);
}

export function Comma(...args) {
  return wrap(token(','), 'Comma')(...args);
}

export function Tilde(...args) {
  return wrap(token('~'), 'Tilde')(...args);
}

export function PseudoNot(...args) {
  return wrap(seq(token(':'), N, O, T, token('(')), 'PseudoNot')(...args);
}

export function Number(...args) {
  return wrap(choice(regex(/[0-9]+/g), seq(regex(/[0-9]*/g), token('.'), regex(/[0-9]+/g))), 'Number')(...args);
}

export function String(...args) {
  return wrap(
    choice(
      seq(token('"'), any(choice(invert(regex(/[\n\r\f\\"]/g)), seq(token('\\'), Newline), Nonascii, Escape)), token('"')),
      seq(token("'"), any(choice(invert(regex(/[\n\r\f\\']/g)), seq(token('\\'), Newline), Nonascii, Escape)), token("'"))
    ),
    'String'
  )(...args);
}

export function PrefixMatch(...args) {
  return wrap(token('^='), 'PrefixMatch')(...args);
}

export function SuffixMatch(...args) {
  return wrap(token('$='), 'SuffixMatch')(...args);
}

export function SubstringMatch(...args) {
  return wrap(token('*='), 'SubstringMatch')(...args);
}

export function FontFace(...args) {
  return wrap(seq(token('@'), F, O, N, T, DashChar, F, A, C, E), 'FontFace')(...args);
}

export function Supports(...args) {
  return wrap(seq(token('@'), S, U, P, P, O, R, T, S), 'Supports')(...args);
}

export function Or(...args) {
  return wrap(seq(O, R), 'Or')(...args);
}

export function VendorPrefix(...args) {
  return wrap(choice(seq(token('-'), M, O, Z, token('-')), seq(token('-'), W, E, B, K, I, T, token('-')), seq(token('-'), O, token('-'))), 'VendorPrefix')(...args);
}

export function Keyframes(...args) {
  return wrap(seq(token('@'), option(VendorPrefix), K, E, Y, F, R, A, M, E, S), 'Keyframes')(...args);
}

export function From(...args) {
  return wrap(seq(F, R, O, M), 'From')(...args);
}

export function To(...args) {
  return wrap(seq(T, O), 'To')(...args);
}

export function Calc(...args) {
  return wrap(token('calc('), 'Calc')(...args);
}

export function Viewport(...args) {
  return wrap(seq(token('@'), V, I, E, W, P, O, R, T), 'Viewport')(...args);
}

export function CounterStyle(...args) {
  return wrap(seq(token('@'), C, O, U, N, T, E, R, DashChar, S, T, Y, L, E), 'CounterStyle')(...args);
}

export function FontFeatureValues(...args) {
  return wrap(seq(token('@'), F, O, N, T, DashChar, F, E, A, T, U, R, E, DashChar, V, A, L, U, E, S), 'FontFeatureValues')(...args);
}

export function DxImageTransform(...args) {
  return wrap(seq(token('progid:DXImageTransform.Microsoft.'), Function), 'DxImageTransform')(...args);
}

export function Variable(...args) {
  return wrap(seq(token('--'), Nmstart, any(Nmchar)), 'Variable')(...args);
}

export function Var(...args) {
  return wrap(token('variable('), 'Var')(...args);
}

export function Ident(...args) {
  return wrap(seq(option(token('-')), Nmstart, any(Nmchar)), 'Ident')(...args);
}

export function Function(...args) {
  return wrap(seq(Ident, token('(')), 'Function')(...args);
}

export default {
  stylesheet,
  charset,
  imports,
  namespace,
  namespacePrefix,
  media,
  mediaQueryList,
  mediaQuery,
  mediaType,
  mediaExpression,
  mediaFeature,
  page,
  pseudoPage,
  selectorGroup,
  selector,
  combinator,
  simpleSelectorSequence,
  typeSelector,
  typeNamespacePrefix,
  elementName,
  universal,
  className,
  attrib,
  pseudo,
  functionalPseudo,
  expression,
  negation,
  negationArg,
  operator,
  property,
  ruleset,
  declarationList,
  declaration,
  prio,
  value,
  expr,
  term,
  func,
  dxImageTransform,
  hexcolor,
  number,
  percentage,
  dimension,
  unknownDimension,
  any,
  atRule,
  atKeyword,
  unused,
  block,
  nestedStatement,
  groupRuleBody,
  supportsRule,
  supportsCondition,
  supportsConditionInParens,
  supportsNegation,
  supportsConjunction,
  supportsDisjunction,
  supportsDeclarationCondition,
  generalEnclosed,
  variable,
  calc,
  calcSum,
  calcProduct,
  calcValue,
  fontFaceRule,
  fontFaceDeclaration,
  keyframesRule,
  keyframesBlocks,
  keyframeSelector,
  viewport,
  counterStyle,
  fontFeatureValuesRule,
  fontFamilyNameList,
  fontFamilyName,
  featureValueBlock,
  featureType,
  featureValueDefinition,
  ident,
  ws,
  Hex,
  NewlineOrSpace,
  Unicode,
  Escape,
  Nmstart,
  Nmchar,
  Comment,
  Name,
  Url,
  Space,
  Whitespace,
  Newline,
  ZeroToFourZeros,
  A,
  B,
  C,
  D,
  E,
  F,
  G,
  H,
  I,
  K,
  L,
  M,
  N,
  O,
  P,
  Q,
  R,
  S,
  T,
  U,
  V,
  W,
  X,
  Y,
  Z,
  DashChar,
  Cdo,
  Cdc,
  Includes,
  DashMatch,
  Hash,
  Import,
  Page,
  Media,
  Namespace,
  AtKeyword,
  Charset,
  Important,
  FontRelative,
  ViewportRelative,
  AbsLength,
  Angle,
  Time,
  Freq,
  Percentage,
  Uri,
  UnicodeRange,
  MediaOnly,
  Not,
  And,
  Resolution,
  Length,
  Dimension,
  UnknownDimension,
  Nonascii,
  Plus,
  Minus,
  Greater,
  Comma,
  Tilde,
  PseudoNot,
  Number,
  String,
  PrefixMatch,
  SuffixMatch,
  SubstringMatch,
  FontFace,
  Supports,
  Or,
  VendorPrefix,
  Keyframes,
  From,
  To,
  Calc,
  Viewport,
  CounterStyle,
  FontFeatureValues,
  DxImageTransform,
  Variable,
  Var,
  Ident,
  Function
};