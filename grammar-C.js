import { any, char, choice, eof, ignore, invert, many, option, regex, seq, token } from '../parse/fn.js';

function wrap(parser, name) {
  return (str, pos) => {
    let r = parser(str, pos);
    if(r[0] || name.startsWith('direct')) console.log('matched (' + name + ') ' + pos + ' - ' + r[2] + ": '", r[1], "'");
    return r;
  };
}

function primaryExpression(...args) {
  return wrap(
    choice(
      Identifier,
      Constant,
      many(StringLiteral),
      seq(token('('), expression, token(')')),
      genericSelection,
      seq(option(token('__extension__')), token('('), compoundStatement, token(')')),
      seq(token('__builtin_va_arg'), token('('), unaryExpression, token(','), typeName, token(')')),
      seq(token('__builtin_offsetof'), token('('), typeName, token(','), unaryExpression, token(')'))
    ),
    'primaryExpression'
  )(...args);
}

function genericSelection(...args) {
  return wrap(seq(token('_Generic'), token('('), assignmentExpression, token(','), genericAssocList, token(')')), 'genericSelection')(...args);
}

function genericAssocList(...args) {
  return wrap(seq(genericAssociation, option(seq(token(','), genericAssociation))), 'genericAssocList')(...args);
}

function genericAssociation(...args) {
  return wrap(choice(seq(typeName, token(':'), assignmentExpression), seq(token('default'), token(':'), assignmentExpression)), 'genericAssociation')(...args);
}

function postfixExpression(...args) {
  return wrap(
    seq(
      choice(
        primaryExpression,
        seq(token('('), typeName, token(')'), token('{'), initializerList, token('}')),
        seq(token('('), typeName, token(')'), token('{'), initializerList, token(','), token('}')),
        seq(token('__extension__'), token('('), typeName, token(')'), token('{'), initializerList, token('}')),
        seq(token('__extension__'), token('('), typeName, token(')'), token('{'), initializerList, token(','), token('}'))
      ),
      option(
        choice(
          seq(token('['), expression, token(']')),
          seq(token('('), option(argumentExpressionList), token(')')),
          seq(token('.'), Identifier),
          seq(token('->'), Identifier),
          token('++'),
          token('--')
        )
      )
    ),
    'postfixExpression'
  )(...args);
}

function argumentExpressionList(...args) {
  return wrap(seq(assignmentExpression, option(seq(token(','), assignmentExpression))), 'argumentExpressionList')(...args);
}

function unaryExpression(...args) {
  return wrap(
    choice(
      postfixExpression,
      seq(token('++'), unaryExpression),
      seq(token('--'), unaryExpression),
      seq(unaryOperator, castExpression),
      seq(token('sizeof'), unaryExpression),
      seq(token('sizeof'), token('('), typeName, token(')')),
      seq(token('_Alignof'), token('('), typeName, token(')')),
      seq(token('&&'), Identifier)
    ),
    'unaryExpression'
  )(...args);
}

function unaryOperator(...args) {
  return wrap(choice(token('&'), token('*'), token('+'), token('-'), token('~'), token('!')), 'unaryOperator')(...args);
}

function castExpression(...args) {
  return wrap(
    choice(seq(token('('), typeName, token(')'), castExpression), seq(token('__extension__'), token('('), typeName, token(')'), castExpression), unaryExpression, DigitSequence),
    'castExpression'
  )(...args);
}

function multiplicativeExpression(...args) {
  return wrap(seq(castExpression, option(choice(seq(token('*'), castExpression), seq(token('/'), castExpression), seq(token('%'), castExpression)))), 'multiplicativeExpression')(...args);
}

function additiveExpression(...args) {
  return wrap(seq(multiplicativeExpression, option(choice(seq(token('+'), multiplicativeExpression), seq(token('-'), multiplicativeExpression)))), 'additiveExpression')(...args);
}

function shiftExpression(...args) {
  return wrap(seq(additiveExpression, option(choice(seq(token('<<'), additiveExpression), seq(token('>>'), additiveExpression)))), 'shiftExpression')(...args);
}

function relationalExpression(...args) {
  return wrap(
    seq(shiftExpression, option(choice(seq(token('<'), shiftExpression), seq(token('>'), shiftExpression), seq(token('<='), shiftExpression), seq(token('>='), shiftExpression)))),
    'relationalExpression'
  )(...args);
}

function equalityExpression(...args) {
  return wrap(seq(relationalExpression, option(choice(seq(token('=='), relationalExpression), seq(token('!='), relationalExpression)))), 'equalityExpression')(...args);
}

function andExpression(...args) {
  return wrap(seq(equalityExpression, option(seq(token('&'), equalityExpression))), 'andExpression')(...args);
}

function exclusiveOrExpression(...args) {
  return wrap(seq(andExpression, option(seq(token('^'), andExpression))), 'exclusiveOrExpression')(...args);
}

function inclusiveOrExpression(...args) {
  return wrap(seq(exclusiveOrExpression, option(seq(token('|'), exclusiveOrExpression))), 'inclusiveOrExpression')(...args);
}

function logicalAndExpression(...args) {
  return wrap(seq(inclusiveOrExpression, option(seq(token('&&'), inclusiveOrExpression))), 'logicalAndExpression')(...args);
}

function logicalOrExpression(...args) {
  return wrap(seq(logicalAndExpression, option(seq(token('||'), logicalAndExpression))), 'logicalOrExpression')(...args);
}

function conditionalExpression(...args) {
  return wrap(seq(logicalOrExpression, option(seq(token('?'), expression, token(':'), conditionalExpression))), 'conditionalExpression')(...args);
}

function assignmentExpression(...args) {
  return wrap(choice(conditionalExpression, seq(unaryExpression, assignmentOperator, assignmentExpression), DigitSequence), 'assignmentExpression')(...args);
}

function assignmentOperator(...args) {
  return wrap(choice(token('='), token('*='), token('/='), token('%='), token('+='), token('-='), token('<<='), token('>>='), token('&='), token('^='), token('|=')), 'assignmentOperator')(...args);
}

function expression(...args) {
  return wrap(seq(assignmentExpression, option(seq(token(','), assignmentExpression))), 'expression')(...args);
}

function constantExpression(...args) {
  return wrap(conditionalExpression, 'constantExpression')(...args);
}

function declaration(...args) {
  return wrap(choice(seq(declarationSpecifiers, initDeclaratorList, token(';')), seq(declarationSpecifiers, token(';')), staticAssertDeclaration), 'declaration')(...args);
}

function declarationSpecifiers(...args) {
  return wrap(many(declarationSpecifier), 'declarationSpecifiers')(...args);
}

function declarationSpecifiers2(...args) {
  return wrap(many(declarationSpecifier), 'declarationSpecifiers2')(...args);
}

function declarationSpecifier(...args) {
  return wrap(choice(storageClassSpecifier, typeSpecifier, typeQualifier, functionSpecifier, alignmentSpecifier), 'declarationSpecifier')(...args);
}

function initDeclaratorList(...args) {
  return wrap(seq(initDeclarator, option(seq(token(','), initDeclarator))), 'initDeclaratorList')(...args);
}

function initDeclarator(...args) {
  return wrap(choice(declarator, seq(declarator, token('='), initializer)), 'initDeclarator')(...args);
}

function storageClassSpecifier(...args) {
  return wrap(choice(token('typedef'), token('extern'), token('static'), token('_Thread_local'), token('auto'), token('register')), 'storageClassSpecifier')(...args);
}

function typeSpecifier(...args) {
  return wrap(
    seq(
      choice(
        choice(
          token('void'),
          token('char'),
          token('short'),
          token('int'),
          token('long'),
          token('float'),
          token('double'),
          token('signed'),
          token('unsigned'),
          token('_Bool'),
          token('_Complex'),
          token('__m128'),
          token('__m128d'),
          token('__m128i')
        ),
        seq(token('__extension__'), token('('), choice(token('__m128'), token('__m128d'), token('__m128i')), token(')')),
        atomicTypeSpecifier,
        structOrUnionSpecifier,
        enumSpecifier,
        typedefName,
        seq(token('__typeof__'), token('('), constantExpression, token(')'))
      ),
      option(pointer)
    ),
    'typeSpecifier'
  )(...args);
}

function structOrUnionSpecifier(...args) {
  return wrap(choice(seq(structOrUnion, option(Identifier), token('{'), structDeclarationList, token('}')), seq(structOrUnion, Identifier)), 'structOrUnionSpecifier')(...args);
}

function structOrUnion(...args) {
  return wrap(choice(token('struct'), token('union')), 'structOrUnion')(...args);
}

function structDeclarationList(...args) {
  return wrap(seq(structDeclaration, option(structDeclaration)), 'structDeclarationList')(...args);
}

function structDeclaration(...args) {
  return wrap(choice(seq(specifierQualifierList, option(structDeclaratorList), token(';')), staticAssertDeclaration), 'structDeclaration')(...args);
}

function specifierQualifierList(...args) {
  return wrap(choice(seq(typeSpecifier, option(specifierQualifierList)), seq(typeQualifier, option(specifierQualifierList))), 'specifierQualifierList')(...args);
}

function structDeclaratorList(...args) {
  return wrap(seq(structDeclarator, option(seq(token(','), structDeclarator))), 'structDeclaratorList')(...args);
}

function structDeclarator(...args) {
  return wrap(choice(declarator, seq(option(declarator), token(':'), constantExpression)), 'structDeclarator')(...args);
}

function enumSpecifier(...args) {
  return wrap(
    choice(
      seq(token('enum'), option(Identifier), token('{'), enumeratorList, token('}')),
      seq(token('enum'), option(Identifier), token('{'), enumeratorList, token(','), token('}')),
      seq(token('enum'), Identifier)
    ),
    'enumSpecifier'
  )(...args);
}

function enumeratorList(...args) {
  return wrap(seq(enumerator, option(seq(token(','), enumerator))), 'enumeratorList')(...args);
}

function enumerator(...args) {
  return wrap(choice(enumerationConstant, seq(enumerationConstant, token('='), constantExpression)), 'enumerator')(...args);
}

function enumerationConstant(...args) {
  return wrap(Identifier, 'enumerationConstant')(...args);
}

function atomicTypeSpecifier(...args) {
  return wrap(seq(token('_Atomic'), token('('), typeName, token(')')), 'atomicTypeSpecifier')(...args);
}

function typeQualifier(...args) {
  return wrap(choice(token('const'), token('restrict'), token('volatile'), token('_Atomic')), 'typeQualifier')(...args);
}

function functionSpecifier(...args) {
  return wrap(
    choice(choice(token('inline'), token('_Noreturn'), token('__inline__'), token('__stdcall')), gccAttributeSpecifier, seq(token('__declspec'), token('('), Identifier, token(')'))),
    'functionSpecifier'
  )(...args);
}

function alignmentSpecifier(...args) {
  return wrap(choice(seq(token('_Alignas'), token('('), typeName, token(')')), seq(token('_Alignas'), token('('), constantExpression, token(')'))), 'alignmentSpecifier')(...args);
}

function declarator(...args) {
  return wrap(seq(option(pointer), directDeclarator, any(gccDeclaratorExtension)), 'declarator')(...args);
}

function directDeclarator(...args) {
  return wrap(
    seq(
      choice(Identifier, seq(token('('), declarator, token(')')), seq(Identifier, token(':'), DigitSequence), seq(token('('), option(typeSpecifier), pointer, directDeclarator, token(')'))),
      option(
        choice(
          seq(token('['), option(typeQualifierList), option(assignmentExpression), token(']')),
          seq(token('['), token('static'), option(typeQualifierList), assignmentExpression, token(']')),
          seq(token('['), typeQualifierList, token('static'), assignmentExpression, token(']')),
          seq(token('['), option(typeQualifierList), token('*'), token(']')),
          seq(token('('), parameterTypeList, token(')')),
          seq(token('('), option(identifierList), token(')'))
        )
      )
    ),
    'directDeclarator'
  )(...args);
}

function gccDeclaratorExtension(...args) {
  return wrap(choice(seq(token('__asm'), token('('), many(StringLiteral), token(')')), gccAttributeSpecifier), 'gccDeclaratorExtension')(...args);
}

function gccAttributeSpecifier(...args) {
  return wrap(seq(token('__attribute__'), token('('), token('('), gccAttributeList, token(')'), token(')')), 'gccAttributeSpecifier')(...args);
}

function gccAttributeList(...args) {
  return wrap(choice(seq(gccAttribute, any(seq(token(','), gccAttribute))), empty()), 'gccAttributeList')(...args);
}

function gccAttribute(...args) {
  return wrap(choice(seq(invert(choice(token(','), token('('), token(')'))), option(seq(token('('), option(argumentExpressionList), token(')')))), empty()), 'gccAttribute')(...args);
}

function nestedParenthesesBlock(...args) {
  return wrap(any(choice(invert(choice(token('('), token(')'))), seq(token('('), nestedParenthesesBlock, token(')')))), 'nestedParenthesesBlock')(...args);
}

function pointer(...args) {
  return wrap(
    choice(
      seq(token('*'), option(typeQualifierList)),
      seq(token('*'), option(typeQualifierList), pointer),
      seq(token('^'), option(typeQualifierList)),
      seq(token('^'), option(typeQualifierList), pointer)
    ),
    'pointer'
  )(...args);
}

function typeQualifierList(...args) {
  return wrap(seq(typeQualifier, option(typeQualifier)), 'typeQualifierList')(...args);
}

function parameterTypeList(...args) {
  return wrap(choice(parameterList, seq(parameterList, token(','), token('...'))), 'parameterTypeList')(...args);
}

function parameterList(...args) {
  return wrap(seq(parameterDeclaration, option(seq(token(','), parameterDeclaration))), 'parameterList')(...args);
}

function parameterDeclaration(...args) {
  return wrap(choice(seq(declarationSpecifiers, declarator), seq(declarationSpecifiers2, option(abstractDeclarator))), 'parameterDeclaration')(...args);
}

function identifierList(...args) {
  return wrap(seq(Identifier, option(seq(token(','), Identifier))), 'identifierList')(...args);
}

function typeName(...args) {
  return wrap(seq(specifierQualifierList, option(abstractDeclarator)), 'typeName')(...args);
}

function abstractDeclarator(...args) {
  return wrap(choice(pointer, seq(option(pointer), directAbstractDeclarator, any(gccDeclaratorExtension))), 'abstractDeclarator')(...args);
}

function directAbstractDeclarator(...args) {
  return wrap(
    seq(
      choice(
        seq(token('('), abstractDeclarator, token(')'), any(gccDeclaratorExtension)),
        seq(token('['), option(typeQualifierList), option(assignmentExpression), token(']')),
        seq(token('['), token('static'), option(typeQualifierList), assignmentExpression, token(']')),
        seq(token('['), typeQualifierList, token('static'), assignmentExpression, token(']')),
        seq(token('['), token('*'), token(']')),
        seq(token('('), option(parameterTypeList), token(')'), any(gccDeclaratorExtension))
      ),
      option(
        choice(
          seq(token('['), option(typeQualifierList), option(assignmentExpression), token(']')),
          seq(token('['), token('static'), option(typeQualifierList), assignmentExpression, token(']')),
          seq(token('['), typeQualifierList, token('static'), assignmentExpression, token(']')),
          seq(token('['), token('*'), token(']')),
          seq(token('('), option(parameterTypeList), token(')'), any(gccDeclaratorExtension))
        )
      )
    ),
    'directAbstractDeclarator'
  )(...args);
}

function typedefName(...args) {
  return wrap(Identifier, 'typedefName')(...args);
}

function initializer(...args) {
  return wrap(choice(assignmentExpression, seq(token('{'), initializerList, token('}')), seq(token('{'), initializerList, token(','), token('}'))), 'initializer')(...args);
}

function initializerList(...args) {
  return wrap(seq(seq(option(designation), initializer), option(seq(token(','), option(designation), initializer))), 'initializerList')(...args);
}

function designation(...args) {
  return wrap(seq(designatorList, token('=')), 'designation')(...args);
}

function designatorList(...args) {
  return wrap(seq(designator, option(designator)), 'designatorList')(...args);
}

function designator(...args) {
  return wrap(choice(seq(token('['), constantExpression, token(']')), seq(token('.'), Identifier)), 'designator')(...args);
}

function staticAssertDeclaration(...args) {
  return wrap(seq(token('_Static_assert'), token('('), constantExpression, token(','), many(StringLiteral), token(')'), token(';')), 'staticAssertDeclaration')(...args);
}

function statement(...args) {
  return wrap(
    choice(
      labeledStatement,
      compoundStatement,
      expressionStatement,
      selectionStatement,
      iterationStatement,
      jumpStatement,
      seq(
        choice(token('__asm'), token('__asm__')),
        choice(token('volatile'), token('__volatile__')),
        token('('),
        option(seq(logicalOrExpression, any(seq(token(','), logicalOrExpression)))),
        any(seq(token(':'), option(seq(logicalOrExpression, any(seq(token(','), logicalOrExpression)))))),
        token(')'),
        token(';')
      )
    ),
    'statement'
  )(...args);
}

function labeledStatement(...args) {
  return wrap(choice(seq(Identifier, token(':'), statement), seq(token('case'), constantExpression, token(':'), statement), seq(token('default'), token(':'), statement)), 'labeledStatement')(...args);
}

function compoundStatement(...args) {
  return wrap(seq(token('{'), option(blockItemList), token('}')), 'compoundStatement')(...args);
}

function blockItemList(...args) {
  return wrap(seq(blockItem, option(blockItem)), 'blockItemList')(...args);
}

function blockItem(...args) {
  return wrap(choice(statement, declaration), 'blockItem')(...args);
}

function expressionStatement(...args) {
  return wrap(seq(option(expression), token(';')), 'expressionStatement')(...args);
}

function selectionStatement(...args) {
  return wrap(
    choice(seq(token('if'), token('('), expression, token(')'), statement, option(seq(token('else'), statement))), seq(token('switch'), token('('), expression, token(')'), statement)),
    'selectionStatement'
  )(...args);
}

function iterationStatement(...args) {
  return wrap(
    choice(
      seq(While, token('('), expression, token(')'), statement),
      seq(Do, statement, While, token('('), expression, token(')'), token(';')),
      seq(For, token('('), forCondition, token(')'), statement)
    ),
    'iterationStatement'
  )(...args);
}

function forCondition(...args) {
  return wrap(
    choice(seq(forDeclaration, token(';'), option(forExpression), token(';'), option(forExpression)), seq(option(expression), token(';'), option(forExpression), token(';'), option(forExpression))),
    'forCondition'
  )(...args);
}

function forDeclaration(...args) {
  return wrap(choice(seq(declarationSpecifiers, initDeclaratorList), declarationSpecifiers), 'forDeclaration')(...args);
}

function forExpression(...args) {
  return wrap(seq(assignmentExpression, option(seq(token(','), assignmentExpression))), 'forExpression')(...args);
}

function jumpStatement(...args) {
  return wrap(
    choice(
      seq(token('goto'), Identifier, token(';')),
      seq(token('continue'), token(';')),
      seq(token('break'), token(';')),
      seq(token('return'), option(expression), token(';')),
      seq(token('goto'), unaryExpression, token(';'))
    ),
    'jumpStatement'
  )(...args);
}

function compilationUnit(...args) {
  return wrap(seq(option(translationUnit), eof()), 'compilationUnit')(...args);
}

function translationUnit(...args) {
  return wrap(seq(externalDeclaration, option(externalDeclaration)), 'translationUnit')(...args);
}

function externalDeclaration(...args) {
  return wrap(choice(functionDefinition, declaration, token(';')), 'externalDeclaration')(...args);
}

function functionDefinition(...args) {
  return wrap(seq(option(declarationSpecifiers), declarator, option(declarationList), compoundStatement), 'functionDefinition')(...args);
}

function declarationList(...args) {
  return wrap(seq(declaration, option(declaration)), 'declarationList')(...args);
}

function Auto(...args) {
  return wrap(token('auto'), 'Auto')(...args);
}

function Break(...args) {
  return wrap(token('break'), 'Break')(...args);
}

function Case(...args) {
  return wrap(token('case'), 'Case')(...args);
}

function Char(...args) {
  return wrap(token('char'), 'Char')(...args);
}

function Const(...args) {
  return wrap(token('const'), 'Const')(...args);
}

function Continue(...args) {
  return wrap(token('continue'), 'Continue')(...args);
}

function Default(...args) {
  return wrap(token('default'), 'Default')(...args);
}

function Do(...args) {
  return wrap(token('do'), 'Do')(...args);
}

function Double(...args) {
  return wrap(token('double'), 'Double')(...args);
}

function Else(...args) {
  return wrap(token('else'), 'Else')(...args);
}

function Enum(...args) {
  return wrap(token('enum'), 'Enum')(...args);
}

function Extern(...args) {
  return wrap(token('extern'), 'Extern')(...args);
}

function Float(...args) {
  return wrap(token('float'), 'Float')(...args);
}

function For(...args) {
  return wrap(token('for'), 'For')(...args);
}

function Goto(...args) {
  return wrap(token('goto'), 'Goto')(...args);
}

function If(...args) {
  return wrap(token('if'), 'If')(...args);
}

function Inline(...args) {
  return wrap(token('inline'), 'Inline')(...args);
}

function Int(...args) {
  return wrap(token('int'), 'Int')(...args);
}

function Long(...args) {
  return wrap(token('long'), 'Long')(...args);
}

function Register(...args) {
  return wrap(token('register'), 'Register')(...args);
}

function Restrict(...args) {
  return wrap(token('restrict'), 'Restrict')(...args);
}

function Return(...args) {
  return wrap(token('return'), 'Return')(...args);
}

function Short(...args) {
  return wrap(token('short'), 'Short')(...args);
}

function Signed(...args) {
  return wrap(token('signed'), 'Signed')(...args);
}

function Sizeof(...args) {
  return wrap(token('sizeof'), 'Sizeof')(...args);
}

function Static(...args) {
  return wrap(token('static'), 'Static')(...args);
}

function Struct(...args) {
  return wrap(token('struct'), 'Struct')(...args);
}

function Switch(...args) {
  return wrap(token('switch'), 'Switch')(...args);
}

function Typedef(...args) {
  return wrap(token('typedef'), 'Typedef')(...args);
}

function Union(...args) {
  return wrap(token('union'), 'Union')(...args);
}

function Unsigned(...args) {
  return wrap(token('unsigned'), 'Unsigned')(...args);
}

function Void(...args) {
  return wrap(token('void'), 'Void')(...args);
}

function Volatile(...args) {
  return wrap(token('volatile'), 'Volatile')(...args);
}

function While(...args) {
  return wrap(token('while'), 'While')(...args);
}

function Alignas(...args) {
  return wrap(token('_Alignas'), 'Alignas')(...args);
}

function Alignof(...args) {
  return wrap(token('_Alignof'), 'Alignof')(...args);
}

function Atomic(...args) {
  return wrap(token('_Atomic'), 'Atomic')(...args);
}

function Bool(...args) {
  return wrap(token('_Bool'), 'Bool')(...args);
}

function Complex(...args) {
  return wrap(token('_Complex'), 'Complex')(...args);
}

function Generic(...args) {
  return wrap(token('_Generic'), 'Generic')(...args);
}

function Imaginary(...args) {
  return wrap(token('_Imaginary'), 'Imaginary')(...args);
}

function Noreturn(...args) {
  return wrap(token('_Noreturn'), 'Noreturn')(...args);
}

function StaticAssert(...args) {
  return wrap(token('_Static_assert'), 'StaticAssert')(...args);
}

function ThreadLocal(...args) {
  return wrap(token('_Thread_local'), 'ThreadLocal')(...args);
}

function LeftParen(...args) {
  return wrap(token('('), 'LeftParen')(...args);
}

function RightParen(...args) {
  return wrap(token(')'), 'RightParen')(...args);
}

function LeftBracket(...args) {
  return wrap(token('['), 'LeftBracket')(...args);
}

function RightBracket(...args) {
  return wrap(token(']'), 'RightBracket')(...args);
}

function LeftBrace(...args) {
  return wrap(token('{'), 'LeftBrace')(...args);
}

function RightBrace(...args) {
  return wrap(token('}'), 'RightBrace')(...args);
}

function Less(...args) {
  return wrap(token('<'), 'Less')(...args);
}

function LessEqual(...args) {
  return wrap(token('<='), 'LessEqual')(...args);
}

function Greater(...args) {
  return wrap(token('>'), 'Greater')(...args);
}

function GreaterEqual(...args) {
  return wrap(token('>='), 'GreaterEqual')(...args);
}

function LeftShift(...args) {
  return wrap(token('<<'), 'LeftShift')(...args);
}

function RightShift(...args) {
  return wrap(token('>>'), 'RightShift')(...args);
}

function Plus(...args) {
  return wrap(token('+'), 'Plus')(...args);
}

function PlusPlus(...args) {
  return wrap(token('++'), 'PlusPlus')(...args);
}

function Minus(...args) {
  return wrap(token('-'), 'Minus')(...args);
}

function MinusMinus(...args) {
  return wrap(token('--'), 'MinusMinus')(...args);
}

function Star(...args) {
  return wrap(token('*'), 'Star')(...args);
}

function Div(...args) {
  return wrap(token('/'), 'Div')(...args);
}

function Mod(...args) {
  return wrap(token('%'), 'Mod')(...args);
}

function And(...args) {
  return wrap(token('&'), 'And')(...args);
}

function Or(...args) {
  return wrap(token('|'), 'Or')(...args);
}

function AndAnd(...args) {
  return wrap(token('&&'), 'AndAnd')(...args);
}

function OrOr(...args) {
  return wrap(token('||'), 'OrOr')(...args);
}

function Caret(...args) {
  return wrap(token('^'), 'Caret')(...args);
}

function Not(...args) {
  return wrap(token('!'), 'Not')(...args);
}

function Tilde(...args) {
  return wrap(token('~'), 'Tilde')(...args);
}

function Question(...args) {
  return wrap(token('?'), 'Question')(...args);
}

function Colon(...args) {
  return wrap(token(':'), 'Colon')(...args);
}

function Semi(...args) {
  return wrap(token(';'), 'Semi')(...args);
}

function Comma(...args) {
  return wrap(token(','), 'Comma')(...args);
}

function Assign(...args) {
  return wrap(token('='), 'Assign')(...args);
}

function StarAssign(...args) {
  return wrap(token('*='), 'StarAssign')(...args);
}

function DivAssign(...args) {
  return wrap(token('/='), 'DivAssign')(...args);
}

function ModAssign(...args) {
  return wrap(token('%='), 'ModAssign')(...args);
}

function PlusAssign(...args) {
  return wrap(token('+='), 'PlusAssign')(...args);
}

function MinusAssign(...args) {
  return wrap(token('-='), 'MinusAssign')(...args);
}

function LeftShiftAssign(...args) {
  return wrap(token('<<='), 'LeftShiftAssign')(...args);
}

function RightShiftAssign(...args) {
  return wrap(token('>>='), 'RightShiftAssign')(...args);
}

function AndAssign(...args) {
  return wrap(token('&='), 'AndAssign')(...args);
}

function XorAssign(...args) {
  return wrap(token('^='), 'XorAssign')(...args);
}

function OrAssign(...args) {
  return wrap(token('|='), 'OrAssign')(...args);
}

function Equal(...args) {
  return wrap(token('=='), 'Equal')(...args);
}

function NotEqual(...args) {
  return wrap(token('!='), 'NotEqual')(...args);
}

function Arrow(...args) {
  return wrap(token('->'), 'Arrow')(...args);
}

function Dot(...args) {
  return wrap(token('.'), 'Dot')(...args);
}

function Ellipsis(...args) {
  return wrap(token('...'), 'Ellipsis')(...args);
}

function Identifier(...args) {
  return wrap(regex(/[a-zA-Z_][a-zA-Z0-9_]*/g), 'Identifier')(...args);
}

function IdentifierNondigit(...args) {
  return wrap(choice(Nondigit, UniversalCharacterName), 'IdentifierNondigit')(...args);
}

function Nondigit(...args) {
  return wrap(regex(/[a-zA-Z_]/g), 'Nondigit')(...args);
}

function Digit(...args) {
  return wrap(regex(/[0-9]/g), 'Digit')(...args);
}

function UniversalCharacterName(...args) {
  return wrap(choice(seq(token('\\u'), HexQuad), seq(token('\\U'), HexQuad, HexQuad)), 'UniversalCharacterName')(...args);
}

function HexQuad(...args) {
  return wrap(seq(HexadecimalDigit, HexadecimalDigit, HexadecimalDigit, HexadecimalDigit), 'HexQuad')(...args);
}

function Constant(...args) {
  return wrap(choice(IntegerConstant, FloatingConstant, CharacterConstant), 'Constant')(...args);
}

function IntegerConstant(...args) {
  return wrap(
    choice(seq(DecimalConstant, option(IntegerSuffix)), seq(OctalConstant, option(IntegerSuffix)), seq(HexadecimalConstant, option(IntegerSuffix)), BinaryConstant),
    'IntegerConstant'
  )(...args);
}

function BinaryConstant(...args) {
  return wrap(seq(token('0'), regex(/[bB]/g), regex(/[0-1]+/g)), 'BinaryConstant')(...args);
}

function DecimalConstant(...args) {
  return wrap(seq(NonzeroDigit, any(Digit)), 'DecimalConstant')(...args);
}

function OctalConstant(...args) {
  return wrap(seq(token('0'), any(OctalDigit)), 'OctalConstant')(...args);
}

function HexadecimalConstant(...args) {
  return wrap(seq(HexadecimalPrefix, many(HexadecimalDigit)), 'HexadecimalConstant')(...args);
}

function HexadecimalPrefix(...args) {
  return wrap(seq(token('0'), regex(/[xX]/g)), 'HexadecimalPrefix')(...args);
}

function NonzeroDigit(...args) {
  return wrap(regex(/[1-9]/g), 'NonzeroDigit')(...args);
}

function OctalDigit(...args) {
  return wrap(regex(/[0-7]/g), 'OctalDigit')(...args);
}

function HexadecimalDigit(...args) {
  return wrap(regex(/[0-9a-fA-F]/g), 'HexadecimalDigit')(...args);
}

function IntegerSuffix(...args) {
  return wrap(
    choice(seq(UnsignedSuffix, option(LongSuffix)), seq(UnsignedSuffix, LongLongSuffix), seq(LongSuffix, option(UnsignedSuffix)), seq(LongLongSuffix, option(UnsignedSuffix))),
    'IntegerSuffix'
  )(...args);
}

function UnsignedSuffix(...args) {
  return wrap(regex(/[uU]/g), 'UnsignedSuffix')(...args);
}

function LongSuffix(...args) {
  return wrap(regex(/[lL]/g), 'LongSuffix')(...args);
}

function LongLongSuffix(...args) {
  return wrap(choice(token('ll'), token('LL')), 'LongLongSuffix')(...args);
}

function FloatingConstant(...args) {
  return wrap(choice(DecimalFloatingConstant, HexadecimalFloatingConstant), 'FloatingConstant')(...args);
}

function DecimalFloatingConstant(...args) {
  return wrap(choice(seq(FractionalConstant, option(ExponentPart), option(FloatingSuffix)), seq(DigitSequence, ExponentPart, option(FloatingSuffix))), 'DecimalFloatingConstant')(...args);
}

function HexadecimalFloatingConstant(...args) {
  return wrap(
    choice(
      seq(HexadecimalPrefix, HexadecimalFractionalConstant, BinaryExponentPart, option(FloatingSuffix)),
      seq(HexadecimalPrefix, HexadecimalDigitSequence, BinaryExponentPart, option(FloatingSuffix))
    ),
    'HexadecimalFloatingConstant'
  )(...args);
}

function FractionalConstant(...args) {
  return wrap(choice(seq(option(DigitSequence), token('.'), DigitSequence), seq(DigitSequence, token('.'))), 'FractionalConstant')(...args);
}

function ExponentPart(...args) {
  return wrap(choice(seq(token('e'), option(Sign), DigitSequence), seq(token('E'), option(Sign), DigitSequence)), 'ExponentPart')(...args);
}

function Sign(...args) {
  return wrap(choice(token('+'), token('-')), 'Sign')(...args);
}

function DigitSequence(...args) {
  return wrap(many(Digit), 'DigitSequence')(...args);
}

function HexadecimalFractionalConstant(...args) {
  return wrap(choice(seq(option(HexadecimalDigitSequence), token('.'), HexadecimalDigitSequence), seq(HexadecimalDigitSequence, token('.'))), 'HexadecimalFractionalConstant')(...args);
}

function BinaryExponentPart(...args) {
  return wrap(choice(seq(token('p'), option(Sign), DigitSequence), seq(token('P'), option(Sign), DigitSequence)), 'BinaryExponentPart')(...args);
}

function HexadecimalDigitSequence(...args) {
  return wrap(many(HexadecimalDigit), 'HexadecimalDigitSequence')(...args);
}

function FloatingSuffix(...args) {
  return wrap(choice(token('f'), token('l'), token('F'), token('L')), 'FloatingSuffix')(...args);
}

function CharacterConstant(...args) {
  return wrap(
    choice(seq(token("'"), CCharSequence, token("'")), seq(token("L'"), CCharSequence, token("'")), seq(token("u'"), CCharSequence, token("'")), seq(token("U'"), CCharSequence, token("'"))),
    'CharacterConstant'
  )(...args);
}

function CCharSequence(...args) {
  return wrap(many(CChar), 'CCharSequence')(...args);
}

function CChar(...args) {
  return wrap(choice(invert(regex(/['\\\r\n]/g)), EscapeSequence), 'CChar')(...args);
}

function EscapeSequence(...args) {
  return wrap(choice(SimpleEscapeSequence, OctalEscapeSequence, HexadecimalEscapeSequence, UniversalCharacterName), 'EscapeSequence')(...args);
}

function SimpleEscapeSequence(...args) {
  return wrap(seq(token('\\'), regex(/['"?abfnrtv\\]/g)), 'SimpleEscapeSequence')(...args);
}

function OctalEscapeSequencef(...args) {
  return wrap(choice(seq(token('\\'), OctalDigit), seq(token('\\'), OctalDigit, OctalDigit), seq(token('\\'), OctalDigit, OctalDigit, OctalDigit)), 'OctalEscapeSequencef')(...args);
}

function HexadecimalEscapeSequence(...args) {
  return wrap(seq(token('\\x'), many(HexadecimalDigit)), 'HexadecimalEscapeSequence')(...args);
}

function StringLiteral(...args) {
  return wrap(seq(option(EncodingPrefix), token('"'), option(SCharSequence), token('"')), 'StringLiteral')(...args);
}

function EncodingPrefix(...args) {
  return wrap(choice(token('u8'), token('u'), token('U'), token('L')), 'EncodingPrefix')(...args);
}

function SCharSequence(...args) {
  return wrap(many(SChar), 'SCharSequence')(...args);
}

function SChar(...args) {
  return wrap(choice(invert(regex(/["\\\r\n]/g)), EscapeSequence, token('\\\n'), token('\\\r\n')), 'SChar')(...args);
}

function ComplexDefine(...args) {
  return wrap(ignore(seq(token('#'), option(Whitespace), token('define'), invert(regex(/[#]*/g)))), 'ComplexDefine')(...args);
}

function IncludeDirective(...args) {
  return wrap(
    ignore(
      seq(
        token('#'),
        option(Whitespace),
        token('include'),
        option(Whitespace),
        choice(seq(token('"'), invert(regex(/[\r\n]*/g)), token('"')), seq(token('<'), invert(regex(/[\r\n]*/g)), token('>'))),
        option(Whitespace),
        Newline
      )
    ),
    'IncludeDirective'
  )(...args);
}

function AsmBlock(...args) {
  return wrap(ignore(seq(token('asm'), invert(any(token('{'))), token('{'), invert(any(token('}'))), token('}'))), 'AsmBlock')(...args);
}

function LineAfterPreprocessing(...args) {
  return wrap(ignore(seq(token('#line'), any(Whitespace), invert(regex(/[\r\n]*/g)))), 'LineAfterPreprocessing')(...args);
}

function LineDirective(...args) {
  return wrap(ignore(seq(token('#'), option(Whitespace), DecimalConstant, option(Whitespace), StringLiteral, invert(regex(/[\r\n]*/g)))), 'LineDirective')(...args);
}

function PragmaDirective(...args) {
  return wrap(ignore(seq(token('#'), option(Whitespace), token('pragma'), Whitespace, invert(regex(/[\r\n]*/g)))), 'PragmaDirective')(...args);
}

function Whitespace(...args) {
  return wrap(ignore(regex(/[ \t]+/g)), 'Whitespace')(...args);
}

function Newline(...args) {
  return wrap(ignore(choice(seq(char('\r'), option(char('\n'))), char('\n'))), 'Newline')(...args);
}

function BlockComment(...args) {
  return wrap(ignore(seq(token('/*'), regex(/.*?/g), token('*/'))), 'BlockComment')(...args);
}

function LineComment(...args) {
  return wrap(ignore(seq(token('//'), invert(regex(/[\r\n]*/g)))), 'LineComment')(...args);
}

export default {
  primaryExpression,
  genericSelection,
  genericAssocList,
  genericAssociation,
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
  expression,
  constantExpression,
  declaration,
  declarationSpecifiers,
  declarationSpecifiers2,
  declarationSpecifier,
  initDeclaratorList,
  initDeclarator,
  storageClassSpecifier,
  typeSpecifier,
  structOrUnionSpecifier,
  structOrUnion,
  structDeclarationList,
  structDeclaration,
  specifierQualifierList,
  structDeclaratorList,
  structDeclarator,
  enumSpecifier,
  enumeratorList,
  enumerator,
  enumerationConstant,
  atomicTypeSpecifier,
  typeQualifier,
  functionSpecifier,
  alignmentSpecifier,
  declarator,
  directDeclarator,
  gccDeclaratorExtension,
  gccAttributeSpecifier,
  gccAttributeList,
  gccAttribute,
  nestedParenthesesBlock,
  pointer,
  typeQualifierList,
  parameterTypeList,
  parameterList,
  parameterDeclaration,
  identifierList,
  typeName,
  abstractDeclarator,
  directAbstractDeclarator,
  typedefName,
  initializer,
  initializerList,
  designation,
  designatorList,
  designator,
  staticAssertDeclaration,
  statement,
  labeledStatement,
  compoundStatement,
  blockItemList,
  blockItem,
  expressionStatement,
  selectionStatement,
  iterationStatement,
  forCondition,
  forDeclaration,
  forExpression,
  jumpStatement,
  compilationUnit,
  translationUnit,
  externalDeclaration,
  functionDefinition,
  declarationList,
  Auto,
  Break,
  Case,
  Char,
  Const,
  Continue,
  Default,
  Do,
  Double,
  Else,
  Enum,
  Extern,
  Float,
  For,
  Goto,
  If,
  Inline,
  Int,
  Long,
  Register,
  Restrict,
  Return,
  Short,
  Signed,
  Sizeof,
  Static,
  Struct,
  Switch,
  Typedef,
  Union,
  Unsigned,
  Void,
  Volatile,
  While,
  Alignas,
  Alignof,
  Atomic,
  Bool,
  Complex,
  Generic,
  Imaginary,
  Noreturn,
  StaticAssert,
  ThreadLocal,
  LeftParen,
  RightParen,
  LeftBracket,
  RightBracket,
  LeftBrace,
  RightBrace,
  Less,
  LessEqual,
  Greater,
  GreaterEqual,
  LeftShift,
  RightShift,
  Plus,
  PlusPlus,
  Minus,
  MinusMinus,
  Star,
  Div,
  Mod,
  And,
  Or,
  AndAnd,
  OrOr,
  Caret,
  Not,
  Tilde,
  Question,
  Colon,
  Semi,
  Comma,
  Assign,
  StarAssign,
  DivAssign,
  ModAssign,
  PlusAssign,
  MinusAssign,
  LeftShiftAssign,
  RightShiftAssign,
  AndAssign,
  XorAssign,
  OrAssign,
  Equal,
  NotEqual,
  Arrow,
  Dot,
  Ellipsis,
  Identifier,
  IdentifierNondigit,
  Nondigit,
  Digit,
  UniversalCharacterName,
  HexQuad,
  Constant,
  IntegerConstant,
  BinaryConstant,
  DecimalConstant,
  OctalConstant,
  HexadecimalConstant,
  HexadecimalPrefix,
  NonzeroDigit,
  OctalDigit,
  HexadecimalDigit,
  IntegerSuffix,
  UnsignedSuffix,
  LongSuffix,
  LongLongSuffix,
  FloatingConstant,
  DecimalFloatingConstant,
  HexadecimalFloatingConstant,
  FractionalConstant,
  ExponentPart,
  Sign,
  DigitSequence,
  HexadecimalFractionalConstant,
  BinaryExponentPart,
  HexadecimalDigitSequence,
  FloatingSuffix,
  CharacterConstant,
  CCharSequence,
  CChar,
  EscapeSequence,
  SimpleEscapeSequence,
  OctalEscapeSequencef,
  HexadecimalEscapeSequence,
  StringLiteral,
  EncodingPrefix,
  SCharSequence,
  SChar,
  ComplexDefine,
  IncludeDirective,
  AsmBlock,
  LineAfterPreprocessing,
  LineDirective,
  PragmaDirective,
  Whitespace,
  Newline,
  BlockComment,
  LineComment
};