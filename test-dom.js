import { ClassSelector, IdSelector, parseSelectors, PseudoClassSelector, TypeSelector } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { clone, delegate, equals, extend, find, flatten, forEach, get, iterate, MAXDEPTH_MASK, NO_THROW, PATH_AS_STRING, RETURN_MASK, RETURN_PATH, RETURN_PATH_VALUE, RETURN_VALUE, RETURN_VALUE_PATH, select, set, transform, unflatten, unset } from './quickjs/qjs-modules/lib/deep.js';
import { Attr, Comment, Document, Element, Entities, Factory, GetType, Interface, NamedNodeMap, Node, NodeList, nodeTypes, Parser, Prototypes, Text, TokenList } from './quickjs/qjs-modules/lib/dom.js';
import { read, write } from './quickjs/qjs-modules/lib/xml.js';
import { default as xpath, getSiblings, ImmutableXPath, parseXPath } from './quickjs/qjs-modules/lib/xpath.js';

const dom = {
  Entities,
  nodeTypes,
  Prototypes,
  Factory,
  Parser,
  Interface,
  Node,
  NodeList,
  NamedNodeMap,
  Element,
  Document,
  Attr,
  Text,
  Comment,
  TokenList,
  GetType
};

const css = { TypeSelector, ClassSelector, PseudoClassSelector, IdSelector, parseSelectors };

Object.assign(globalThis, {
  dom,
  css,
  xml: { read, write },
  xpath: { ImmutableXPath, parseXPath, getSiblings, xpath },
  deep: {
    RETURN_VALUE_PATH,
    RETURN_PATH,
    RETURN_VALUE,
    RETURN_PATH_VALUE,
    RETURN_MASK,
    PATH_AS_STRING,
    NO_THROW,
    MAXDEPTH_MASK,
    clone,
    equals,
    extend,
    select,
    find,
    forEach,
    iterate,
    flatten,
    get,
    set,
    delegate,
    transform,
    unset,
    unflatten
  }
});

/*(name =>
  fetch(name)
    .then(r => r.text())
    .then(r => {
      let p = new dom.Parser();
      globalThis.doc = p.parseFromString(r);
      console.log(`Loaded '${name}':`, dom.Node.raw(globalThis.doc));
    }))('data/LeoStick.brd');
*/