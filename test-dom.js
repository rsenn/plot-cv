import { ClassSelector } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { IdSelector } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { parseSelectors } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { PseudoClassSelector } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { TypeSelector } from './quickjs/qjs-modules/lib/css3-selectors.js';
import { clone } from './quickjs/qjs-modules/lib/deep.js';
import { delegate } from './quickjs/qjs-modules/lib/deep.js';
import { equals } from './quickjs/qjs-modules/lib/deep.js';
import { extend } from './quickjs/qjs-modules/lib/deep.js';
import { find } from './quickjs/qjs-modules/lib/deep.js';
import { flatten } from './quickjs/qjs-modules/lib/deep.js';
import { forEach } from './quickjs/qjs-modules/lib/deep.js';
import { get } from './quickjs/qjs-modules/lib/deep.js';
import { iterate } from './quickjs/qjs-modules/lib/deep.js';
import { MAXDEPTH_MASK } from './quickjs/qjs-modules/lib/deep.js';
import { NO_THROW } from './quickjs/qjs-modules/lib/deep.js';
import { PATH_AS_STRING } from './quickjs/qjs-modules/lib/deep.js';
import { RETURN_MASK } from './quickjs/qjs-modules/lib/deep.js';
import { RETURN_PATH } from './quickjs/qjs-modules/lib/deep.js';
import { RETURN_PATH_VALUE } from './quickjs/qjs-modules/lib/deep.js';
import { RETURN_VALUE } from './quickjs/qjs-modules/lib/deep.js';
import { RETURN_VALUE_PATH } from './quickjs/qjs-modules/lib/deep.js';
import { select } from './quickjs/qjs-modules/lib/deep.js';
import { set } from './quickjs/qjs-modules/lib/deep.js';
import { transform } from './quickjs/qjs-modules/lib/deep.js';
import { unflatten } from './quickjs/qjs-modules/lib/deep.js';
import { unset } from './quickjs/qjs-modules/lib/deep.js';
import { Attr } from './quickjs/qjs-modules/lib/dom.js';
import { Comment } from './quickjs/qjs-modules/lib/dom.js';
import { Document } from './quickjs/qjs-modules/lib/dom.js';
import { Element } from './quickjs/qjs-modules/lib/dom.js';
import { Entities } from './quickjs/qjs-modules/lib/dom.js';
import { Factory } from './quickjs/qjs-modules/lib/dom.js';
import { GetType } from './quickjs/qjs-modules/lib/dom.js';
import { Interface } from './quickjs/qjs-modules/lib/dom.js';
import { NamedNodeMap } from './quickjs/qjs-modules/lib/dom.js';
import { Node } from './quickjs/qjs-modules/lib/dom.js';
import { NodeList } from './quickjs/qjs-modules/lib/dom.js';
import { nodeTypes } from './quickjs/qjs-modules/lib/dom.js';
import { Parser } from './quickjs/qjs-modules/lib/dom.js';
import { Prototypes } from './quickjs/qjs-modules/lib/dom.js';
import { Text } from './quickjs/qjs-modules/lib/dom.js';
import { TokenList } from './quickjs/qjs-modules/lib/dom.js';
import { read } from './quickjs/qjs-modules/lib/xml.js';
import { write } from './quickjs/qjs-modules/lib/xml.js';
import { default as xpath } from './quickjs/qjs-modules/lib/xpath.js';
import { getSiblings } from './quickjs/qjs-modules/lib/xpath.js';
import { ImmutableXPath } from './quickjs/qjs-modules/lib/xpath.js';
import { parseXPath } from './quickjs/qjs-modules/lib/xpath.js';

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