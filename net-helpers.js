import { fetch } from 'fetch';
import { define, isObject, isString, types } from 'util';
import { Parser } from 'dom';

export async function httpGet(url, opts = {}) {
  const resp = await fetch(url, { tls: { key: 'localhost.key', cert: 'localhost.crt', /*ca: 'ca.crt',*/ rejectUnauthorized: false }, h2: false, ...opts });

  let r = await resp.text();

  if(opts.parse) return new Parser().parseFromString(r);

  return r;
}

export function getNodeDepth(elem) {
  let i = 0;
  while(elem.parentElement) {
    elem = elem.parentElement;
    i++;
  }
  return i;
}

export function getTextNodes(elem) {
  return [...elem.querySelectorAll('*')].filter(e => e.nodeType == e.TEXT_NODE);
}

export function textPredicate(pred) {
  const re = isString(pred) ? new RegExp(pred, 'gi') : types.isRegExp(pred) ? pred : null;

  return text => re.test(text);
}

export function findText(doc, text) {
  let it = getTextNodes(doc.body);
  let predicate = textPredicate(text);

  for(let node of it) if(node.nodeType == node.TEXT_NODE) if (predicate(node.innerText)) return node;
}

export function getTextLines(text) {
  if(isObject(text) && text.nodeType) text = text.innerText;

  return [...text.matchAll(/[A-Z][^.]+[^\s][\.\!\?]\s*/g)].map(m => m[0]);
}

export function* iterate(node, pred) {
  if(!pred || pred(node)) yield node;

  if(types.isIterable(node.children))
    for(let child of node.children) {
      if(/(script|style)/gi.test(child.tagName)) continue;
      yield* iterate(child, pred);
    }
}

export function searchNode(root, pred) {
  for(let node of iterate(root, pred)) return node;
}

export function getText(doc) {
  return [...doc.querySelectorAll('*')].filter(e => e.nodeType == e.ELEMENT_NODE).filter(e => !/(script|style|html|head|meta|title|body|link)/i.test(e.tagName));
  //.filter(e => !/<(script|style)/i.test(e.innerHTML))
  //.filter(e => [...e.childNodes].some(e => e.nodeType == e.TEXT_NODE));
}

export async function* searchGoogle(query, opts = {}) {
  let ok, text;
  let { start = 0, step = 20 } = opts;
  const parser = new Parser();

  do {
    const text = await httpGet(`http://www.google.com/search?q=${encodeURIComponent(query)}&num=${step}&start=${start}`);

    if((ok = text)) {
      const doc = (globalThis.doc = parser.parseFromString(text));

      for(let elem of doc.querySelectorAll('h3')) {
        while(elem.tagName != 'a') elem = elem.parentElement;

        yield decodeURIComponent(elem.getAttribute('href').replace(/.*\bq=([^&]+)&.*/g, '$1'));
      }
    }

    start += step;
  } while(ok);
}