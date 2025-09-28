#!/usr/bin/env qjsm
import { ReadFd, ReadFile } from './io-helpers.js';
import { Console } from 'console';
import { read as readXML } from 'xml';

function* injectSeparator(iter, sep = ', ', pad = ' ') {
  let i = 0;
  for(let item of iter) {
    yield i++ == 0 ? pad : sep;
    yield item;
  }
  if(i) yield pad;
}

function* iterateKV(obj) {
  for(let key in obj) yield [key, obj[key]];
}

function* mapIterator(iter, fn = (elem, i, iter) => undefined) {
  let i = 0;
  for(let elem of iter) yield fn(elem, i++, iter);
}

function* xml2h(xml, depth = 0) {
  const { tagName, attributes, children } = xml;
  let sep = ',\n' + '  '.repeat(depth);
  let nl = '\n' + '  '.repeat(depth);
  let separate = it => injectSeparator(it, sep, nl);

  if(typeof tagName == 'symbol') yield `h(${tagName.description}, {`;
  else yield `h('${tagName}', {`;

  let i = 0,
    j = 0;

  yield* separate(mapIterator(iterateKV(attributes), ([k, v]) => `${k}: '${v}'`));

  yield `}`;
  //for(let name in attributes) yield (i++ == 0 ? ' ' : ', ') + `${name}: '${attributes[name]}'`;

  if(children && Array.isArray(children) && children.length > 0) {
    yield `, [`;

    //yield* injectSeparator(mapIterator(children, child => typeof child == 'string' ? yield `'${child}'` : yield* xml2h(child))
    for(let child of children) {
      yield j++ == 0 ? nl : sep;

      if(typeof child == 'string') yield `'${child}'`;
      else yield* xml2h(child, depth + 1);
    }
    //  yield '\n'+'  '.repeat(Math.max(0, depth-1));
    // yield nl;

    yield `]`;
  }
  // yield '\n'+'  '.repeat(Math.max(0, depth-1));
  yield `)`;
}

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    maxStringLength: 100,
    depth: 2,
  });

  if(args.length == 0) args = ['/dev/stdin'];

  for(let arg of args) {
    console.log('arg', arg);

    let data = arg == '/dev/stdin' ? ReadFd(0) : ReadFile(arg);

    console.log('data', { data });

    let xml = readXML(data);

    console.log('xml', xml);

    if(Array.isArray(xml)) {
      if(xml.length == 1) xml = xml[0];
      else if(xml.length > 1) xml = { tagName: Symbol.for('Fragment'), attributes: {}, children: xml };
    }

    let str = '';
    for(let frag of xml2h(xml)) {
      str += frag;
    }
    console.log('str', str);
  }
}

main(...scriptArgs.slice(1));
