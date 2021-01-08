import { fetch } from 'net';
import Util from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';
import { tXml, XPath } from './lib/xml.js';
import * as deep from './lib/deep.js';

async function Search(q, languages) {
  let uri = Util.makeURL({
    protocol: 'https',
    host: 'libraries.io',
    location: '/search',
    query: { languages, q: 'parser' }
  });
  console.log('uri:', uri);
  const res = await fetch(uri);

  const { ok, status, type } = res;
  console.log('res:', { ok, status, type });

  const html = await res.text();
  console.log('html:', html);

  const [doc] = tXml(html);
  console.log('doc:', doc);

  const flat = deep.flatten(
    doc,
    new Map(),
    (v, p) => typeof v == 'object' && v != null && 'tagName' in v,
    (p, v) => [XPath.from(p, doc), v]
  );
  console.log('flat:', flat);
  return doc;
}

async function main(...args) {
  await ConsoleSetup({
    breakLength: 120,
    maxStringLength: 200,
    maxArrayLength: 20,
    multiline: 1,
    alignMap: true
  });
  if (args.length == 0) args = ['parser'];
  let languages;
  const add = (arr, ...items) => [...(arr ? arr : []), ...items];

  while (/^-/.test(args[0])) {
    let opt = args.shift();

    switch (opt) {
      case '-l': {
        languages = add(languages, ...args.shift().split(','));
        break;
      }
    }
  }

  for (let arg of args) {
    Search(arg, languages);
  }
}

Util.callMain(main, true);
