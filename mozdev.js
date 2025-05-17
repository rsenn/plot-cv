import { Parser, URL } from 'dom';
import { urlGet } from 'std';
import { types, define, properties, nonenumerable } from 'util';

export function FetchClass(url) {
  const u = new URL(url.replace(/\/[^\/]*$/, ''));
  const base = u.pathname;
  const re = new RegExp('^' + base + '/');
  const doc = new Parser().parseFromString(urlGet(url));

  const keys = {
    Constructor: 'constructor',
    'Instance properties': 'properties',
    'Instance methods': 'methods',
    Events: 'events',
    Inheritance: 'bases',
    'Related pages for Web Audio API': 'related',
  };

  const summaryElements = [...doc.querySelectorAll('summary')].filter(e => e.innerText in keys);

  const simplifyList = list => {
    if(list.every(([a, b]) => a.replace(/\sExperimental$/, '') == b)) return list.map(([a]) => a);

    return new Map(list);
  };

  const summaryList = summary =>
    simplifyList([...summary.nextSibling.querySelectorAll('li')].map(e => [e.innerText, e.querySelector('a').getAttribute('href').replace(re, '')]).filter(([name]) => !/\sDeprecated$/.test(name)));

  const summaries = summaryElements.reduce((a, e) => ({ ...a, [keys[e.innerText] ?? e.innerText]: summaryList(e) }), Object.setPrototypeOf({}, null));

  const makeURL = p => u + '/' + p;

  return define(
    summaries,
    nonenumerable({
      getURL(name) {
        const re = new RegExp('^' + name + '(\(\)|)(\s.*|)' + '$');

        for(let key of this.keys()) {
          if(key == name) return [...this[key]].map(entry => (Array.isArray(entry) ? entry[1] : entry)).map(makeURL);

          for(let entry of this[key]) {
            if(typeof entry == 'string' && re.test(entry)) return makeURL(entry);

            const [prop, pathname] = entry;

            if(re.test(prop)) return makeURL(pathname);
          }
        }
      },
      *keys() {
        for(let id in keys) yield keys[id];
      },
      *values() {
        for(let id in keys) yield this[keys[id]];
      },
      *entries() {
        for(let id in keys) yield [keys[id], this[keys[id]]];
      },
      *getMembers(pred = (k, v) => true) {
        if(Array.isArray(pred)) {
          const arr = pred;
          pred = (key, value) => arr.indexOf(key) != -1;
        }
        for(let [key, value] of this.entries()) if(types.isMap(value)) if (pred(key, value)) yield* value.entries();
      },
      get name() {
        return this.getMembers(['constructor']).next()?.value?.[0]?.replace(/\(\)$/, '');
      },
    }),
  );
}
