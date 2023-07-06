import { memoize, unique, filter } from './lib/misc.js';
import { ReadJSON } from './io-helpers.js';
import * as path from 'path';

export const importReplacer = {
  replacementMap: memoize(() =>
    Object.entries(ReadJSON('package.json')._moduleAliases)
      .filter(([k, v]) => !/[.\/]/.test(v))
      .map(a => a.reverse())
  ),
  regexp() {
    return new RegExp("\\bfrom\\s*'(\\" + unique(this.replacementMap().map(([k, v]) => k)).join('|') + ")';", 'gm');
  },
  replacerFunc(from) {
    let dir = path.dirname(from);
    let rel = path.relative(dir, '.', '.');
    let rm = this.replacementMap();
    let rrm = rm.map(([k, v]) => [k, path.normalize(path.join(rel, v))]);
    let m = Object.fromEntries(rrm);
    return (w, s, i) => `from '${m[s]}';`;
  },
  replace(s, from) {
    return s.replaceAll(this.regexp(), this.replacerFunc(from));
  }
};

export default importReplacer;
