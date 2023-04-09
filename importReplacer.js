import { memoize, unique, filter } from './lib/misc.js';
import { ReadJSON } from './io-helpers.js';

export const importReplacer = {
  replacementMap: memoize(() =>
    Object.entries(ReadJSON('package.json')._moduleAliases)
      .filter(([k, v]) => !/[.\/]/.test(v))
      .map(a => a.reverse())
  ),
  regexp() {
    return new RegExp("\\bfrom\\s*'(\\" + unique(this.replacementMap().map(([k, v]) => k)).join('|') + ")';", 'gm');
  },
  replacerFunc() {
    let m = Object.fromEntries(this.replacementMap());

    return (w, s, i) => `from '${m[s]}';`;
  },
  replace(s) {
    return s.replaceAll(this.regexp(), this.replacerFunc());
  }
};

export default importReplacer;
