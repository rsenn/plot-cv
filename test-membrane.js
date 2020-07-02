import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import util from 'util';
import fs from 'fs';
import { Path, MutablePath, ImmutablePath, XPath, MutableXPath, ImmutableXPath, XmlObject, PathMapper, toXML, TreeObserver, findXPath, XMLIterator, XmlIterator } from './lib/json.js';
import { Console } from 'console';

const inspect = (arg, depth = 10, colors = true, breakLength = Number.Infinity) => util.inspect(arg, { depth, breakLength, compact: true, showProxy: true, colors });

const printNode = node => {
  let s = toXML(node).replace(/\n.*/g, '');
  return s;
};
// prettier-ignore
Util.toString.defaultOpts = {
  spacing: '',
  separator: '\x1b[1;36m, ',
  stringColor: [1, 36],
  colon: '܃',
  padding: ' ',
  quote: '',
  depth: 1
};

const CH = 'children';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: {
    depth: 2,
    colors: true,
    breakLength: 200,
    compact: true,
    showProxy: true,
    customInspect: true
  }
});

Error.stackTraceLimit = 100;
try {
  const mapper = new PathMapper();
  let observer = new TreeObserver(mapper, false);
  function main(...args) {
    let str = fs.readFileSync(args.length ? args[0] : '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

    let xml = tXml(str)[0];

    const obj2path = Util.weakMapper((v, p) => new ImmutablePath(p, true));
    const path2xpath = path => {
      //console.log('path:', path.xpath(xml));
      return new ImmutablePath(path, true).xpath(xml);
    }; // Util.weakMapper((v, p) => obj2xpath(v).xpath(xml));

    let path2obj = deep.flatten(
      xml,
      new Map(),
      (v, p, r) => typeof v == 'object' && v !== null && v.tagName !== undefined,
      (p, v) => obj2path(v, p) + '',
      ({ tagName, attributes, children, ...value }) => (children.length ? [tagName, attributes, children] : [tagName, attributes])
    );

    let flat = deep.flatten(
      xml,
      new Map(),
      (v, p, r) => typeof v == 'object' && v !== null && v.tagName !== undefined,
      (p, v) => obj2path(v, p),
      (v, p) => v
    );
    let rel,
      prev = new ImmutablePath([], true),
      prevParent,
      relTo = [];

    console.log('prev:', prev);

    const mk = ([path, value]) => {
      let p = new ImmutablePath(path);
      path = p.toArray();
      //console.log('mk=', { path, value });
      let thisParent = p.parent;
      if(thisParent.equal(prev)) path = p.relativeTo(thisParent);
      if(prev.equal(thisParent)) {
        relTo = prev;
        path = p.relativeTo(relTo);
      } else if(thisParent.equal(prevParent)) {
        relTo = prevParent;
        path = p.relativeTo(relTo);
      } else {
        prev = p.makeAbsolute(relTo);
        relTo = [];
      }
      if(prevParent && !prevParent.equal(thisParent)) {
        relTo = [];
      }
      prevParent = thisParent;
      return path;
    };
    //console.log('drawing:', findXPath('//drawing', flat, { root: xml, entries: true, recursive: false }).map(mk));

    let p = observer.get(Object.fromEntries(flat.entries()));
    let node = p;
    let unwrapped, type, path;

    observer.subscribe((what, target, path, value) => {
      if(what == 'access') return;
      //S  let target = {type: observer.getType(target), keys: Object.keys(target).join(',') };
      let valueType = typeof value;
      let xpath = path.xpath(xml).slice(-2) + '';
      path = path.slice(-2) + '';
      let string = typeof value == 'string' ? value : '';
      console.log('handler', what, path, /*target,*/ /*string,*/ value);
    });
    mapper.set(observer.unwrap(node), []);
    let tree = observer.get(xml);

    const incr = (obj, prop, i = 1) => {
      if(!obj) obj = {};
      return { ...obj, [prop]: (obj[prop] || 0) + i };
    };

    let tags = {};
    let iter = new XMLIterator({ children: xml.children, tagName: tree.tagName, attributes: tree.attributes }, (v, p) => true);
    console.log('iter:', iter);
    for(let [v, p] of iter) {
      if(!(p instanceof ImmutablePath)) p = new ImmutablePath(p, true);

      console.log('p:', p);
      tags = incr(tags, v.tagName);
    }
    let lists = [];
    for(let key in tags) {
      let lkey = key == 'library' ? 'libraries' : key + 's';
      if(tags[lkey] !== undefined) lists.push(lkey);
    }

    console.log('lists:', lists);

    // tags = tags.filter(([k,v]) => console.log(k,v));

    tags = Object.entries(tags).sort((a, b) => a[1] - b[1]);
    //console.log('lists', lists);

    tags = lists
      //  .filter(([k, v]) => v == 1)
      .map(t => {
        let { path, value } = deep.find(xml, (v, p) => v.tagName == t);
        let selected = deep.select(xml, (v, p) => v.tagName == t);
        let xpath = new ImmutablePath(path, true).xpath(xml);
        xpath = xpath.slice(-2);
        let q = xpath.toRegExp();

        path = obj2path(value);
        let dumps = selected
          .map(({ path, value }) => [new ImmutablePath(path, true), value])
          .map(([p, v]) => [p.xpath(xml), v])
          .map(([p, v]) => [p, v, p.offset((o, i, p) => !(/(\[|board$|sheets$)/.test(o) || p[i + 1] == 'attributes'))])
          .map(([p, v, o]) => [p.slice(o - 2), p.slice(0, o - 2), v, o])
          .map(([p, s, v, o]) => [p, s, `children: ${v.children.length}`, `offset: ${o}`])
          .map(([p, s, v, o]) => [p, s, v, p.toRegExp()]);
        console.log(
          'r:',
          dumps
            .map(([p, s, v, r]) => [p, s, v, r.test(p), r.test(s), [...p.toString().match(q)], [...q.exec(p)].slice(1)])
            .map(([p, s, ...rest]) => [p[Symbol.for('nodejs.util.inspect.custom')](), s[Symbol.for('nodejs.util.inspect.custom')](), ...rest.map(i => Util.toSource(i))])
            .map(([p]) => p + '')
            .join('\n  |')
        );
        //.map((a, i) => '\n' + i + ':\n  ' + a.join('\n  ')) .join('\n') )
        return [xpath, new Map(selected.map(({ path, value }) => [path2xpath(path).down('*'), value]))];
      });
    tags = Object.fromEntries(tags);
    for(let [search, value] of Object.entries(tags)) console.log('tags', search, inspect(value, 1, true, 80));

    let x = new ImmutableXPath('/eagle/drawing/board');
    /*
  let o = x.apply(xml, true);
      console.log('o:', o);


  o*/

    let drawing = deep.find(xml, v => v.tagName == 'drawing');
    let board = deep.find(xml, v => v.tagName == 'board');
    let w = new ImmutablePath(board.path, false);
    console.log('w:', w + '');
    let y = w.xpath(xml);
    console.log('y:', y);
    // console.log('path2obj.keys:', [...path2obj.keys()]);

    let z = w.apply(xml); // path2obj.get(w+'');
    let u = observer.get(z); // path2obj.get(w+'');
    console.log('z:', z);
    console.log('u:', u);
    console.log('observer.getType(u):', observer.getType(u));

    u.attributes['name'] = 'test';
  }

  main(...process.argv.slice(2));
} catch(err) {
  //console.log('err:', err);
}
