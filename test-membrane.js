import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import util from 'util';
import fs from 'fs';
import { Path, XPath, MutablePath, MutableXPath, XmlObject, PathMapper, toXML, TreeObserver, findXPath, XmlIterator } from './lib/json.js';
import { Console } from 'console';

const inspect = (arg, depth = 10) => util.inspect(arg, { depth, breakLength: 80, compact: true, showProxy: true, color: true });

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
    depth: 0,
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
  let treeObserve = new TreeObserver(mapper, false);
  function main(...args) {
    let str = fs.readFileSync(args.length ? args[0] : '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

    let xml = tXml(str)[0];

    const obj2path = Util.weakMapper((v, p) => new Path(p, true));
    const path2xpath = path => {
      //console.log('path:', path.xpath(xml));
      return new Path(path, true).xpath(xml);
    }; // Util.weakMapper((v, p) => obj2xpath(v).xpath(xml));

    let path2obj = deep.flatten(
      xml,
      new Map(),
      (v, p, r) => typeof v == 'object' && v !== null && v.tagName !== undefined,
      (p, v) => obj2path(v, p),
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
      prev = new Path([], true),
      prevParent,
      relTo = [];
    const mk = ([path, value]) => {
      let p = new Path(path);
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

    let p = treeObserve.get(Object.fromEntries(flat.entries()));
    let node = p;
    let unwrapped, type, path;
    treeObserve.subscribe((what, target, path, value) => {
      if(what == 'access') return;
      let targetType = treeObserve.getType(target);
      let targetKeys = Object.keys(target).join(',');
      let valueType = typeof value;
      let xpath = path.xpath(xml).slice(-2) + '';
      path = path.slice(-2) + '';
      let string = typeof value == 'string' ? value : '';
    });
    mapper.set(treeObserve.unwrap(node), []);
    let tree = treeObserve.get(xml);

    const incr = (obj, prop, i = 1) => {
      if(!obj) obj = {};
      return { ...obj, [prop]: (obj[prop] || 0) + i };
    };

    let tags = {};
    for(let [v, p] of XmlIterator({ children: xml.children, tagName: tree.tagName, attributes: tree.attributes }, (v, p) => true)) {
      if(!(p instanceof Path)) p = new Path(p, true);
      tags = incr(tags, v.tagName);
    }
    let lists = [];
    for(let key in tags) {
      let lkey = key == 'library' ? 'libraries' : key + 's';
      if(tags[lkey] !== undefined) lists.push(lkey);
    }

    // tags = tags.filter(([k,v]) => console.log(k,v));

    tags = Object.entries(tags).sort((a, b) => a[1] - b[1]);
    console.log('lists', lists);

    tags = lists
      //  .filter(([k, v]) => v == 1)
      .map(t => {
        let { path, value } = deep.find(xml, (v, p) => v.tagName == t);
        let selected = deep.select(xml, (v, p) => v.tagName == t);
        let xpath = new Path(path, true).xpath(xml);
        xpath = xpath.slice(-2);
        path = obj2path(value);
        console.log(
          'r:',

          selected
            .map(({ path, value }) => [new Path(path, true), value])
            .map(([p, v]) => [p.xpath(xml), v])
            .map(([p, v]) => [
              p,
              v,
              p.offset((o, i) => {
                let r = !/\[/.test(o);
                /* console.log("o:",o,i,r); */ return r;
              }) - 2
            ])
            .map(([p, v, o]) => [p.shift(o).unshift('/'), p.slice(0, o), v])
            .map(([p, o, v]) => [p[Symbol.for('nodejs.util.inspect.custom')](), o, v.children.length])
            .map(a => a.join(' '))
            .join('\n')
        );
        return [t, selected];
      });
    tags = new Map(tags);
    console.log('tags', tags);

    console.log(XPath + '');
    let x = new XPath('/eagle/drawing/board');
    let drawing = deep.find(xml, v => v.tagName == 'drawing');
    let board = deep.find(xml, v => v.tagName == 'board');
    let w = new Path(board.path);
    let y = w.xpath(xml);
    let z = x.apply(board.value);
  }

  main(...process.argv.slice(2));
} catch(err) {
  //console.log('err:', err);
}
