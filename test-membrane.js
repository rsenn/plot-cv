import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import fs from 'fs';
import { Path, PathMapper, toXML, TreeObserver, findXPath, XmlIterator } from './lib/json.js';
import { Console } from 'console';

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
  inspectOptions: { depth: 1, colors: true }
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
      console.log('mk=', { path, value });
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
    console.log('drawing:', findXPath('//drawing', flat, { root: xml, entries: true, recursive: false }).map(mk));
    
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
      console.log(
        `event`,
        Util.toString({
          what,
           valueType,
          path,
           string
        })
      );
    });

    mapper.set(treeObserve.unwrap(node), []);
    for(let [path, obj] of path2obj) {
      let [tagName, attributes, children] = obj;
      if(path == '0') path = '';
      path = new Path(path, true);
      obj.attributes;
      if(Object.keys(attributes).length == 0) continue;
      let xpath = path2xpath(obj2path(path.apply(xml)));
    }
    let iterated = new Map([...XmlIterator(xml, (v, p) => true)].map(([v, p]) => [new Path(p, true).xpath(xml), v]));
    console.log('iterated:', iterated);
  }

  main(...process.argv.slice(2));
} catch(err) {
  console.log('err:', err);
}
