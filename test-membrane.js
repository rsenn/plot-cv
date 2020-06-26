import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import fs from 'fs';
import { Path, PathMapper, toXML, TreeObserver, findXPath } from './lib/json.js';
import { Console } from 'console';
const printNode = node => {
  let s = toXML(node).replace(/\n.*/g, '');
  return s;
};
const CH = 'children';
global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 2, colors: true }
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
      {},
      (v, p, r) => typeof v == 'object' && v !== null && v.tagName !== undefined,
      (p, v) => obj2path(v, p), //.xpath(xml)
      ({ tagName, attributes, children, ...value }) => (children.length ? [tagName, attributes, children] : [tagName, attributes])
    );
    let flat = new Map(Object.entries(path2obj));

    console.log('flat:', path2obj['children.0.children.0.children.3.children.8.children.302.children.1']);
    //console.log('flat:', path2obj);
    let signals = findXPath('/eagle/drawing/board/signals/signal', flat, { root: xml, recursive: false, entries: false });
    let all = findXPath('//board/[^/]*/?[^/]*', flat, { root: xml, recursive: false, entries: false });
    let k = [...all.keys()];
    console.log('signals:', signals);
    console.log('all:', k); //.map(k => (k+'').replace(/.*\//g, "")));mk
    let rel,
      prev = new Path([], true),
      prevParent,
      relTo = [];
    const mk = ([path, value]) => {
      path = path.join('/').replace(/\/\[/g, '[');

      let p = new Path(path, true);
      //((prev||[]).slice(0, -1)+'');

      let thisParent = p.parent;
     // console.log('mk=', { thisParent, p, prev, prevParent });
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
    console.log('drawing:', findXPath('//drawing', flat, { root: xml, entries: true, recursive: true }).map(mk));

    //process.exit(0);
    let p = treeObserve.get(path2obj);
    let node = p;
    let unwrapped, type, path;
    treeObserve.subscribe((what, target, key, path) => {
      if(what == 'access') return;
      console.log('event', what, key, path);
      //    if(mapper === null) process.exit(1);
    });
    mapper.set(treeObserve.unwrap(node), []);
    for(let [path, obj] of flat) {
      let [tagName, attributes, children] = obj;
      if(path == '0') path = '';
      path = new Path(path, true);
      //console.log('path:', path.xpath(xml));
      if(Object.keys(attributes).length == 0) continue;

      let xpath = path2xpath(obj2path(path.apply(xml)));
      console.log('element:', /*path,*/ xpath + '', toXML({ tagName, ...attributes }, 1, ''));
    }
    while(node) {
      unwrapped = treeObserve.unwrap(node);
      type = treeObserve.getType(node);
      path = treeObserve.getXPath(node).toString();
      if(Util.isObject(node)) {
        if(!Util.isObject(node.attributes)) node.attributes = {};
      }
      const { x1, x2, y1, y2 } = node.attributes;
      if(x1 !== undefined) console.log('x1,x2,y1,y2', { x1, x2, y1, y2 });
      console.log('tagName:', node.tagName);
      console.log('type:', type, path);
      if(node.children === undefined || !node.children.length) break;
      node = node.children[node.children.length - 1];
    }
    let found = deep.find(xml, (node, path) => node.tagName !== undefined && ['parts', 'instances', 'elements'].indexOf(node.tagName) != -1);
    node = flat.get(found.path); //.value || mapper.atlat.(found.path);
    console.log('p:', printNode(p));
    console.log('found:', found);
    console.log('node:', node);
    let xpath;
    xpath = mapper.xpath(unwrapped);
    let r = Path.parseXPath('[0]');
    //console.log('xpath:', [...r]);
    //console.log('xpath:', r.toString());
    console.log('xpath r', mapper.xpath(unwrapped).toString());
    console.log('result', r);
    p = Path.parseXPath('/eagle/drawing/board/signals');
    console.log('mapper', p);
    p = Path.parseXPath("/eagle/drawing/board/signals/signal[@name='N$10']");
    console.log('mapper', p);
  }
  main(...process.argv.slice(2));
} catch(err) {
  console.log('err:', err);
}
