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
  inspectOptions: { depth: 0, colors: true }
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

    //    let flat = path2obj instanceof Map ? path2obj : new Map(Object.entries(path2obj));

    let rel,
      prev = new Path([], true),
      prevParent,
      relTo = [];
    const mk = ([path, value]) => {
      path = [...path].join('/').replace(/\/\[/g, '[');

      let p = new Path(path, true);
      console.log('mk=', { path, p });

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

    //process.exit(0);
    let p = treeObserve.get(Object.fromEntries(flat.entries()));
    let node = p;
    let unwrapped, type, path;

    treeObserve.subscribe((what, target, key, p, value) => {
      if(what == 'access') return;
      if(target[key] === undefined) return;
      let [path, k] = p;
      path = new Path(path);
      let targetType = treeObserve.getType(target);
      let targetKeys = Object.keys(target).join(',');
      let valueType = typeof value;
      console.log(`event`, Util.toString({ what,/* targetType, key, targetKeys, */ valueType, 
        xpath: path.xpath(xml)+'', string: typeof value == 'string' ? value : '' }));
    });
    mapper.set(treeObserve.unwrap(node), []);

    for(let [path, obj] of path2obj) {
      //   console.log('path2obj:', { path, obj });

      let [tagName, attributes, children] = obj;
      if(path == '0') path = '';
      path = new Path(path, true);

      obj.attributes;

      if(Object.keys(attributes).length == 0) continue;

      let xpath = path2xpath(obj2path(path.apply(xml)));
    }
    for(let path in p) {
      let obj = p[path];
      let { tagName, attributes, children } = obj;
      path = new Path(path, true);
      let str = path.toString('/');
      if(str == '0') str = '';
      else if(str.length > 0 && /[0-9]/.test(str[0])) str = '/children/' + str;
      else if(str.length > 0 && !str.startsWith('/')) str = '/' + str;
      let o = p[str];
      if(o === undefined) continue;
      let keys = Object.keys(o);
      if(o.attributes) o.attributes.test = 'AAA';
    }
    let found = deep.find(xml, (node, path) => node.tagName !== undefined && ['parts', 'instances', 'elements'].indexOf(node.tagName) != -1);
    node = flat.get(found.path);
    console.log('found:', found);
    console.log('node:', node);
    let xpath;
    xpath = mapper.xpath(unwrapped);
    let r = Path.parseXPath('[0]');
  }
  main(...process.argv.slice(2));
} catch(err) {
  console.log('err:', err);
}
