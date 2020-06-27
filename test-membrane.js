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
      //console.log(`event`, Util.toString({ what, valueType, path, string }));
    });

    mapper.set(treeObserve.unwrap(node), []);
    /*    for(let [path, obj] of path2obj) {
      let [tagName, attributes, children] = obj;
      if(path == '0') path = '';
      path = new Path(path, true);
      obj.attributes;
      if(Object.keys(attributes).length == 0) continue;
      let xpath = path2xpath(obj2path(path.apply(xml)));
    }*/
    let tree = treeObserve.get(xml);

    /*  let iterated = new Map(
      [...].map(([value, path]) => [
        new Path(path, true)[Symbol.toStringTag]() ,
        value
      ])o
    );*/

    const incr = (obj, prop, i = 1) => {
      if(!obj) obj = {};
      return { ...obj, [prop]: (obj[prop] || 0) + i };
    };

    let tags = {};
    for(let [v, p] of XmlIterator({ children: xml.children, tagName: tree.tagName, attributes: tree.attributes }, (v, p) => true)) {
      if(!(p instanceof Path)) p = new Path(p, true);

      tags = incr(tags, v.tagName);

      //v = treeObserve.unwrap(v);
      //console.log('iterate', p.xpath(xml), ' =', v);
    }
    //e.log('tags', tags);

    tags = Object.entries(tags).sort((a, b) => a[1] - b[1]);
    tags = tags
      .filter(([k, v]) => v == 1)
      .map(([t]) => {
        const { path, value } = deep.find(xml, (v, p) => v.tagName == t);
        //console.log('map tags', path);
        let xpath = new Path(path, true).xpath(xml);
        return [t, xpath.slice(-4)];
      });
    tags = new Map(tags);
    // console.log('tags', tags);

    let x = new MutableXPath('/eagle/drawing/board');
    //console.log('x:', x);
    //console.log('x:', ...[...x]);

    let drawing = deep.find(xml, v => v.tagName == 'drawing');
    let board = deep.find(xml, v => v.tagName == 'board');
    // let designrules = deep.find(xml, v => v.tagName == 'designrules');
    //console.log('board.path', board.path, 'board.value', toXML(board.value, 0));
    let w = new Path(board.path);
    //console.log('w:', w);

    /* console.log('drawing.path', w, 'drawing.value', toXML(drawing.value, 0));
  //console.log('board.path', w, 'board.value', toXML(board.value, 0));*/

    let y = w.xpath(xml);

    //console.log('y:', y);
    //console.log('y:', ...[...y]);

    //console.log('x.equal(y):', x.equal(y));

    let z = x.apply(board.value);
    //console.log('z:', z);

    //console.log('y:', Util.className(y));
    //console.log('y:', inspect(y.toArray()));
    //console.log('z:', inspect(z, 2));
  }

  main(...process.argv.slice(2));
} catch(err) {
  //console.log('err:', err);
}
