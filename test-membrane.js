import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import fs from 'fs';
import { Path, PathMapper, toXML, TreeObserver } from './lib/json.js';
import { Console } from 'console';

const printNode = node => {
  let s = toXML(node).replace(/\n.*/g, '');
  return s;
};

const CH = 'children'; //Path.CHILDREN;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 1, colors: true }
});
Error.stackTraceLimit = 100;

try {
  const mapper = new PathMapper();
  let treeObserve = new TreeObserver(mapper, false);

  function main() {
    let str = fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

    let xml = tXml(str)[0];
    //console.log('xml:', xml);

    let p = treeObserve.get(xml);
    let node = p;
    let unwrapped, type, path;

    treeObserve.subscribe((what, target, key, path) => {
      if(what == 'access') return;
      console.log('event', what, key, path);
      //   console.log('target.attributes', target.attributes);
      if(mapper === null) process.exit(1);
    });

    mapper.set(treeObserve.unwrap(node), []);

    while(node) {
      unwrapped = treeObserve.unwrap(node);
      type = treeObserve.getType(node);
      path = treeObserve.getXPath(node).toString();

      //  node['test'] = true;
      if(Util.isObject(node)) {
        if(!Util.isObject(node.attributes)) node.attributes = {};

        node['test'] = true;
      }
      const { x1, x2, y1, y2 } = node.attributes;
      if(x1 !== undefined)
        //  console.log('unwrapped:', toXML(unwrapped, false).replace(/\n.*/g, ''));
        console.log('x1,x2,y1,y2', { x1, x2, y1, y2 });
      //console.log('xpath', r.toString());
      console.log('tagName:', node.tagName);
      console.log('node:', printNode(node));
      console.log('type:', type, path);

      if(node.children === undefined || !node.children.length) break;
      node = node.children[node.children.length - 1];
    }

    console.log('node:', printNode(mapper.at([])));

    console.log('node:', printNode(mapper.parent(mapper.at([CH, 0]))));
    console.log('node:', printNode(mapper.nextSibling(mapper.at([CH, 0, CH, 0, CH, 0]))));
    console.log('node:', printNode(mapper.firstChild(mapper.at([CH, 0, CH, 0]))));
    console.log('node:', printNode(mapper.nextSibling(mapper.firstChild(mapper.at([CH, 0, CH, 0])))));
    //    console.log('node:',  printNode(mapper.xpath(mapper.firstChild(mapper.at([CH, 0, CH, 0]))).toString());
    let xpath;

    xpath = mapper.xpath(unwrapped);
    console.log('node:', printNode(xpath));
    let r = Path.parseXPath('[0]'); //Path.parseXPath('/eagle');
    console.log('xpath', [...r]);
    console.log('xpath', r.toString());
    console.log('xpath r', mapper.xpath(unwrapped).toString());
    //  process.exit(0);

    console.log('result', r);
    console.log('node:', printNode(mapper.at('/eagle/drawing/board')));
    p = Path.parseXPath('/eagle/drawing/board/signals');
    console.log('mapper', p);
    console.log('node:', printNode(mapper.at(p)));
    p = Path.parseXPath("/eagle/drawing/board/signals/signal[@name='N$10']");
    console.log('mapper', p);
    console.log('node:', printNode(toXML(mapper.at(p))));
  }

  main(process.argv.slice(2));
} catch(err) {
  console.log('err:', err);
}
