import ObservableMembrane from './lib/proxy/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import fs from 'fs';
import { Path, PathMapper, toXML, TreeObserver } from './lib/json.js';
import { Console } from 'console';

const CH = 'children'; //Path.CHILDREN;

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 1, colors: true }
});

try {
  const path = new PathMapper();
  let membrane = new TreeObserver(path);

  function main() {
    let str = fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

    let xml = tXml(str)[0];
    //console.log('xml:', xml);

    let p = membrane.getProxy(xml);
    let node = p;
    let unwrapped;

    path.set(membrane.unwrapProxy(node), []);

    while(node) {
      unwrapped = membrane.unwrapProxy(node);
      console.log('unwrapped:', toXML(unwrapped, false).replace(/\n.*/g, ''));
      console.log('path:', path.get(unwrapped) + '');
      console.log('tagName:', node.tagName);

      if(node.children === undefined || !node.children.length) break;
      node = node.children[node.children.length - 1];
    }

    console.log('node:', path.at([]));

    console.log('node:', path.parent(path.at([CH, 0])));
    console.log('node:', path.nextSibling(path.at([CH, 0, CH, 0, CH, 0])));
    console.log('node:', path.firstChild(path.at([CH, 0, CH, 0])));
    console.log('node:', path.nextSibling(path.firstChild(path.at([CH, 0, CH, 0]))));
//    console.log('node:', path.xpath(path.firstChild(path.at([CH, 0, CH, 0]))).toString());
    let xpath;

xpath = path.xpath(unwrapped);
    console.log('node:', xpath);
    let r = Path.parseXPath('/eagle');
    console.log('xpath', xpath);

    console.log('result', r);
    console.log('node', path.at('/eagle/drawing/board'));
    p = Path.parseXPath('/eagle/drawing/board/signals');
    console.log('path', p);
    console.log('node', path.at(p));
    p = Path.parseXPath("/eagle/drawing/board/signals/signal[@name='N$10']");
    console.log('path', p);
    console.log('node', toXML(path.at(p)));
  }

  main(process.argv.slice(2));
} catch(err) {
  console.log('err:', err);
}
