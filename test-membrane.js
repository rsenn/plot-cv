import ObservableMembrane from './lib/observableMembrane.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';
import fs from 'fs';
import { text, EagleInterface, toXML, dump } from './lib/eagle/common.js';
import { EaglePath } from './lib/eagle/locator.js';

import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 1, colors: true }
});


function PathMapper() {
  let pathMap = new WeakMap();
  let locationMap = new Map();

  return {
    pathMap,
    locationMap,
    at(path) {
      return locationMap.get(path.join('.'));
    },
    set(obj, path) {
      if(!(path instanceof EaglePath)) path = new EaglePath(path);
      locationMap.set(path.join('.'), obj);
      pathMap.set(obj, path);
    },
    get(obj) {
      let r = pathMap.get(obj) || null;
      return r;
    }
  };
}
const path = new PathMapper();

let membrane = new ObservableMembrane({
  valueObserved(target, key) {
    // where target is the object that was accessed
    // and key is the key that was read
    let p = path.get(membrane.unwrapProxy(target) || target);

    let value;
    if(Util.isObject(p)) {
      value = p[key] || target[key];

      if(Util.isObject(value)) {
        for(let prop in value)
          if(Util.isObject(value[prop])) {
            //console.log('value[prop]:', value[prop]);

            path.set(value[prop], Util.isNumeric(prop) ? [...p, 'children', +prop] : [...p, prop]);
          }
      }

      p = [...p, key];
    }
    //console.log('accessed ', { target, key, value, p });
  },
  valueMutated(target, key) {
    // where target is the object that was mutated
    // and key is the key that was mutated
    //console.log('mutated ', key);
  },
  valueDistortion(value) {
    if(!Util.isObject(value)) return value;

    let proto = typeof value == 'object' ? Object.getPrototypeOf(value) : Object.prototype;

    let p = path.get(value);
    const args = [...arguments];
    //console.log('distorting ', { value, p, args });
    if(proto === Proxy.prototype) value = membrane.getReadOnlyProxy(value);
    return value;
  }
});

async function main() {
  let str = fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

  let xml = tXml(str)[0];
  //console.log('xml:', xml);

  let p = membrane.getProxy(xml);
  let node = p;

  path.set(membrane.unwrapProxy(node), []);

  while(node) {
    let unwrapped = membrane.unwrapProxy(node);
    console.log('unwrapped:', toXML(unwrapped, false).replace(/\n.*/g, ''));
    console.log('path:', path.get(unwrapped) + '');
    console.log('tagName:', node.tagName);

    if(node.children === undefined || !node.children.length) break;
    node = node.children[node.children.length - 1];
  }
  console.log('root:', path.at([]));
  console.log('root:', path.at(['children', 0]));
}

main(process.argv.slice(2));
