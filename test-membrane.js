import ObservableMembrane from './lib/proxy/observableMembrane.js';
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

class PathMapper {
  map = null;
  root = null;

  constructor() {
    this.map = new WeakMap();
  }
  at(path) {
    if(!(path instanceof EaglePath)) path = new EaglePath(path);
    return path.apply(this.root);
  }
  set(obj, path) {
    if(!(path instanceof EaglePath)) path = new EaglePath(path);
    if(path.length === 0) this.root = obj;
    this.map.set(obj, path);
  }
  get(obj) {
    let path = this.map.get(obj) || null;
    return path;
  }
  walk(obj, fn = path => path) {
    let path = this.get(obj);
    path = fn(path);
    if(!(path instanceof EaglePath)) path = new EaglePath(path);
    return this.at(path);
  }
  parent(obj) {
    return this.walk(obj, path => path.slice(0, typeof path.last == 'number' ? -2 : -1));
  }
  firstChild(obj) {
    return this.walk(obj, path => [...path, 'children', 0]);
  }
  lastChild(obj) {
    return this.walk(obj, path => [...path, 'children', -1]);
  }
  nextSibling(obj) {
    return this.walk(obj, path => [...path.slice(0, -1), path.last + 1]);
  }
  previousSibling(obj) {
    return this.walk(obj, path => [...path.slice(0, -1), path.last - 1]);
  }
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
  console.log('root:', path.parent(path.at(['children', 0])));
  console.log('root:', path.nextSibling(path.at(['children', 0, 'children', 0, 'children', 0])));
  console.log('root:', path.firstChild(path.at(['children', 0, 'children', 0])));
  console.log('root:', path.nextSibling(path.firstChild(path.at(['children', 0, 'children', 0]))));
}

main(process.argv.slice(2));
