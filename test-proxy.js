//import RecursiveObject from './lib/proxy/recursiveObject.js';

import fs from 'fs';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';

import { Console } from 'console';

global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 3, colors: true }
});

/*
class Node extends (Util.proxyObject) {
  constructor(root, handler) {
    super(root, handler);
  }
}
*/
class Node {
  constructor(raw, path) {
    Util.define(this, { raw, path });
    this.raw = raw;
    this.path = path;
  }
}
class NodeList {
  constructor(raw, path) {
    this.raw = raw;
    this.path = path;
  }
}

const proxyObject = (root, handler) => {
  const ptr = path => path.reduce((a, i) => a[i], root);
  const nodes = Util.weakMapper(
    (value, path) =>
      new Proxy(handler && handler.construct ? handler.construct(value, path) : value, {
        get(target, key) {
          let prop = value[key];

          //Util.log('get ', { key, prop });

          if(key == 'attributes') return prop;

          if(key !== 'attributes' && (Util.isObject(prop) || Util.isArray(prop))) return new node([...path, key]);

          return handler && handler.get ? handler.get(prop, key) : prop;
        },
        ownKeys(target) {
          if('attributes' in value) {
            //Util.log('ownKeys', Object.keys(value.attributes));

            return Object.keys(value.attributes);
          }

          return Reflect.keys(target);
        }
      })
  );

  function node(path) {
    let value = ptr(path);
    //Util.log("node:",{path,value});

    let proxy = nodes(value, path);

    return proxy;
  }

  return new node([]);
};

async function main() {
  let str = fs.readFileSync('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();

  let xml = tXml(str);
  //Util.log('xml:', Util.abbreviate(xml));

  let p = proxyObject(xml[0], {
    construct(value, path) {
      //Util.log('construct', { value, path });
      return 'tagName' in value ? new Node(value, path) : new NodeList(value, path);
    }
  });
  //Util.log('obj', p);
  //Util.log('tagName', p.tagName);
  //Util.log('children[0]', p.children[0]);
  //Util.log('children[0].tagName', p.children[0].tagName);
  //Util.log('keys(children[0])', Object.keys(p.children[0]));

  let result = deep.select(p, o => {
    //Util.log('o:', o);
    return Util.isObject(o) && o.attributes !== undefined && o.name !== undefined;
  });
  //Util.log('result:', result);

  for(let { path, value } of result) {
    const {
      tagName: type,
      attributes: { name }
    } = value;
    //Util.log('found:', { type, name });
  }
}

main(process.argv.slice(2));
