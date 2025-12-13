import { ReadFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { abbreviate, define, isObject, weakMapper } from './lib/misc.js';
import tXml from './lib/tXml.js';
class Node {
  constructor(raw, path) {
    define(this, { raw, path });
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
  const nodes = weakMapper(
    (value, path) =>
      new Proxy(handler && handler.construct ? handler.construct(value, path) : value, {
        get(target, key) {
          let prop = value[key];

          //console.log('get ', { key, prop });

          if(key == 'attributes') return prop;

          if(key !== 'attributes' && (isObject(prop) || Array.isArray(prop))) return new node([...path, key]);

          return handler && handler.get ? handler.get(prop, key) : prop;
        },
        ownKeys(target) {
          if('attributes' in value) {
            //console.log('ownKeys', Object.keys(value.attributes));

            return Object.keys(value.attributes);
          }

          return Reflect.keys(target);
        }
      })
  );

  function node(path) {
    let value = ptr(path);
    console.log('node:', { path, value });

    let proxy = nodes(value, path);

    return proxy;
  }

  return new node([]);
};

function main() {
  let str = ReadFile('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd');

  let xml = tXml(str);
  console.log('xml:', abbreviate(xml));

  let p = proxyObject(xml[0], {
    construct(value, path) {
      console.log('construct', { value, path });
      return 'tagName' in value ? new Node(value, path) : new NodeList(value, path);
    }
  });
  console.log('obj', p);
  console.log('tagName', p.tagName);
  console.log('children[0]', p.children[0]);
  console.log('children[0].tagName', p.children[0].tagName);
  console.log('keys(children[0])', Object.keys(p.children[0]));

  let result = deep.select(p, o => isObject(o) && o.attributes !== undefined && o.name !== undefined);
  console.log('result:', result);

  for(let { path, value } of result) {
    const {
      tagName: type,
      attributes: { name }
    } = value;
    console.log('found:', { type, name });
  }
}

main(...scriptArgs.slice(1));