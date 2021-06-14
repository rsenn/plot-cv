import ConsoleSetup from './lib/consoleSetup.js';
import PortableFileSystem from './lib/filesystem.js';
import tXml from './lib/tXml.js';
import deep from './lib/deep.js';
import Util from './lib/util.js';

let filesystem;

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

          //console.log('get ', { key, prop });

          if(key == 'attributes') return prop;

          if(key !== 'attributes' && (Util.isObject(prop) || Util.isArray(prop)))
            return new node([...path, key]);

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
    //console.log("node:",{path,value});

    let proxy = nodes(value, path);

    return proxy;
  }

  return new node([]);
};

async function main() {
  //  await ConsoleSetup({ breakLength: 120, depth: 10 });
  await PortableFileSystem(console.log);

  let str = filesystem
    .readFile('../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd')
    .toString();

  let xml = tXml(str);
  //console.log('xml:', Util.abbreviate(xml));

  let p = proxyObject(xml[0], {
    construct(value, path) {
      //console.log('construct', { value, path });
      return 'tagName' in value ? new Node(value, path) : new NodeList(value, path);
    }
  });
  //console.log('obj', p);
  //console.log('tagName', p.tagName);
  //console.log('children[0]', p.children[0]);
  //console.log('children[0].tagName', p.children[0].tagName);
  //console.log('keys(children[0])', Object.keys(p.children[0]));

  let result = deep.select(
    p,
    (
      o //console.log('o:', o);
    ) => Util.isObject(o) && o.attributes !== undefined && o.name !== undefined
  );
  //console.log('result:', result);

  for(let { path, value } of result) {
    const {
      tagName: type,
      attributes: { name }
    } = value;
    //console.log('found:', { type, name });
  }
}

main(Util.getArgs());
