import { toXML } from './lib/json/util.js';
import * as path from './lib/path.js';
import Tree from './lib/tree.js';
import parse from './lib/xml/parse.js';
function WriteFile(name, data) {
  if(Array.isArray(data)) data = data.join('\n');
  if(typeof data != 'string') data = '' + data;

  filesystem.writeFile(name, data + '\n');

  console.log(`Wrote ${name}: ${data.length} bytes`);
}

class Comment extends String {
  constructor(s) {
    super(s);
    this.#field = 1337;
  }

  [Symbol.for('nodejs.util.inspect.custom')]() {
    return `Comment('${this}')`;
  }
}

class PList extends Array {
  constructor(...args) {
    super();
    this.push(...args);
  }
  [Symbol.for('nodejs.util.inspect.custom')](options) {
    return '\x1b[1;31mPList\x1b[0m [\n  ' + this.map(item => (item.inspect ? item.inspect(options) : console.inspect(item, options)).replace(/\n/g, '\n    ')).join(',\n  ') + '\n';
  }
}

class Pair {
  constructor(key, value) {
    Object.assign(this, { key, value });
  }

  [Symbol.for('nodejs.util.inspect.custom')](options) {
    const { key, value } = this;
    return `\x1b[1;31mPair \x1b[1;33m${key}\x1b[0m => ${console.inspect(value, options)}`;
  }

  [Symbol.iterator]() {
    const { key, value } = this;
    return [key, value][Symbol.iterator]();
  }
}

class Dict extends Array {
  constructor() {
    super();
  }

  [Symbol.for('nodejs.util.inspect.custom')](options) {
    return `\x1b[1;31mDict\x1b[0m {\n  ${this.map(([key, value]) => {
      let s = key + ' => ';
      if(value instanceof Array) s += '[\n    ' + value.map(item => (item.inspect ? item.inspect(options) : console.inspect(item, options)).replace(/\n/g, '\n    ')).join(',\n    ') + '\n  ]';
      else s += `${(value.inspect ? value.inspect(options) : console.inspect(value, options)).replace(/\n/g, '\n    ')}`;
      return s;
    }).join(',\n  ')}\n}`;
  }

  set(key, value) {
    //console.log('Dict.set', { key, value });
    this.push(new Pair(key, value));
  }
}

function Element2Object(element, key) {
  //console.log('Element2Object', { element, key });
  switch (element.tagName) {
    case '!--': {
      return new Comment(element.children[0]);
    }
    case 'array': {
      return element.children.map(Element2Object);
    }
    case 'plist': {
      return new PList(...element.children.map(Element2Object));
    }
    case 'string': {
      return element.children.join('\n') /*.split(/\n/g)*/;
    }
    case 'dict': {
      let ret = new Dict();
      let dict = element.children.slice();
      while(true) {
        while(dict.length && dict[0].tagName != 'key') {
          ret.push(Element2Object(dict.shift()));
        }

        if(dict.length == 0) break;
        let key = dict.shift().children[0];
        let value = dict.shift();
        ret.set(key, Element2Object(value, key));
      }
      return ret;
    }
  }
}

function Object2Element(object, path = []) {
  let type = typeOf(object);
  //console.log('Object2Element', { type,  path });
  switch (type) {
    case 'Comment': {
      return { tagName: '!--', children: [object + ''] };
    }
    case 'Array': {
      return {
        tagName: 'array',
        children: object.map((v, k) => Object2Element(v, path.concat([k])))
      };
    }
    case 'PList': {
      return {
        tagName: 'plist',
        attributes: { version: '1.0' },
        children: object.map((v, k) => Object2Element(v, path.concat([k])))
      };
    }
    case 'String': {
      return { tagName: 'string', children: [object] };
    }

    case 'Dict': {
      let ret = { tagName: 'dict', children: [] };
      let { children } = ret;
      //console.log('Object2Element Dict', object.length);
      for(let i = 0; i < object.length; i++) {
        const entry = object[i];
        const entryType = typeOf(entry);
        //console.log('Object2Element Dict', { entryType, entry });
        if(entryType == 'Pair' || (entry instanceof Array && entry.length == 2)) {
          const [key, value] = entry;
          children.push({ tagName: 'key', children: [key] });
          children.push(Object2Element(value, path.concat([key])));
        } else {
          children.push(Object2Element(entry, path.concat([i])));
        }
      }
      return ret;
    }
    default: {
      console.error({ object, path });
      throw new Error(`Unhandled type: ${type} ${path}`);
    }
  }
}

async function main(...args) {

  for(let file of args) {
    let base = path.basename(file, /\.[^.]*$/);

    let data = filesystem.readFileSync(file);
    console.log('data:', data);
    let xml = parse(data);

    let tree = new Tree(xml);

    let array = tree.find(n => isObject(n) && n.tagName == 'plist');
    let objs = Element2Object(array);

    /*for(let obj of objs) {
      const { scope } = obj;

      if(typeof scope == 'string') {
        let a = scope
          .split(/,\s+/g)
          .map(s => s.trim())
          .filter(s => s != '');

        scopes.push(...s);
      }
    }*/

    //    console.log('array:', array);
    //  console.log('xml:', xml);
    console.log('file:', file);
    let st = new Tree(objs);
    let pairs = st.flat(([, node]) => isObject(node) && node instanceof Pair);

    let scopes = [];

    if(/\.tmLanguage$/.test(file)) {
      scopes.push(...[...pairs.values()].filter(pair => pair.key == 'name' && /\./.test(pair.value)).map(pair => pair.value));
      //console.log('scopes:', scopes);
    } else {
      let a = [...pairs.values()].filter(pair => pair.key == 'scope' && /\./.test(pair.value)).map(pair => pair.value);

      scopes.push(
        ...a
          .map(s => s.split(/,\s+/g))
          .flat()
          .map(s => s.trim())
      );
    }

    let modified = Object2Element(objs);
    // console.log('objs:', objs);
    //  console.log('modified:', modified.children.slice(39,41));

    tree.replace(array, modified);

    //console.log('array:', array);
    let str = toXML(xml);
    //    console.log('str:', str);
    WriteFile(file, str);
    scopes = unique(scopes)
      .filter(s => !/(^col_|^-$|^\s*$)/.test(s) && !/^\s*$/.test(s))
      .sort();
    console.log('scopes:', scopes);
    WriteFile(`scopes-${base}.txt`, scopes.join('\n'));
  }

  return;
}

main(...scriptArgs.slice(1));