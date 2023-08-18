import { ReadFile } from './io-helpers.js';
import deep from './lib/deep.js';
import { ImmutablePath, PathMapper, toXML, TreeObserver } from './lib/json.js';
import tXml from './lib/tXml.js';
import { XMLIterator } from './lib/xml/util.js';
import { ImmutableXPath, parseXPath } from './lib/xml/xpath.js';
//import { ImmutableXPath, parseXPath, XMLIterator } from './lib/xml.js';

const printNode = node => {
  let s = toXML(node).replace(/\n.*/g, '');
  return s;
};

const CH = 'children';

function main(...args) {
  let str = ReadFile(args.length ? args[0] : '../an-tronics/eagle/Headphone-Amplifier-ClassAB-alt3.brd').toString();
  let xml = tXml(str)[0];
  const mapper = new PathMapper(xml, parseXPath);
  let observer = new TreeObserver(mapper, false);
  const path2obj = path => mapper.get(path);
  const obj2path = observer.getField('path');
  const obj2type = observer.getField('type');
  const path2xpath = path => {
    if(!(path instanceof ImmutablePath)) path = new ImmutablePath(path, true);
    return path;
  };

  let flat = new Map();
  let rel,
    prev = new ImmutablePath([], true),
    prevParent,
    relTo = [];
  const mk = ([path, value]) => {
    let p = new ImmutablePath(path);
    path = p.toArray();
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

  flat.set('A', 0);
  flat.set('B', 1);
  let k = flat.entries();
  let o = Object.fromEntries(k);
  let p = observer.get(o);
  let node = p;
  let unwrapped, type, path;
  observer.subscribe((...args) => {
    let [what, target, path, value] = args;
    let basePath = obj2path(target);
    path = new ImmutablePath(path, true);
    let parentPath = path.parentNode;
    let parent = path2obj(parentPath);
    let valueType = Util.typeOf(value);
    let targetType = observer.type(target);
    let targetPath = obj2path(target);
    let targetXPath = path2xpath(targetPath);
    let xpath = ImmutableXPath.from(path, target);
    let string = typeof value == 'string' ? value : '';
  });

  mapper.set(observer.unwrap(node), []);
  let tree = observer.get(xml);

  /* { function_description }
   *
   * @param      {<type>}  obj     The object
   * @param      {<type>}  prop    The property
   * @param      {number}  [i=1]   { parameter_description }
   * @return     {Object}  { description_of_the_return_value }
   */
  const incr = (obj, prop, i = 1) => {
    if(!obj) obj = {};
    return { ...obj, [prop]: (obj[prop] || 0) + i };
  };

  let tags = {};
  let iter = new XMLIterator(
    {
      children: xml.children,
      tagName: tree.tagName,
      attributes: tree.attributes
    },
    (v, p) => true
  );
  for(let [v, p] of iter) {
    if(!(p instanceof ImmutablePath)) p = new ImmutablePath(p, true);
    tags = incr(tags, v.tagName);
  }
  let lists = [];
  for(let key in tags) {
    let lkey = key == 'library' ? 'libraries' : key + 's';
    if(tags[lkey] !== undefined) lists.push(lkey);
  }
  tags = Object.entries(tags).sort((a, b) => a[1] - b[1]);
  tags = lists.map(t => {
    let { path, value } = deep.find(xml, (v, p) => v.tagName == t);
    let selected = deep.select(xml, (v, p) => v.tagName == t);
    let xpath = ImmutableXPath.from(path, xml);
    xpath = xpath.slice(-2);
    let q = new RegExp('(.)*');
    path = obj2path(value);

    let dumps = selected.map(({ path, value }) => [new ImmutablePath(path, true), value]);

    dumps = dumps.map(([p, v]) => [ImmutableXPath.from(p, xml), v]);

    dumps = dumps.map(([p, v]) => [p, v, p.offset((o, i, p) => !(/(\[|board$|sheets$)/.test(o) || p[i + 1] == 'attributes'))]);

    dumps = dumps.map(([p, v, o]) => [p.slice(o - 2), p.slice(0, o - 2), v, o]);
    dumps = dumps.map(([p, s, v, o]) => [p, s, `children: ${v.children.length}`, `offset: ${o}`]);
    dumps = dumps.map(([p, s, v, o]) => [p, s, v, p.toRegExp()]);

    dumps = dumps
      .map(([p, s, v, r]) => [p, s, v, r.test(p), r.test(s), [...(p.toString() + '').match(q)], [...q.exec(p)].slice(1)])
      .map(([p, s, ...rest]) => [p[Symbol.for('nodejs.util.inspect.custom')](), s[Symbol.for('nodejs.util.inspect.custom')](), ...rest.map(i => Util.toSource(i))])
      .map(([p]) => p + '')
      .join('\n  |');

    //console.log('result:\n  ', dumps);
    return [xpath, new Map(selected.map(({ path, value }) => [path2xpath(path).down('*'), value]))];
  });

  tags = Object.fromEntries(tags);
  for(let [search, value] of Object.entries(tags)) {
    let path = new ImmutablePath(search);
  }
  let x = new ImmutableXPath('/eagle/drawing/board');
  let drawing = deep.find(xml, v => v.tagName == 'drawing');
  let w = new ImmutablePath('children/0/children/0/children/3', true);
  //console.log('w:', w.toString());
  let y = ImmutableXPath.from(w, xml);
  //console.log('y:', y, [...y]);
  let z = w.apply(xml);
  let u = observer.get(z);
  //console.log('z:', z);
}

main(...scriptArgs.slice(1));