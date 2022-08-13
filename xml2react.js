import { read as readXML } from 'xml';
import { IfDebug, LogIfDebug, ReadFile, LoadHistory, ReadJSON, ReadXML, MapFile, WriteFile, WriteJSON, WriteXML, ReadBJSON, WriteBJSON, DirIterator, RecursiveDirIterator, ReadDirRecursive, Filter, FilterImages, SortFiles, StatFiles, ReadFd, FdReader, CopyToClipboard, ReadCallback, LogCall, Spawn, FetchURL } from './io-helpers.js';
import { Console } from 'console';

function* injectSeparator(iter, sep = ', ', pad = ' ') {
  let i = 0;
  for(let item of iter) {
    yield i++ == 0 ? pad : sep;
    yield item;
  }
  if(i) yield pad;
}

function* iterateKV(obj) {
  for(let key in obj) yield [key, obj[key]];
}

function* mapIterator(iter, fn = (elem, i, iter) => undefined) {
  let i = 0;
  for(let elem of iter) yield fn(elem, i++, iter);
}

function* xml2h(xml, depth = 0) {
  const { tagName, attributes, children } = xml;
  let sep = ',\n' + '  '.repeat(depth);
  let nl = '\n' + '  '.repeat(depth);
  let separate = it => injectSeparator(it, sep, nl);
  yield `h('${tagName}', {`;

  let i = 0,
    j = 0;

  yield* separate(mapIterator(iterateKV(attributes), ([k, v]) => `${k}: '${v}'`));

  yield `}`;
  //for(let name in attributes) yield (i++ == 0 ? ' ' : ', ') + `${name}: '${attributes[name]}'`;

  if(children && Array.isArray(children) && children.length > 0) {
    yield `, [`;

    //yield* injectSeparator(mapIterator(children, child => typeof child == 'string' ? yield `'${child}'` : yield* xml2h(child))
    for(let child of children) {
      yield j++ == 0 ? nl : sep;

      if(typeof child == 'string') yield `'${child}'`;
      else yield* xml2h(child, depth + 1);
    }
    //  yield '\n'+'  '.repeat(Math.max(0, depth-1));
    // yield nl;

    yield `]`;
  }
  // yield '\n'+'  '.repeat(Math.max(0, depth-1));
  yield `)`;
}

function main(...args) {
  globalThis.console = new Console({
    colors: true,
    depth: Infinity
  });

  for(let arg of args) {
    let data = ReadFile(arg);
    let [doctype, xml] = readXML(data);

    console.log('xml', xml);

    let str = '';
    for(let frag of xml2h(xml)) {
      str += frag;
    }
    console.log('str', str);
  }
}

main(scriptArgs.slice(1));
