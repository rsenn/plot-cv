import Util from './lib/util.js';
import PortableFileSystem from './lib/filesystem.js';
import PortableConsole from './lib/consoleSetup.js';
import { parse2 } from './lib/xml/parse.js';
import { TreeWalker } from 'tree-walker.so';

async function main(...args) {
  await PortableFileSystem();
  await PortableConsole({ compact: 2 });

  let data = filesystem.readFile(args[0] ?? 'FM-Radio-Simple-Receiver-Dip1.sch', 'utf-8');

  //console.log('data:', Util.abbreviate(Util.escape(data)));
  console.log('data.length:', data.length);

  let result = /*tXml(data); //*/ parse2(data);
  console.log('result:', result);

  let walk = new TreeWalker(result[0]);
 console.log('walk:', walk.toString());

  /*  console.log('root:', walk.root);
  console.log('length:', walk.length);
  console.log('key:', walk.key);
  console.log('firstChild:', walk.firstChild());
  do {
    const { key, currentNode, index } = walk;
    console.log('walk', { key, currentNode, index });
  } while(walk.nextSibling());
  console.log('length:', walk.length);
  console.log('depth:', walk.depth);
  console.log('index:', walk.index);*/
let i= 0;
walk.tagMask = 0b111011110;
 console.log('~TreeWalker.MASK_PRIMITIVE:', TreeWalker.MASK_PRIMITIVE.toString(2));
 console.log(' TreeWalker.MASK_ALL:', TreeWalker.MASK_ALL);
 console.log(' TreeWalker.MASK_ALL:', TreeWalker.MASK_ALL.toString(2));

//walk.tagMask = 0xffff;
walk.tagMask = TreeWalker.MASK_PRIMITIVE;
 //walk.expr = "^(-?[0-9.]*)$";
 while(walk.nextNode()) {
 //console.log('walk:', walk.toString());
   // const { currentKey, currentPath, currentNode, index, depth } = walk;
   // console.log('nextNode:', { i, index, depth, currentKey, currentPath, currentNode });

   console.log('type:',  typeof walk.currentNode, 'path:', walk.currentPath.join('.').replace(/\.?children\.?/g, "/"), typeof walk.currentNode != 'object' ? walk.currentNode : '');
let node = walk.currentNode;

if(typeof node == 'object') {
   console.log('attributes:', Object.getOwnPropertyNames(node).filter(n => typeof node[n] != 'object').reduce((acc,name) => ({ ...acc, [name]: node[name] }), {}) );
 }
    i++;
  }

 console.log('walk:', walk.toString());

 await import('std').then(std => std.gc());
}

Util.callMain(main, true);
