import Tree from './lib/tree.js';
import { Util } from './lib/util.js';
import ConsoleSetup from './lib/consoleSetup.js';

const treeObj = {
  a: 1,
  b: [
    1,
    2,
    3,
    {
      haha: { x: 0 },
      blah: { y: 1 },
      '*': [0, 0, 0]
    },
    4,
    5,
    6
  ]
};

async function main(...args) {
  await ConsoleSetup({ colors: true, depth: 6 });
  console.log('treeObj:', treeObj);

  let tree = new Tree(treeObj);

  console.log('tree:', tree);
  /*  console.log('tree():', tree(treeObj.b[3].haha));
  console.log('tree().path:', tree(treeObj.b[3].haha).path);*/
  console.log('tree.path():', tree.pathOf(treeObj.b[3].haha));
  console.log('tree.at():', tree.at(['b', 3, 'haha']));
  console.log('tree.parentNode():', tree.parentNode(tree.parentNode(treeObj.b[3].haha)));
}

Util.callMain(main, true);
