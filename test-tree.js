import Tree from './lib/tree.js';

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
  ],
  c: {
    test: [1, 2, 3, 4],
    array: [{ type: 'x' }, { type: 'y' }, { type: 'z' }],
    strings: ['a', 'b', 'c', 'd']
  }
};

async function main(...args) {
  let tree = new Tree(treeObj);

  console.log('tree:', tree);
  /*  console.log('tree():', tree(treeObj.b[3].haha));
  console.log('tree().path:', tree(treeObj.b[3].haha).path);*/
  let map = new Map(tree.entries());
  console.log('map:', map);

  console.log('tree.indexOf():', tree.indexOf(treeObj.b[3], treeObj.b[3]['*']));
  console.log('tree.indexOf():', tree.indexOf(treeObj.b[3]['*']));

  console.log('tree.keyOf():', tree.keyOf(treeObj.b[3], treeObj.b[3]['*']));
  console.log('tree.keyOf():', tree.keyOf(treeObj.b[3]['*']));

  console.log('tree.pathOf():', tree.pathOf(treeObj.b[3].haha));
  console.log('tree.at():', tree.at(['b', 3, 'haha']));
  console.log('tree.parentNode():', tree.parentNode(tree.parentNode(treeObj.b[3].haha)));
  console.log('tree.push():', tree.push(treeObj.b[3]['*'], 9, 8, 7));
  console.log('tree.keyOf():', tree.keyOf(treeObj.b[3]));
  console.log('treeObj:', treeObj);
  console.log('tree.shift():', tree.shift(treeObj.b[3]));
  //console.log('typeof entries:', Util.className(entries), Object.getPrototypeOf(entries).constructor, Util.isGenerator(entries), Util.isIterable(entries));
  //console.log('tree.entries():', [...tree.entries()]);
  console.log('tree.values():', [...tree.values()]);
  console.log('tree.flat():', tree.flat());
  console.log('tree.remove():', tree.remove(treeObj.b));
  console.log('tree():', tree(treeObj.c));
  console.log('tree():', tree(['c']));
  console.log('tree():', tree('c.array'));
  console.log('[...tree]:', [...tree]);

  console.log('treeObj:', treeObj);
}

main(...scriptArgs.slice(1));