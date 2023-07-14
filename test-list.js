import { define } from 'util';
import { Console } from 'console';
import { List } from 'list';
globalThis.console = new Console({
  inspectOptions: {
    compact: 2,
    depth: Infinity,
    customInspect: true,
    maxArrayLength: 200
  }
});

class MyList extends List {
  constructor(...args) {
    super(...args);
  }
}

define(MyList, {
  [Symbol.species]: Set
});

let l = new MyList([6, 5, 4, 3, 2, 1]);
console.log(l);
console.log(MyList[Symbol.species]);
console.log((l = l.concat([1, 2, 3])));
console.log(Object.getPrototypeOf(l));