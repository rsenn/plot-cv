import HashMultimap from './lib/container/Hash-Multimap.js';
import MultiBiMap from './lib/container/multibimap.js';
import MultiKeyMap from './lib/container/multikeymap.js';
async function main(...args) {
  let multiKey = new MultiKeyMap();
  multiKey.set(['A', 'B', 'C'], 123);
  console.log('multiKey', [...multiKey]);

  let multiBi = new MultiBiMap({ iterableKey: false, iterableValue: true });

  multiBi.add('A', [1, 2, 3, 4]);
  console.log('multiBi', [...multiBi]);
  console.log('multiBi keys', multiBi.keys());
  console.log('multiBi values', multiBi.values());

  let hashMulti = new HashMultimap();

  hashMulti.put('A', 1, 2, 3, 4);
  console.log('hashMulti', [...hashMulti]);
}

main(...scriptArgs.slice(1));