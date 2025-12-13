import trkl from './lib/trkl.js';
let testObj = {}, testValues = [1, 2, 3, 4, 5];

trkl.bind(testObj, 'prop1', value => (value === undefined ? testValues[0] : (testValues[0] = value)));

console.log('testObj.prop1', testObj.prop1);

testObj.prop1 = 'a';
console.log('testObj.prop1', testObj.prop1);
console.log('testValues', testValues);