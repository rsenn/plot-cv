import { toPointer } from 'util';
import { CV_8UC4, Mat } from 'opencv';
let buf = new SharedArrayBuffer(640 * 480 * 4);

console.log('toPointer(buf)', toPointer(buf));

let m = new Mat(480, 640, CV_8UC4, buf);

const { elemSize, elemSize1 } = m;

console.log('m', m);
console.log('m.buffer', m.buffer);
console.log('toPointer(m.buffer)', toPointer(m.buffer));
console.log('m', { elemSize, elemSize1 });