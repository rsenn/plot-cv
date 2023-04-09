import filesystem from 'fs';
import require from 'require';
import { isJpeg, jpegProps } from './lib/jpeg.js';

let data = filesystem.readFileSync('blah.jpg');
console.log('data:', data);

let props = jpegProps(new Uint8Array(data));

console.log('props:', props);
