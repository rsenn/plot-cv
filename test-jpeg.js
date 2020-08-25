const fs = require('fs');
const { isJpeg, jpegProps } = require('./lib/jpeg.js');

let data = filesystem.readFile('1072.jpg');

let props = jpegProps(data);

//console.log('props:', props);
