const fs = require("fs");
const { isJpeg, jpegProps } = require("./lib/jpeg.js");

let data = fs.readFileSync("1072.jpg");

let props = jpegProps(data);

console.log("props:", props);
