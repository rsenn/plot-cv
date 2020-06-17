import INIGrammar from './ini-grammar.js';
import fs from 'fs';
import { Console } from 'console';
global.console = new Console({
  stdout: process.stdout,
  stderr: process.stderr,
  inspectOptions: { depth: 10, colors: true }
});
let filename = '../pictest/build/mplab/7segtest-16f876a-xc8-debug.mcp';
let src = fs.readFileSync(filename).toString();

console.log('src:', src);
let [done, data, pos] = INIGrammar.ini(src, 0);

let sections = data[0].reduce((acc, sdata) => ({ ...acc, [sdata[0]]: new Map(sdata[1]) }), {});
console.log('data:', sections);

let file_keys = sections.FILE_INFO.keys();
let file_sections = Object.keys(sections).filter(name => /FILE/.test(name));

let files = [];
for(let key of file_keys) {
  let file = {};
  for(let sect of file_sections) {
    let value = sections[sect].get(key);
    if(value) file[sect] = value;
  }
  files.push(file);
}

for(let sect of file_sections) {
  sections[sect].clear();
}

let filenames = files.map(f => f.FILE_INFO);

files = files.filter(file => !/.*(buffer|comparator|lcd|format|ds18b20|hd44).*/.test(file.FILE_INFO));

console.log('data:', file_sections);
console.log('files:', filenames);
let i = 0;
for(let file of files) {
  for(let field in file) {
    sections[field].set(`file_${(i + '').padStart(3, '0')}`, file[field]);
  }

  i++;
}

let out = '';
for(let section in sections) {
  out += `[${section}]\r\n`;
  for(let [key, value] of sections[section]) {
    out += `${key}=${value}\r\n`;
  }
}

console.log('out:', out);

fs.writeFileSync(filename, out);
